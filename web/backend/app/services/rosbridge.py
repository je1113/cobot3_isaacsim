"""ROS 경계 — 백엔드에서 rclpy 를 아는 유일한 파일.

라우터와 서비스는 전부 `Bridge` 만 보고, rclpy 를 import 하지 않는다.
그래서 ROS 없이(COBOT3_ROS=0) 백엔드 전체가 뜨고 테스트가 돈다.

★ rclpy 는 asyncio 와 루프가 다르다. 그래서 RclpyBridge 는 rclpy 를 **별도
  스레드**의 MultiThreadedExecutor 에서 돌리고, 결과를 loop.call_soon_threadsafe
  로 asyncio 쪽에 넘긴다. 반대 방향(요청)은 concurrent.futures 로 기다린다.
  이 파일 밖에서는 그 사실이 보이지 않는다.

★ /orchestrator/state 는 std_msgs/String 의 "key=value | key=value" 형식이고,
  carrier_code_reader 와의 **계약**이다(task_manager.py:982 주석 — 그 노드는
  state=="patrol" 일 때만 QR 폴링을 돈다). 구조화된 메시지로 바꾸면 그 노드가
  깨지므로 건드리지 않고 여기서 파싱한다.
"""

from __future__ import annotations

import asyncio
import logging
import math
import threading
from typing import Any, Awaitable, Callable

from ..config import settings
from ..errors import BridgeUnavailable

log = logging.getLogger(__name__)

# 브리지가 위로 올리는 사건. main 이 여기에 핸들러를 건다.
EventSink = Callable[[str, dict], Awaitable[None]]


def parse_state(raw: str) -> dict[str, Any]:
    """'state=pick | detected=True | carrier=F1-MGZB-1' → dict.

    값이 없는 토큰(FAILED)은 True 로 둔다 — task_manager 가 그렇게 싣는다.
    """
    out: dict[str, Any] = {}
    for token in raw.split("|"):
        token = token.strip()
        if not token:
            continue
        if "=" in token:
            k, v = token.split("=", 1)
            k, v = k.strip(), v.strip()
            if v in {"True", "False"}:
                out[k] = v == "True"
            elif v.isdigit():
                out[k] = int(v)
            else:
                out[k] = v
        else:
            out[token] = True
    return out


class Bridge:
    """인터페이스. 구현 둘 — NullBridge / RclpyBridge."""

    on_event: EventSink | None = None

    async def start(self) -> None: ...
    async def stop(self) -> None: ...

    @property
    def available(self) -> bool:
        return False

    async def execute_task(self, robot_id: str, order: dict) -> None:
        raise BridgeUnavailable("ROS 브리지가 꺼져 있다 (COBOT3_ROS=0)")

    async def cancel_task(self, robot_id: str) -> bool:
        raise BridgeUnavailable("ROS 브리지가 꺼져 있다 (COBOT3_ROS=0)")

    async def command(self, robot_id: str, command: str, reason: str = "") -> dict:
        raise BridgeUnavailable("ROS 브리지가 꺼져 있다 (COBOT3_ROS=0)")

    async def reload_config(self, robot_id: str, scope: str) -> dict:
        raise BridgeUnavailable("ROS 브리지가 꺼져 있다 (COBOT3_ROS=0)")

    async def capture_pose(self, robot_id: str) -> dict:
        raise BridgeUnavailable("ROS 브리지가 꺼져 있다 (COBOT3_ROS=0)")


class NullBridge(Bridge):
    """ROS 없이 백엔드만 띄울 때. 읽기 API·설정 편집·큐 관리는 전부 동작한다.

    막히는 것은 '로봇에게 실제로 시키는' 부분뿐이고, 그건 503 으로 명확히 거절한다 —
    조용히 성공한 척하면 화면은 배차된 줄 알고 영원히 기다린다.
    """

    async def start(self) -> None:
        log.warning("ROS 브리지 꺼짐 — 실시간 채널과 제어 명령이 동작하지 않는다 (COBOT3_ROS=1 로 켠다)")


class RclpyBridge(Bridge):
    """실제 ROS 연결. rclpy 는 이 클래스 안에서만 import 한다."""

    def __init__(self) -> None:
        self._loop: asyncio.AbstractEventLoop | None = None
        self._node = None
        self._executor = None
        self._thread: threading.Thread | None = None
        self._goal_handles: dict[str, Any] = {}
        self._last_pose_sent: dict[str, float] = {}
        self._last_pose: dict[str, dict] = {}

    @property
    def available(self) -> bool:
        return self._node is not None

    # ── 수명 ─────────────────────────────────────────────────────────
    async def start(self) -> None:
        import rclpy
        from rclpy.executors import MultiThreadedExecutor

        self._loop = asyncio.get_running_loop()
        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = self._build_node()
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._executor.spin, name="rclpy", daemon=True)
        self._thread.start()
        log.info("ROS 브리지 시작 — 네임스페이스 %s", ", ".join(settings().namespaces))

    async def stop(self) -> None:
        import rclpy

        if self._executor is not None:
            self._executor.shutdown()
        if self._node is not None:
            self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self._node = self._executor = None

    # ── 노드 구성 ────────────────────────────────────────────────────
    def _build_node(self):
        from cobot3_interfaces.action import ExecuteTask
        from cobot3_interfaces.msg import RobotAlert
        from cobot3_interfaces.srv import ReloadConfig, RobotCommand
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from rclpy.action import ActionClient
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String

        s = settings()
        node = Node("cobot3_web_bridge")

        # ★ amcl 은 로봇이 움직일 때만 amcl_pose 를 낸다. VOLATILE 로 구독하면
        #   서 있는 로봇의 위치는 영영 못 받는다. amcl 이 TRANSIENT_LOCAL 로
        #   내므로 같게 맞춰 마지막 자세를 붙자마자 받는다.
        pose_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._action: dict[str, Any] = {}
        self._cmd_cli: dict[str, Any] = {}
        self._reload_cli: dict[str, Any] = {}
        self._joints: dict[str, Any] = {}

        for robot in s.namespaces:
            node.create_subscription(
                String,
                s.state_topic_tmpl.format(robot=robot),
                lambda msg, r=robot: self._emit("robot.state", {"robot_id": r, **parse_state(msg.data)}, r),
                10,
            )
            node.create_subscription(
                PoseWithCovarianceStamped,
                s.pose_topic_tmpl.format(robot=robot),
                lambda msg, r=robot: self._on_pose(r, msg),
                pose_qos,
            )
            node.create_subscription(
                JointState,
                f"/{robot}/joint_states",
                lambda msg, r=robot: self._joints.__setitem__(
                    r, {"name": list(msg.name), "position": list(msg.position)}
                ),
                10,
            )
            self._action[robot] = ActionClient(node, ExecuteTask, f"/{robot}/orchestrator/execute_task")
            self._cmd_cli[robot] = node.create_client(RobotCommand, f"/{robot}/orchestrator/command")
            self._reload_cli[robot] = node.create_client(ReloadConfig, f"/{robot}/config/reload")

        # payload 없는 사건 — §4-6 의 구멍. 전역 토픽 하나.
        node.create_subscription(
            RobotAlert,
            "/trace/alert",
            lambda msg: self._emit(
                "alert",
                {
                    "robot_id": msg.robot_id,
                    "severity": int(msg.severity),
                    "code": msg.code,
                    "detail": msg.detail,
                    "stage": msg.stage,
                },
                msg.robot_id,
            ),
            20,
        )
        # ★ 허브는 놓친 메시지를 재전송하지 않는다. 브라우저가 나중에 붙으면
        #   서 있는 로봇의 위치를 못 받으므로, 마지막 자세를 1초마다 다시 흘린다.
        #   저장이 아니라 흘려보내기의 반복이다 — 상태 토픽도 같은 식으로 반복된다.
        node.create_timer(1.0, self._repeat_pose)
        return node

    def _repeat_pose(self) -> None:
        import time

        now = time.monotonic()
        for robot, pose in list(self._last_pose.items()):
            if now - self._last_pose_sent.get(robot, 0.0) >= 1.0:
                self._last_pose_sent[robot] = now
                self._emit("robot.pose", pose, robot)

    # ── 스레드 → asyncio ─────────────────────────────────────────────
    def _emit(self, ch: str, data: dict, robot_id: str | None = None) -> None:
        if self.on_event is None or self._loop is None:
            return
        coro = self.on_event(ch, {"robot_id": robot_id, **data})
        asyncio.run_coroutine_threadsafe(coro, self._loop)

    def _on_pose(self, robot: str, msg) -> None:
        """6.1 — 5~10Hz 로 솎아낸다. 저장은 하지 않는다."""
        import time

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # ★ theta 까지 여기서 만든다. 화면이 쓰는 것은 yaw 하나이고, 쿼터니언을
        #   푸는 것은 ROS 를 아는 쪽의 일이다 — dispatcher 가 q 를 알 이유가 없다.
        #   (qz·qw 도 같이 남긴다. 평면 주행이라 yaw 로 충분하지만, 원본을 버리면
        #    나중에 3D 자세가 필요해질 때 여기까지 다시 와야 한다.)
        theta = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        pose = {"x": p.x, "y": p.y, "theta": theta, "qz": q.z, "qw": q.w,
                "frame": msg.header.frame_id}
        # 솎아내기 전에 기억한다 — 멈춘 직후의 마지막 자세가 버려지면
        # _repeat_pose 가 그 앞의 낡은 자세를 계속 흘린다.
        self._last_pose[robot] = pose

        now = time.monotonic()
        period = 1.0 / max(settings().pose_hz, 0.1)
        if now - self._last_pose_sent.get(robot, 0.0) < period:
            return
        self._last_pose_sent[robot] = now
        self._emit("robot.pose", pose, robot)

    # ── 요청 ─────────────────────────────────────────────────────────
    async def execute_task(self, robot_id: str, order: dict) -> None:
        from cobot3_interfaces.action import ExecuteTask

        client = self._require(self._action, robot_id, "ExecuteTask 액션 서버")
        if not client.wait_for_server(timeout_sec=3.0):
            raise BridgeUnavailable(f"{robot_id} 의 execute_task 액션 서버가 없다")

        goal = ExecuteTask.Goal()
        goal.task_id = int(order["task_id"])
        goal.kind = str(order["kind"])
        goal.target_ref = str(order["target_ref"])
        goal.resume_progress = float(order.get("resume_progress") or 0.0)
        fut = client.send_goal_async(goal, feedback_callback=lambda fb: self._on_feedback(robot_id, order, fb))
        fut.add_done_callback(lambda f: self._on_accepted(robot_id, order, f))

    def _on_accepted(self, robot_id: str, order: dict, fut) -> None:
        handle = fut.result()
        if not handle.accepted:
            self._emit("task.rejected", {"task_id": order["task_id"]}, robot_id)
            return
        self._goal_handles[robot_id] = handle
        handle.get_result_async().add_done_callback(
            lambda f: self._on_result(robot_id, order, f)
        )

    def _on_feedback(self, robot_id: str, order: dict, fb) -> None:
        f = fb.feedback
        self._emit(
            "task.feedback",
            {
                "task_id": order["task_id"],
                "stage": f.stage,
                "progress": float(f.progress),
                "run_id": f.run_id or None,
                "carrier_id": f.carrier_id or None,
            },
            robot_id,
        )

    def _on_result(self, robot_id: str, order: dict, fut) -> None:
        r = fut.result().result
        self._goal_handles.pop(robot_id, None)
        self._emit(
            "task.result",
            {
                "task_id": order["task_id"],
                "success": bool(r.success),
                "more_at_target": bool(r.more_at_target),
                "run_id": r.run_id or None,
                "fail_reason": int(r.fail_reason),
                "fail_detail": r.fail_detail,
            },
            robot_id,
        )

    async def cancel_task(self, robot_id: str) -> bool:
        handle = self._goal_handles.get(robot_id)
        if handle is None:
            return False
        handle.cancel_goal_async()
        return True

    async def command(self, robot_id: str, command: str, reason: str = "") -> dict:
        from cobot3_interfaces.srv import RobotCommand

        cli = self._require(self._cmd_cli, robot_id, "RobotCommand 서비스")
        if not cli.wait_for_service(timeout_sec=3.0):
            raise BridgeUnavailable(f"{robot_id} 의 command 서비스가 없다")
        res = await self._call(cli, RobotCommand.Request(command=command, reason=reason))
        return {"accepted": res.accepted, "rejected_because": res.rejected_because, "state": res.state}

    async def reload_config(self, robot_id: str, scope: str) -> dict:
        from cobot3_interfaces.srv import ReloadConfig

        cli = self._require(self._reload_cli, robot_id, "ReloadConfig 서비스")
        if not cli.wait_for_service(timeout_sec=2.0):
            # reload 를 못 보내는 것은 치명적이지 않다 — 다음 미션에서 어차피 다시 읽는다.
            return {"reloaded": False, "revision": "", "rejected_because": "서비스 없음"}
        res = await self._call(cli, ReloadConfig.Request(scope=scope))
        return {
            "reloaded": res.reloaded,
            "revision": res.revision,
            "rejected_because": res.rejected_because,
        }

    async def capture_pose(self, robot_id: str) -> dict:
        """현재 관절값 (3.3). 전용 서비스를 만들지 않고 /joint_states 를 쓴다 —
        값을 읽기만 하는 데 새 인터페이스를 늘릴 이유가 없다."""
        snap = self._joints.get(robot_id)
        if snap is None:
            raise BridgeUnavailable(f"{robot_id} 의 /joint_states 를 아직 못 받았다")
        return snap

    # ── 유틸 ─────────────────────────────────────────────────────────
    def _require(self, table: dict, robot_id: str, what: str):
        if self._node is None:
            raise BridgeUnavailable("ROS 브리지가 시작되지 않았다")
        if robot_id not in table:
            raise BridgeUnavailable(
                f"모르는 네임스페이스: {robot_id} ({what}). "
                f"COBOT3_ROBOTS 에 'AMR-0N={robot_id}' 형태로 넣어야 한다"
            )
        return table[robot_id]

    async def _call(self, cli, req):
        """rclpy 서비스 호출을 asyncio 에서 기다린다."""
        fut = cli.call_async(req)
        loop = asyncio.get_running_loop()
        done: asyncio.Future = loop.create_future()

        def _cb(f):
            if not done.done():
                loop.call_soon_threadsafe(done.set_result, f.result())

        fut.add_done_callback(_cb)
        return await asyncio.wait_for(done, timeout=10)


def make_bridge() -> Bridge:
    if settings().ros_enabled:
        return RclpyBridge()
    return NullBridge()


bridge: Bridge = make_bridge()
