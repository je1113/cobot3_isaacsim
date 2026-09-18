"""
task_manager — orchestrator 상태기계.

docs/08_ROS2_NODE_Graph.html 의 확정안(01-03)이 기준이다. 노드 5개 중
"미션 어휘는 전부 여기" 에 해당하는 노드이고, 나머지 셋은 좌표 하나 또는
종류 하나만 받는다.

    ros2 run cobot3_orchestrator task_manager

기본 기능 두 가지
  1) 로봇의 상태 저장 — patrol / hold / scan / pick / nav / place 중 하나
  2) 명령 큐 — carrier_scan 으로 들어온 물체의 종류(1, 2)를 저장

상태 전이
  patrol  기본 상태. 돌아다니면서 QR 을 찾는다.
          /perception/carrier_detected 로 "QR 이 보인다" 신호를 받으면
    -> hold   로봇을 멈춘다. 진행 중인 NavigateTo goal 을 취소하는 것이
              정지 명령이다. goal 을 거두면 Nav2 가 cmd_vel 발행을 멈추고
              로봇이 선다 (cmd_vel 에 0 을 직접 쏘면 살아 있는 Nav2 와 싸운다).
    -> scan   /perception/carrier_scan 서비스를 호출해 QR 정보를 받아온다.
              멈춘 뒤에 읽어야 정확하다 (05 §12-1 정지 상태에서 근접 판독).
              받아온 정보를 명령 큐에 넣는다.
    -> pick   /manipulation/pick_carrier 로 픽 명령. 멈춘 그 자리에서 집는다
              (픽하러 가는 주행 단계는 없다).
    -> nav    /navigation/navigate_to 로 목적지 좌표 전송.
    -> place  /manipulation/place_carrier 로 배치 명령.
    -> patrol 배치가 끝나면 다시 순찰.

사용하는 인터페이스 — 08 문서에서 확정된 것만 쓴다
  구독  /perception/carrier_detected  std_msgs/Bool
  호출  /perception/carrier_scan      CarrierScan.srv
  액션  /navigation/navigate_to       NavigateTo.action
  액션  /manipulation/pick_carrier    PickCarrier.action
  액션  /manipulation/place_carrier   PlaceCarrier.action
  발행  /orchestrator/state           std_msgs/String  (상태 · 실패 단계 확인용)

/orchestrator/state 만 확정안 밖의 추가분이다. 어느 단계에서 실패했는지
밖에서 확인할 수 있어야 해서 넣었고, std_msgs/String 이라 cobot3_interfaces
에는 파일이 생기지 않는다.

실패하면
  로봇을 멈추고 (진행 중인 goal 을 취소) 그 자리에서 정지한다. 상태는
  실패한 그 상태 그대로 둔다 — pick 에서 실패했으면 상태는 pick 이다.
  자동 복귀도 재시도도 하지 않는다. 어느 단계에서 왜 실패했는지는 에러 로그와
  /orchestrator/state 토픽 두 곳에서 확인한다.

    ros2 topic echo /orchestrator/state

알려진 갭
  - /perception/carrier_detected 를 발행하는 쪽이 cobot3_perception 에 아직
    없다. 그때까지는 아래로 직접 쏴서 시험한다.
        ros2 topic pub --once /perception/carrier_detected std_msgs/Bool "{data: true}"
  - /navigation/navigate_to 액션 서버(nav_server)가 아직 없다. 서버가 붙기
    전까지는 patrol 과 nav 단계에서 SERVER_UNAVAILABLE 로 멈춘다. 이전 판이
    쓰던 SimClient.teleport_base 직통 호출은 확정된 NavigateTo 액션을 쓰기로
    해서 전부 걷어냈다.
  - 이 씬의 QR 은 숫자 하나("1"/"2")만 담아서 같은 종류의 여러 개체를 구분하지
    못한다. carriers.yaml 에서 그 종류의 첫 항목을 쓴다.
  - 작업 중(scan/pick/nav/place)에 들어온 carrier_detected 는 버린다. 물건은
    그 자리에 그대로 있으므로 다음 순찰에 다시 보인다. 나중에 perception 을
    껐다 켜는 로직이 붙으면 그때 이 자리를 대체한다.
"""

import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String

from cobot3_interfaces.action import NavigateTo, PickCarrier, PlaceCarrier
from cobot3_interfaces.srv import CarrierScan

# ══════════════════════════════════════════════════════════════════════════
#  좌표 — 아직 안 정해졌다. 아래 두 곳을 채워 넣어야 로봇이 실제로 움직인다.
#
#  좌표 형식은 둘 다 같다:   (x, y, yaw_deg)
#      x, y      map 프레임 기준 위치, 단위 m
#      yaw_deg   그 자리에서 로봇이 바라볼 방향, 단위 도(degree).
#                +x 축이 0도이고 반시계 방향이 +. 예: 90.0 이면 +y 를 본다.
#  세 값 모두 float 이다. 아래 _to_pose() 가 이걸 NavigateTo goal 의
#  PoseStamped(frame_id="map") 로 바꿔서 보낸다.
# ══════════════════════════════════════════════════════════════════════════

# 순찰 경로 — patrol 상태에서 이 좌표들을 순서대로 돌고, 끝까지 가면 다시
# 처음으로 돌아가 반복한다. QR 은 0.28 m 안에서만 읽히므로(05 §2-4) 각 정차점은
# 선반 라벨 가까이에, 그리고 멈춘 자리에서 팔이 매거진에 닿는 거리에 잡아야 한다
# — 픽하러 따로 이동하지 않고 선 그 자리에서 집기 때문이다(M0609 도달거리 900 mm).
#
# 시험용 경로다. 두 점을 끝까지 돌면 다시 처음으로 돌아가므로 실제로는 이 구간을
# 왔다 갔다 하게 된다 — 지금은 그걸로 충분하다.
PATROL_ROUTE = [
    (-7.05, 1.45, 0.0),
    (-4.55, 1.45, 0.0),
]

# place 하러 갈 목적지 — 테스트 스테이션 로더 앞 주차 위치.
# 지금은 매거진 1 · 2 를 전부 여기로 가져다 놓는다. (08 문서의 "매거진은 패키징
# 로더로" 규칙은 지금 적용하지 않는다 — pkg_loader 는 나중에 추가한다.)
TEST_LOADER = (4.0, 0.0, 0.0)

# ── 로봇 상태 ──────────────────────────────────────────────────────────────
PATROL = "patrol"
HOLD = "hold"
SCAN = "scan"
PICK = "pick"
NAV = "nav"
PLACE = "place"

# QR payload -> grasp.yaml / place.yaml 의 variant 키.
# 이 씬의 QR 은 "1" 또는 "2" 한 글자만 담고 있다.
NUMERIC_TO_VARIANT = {"1": "magazine_1_orange", "2": "magazine_2_blue"}

# 각 단계를 이만큼 기다려도 안 끝나면 실패로 본다. 단위 초.
SCAN_TIMEOUT_S = 10.0
NAV_TIMEOUT_S = 300.0
PICK_TIMEOUT_S = 120.0
PLACE_TIMEOUT_S = 120.0
SERVER_WAIT_S = 5.0


def _find_ws_root():
    p = Path(__file__).resolve()
    for _ in range(10):
        if (p / "isaacpjt").is_dir():
            return p
        if p.parent == p:
            break
        p = p.parent
    raise RuntimeError("cobot3_ws 를 못 찾았다")


WS_ROOT = _find_ws_root()
CARRIERS_YAML = WS_ROOT / "src/cobot3_bringup/config/carriers.yaml"


@dataclass
class QueuedCommand:
    """명령 큐 한 건. carrier_scan 응답에서 만든다."""

    kind: str             # QR 원문. 이 씬에서는 "1" 또는 "2" — 물체의 종류다
    variant: str          # kind 를 grasp.yaml/place.yaml 키로 푼 것
    carrier_id: str       # carriers.yaml 조회 결과. 로그·표시용
    qr_pose: PoseStamped  # PickCarrier 가 플랜지를 찾을 탐색 창 prior


class MissionError(Exception):
    """단계 하나가 실패했다. stage 가 어느 단계인지, reason 이 왜인지."""

    def __init__(self, stage, reason):
        super().__init__(f"{stage} 단계 실패: {reason}")
        self.stage = stage
        self.reason = reason


def _reason_name(result_cls, code):
    """액션 result 의 fail_reason 코드를 .action 에 적힌 상수명으로 바꾼다."""
    for name in dir(result_cls):
        if not name.isupper():
            continue
        value = getattr(result_cls, name, None)
        if isinstance(value, int) and value == code:
            return f"{name}({code})"
    return str(code)


def _to_pose(xy_yaw_deg):
    """(x, y, yaw_deg) -> NavigateTo goal 의 PoseStamped(map)."""
    x, y, yaw_deg = xy_yaw_deg
    yaw = math.radians(yaw_deg)
    ps = PoseStamped()
    ps.header.frame_id = "map"
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.orientation.z = math.sin(yaw / 2.0)
    ps.pose.orientation.w = math.cos(yaw / 2.0)
    return ps


class TaskManager(Node):

    def __init__(self):
        super().__init__("task_manager")
        self.carriers = yaml.safe_load(CARRIERS_YAML.read_text(encoding="utf-8"))

        # ── 기본 기능 1) 로봇의 상태 저장 ──
        self._state = PATROL
        self._state_lock = threading.Lock()

        # ── 기본 기능 2) 명령 큐 ──
        self._queue = deque()

        # 실패 기록 — 어느 단계에서 왜 멈췄나
        self._failed = threading.Event()
        self._fail_stage = ""
        self._fail_reason = ""

        # 지금 붙들고 있는 것
        self._current = None       # 처리 중인 QueuedCommand
        self._active_goal = None   # 진행 중인 액션 goal handle (정지·실패 시 취소용)
        self._detected = threading.Event()
        self._shutdown = threading.Event()
        self._patrol_idx = 0

        cb = ReentrantCallbackGroup()
        self._carrier_scan = self.create_client(
            CarrierScan, "/perception/carrier_scan", callback_group=cb)
        self._nav = ActionClient(
            self, NavigateTo, "/navigation/navigate_to", callback_group=cb)
        self._pick = ActionClient(
            self, PickCarrier, "/manipulation/pick_carrier", callback_group=cb)
        self._place = ActionClient(
            self, PlaceCarrier, "/manipulation/place_carrier", callback_group=cb)

        self.create_subscription(
            Bool, "/perception/carrier_detected", self._on_carrier_detected, 10,
            callback_group=cb)
        self._state_pub = self.create_publisher(String, "/orchestrator/state", 10)
        self.create_timer(1.0, self._publish_state, callback_group=cb)

        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

        self.get_logger().info("task_manager ready — 상태 patrol")
        if not PATROL_ROUTE:
            self.get_logger().warning(
                "PATROL_ROUTE 가 비어 있다 — 순찰하지 않고 제자리에서 "
                "carrier_detected 만 기다린다. task_manager.py 상단에 좌표를 넣어라.")
        if TEST_LOADER is None:
            self.get_logger().warning(
                "TEST_LOADER 가 비어 있다 — pick 까지는 되지만 nav 단계에서 멈춘다. "
                "task_manager.py 상단에 좌표를 넣어라.")

    # ── 상태 ──────────────────────────────────────────────────────────────
    @property
    def state(self):
        with self._state_lock:
            return self._state

    def _set_state(self, new_state):
        with self._state_lock:
            old, self._state = self._state, new_state
        if old != new_state:
            self.get_logger().info(f"상태 {old} -> {new_state}")
        self._publish_state()

    def _publish_state(self):
        msg = String()
        parts = [f"state={self.state}", f"queue={len(self._queue)}"]
        if self._current is not None:
            parts.append(f"carrier={self._current.carrier_id}")
            parts.append(f"variant={self._current.variant}")
        if self._failed.is_set():
            parts.append("FAILED")
            parts.append(f"fail_stage={self._fail_stage}")
            parts.append(f"fail_reason={self._fail_reason}")
        msg.data = " | ".join(parts)
        self._state_pub.publish(msg)

    def _fail(self, stage, reason):
        """실패 — 로봇을 멈추고 그 상태 그대로 정지한다. 자동 복귀 없음."""
        self._fail_stage = stage
        self._fail_reason = reason
        self._failed.set()
        self._cancel_active_goal()
        self.get_logger().error(
            f"[실패] {stage} 단계에서 멈췄다. 이유: {reason} — "
            f"상태는 '{self.state}' 그대로 둔다. 자동 복귀하지 않는다.")
        self._publish_state()

    # ── carrier_detected ──────────────────────────────────────────────────
    def _on_carrier_detected(self, msg):
        if not msg.data:
            return
        if self._failed.is_set():
            return
        if self.state != PATROL:
            # 작업 중에 들어온 신호는 버린다. 물건은 그 자리에 그대로 있으므로
            # 다음 순찰에 다시 보인다.
            self.get_logger().debug(f"carrier_detected 무시 (상태 {self.state})")
            return
        if not self._detected.is_set():
            self.get_logger().info("carrier_detected — QR 이 보인다")
            self._detected.set()

    # ── 메인 루프 ─────────────────────────────────────────────────────────
    def _run(self):
        while rclpy.ok() and not self._shutdown.is_set():
            if self._failed.is_set():
                time.sleep(0.2)
                continue
            if self._detected.is_set():
                self._detected.clear()
                self._run_mission()
                continue
            if self.state == PATROL:
                self._patrol_step()
            else:
                time.sleep(0.1)

    def _patrol_step(self):
        """patrol — 돌아다니면서 QR 을 찾는다. 순찰 정차점을 순서대로 돈다."""
        if not PATROL_ROUTE:
            time.sleep(0.2)   # 경로가 없으면 제자리에서 carrier_detected 만 기다린다
            return

        target = PATROL_ROUTE[self._patrol_idx % len(PATROL_ROUTE)]
        self._patrol_idx += 1
        self.get_logger().info(f"patrol -> 정차점 {target}")
        try:
            gh = self._send_goal(
                self._nav, "/navigation/navigate_to",
                NavigateTo.Goal(pose=_to_pose(target)), PATROL)
        except MissionError as e:
            self._fail(e.stage, e.reason)
            return
        self._active_goal = gh

        # 도착할 때까지 기다리되, 그 사이 QR 이 보이면 즉시 빠져나온다.
        # 실제 정지(goal 취소)는 _run_mission 의 hold 단계에서 한다.
        fut = gh.get_result_async()
        deadline = time.monotonic() + NAV_TIMEOUT_S
        while not fut.done():
            if self._detected.is_set() or self._shutdown.is_set():
                return
            if time.monotonic() > deadline:
                self._fail(PATROL, f"TIMEOUT({NAV_TIMEOUT_S:.0f}s)")
                return
            time.sleep(0.05)

        self._active_goal = None
        result = fut.result().result
        if not result.success:
            reason = _reason_name(NavigateTo.Result, result.fail_reason)
            if reason.startswith("CANCELED"):
                return      # 우리가 세운 것이다 — 실패가 아니다
            self._fail(PATROL, reason)

    def _run_mission(self):
        """hold -> scan -> pick -> nav -> place -> patrol."""
        try:
            self._step_hold()
            self._step_scan()
            self._step_pick()
            self._step_nav()
            self._step_place()
        except MissionError as e:
            self._fail(e.stage, e.reason)
            return

        self._current = None
        self._set_state(PATROL)
        self.get_logger().info("사이클 완료 — patrol 로 복귀")

    def _step_hold(self):
        """로봇을 멈춘다. 진행 중인 NavigateTo goal 을 취소하는 것이 정지 명령이다."""
        self._set_state(HOLD)
        self._cancel_active_goal()

    def _step_scan(self):
        """멈춘 뒤 carrier_scan 서비스로 QR 정보를 받아 명령 큐에 넣는다."""
        self._set_state(SCAN)
        if not self._carrier_scan.wait_for_service(timeout_sec=SERVER_WAIT_S):
            raise MissionError(SCAN, "SERVICE_UNAVAILABLE(/perception/carrier_scan)")

        fut = self._carrier_scan.call_async(CarrierScan.Request())
        res = self._wait(fut, SCAN_TIMEOUT_S, SCAN)
        if not res.found:
            raise MissionError(SCAN, "NOT_FOUND(found=false)")

        variant = NUMERIC_TO_VARIANT.get(res.payload)
        if variant is None:
            raise MissionError(SCAN, f"UNKNOWN_PAYLOAD({res.payload!r})")

        qr_pose = PoseStamped()
        qr_pose.header = res.header
        qr_pose.pose = res.qr_pose
        cmd = QueuedCommand(
            kind=res.payload,
            variant=variant,
            carrier_id=self._lookup_carrier_id(variant),
            qr_pose=qr_pose,
        )
        self._queue.append(cmd)
        self.get_logger().info(
            f"scan — 종류={cmd.kind} variant={cmd.variant} "
            f"carrier={cmd.carrier_id} / 명령 큐에 넣음 (큐 {len(self._queue)}건)")

    def _step_pick(self):
        """멈춘 그 자리에서 집는다. 픽하러 가는 주행 단계는 없다."""
        if not self._queue:
            raise MissionError(PICK, "EMPTY_QUEUE")
        self._current = self._queue.popleft()
        self._set_state(PICK)

        goal = PickCarrier.Goal(variant=self._current.variant,
                                qr_pose=self._current.qr_pose)
        gh = self._send_goal(self._pick, "/manipulation/pick_carrier", goal, PICK,
                             self._log_phase("PICK"))
        result = self._await(gh, PICK_TIMEOUT_S, PICK)
        if not result.success:
            raise MissionError(PICK, _reason_name(PickCarrier.Result, result.fail_reason))

    def _step_nav(self):
        """목적지 좌표로 주행. 지금은 매거진 1 · 2 전부 test_loader 로 간다."""
        self._set_state(NAV)
        if TEST_LOADER is None:
            raise MissionError(
                NAV, "NO_DESTINATION(task_manager.py 의 TEST_LOADER 가 비어 있다)")

        gh = self._send_goal(
            self._nav, "/navigation/navigate_to",
            NavigateTo.Goal(pose=_to_pose(TEST_LOADER)), NAV)
        result = self._await(gh, NAV_TIMEOUT_S, NAV)
        if not result.success:
            raise MissionError(NAV, _reason_name(NavigateTo.Result, result.fail_reason))

    def _step_place(self):
        """놓을 자리는 종류로 정해진다 — 좌표를 넘기지 않는다."""
        self._set_state(PLACE)
        goal = PlaceCarrier.Goal(variant=self._current.variant)
        gh = self._send_goal(self._place, "/manipulation/place_carrier", goal, PLACE,
                             self._log_phase("PLACE"))
        result = self._await(gh, PLACE_TIMEOUT_S, PLACE)
        if not result.success:
            raise MissionError(PLACE, _reason_name(PlaceCarrier.Result, result.fail_reason))

    # ── 도우미 ────────────────────────────────────────────────────────────
    def _lookup_carrier_id(self, variant):
        """carriers.yaml 에서 그 variant 의 첫 항목. QR 이 종류만 담아서 개체는 못 가린다."""
        for cid, info in self.carriers.items():
            if isinstance(info, dict) and info.get("variant") == variant:
                return cid
        return ""

    def _log_phase(self, label):
        return lambda fb: self.get_logger().info(f"  {label} phase={fb.feedback.phase}")

    def _send_goal(self, client, name, goal, stage, feedback_cb=None):
        """goal 을 보내고 수락된 handle 을 돌려준다. 이후 _active_goal 로 추적한다."""
        if not client.wait_for_server(timeout_sec=SERVER_WAIT_S):
            raise MissionError(stage, f"SERVER_UNAVAILABLE({name})")
        fut = client.send_goal_async(goal, feedback_callback=feedback_cb)
        gh = self._wait(fut, SERVER_WAIT_S * 2, stage)
        if not gh.accepted:
            raise MissionError(stage, "GOAL_REJECTED")
        self._active_goal = gh
        return gh

    def _await(self, goal_handle, timeout_s, stage):
        """result 를 기다린다. 끝나면(성공이든 실패든) 더 이상 진행 중인 goal 이 아니다."""
        result = self._wait(goal_handle.get_result_async(), timeout_s, stage).result
        self._active_goal = None
        return result

    def _wait(self, future, timeout_s, stage):
        done = threading.Event()
        future.add_done_callback(lambda f: done.set())
        if not done.wait(timeout=timeout_s):
            raise MissionError(stage, f"TIMEOUT({timeout_s:.0f}s)")
        return future.result()

    def _cancel_active_goal(self):
        """진행 중인 액션 goal 을 거둔다. hold 에서는 이게 로봇을 세우는 명령이고,
        실패 시에는 팔이든 베이스든 하던 동작을 멈추는 것이다."""
        gh = self._active_goal
        self._active_goal = None
        if gh is None:
            return
        self.get_logger().info("진행 중인 goal 취소")
        done = threading.Event()
        gh.cancel_goal_async().add_done_callback(lambda f: done.set())
        if not done.wait(timeout=SERVER_WAIT_S):
            self.get_logger().warning("goal 취소 응답이 없다 — 그대로 진행한다")

    def destroy_node(self):
        self._shutdown.set()
        super().destroy_node()


def main():
    rclpy.init()
    node = TaskManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
