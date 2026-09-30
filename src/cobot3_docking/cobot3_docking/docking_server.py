"""
docking_server — 도크 접점 붙이기 · 충전 대기. ★ 전역 1개.

    ros2 run cobot3_docking docking_server

docs/02 §2 의 "전역 노드 3 (event_logger · web_bridge · docking_server)" 중
마지막으로 붙는 것이다. 미션 노드 넷(nav_server · carrier_code_reader ·
pick_place_server · task_manager)은 로봇 네임스페이스마다 한 벌씩 뜨고 이름이
전부 상대이름이라 "누가" 를 배선이 보장한다. 이 노드는 도크가 공용 자원이라
네임스페이스 밖에 하나만 두고, 대신 Dock.action 의 robot_name 필드로 어느
로봇인지 듣는다 — 식별자가 필드로 올라오는 유일한 예외다.

무엇을 하고 무엇을 안 하나
  한다      goal 의 robot_name 으로 frames.yaml dock_pads 에서 자리를 찾는다.
            charge_duration_s 동안 기다리며 charge_pct feedback 을 올린다.
            끝나면 success. 취소되면 CANCELED. 한도를 넘으면 TIMEOUT.
            /docking/state (std_msgs/String) 로 누가 충전 중인지 흘린다.
  안 한다   로봇을 움직이지 않는다. 도크 앞까지 가는 것(DOCK_NAV · DOCK_IN)과
            나오는 것(DOCK_OUT)은 task_manager 가 NavigateTo / patrol_to 로 한다.
            자리 다툼(선점)도 없다 — 자리가 로봇별로 고정이라(dock_pads) 다른
            로봇이 그 자리에 갈 경로 자체가 없다. 그래서 PAD_BUSY 같은 실패
            사유를 두지 않았다.

★ 시뮬에는 배터리가 없다 — 전부 타이머다 (NavigateTo.action 머리주석).
  "도크에 charge_duration_s 초 서 있으면 충전 완료" 로 본다. 실기로 가면 이
  파일 안에서 opennav_docking(Nav2 docking_server)을 부르고 배터리 토픽을 보는
  것으로 바뀌며, task_manager 쪽 계약(Dock.action)은 그대로다.

★ 두 goal 을 동시에 받는다. 두 로봇이 각자 자기 자리에서 병렬로 충전하므로
  rclpy ActionServer 기본(순차)으로는 둘째 로봇이 첫째가 끝날 때까지 도크 앞에
  서 있게 된다. ReentrantCallbackGroup + MultiThreadedExecutor 로 execute 콜백이
  동시에 돈다(docs/02 §2 "handle_accepted_callback 을 붙여야 한다" 의 실제 구현).

파라미터
  frames_yaml        (string)  dock_pads 를 읽는 파일. 기본 src/cobot3_bringup/config/frames.yaml
  charge_duration_s  (double)  이만큼 서 있으면 충전 완료. 기본 60 (사용자 결정: 1분 충전)
  dock_timeout_s     (double)  이 안에 안 끝나면 TIMEOUT. 기본 charge_duration_s + 60
  feedback_period_s  (double)  charge_pct feedback 주기. 기본 1.0
"""

import threading
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from cobot3_interfaces.action import Dock

# ★ 절대이름. 네임스페이스 없이 뜨므로 그대로 /docking/dock 이 된다.
#   task_manager 도 "/docking/dock" 으로 부른다 — 상대이름으로 두면
#   /robot1/docking/dock 을 찾다가 서버가 없다고 얼어붙는다.
DOCK_ACTION = "/docking/dock"
STATE_TOPIC = "/docking/state"

DEFAULT_CHARGE_DURATION_S = 60.0    # 사용자 결정(2026-09-28): 15분 일하고 1분 충전
DEFAULT_FEEDBACK_PERIOD_S = 1.0
STATE_PUBLISH_PERIOD_S = 1.0


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
DEFAULT_FRAMES_YAML = WS_ROOT / "src/cobot3_bringup/config/frames.yaml"


def load_dock_pads(path):
    """frames.yaml 의 dock_pads 절 → {robot_name: (x, y, yaw_deg)}.

    task_manager 도 같은 절을 같은 모양으로 읽는다(_resolve_dock_pose). 여기서는
    좌표를 쓸 일이 없고(로봇을 안 움직인다) "이 robot_name 이 아는 로봇인가" 만
    본다 — 모르는 이름이면 UNKNOWN_ROBOT 으로 닫아 웹/로그에서 frames.yaml 과
    launch 가 어긋난 것을 바로 보이게 한다.
    """
    cfg = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    pads = cfg.get("dock_pads") or {}
    out = {}
    for name, pad in pads.items():
        if not isinstance(pad, dict):
            continue
        xy = pad.get("xy_yaw_deg")
        if not (isinstance(xy, (list, tuple)) and len(xy) == 3):
            continue
        out[str(name)] = (float(xy[0]), float(xy[1]), float(xy[2]))
    return out


class DockingServer(Node):

    def __init__(self):
        super().__init__("docking_server")
        self.declare_parameter("frames_yaml", str(DEFAULT_FRAMES_YAML))
        self.declare_parameter("charge_duration_s", DEFAULT_CHARGE_DURATION_S)
        self.declare_parameter("dock_timeout_s", 0.0)     # 0 = charge_duration_s + 60
        self.declare_parameter("feedback_period_s", DEFAULT_FEEDBACK_PERIOD_S)

        frames = self.get_parameter("frames_yaml").value
        self.charge_duration_s = max(0.0, float(self.get_parameter("charge_duration_s").value))
        timeout = float(self.get_parameter("dock_timeout_s").value)
        self.dock_timeout_s = timeout if timeout > 0.0 else self.charge_duration_s + 60.0
        self.feedback_period_s = max(0.1, float(self.get_parameter("feedback_period_s").value))

        try:
            self.pads = load_dock_pads(frames)
        except Exception as e:   # noqa: BLE001 — 뜨긴 뜨되 모든 goal 을 UNKNOWN_ROBOT 으로 닫는다
            self.pads = {}
            self.get_logger().error(f"frames.yaml 을 못 읽었다({frames}): {e} — dock_pads 없음")
        if not self.pads:
            self.get_logger().error(
                f"dock_pads 가 비어 있다({frames}) — 어떤 로봇의 Dock goal 도 "
                f"UNKNOWN_ROBOT 으로 닫힌다. frames.yaml 에 dock_pads 절을 채워라.")

        # robot_name -> {"charge_pct": float, "since": monotonic}. 표시용.
        self._active = {}
        self._lock = threading.Lock()

        self._group = ReentrantCallbackGroup()
        self._server = ActionServer(
            self, Dock, DOCK_ACTION,
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._group)
        self._state_pub = self.create_publisher(String, STATE_TOPIC, 10)
        self.create_timer(STATE_PUBLISH_PERIOD_S, self._publish_state)

        pads = ", ".join(f"{k}=({v[0]:.3f}, {v[1]:.3f}, {v[2]:.0f}°)"
                         for k, v in sorted(self.pads.items()))
        self.get_logger().info(
            f"docking_server ready — {DOCK_ACTION} · 충전 {self.charge_duration_s:.0f} s · "
            f"한도 {self.dock_timeout_s:.0f} s · 자리 [{pads or '없음'}]")

    # ── 액션 ──────────────────────────────────────────────────────────────
    def _on_goal(self, request):
        # 모르는 로봇도 받는다 — 거절(REJECT)은 사유를 실어 보낼 수 없어서,
        # 받고 UNKNOWN_ROBOT 으로 닫는 편이 task_manager 로그에 이유가 남는다.
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        goal = goal_handle.request
        name = goal.robot_name or "?"
        result = Dock.Result()

        if name not in self.pads:
            self.get_logger().error(
                f"[{name}] dock_pads 에 없는 로봇 — UNKNOWN_ROBOT. 아는 로봇: {sorted(self.pads)}")
            result.success = False
            result.fail_reason = Dock.Result.UNKNOWN_ROBOT
            result.charge_pct = 0.0
            goal_handle.abort()
            return result

        target_pct = goal.charge_target_pct if 0.0 < goal.charge_target_pct <= 100.0 else 100.0
        # 목표 % 에 비례한 시간. 100% 면 charge_duration_s 그대로.
        duration = self.charge_duration_s * (target_pct / 100.0)
        with self._lock:
            if name in self._active:
                self.get_logger().warning(
                    f"[{name}] 이미 충전 중인데 goal 이 또 왔다 — 새 goal 로 이어간다")
            self._active[name] = {"charge_pct": 0.0, "since": time.monotonic()}
        self.get_logger().info(
            f"[{name}] 도킹 — 접점 붙임(시뮬: 즉시). 충전 {duration:.0f} s 대기 시작")

        t0 = time.monotonic()
        last_fb = 0.0
        try:
            while True:
                now = time.monotonic()
                elapsed = now - t0
                pct = min(target_pct, target_pct * (elapsed / duration)) if duration > 0 else target_pct
                with self._lock:
                    self._active[name]["charge_pct"] = pct

                if goal_handle.is_cancel_requested:
                    self.get_logger().info(f"[{name}] 충전 취소 (charge {pct:.0f}%)")
                    result.success = False
                    result.fail_reason = Dock.Result.CANCELED
                    result.charge_pct = float(pct)
                    goal_handle.canceled()
                    return result

                if elapsed >= duration:
                    self.get_logger().info(f"[{name}] 충전 완료 {target_pct:.0f}% ({elapsed:.0f} s)")
                    result.success = True
                    result.fail_reason = Dock.Result.NONE
                    result.charge_pct = float(target_pct)
                    goal_handle.succeed()
                    return result

                if elapsed >= self.dock_timeout_s:
                    self.get_logger().error(
                        f"[{name}] 충전이 {self.dock_timeout_s:.0f} s 안에 안 끝났다 — TIMEOUT")
                    result.success = False
                    result.fail_reason = Dock.Result.TIMEOUT
                    result.charge_pct = float(pct)
                    goal_handle.abort()
                    return result

                if now - last_fb >= self.feedback_period_s:
                    last_fb = now
                    fb = Dock.Feedback()
                    fb.charge_pct = float(pct)
                    fb.remaining_s = float(max(0.0, duration - elapsed))
                    goal_handle.publish_feedback(fb)

                time.sleep(0.1)
        finally:
            with self._lock:
                self._active.pop(name, None)

    # ── 표시 ──────────────────────────────────────────────────────────────
    def _publish_state(self):
        with self._lock:
            parts = [f"{n}=charging:{v['charge_pct']:.0f}" for n, v in sorted(self._active.items())]
        idle = [n for n in sorted(self.pads) if n not in {p.split("=")[0] for p in parts}]
        parts += [f"{n}=idle" for n in idle]
        msg = String()
        msg.data = " | ".join(parts) if parts else "-"
        self._state_pub.publish(msg)


def main():
    rclpy.init()
    node = DockingServer()
    # execute 콜백이 충전 시간만큼 블로킹한다 — 두 로봇이 동시에 충전하려면
    # 스레드가 둘 이상이어야 한다(+ 상태 발행 타이머 1).
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
