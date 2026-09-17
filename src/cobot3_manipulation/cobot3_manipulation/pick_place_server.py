"""
pick_place_server — PickCarrier · PlaceCarrier 액션 서버.

    ros2 run cobot3_manipulation pick_place_server

시뮬 실행 방법 (실제 Isaac API 호출은 전부 sim_backend.py 프로세스가 한다.
이유는 isaacpjt/ros_bridge/sim_backend.py 상단 주석 참고):
    isaac_python isaacpjt/ros_bridge/sim_backend.py     # 터미널 1
    ros_set && source install/setup.bash
    ros2 run cobot3_manipulation pick_place_server      # 터미널 2

동작:
  PickCarrier:  APPROACH -> DESCEND -> SUCTION -> LIFT -> STOW
  PlaceCarrier: APPROACH -> DESCEND -> RELEASE -> RETRACT
  둘 다 goal 은 pose 하나뿐이다(PickCarrier/PlaceCarrier.action 리팩터
  이후). "잡아도/놓아도 되는 캐리어인가" 는 task_manager 가 goal 을
  보내기 전에 끝낸다 — 이 노드는 캐리어의 ID·kind·variant 를 모른다.
  VerifyCarrier 서비스는 그 인터록과 함께 인터페이스에서 삭제됐다.

이 노드가 declare_parameter 로 정의하는 것 (PickCarrier/PlaceCarrier.action
★ 블록 그대로):
  approach_dist_m      플랜지/슬롯 위 이 높이에서 수직 하강 시작
  grip_gaps_m          흡착 재시도 갭 시퀀스
  cup_diameter_m       참고용 (offset_limit_m 을 직접 declare 해서 실제로는 안 씀)
  offset_limit_m       흡착 순간 횡오차 상한 — OFF_FLANGE 판정
  coaxial_force_limit, shear_force_limit, max_grip_distance
                       ★ 알려진 갭: 선언만 하고 아직 sim_backend 로 안 흘려보낸다
                       (씬 초기화 시 이미 같은 값으로 하드코딩돼 있다 — sim_backend.py
                       COAXIAL_FORCE_LIMIT 등). 런타임에 바꿔야 할 일이 생기면
                       그때 sim_backend 에 setter RPC 를 추가한다.
  lift_height_m        흡착 후 들어 올리는 높이
  release_height_m     PlaceCarrier — 슬롯 바닥 위 이 높이에서 흡착 OFF

  기본값은 action 파일의 "제안"값이 아니라 grasp.yaml/12_pick_test.py 가
  실측으로 검증한 값을 쓴다(approach_dist_m=0.15, lift_height_m=0.10) —
  "제안"은 액션 설계자가 인터페이스만 보고 어림한 것이고, 이 값들은 실제로
  Isaac 에서 돌려서 10/10 성공을 낸 값이다. 다르게 튜닝하고 싶으면 launch
  파라미터로 덮어쓴다.
"""

import sys
import threading
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from cobot3_interfaces.action import PickCarrier, PlaceCarrier

def _add_ros_bridge_to_syspath():
    # colcon 빌드가 src/<pkg>/<pkg>/file.py 를 build/ 밑으로 복사하거나
    # symlink 하는데, 정확히 몇 단계 위가 cobot3_ws 인지는 빌드 방식에 따라
    # 달라진다 — parents[N] 고정 인덱스를 썼다가 한 번 깨졌다(4가 아니라
    # 3이어야 했다). 위로 올라가며 isaacpjt/ros_bridge 를 직접 찾는다.
    p = Path(__file__).resolve()
    for _ in range(10):
        candidate = p / "isaacpjt" / "ros_bridge"
        if candidate.is_dir():
            sys.path.insert(0, str(candidate))
            return
        if p.parent == p:
            break
        p = p.parent
    raise RuntimeError("isaacpjt/ros_bridge 를 못 찾았다 (cobot3_ws 밖에서 실행?)")


_add_ros_bridge_to_syspath()
from sim_client import SimClient, SimClientError  # noqa: E402

_PHASE = {"APPROACH": PickCarrier.Feedback.APPROACH, "DESCEND": PickCarrier.Feedback.DESCEND,
         "SUCTION": PickCarrier.Feedback.SUCTION, "LIFT": PickCarrier.Feedback.LIFT,
         "STOW": PickCarrier.Feedback.STOW}
_FAIL = {"NONE": PickCarrier.Result.NONE, "NO_IK": PickCarrier.Result.NO_IK,
        "COLLISION": PickCarrier.Result.COLLISION,
        "NO_ATTACH": PickCarrier.Result.NO_ATTACH, "SLIP": PickCarrier.Result.SLIP,
        "OFF_FLANGE": PickCarrier.Result.OFF_FLANGE, "CANCELED": PickCarrier.Result.CANCELED}

_PLACE_PHASE = {"APPROACH": PlaceCarrier.Feedback.APPROACH, "DESCEND": PlaceCarrier.Feedback.DESCEND,
               "RELEASE": PlaceCarrier.Feedback.RELEASE, "RETRACT": PlaceCarrier.Feedback.RETRACT}
_PLACE_FAIL = {"NONE": PlaceCarrier.Result.NONE, "NO_IK": PlaceCarrier.Result.NO_IK,
              "COLLISION": PlaceCarrier.Result.COLLISION,
              "PORT_OCCUPIED": PlaceCarrier.Result.PORT_OCCUPIED,
              "NOT_GRIPPED": PlaceCarrier.Result.NOT_GRIPPED,
              "CANCELED": PlaceCarrier.Result.CANCELED}


class PickPlaceServer(Node):
    def __init__(self):
        super().__init__("pick_place_server")
        # ── PickCarrier/PlaceCarrier.action ★ 블록 — 값 출처는 모듈 독스트링 ──
        self.declare_parameter("approach_dist_m", 0.15)          # grasp.yaml pre_grasp_m
        self.declare_parameter("grip_gaps_m", [0.005, 0.002, 0.0, -0.003])
        self.declare_parameter("cup_diameter_m", 0.050)
        self.declare_parameter("offset_limit_m", 0.015)          # (80mm-50mm)/2
        self.declare_parameter("coaxial_force_limit", 200.0)
        self.declare_parameter("shear_force_limit", 100.0)
        self.declare_parameter("max_grip_distance", 0.03)
        self.declare_parameter("lift_height_m", 0.10)            # 12_pick_test.py 검증값
        self.declare_parameter("release_height_m", 0.005)

        self.sim = SimClient()
        cb = ReentrantCallbackGroup()
        self._server = ActionServer(
            self, PickCarrier, "/manipulation/pick_carrier",
            execute_callback=self._execute,
            goal_callback=self._on_goal, cancel_callback=self._on_cancel,
            callback_group=cb)
        self._place_server = ActionServer(
            self, PlaceCarrier, "/manipulation/place_carrier",
            execute_callback=self._execute_place,
            goal_callback=self._on_goal, cancel_callback=self._on_cancel,
            callback_group=cb)
        self.get_logger().info(
            "pick_place_server ready — /manipulation/pick_carrier, /manipulation/place_carrier")

    def _on_goal(self, goal_request):
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def _poll_until(self, thread, goal_handle, feedback, phase_map=_PHASE):
        while thread.is_alive():
            try:
                status = self.sim.get_status()
                feedback.phase = phase_map.get(status["phase"], feedback.phase)
                goal_handle.publish_feedback(feedback)
            except SimClientError as e:
                self.get_logger().warn(f"status 폴링 실패: {e}")
            time.sleep(0.15)

    # ── PickCarrier ──
    def _execute(self, goal_handle):
        goal = goal_handle.request
        result = PickCarrier.Result()
        feedback = PickCarrier.Feedback()
        approach_dist_m = float(self.get_parameter("approach_dist_m").value)
        grip_gaps_m = list(self.get_parameter("grip_gaps_m").value)
        offset_limit_m = float(self.get_parameter("offset_limit_m").value)
        lift_height_m = float(self.get_parameter("lift_height_m").value)

        # ── APPROACH ──
        feedback.phase = PickCarrier.Feedback.APPROACH
        goal_handle.publish_feedback(feedback)
        holder = {}
        flange = goal.flange_pose.pose
        params = dict(
            flange_pose_base_link={
                "position": [flange.position.x, flange.position.y, flange.position.z],
                "quat_wxyz": [flange.orientation.w, flange.orientation.x,
                             flange.orientation.y, flange.orientation.z]},
            approach_dist_m=approach_dist_m)
        t = threading.Thread(target=lambda: holder.__setitem__(
            "r", self._safe_call("pick_phase1_approach", **params)))
        t.start()
        self._poll_until(t, goal_handle, feedback)
        r1 = holder.get("r")
        if r1 is None or not r1.get("success"):
            return self._abort(goal_handle, result,
                               (r1 or {}).get("fail_reason", "NO_IK"))

        # ── DESCEND -> SUCTION -> LIFT -> STOW ──
        holder2 = {}
        t2 = threading.Thread(target=lambda: holder2.__setitem__(
            "r", self._safe_call("pick_phase2_finish", timeout_s=120.0,
                                 grip_gaps_m=grip_gaps_m, offset_limit_m=offset_limit_m,
                                 lift_height_m=lift_height_m)))
        t2.start()
        self._poll_until(t2, goal_handle, feedback)
        r2 = holder2.get("r")
        if r2 is None:
            return self._abort(goal_handle, result, "NO_ATTACH")

        result.success = bool(r2.get("success", False))
        result.fail_reason = _FAIL.get(r2.get("fail_reason", "NONE"), PickCarrier.Result.NONE)
        offset_mm = float(r2.get("final_offset_m", 0.0)) * 1000
        limit_mm = float(r2.get("offset_limit_m", offset_limit_m)) * 1000
        if result.success:
            goal_handle.succeed()
            self.get_logger().info(f"PICK 성공  offset {offset_mm:.1f}/{limit_mm:.1f} mm")
        else:
            goal_handle.abort()
            self.get_logger().warn(
                f"PICK 실패  reason={r2.get('fail_reason')}  offset {offset_mm:.1f}/{limit_mm:.1f} mm")
        return result

    # ── PlaceCarrier ──
    def _execute_place(self, goal_handle):
        """★ 알려진 갭: PORT_OCCUPIED 판정은 안 한다 — 씬에 슬롯 자체가
        없어서(포트 지오메트리는 다음 범위) 항상 비어 있다고 본다."""
        goal = goal_handle.request
        result = PlaceCarrier.Result()
        feedback = PlaceCarrier.Feedback()
        approach_dist_m = float(self.get_parameter("approach_dist_m").value)
        release_height_m = float(self.get_parameter("release_height_m").value)

        # ── APPROACH ──
        feedback.phase = PlaceCarrier.Feedback.APPROACH
        goal_handle.publish_feedback(feedback)
        holder = {}
        slot = goal.slot_pose.pose
        params = dict(slot_pose_base_link={
            "position": [slot.position.x, slot.position.y, slot.position.z],
            "quat_wxyz": [slot.orientation.w, slot.orientation.x,
                         slot.orientation.y, slot.orientation.z]},
            approach_dist_m=approach_dist_m)
        t = threading.Thread(target=lambda: holder.__setitem__(
            "r", self._safe_call_place("place_phase1_approach", **params)))
        t.start()
        self._poll_until(t, goal_handle, feedback, phase_map=_PLACE_PHASE)
        r1 = holder.get("r")
        if r1 is None or not r1.get("success"):
            return self._abort_place(goal_handle, result,
                                     (r1 or {}).get("fail_reason", "NO_IK"))

        # ── DESCEND -> RELEASE -> RETRACT ──
        holder2 = {}
        t2 = threading.Thread(target=lambda: holder2.__setitem__(
            "r", self._safe_call_place("place_phase2_finish", release_height_m=release_height_m)))
        t2.start()
        self._poll_until(t2, goal_handle, feedback, phase_map=_PLACE_PHASE)
        r2 = holder2.get("r")
        if r2 is None:
            return self._abort_place(goal_handle, result, "COLLISION")

        result.success = bool(r2.get("success", False))
        result.fail_reason = _PLACE_FAIL.get(r2.get("fail_reason", "NONE"), PlaceCarrier.Result.NONE)
        if result.success:
            goal_handle.succeed()
            self.get_logger().info("PLACE 성공")
        else:
            goal_handle.abort()
            self.get_logger().warn(f"PLACE 실패  reason={r2.get('fail_reason')}")
        return result

    def _safe_call_place(self, method, **kw):
        try:
            return self.sim.call(method, **kw)
        except SimClientError as e:
            self.get_logger().error(f"{method} 실패: {e}")
            return {"success": False, "fail_reason": "NO_IK"}

    def _abort_place(self, goal_handle, result, fail_reason):
        result.success = False
        result.fail_reason = _PLACE_FAIL.get(fail_reason, PlaceCarrier.Result.NO_IK)
        goal_handle.abort()
        return result

    def _safe_call(self, method, **kw):
        try:
            return self.sim.call(method, **kw)
        except SimClientError as e:
            self.get_logger().error(f"{method} 실패: {e}")
            return {"success": False, "fail_reason": "NO_IK"}

    def _abort(self, goal_handle, result, fail_reason):
        result.success = False
        result.fail_reason = _FAIL.get(fail_reason, PickCarrier.Result.NO_IK)
        goal_handle.abort()
        return result


def main():
    rclpy.init()
    node = PickPlaceServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
