"""
pick_place_server — PickCarrier · PlaceCarrier 액션 서버.

    ros2 run cobot3_manipulation pick_place_server --ros-args -r __ns:=/robot1

★ 액션 이름이 상대이름이다 — manipulation/pick_carrier · manipulation/
  place_carrier 에 / 가 앞에 없고, 노드가 뜬 네임스페이스가 붙는다. robot1 로
  띄우면 /robot1/manipulation/pick_carrier 가 된다. 그래서 이 노드는 자기가
  어느 로봇인지 모른다 — 같은 네임스페이스의 task_manager 만 이 서버를 부른다.
  보통은 launch 가 네임스페이스를 준다:
    ros2 launch cobot3_bringup mission_nodes.launch.py robots:=robot1,robot2

  ★ 아직 남은 것: self.sim(SimClient)은 sim_backend 의 단일 로봇
  (nova_carter1)에 붙는다. 두 번째 로봇의 pick/place 를 실제로 돌리려면
  sim_backend 가 로봇을 구분해야 한다 — ROS 배선과 별개의 작업이다.

시뮬 실행 방법 (실제 Isaac API 호출은 전부 sim_backend.py 프로세스가 한다.
이유는 isaacpjt/ros_bridge/sim_backend.py 상단 주석 참고):
    isaac_python isaacpjt/ros_bridge/sim_backend.py     # 터미널 1
    ros_set && source install/setup.bash
    ros2 run cobot3_manipulation pick_place_server      # 터미널 2

동작:
  PickCarrier:  OBSERVE -> APPROACH -> DESCEND -> SUCTION -> LIFT -> STOW
  PlaceCarrier: MOVE -> LOWER -> RELEASE -> RETREAT
  goal 은 각각 (variant, qr_pose) · variant 뿐이다(a2caf63 리팩터 이후).
  "잡아도/놓아도 되는 캐리어인가" 는 task_manager 가 goal 을 보내기 전에
  끝낸다 — 이 노드는 캐리어의 ID 를 모른다. 다만 variant 는 안다(자기
  설정 인덱스 — grasp.yaml/place.yaml 키). 파지점 · 배치점 계산은 여기
  안에서 한다: OBSERVE 단계가 qr_pose(prior) + grasp.yaml 의
  T_QR_grasp_xyz 로 대략 파지점(prior)을 만들고, 손목캠을 그 위로 가져가
  sim_backend.pick_observe_flange() -> cobot3_perception.flange_topview.
  detect_flange() 로 진짜 파지점(중심 xy·윗면 z·yaw)을 잰다 — QR prior 는
  검증 안 된 거리/각도에서 오차가 커질 수 있어서(WP_PICK 이 taught_poses.yaml
  관측 자세의 원래 티칭 거리보다 훨씬 가깝다) 그대로 쓰지 않는다. 이 관측이
  실패하면 NO_FLANGE 로 그 자리서 중단한다(APPROACH 를 시도하지 않는다).
  PlaceCarrier 는 variant 로 place.yaml 에서 슬롯 자세를 찾는다.
  VerifyCarrier 서비스는 그 인터록과 함께 인터페이스에서 삭제됐다.

이 노드가 declare_parameter 로 정의하는 것 (PickCarrier/PlaceCarrier.action
★ 블록 그대로):
  grasp_yaml           cobot3_bringup/config/grasp.yaml 경로. T_QR_grasp_xyz
                       를 여기서 찾는다(OBSERVE 단계, variant 키).
  place_yaml           cobot3_bringup/config/place.yaml 경로. variant → 배치
                       자세(base_link). 좌표는 아직 로더 실물 미정 — place.yaml
                       주석 참고.
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
  place_drop_m         PlaceCarrier — 슬롯 바닥 위 이 높이에서 흡착 OFF
                       (구 release_height_m — PlaceCarrier.action ★ 블록 이름으로 맞춤)
  place_pos_tol_m, place_yaw_tol_rad
                       ★ 알려진 갭: 선언만 하고 아직 검증하지 않는다 —
                       비교할 배치 목표 실측(포트 마커)이 씬에 없다
                       (PlaceCarrier.action "남은 것" 참고).

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

import numpy as np
import rclpy
import yaml
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from cobot3_interfaces.action import PickCarrier, PlaceCarrier

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
GRASP_YAML = WS_ROOT / "src/cobot3_bringup/config/grasp.yaml"
PLACE_YAML = WS_ROOT / "src/cobot3_bringup/config/place.yaml"


def quat_wxyz_to_mat(w, x, y, z):
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


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

# sim_backend.get_status()["phase"] 문자열 -> PickCarrier.Feedback 상수.
# OBSERVE 는 sim_backend 가 내는 phase 가 아니라 이 노드가 goal 을 받자마자
# 직접 publish_feedback 으로 알린다(아래 _execute).
_PHASE = {"APPROACH": PickCarrier.Feedback.APPROACH, "DESCEND": PickCarrier.Feedback.DESCEND,
         "SUCTION": PickCarrier.Feedback.SUCTION, "LIFT": PickCarrier.Feedback.LIFT,
         "STOW": PickCarrier.Feedback.STOW}
_FAIL = {"NONE": PickCarrier.Result.NONE, "NO_IK": PickCarrier.Result.NO_IK,
        "COLLISION": PickCarrier.Result.COLLISION, "NO_FLANGE": PickCarrier.Result.NO_FLANGE,
        "NO_ATTACH": PickCarrier.Result.NO_ATTACH, "SLIP": PickCarrier.Result.SLIP,
        "OFF_FLANGE": PickCarrier.Result.OFF_FLANGE, "CANCELED": PickCarrier.Result.CANCELED}

# sim_backend 의 place_phase*() 는 내부적으로 phase 를 여전히
# APPROACH/DESCEND/RELEASE/RETRACT 로 부른다(이건 sim_backend 자체
# 표기이지 PlaceCarrier.action 의 MOVE/LOWER/RELEASE/RETREAT 와는 별개 이름
# 공간이다) — 그래서 딕셔너리 키는 그대로 두고 값만 현재 액션 상수로 맞춘다.
_PLACE_PHASE = {"APPROACH": PlaceCarrier.Feedback.MOVE, "DESCEND": PlaceCarrier.Feedback.LOWER,
               "RELEASE": PlaceCarrier.Feedback.RELEASE, "RETRACT": PlaceCarrier.Feedback.RETREAT}
_PLACE_FAIL = {"NONE": PlaceCarrier.Result.NONE, "NO_IK": PlaceCarrier.Result.NO_IK,
              "COLLISION": PlaceCarrier.Result.COLLISION,
              "PORT_OCCUPIED": PlaceCarrier.Result.PORT_OCCUPIED,
              "NOT_GRIPPED": PlaceCarrier.Result.NOT_GRIPPED,
              "UNKNOWN_VARIANT": PlaceCarrier.Result.UNKNOWN_VARIANT,
              "CANCELED": PlaceCarrier.Result.CANCELED}


class PickPlaceServer(Node):
    def __init__(self):
        super().__init__("pick_place_server")
        # ── PickCarrier/PlaceCarrier.action ★ 블록 — 값 출처는 모듈 독스트링 ──
        self.declare_parameter("grasp_yaml", str(GRASP_YAML))
        self.declare_parameter("place_yaml", str(PLACE_YAML))
        self.declare_parameter("approach_dist_m", 0.15)          # grasp.yaml pre_grasp_m
        self.declare_parameter("grip_gaps_m", [0.005, 0.002, 0.0, -0.003])
        self.declare_parameter("cup_diameter_m", 0.050)
        self.declare_parameter("offset_limit_m", 0.015)          # (80mm-50mm)/2
        self.declare_parameter("coaxial_force_limit", 200.0)
        self.declare_parameter("shear_force_limit", 100.0)
        self.declare_parameter("max_grip_distance", 0.03)
        self.declare_parameter("lift_height_m", 0.10)            # 12_pick_test.py 검증값
        self.declare_parameter("place_drop_m", 0.005)
        self.declare_parameter("place_pos_tol_m", 0.002)         # ★ 알려진 갭: 미검증
        self.declare_parameter("place_yaw_tol_rad", 0.017)       # ★ 알려진 갭: 미검증

        grasp_path = Path(self.get_parameter("grasp_yaml").value)
        place_path = Path(self.get_parameter("place_yaml").value)
        self.grasp = yaml.safe_load(grasp_path.read_text(encoding="utf-8"))["magazines"]
        self.place = yaml.safe_load(place_path.read_text(encoding="utf-8"))["magazines"]

        self.sim = SimClient()
        # sim_backend.py 가 로봇 두 대(robot1/robot2)를 한 소켓에서 같이
        # 관리한다 — 네임스페이스를 그대로 robot_id 로 실어 보내야 RPC 가
        # 어느 로봇을 움직일지 안다(_safe_call/_safe_call_place 참고).
        self.robot_id = self.get_namespace().strip("/") or "robot1"
        cb = ReentrantCallbackGroup()
        self._server = ActionServer(
            self, PickCarrier, "manipulation/pick_carrier",
            execute_callback=self._execute,
            goal_callback=self._on_goal, cancel_callback=self._on_cancel,
            callback_group=cb)
        self._place_server = ActionServer(
            self, PlaceCarrier, "manipulation/place_carrier",
            execute_callback=self._execute_place,
            goal_callback=self._on_goal, cancel_callback=self._on_cancel,
            callback_group=cb)
        ns = self.get_namespace().rstrip("/")
        self.get_logger().info(
            f"pick_place_server ready — {ns}/manipulation/pick_carrier, "
            f"{ns}/manipulation/place_carrier")

    def _on_goal(self, goal_request):
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def _poll_until(self, thread, goal_handle, feedback, phase_map=_PHASE):
        while thread.is_alive():
            try:
                status = self.sim.get_status(self.robot_id)
                feedback.phase = phase_map.get(status["phase"], feedback.phase)
                goal_handle.publish_feedback(feedback)
            except SimClientError as e:
                self.get_logger().warn(f"status 폴링 실패: {e}")
            time.sleep(0.15)

    def _flange_pose_from_qr(self, variant, qr_pose_stamped):
        """variant + qr_pose(prior, CarrierScan 규약 +Z=바깥) -> flange_pose_base_link.

        task_manager._step3_compute_flange_pose 에 있던 수식을 그대로 옮긴
        것이다(a2caf63 리팩터로 파지점 계산 책임이 여기로 넘어왔다).
        """
        entry = self.grasp[variant]         # variant 가 없으면 KeyError -> NO_FLANGE
        p = qr_pose_stamped.pose
        w, x, y, z = p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z
        # CarrierScan 규약(+Z=바깥) -> grasp.yaml 내부 규약(+Z=안쪽)으로 되돌린다.
        # carrier_code_reader.py 와 같은 x축-180도 변환의 역이며, 자기 자신의
        # 역이다(두 번 적용하면 원래로 돌아온다).
        q_internal = np.array([-x, w, z, -y])
        R = quat_wxyz_to_mat(*q_internal)
        p_qr = np.array([p.position.x, p.position.y, p.position.z])

        T = np.array(entry["T_QR_grasp_xyz"])
        flange_pos = p_qr + R @ T

        # 로봇은 항상 수평(pitch=roll=0)이라는 전제로, base_link 프레임에서
        # 곧장 yaw 만 뽑아 flange_pose 의 orientation(+Z=위)을 만든다.
        yaw = float(np.arctan2(R[1, 0], R[0, 0]))
        half = yaw / 2.0
        return {"position": flange_pos.tolist(),
               "quat_wxyz": [float(np.cos(half)), 0.0, 0.0, float(np.sin(half))]}

    # ── PickCarrier ──
    def _execute(self, goal_handle):
        goal = goal_handle.request
        result = PickCarrier.Result()
        feedback = PickCarrier.Feedback()
        approach_dist_m = float(self.get_parameter("approach_dist_m").value)
        grip_gaps_m = list(self.get_parameter("grip_gaps_m").value)
        offset_limit_m = float(self.get_parameter("offset_limit_m").value)
        lift_height_m = float(self.get_parameter("lift_height_m").value)

        # ── OBSERVE: qr_pose(prior) + grasp.yaml 로 대략 파지점 계산 ──
        feedback.phase = PickCarrier.Feedback.OBSERVE
        goal_handle.publish_feedback(feedback)
        try:
            prior_pose = self._flange_pose_from_qr(goal.variant, goal.qr_pose)
        except KeyError:
            self.get_logger().warn(f"grasp.yaml 에 없는 variant: {goal.variant}")
            return self._abort(goal_handle, result, "NO_FLANGE")

        # ── OBSERVE 계속: 손목캠으로 prior 위를 다시 관측해 진짜 파지점을
        # 잰다(flange_topview.detect_flange) — PickCarrier.action ★ 블록이
        # 요구하는 단계이자 sim_backend.py pick_observe_flange 독스트링이
        # 채우는 "알려진 갭"이다. QR prior 는 검증 안 된 거리/각도에서 오차가
        # 커질 수 있어서(파지점 계산에 넣기 전에) 실제로 보고 확정한다.
        holder0 = {}
        t0 = threading.Thread(target=lambda: holder0.__setitem__(
            "r", self._safe_call("pick_observe_flange",
                                 flange_pose_base_link=prior_pose, variant=goal.variant)))
        t0.start()
        self._poll_until(t0, goal_handle, feedback)
        r0 = holder0.get("r")
        if r0 is None or not r0.get("ok"):
            # ★ prior 를 같이 찍는다. 이유만으로는 "멀어서 못 봤다" 와
            #   "엉뚱한 데를 봤다" 가 안 갈린다 — 손목캠을 어디로 보냈는지가
            #   있어야 QR prior 가 틀린 것인지 거리가 문제인지 판단할 수 있다.
            #   (flange_topview 의 이유 문구별 뜻)
            #     "충분히 큰 성분이 없다 (최대 N px, 기대 M px)"  거리 문제.
            #         N 이 M 보다 많이 작으면 멀다 — 정차선을 매거진 쪽으로.
            #     "예상 위치가 화면 밖이다 uv=(...)"             prior 가 틀렸다.
            #     "... 영역이 탐색 창 안에 없다"                  prior 가 틀렸다.
            #     "크기 불일치 W x H mm"                         다른 물체를 봤다.
            #     "depth 도 color 도 없다"                       카메라 스트림 문제.
            pos = (prior_pose or {}).get("position") or []
            where = (f"({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f})"
                     if len(pos) == 3 else "?")
            self.get_logger().warn(
                f"OBSERVE(손목캠) 실패: {(r0 or {}).get('reason', '응답 없음')} — "
                f"파지를 중단한다 [variant={goal.variant}, "
                f"prior(base_link)={where}]")
            return self._abort(goal_handle, result, "NO_FLANGE")
        flange_pose_base_link = {"position": r0["position"], "quat_wxyz": r0["quat_wxyz"]}

        # ── APPROACH ──
        feedback.phase = PickCarrier.Feedback.APPROACH
        goal_handle.publish_feedback(feedback)
        holder = {}
        params = dict(
            flange_pose_base_link=flange_pose_base_link,
            approach_dist_m=approach_dist_m)
        t = threading.Thread(target=lambda: holder.__setitem__(
            "r", self._safe_call("pick_phase1_approach", **params)))
        t.start()
        self._poll_until(t, goal_handle, feedback)
        r1 = holder.get("r")
        if r1 is None or not r1.get("success"):
            # ★ 실패를 그냥 올리지 말고 어디서 왜인지 남긴다. OBSERVE 쪽이
            #   이유를 버려서 "팔이 못 닿는다" 로 오진하고 좌표를 몇 시간
            #   튜닝한 적이 있다(진짜 원인은 IK 베이스 동기화 누락이었다).
            #   여기 찍는 값의 뜻:
            #     phase        sim_backend 가 어느 단계에서 죽었나
            #     flange       손목캠이 실제로 잰 파지점 (base_link 기준)
            #     approach     그 위 몇 m 로 올라가려 했나
            #   APPROACH 목표는 flange + [0,0,approach] 이므로, 이 둘이면
            #   팔이 어디로 가려다 실패했는지 그대로 재현할 수 있다.
            pos = (flange_pose_base_link or {}).get("position") or []
            where = (f"({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f})"
                     if len(pos) == 3 else "?")
            self.get_logger().warn(
                f"PICK APPROACH 실패: "
                f"reason={(r1 or {}).get('fail_reason', '응답 없음')} "
                f"phase={(r1 or {}).get('phase', '?')} "
                f"[variant={goal.variant}, flange(base_link)={where}, "
                f"approach={approach_dist_m:.3f} m]")
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
            self.get_logger().warn(
                "PICK DESCEND~STOW 응답 없음 — sim_backend 가 결과를 안 줬다 "
                "(타임아웃 120 s 또는 통신 끊김)")
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
        place_drop_m = float(self.get_parameter("place_drop_m").value)

        slot_pose_base_link = self.place.get(goal.variant)
        if slot_pose_base_link is None:
            self.get_logger().warn(f"place.yaml 에 없는 variant: {goal.variant}")
            return self._abort_place(goal_handle, result, "UNKNOWN_VARIANT")

        # ── MOVE ──
        feedback.phase = PlaceCarrier.Feedback.MOVE
        goal_handle.publish_feedback(feedback)
        holder = {}
        params = dict(slot_pose_base_link=slot_pose_base_link, approach_dist_m=approach_dist_m)
        t = threading.Thread(target=lambda: holder.__setitem__(
            "r", self._safe_call_place("place_phase1_approach", **params)))
        t.start()
        self._poll_until(t, goal_handle, feedback, phase_map=_PLACE_PHASE)
        r1 = holder.get("r")
        if r1 is None or not r1.get("success"):
            pos = (slot_pose_base_link or {}).get("position") or []
            where = (f"({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f})"
                     if len(pos) == 3 else "?")
            self.get_logger().warn(
                f"PLACE MOVE 실패: "
                f"reason={(r1 or {}).get('fail_reason', '응답 없음')} "
                f"phase={(r1 or {}).get('phase', '?')} "
                f"[variant={goal.variant}, slot(base_link)={where}, "
                f"approach={approach_dist_m:.3f} m]")
            return self._abort_place(goal_handle, result,
                                     (r1 or {}).get("fail_reason", "NO_IK"))

        # ── LOWER -> RELEASE -> RETREAT ──
        holder2 = {}
        t2 = threading.Thread(target=lambda: holder2.__setitem__(
            "r", self._safe_call_place("place_phase2_finish", release_height_m=place_drop_m)))
        t2.start()
        self._poll_until(t2, goal_handle, feedback, phase_map=_PLACE_PHASE)
        r2 = holder2.get("r")
        if r2 is None:
            self.get_logger().warn(
                "PLACE LOWER~RETREAT 응답 없음 — sim_backend 가 결과를 안 줬다")
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
        kw.setdefault("robot_id", self.robot_id)
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
        kw.setdefault("robot_id", self.robot_id)
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
