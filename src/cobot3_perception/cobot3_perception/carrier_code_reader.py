"""
carrier_code_reader — QR 판독 노드. docs/08_ROS2_NODE_Graph.html 확정안
(§01/§03)의 carrier_code_reader 그대로 — perception/carrier_detected 발행과
perception/carrier_scan 서비스를 같은 노드 하나가 맡는다.

    ros2 run cobot3_perception carrier_code_reader --ros-args -r __ns:=/robot1

★ 이름이 전부 상대이름이다 — perception/… · orchestrator/state · amcl_pose 에
  / 가 앞에 없고, 노드가 뜬 네임스페이스가 붙는다. robot1 로 띄우면 구독처가
  /robot1/amcl_pose · /robot1/orchestrator/state 가 되고, 발행처가
  /robot1/perception/carrier_detected 가 된다. 이 노드는 자기가 어느 로봇인지
  모르고, 알 필요도 없다 — 이전 판이 /robot1/amcl_pose 를 소스에 박아 두는
  바람에 로봇을 늘릴 수 없었던 자리가 여기다(docs/02 §2 "carrier_code_reader
  의 /robot1/amcl_pose 하드코딩"). 보통은 launch 가 네임스페이스를 준다:
    ros2 launch cobot3_bringup mission_nodes.launch.py robots:=robot1,robot2

qr_pose 계산은 cobot3_perception.qr_pose(깊이 평면 기반, docs/08 §5 검증:
위치 0.43mm, yaw 0.25도)를 그대로 쓴다 — sim_backend.scan_qr() 가 이미
그 모듈을 불러 쓰고 있어서, 이 노드는 결과를 CarrierScan 응답 형태로
옮겨 담기만 한다.

실제로는 "매 프레임이 아니라 다수결 통과한 판독만" 이지만(CarrierScan.srv
주석), 지금은 서비스 요청으로 트리거된 한 번의 관측을 다중 프레임
평균(qr_pose.aggregate_qr_poses)으로 대신한다 — 판정 방식은 다르지만
"단발성 오판독을 거른다"는 목적은 같다.

★ 7ea79bf 리팩터(CarrierScan: 토픽 메시지 → 서비스) 반영: 이전에는
/perception/carrier_scan 토픽에 CarrierScan.msg 를 발행하고, 그걸 읽을
타이밍은 별도의 로컬 /perception/scan_now (std_srvs/Trigger) 로 맞췄다.
지금은 CarrierScan.srv 요청 자체가 "지금 읽어라" 트리거이고 응답에
found·header·payload·qr_pose 가 그대로 실려 나가므로, scan_now 트리거가
따로 필요 없다 — 그 자리를 CarrierScan 서비스 하나가 대신한다.

/perception/carrier_detected (①, 순찰 중 "QR 이 보인다" 신호)는 확정안
표(§01·§03)에 이 노드가 보내는 것으로 돼 있다. 방법은 isaacpjt/tools/
sweep_scan.py 가 검증한 것과 같다 — 팔을 관측 자세(observe_pose)로 고정해
두고 베이스가 지나가는 동안 손목 카메라를 계속 디코드 시도한다. QR 판독
가능 거리가 0.45m 안쪽뿐이라(frames.yaml qr.measured_1280x720), 이 자세는
patrol 이 실제로 선반 앞(shelves.yaml 의 waypoint±standoff 영역, amcl_pose
로 판정)을 지날 때만 잡는다 — 그 전까지 팔을 건드리면 개활지 회전 구간과
간섭할 수 있고,
PICK/PLACE 중(HOLD~PLACE, /orchestrator/state 로 판정)에는 pick_place_server
가 같은 팔을 쓰고 있어 절대 건드리면 안 된다. 한 번 감지해서 발행하면 그
구역에 머무는 동안은 재폴링하지 않는다(task_manager 가 HOLD 로 넘어가길
기다리는 것뿐이라 계속 볼 이유가 없다).

CarrierScan.msg 리팩터 이후: raw_payload · symbology · frame_votes ·
decode_latency_ms · range_m · reader_id · payload_valid 필드가 메시지에서
빠졌다(튜닝·분석 지표라 노드 로그로 이동 — n_used/n_total 은 아래 로그에
남긴다). decode_hz 는 이제 carrier_detected 폴링 주기로 실제로 쓰인다.
roi_fraction/vote_window/vote_required/publish_debug 는 여전히 ★ 알려진
갭: 선언만 하고 다중 프레임 다수결에 연결하지 않았다 — scan_qr 이 서비스
쪽에서는 이미 자체적으로 n_frames 평균을 낸다(qr_pose.aggregate_qr_poses).
output_frame 은 실제로 쓴다: qr_pose 값이 base_link 기준 상대좌표라 그게
맞는 기본값이다("wrist_camera" 라고 적혀 있던 이전 버전은 명칭과 실제
좌표계가 어긋나 있었다).

★ a2caf63 리팩터(carrier_id · carrier_kind → payload) 반영: carrier_id ·
carrier_kind 필드와 MAGAZINE 상수가 메시지에서 사라졌다. 이 씬의 QR 은
숫자("1"/"2")만 담고 있어 <kind>-<serial> 파싱은 아직 필요 없으므로,
그 숫자 문자열을 payload 에 그대로 싣는다 — 실물 페이로드
(carriers.yaml 의 "C3.MAG.A17.9" 형식) 파싱이 붙으면 여기서 그 문자열을
만들어야 한다. qr_pose 도 759212e 이후 PoseStamped 가 아니라
geometry_msgs/Pose 다 — 좌표계는 header.frame_id 하나로 말한다.

층별 관측 자세: PATROL_ROUTE 는 두 점 사이 직선 왕복이다(task_manager.py).
task_manager 가 /orchestrator/state 에 patrol_target(지금 향하는 인덱스)을
같이 발행하고, 이 노드는 그걸 보고 선반 scan_passes 중 쓸 층(패스)을 고른다
— 끝점(1)으로 가는 중이면 가장 높은 level(2층), 시작점(0)으로 돌아가는
중이면 가장 낮은 level(1층). 그 패스의 arm_teach_pose(shelves.yaml, rad)가
6 칸 다 찼으면 그 관절값을 observe_pose 에 직접 넘기고, 아직 null 이면
(미티칭) taught_poses.yaml 의 pose_name 으로 폴백한다(FALLBACK_…).

★ 알려진 한계: 폴백 pose_name 두 개(shelf_1_top_close_centered,
s1_bottom_scan)는 taught_poses.yaml 의 같은 베이스 위치(x≈-6.5, y=1.45)
기준으로 티칭됐다. 관절값 티칭이 끝나면 이 폴백은 죽게 된다 — 그 전까지는
각 선반 어디서든 같은 관절값을 쓰므로, 구역 가장자리에서 선반과 팔이
간섭하지 않는지는 시뮬에서 직접 확인이 필요하다.
"""

import math
import sys
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node

from cobot3_interfaces.srv import CarrierScan
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from std_msgs.msg import Bool, String


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
SHELVES_YAML = WS_ROOT / "src/cobot3_bringup/config/shelves.yaml"

# ★ 미티칭(null arm_teach_pose)일 때만 쓰는 폴백 자세. task_manager 가
# /orchestrator/state 로 알려주는 patrol_target(지금 향하는 PATROL_ROUTE
# 인덱스)과 층을 잇는다 — 끝점(1)으로 가는 중이면 2층, 시작점(0)으로
# 돌아가는 중이면 1층. taught_poses.yaml 에 그 두 자세가 이미 있다.
# shelves.yaml 의 arm_teach_pose 가 6 칸 다 채워진 순간 이 폴백은 안 쓰인다.
FALLBACK_POSE_BY_PATROL_TARGET = {
    1: "shelf_1_top_close_centered",
    0: "s1_bottom_scan",
}
DEFAULT_POSE_NAME = "shelf_1_top_close_centered"

# ★ 선반마다 자세가 다른 경우는 위 표(진행 방향으로 층을 고르는 규칙)로 못
#   맞춘다. 스택 자리(PKG-OUT)가 그렇다 — 매거진 선반이 아니라 포장 출력
#   선반이고, taught_poses.yaml 에 전용 자세가 따로 있다.
#
#   이 표가 없던 동안에는 그 자세를 쓰려고 shelves.yaml 의 arm_teach_pose 에
#   관절값을 직접 박아 넣었다. 그러면 _observe() 가 joints_deg 경로로 갈리는데,
#   그건 "실측으로 티칭했다" 는 뜻이라 티칭 완료 판정(shapes.shelf_is_taught)
#   까지 같이 켜진다. 자세를 고르려고 티칭 상태를 건드리는 셈이라 이름으로
#   부르는 편이 맞다.
#
#   shelf_id 로 찾고, 없으면 위의 patrol_target 표로 내려간다.
FALLBACK_POSE_BY_SHELF = {
    "PKG-OUT": "packaging_output_scan",
}


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


class CarrierCodeReader(Node):
    def __init__(self):
        super().__init__("carrier_code_reader")
        self.declare_parameter("decode_hz", 5.0)
        self.declare_parameter("roi_fraction", 0.6)
        self.declare_parameter("vote_window", 5)
        self.declare_parameter("vote_required", 3)
        self.declare_parameter("output_frame", "base_link")
        self.declare_parameter("publish_debug", False)
        self.declare_parameter("shelves_yaml", str(SHELVES_YAML))
        # ★ 순찰 중 QR 폴링(= 팔을 관측 자세로 올리는 일)을 끄는 스위치.
        #
        #   왜 필요한가 — 팔은 충돌 계산에 안 들어간다. Nav2 코스트맵이 아는
        #   차체는 nav2 params 의 footprint(앞으로 0.14 m)뿐이고, 순찰은
        #   collision_monitor 도 거치지 않는다(nav_server 가 cmd_vel 에 직접
        #   쓴다). 그런데 관측 자세는 팔을 앞으로 길게 뻗는다 — 예를 들어
        #   taught_poses.yaml 의 s1_bottom_scan 은 J2=90°(어깨 수평)라
        #   M0609 리치 900 mm 가 거의 그대로 나간다. 선반 앞 여유가
        #   0.40~0.45 m 인 통로에서 그 팔을 뻗으면 아무도 안 막아준다.
        #
        #   그래서 베이스 순찰만 먼저 검증할 때 이걸 끈다. 끄면 이 노드는
        #   carrier_scan 서비스(HOLD 뒤 정지 상태에서 부르는 쪽)만 담당하고,
        #   주행 중에는 팔에 손대지 않는다.
        self.declare_parameter("patrol_scan", True)
        # ★ "QR 이 보인다" 와 "QR 옆에 왔다" 는 다르다.
        #
        #   손목캠은 매거진을 한참 앞에서부터 비스듬히 본다. 보이자마자
        #   carrier_detected 를 올리면 task_manager 가 그 자리에서 HOLD 하고,
        #   PICK 은 "멈춘 그 자리에서" 집으려 든다. 그런데 그 자리는 매거진
        #   정면이 아니다 — 실측: robot1 이 매거진에서 진행방향으로 0.45 m
        #   못 미친 곳에 섰고, 매거진까지 직선거리가 0.76 m 가 됐다(티칭 0.55).
        #   팔이 그만큼 대각선으로 못 뻗어서 손목캠이 파지점 위로 못 가고
        #   pick 이 NO_FLANGE 로 죽었다.
        #
        #   그래서 진행방향(base_link +x) 오프셋이 이 값 안에 들어왔을 때만
        #   감지로 친다. 그 전에는 계속 폴링만 하고 로봇은 그대로 지나간다.
        #   성공 경로였던 12_place_test2.py 의 go_to_pick 도 매거진 x 까지
        #   주행해서 정면에 선 뒤에 집는다 — 같은 조건을 만드는 것이다.
        #
        #   0 이하로 두면 이 게이트를 끈다(예전처럼 보이는 즉시 감지).
        self.declare_parameter("detect_align_tol_m", 0.15)
        self.sim = SimClient()
        # sim_backend.py 가 로봇 두 대(robot1/robot2)를 한 소켓에서 같이
        # 관리한다 — 네임스페이스를 그대로 robot_id 로 실어 보내야 observe_pose/
        # scan_qr 이 어느 로봇의 팔·손목캠을 쓸지 안다.
        self.robot_id = self.get_namespace().strip("/") or "robot1"
        self._load_shelves()
        self._srv = self.create_service(CarrierScan, "perception/carrier_scan", self._on_carrier_scan)

        # ── ① /perception/carrier_detected — patrol 중 선반 구역 실시간 감시 ──
        self._detected_pub = self.create_publisher(Bool, "perception/carrier_detected", 10)
        self.create_subscription(String, "orchestrator/state", self._on_orchestrator_state, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "amcl_pose", self._on_amcl_pose, 10)
        self._orchestrator_state = None
        self._patrol_target_idx = None  # task_manager 가 지금 향하는 PATROL_ROUTE 인덱스
        self._base_pose = None
        self._armed = False    # 이번 구역 체류 동안 observe_pose 를 이미 호출했는지
        self._found = False    # 이번 구역 체류 동안 이미 감지해서 발행했는지
        hz = float(self.get_parameter("decode_hz").value)
        self._patrol_scan = bool(self.get_parameter("patrol_scan").value)
        self._align_tol = float(self.get_parameter("detect_align_tol_m").value)
        self.create_timer(1.0 / hz, self._detect_tick)

        self.get_logger().info(
            f"carrier_code_reader ready [{self.get_namespace()}]")
        if not self._patrol_scan:
            self.get_logger().warning(
                "patrol_scan=False — 순찰 중 QR 폴링을 끈다. 주행 중에 팔을 "
                "관측 자세로 올리지 않는다(팔은 충돌 계산에 안 들어간다). "
                "carrier_scan 서비스는 그대로 동작한다.")

    # ── carrier_detected ────────────────────────────────────────────────
    def _on_amcl_pose(self, msg):
        self._base_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    # ── shelves.yaml ────────────────────────────────────────────────────
    def _load_shelves(self):
        path = Path(self.get_parameter("shelves_yaml").value)
        doc = yaml.safe_load(path.read_text(encoding="utf-8"))
        self._shelves = doc["shelves"]
        self.get_logger().info(
            f"shelves.yaml({path}) 로드 — 선반 {len(self._shelves)} 개")

    @staticmethod
    def _shelf_zone(shelf):
        """선반마다 QR 폴링 구역. waypoint_start~end(베이스가 도는 직선)를
        standoff_distance 만큼 상하좌우로 키운 사각형이다 — 그 직선이 이미
        전면 기둥에서 standoff_distance 만큼 물러나 있으므로, 이 여유는
        도착·회전·미세정렬이 빚는 오버슈트를 삼키는 역할이다.
        """
        a, b = shelf["waypoint_start"], shelf["waypoint_end"]
        d = float(shelf.get("standoff_distance") or 0.5)
        xs = (min(a["x"], b["x"]) - d, max(a["x"], b["x"]) + d)
        ys = (min(a["y"], b["y"]) - d, max(a["y"], b["y"]) + d)
        return xs, ys

    def _shelf_at(self, x, y):
        for shelf in self._shelves:
            xs, ys = self._shelf_zone(shelf)
            if xs[0] <= x <= xs[1] and ys[0] <= y <= ys[1]:
                return shelf
        return None

    def _current_shelf(self):
        if self._base_pose is None:
            return None
        return self._shelf_at(*self._base_pose)

    def _select_pass(self, shelf):
        """해당 선반의 관측 패스. patrol_target 이 1(끝점=선반 쪽)이거나
        아직 모르면 가장 높은 level(2층), 0(시작점 복귀)이면 가장 낮은
        level(1층) — 폴백 FALLBACK_… 이 patrol_target 1/0 에 2층/1층을 묶은
        것과 같은 규칙이다. 패스가 없으면 None.
        """
        passes = (shelf or {}).get("scan_passes") or []
        if not passes:
            return None
        levels = sorted({int(p.get("level") or 1) for p in passes})
        want = levels[-1] if self._patrol_target_idx != 0 else levels[0]
        for p in passes:
            if int(p.get("level") or 1) == want:
                return p
        return passes[0]

    def _teach_joints_deg(self, shelf):
        """관측 패스의 arm_teach_pose(shelves.yaml, rad 단위)를 도로 바꿔
        반환한다. 6 칸이 전부 채워져 있어야(티칭 완료) 하며 하나라도 null
        이면 None — 그 패스는 아직 못 쓴다.
        """
        p = self._select_pass(shelf)
        joints = (p or {}).get("arm_teach_pose")
        if not joints or not all(j is not None for j in joints):
            return None
        return [math.degrees(float(j)) for j in joints]

    def _observe(self, shelf, timeout_s=15.0):
        """관측 자세로 팔을 고정한다. shelves.yaml arm_teach_pose 가 티칭돼
        있으면 그 관절값(도)을 observe_pose 에 직접 넘기고, 아니면
        taught_poses.yaml pose_name 으로 폴백한다.

        ★ 어느 자세를 골랐는지 반드시 남긴다. 폴백 경로는 진행 방향
          (patrol_target)에 따라 2층/1층 자세를 번갈아 쓰는데
          (FALLBACK_POSE_BY_PATROL_TARGET), 시험 씬의 매거진은
          top_magazines 뿐이라 1층 자세로는 못 본다. 그러면 scan 이
          NOT_FOUND 로 죽는데, 로그만 봐서는 "왜 한 방향에서만 실패하지" 를
          알 수가 없다. shelf 가 None 인 것도 같이 찍는다 — 구역 밖이면
          선반별 티칭값 대신 이 폴백으로 떨어지기 때문이다.
        """
        joints = self._teach_joints_deg(shelf)
        shelf_id = (shelf or {}).get("shelf_id", "구역밖(None)")
        if joints is not None:
            self.get_logger().info(
                f"관측 자세: shelves.yaml {shelf_id} 티칭값 "
                f"{[round(j, 1) for j in joints]}")
            self.sim.call("observe_pose", joints_deg=joints,
                          robot_id=self.robot_id, timeout_s=timeout_s)
        else:
            # 선반 전용 자세가 있으면 그걸 먼저 쓴다(FALLBACK_POSE_BY_SHELF).
            # 없으면 진행 방향으로 층을 고르는 기존 규칙.
            pose = FALLBACK_POSE_BY_SHELF.get(shelf_id)
            how = f"선반 전용({shelf_id})"
            if pose is None:
                pose = FALLBACK_POSE_BY_PATROL_TARGET.get(
                    self._patrol_target_idx, DEFAULT_POSE_NAME)
                how = f"patrol_target={self._patrol_target_idx}"
            self.get_logger().info(
                f"관측 자세: 폴백 '{pose}' [{how}] "
                f"(shelf={shelf_id}) — shelves.yaml 의 arm_teach_pose 가 "
                f"비어 있어서 taught_poses.yaml 이름으로 간다")
            self.sim.call("observe_pose", pose_name=pose,
                          robot_id=self.robot_id, timeout_s=timeout_s)

    def _on_orchestrator_state(self, msg):
        new_state = None
        for part in msg.data.split("|"):
            part = part.strip()
            if part.startswith("state="):
                new_state = part[len("state="):]
            elif part.startswith("patrol_target="):
                try:
                    self._patrol_target_idx = int(part[len("patrol_target="):])
                except ValueError:
                    pass
        if new_state != self._orchestrator_state and new_state != "patrol":
            # patrol 을 벗어나면(HOLD/SCAN/PICK/NAV/PLACE) 팔이 다른 자세로
            # 움직일 것이므로, 다음에 patrol 로 돌아오면 다시 observe_pose 부터.
            # patrol_target 은 그대로 둔다 — HOLD~SCAN 동안 이 상태 문자열엔
            # 그 필드가 안 실리므로(task_manager 가 patrol 에서만 채운다),
            # 마지막으로 알던 층을 carrier_scan 서비스가 그대로 써야 한다.
            self._armed = False
            self._found = False
        self._orchestrator_state = new_state

    def _detect_tick(self):
        if not self._patrol_scan:
            return
        if self._orchestrator_state != "patrol":
            return

        shelf = self._current_shelf()
        if shelf is None:
            # 구역 밖이다 — 다음에 구역에 다시 들어올 때 재관측하도록 초기화.
            self._armed = False
            self._found = False
            return

        if self._found:
            # 이미 이번 체류에서 감지해서 발행했다. task_manager 가 HOLD 로
            # 넘어갈 때까지 더 폴링하지 않는다.
            return

        try:
            if not self._armed:
                self._observe(shelf)
                self._armed = True
            r = self.sim.call("scan_qr", timeout_s=5.0, expected_id=None, n_frames=1,
                              robot_id=self.robot_id)
        except SimClientError as e:
            self.get_logger().warn(f"carrier_detected 폴링 실패: {e}")
            return

        if r.get("ok"):
            # ★ 진행방향 정렬 게이트 — declare_parameter 주석 참고.
            #   qr_pose_base_link 는 base_link 기준이고 +x 가 로봇 정면이다.
            #   매거진은 옆(±y)에 있으므로, x 성분이 곧 "아직 얼마나 덜 왔나" 다.
            along = None
            pos = ((r.get("qr_pose_base_link") or {}).get("position")) or []
            if len(pos) >= 1:
                along = float(pos[0])
            if self._align_tol > 0.0 and along is not None \
                    and abs(along) > self._align_tol:
                self.get_logger().info(
                    f"QR 보임({r.get('decoded')}) — 아직 {along:+.2f} m 덜 왔다 "
                    f"(허용 ±{self._align_tol:.2f} m). 계속 간다",
                    throttle_duration_sec=2.0)
                return
            self.get_logger().info(
                f"QR 감지: {r.get('decoded')}"
                + (f" (진행방향 오프셋 {along:+.2f} m)" if along is not None else ""))
            self._found = True
            self._detected_pub.publish(Bool(data=True))
            return

        # ★ 실패 이유를 반드시 남긴다. scan_qr 는 왜 실패했는지 reason 으로
        #   돌려주는데(sim_backend.scan_qr -> aggregate_qr_poses.reason),
        #   여기서 그냥 버리면 "QR 이 화면에 보이는데 왜 인식이 안 되지" 를
        #   추적할 단서가 하나도 없다. 실제로 그래서 한참 헤맸다.
        #
        #   이유마다 손볼 곳이 다르다:
        #     "QR 이 안 보인다"            관측 자세/정차 위치 — 화면에 안 들어왔다
        #     "decode 실패"                들어왔지만 못 읽는다 — 거리·해상도·흐림
        #     "깊이 영상이 없다"           카메라 깊이 스트림 문제
        #     "탐색창 안 유효 깊이 점이 N" 깊이는 오는데 QR 자리에 값이 없다
        #     "벽면 법선이 N도 기울어"     자세가 비스듬하다
        #
        #   폴링이 5 Hz 라 throttle 로 묶는다 — 같은 이유가 초당 다섯 번
        #   찍히면 로그가 다른 것을 덮는다.
        reason = r.get("reason") or "(이유 없음)"
        self.get_logger().warning(
            f"QR 인식 실패 [{shelf.get('shelf_id')}]: {reason}",
            throttle_duration_sec=3.0)

    # ── ③④ /perception/carrier_scan ─────────────────────────────────────
    def _on_carrier_scan(self, request, response):
        try:
            # HOLD 로 세운 자리는 patrol 이 지난 선반 앞이라 _current_shelf 가
            # 그 선반을 잡는다(amcl_pose 로 매번 다시 물어본다). 없으면
            # pose_name 폴백 — 이전 판과 같은 동작.
            self._observe(self._current_shelf(), timeout_s=60.0)
            r = self.sim.call("scan_qr", expected_id=None, n_frames=3, robot_id=self.robot_id)
        except SimClientError as e:
            response.found = False
            self.get_logger().warn(f"carrier_scan: {e}")
            return response

        if not r["ok"]:
            response.found = False
            self.get_logger().warn(f"carrier_scan: {r.get('reason', '검출 실패')}")
            return response

        response.found = True
        response.header.stamp = self.get_clock().now().to_msg()
        response.header.frame_id = self.get_parameter("output_frame").value
        response.payload = r["decoded"]    # gen_carrier_assets.py 의 숫자 ID ("1"/"2")
                                            # 를 그대로 실었다 — 실물 페이로드
                                            # (C3.MAG.A17.9) 파싱은 아직 안 붙었다
        p = r["qr_pose_base_link"]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = p["position"]
        # CarrierScan.qr_pose 규약: +Z = 라벨 법선(바깥). qr_pose.py 내부 규약은
        # z = 안쪽(-법선) 이라 부호가 반대다. x(가로)는 그대로 두고 로컬 x축
        # 기준 180도 회전(new_x=old_x, new_y=-old_y, new_z=-old_z)을 곱하면
        # z 가 뒤집히면서 오른손 좌표계가 유지된다.
        # q_new = q_old ⊗ (w=0,x=1,y=0,z=0) = (-x, w, z, -y)  (직접 전개해 확인)
        w, x, y, z = p["quat_wxyz"]
        pose.orientation.w = -x
        pose.orientation.x = w
        pose.orientation.y = z
        pose.orientation.z = -y
        response.qr_pose = pose

        self.get_logger().info(
            f"CarrierScan 응답: payload={response.payload} n_used={r['n_used']}/{r['n_total']}")
        return response


def main():
    rclpy.init()
    node = CarrierCodeReader()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
