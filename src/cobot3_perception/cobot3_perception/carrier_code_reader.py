"""
carrier_code_reader — QR 판독 노드. docs/08_ROS2_NODE_Graph.html 확정안
(§01/§03)의 carrier_code_reader 그대로 — /perception/carrier_detected 발행과
/perception/carrier_scan 서비스를 같은 노드 하나가 맡는다.

    ros2 run cobot3_perception carrier_code_reader

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
patrol 이 실제로 선반 앞(SHELF_ZONE, /robot1/amcl_pose 로 판정)을 지날
때만 잡는다 — 그 전까지 팔을 건드리면 개활지 회전 구간과 간섭할 수 있고,
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

★ 알려진 한계: observe_pose 의 관절값은 taught_poses.yaml 의 특정 베이스
위치(x=-6.498, y=1.45) 기준으로 티칭된 것 하나뿐이다. SHELF_ZONE 안 어디서든
같은 관절값을 쓰므로, 구역 가장자리(x=-7.05/-4.55 근처)에서 선반과 팔이
간섭하지 않는지는 시뮬에서 직접 확인이 필요하다.
"""

import sys
from pathlib import Path

import rclpy
from rclpy.node import Node

from cobot3_interfaces.srv import CarrierScan
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from std_msgs.msg import Bool, String

# taught_poses.yaml 의 shelf_1_top_close_centered 가 전제하는 베이스 위치
# (x=-6.498, y=1.45) 주변. x 는 frames.yaml observation_poses.shelf_1_top 의
# scan_x_range([-6.75,-4.75])에 여유를 조금 더 줬다. y 는 선반 앞 정차선
# (1.45) 근처로 좁혀서, 개활지 회전 구간(y=0.30, task_manager.py PATROL_ROUTE)
# 은 확실히 빠지게 했다.
SHELF_ZONE_X = (-6.85, -4.65)
SHELF_ZONE_Y = (1.10, 1.80)


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
        self.sim = SimClient()
        self._srv = self.create_service(CarrierScan, "/perception/carrier_scan", self._on_carrier_scan)

        # ── ① /perception/carrier_detected — patrol 중 선반 구역 실시간 감시 ──
        self._detected_pub = self.create_publisher(Bool, "/perception/carrier_detected", 10)
        self.create_subscription(String, "/orchestrator/state", self._on_orchestrator_state, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "/robot1/amcl_pose", self._on_amcl_pose, 10)
        self._orchestrator_state = None
        self._base_pose = None
        self._armed = False    # 이번 구역 체류 동안 observe_pose 를 이미 호출했는지
        self._found = False    # 이번 구역 체류 동안 이미 감지해서 발행했는지
        hz = float(self.get_parameter("decode_hz").value)
        self.create_timer(1.0 / hz, self._detect_tick)

        self.get_logger().info("carrier_code_reader ready")

    # ── carrier_detected ────────────────────────────────────────────────
    def _on_amcl_pose(self, msg):
        self._base_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _in_shelf_zone(self):
        if self._base_pose is None:
            return False
        x, y = self._base_pose
        return SHELF_ZONE_X[0] <= x <= SHELF_ZONE_X[1] and SHELF_ZONE_Y[0] <= y <= SHELF_ZONE_Y[1]

    def _on_orchestrator_state(self, msg):
        new_state = None
        for part in msg.data.split("|"):
            part = part.strip()
            if part.startswith("state="):
                new_state = part[len("state="):]
                break
        if new_state != self._orchestrator_state and new_state != "patrol":
            # patrol 을 벗어나면(HOLD/SCAN/PICK/NAV/PLACE) 팔이 다른 자세로
            # 움직일 것이므로, 다음에 patrol 로 돌아오면 다시 observe_pose 부터.
            self._armed = False
            self._found = False
        self._orchestrator_state = new_state

    def _detect_tick(self):
        if self._orchestrator_state != "patrol":
            return

        if not self._in_shelf_zone():
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
                self.sim.call("observe_pose", timeout_s=15.0)
                self._armed = True
            r = self.sim.call("scan_qr", timeout_s=5.0, expected_id=None, n_frames=1)
        except SimClientError as e:
            self.get_logger().warn(f"carrier_detected 폴링 실패: {e}")
            return

        if r.get("ok"):
            self.get_logger().info(f"QR 감지: {r.get('decoded')}")
            self._found = True
            self._detected_pub.publish(Bool(data=True))

    # ── ③④ /perception/carrier_scan ─────────────────────────────────────
    def _on_carrier_scan(self, request, response):
        try:
            self.sim.call("observe_pose")
            r = self.sim.call("scan_qr", expected_id=None, n_frames=3)
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
