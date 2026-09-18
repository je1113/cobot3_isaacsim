"""
carrier_code_reader — QR 판독 노드. CarrierScan 서비스를 서빙한다.

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
/perception/carrier_detected (std_msgs/Bool, 주행 중 감지 신호)는 3-1
주행 스캔이 이 노드에 아직 안 물려 있어 미구현이다.

CarrierScan.msg 리팩터 이후: raw_payload · symbology · frame_votes ·
decode_latency_ms · range_m · reader_id · payload_valid 필드가 메시지에서
빠졌다(튜닝·분석 지표라 노드 로그로 이동 — n_used/n_total 은 아래 로그에
남긴다). decode_hz/roi_fraction/vote_window/vote_required/publish_debug
파라미터는 ★ 알려진 갭: 선언만 하고 아직 실제 디코드 루프(다중 프레임
다수결)에 연결하지 않았다 — scan_qr 이 이미 자체적으로 n_frames 평균을
낸다(qr_pose.aggregate_qr_poses). output_frame 은 실제로 쓴다: qr_pose
값이 base_link 기준 상대좌표라 그게 맞는 기본값이다("wrist_camera" 라고
적혀 있던 이전 버전은 명칭과 실제 좌표계가 어긋나 있었다).

★ a2caf63 리팩터(carrier_id · carrier_kind → payload) 반영: carrier_id ·
carrier_kind 필드와 MAGAZINE 상수가 메시지에서 사라졌다. 이 씬의 QR 은
숫자("1"/"2")만 담고 있어 <kind>-<serial> 파싱은 아직 필요 없으므로,
그 숫자 문자열을 payload 에 그대로 싣는다 — 실물 페이로드
(carriers.yaml 의 "C3.MAG.A17.9" 형식) 파싱이 붙으면 여기서 그 문자열을
만들어야 한다. qr_pose 도 759212e 이후 PoseStamped 가 아니라
geometry_msgs/Pose 다 — 좌표계는 header.frame_id 하나로 말한다.
"""

import sys
from pathlib import Path

import rclpy
from rclpy.node import Node

from cobot3_interfaces.srv import CarrierScan
from geometry_msgs.msg import Pose

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
        self.get_logger().info("carrier_code_reader ready")

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
