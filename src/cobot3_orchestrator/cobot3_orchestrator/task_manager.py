"""
task_manager — orchestrator. docs/07_ROS2_Node_Graph.html §4 상태머신 중
SCAN·PICK·TRANSIT·PLACE 구간을 구현했다 (PATROL·RETURN 은 다음 범위).

    ros2 run cobot3_orchestrator task_manager

트리거: /orchestrator/run_demo_pick (std_srvs/Trigger) — cobot3_interfaces
에 "미션 시작" 인터페이스가 아직 없어서(RegisterTask.srv 가 보류 상태)
로컬 트리거를 하나 뒀다. RegisterTask.srv 필드가 정해지면 이걸로 옮긴다.

★ 인터페이스 리팩터(a2caf63 "패키지 역할 정의에 맞춰 5종 재정의") 이후
흐름:
  1. teleport_base(shelf_1_s3)  — 이동. nav_server_stub 을 지웠다(아무
     의미 있는 주행을 안 하는 순간이동일 뿐이었다 — 어차피 붙들고 있으면
     한 스텝만에 흡착이 깨지는 걸 GUI 로 실측했다). pick_place_server 와
     같은 방식으로 이 노드가 SimClient 로 sim_backend 를 직접 부른다.
  2. /perception/scan_now 트리거 -> CarrierScan 수신 대기
  3. carriers.yaml 로 payload 를 variant 로 해석 (파지점 계산은 안 한다 —
     a2caf63 이후 pick_place_server 책임. qr_pose 는 prior 로 그대로 넘긴다)
  4. PickCarrier(variant, qr_pose) 전송
  5. teleport_base(pkg_loader)  — 이송(TRANSIT)
  6. PlaceCarrier(variant) 전송

NavigateTo.action/nav_server 는 더 안 쓴다 — "이동"을 리스트에서 빼자는
게 아니라, 진짜 주행(Nav2/SLAM, cobot3_navigation 패키지, 팀원이 다른
브랜치에서 작업 중)이 붙기 전까지는 이 노드가 지금의 유일하게 실재하는
이동 수단(teleport_base)을 있는 그대로 부르는 게 "가짜 주행 액션 서버"
를 하나 더 두는 것보다 솔직하다. 진짜 nav_server 가 생기면 이 두 호출을
NavigateTo 액션 전송으로 되돌린다.

VerifyCarrier 서비스는 인터페이스에서 통째로 삭제됐다 — "잡아도/놓아도
되는 캐리어인가" 를 pick_place_server 에 되묻는 인터록이 없어졌다. 이
데모는 scan 으로 직접 확인한 carrier_id 를 그 즉시 집으므로(따로 명령을
내리고 나중에 확인하는 구조가 아니다) 별도의 로컬 검증 없이도 안전하다.

알려진 갭 · 단순화 (보고용, 코드 안에도 표시):
  - 이 씬의 QR 은 숫자 하나("1"/"2")만 담아서 "몇 번째 개체"까지는 구분 못
    한다. carriers.yaml 에서 그 variant 의 첫 항목을 쓴다.
  - flange_pose(파지점) · slot_pose(배치점) 계산은 이제 이 노드가 안 한다
    — a2caf63 이후 pick_place_server 가 variant + qr_pose(prior)로 직접
    계산한다(grasp.yaml/place.yaml). PLACE_SLOT_POSE_BASE_LINK 하드코딩도
    같이 옮겨서 cobot3_bringup/config/place.yaml 이 됐다. NAV_WAYPOINTS
    ["pkg_loader"] 가 바뀌면 place.yaml 값도 다시 맞춰야 한다 — teleport_base
    가 실제 도착 pose 를 돌려주지만(base_x/base_y), place.yaml 계산에는
    아직 안 먹인다(계산이 아니라 가정).
  - 흡착 중 이송(teleport_base)이 흡착을 깨는 시뮬 한계는 sim_backend.py
    의 _carry_gripped_object_through_teleport 로 흡수한다 — 실측(GUI,
    점진적 이동 142 스텝 중 24 스텝째 이탈)으로 "거리와 무관하게 깨진다"
    는 팀 기존 결론을 재확인했다. 이 재부착 자체도 아직 불안정하다(다음
    갭).
"""

import threading
import sys
from pathlib import Path

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped

from cobot3_interfaces.action import PickCarrier, PlaceCarrier
from cobot3_interfaces.msg import CarrierScan

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
FRAMES_YAML = WS_ROOT / "src/cobot3_bringup/config/frames.yaml"
CARRIERS_YAML = WS_ROOT / "src/cobot3_bringup/config/carriers.yaml"

def _add_ros_bridge_to_syspath():
    # pick_place_server.py/carrier_code_reader.py 와 같은 이유 · 같은 탐색
    # 방식 — 그쪽 파일들의 주석 참고.
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

NUMERIC_TO_VARIANT = {"1": "magazine_1_orange", "2": "magazine_2_blue"}

# 이름 -> (x, y, yaw_deg), world 좌표. 값 출처는 삭제된 nav_server_stub.py
# 의 WP_TABLE — isaacpjt/M0609/lula_ik/12_pick_test.py(shelf_1_s3),
# 12_place_test.py(pkg_loader) 검증값 그대로.
NAV_WAYPOINTS = {
    "shelf_1_s3": (-6.5568, 1.45, 0.0),
    "pkg_loader": (4.0, 0.0, 0.0),
}


class TaskManager(Node):
    def __init__(self):
        super().__init__("task_manager")
        self.frames = yaml.safe_load(FRAMES_YAML.read_text(encoding="utf-8"))
        self.carriers = yaml.safe_load(CARRIERS_YAML.read_text(encoding="utf-8"))
        self.sim = SimClient()

        cb = ReentrantCallbackGroup()
        self._latest_scan = None
        self._scan_event = threading.Event()
        self.create_subscription(CarrierScan, "/perception/carrier_scan",
                                 self._on_scan, 5, callback_group=cb)
        self._scan_now = self.create_client(Trigger, "/perception/scan_now", callback_group=cb)
        self._pick = ActionClient(self, PickCarrier, "/manipulation/pick_carrier",
                                  callback_group=cb)
        self._place = ActionClient(self, PlaceCarrier, "/manipulation/place_carrier",
                                   callback_group=cb)

        self.create_service(Trigger, "/orchestrator/run_demo_pick",
                            self._on_run_demo_pick, callback_group=cb)
        self.get_logger().info("task_manager ready — /orchestrator/run_demo_pick 로 트리거")

    def _on_scan(self, msg):
        self._latest_scan = msg
        self._scan_event.set()

    def _resolve_carrier(self, numeric_id):
        """QR 숫자 -> (carrier_id, variant). 이 씬의 QR 은 '1'/'2' 뿐이라
        같은 variant 의 여러 개체를 구분 못 한다 — carriers.yaml 의 첫 매치를 쓴다."""
        variant = NUMERIC_TO_VARIANT.get(numeric_id)
        if variant is None:
            return None, None
        for cid, info in self.carriers.items():
            if isinstance(info, dict) and info.get("variant") == variant:
                return cid, variant
        return None, variant

    # ── 데모 미션 ──
    def _on_run_demo_pick(self, request, response):
        t = threading.Thread(target=self._run_demo_pick_impl, args=(response,))
        t.start()
        t.join()   # Trigger 는 결과를 바로 돌려줘야 하니 동기적으로 기다린다
        return response

    def _run_demo_pick_impl(self, response):
        try:
            self._step1_navigate("shelf_1_s3")
            self._step2_scan()
            carrier_id, variant, qr_pose = self._step3_identify_carrier()
            pick_result = self._step4_pick(variant, qr_pose)
            pick_msg = (f"carrier={carrier_id} variant={variant} "
                       f"pick_success={pick_result.success} pick_fail_reason={pick_result.fail_reason}")
            if not pick_result.success:
                response.success = False
                response.message = pick_msg
                return

            self._step5_navigate("pkg_loader")
            place_result = self._step6_place(variant)
            response.success = bool(place_result.success)
            response.message = (f"{pick_msg} | place_success={place_result.success} "
                                f"place_fail_reason={place_result.fail_reason}")
        except Exception as e:
            self.get_logger().error(f"데모 미션 실패: {e}")
            response.success = False
            response.message = str(e)

    def _step1_navigate(self, name):
        self.get_logger().info(f"[1] teleport_base {name}")
        x, y, yaw_deg = NAV_WAYPOINTS[name]
        try:
            r = self.sim.call("teleport_base", x=x, y=y, yaw_deg=yaw_deg)
        except SimClientError as e:
            raise RuntimeError(f"teleport_base 실패: {e}")
        self.get_logger().info(
            f"  -> ({r['base_x']:.3f},{r['base_y']:.3f})"
            f"  dropped_at_step={r.get('dropped_at_step')}  carried_ok={r.get('carried_ok')}")

    def _step2_scan(self):
        self.get_logger().info("[2] 관측 자세 이동 + 재촬영 트리거")
        self._scan_event.clear()
        if not self._scan_now.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("carrier_code_reader 없음")
        fut = self._scan_now.call_async(Trigger.Request())
        r = self._wait(fut)
        if not r.success:
            raise RuntimeError(f"scan_now 실패: {r.message}")
        if not self._scan_event.wait(timeout=5.0):
            raise RuntimeError("CarrierScan 을 못 받았다")

    def _step3_identify_carrier(self):
        """QR payload 를 variant 로 해석한다. 파지점(flange_pose) 계산은
        더 이상 여기서 안 한다 — a2caf63 이후 pick_place_server 가
        variant + qr_pose(prior)로 직접 한다. qr_pose 는 CarrierScan 의
        것을 그대로 PoseStamped 로 감싸서 넘긴다(PickCarrier.action 주석
        "CarrierScan 의 것을 그대로 넘긴다")."""
        scan = self._latest_scan
        carrier_id, variant = self._resolve_carrier(scan.payload)
        if variant is None:
            raise RuntimeError(f"모르는 payload: {scan.payload}")
        self.get_logger().info(f"[3] carrier_id={carrier_id} variant={variant}")

        qr_pose = PoseStamped()
        qr_pose.header = scan.header
        qr_pose.pose = scan.qr_pose
        return carrier_id, variant, qr_pose

    def _step4_pick(self, variant, qr_pose):
        self.get_logger().info(f"[4] PickCarrier variant={variant}")
        if not self._pick.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("pick_place_server 없음")
        goal = PickCarrier.Goal(variant=variant, qr_pose=qr_pose)
        fut = self._pick.send_goal_async(
            goal, feedback_callback=lambda fb: self.get_logger().info(
                f"  PICK phase={fb.feedback.phase}"))
        gh = self._wait(fut)
        if not gh.accepted:
            raise RuntimeError("PickCarrier 거부됨")
        return self._wait(gh.get_result_async()).result

    def _step5_navigate(self, name):
        self.get_logger().info(f"[5] teleport_base {name} (이송)")
        self._step1_navigate(name)   # teleport_base 호출부는 동일 로직 재사용

    def _step6_place(self, variant):
        self.get_logger().info(f"[6] PlaceCarrier variant={variant}")
        if not self._place.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("pick_place_server 없음(place_carrier)")
        goal = PlaceCarrier.Goal(variant=variant)
        fut = self._place.send_goal_async(
            goal, feedback_callback=lambda fb: self.get_logger().info(
                f"  PLACE phase={fb.feedback.phase}"))
        gh = self._wait(fut)
        if not gh.accepted:
            raise RuntimeError("PlaceCarrier 거부됨")
        return self._wait(gh.get_result_async()).result

    @staticmethod
    def _wait(future, timeout_s=90.0):
        done = threading.Event()
        future.add_done_callback(lambda f: done.set())
        if not done.wait(timeout=timeout_s):
            raise TimeoutError("future 대기 시간 초과")
        return future.result()


def main():
    rclpy.init()
    node = TaskManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
