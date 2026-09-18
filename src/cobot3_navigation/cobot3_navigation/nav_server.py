"""
nav_server — /navigation/navigate_to (cobot3_interfaces/NavigateTo) 를
Nav2 의 표준 액션 navigate_to_pose (nav2_msgs/action/NavigateToPose) 로
감싼다. docs/08_ROS2_NODE_Graph.html 확정안의 nav_server 역할 그대로다.

    ros2 run cobot3_navigation nav_server

씬(isaacpjt/worlds/simple_factory_layout.usda)의 nova_carter1 은 Isaac 쪽에서
isaac:namespace = "robot1" 로 떠 있고(cobot3_navigation/multi_navigation.launch.py
가 그 네임스페이스로 Nav2 를 붙인다), 그래서 기본 타겟은
/robot1/navigate_to_pose 다. 다른 로봇에 붙이려면
    ros2 run cobot3_navigation nav_server --ros-args -p robot_namespace:=robot2

취소 전달: task_manager 가 NavigateTo goal 을 취소하면 이 노드가 Nav2 goal 을
그대로 취소한다. Nav2 가 cmd_vel 발행을 멈추면 로봇이 선다 — "정지" 라는
개념은 이 노드도 모른다(NavigateTo.action 주석 그대로), 취소를 전달만 한다.

속도 프로파일: 목표가 선반 구역(SHELF_ZONE, cobot3_perception/
carrier_code_reader.py 의 SHELF_ZONE 과 반드시 같은 값 — QR 스캔 대상
구간과 저속 구간이 어긋나면 스캔 중에도 빠르게 달리는 사고가 난다) 안이면
저속(SLOW), 밖이면 고속(FAST)으로 controller_server(FollowPath)·
velocity_smoother 파라미터를 goal 마다 눌러준다. 저속값은 흔들림 없이
QR 을 읽으려고 낮춘 값(robot1_nav2_params.yaml 원래 기본값이었다), 고속값은
그 낮춘 속도로는 선반 밖 이동까지 너무 느려서(실측: 8m 이동에 93초, recovery
5회) 따로 되살린 것이다.

도착 후 미세정렬: Nav2 가 SUCCEEDED 를 보고해도 실제 최종 자세가 크게
어긋나 있을 수 있다 — footprint 의 제자리회전 스윕 반경(0.66m)이 선반 앞
실제 여유(0.26~0.48m)보다 넓어서, Nav2 자신도 마지막 정렬을 힘들어한다
(실측: SUCCEEDED 보고 직후 AMCL 로 재보니 yaw 가 36도나 어긋나 있었다).
QR 관측(observe_pose 의 고정 관절 자세)은 이 오차를 못 버티므로, SUCCEEDED
직후 AMCL pose 를 다시 확인해서 yaw 오차가 크면 cmd_vel 로 아주 느리게
제자리에서 마지막 정렬만 직접 보정한다. AMCL 의 amcl_pose 는 로봇이 안
움직이면 새 메시지를 안 내므로(update_min_d/a 문턱), 구독은 RELIABLE +
TRANSIENT_LOCAL 로 해야 latched 된 최신값을 바로 받는다(BEST_EFFORT 로 했다가
정지 상태에서 pose 를 못 받는 걸 실측으로 확인했다).
"""

import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import (
    Parameter as ParameterMsg, ParameterType, ParameterValue,
)
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from cobot3_interfaces.action import NavigateTo

ALIGN_YAW_TOL_RAD = math.radians(5.0)   # QR 관측 자세가 버틸 수 있는 최종 yaw 오차
ALIGN_MAX_W = 0.15                      # rad/s — 선반 근처라 아주 느리게만 돈다
ALIGN_TIMEOUT_S = 15.0


def _yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

# cobot3_perception/carrier_code_reader.py 의 SHELF_ZONE_X/Y 와 반드시 같아야
# 한다 — 스캔 구역과 저속 구역이 같은 자리를 가리켜야 한다.
SHELF_ZONE_X = (-6.85, -4.65)
SHELF_ZONE_Y = (1.10, 1.80)

# (vel_x m/s, vel_theta rad/s, acc_x m/s^2, acc_theta rad/s^2)
SLOW = (0.25, 0.3, 0.4, 0.8)   # 선반 구역 — QR 스캔이 흔들리면 안 된다
FAST = (0.8, 0.7, 2.5, 3.5)    # 그 외 — robot1_nav2_params.yaml 원래 기본값


class NavServer(Node):

    def __init__(self):
        super().__init__("nav_server")
        # ── NavigateTo.action ★ 블록 ──
        self.declare_parameter("robot_namespace", "robot1")
        self.declare_parameter("xy_goal_tolerance_m", 0.25)
        self.declare_parameter("yaw_goal_tolerance_rad", 0.15)

        ns = self.get_parameter("robot_namespace").value
        self._ns = ns
        self._nav2 = ActionClient(self, NavigateToPose, f"/{ns}/navigate_to_pose")
        self._controller_params = self.create_client(
            SetParameters, f"/{ns}/controller_server/set_parameters")
        self._smoother_params = self.create_client(
            SetParameters, f"/{ns}/velocity_smoother/set_parameters")
        self._speed_profile = None   # "slow" | "fast" | None(아직 안 눌러봄)

        amcl_qos = QoSProfile(depth=1)
        amcl_qos.reliability = ReliabilityPolicy.RELIABLE
        amcl_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._amcl_pose = None
        self.create_subscription(
            PoseWithCovarianceStamped, f"/{ns}/amcl_pose", self._on_amcl_pose, amcl_qos)
        self._cmd_vel_pub = self.create_publisher(Twist, f"/{ns}/cmd_vel", 10)

        cb = ReentrantCallbackGroup()
        self._server = ActionServer(
            self, NavigateTo, "/navigation/navigate_to",
            execute_callback=self._execute,
            goal_callback=self._on_goal, cancel_callback=self._on_cancel,
            callback_group=cb)

        self._push_goal_tolerance(ns)
        self.get_logger().info(
            f"nav_server ready — /navigation/navigate_to -> /{ns}/navigate_to_pose")

    def _on_amcl_pose(self, msg):
        self._amcl_pose = msg.pose.pose

    def _fine_align(self, goal_pose):
        """Nav2 SUCCEEDED 직후 실제 yaw 오차를 재확인하고, 크면 cmd_vel 로
        느리게 제자리 보정한다. 모듈 독스트링 "도착 후 미세정렬" 참고."""
        goal_yaw = _yaw_of(goal_pose.orientation)
        deadline = time.monotonic() + ALIGN_TIMEOUT_S
        corrected = False
        while time.monotonic() < deadline:
            if self._amcl_pose is None:
                time.sleep(0.1)
                continue
            err = _wrap(goal_yaw - _yaw_of(self._amcl_pose.orientation))
            if abs(err) <= ALIGN_YAW_TOL_RAD:
                break
            corrected = True
            tw = Twist()
            tw.angular.z = max(-ALIGN_MAX_W, min(ALIGN_MAX_W, 1.0 * err))
            self._cmd_vel_pub.publish(tw)
            time.sleep(0.1)
        else:
            self.get_logger().warning(
                f"미세정렬 타임아웃({ALIGN_TIMEOUT_S:.0f}s) — 남은 yaw 오차를 그대로 두고 진행한다")
        self._cmd_vel_pub.publish(Twist())
        if corrected:
            self.get_logger().info("도착 후 yaw 미세정렬 완료")

    def _in_shelf_zone(self, x, y):
        return SHELF_ZONE_X[0] <= x <= SHELF_ZONE_X[1] and SHELF_ZONE_Y[0] <= y <= SHELF_ZONE_Y[1]

    def _apply_speed_profile(self, x, y):
        """goal 위치를 보고 선반 구역이면 SLOW, 아니면 FAST 를 누른다. 이전과
        같은 프로파일이면 다시 안 누른다(매 goal 마다 SetParameters 왕복할
        필요 없다)."""
        want = "slow" if self._in_shelf_zone(x, y) else "fast"
        if want == self._speed_profile:
            return
        vel_x, vel_theta, acc, acc_theta = SLOW if want == "slow" else FAST

        if self._controller_params.wait_for_service(timeout_sec=2.0):
            self._controller_params.call_async(SetParameters.Request(parameters=[
                ParameterMsg(name="FollowPath.max_vel_x",
                            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=vel_x)),
                ParameterMsg(name="FollowPath.max_speed_xy",
                            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=vel_x)),
                ParameterMsg(name="FollowPath.max_vel_theta",
                            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=vel_theta)),
                ParameterMsg(name="FollowPath.acc_lim_x",
                            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=acc)),
                ParameterMsg(name="FollowPath.decel_lim_x",
                            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=-acc)),
                ParameterMsg(name="FollowPath.acc_lim_theta",
                            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=acc_theta)),
                ParameterMsg(name="FollowPath.decel_lim_theta",
                            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=-acc_theta)),
            ]))
        else:
            self.get_logger().warning(f"/{self._ns}/controller_server 가 없다 — 속도 프로파일 못 바꿈")

        if self._smoother_params.wait_for_service(timeout_sec=2.0):
            # velocity_smoother 는 [x, y, theta] 배열 파라미터라 x/theta 만 갈아
            # 끼우고 y(0.0, 차동구동이라 항상 0)는 그대로 둔다.
            self._smoother_params.call_async(SetParameters.Request(parameters=[
                ParameterMsg(name="max_velocity", value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                    double_array_value=[vel_x, 0.0, vel_theta])),
                ParameterMsg(name="min_velocity", value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                    double_array_value=[-vel_x, 0.0, -vel_theta])),
                ParameterMsg(name="max_accel", value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                    double_array_value=[acc, 0.0, acc_theta])),
                ParameterMsg(name="max_decel", value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                    double_array_value=[-acc, 0.0, -acc_theta])),
            ]))
        else:
            self.get_logger().warning(f"/{self._ns}/velocity_smoother 가 없다 — 속도 프로파일 못 바꿈")

        self._speed_profile = want
        self.get_logger().info(f"속도 프로파일 -> {want} (vel_x={vel_x}, vel_theta={vel_theta})")

    def _push_goal_tolerance(self, ns):
        """xy/yaw_goal_tolerance 를 controller_server 의 general_goal_checker 로
        흘려보낸다(robot1_nav2_params.yaml 의 plugin 이름과 일치). Nav2 가 아직
        안 떠 있으면 그냥 넘어간다 — params yaml 기본값(0.25/0.25)으로 돈다."""
        xy = float(self.get_parameter("xy_goal_tolerance_m").value)
        yaw = float(self.get_parameter("yaw_goal_tolerance_rad").value)
        client = self.create_client(SetParameters, f"/{ns}/controller_server/set_parameters")
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning(
                f"/{ns}/controller_server 가 아직 안 떠 있다 — goal tolerance 는 "
                "params yaml 기본값을 그대로 쓴다")
            return
        req = SetParameters.Request(parameters=[
            ParameterMsg(name="general_goal_checker.xy_goal_tolerance",
                        value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=xy)),
            ParameterMsg(name="general_goal_checker.yaw_goal_tolerance",
                        value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=yaw)),
        ])
        client.call_async(req)

    def _on_goal(self, goal_request):
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        result = NavigateTo.Result()
        feedback = NavigateTo.Feedback()

        if not self._nav2.wait_for_server(timeout_sec=30.0):
            # 5초는 너무 짧다 — task_manager 가 Nav2 보다 먼저(혹은 거의 동시에)
            # 뜨면 bt_navigator 가 아직 액션 서버를 안 올린 상태라 첫 patrol
            # goal 이 여기서 걸린다. task_manager 는 한 번 실패하면 절대
            # 재시도하지 않으므로(확정 설계), 순수 시작 타이밍 때문에 세션
            # 전체가 영구히 멈추는 걸 막으려고 넉넉하게 잡았다. "진짜 실패면
            # 재시도 안 한다"는 정책 자체는 안 건드린다 — 서버가 아예 없을 때만
            # 더 기다릴 뿐이다.
            self.get_logger().error("Nav2 navigate_to_pose 서버가 없다")
            result.success = False
            result.fail_reason = NavigateTo.Result.PLAN_FAIL
            goal_handle.abort()
            return result

        target = goal_handle.request.pose.pose.position
        self._apply_speed_profile(target.x, target.y)

        nav2_goal = NavigateToPose.Goal(pose=goal_handle.request.pose)
        send_fut = self._nav2.send_goal_async(
            nav2_goal,
            feedback_callback=lambda fb: self._relay_feedback(fb, goal_handle, feedback))
        nav2_handle = self._block(send_fut, 10.0)

        if nav2_handle is None or not nav2_handle.accepted:
            self.get_logger().warning("Nav2 goal 거부됨")
            result.success = False
            result.fail_reason = NavigateTo.Result.PLAN_FAIL
            goal_handle.abort()
            return result

        result_fut = nav2_handle.get_result_async()
        while not result_fut.done():
            if goal_handle.is_cancel_requested:
                self._block(nav2_handle.cancel_goal_async(), 5.0)
            time.sleep(0.05)

        status = result_fut.result().status

        if status == GoalStatus.STATUS_CANCELED:
            result.success = False
            result.fail_reason = NavigateTo.Result.CANCELED
            goal_handle.canceled()
            return result

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._fine_align(goal_handle.request.pose.pose)
            result.success = True
            result.fail_reason = NavigateTo.Result.NONE
            goal_handle.succeed()
            return result

        # ABORTED 등 — 지역 회피 실패 / 복구 소진으로 본다.
        result.success = False
        result.fail_reason = NavigateTo.Result.BLOCKED
        goal_handle.abort()
        return result

    @staticmethod
    def _relay_feedback(nav2_feedback_msg, goal_handle, feedback):
        feedback.distance_remaining_m = nav2_feedback_msg.feedback.distance_remaining
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _block(future, timeout_s):
        done = threading.Event()
        future.add_done_callback(lambda f: done.set())
        if not done.wait(timeout=timeout_s):
            return None
        return future.result()


def main():
    rclpy.init()
    node = NavServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
