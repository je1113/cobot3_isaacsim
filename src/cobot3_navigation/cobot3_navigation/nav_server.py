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

선반 구역은 cmd_vel 로 직접 몬다: 목표가 선반 구역(SHELF_ZONE, cobot3_perception/
carrier_code_reader.py 의 SHELF_ZONE 과 반드시 같은 값) 안이면 Nav2 에
아예 안 맡기고, 이 노드가 amcl_pose 를 보며 직접 cmd_vel 로 느리게 몬다.
Nav2(DWB) 에 맡겼을 때 계속 BLOCKED/ABORTED 로 막혔던 이유가 RotateToGoal/
BaseObstacle 크리틱이 이 구간의 footprint 제자리회전 스윕(0.66m)을 선반
여유(0.26~0.48m)보다 넓다고 판단해서였다 — 근데 이 구간은 두 티칭 위치
사이 직선이고 항상 yaw=0 만 유지하면 되니 애초에 회전이 필요 없다. DWB 는
그걸 몰라서 매번 헤맸을 뿐이라, 아예 우회하는 게 맞다. 선반 밖 목표는
그대로 Nav2(FAST 프로파일)에 맡긴다 — 이 구간은 회전이 필요해도 여유가
있어서 문제없다.

후진: 선반 구역 목표가 로봇 진행 방향 뒤에 있으면(예: 반대편 티칭 위치로
돌아가는 길) 제자리로 돌지 않고 그냥 후진한다 — yaw=0 을 유지한 채로.

도착 후 미세정렬: 선반 구역이든 Nav2 위임이든, 도착 직후 실제 yaw 오차를
AMCL 로 다시 확인해서 크면 cmd_vel 로 아주 느리게 제자리에서 마지막
정렬만 보정한다(QR 관측의 고정 관절 자세는 오차를 못 버틴다). amcl_pose
는 로봇이 안 움직이면 새 메시지를 안 내므로(update_min_d/a 문턱), 구독은
RELIABLE + TRANSIENT_LOCAL 로 해야 latched 된 최신값을 바로 받는다
(BEST_EFFORT 로 했다가 정지 상태에서 pose 를 못 받는 걸 실측으로 확인했다).
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

SHELF_DRIVE_MAX_V = 0.12    # m/s — 선반 구역 cmd_vel 직접 주행 속도. 흔들리면
                             # QR 디코드가 깨지므로 아주 느리게 잡았다.
SHELF_DRIVE_POS_TOL = 0.08  # m
SHELF_DRIVE_TIMEOUT_S = 90.0

# 하드 안전장치 — 조향 로직에 버그가 있든 없든, y 가 이 선을 넘으면 무조건
# 전진을 막는다. 실측 충돌(정렬 오차로 궤적이 +y 로 휘어져 선반에 닿음)
# 이후 추가한 방어선이다. SHELF_ZONE_Y 상한(1.80)보다 안쪽으로 잡았다.
SHELF_SAFE_Y_MAX = 1.70


def _yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

# cobot3_perception/carrier_code_reader.py 의 SHELF_ZONE_X/Y 와 반드시 같아야
# 한다 — 스캔 구역과 저속 구역이 같은 자리를 가리켜야 한다.
SHELF_ZONE_X = (-6.85, -4.65)
SHELF_ZONE_Y = (1.10, 1.80)

# (vel_x m/s, vel_theta rad/s, acc_x m/s^2, acc_theta rad/s^2)
# 선반 구역은 이제 Nav2 를 안 쓰므로(cmd_vel 직접 주행), FAST 하나만 쓴다 —
# 원래 기본값(0.8/0.7/2.5/3.5)의 3배.
FAST = (2.4, 2.1, 7.5, 10.5)


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

    def _drive_shelf_segment(self, goal_handle, result, feedback):
        """선반 구역 안 목표는 Nav2 에 안 맡기고 amcl_pose 를 보며 직접
        cmd_vel 로 천천히 몬다. 모듈 독스트링 "선반 구역은 cmd_vel 로 직접
        몬다" 참고 — 이 구간은 항상 yaw=0 만 유지하면 되므로 회전이 원래
        필요 없다. 목표가 진행 방향 뒤에 있으면(반대쪽 티칭 위치로 돌아가는
        길) 돌지 않고 그냥 후진한다.

        ★ 주의: Nav2 의 costmap 기반 장애물 회피를 완전히 우회한다 — 이
        구간은 두 타칭 위치 사이 고정된 직선이고 yaw=0 유지 시 선반과의
        간격이 이미 확인된 자리라서 허용했다. 씬에 새 장애물이 생기면
        이 가정이 깨진다.
        """
        goal_pose = goal_handle.request.pose.pose
        gx, gy = goal_pose.position.x, goal_pose.position.y
        gyaw = _yaw_of(goal_pose.orientation)

        deadline = time.monotonic() + SHELF_DRIVE_TIMEOUT_S
        reached = False
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                self._cmd_vel_pub.publish(Twist())
                result.success = False
                result.fail_reason = NavigateTo.Result.CANCELED
                goal_handle.canceled()
                return result
            if self._amcl_pose is None:
                time.sleep(0.1)
                continue

            x = self._amcl_pose.position.x
            y = self._amcl_pose.position.y
            yaw = _yaw_of(self._amcl_pose.orientation)
            dx, dy = gx - x, gy - y
            dist = math.hypot(dx, dy)
            if dist <= SHELF_DRIVE_POS_TOL:
                reached = True
                break

            # ★ 실측 충돌로 확인된 이전 버그: 회전 보정을 목표 지점 방향이
            # 아니라 최종 목표 yaw(gyaw, 고정 0도)로 맞췄었다 — 그러면 실제
            # 진행 방향(로컬 x축)과 목표 지점 방향이 어긋난 채로 전진하면서
            # 궤적이 옆(+y, 선반 쪽)으로 휘어진다. 반드시 목표 "지점" 방향
            # (heading_to_goal)으로 조향해야 한다 — gyaw 정렬은 도착 후
            # _fine_align 이 따로 한다.
            #
            # 후진 판단: heading_to_goal 이 지금 yaw 기준 뒤(±90도 밖)면
            # 후진하면서 "뒤쪽" 기준으로 조향한다.
            heading_to_goal = math.atan2(dy, dx)
            fwd_err = _wrap(heading_to_goal - yaw)
            if abs(fwd_err) <= math.pi / 2:
                direction = 1.0
                steer_err = fwd_err
            else:
                direction = -1.0
                steer_err = _wrap(heading_to_goal + math.pi - yaw)

            # 조향 오차가 크면(45도 넘으면) 속도를 0까지 줄인다 — 정렬 안 된
            # 채로 확신하고 밀고 나가다가 선반에 부딪힌 게 이전 사고였다.
            speed_scale = max(0.0, 1.0 - abs(steer_err) / math.radians(45))
            tw = Twist()
            tw.linear.x = direction * SHELF_DRIVE_MAX_V * speed_scale
            tw.angular.z = max(-ALIGN_MAX_W, min(ALIGN_MAX_W, 1.5 * steer_err))

            # 하드 안전장치 — 조향 로직을 신뢰하지 않는다. y 가 이미 선반
            # 쪽으로 너무 들어가 있으면 전진이든 후진이든 직진 이동 자체를
            # 무조건 막는다(회전 보정만 허용) — local x 전진/후진이 heading
            # 에 따라 어느 쪽이든 y 를 더 키울 수 있어서, 방향을 안 가리고
            # 막는다. SHELF_SAFE_Y_MAX 정의부 주석 참고.
            if y >= SHELF_SAFE_Y_MAX:
                tw.linear.x = 0.0
                self.get_logger().warning(
                    f"y={y:.3f} 가 안전선({SHELF_SAFE_Y_MAX})을 넘어 직진을 막았다")

            self._cmd_vel_pub.publish(tw)

            feedback.distance_remaining_m = float(dist)
            goal_handle.publish_feedback(feedback)
            time.sleep(0.1)

        self._cmd_vel_pub.publish(Twist())

        if not reached:
            self.get_logger().warning(f"선반 구간 주행 타임아웃({SHELF_DRIVE_TIMEOUT_S:.0f}s)")
            result.success = False
            result.fail_reason = NavigateTo.Result.BLOCKED
            goal_handle.abort()
            return result

        self._fine_align(goal_pose)
        result.success = True
        result.fail_reason = NavigateTo.Result.NONE
        goal_handle.succeed()
        return result

    def _ensure_fast_profile(self):
        """선반 구역 밖(Nav2 위임) 목표에만 쓴다 — 선반 구역은 이제 Nav2 를
        아예 안 쓰므로 프로파일 전환이 필요 없다. 한 번 누르면 다시 안
        누른다(매 goal 마다 SetParameters 왕복할 필요 없다)."""
        if self._speed_profile == "fast":
            return
        vel_x, vel_theta, acc, acc_theta = FAST

        if self._controller_params.wait_for_service(timeout_sec=2.0):
            self._controller_params.call_async(SetParameters.Request(parameters=[
                ParameterMsg(name="FollowPath.max_vel_x",
                            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=vel_x)),
                # 후진 허용 — 모듈 독스트링 "후진 허용" 참고. 0.0 으로 두면
                # PATROL_ROUTE 왕복 중 한쪽 방향에서 선반 앞 제자리 회전이
                # 강제된다.
                ParameterMsg(name="FollowPath.min_vel_x",
                            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=-vel_x)),
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

        self._speed_profile = "fast"
        self.get_logger().info(f"속도 프로파일 -> fast (vel_x={vel_x}, vel_theta={vel_theta})")

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

        # 목표만 보면 안 된다 — 두 티칭 위치 중 하나가 구역 밖에 있어서
        # (예: 시작점), "구역 안(끝점) -> 구역 밖(시작점)" 으로 돌아갈 때
        # 출발지가 아직 선반 바로 옆인데 목표만 보고 Nav2(FAST)로 넘기면
        # 출발 직후 같은 회전 문제가 재현된다. 현재 위치나 목표 둘 중
        # 하나라도 구역 안이면 직접 주행으로 처리한다.
        target = goal_handle.request.pose.pose.position
        cur = self._amcl_pose
        cur_in_zone = cur is not None and self._in_shelf_zone(cur.position.x, cur.position.y)
        if cur_in_zone or self._in_shelf_zone(target.x, target.y):
            return self._drive_shelf_segment(goal_handle, result, feedback)

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

        self._ensure_fast_profile()

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
