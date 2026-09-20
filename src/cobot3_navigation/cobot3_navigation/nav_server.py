"""
nav_server — /navigation/navigate_to (cobot3_interfaces/NavigateTo) 를
cmd_vel 직접 주행으로 처리한다. 원래는 Nav2(navigate_to_pose)에 위임하는
설계였는데(docs/08_ROS2_NODE_Graph.html 확정안), 두 가지 실측 문제로
전 구간을 이 노드가 amcl_pose 를 보며 직접 cmd_vel 로 몰도록 바꿨다:
  1) DWB 가 선반 근처에서 계속 BLOCKED/ABORTED 로 막혔다 — footprint 의
     제자리회전 스윕(0.66m)이 선반 여유(0.26~0.48m)보다 넓어서다.
  2) 선반 구역만 직접주행하고 그 밖은 Nav2 에 맡기는 절충안도 시도했는데,
     선반 밖으로 나가서 Nav2 로 넘어가는 순간 힘들게 맞춘 정렬이 다시
     흐트러졌다(실측). Nav2 의 DWB 자체가 이 로봇 footprint 에 안 맞아서
     생기는 문제라, 선반 안팎을 가리지 않고 이 노드가 전부 직접 몬다.

    ros2 run cobot3_navigation nav_server

씬(isaacpjt/worlds/simple_factory_layout.usda)의 nova_carter1 은 Isaac 쪽에서
isaac:namespace = "robot1" 로 떠 있다. 다른 로봇에 붙이려면
    ros2 run cobot3_navigation nav_server --ros-args -p robot_namespace:=robot2

★ 트레이드오프: Nav2 의 costmap 기반 장애물 회피·전역 경로계획을 이제 전혀
안 쓴다 — amcl_pose 로컬라이제이션만 계속 쓰고(그래서 multi_navigation.
launch.py 는 여전히 필요하다), controller_server/planner_server/
bt_navigator 는 이 노드가 안 부른다. 지금 씬의 목표들(PATROL_ROUTE 두 점,
TEST_LOADER)이 열린 공간을 지나는 걸 전제로 한 결정이다 — 경로 중간에
예상 못 한 장애물이 생기는 시나리오라면 이 접근은 안 맞는다.

속도: 목표나 현재 위치 둘 중 하나라도 선반 구역(SHELF_ZONE, cobot3_perception/
carrier_code_reader.py 의 SHELF_ZONE 과 반드시 같은 값) 안이면
SHELF_DRIVE_MAX_V(느림 — QR 스캔이 흔들리면 안 된다), 완전히 밖이면
OPEN_DRIVE_MAX_V(빠름)를 쓴다.

후진: 목표가 로봇 진행 방향 뒤에 있으면 제자리로 돌지 않고 그냥 후진한다
— yaw=0 을 유지한 채로.

★ 조향 버그 이력: 회전 보정을 최종 목표 yaw 가 아니라 목표 "지점" 방향
(heading_to_goal)으로 해야 한다. 이걸 gyaw 로 잘못 맞췄다가 궤적이 옆으로
휘어져 실제로 선반에 부딪힌 적이 있다 — 그 뒤로 고쳤다. 정렬 오차가 크면
속도를 0까지 줄이는 안전장치와, 선반 구역에서의 y 하드 리밋
(SHELF_SAFE_Y_MAX)도 그 사고 이후 추가했다.

도착 후 미세정렬: 도착 직후 실제 yaw 오차를 AMCL 로 다시 확인해서 크면
cmd_vel 로 아주 느리게 제자리에서 마지막 정렬만 보정한다(QR 관측의 고정
관절 자세는 오차를 못 버틴다). amcl_pose 는 로봇이 안 움직이면 새 메시지를
안 내므로(update_min_d/a 문턱), 구독은 RELIABLE + TRANSIENT_LOCAL 로 해야
latched 된 최신값을 바로 받는다(BEST_EFFORT 로 했다가 정지 상태에서 pose 를
못 받는 걸 실측으로 확인했다).
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from cobot3_interfaces.action import NavigateTo

ALIGN_YAW_TOL_RAD = math.radians(5.0)   # QR 관측 자세가 버틸 수 있는 최종 yaw 오차
ALIGN_MAX_W = 0.15                      # rad/s — 선반 근처라 아주 느리게만 돈다
ALIGN_TIMEOUT_S = 15.0

SHELF_DRIVE_MAX_V = 0.12   # m/s — 선반 구역. 흔들리면 QR 디코드가 깨지므로 느리게
# ★ 임시로 올림(디버깅용, 12_place_test.py TRANSPORT 반복 검증 때 180s 넘게
# 걸리는 걸 줄이려고) — 검증 끝나면 0.5 로 되돌릴 것. 선반 구역
# (SHELF_DRIVE_MAX_V)은 안전 때문에 그대로 둔다.
OPEN_DRIVE_MAX_V = 1.2     # m/s — 선반 밖. Nav2 없이 직접 모니 보수적으로 잡았다
DRIVE_POS_TOL = 0.08       # m
DRIVE_TIMEOUT_S = 180.0    # 선반 밖은 거리가 멀 수 있어 넉넉하게

# 하드 안전장치 — 조향 로직에 버그가 있든 없든, 선반 구역에서 y 가 이 선을
# 넘으면 무조건 전진을 막는다. 실측 충돌(정렬 오차로 궤적이 +y 로 휘어져
# 선반에 닿음) 이후 추가한 방어선이다. SHELF_ZONE_Y 상한(아래)보다 위로
# 잡아서, 정상 주행 중에는 거의 안 걸리고 진짜 이탈할 때만 작동한다.
SHELF_SAFE_Y_MAX = 1.70

# cobot3_perception/carrier_code_reader.py 의 SHELF_ZONE_X/Y 와 반드시 같아야
# 한다 — 스캔 구역과 저속 구역이 같은 자리를 가리켜야 한다. QR 감지가 잘
# 안 걸린다는 실측 피드백으로 원래(-6.85,-4.65)/(1.10,1.80)보다 넓혔다 —
# Y 상한은 SHELF_SAFE_Y_MAX(1.70) 보다 낮게 유지해야 한다(넘으면 안전장치가
# 정상 주행 중에도 걸린다).
SHELF_ZONE_X = (-7.10, -4.40)
SHELF_ZONE_Y = (1.00, 1.65)


def _yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class NavServer(Node):

    def __init__(self):
        super().__init__("nav_server")
        self.declare_parameter("robot_namespace", "robot1")
        ns = self.get_parameter("robot_namespace").value
        self._ns = ns

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

        self.get_logger().info(
            "nav_server ready — /navigation/navigate_to (cmd_vel 직접 주행, Nav2 위임 없음)")

    def _on_amcl_pose(self, msg):
        self._amcl_pose = msg.pose.pose

    def _in_shelf_zone(self, x, y):
        return SHELF_ZONE_X[0] <= x <= SHELF_ZONE_X[1] and SHELF_ZONE_Y[0] <= y <= SHELF_ZONE_Y[1]

    def _fine_align(self, goal_pose):
        """도착 직후 실제 yaw 오차를 재확인하고, 크면 cmd_vel 로 느리게
        제자리 보정한다. 모듈 독스트링 "도착 후 미세정렬" 참고."""
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

    def _on_goal(self, goal_request):
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        """목표를 amcl_pose 를 보며 직접 cmd_vel 로 몬다 — 모듈 독스트링
        참고. Nav2(navigate_to_pose)는 이제 아예 안 쓴다."""
        result = NavigateTo.Result()
        feedback = NavigateTo.Feedback()

        goal_pose = goal_handle.request.pose.pose
        gx, gy = goal_pose.position.x, goal_pose.position.y
        gyaw = _yaw_of(goal_pose.orientation)

        cur = self._amcl_pose
        cur_in_zone = cur is not None and self._in_shelf_zone(cur.position.x, cur.position.y)
        slow = cur_in_zone or self._in_shelf_zone(gx, gy)
        max_v = SHELF_DRIVE_MAX_V if slow else OPEN_DRIVE_MAX_V

        deadline = time.monotonic() + DRIVE_TIMEOUT_S
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
            if dist <= DRIVE_POS_TOL:
                reached = True
                break

            # ★ 실측 충돌로 확인된 버그: 회전 보정을 목표 지점 방향이 아니라
            # 최종 목표 yaw(gyaw, 고정 0도)로 맞추면, 실제 진행 방향(로컬
            # x축)과 목표 지점 방향이 어긋난 채로 전진하면서 궤적이 옆으로
            # 휘어진다. 반드시 목표 "지점" 방향(heading_to_goal)으로 조향
            # 해야 한다 — gyaw 정렬은 도착 후 _fine_align 이 따로 한다.
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
            tw.linear.x = direction * max_v * speed_scale
            tw.angular.z = max(-ALIGN_MAX_W, min(ALIGN_MAX_W, 1.5 * steer_err))

            # 하드 안전장치 — 선반 구역에서만 적용. y 가 이미 선반 쪽으로
            # 너무 들어가 있으면 방향을 안 가리고 직진 자체를 막는다.
            if slow and y >= SHELF_SAFE_Y_MAX:
                tw.linear.x = 0.0
                self.get_logger().warning(
                    f"y={y:.3f} 가 안전선({SHELF_SAFE_Y_MAX})을 넘어 직진을 막았다")

            self._cmd_vel_pub.publish(tw)
            feedback.distance_remaining_m = float(dist)
            goal_handle.publish_feedback(feedback)
            time.sleep(0.1)

        self._cmd_vel_pub.publish(Twist())

        if not reached:
            self.get_logger().warning(f"주행 타임아웃({DRIVE_TIMEOUT_S:.0f}s)")
            result.success = False
            result.fail_reason = NavigateTo.Result.BLOCKED
            goal_handle.abort()
            return result

        self._fine_align(goal_pose)
        result.success = True
        result.fail_reason = NavigateTo.Result.NONE
        goal_handle.succeed()
        return result


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
