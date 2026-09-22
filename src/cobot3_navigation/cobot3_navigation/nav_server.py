#!/usr/bin/env python3

"""
nav_server

두 종류의 navigation action을 제공한다.

1) navigation/navigate_to
   - 일반 이동
   - START / RETURN / 배송 이동
   - Nav2 NavigateToPose에 그대로 위임
   - 장애물 회피 / replanning / costmap 사용

2) navigation/patrol_to
   - PATROL A <-> B 전용
   - AMCL pose를 보며 직접 cmd_vel 제어
   - Nav2 controller / DWB 사용하지 않음
   - goal 시작 시 FORWARD / REVERSE를 한 번만 결정
"""

import math
import time

import rclpy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose

from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from cobot3_interfaces.action import NavigateTo


# ============================================================
# Common
# ============================================================

CONTROL_PERIOD_S = 0.1
LOG_PERIOD_S = 5.0


# ============================================================
# Nav2
# ============================================================

NAV2_SERVER_TIMEOUT_S = 10.0
NAV2_CANCEL_WAIT_S = 3.0


# ============================================================
# Direct patrol
# ============================================================

DRIVE_SPEED = 0.12
MIN_DRIVE_SPEED = 0.03
SLOWDOWN_DISTANCE = 0.40

POSITION_TOLERANCE = 0.12

STEER_KP = 0.8
MAX_ANGULAR_SPEED = 0.12
STEER_DEADBAND = math.radians(2.0)

# 실제 목표 방향과 차체 방향이 많이 틀어졌을 때는
# 직진/후진을 잠시 멈추고 방향부터 맞춘다.
TURN_IN_PLACE_ANGLE = math.radians(30.0)

DRIVE_TIMEOUT_S = 300.0


# ============================================================
# Math
# ============================================================

def yaw_of(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value, low, high):
    return max(low, min(high, value))


# ============================================================
# NavServer
# ============================================================

class NavServer(Node):

    def __init__(self):
        super().__init__("nav_server")

        self._amcl_pose = None
        self._last_patrol_log = 0.0

        self._cb_group = ReentrantCallbackGroup()

        # ====================================================
        # AMCL
        # direct patrol에서 사용
        # ====================================================

        amcl_qos = QoSProfile(depth=1)
        amcl_qos.reliability = ReliabilityPolicy.RELIABLE
        amcl_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            PoseWithCovarianceStamped,
            "amcl_pose",
            self._on_amcl_pose,
            amcl_qos,
        )

        # ====================================================
        # cmd_vel
        # direct patrol에서만 직접 사용
        # ====================================================

        self._cmd_vel_pub = self.create_publisher(
            Twist,
            "cmd_vel",
            10,
        )

        # ====================================================
        # Nav2 NavigateToPose client
        #
        # robot1 namespace:
        # /robot1/navigate_to_pose
        # ====================================================

        self._nav2_client = ActionClient(
            self,
            NavigateToPose,
            "navigate_to_pose",
            callback_group=self._cb_group,
        )

        # ====================================================
        # 일반 Navigation
        #
        # task_manager START / RETURN / NAV 등이 사용
        #
        # /robot1/navigation/navigate_to
        # ====================================================

        self._nav_server = ActionServer(
            self,
            NavigateTo,
            "navigation/navigate_to",
            execute_callback=self._execute_nav2,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._cb_group,
        )

        # ====================================================
        # Patrol 전용
        #
        # /robot1/navigation/patrol_to
        # ====================================================

        self._patrol_server = ActionServer(
            self,
            NavigateTo,
            "navigation/patrol_to",
            execute_callback=self._execute_patrol,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._cb_group,
        )

        ns = self.get_namespace().rstrip("/")

        self.get_logger().info(
            f"nav_server ready"
        )

        self.get_logger().info(
            f"NAV2   : {ns}/navigation/navigate_to"
        )

        self.get_logger().info(
            f"PATROL : {ns}/navigation/patrol_to"
        )

    # ========================================================
    # ROS callbacks
    # ========================================================

    def _on_amcl_pose(self, msg):
        self._amcl_pose = msg.pose.pose

    def _on_goal(self, goal_request):
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    # ========================================================
    # Helpers
    # ========================================================

    def _stop(self):
        self._cmd_vel_pub.publish(Twist())

    def _cancel_custom_goal(
        self,
        goal_handle,
        result,
    ):
        result.success = False
        result.fail_reason = NavigateTo.Result.CANCELED

        goal_handle.canceled()

        return result

    def _abort_custom_goal(
        self,
        goal_handle,
        result,
    ):
        result.success = False
        result.fail_reason = NavigateTo.Result.BLOCKED

        goal_handle.abort()

        return result

    # ========================================================
    # NAV2 MODE
    # ========================================================

    def _execute_nav2(self, goal_handle):
        """
        일반 navigation.

        custom NavigateTo
            ->
        Nav2 NavigateToPose

        실제 cmd_vel은 Nav2 controller_server가 만든다.
        """

        result = NavigateTo.Result()

        target_pose = goal_handle.request.pose

        gx = target_pose.pose.position.x
        gy = target_pose.pose.position.y

        self.get_logger().info(
            f"[NAV2] GOAL "
            f"x={gx:.3f}, "
            f"y={gy:.3f}"
        )

        # ----------------------------------------------------
        # Nav2 server
        # ----------------------------------------------------

        if not self._nav2_client.wait_for_server(
            timeout_sec=NAV2_SERVER_TIMEOUT_S
        ):
            self.get_logger().error(
                "[NAV2] navigate_to_pose unavailable"
            )

            return self._abort_custom_goal(
                goal_handle,
                result,
            )

        # ----------------------------------------------------
        # Nav2 goal
        # ----------------------------------------------------

        nav2_goal = NavigateToPose.Goal()

        nav2_goal.pose = target_pose
        nav2_goal.behavior_tree = ""

        self.get_logger().info(
            "[NAV2] sending goal"
        )

        send_future = self._nav2_client.send_goal_async(
            nav2_goal
        )

        # ----------------------------------------------------
        # Goal acceptance
        # ----------------------------------------------------

        while rclpy.ok() and not send_future.done():

            if goal_handle.is_cancel_requested:

                self.get_logger().info(
                    "[NAV2] canceled before acceptance"
                )

                return self._cancel_custom_goal(
                    goal_handle,
                    result,
                )

            time.sleep(CONTROL_PERIOD_S)

        if not rclpy.ok():

            return self._abort_custom_goal(
                goal_handle,
                result,
            )

        nav2_goal_handle = send_future.result()

        if (
            nav2_goal_handle is None
            or not nav2_goal_handle.accepted
        ):
            self.get_logger().error(
                "[NAV2] goal rejected"
            )

            return self._abort_custom_goal(
                goal_handle,
                result,
            )

        self.get_logger().info(
            "[NAV2] goal accepted"
        )

        nav2_result_future = (
            nav2_goal_handle.get_result_async()
        )

        last_log_time = 0.0

        # ----------------------------------------------------
        # Nav2 navigation
        # ----------------------------------------------------

        while (
            rclpy.ok()
            and not nav2_result_future.done()
        ):

            # -----------------------------------------------
            # Custom action cancel
            #
            # 예:
            # patrol 중 QR 감지 → task_manager preemption
            # RETURN 중 상위 트리가 취소
            # -----------------------------------------------

            if goal_handle.is_cancel_requested:

                self.get_logger().info(
                    "[NAV2] cancel requested"
                )

                cancel_future = (
                    nav2_goal_handle.cancel_goal_async()
                )

                deadline = (
                    time.monotonic()
                    + NAV2_CANCEL_WAIT_S
                )

                # Nav2가 취소를 처리할 시간을 준다.
                # outer action을 너무 빨리 CANCELED 처리하면
                # task_manager는 정지했다고 생각하지만
                # Nav2 controller가 잠깐 더 움직일 수 있다.
                while (
                    rclpy.ok()
                    and time.monotonic() < deadline
                ):

                    if nav2_result_future.done():
                        break

                    time.sleep(
                        CONTROL_PERIOD_S
                    )

                self.get_logger().info(
                    "[NAV2] navigation canceled"
                )

                return self._cancel_custom_goal(
                    goal_handle,
                    result,
                )

            now = time.monotonic()

            if now - last_log_time >= LOG_PERIOD_S:

                self.get_logger().info(
                    "[NAV2] navigating..."
                )

                last_log_time = now

            time.sleep(CONTROL_PERIOD_S)

        # ----------------------------------------------------
        # Result
        # ----------------------------------------------------

        if not rclpy.ok():

            return self._abort_custom_goal(
                goal_handle,
                result,
            )

        wrapped_result = (
            nav2_result_future.result()
        )

        if wrapped_result is None:

            self.get_logger().error(
                "[NAV2] no result"
            )

            return self._abort_custom_goal(
                goal_handle,
                result,
            )

        status = wrapped_result.status

        # SUCCESS
        if status == GoalStatus.STATUS_SUCCEEDED:

            result.success = True
            result.fail_reason = NavigateTo.Result.NONE

            goal_handle.succeed()

            self.get_logger().info(
                f"[NAV2] ARRIVED "
                f"x={gx:.3f}, "
                f"y={gy:.3f}"
            )

            return result

        # CANCELED
        if status == GoalStatus.STATUS_CANCELED:

            self.get_logger().info(
                "[NAV2] CANCELED"
            )

            return self._cancel_custom_goal(
                goal_handle,
                result,
            )

        # FAILED / ABORTED
        self.get_logger().warning(
            f"[NAV2] FAILED "
            f"status={status}"
        )

        return self._abort_custom_goal(
            goal_handle,
            result,
        )

    # ========================================================
    # DIRECT PATROL MODE
    # ========================================================

    def _execute_patrol(self, goal_handle):
        """
        PATROL A <-> B 전용.

        Nav2 controller 사용 안 함.

        goal을 받은 순간:
            현재 yaw와 목표 위치 방향 비교
            FORWARD / REVERSE 한 번 결정

        그 goal이 끝날 때까지 direction 고정.
        """

        result = NavigateTo.Result()
        feedback = NavigateTo.Feedback()

        goal_pose = goal_handle.request.pose.pose

        gx = float(goal_pose.position.x)
        gy = float(goal_pose.position.y)

        self.get_logger().info(
            f"[PATROL] GOAL "
            f"x={gx:.3f}, "
            f"y={gy:.3f}"
        )

        # ----------------------------------------------------
        # AMCL 기다리기
        # ----------------------------------------------------

        while (
            rclpy.ok()
            and self._amcl_pose is None
        ):

            if goal_handle.is_cancel_requested:

                self._stop()

                return self._cancel_custom_goal(
                    goal_handle,
                    result,
                )

            time.sleep(CONTROL_PERIOD_S)

        if not rclpy.ok():

            self._stop()

            return self._abort_custom_goal(
                goal_handle,
                result,
            )

        # ----------------------------------------------------
        # FORWARD / REVERSE
        #
        # goal 시작 시 딱 한 번 결정한다.
        # ----------------------------------------------------

        start_x = self._amcl_pose.position.x
        start_y = self._amcl_pose.position.y

        start_yaw = yaw_of(
            self._amcl_pose.orientation
        )

        initial_heading = math.atan2(
            gy - start_y,
            gx - start_x,
        )

        initial_error = wrap_angle(
            initial_heading
            - start_yaw
        )

        if abs(initial_error) <= math.pi / 2.0:

            direction = 1.0
            direction_name = "FORWARD"

        else:

            direction = -1.0
            direction_name = "REVERSE"

        self.get_logger().info(
            f"[PATROL] direction locked: "
            f"{direction_name} | "
            f"yaw={math.degrees(start_yaw):.1f} | "
            f"heading={math.degrees(initial_heading):.1f}"
        )

        # ----------------------------------------------------
        # Drive
        # ----------------------------------------------------

        deadline = (
            time.monotonic()
            + DRIVE_TIMEOUT_S
        )

        self._last_patrol_log = 0.0

        while (
            rclpy.ok()
            and time.monotonic() < deadline
        ):

            # -----------------------------------------------
            # Cancel
            # -----------------------------------------------

            if goal_handle.is_cancel_requested:

                self._stop()

                self.get_logger().info(
                    "[PATROL] canceled"
                )

                return self._cancel_custom_goal(
                    goal_handle,
                    result,
                )

            if self._amcl_pose is None:

                self._stop()

                time.sleep(
                    CONTROL_PERIOD_S
                )

                continue

            # -----------------------------------------------
            # Current pose
            # -----------------------------------------------

            x = self._amcl_pose.position.x
            y = self._amcl_pose.position.y

            yaw = yaw_of(
                self._amcl_pose.orientation
            )

            dx = gx - x
            dy = gy - y

            distance = math.hypot(
                dx,
                dy,
            )

            # -----------------------------------------------
            # ARRIVED
            # -----------------------------------------------

            if distance <= POSITION_TOLERANCE:

                self._stop()

                result.success = True
                result.fail_reason = NavigateTo.Result.NONE

                goal_handle.succeed()

                self.get_logger().info(
                    f"[PATROL] ARRIVED "
                    f"x={gx:.3f}, "
                    f"y={gy:.3f}, "
                    f"distance={distance:.3f}"
                )

                return result

            # -----------------------------------------------
            # 실제 현재 위치 -> 목표 위치 방향
            # -----------------------------------------------

            heading_to_goal = math.atan2(
                dy,
                dx,
            )

            # -----------------------------------------------
            # Body yaw
            #
            # FORWARD:
            #    앞이 목표를 향함
            #
            # REVERSE:
            #    뒤가 목표를 향함
            # -----------------------------------------------

            if direction > 0.0:

                desired_body_yaw = (
                    heading_to_goal
                )

            else:

                desired_body_yaw = (
                    wrap_angle(
                        heading_to_goal
                        + math.pi
                    )
                )

            steer_error = wrap_angle(
                desired_body_yaw
                - yaw
            )

            if abs(steer_error) < STEER_DEADBAND:
                steer_error = 0.0

            # -----------------------------------------------
            # Speed
            # -----------------------------------------------

            if distance < SLOWDOWN_DISTANCE:

                speed = (
                    DRIVE_SPEED
                    * distance
                    / SLOWDOWN_DISTANCE
                )

                speed = max(
                    MIN_DRIVE_SPEED,
                    speed,
                )

            else:

                speed = DRIVE_SPEED

            twist = Twist()

            # -----------------------------------------------
            # 방향이 너무 많이 틀어진 경우
            # 실제 goal 방향으로만 회전한다.
            # -----------------------------------------------

            if abs(steer_error) > TURN_IN_PLACE_ANGLE:

                twist.linear.x = 0.0

            else:

                speed_scale = max(
                    0.35,
                    1.0
                    - (
                        abs(steer_error)
                        / TURN_IN_PLACE_ANGLE
                    ),
                )

                twist.linear.x = (
                    direction
                    * speed
                    * speed_scale
                )

            twist.angular.z = clamp(
                STEER_KP * steer_error,
                -MAX_ANGULAR_SPEED,
                MAX_ANGULAR_SPEED,
            )

            self._cmd_vel_pub.publish(
                twist
            )

            # -----------------------------------------------
            # Feedback
            # -----------------------------------------------

            feedback.distance_remaining_m = float(
                distance
            )

            goal_handle.publish_feedback(
                feedback
            )

            # -----------------------------------------------
            # 5초마다 로그
            # -----------------------------------------------

            now = time.monotonic()

            if (
                now - self._last_patrol_log
                >= LOG_PERIOD_S
            ):

                self.get_logger().info(
                    f"[PATROL] {direction_name} "
                    f"pose=({x:.3f},{y:.3f}) "
                    f"goal=({gx:.3f},{gy:.3f}) "
                    f"dist={distance:.3f} "
                    f"yaw={math.degrees(yaw):.1f} "
                    f"heading="
                    f"{math.degrees(heading_to_goal):.1f} "
                    f"desired="
                    f"{math.degrees(desired_body_yaw):.1f} "
                    f"steer="
                    f"{math.degrees(steer_error):.1f} "
                    f"v={twist.linear.x:.3f} "
                    f"w={twist.angular.z:.3f}"
                )

                self._last_patrol_log = now

            time.sleep(
                CONTROL_PERIOD_S
            )

        # ----------------------------------------------------
        # Timeout
        # ----------------------------------------------------

        self._stop()

        self.get_logger().warning(
            f"[PATROL] navigation timeout "
            f"({DRIVE_TIMEOUT_S:.0f}s)"
        )

        return self._abort_custom_goal(
            goal_handle,
            result,
        )


# ============================================================
# Main
# ============================================================

def main():

    rclpy.init()

    node = NavServer()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()

    finally:

        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()