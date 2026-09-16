import math
import time
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped


def get_quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

def get_euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw

def print_final_pose(pose_msg):
    if not pose_msg:
        return
    pos = pose_msg.pose.position
    ori = pose_msg.pose.orientation
    roll_rad, pitch_rad, yaw_rad = get_euler_from_quaternion(ori.x, ori.y, ori.z, ori.w)
    yaw_deg = math.degrees(yaw_rad)

    print("-" * 50)
    print(f"📍 최종 위치: X = {pos.x:.3f} m, Y = {pos.y:.3f} m")
    print(f"🧭 최종 바라보는 방향 (Heading): {yaw_deg:.1f}°")
    print("-" * 50)

def create_pose(navigator, x, y, yaw_deg):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    q = get_quaternion_from_euler(0, 0, math.radians(yaw_deg))
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose

# ==========================================================
# 🚀 메인 주행 로직 (경유지를 goToPose로 하나씩 순차 주행)
#    goThroughPoses는 중간 경유지를 "느슨한" 통과점으로만 취급해
#    실제로 그 지점을 정확히 지나가지 않는 경우가 있어, 각 경유지마다
#    goToPose를 호출해 도착을 확인한 뒤 다음 경유지로 넘어가도록 함.
# ==========================================================
def main():
    rclpy.init()
    nav = BasicNavigator()

    # 1. 출발점 설정
    init_pose = create_pose(nav, -5.79, -1.01, 0.003)
    nav.setInitialPose(init_pose)
    nav.waitUntilNav2Active()

    # 2. 경유지(Waypoints) 리스트 생성
    waypoints = []
    waypoints.append(create_pose(nav, 2.18, 4.17, 0.004))     # 경유지 1
    waypoints.append(create_pose(nav, -4.9, 11.58, 0.006))    # 경유지 2
    waypoints.append(create_pose(nav, -3.61, -4.99, 0.004))   # 경유지 3 (최종 목적지)

    # 3. 경유지를 하나씩 순차적으로 goToPose 실행
    last_pose = None
    final_result = TaskResult.SUCCEEDED

    for idx, wp in enumerate(waypoints, start=1):
        print(f"🚀 경유지 {idx}/{len(waypoints)} 로 이동을 시작합니다...")
        nav.goToPose(wp)

        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            if feedback:
                last_pose = feedback.current_pose
                print(f"경유지 {idx} 까지 남은 거리: {feedback.distance_remaining:.2f} m")
            time.sleep(1.0)

        result = nav.getResult()
        final_result = result

        if result == TaskResult.SUCCEEDED:
            print(f"✅ 경유지 {idx} 도착 완료!")
        elif result == TaskResult.CANCELED:
            print(f"⚠️ 경유지 {idx} 이동 중 취소되었습니다.")
            break
        elif result == TaskResult.FAILED:
            print(f"❌ 경유지 {idx} 이동 실패 (장애물 등으로 경로를 찾을 수 없음).")
            break

    # 4. 결과 처리
    if final_result == TaskResult.SUCCEEDED:
        print('\n🎉 모든 경유지를 거쳐 목적지에 도착 완료!')
        print_final_pose(last_pose)
    elif final_result == TaskResult.CANCELED:
        print('\n⚠️ 주행이 취소되었습니다.')
    elif final_result == TaskResult.FAILED:
        print('\n❌ 주행 실패 (장애물 등으로 경로를 찾을 수 없음).')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
