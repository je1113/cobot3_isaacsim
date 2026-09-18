"""주행 -> 스캔 -> pick -> 주행 -> place 미션을 도는 애플리케이션 노드 4개를 띄운다.

    ros2 launch cobot3_bringup mission_nodes.launch.py

이 launch 가 책임지는 건 애플리케이션 노드뿐이다. 아래는 따로 띄워야 한다
(이 순서로):
    1) isaac_python isaacpjt/ros_bridge/sim_backend.py   # Isaac Sim + RPC 서버
    2) ros2 launch cobot3_navigation multi_navigation.launch.py   # Nav2 (robot1/robot2)
    3) ros2 launch cobot3_bringup tf.launch.py                    # m0609 팔 TF
    4) ros2 launch cobot3_bringup mission_nodes.launch.py         # 이 launch

띄우는 노드 (docs/08_ROS2_NODE_Graph.html 확정안 §01 의 5개 중 event_logger
빼고 4개 — event_logger 는 아직 없다)
    nav_server            /navigation/navigate_to        -> /robot1/navigate_to_pose (Nav2)
    carrier_code_reader   patrol 중 QR 실시간 감시 -> /perception/carrier_detected,
                          /perception/carrier_scan 서비스 (확정안대로 한 노드가 둘 다)
    pick_place_server     /manipulation/pick_carrier, /manipulation/place_carrier
    task_manager          위 셋을 부르는 상태기계 — patrol 로 시작한다

carrier_code_reader 가 실시간 감시까지 맡으면서 수동으로 carrier_detected 를
쏠 필요는 없어졌다 (patrol 중 선반 구역에 들어서면 팔을 관측 자세로 고정해
두고 QR 디코드를 시도한다 — 자세한 원리는 carrier_code_reader.py 상단 주석
참고).

task_manager.py 상단에 적힌 대로 아직 남은 갭:
    - PATROL_ROUTE / TEST_LOADER 좌표가 씬 좌표와 맞는지 계속 확인이 필요하다
      (시험용 값에서 출발했다).
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="cobot3_navigation",
            executable="nav_server",
            name="nav_server",
            output="screen",
        ),
        Node(
            package="cobot3_perception",
            executable="carrier_code_reader",
            name="carrier_code_reader",
            output="screen",
        ),
        Node(
            package="cobot3_manipulation",
            executable="pick_place_server",
            name="pick_place_server",
            output="screen",
        ),
        Node(
            package="cobot3_orchestrator",
            executable="task_manager",
            name="task_manager",
            output="screen",
        ),
    ])
