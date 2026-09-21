"""주행 -> 스캔 -> pick -> 주행 -> place 미션을 도는 애플리케이션 노드 4개를,
로봇 네임스페이스마다 한 벌씩 띄운다.

    ros2 launch cobot3_bringup mission_nodes.launch.py                      # robot1 한 대
    ros2 launch cobot3_bringup mission_nodes.launch.py robots:=robot1,robot2  # 두 대

이 launch 가 책임지는 건 애플리케이션 노드뿐이다. 아래는 따로 띄워야 한다
(이 순서로):
    1) isaac_python isaacpjt/ros_bridge/sim_backend.py   # Isaac Sim + RPC 서버
    2) ros2 launch cobot3_navigation multi_navigation.launch.py   # Nav2 (robot1/robot2)
    3) ros2 launch cobot3_bringup tf.launch.py                    # m0609 팔 TF
    4) ros2 launch cobot3_bringup mission_nodes.launch.py         # 이 launch

띄우는 노드 (docs/08_ROS2_NODE_Graph.html 확정안 §01 의 5개 중 event_logger
빼고 4개 — event_logger 는 아직 없다). 네임스페이스마다 한 벌씩이다:
    nav_server            navigation/navigate_to         (cmd_vel 직접 주행)
    carrier_code_reader   perception/carrier_detected 발행,
                          perception/carrier_scan 서비스 (확정안대로 한 노드가 둘 다)
    pick_place_server     manipulation/pick_carrier, manipulation/place_carrier
    task_manager          위 셋을 부르는 행동트리

★ 네임스페이스가 로봇 식별자다
  노드들이 쓰는 이름은 전부 상대이름이라(앞에 / 가 없다) 여기서 준
  namespace 가 앞에 붙는다. robots:=robot1,robot2 로 띄우면
      /robot1/navigation/navigate_to   /robot2/navigation/navigate_to
      /robot1/manipulation/pick_carrier /robot2/manipulation/pick_carrier
      /robot1/perception/carrier_scan  /robot2/perception/carrier_scan
      /robot1/orchestrator/state       /robot2/orchestrator/state
  이 되고, /robot1 의 task_manager 에게는 /robot1 의 서버만 보인다 — "본 놈이
  가는 것" 이 코드가 아니라 배선으로 보장된다(docs/02 §2). Isaac 씬의
  isaac:namespace 와 Nav2(multi_navigation.launch.py)도 같은 robot1 · robot2
  이름을 쓰므로, amcl_pose · cmd_vel 도 이 네임스페이스로 자연히 맞는다.

  예외 하나: docking_server 는 도크가 공용 자원이라 전역 1개로 두고
  /docking/dock 만 절대이름이 된다(Dock.action 의 robot_name 필드로 구분).
  아직 미구현이라 이 launch 에 없다.

★ 기본값이 robot1 하나인 이유 — robots:=robot1,robot2 로 켜기 전에 할 일
  1) task_manager.py 의 PATROL_ROUTE · TEST_LOADER 가 아직 모듈 전역 상수다.
     지금 그대로 두 대를 띄우면 두 로봇이 같은 좌표로 가서 부딪힌다.
     로봇별 순찰 구역(docs/03 "도는 곳 다르게")을 잡아 파라미터나
     frames.yaml 로 갈라야 한다.
  2) sim_backend 가 nova_carter1 하나만 안다(ROBOT_PRIM_PATH 하드코딩).
     robot2 의 scan · pick · place 는 sim_backend 를 손보기 전까지 실패한다.
     주행(nav_server)은 sim_backend 를 쓰지 않으므로 robot2 도 지금 바로 된다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# 네임스페이스마다 이 넷을 한 벌씩 띄운다.
MISSION_NODES = [
    ("cobot3_navigation", "nav_server"),
    ("cobot3_perception", "carrier_code_reader"),
    ("cobot3_manipulation", "pick_place_server"),
    ("cobot3_orchestrator", "task_manager"),
]


def _setup(context):
    raw = LaunchConfiguration("robots").perform(context)
    namespaces = [ns.strip().strip("/") for ns in raw.split(",") if ns.strip()]
    if not namespaces:
        raise RuntimeError(
            "robots 인자가 비었다 — 예: robots:=robot1 또는 robots:=robot1,robot2")

    nodes = []
    for ns in namespaces:
        for package, executable in MISSION_NODES:
            nodes.append(Node(
                package=package,
                executable=executable,
                name=executable,
                namespace=ns,
                output="screen",
            ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robots", default_value="robot1",
            description="미션 노드를 띄울 로봇 네임스페이스. 쉼표로 여러 개 "
                        "(예: robot1,robot2). 기본값이 robot1 하나인 이유는 "
                        "모듈 독스트링 '기본값이 robot1 하나인 이유' 참고."),
        OpaqueFunction(function=_setup),
    ])
