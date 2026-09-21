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
  1) PATROL_ROUTE 가 아직 task_manager.py 의 모듈 전역 상수다. 두 대가 같은
     순찰 경로를 돌면 선반 앞에서 부딪힌다. 로봇별 순찰 구역(docs/03
     "도는 곳 다르게")을 잡아 파라미터나 frames.yaml 로 갈라야 한다.
     TEST_LOADER 쪽은 아래 "로더 차선 조율" 이 처리한다 — 한 대만 차선에
     들어가고 나머지는 대기 자리에서 기다린다. 다만 그건 출발 시점 판단으로
     커밋하므로 사각지대가 남아 있다(task_manager.py "알려진 갭" 참고).
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


# ══════════════════════════════════════════════════════════════════════════
#  로더 차선 조율 — 두 대가 같은 로더로 갈 때
#
#  로더 주차점에는 제자리회전 여유가 거의 없고(yaw 오차 9도에서 뒷모서리가
#  장애물에 닿는다) 접근선과 이탈선이 겹친다. 근거와 실측은 task_manager.py 의
#  DEFAULT_STAGING_POSE 주석, 검증은 isaacpjt/tools/check_staging_poses.py.
#  그래서 두 대를 띄울 때는 서로의 orchestrator/state 를 보고 차선을 양보한다.
#
#  ★ 우선순위는 고정이다. PRIORITY_ROBOT 만 상대의 'wait' 에 양보하지 않는다.
#    이 한 칸이 동순위를 깨는 장치다 — 양쪽이 서로의 대기를 기다리면 둘 다
#    멈추고, 양쪽이 서로의 대기를 무시하면 동시에 출발해 부딪힌다.
#
#  ★ 기본값을 "양보한다" 쪽으로 둔 이유: 두 로봇에 같은 값이 잘못 들어갔을 때
#    둘 다 우선순위를 가지면 충돌하고, 둘 다 양보하면 대기 자리에서 멈춘다.
#    멈추는 쪽이 안전하다. 그래서 PRIORITY_ROBOT 에 해당하는 한 대만 예외다.
PRIORITY_ROBOT = "robot1"

# 상대가 차선을 쓰고 있다고 보는 단계. task_manager.py 의
# DEFAULT_PEER_BUSY_STAGES 와 같은 값이고, 여기서 로봇별로 한 칸만 달라진다.
#   return 이 들어 있는 이유: 로더에서 후진해 나오는 경로가 접근선과 같은 선이다.
#   approach 가 없는 이유: 차선 밖 대기 자리로 가는 중이라 방해되지 않는다.
LANE_STAGES = ["nav", "push", "place", "return"]

# ★ 대기 자리는 로봇마다 달라야 한다 — 같은 점을 쓰면 대기 자리에서 부딪힌다.
#   세 조건(자유공간 · 도착각 상한 · 이탈 경로 회피)을 전부 통과한 값이다.
#   지도를 새로 만들면 check_staging_poses.py 로 다시 검증해라.
STAGING_BY_ROBOT = {
    "robot1": [0.80, -0.40, 0.0],    # 도착각 +7.5도 · 로더 3.08 m · 이탈여유 0.32 m
    "robot2": [-0.80, 0.00, 0.0],    # 도착각 +0.0도 · 로더 4.65 m · 이탈여유 0.32 m
}

# 상대가 누구인가. 세 대 이상이 되면 이 표로는 안 되고, 도크처럼 전역 조정
# 노드를 두는 편이 낫다(docs/02 §2 의 docking_server 논리와 같다).
PEER_OF = {"robot1": "robot2", "robot2": "robot1"}


def _task_manager_params(ns, namespaces):
    """task_manager 하나에 넘길 파라미터. 조율이 필요 없으면 좌표만 넘긴다."""
    params = {"staging_pose": STAGING_BY_ROBOT.get(ns, [0.80, -0.60, 0.0])}

    peer = PEER_OF.get(ns)
    if not peer or peer not in namespaces:
        # 상대가 같이 안 뜨면 구독하지 않는다. 안 뜨는 토픽을 구독해 둬도
        # 동작은 같지만(한 번도 못 받으면 통과), 로그에서 헷갈린다.
        return params

    busy = list(LANE_STAGES)
    if ns != PRIORITY_ROBOT:
        busy.append("wait")          # 우선순위 없는 쪽만 상대의 대기에도 양보
    params["peer_state_topic"] = f"/{peer}/orchestrator/state"
    params["peer_busy_stages"] = busy
    return params


def _setup(context):
    raw = LaunchConfiguration("robots").perform(context)
    namespaces = [ns.strip().strip("/") for ns in raw.split(",") if ns.strip()]
    if not namespaces:
        raise RuntimeError(
            "robots 인자가 비었다 — 예: robots:=robot1 또는 robots:=robot1,robot2")

    nodes = []
    for ns in namespaces:
        for package, executable in MISSION_NODES:
            params = (_task_manager_params(ns, namespaces)
                      if executable == "task_manager" else {})
            nodes.append(Node(
                package=package,
                executable=executable,
                name=executable,
                namespace=ns,
                output="screen",
                parameters=[params] if params else [],
            ))

    # ★ event_logger 만 네임스페이스 없이 전역 1개다.
    #   /trace/event 가 절대이름이라 로봇이 몇 대든 여기로 모이고,
    #   DB 커넥션을 가진 노드는 이것 하나뿐이다 (docs/DB구성.md §1).
    #   DB 가 안 떠 있어도 무해하다 — 스풀 파일로 흘리고 로봇은 그대로 돈다.
    nodes.append(Node(
        package="cobot3_orchestrator",
        executable="event_logger",
        name="event_logger",
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
