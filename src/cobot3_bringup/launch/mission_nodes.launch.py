"""출발 -> 스캔 -> pick -> 주행 -> place -> (스택) -> 도킹 을 한 판 도는
애플리케이션 노드 4개를, 로봇 네임스페이스마다 한 벌씩 띄운다.

    ros2 launch cobot3_bringup mission_nodes.launch.py                       # robot1 한 대
    ros2 launch cobot3_bringup mission_nodes.launch.py robots:=robot1,robot2   # 두 대

★ 이 브랜치에는 순찰이 없다
  task_manager 는 고정 좌표를 따라 한 판만 돈다(그 파일 머리주석 참고).
  순찰·웹 배차(ExecuteTask)·선반 설정 다시읽기는 트리에서 들어냈다. 그래서
  이 launch 도 시나리오를 고르지 않는다 — 넘기는 것은 로봇별 좌표와 순서뿐이다.

씬은 매거진 둘(로봇당 하나)과 스택 하나만 놓인 시험용을 쓴다:
    SIM_WORLD_USD=$HOME/cobot3_ws/isaacpjt/worlds/simple_factory_layout_test.usda \\
        isaac_python isaacpjt/ros_bridge/sim_backend.py

이 launch 가 책임지는 건 애플리케이션 노드뿐이다. 아래는 따로 띄워야 한다
(이 순서로):
    1) isaac_python isaacpjt/ros_bridge/sim_backend.py   # Isaac Sim + RPC 서버
    2) ros2 launch cobot3_navigation multi_navigation.launch.py   # Nav2 (robot1/robot2)
    3) ros2 launch cobot3_bringup tf.launch.py                    # m0609 팔 TF
    4) ros2 launch cobot3_bringup mission_nodes.launch.py         # 이 launch

띄우는 노드. 네임스페이스마다 한 벌씩이다:
    nav_server            navigation/navigate_to         (cmd_vel 직접 주행)
    carrier_code_reader   perception/carrier_scan 서비스
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

  sim_backend 도 로봇 둘을 안다(RobotRig). pick_place_server 와
  carrier_code_reader 가 자기 네임스페이스를 robot_id 로 실어 보낸다.

  예외 하나: docking_server 는 도크가 공용 자원이라 전역 1개로 두고
  /docking/dock 만 절대이름이 된다. 아직 미구현이라 이 launch 에 없다.
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
#  로봇별 좌표와 순서
#
#  좌표는 전부 (x, y, yaw_deg) 를 평탄화한 것이다. 값의 근거는 task_manager.py
#  의 DEFAULT_* 상수 주석에 있고, robot1 값이 그 기본값과 같다 —
#  isaacpjt/tools/test_peer_yield.py 가 둘이 어긋나지 않았는지 검사한다.
#  robot2 는 같은 규칙을 Shelf_02(통로가 +y 라 yaw 180)와 자기 도크 줄
#  (y=-7.575)에 적용한 것이다.
#
#  ★ 실측이 아니다. 선반 앞 정차 관계(전면에서 0.4167 m)와 nav_server 의 직선
#    주행·후진 규칙에서 기하로 계산한 뒤, 점유격자 위에서 차체 사각형을 실제로
#    놓아 정차점·구간·회전 여유를 확인했다. Isaac 에서 한 번 돌려 보고
#    부딪히는 점이 있으면 여기서 고친다.
#
#  ★ 스택·검사·로더이탈 경로는 로봇마다 다르지 않아서 여기서 안 넘긴다 —
#    task_manager.py 의 기본값을 그대로 쓴다. 스택은 하나뿐이고 한 대만
#    맡으므로 둘이 같은 좌표를 써도 겹치지 않는다.
# ══════════════════════════════════════════════════════════════════════════

# 도크 → 스캔 자리. 자기 줄을 따라 서쪽으로 나간 뒤 선반 줄 동쪽 끝으로
# 올라가고, 마지막 구간은 회전 없이 들어간다(robot1 후진 · robot2 전진).
SCAN_ROUTE_BY_ROBOT = {
    "robot1": [3.5, -6.591, 180.0,   1.0, 2.036, 0.0,     -0.641, 2.036, 0.0],
    "robot2": [3.5, -7.575, 180.0,   1.0, -1.056, 180.0,  -0.516, -1.056, 180.0],
}

# 스캔 자리에서 선반 줄을 벗어나는 동쪽 이탈점. 이 점 없이 바로 북상하면
# 차체가 선반을 친다. 아래 대기 자리와 이어 붙여 approach_route 가 된다.
ROW_EXIT_BY_ROBOT = {
    "robot1": [1.50, 2.036, 0.0],
    "robot2": [1.50, -1.056, 180.0],
}

# ★ 대기 자리는 로봇마다 달라야 한다 — 같은 점을 쓰면 대기 자리에서 부딪힌다.
#   로더 이탈 궤적이 남서로 흐르므로 둘 다 차선 북쪽에 둔다.
#       robot1 (1.50, 5.15)  도착각 -13.7도
#       robot2 (0.10, 5.15)  도착각  -8.5도
#   세 조건(자유공간 · 도착각 상한 · 이탈 경로 회피)을 전부 통과한 값이다.
#   지도를 새로 만들면 check_staging_poses.py 로 다시 검증해라.
STAGING_BY_ROBOT = {
    "robot1": [1.50, 5.15, 0.0],
    "robot2": [0.10, 5.15, 0.0],
}

# 로더(또는 검사 스테이션) → 도크. 마지막 점이 도크이고 씬의 시작 자세와 같다.
# 둘째 점은 자기 줄 위이고 이웃 줄과 0.98 m 떨어져 있어, 거기서 도는 꼬리
# 스윕(0.656 m)이 이웃을 안 친다.
DOCK_ROUTE_BY_ROBOT = {
    "robot1": [3.0, -4.6, 90.0,   3.0, -6.3, 180.0,     5.5, -6.591, 180.0],
    "robot2": [3.0, -4.6, 90.0,   3.0, -7.575, 180.0,   5.5, -7.575, 180.0],
}

# 로더 차선 순서. 1 이 먼저다. 남쪽 로봇(robot2)이 먼저 나가야 북쪽 로봇의
# 꼬리 스윕이 이웃을 안 치므로(task_manager.py "출발 게이트") robot2 가 1 이다.
# 그 순서가 로더에서도 이어져 robot2 가 먼저 place 하고 스택을 맡는 것이
# 기본 흐름이다.
LANE_PRIORITY_BY_ROBOT = {"robot1": 2, "robot2": 1}

# 상대가 차선을 쓰고 있다고 보는 단계. task_manager.py 의
# DEFAULT_PEER_BUSY_STAGES 와 같은 값이어야 한다.
#   nav · push    로더로 들어가는 중
#   place         로더에 붙어서 내려놓는 중
#   stack_*       스택 자리와 검사 스테이션이 로더 곁이라, 그 사이 로더로
#                 들어가면 상대의 경로를 가로지른다
# ★ approach · wait · hold_back 은 넣지 말 것. 줄 서 있는 상태를 양보 대상으로
#   만들면 양쪽이 서로의 대기를 기다려 교착이다.
LANE_STAGES = ["nav", "push", "place",
               "stack_nav", "stack_scan", "stack_pick",
               "stack_deliver", "stack_place"]

# 상대가 누구인가. 세 대 이상이 되면 이 표로는 안 되고, 도크처럼 전역 조정
# 노드를 두는 편이 낫다(docs/02 §2 의 docking_server 논리와 같다).
PEER_OF = {"robot1": "robot2", "robot2": "robot1"}


def _task_manager_params(ns, namespaces):
    """task_manager 하나에 넘길 파라미터.

    ★ 모르는 네임스페이스는 거절한다. 검증된 좌표가 없는데 기본값(robot1 것)을
      물려 주면 두 로봇이 같은 자리로 가서 부딪힌다. 지도가 바뀌면서 실제로
      좌표 하나가 장애물 안으로 들어간 적이 있다(check_staging_poses.py 가
      그래서 있다).
    """
    if ns not in SCAN_ROUTE_BY_ROBOT:
        raise RuntimeError(
            f"{ns} 의 좌표가 없다 — SCAN_ROUTE_BY_ROBOT · ROW_EXIT_BY_ROBOT · "
            f"STAGING_BY_ROBOT · DOCK_ROUTE_BY_ROBOT · LANE_PRIORITY_BY_ROBOT 에 "
            f"넣고 check_staging_poses.py 로 검증해라. 지금 아는 로봇: "
            f"{sorted(SCAN_ROUTE_BY_ROBOT)}")

    params = {
        "scan_route": SCAN_ROUTE_BY_ROBOT[ns],
        # 대기 자리까지 가는 길. 우회의 APPROACH 가 그대로 쓰고, 직행의 NAV 는
        # 여기에 로더를 덧붙여 쓴다(task_manager.py DEFAULT_APPROACH_ROUTE).
        "approach_route": ROW_EXIT_BY_ROBOT[ns] + STAGING_BY_ROBOT[ns],
        "dock_route": DOCK_ROUTE_BY_ROBOT[ns],
        "lane_priority": LANE_PRIORITY_BY_ROBOT[ns],
    }

    peer = PEER_OF.get(ns)
    if not peer or peer not in namespaces:
        # 상대가 같이 안 뜨면 구독하지 않는다. 안 뜨는 토픽을 구독해 둬도
        # 동작은 같지만(한 번도 못 받으면 통과), 로그에서 헷갈린다.
        return params

    # ★ 절대이름이다. 상대이름으로 두면 자기 자신을 구독한다.
    params["peer_state_topic"] = f"/{peer}/orchestrator/state"
    params["peer_busy_stages"] = list(LANE_STAGES)
    # 상대가 도크를 떠났는지, 로더에서 얼마나 멀어졌는지 보려면 상대 위치가
    # 필요하다. 없어도 동작은 하지만(시간과 상태 변화로만 푼다) 그만큼 느리다.
    # Nav2 가 네임스페이스마다 amcl 을 띄우므로 이 토픽은 이미 나와 있다.
    params["peer_pose_topic"] = f"/{peer}/amcl_pose"
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
                        "(예: robot1,robot2). 아는 이름은 robot1 · robot2 뿐이고, "
                        "다른 이름을 주면 좌표가 없어서 거절한다."),
        OpaqueFunction(function=_setup),
    ])
