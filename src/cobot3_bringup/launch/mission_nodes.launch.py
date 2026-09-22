"""주행 -> 스캔 -> pick -> 주행 -> place 미션을 도는 애플리케이션 노드 4개를,
로봇 네임스페이스마다 한 벌씩 띄운다.

    ros2 launch cobot3_bringup mission_nodes.launch.py                      # robot1 한 대
    ros2 launch cobot3_bringup mission_nodes.launch.py robots:=robot1,robot2  # 두 대
    # 순찰 없는 고정 시나리오 (씬: worlds/simple_factory_layout_test.usda)
    ros2 launch cobot3_bringup mission_nodes.launch.py robots:=robot1,robot2 scenario:=static_test

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

★ scenario:=static_test — 순찰 없는 고정 시나리오
  task_manager.py 모듈 독스트링 "순찰 없는 고정 시나리오" 가 트리를 설명한다.
  여기서는 로봇별 좌표와 순서만 준다(아래 STATIC_* 표). 씬은
      SIM_WORLD_USD=isaacpjt/worlds/simple_factory_layout_test.usda \
          isaac_python isaacpjt/ros_bridge/sim_backend.py
  로 띄운다 — 매거진 둘(선반 동쪽 끝 슬롯, 로봇당 하나)과 스택 하나(포장
  출력 선반)가 처음부터 놓여 있다. 좌표의 근거는 task_manager.py 의
  DEFAULT_SCAN_ROUTE 등 상수 주석에 있고, 실측이 아니라 기하 계산이다.
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
#  ★ 두 로봇이 똑같은 규칙을 쓴다. 상대가 차선을 쓰고 있으면 대기 자리로
#    가서 기다리고, 아니면 바로 간다. 순서를 정하는 장치는 없다 — 둘이 같은
#    순간에 대기 자리를 떠나면 차선에서 만난다. task_manager.py 의 "알려진 갭"
#    참고.
#
# 상대가 차선을 쓰고 있다고 보는 단계. task_manager.py 의
# DEFAULT_PEER_BUSY_STAGES 와 같은 값이어야 한다.
#   nav     로더로 바로 가는 중
#   push    대기 자리에서 로더로 들어가는 중. 이름만 다른 "목적지로 가는 중" 이다
#   place   로더에 붙어서 내려놓는 중
#   return  로더에서 후진으로 나오는 중. 이탈선이 접근선과 같은 선이라,
#           이걸 빼면 대기하던 로봇이 출발하는 순간 정면으로 만난다
# ★ 'approach' 와 'wait' 은 넣지 말 것. 둘 다 차선 밖이고, 넣으면 양쪽이 서로의
#   대기를 기다려 교착이다.
LANE_STAGES = ["nav", "push", "place", "return"]

# ★ 대기 자리는 로봇마다 달라야 한다 — 같은 점을 쓰면 대기 자리에서 부딪힌다.
#   세 조건(자유공간 · 도착각 상한 · 이탈 경로 회피)을 전부 통과한 값이다.
#   지도를 새로 만들면 check_staging_poses.py 로 다시 검증해라.
STAGING_BY_ROBOT = {
    "robot1": [1.0, 5.25, 0.0],    # 도착각 +7.5도 · 로더 3.08 m · 이탈여유 0.32 m
    "robot2": [1.0, 5.25, 0.0],    # 도착각 +0.0도 · 로더 4.65 m · 이탈여유 0.32 m
}

# 상대가 누구인가. 세 대 이상이 되면 이 표로는 안 되고, 도크처럼 전역 조정
# 노드를 두는 편이 낫다(docs/02 §2 의 docking_server 논리와 같다).
PEER_OF = {"robot1": "robot2", "robot2": "robot1"}


# ══════════════════════════════════════════════════════════════════════════
#  scenario:=static_test — 순찰 없는 고정 시나리오의 로봇별 표
#
#  좌표는 전부 (x, y, yaw_deg) 를 평탄화한 경로다. 값의 근거는 task_manager.py
#  의 DEFAULT_SCAN_ROUTE · DEFAULT_DOCK_ROUTE · DEFAULT_STACK_* 주석(robot1 값이
#  그 기본값과 같다 — test_peer_yield.py 가 검사한다). robot2 는 같은 규칙을
#  Shelf_02(통로가 +y 라 yaw 180)와 자기 도크 줄(y=-7.575)에 적용한 것이다.
#
#  ★ 실측이 아니다. 선반 앞 정차 관계(전면에서 0.4167 m)와 nav_server 의
#    직선 주행·후진 규칙에서 기하로 계산했다. Isaac 에서 한 번 돌려 보고
#    부딪히는 점이 있으면 여기서 고친다.
# ══════════════════════════════════════════════════════════════════════════

# 도크 → 스캔 자리. 자기 줄을 따라 서쪽으로 나간 뒤(회전 없음) 선반 줄 동쪽
# 끝으로 올라가 마지막 구간은 회전 없이 들어간다(robot1 후진 · robot2 전진).
STATIC_SCAN_ROUTE_BY_ROBOT = {
    "robot1": [3.5, -6.591, 180.0,   1.0, 2.036, 0.0,     -0.641, 2.036, 0.0],
    "robot2": [3.5, -7.575, 180.0,   1.0, -1.056, 180.0,  -0.516, -1.056, 180.0],
}

# 로더/검사 스테이션 → 도크. 마지막 점이 도크(씬의 시작 자세와 같다). 둘째
# 점은 자기 줄 위이고, 이웃 줄과 0.98 m 떨어져 있어 거기서 도는 꼬리 스윕
# (0.70 m)이 이웃을 안 친다 — robot1 은 -6.3(이웃이 남쪽), robot2 는 자기 줄.
STATIC_DOCK_ROUTE_BY_ROBOT = {
    "robot1": [3.0, -4.6, 90.0,   3.0, -6.3, 180.0,     5.5, -6.591, 180.0],
    "robot2": [3.0, -4.6, 90.0,   3.0, -7.575, 180.0,   5.5, -7.575, 180.0],
}

# 로더 차선 순서. 1 이 먼저다. 남쪽 로봇(robot2)이 먼저 나가야 북쪽 로봇의
# 꼬리 스윕이 이웃을 안 치므로(task_manager.py "출발 게이트") robot2 가 1 이다.
# 그 순서가 로더에서도 그대로 이어져 robot2 가 먼저 place 하고 스택을 맡는
# 것이 기본 흐름이다.
LANE_PRIORITY_BY_ROBOT = {"robot1": 2, "robot2": 1}

# static_test 의 양보 목록. task_manager.py 의 STATIC_PEER_BUSY_STAGES 와 같은
# 값이어야 한다. 순찰의 LANE_STAGES 에 스택 다섯 단계를 더한 것 — 스택 자리와
# 검사 스테이션이 로더 곁이라 상대의 스택 사이클 내내 기다린다.
STATIC_LANE_STAGES = LANE_STAGES + [
    "stack_nav", "stack_scan", "stack_pick", "stack_deliver", "stack_place"]


def _task_manager_params(ns, namespaces, scenario="patrol"):
    """task_manager 하나에 넘길 파라미터. 조율이 필요 없으면 빈 dict."""
    static = scenario == "static_test"
    params = {"scenario": scenario} if static else {}
    if static:
        if ns not in STATIC_SCAN_ROUTE_BY_ROBOT:
            raise RuntimeError(
                f"static_test 에 {ns} 의 좌표가 없다 — STATIC_SCAN_ROUTE_BY_ROBOT · "
                f"STATIC_DOCK_ROUTE_BY_ROBOT · LANE_PRIORITY_BY_ROBOT 에 넣어라")
        params.update({
            "scan_route": STATIC_SCAN_ROUTE_BY_ROBOT[ns],
            "dock_route": STATIC_DOCK_ROUTE_BY_ROBOT[ns],
            "lane_priority": LANE_PRIORITY_BY_ROBOT[ns],
        })

    # ★ 모르는 네임스페이스에는 조율을 켜지 않는다. 검증된 대기 자리가 없는데
    #   아무 좌표나 기본값으로 물려 주면, 그 자리가 장애물 안이어도 로봇이
    #   그리로 간다. 지도가 바뀌면서 실제로 좌표 하나가 장애물 안으로 들어간
    #   적이 있다(check_staging_poses.py 가 그래서 있다). 좌표가 없으면 조율
    #   없이 직행만 하게 두는 편이 안전하다.
    if ns not in STAGING_BY_ROBOT:
        return params

    params["staging_pose"] = STAGING_BY_ROBOT[ns]
    peer = PEER_OF.get(ns)
    if not peer or peer not in namespaces:
        # 상대가 같이 안 뜨면 구독하지 않는다. 안 뜨는 토픽을 구독해 둬도
        # 동작은 같지만(한 번도 못 받으면 통과), 로그에서 헷갈린다.
        return params

    params["peer_state_topic"] = f"/{peer}/orchestrator/state"
    params["peer_busy_stages"] = list(STATIC_LANE_STAGES if static else LANE_STAGES)
    # 상대가 로더에서 얼마나 멀어졌는지 보려면 상대 위치가 필요하다. 없어도
    # 동작은 하지만(상대의 return 이 끝날 때까지 기다린다) 그만큼 느리다.
    # Nav2(multi_navigation.launch.py)가 네임스페이스마다 amcl 을 띄우므로
    # 이 토픽은 이미 나와 있다.
    params["peer_pose_topic"] = f"/{peer}/amcl_pose"
    return params


def _setup(context):
    raw = LaunchConfiguration("robots").perform(context)
    namespaces = [ns.strip().strip("/") for ns in raw.split(",") if ns.strip()]
    if not namespaces:
        raise RuntimeError(
            "robots 인자가 비었다 — 예: robots:=robot1 또는 robots:=robot1,robot2")
    scenario = LaunchConfiguration("scenario").perform(context).strip()
    if scenario not in ("patrol", "static_test"):
        raise RuntimeError(f"scenario:={scenario} — patrol 또는 static_test 여야 한다")

    nodes = []
    for ns in namespaces:
        for package, executable in MISSION_NODES:
            params = (_task_manager_params(ns, namespaces, scenario)
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
        DeclareLaunchArgument(
            "scenario", default_value="patrol",
            description="task_manager 의 트리. patrol(순찰) 또는 static_test"
                        "(순찰 없는 고정 시나리오 — 모듈 독스트링 ★ 참고)."),
        OpaqueFunction(function=_setup),
    ])
