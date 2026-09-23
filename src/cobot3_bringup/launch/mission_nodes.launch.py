"""출발 -> 스캔 -> pick -> 주행 -> place -> (스택) -> 도킹 을 한 판 도는
애플리케이션 노드 4개를, 로봇 네임스페이스마다 한 벌씩 띄운다.

    ros2 launch cobot3_bringup mission_nodes.launch.py                       # robot1 한 대
    ros2 launch cobot3_bringup mission_nodes.launch.py robots:=robot1,robot2   # 두 대

★ task_manager 는 순찰 판이다 (main 머지 이후)
  트리는 START → POSE → PATROL 과 캐리어 처리 가지를 둘 다 들고 있고,
  웹 배차(ExecuteTask)·선반 설정 다시읽기도 살아 있다. 한때 이 브랜치에
  순찰을 들어낸 판이 있었지만 origin/main 쪽으로 정리됐다.

★ 아래 좌표표 중 지금 실제로 넘어가는 것은 STAGING_BY_ROBOT 뿐이다
  SCAN_ROUTE · ROW_EXIT · DOCK_ROUTE · LANE_PRIORITY 는 순찰 없는 판의
  task_manager 가 받던 파라미터다. 지금 task_manager 는 그 이름을
  declare_parameter 하지 않는다. rclpy 는 선언되지 않은 override 를
  예외도 경고도 없이 버리므로, 계속 넘기면 "좌표를 줬는데 왜 저기로 가지"
  를 추적할 수 없다. 그래서 넘기지 않고 값만 남겨 둔다 — 받는 쪽이 다시
  생기면 _task_manager_params 에서 되살려라.

★ patrol_route 는 일부러 안 넘긴다
  웹(「설정 > 선반」 → shelves.yaml 의 assigned_robot)이 주기로 한 값이다.
  그때까지 task_manager 는 DEFAULT_PATROL_ROUTE 로 폴백하고 시작 로그에
  "DEFAULT_PATROL_ROUTE 폴백" 경고를 찍는다 — 그 경고가 보이면 정상이다.

씬은 매거진 둘(로봇당 하나)과 스택 하나만 놓인 시험용을 쓴다:
    SIM_WORLD_USD=isaacpjt/worlds/simple_factory_layout_test.usda \\
        ./isaacpjt/ros_bridge/run_sim_backend.sh
  (isaac_python 으로 직접 띄우지 마라 — ROS 셸의 3.12 PYTHONPATH/LD_LIBRARY_PATH
   를 물려받아 SimulationApp 에서 죽는다. 이유는 run_sim_backend.sh 머리 주석.)

★ 로그는 tee 로 받아라
  노드가 import 단계에서 즉사하면(Traceback 한 장 찍고 exit code 1) 그
  Traceback 은 화면에만 뜬다. ~/.ros/log/<실행>/launch.log 에는
  "process has died ... exit code 1" 만 남고 이유는 안 남는다. output="both"
  로 바꿔도 이 시스템은 노드별 로그 파일을 만들지 않는다(확인함). 그래서:

      ros2 launch cobot3_bringup mission_nodes.launch.py robots:=robot1 \
          2>&1 | tee ~/mission.log

  실제로 carrier_code_reader · pick_place_server 가 이렇게 죽었을 때
  원인을 못 찾았다.

이 launch 가 책임지는 건 애플리케이션 노드뿐이다. 아래는 따로 띄워야 한다
(이 순서로):
    1) ./isaacpjt/ros_bridge/run_sim_backend.sh          # Isaac Sim + RPC 서버 (isaac_python 직접 X)
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

import os

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
#  robot2 는 같은 규칙을 Shelf_02(기본 씬에서 매거진이 북쪽 면이라 통로가
#  +y, yaw 180)와 자기 도크 줄(y=-7.575)에 적용한 것이다.
#
#  ★ 기준 씬은 simple_factory_layout.usda 다. 어느 긴 변이냐가 바뀌면 여기와
#    shelves.yaml 이 같이 뒤집혀야 한다 —
#    isaacpjt/tools/derive_patrol_from_scene.py 로 씬에서 직접 뽑아라.
#
#  ★ 실측이 아니다. 선반 앞 정차 관계(전면에서 0.4167 m)와 nav_server 의 직선
#    주행·후진 규칙에서 기하로 계산한 뒤, 점유격자 위에서 차체 사각형을 실제로
#    놓아 정차점·구간·회전 여유를 확인했다. Isaac 에서 한 번 돌려 보고
#    부딪히는 점이 있으면 여기서 고친다.
#
#  ★ 스택·검사·로더이탈 경로는 로봇마다 다르지 않아서 여기서 안 넘긴다 —
#    task_manager.py 의 기본값을 그대로 쓴다. 스택은 하나뿐이고 한 대만
#    맡으므로 둘이 같은 좌표를 써도 겹치지 않는다.
#
#  ★ 지금 task_manager 에게 실제로 넘어가는 것은 STAGING_BY_ROBOT 뿐이다.
#    나머지 표는 받는 쪽이 없다 — 위 머리주석의 두 번째 ★ 참고.
# ══════════════════════════════════════════════════════════════════════════

# 도크 → 스캔 자리. 자기 줄을 따라 서쪽으로 나간 뒤 선반 줄 동쪽 끝으로
# 올라가고, 마지막 구간은 회전 없이 들어간다(robot1 후진 · robot2 전진).
#
# ★ 스캔 정차 x 는 두 로봇이 같다(SCAN_STOP_X). 줄(y)만 다르고 매거진이
#   놓인 슬롯은 두 선반에서 같은 x 자리다. 실측으로 매거진 x(-0.641)에
#   그대로 세우면 차체가 도크 반대쪽으로 치우쳐서, 도크 방향(+x)으로
#   0.20 m 물렸다. 값의 출처는 task_manager.py 의 SCAN_STOP_X 이고,
#   여기서 다시 적는 건 런치가 task_manager 기본값을 덮어쓰기 때문이다 —
#   한쪽만 고치면 안 된다.
SCAN_STOP_X = -0.441

# ☞ 아래 표는 지금 task_manager 에 안 넘어간다 (scan_route 를 declare 하는
#   쪽이 없다). 로봇 이름 검증과 값 보관에만 쓴다 — 자세한 사정은 이 파일
#   머리주석의 두 번째 ★ 참고.
SCAN_ROUTE_BY_ROBOT = {
    "robot1": [3.5, -6.591, 180.0,   1.0, 2.036, 0.0,     SCAN_STOP_X, 2.036, 0.0],
    "robot2": [3.5, -7.575, 180.0,   1.0, -2.8827, 0.0,   SCAN_STOP_X, -2.8827, 0.0],
}

# 스캔 자리에서 선반 줄을 벗어나는 동쪽 이탈점. 이 점 없이 바로 북상하면
# 차체가 선반을 친다. 아래 대기 자리와 이어 붙여 approach_route 가 된다.
# ☞ 지금은 안 넘어간다 (받는 파라미터가 없다).
ROW_EXIT_BY_ROBOT = {
    "robot1": [1.50, 2.036, 0.0],
    "robot2": [1.50, -2.8827, 0.0],
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
# ☞ 지금은 안 넘어간다 (받는 파라미터가 없다).
DOCK_ROUTE_BY_ROBOT = {
    "robot1": [3.0, -4.6, 90.0,   3.0, -6.3, 180.0,     5.5, -6.591, 180.0],
    "robot2": [3.0, -4.6, 90.0,   3.0, -7.575, 180.0,   5.5, -7.575, 180.0],
}

# 로더 차선 순서. 1 이 먼저다. 남쪽 로봇(robot2)이 먼저 나가야 북쪽 로봇의
# 꼬리 스윕이 이웃을 안 치므로(task_manager.py "출발 게이트") robot2 가 1 이다.
# 그 순서가 로더에서도 이어져 robot2 가 먼저 place 하고 스택을 맡는 것이
# 기본 흐름이다.
# ☞ 지금은 안 넘어간다 (받는 파라미터가 없다).
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

# ══════════════════════════════════════════════════════════════════════════
#  스택 회수 — 누가 맡나
#
#  매거진을 로더에 놓은 뒤, 같은 사이클 안에서 포장 스테이션 산출물(스택)을
#  집어 로더로 가져간다. 값은 shelves.yaml 의 shelf_id 다 — 좌표와 관측 자세
#  (arm_teach_pose)가 그 항목에 같이 붙어 있어서 id 로 가리키면 둘이 같이
#  따라온다.
#
#  ★ 스택은 씬에 하나뿐이라 한 대만 맡는다. 둘 다 주면 같은 자리로 간다.
#    지금은 robot1 이 맡고 robot2 는 매거진만 돈다.
#
#  ★ 빈 문자열이면 그 로봇의 트리에 스택 구간이 아예 안 들어간다.
#
#  ☞ 이건 임시 배선이다. 제대로 된 회수는 place 완료 → pending_pickup →
#    ready_at 도래 → RECOVER 작업 배차이고, 그 고리는 web/backend 의
#    pickup.py 에 이미 있다. 다만 task_manager 에 RECOVER 미션이 없고
#    PORT_BY_STAGE 가 stations.yaml 에 없는 "test_loader" 를 보내서 아직
#    안 이어진다. 그때가 되면 이 파라미터는 지운다.
# ══════════════════════════════════════════════════════════════════════════

STACK_SHELF_BY_ROBOT = {"robot1": "PKG-OUT", "robot2": ""}


# ══════════════════════════════════════════════════════════════════════════
#  nav_server 파라미터
#
#  순찰 goal 하나를 받을 때마다, 바퀴를 굴리기 전에 이만큼 제자리에 선다.
#  그 사이 carrier_code_reader 가 팔을 관측 자세로 올린다 — 안 세우면 팔이
#  올라가는 15 초 동안 베이스가 1.8 m 를 가서 선반 구역을 지나쳐 버린다.
#  (근거는 nav_server.py DEFAULT_PATROL_START_HOLD_S 주석)
#
#  ★ 이 정지는 순찰 왕복 편도(약 27 s)에 그대로 더해진다. 줄이려면 여기서
#    줄이되, carrier_code_reader 의 observe_pose 블로킹 한도(15 s)보다 짧으면
#    팔이 올라가는 중에 다시 굴러가서 원래 문제로 돌아간다.
#  ★ 0 으로 두면 이 단계를 끈다.
# ══════════════════════════════════════════════════════════════════════════

PATROL_START_HOLD_S = 15.0


def _nav_server_params():
    return {"patrol_start_hold_s": PATROL_START_HOLD_S}


# ══════════════════════════════════════════════════════════════════════════
#  carrier_code_reader 파라미터
#
#  순찰 중 QR 폴링 — 곧 "주행 중에 팔을 관측 자세로 올릴지" 다.
#
#  ★ 끌 수 있게 해 둔 이유(기본은 켬): 팔이 충돌 계산에 안 들어간다. Nav2 코스트맵이 아는
#    차체는 footprint(앞으로 0.14 m)뿐이고, 순찰은 collision_monitor 도
#    거치지 않는다(nav_server 가 cmd_vel 에 직접 쓴다). 그런데 폴백 관측
#    자세(taught_poses.yaml 의 s1_bottom_scan)는 J2=90° 라 팔이 앞으로 거의
#    다 뻗는다. 선반 앞 여유가 0.40~0.45 m 인 통로에서 그러면 박는다.
#
#  ★ 더 근본적으로, 그 폴백 자세들은 **옛 레이아웃에서 티칭된 값**이다.
#    taught_poses.yaml 의 base_link_world 가 x≈-6.5 인데 지금 선반은
#    x[-2.825,-0.325] 다. 선반을 정면으로 마주보던 시절 자세라, 선반 옆을
#    따라 지나가는 지금 구조에는 기하가 맞지 않는다. Shelf_02 용 자세는
#    아예 없어서 robot2 도 shelf_1 자세를 쓴다.
#
#  ★ 켜려면: 먼저 지금 레이아웃에서 두 선반의 관측 자세를 다시 티칭해
#    shelves.yaml 의 arm_teach_pose 6 칸을 채워라. 채워지면
#    carrier_code_reader 가 폴백 대신 그 값을 쓴다(_teach_joints_deg).
#    자세가 안 맞은 채로 순찰하다 팔이 박으면, 원인을 가르기 위해
#    여기를 False 로 두고 베이스 주행만 따로 볼 수 있다.
# ══════════════════════════════════════════════════════════════════════════

PATROL_SCAN = True

#  ★ 감지를 "QR 이 보인다" 가 아니라 "QR 옆에 왔다" 로 좁히는 허용오차(m).
#    손목캠은 매거진을 한참 앞에서부터 비스듬히 본다. 보이자마자 멈추면
#    매거진 정면이 아니라 비스듬히 먼 자리에 서고, 팔이 거기까지 못 뻗어
#    pick 이 NO_FLANGE 로 죽는다(실측: 진행방향 0.45 m 못 미쳐 정차,
#    매거진까지 0.76 m — 티칭 0.55).
#    base_link +x(로봇 정면) 오프셋이 이 값 안에 들어와야 감지로 친다.
#    0 이하면 게이트를 끈다.
DETECT_ALIGN_TOL_M = 0.15


def _carrier_code_reader_params():
    return {"patrol_scan": PATROL_SCAN,
            "detect_align_tol_m": DETECT_ALIGN_TOL_M}


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

    # ★ 여기 넣는 이름은 task_manager 가 declare_parameter 하는 것뿐이어야 한다.
    #   rclpy 는 선언 안 된 override 를 조용히 버린다 — 예외도 경고도 없다.
    #   그래서 오타나 "받는 쪽이 사라진 이름" 이 여기 남아 있으면, 좌표를 준 줄
    #   알고 있는데 노드는 기본값으로 도는 상태가 되고 로그에 아무 단서도 없다.
    #   지금 task_manager 가 선언하는 이름(task_manager.py __init__ 참고):
    #       shelves_yaml · patrol_shelf · patrol_route · reload_service
    #       execute_task_action · wait_for_task · empty_sweeps
    #       observe_pose_service · peer_state_topic · peer_pose_topic
    #       loader_clear_radius_m · peer_busy_stages · peer_camera_stages · staging_pose
    params = {
        # 로더 차선이 막혔을 때 비켜 서는 자리. 로봇마다 달라야 한다 —
        # 같은 점을 쓰면 대기 자리에서 둘이 부딪힌다(STAGING_BY_ROBOT 주석).
        "staging_pose": STAGING_BY_ROBOT[ns],
        # 스택을 맡을 로봇만 값이 있다 (STACK_SHELF_BY_ROBOT 주석 참고).
        "stack_shelf": STACK_SHELF_BY_ROBOT.get(ns, ""),
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
            if executable == "task_manager":
                params = _task_manager_params(ns, namespaces)
            elif executable == "nav_server":
                params = _nav_server_params()
            elif executable == "carrier_code_reader":
                params = _carrier_code_reader_params()
            else:
                params = {}
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
    #
    # ★ event_logger:=false 는 이 launch 를 두 번 띄울 때 쓴다.
    #   도크가 0.98 m 간격인데 회전 꼬리 스윕이 0.656 m 라, 두 대를 한 번에
    #   띄우면 동시에 도크를 떠나면서 서로 침범한다. 이 판 task_manager 에는
    #   출발 조율(lane_priority·출발 게이트)이 없어서 코드가 막아 주지 않는다.
    #   그래서 robot1 을 먼저 띄워 도크를 벗어나게 한 뒤 robot2 를 띄우는데,
    #   그때 둘째 launch 까지 event_logger 를 띄우면 같은 이름의 노드가 둘이
    #   되어 /trace/event 를 양쪽이 받는다. 둘째에 false 를 준다.
    if _as_bool(LaunchConfiguration("event_logger").perform(context)):
        # ★ db_dsn 을 여기서 명시적으로 넘긴다 — 안 그러면 event_logger 의
        #   declare_parameter 기본값(os.environ.get("COBOT3_DB_DSN", ""))이
        #   이 launch 를 실행한 셸의 환경에만 달린다. 실측: 그 셸에
        #   COBOT3_DB_DSN 을 export 한 적이 없어서 이 노드가 계속 빈 DSN으로
        #   떠 있었고, SCAN이 성공해 run이 생기고 pick·nav 단계까지 지나도
        #   DB(magazine_log/stack_log/carrier_log)에 행이 하나도 안 쌓였다
        #   (docs/DB구성.md §9 배선 자체는 정상 — /trace/event 구독도 붙어
        #   있었다. 그냥 이 노드가 어느 DB 로 쓸지를 몰랐던 것뿐이다).
        #
        #   웹 백엔드(FastAPI, COBOT3_DSN)가 다른 머신에서 돌아도 DB는 보통
        #   하나를 같이 본다 — 그 DSN과 같은 값을 여기 db_dsn launch 인자로
        #   줘야 한다. 기본값은 COBOT3_DB_DSN 환경변수를 그대로 물려받는다
        #   (전에 export 해 둔 셸이면 그대로 동작) — 그것도 없으면 빈 문자열
        #   이고, event_logger 는 그 경우 자기 로그로 크게 경고하고 스풀
        #   파일로만 흘린다(원인은 알 수 있어도 DB 에는 안 쌓인다).
        db_dsn = LaunchConfiguration("db_dsn").perform(context)
        if not db_dsn:
            print("   !! db_dsn 이 비었다 — event_logger 가 스풀 파일로만 흘린다. "
                  "-p db_dsn:=postgresql://... 로 주거나 COBOT3_DB_DSN 을 export 해라.")
        nodes.append(Node(
            package="cobot3_orchestrator",
            executable="event_logger",
            name="event_logger",
            output="screen",
            parameters=[{"db_dsn": db_dsn}],
        ))
    return nodes


def _as_bool(text):
    return str(text).strip().lower() not in ("0", "false", "no", "off", "")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "event_logger", default_value="true",
            description="전역 event_logger 를 이 launch 가 띄울지. 이 launch 를 "
                        "두 번 나눠 띄울 때(로봇 시차 출발) 둘째에 false 를 줘서 "
                        "같은 이름의 노드가 둘이 되는 것을 막는다."),
        DeclareLaunchArgument(
            "robots", default_value="robot1",
            description="미션 노드를 띄울 로봇 네임스페이스. 쉼표로 여러 개 "
                        "(예: robot1,robot2). 아는 이름은 robot1 · robot2 뿐이고, "
                        "다른 이름을 주면 좌표가 없어서 거절한다."),
        DeclareLaunchArgument(
            "db_dsn", default_value=os.environ.get("COBOT3_DB_DSN", ""),
            description="event_logger 가 쓸 PostgreSQL DSN. 기본값은 이 launch 를 "
                        "실행한 셸의 COBOT3_DB_DSN 환경변수다. 웹 백엔드가 다른 "
                        "머신에서 돌아도(COBOT3_DSN) 보통 같은 DB 하나를 보므로 "
                        "그 값과 같아야 한다. 비우면 event_logger 가 전부 스풀 "
                        "파일(runs/trace_spool.jsonl)로만 흘리고 DB에는 안 쌓인다."),
        OpaqueFunction(function=_setup),
    ])
