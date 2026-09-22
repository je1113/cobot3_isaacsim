"""
task_manager — orchestrator 행동트리(Behavior Tree). 순찰 없는 고정 시나리오.

    ros2 run cobot3_orchestrator task_manager

로봇 두 대가 도크에서 출발해 아래를 **한 번** 돌고 도크로 돌아와 선다.
씬은 isaacpjt/worlds/simple_factory_layout_test.usda — 매거진 둘(로봇당 하나,
각자의 스캔 자리 바로 앞)과 스택 하나(포장 출력 선반)가 처음부터 놓여 있다.

★ 이 판에는 순찰이 없다
  이전 판은 "순찰하다 QR 이 보이면 처리한다" 였고, 그 가지에 웹 작업 지시
  (ExecuteTask)·선반 설정 다시읽기(config/reload)·순찰 정차점 관리가 전부
  매달려 있었다. 이 브랜치는 순찰 없이 고정 좌표로 한 판 도는 것만 시험하므로
  그 전부를 들어냈다. 지워진 것:
      순찰 가지(START 1회 · POSE · 순찰 잎)와 RETURN
      carrier_detected 구독과 Detected · Hold 잎 (감지로 멈추는 경로)
      shelves.yaml 순찰 경로 해석 · config/reload 서비스
      ExecuteTask 액션 서버 (웹 배차)
      observe_pose 서비스 호출 — 관측 자세는 carrier_scan 이 스스로 잡는다
  남은 것은 좌표를 따라가는 주행, SCAN·PICK·PLACE, 로더 차선 조율, 생산
  트래킹(TraceEvent), 그리고 실패 시 정지와 /orchestrator/resume 이다.

행동트리가 뭔가 — 이 파일을 읽는 데 필요한 만큼만
  트리를 일정 주기로 뿌리부터 "tick" 한다(여기서는 10 Hz). tick 을 받은 노드는
  SUCCESS / FAILURE / RUNNING 셋 중 하나를 돌려준다. RUNNING 이 핵심이다 —
  "아직 하는 중" 을 상태 변수에 저장하는 대신 잎이 tick 마다 RUNNING 을 돌려주는
  것으로 표현한다. 그래서 어떤 잎도 블로킹하면 안 된다.

    Sequence [→]  왼쪽부터. SUCCESS 면 다음, 하나라도 FAILURE 면 즉시 FAILURE.
    Selector [?]  왼쪽부터. FAILURE 면 다음, SUCCESS/RUNNING 이면 거기서 멈춤.

이 노드의 트리

    [→] 고정 시나리오                Sequence, memory=True — 한 번만 돈다
     ├─ START (게이트)               후순위 로봇은 선순위 상대가 도크 곁을
     │                               떠날 때까지 기다린다 (WaitPeerAway)
     ├─ START                        scan_route 를 따라 스캔 자리로
     ├─ SCAN                         그 자리에서 바로 carrier_scan
     ├─ PICK
     ├─ [?] 배송                     Selector, memory=True
     │   ├─ [→] 직행                 상대가 차선을 안 쓰면 이쪽
     │   │   ├─ 상대 한가?           아래 "순서를 정하는 기준"
     │   │   └─ NAV                  approach_route + 로더
     │   └─ [→] 우회                 상대가 쓰고 있으면 이쪽
     │       ├─ HOLD_BACK            스캔 자리에서 — 상대가 로더 구역에서
     │       │                       움직이는 동안은 올라가지 않는다
     │       ├─ APPROACH             approach_route (대기 자리까지)
     │       ├─ WAIT                 대기 자리에서 — 로더가 빌 때까지
     │       └─ PUSH                 로더로
     ├─ PLACE
     ├─ 매거진 완료
     ├─ [?] 다음 일                  Selector, memory=True
     │   ├─ [→] 스택                 먼저 place 한 쪽만 들어온다 (StackFirst)
     │   │   ├─ STACK_NAV            stack_pick_route 로 스택 앞에
     │   │   ├─ STACK_SCAN           스택 QR (F3-STKO-1 → tray_1_orange)
     │   │   ├─ STACK_PICK
     │   │   ├─ STACK_DELIVER        stack_deliver_route 로 검사 스테이션에
     │   │   ├─ STACK_PLACE
     │   │   └─ 스택 완료
     │   └─ [→] 나중                 loader_exit_route (로더에서 서쪽으로)
     ├─ DOCK                         dock_route (마지막 점이 도크)
     └─ DONE                         영원히 RUNNING. state=done

  ★ 최상위가 Sequence(memory=True) 다 — 선점 Selector 가 없다. 순찰이 없으니
    "지금 하던 일을 끊고 다른 가지로" 가 필요한 경우가 없다. 한 번 시작한
    단계는 끝나거나 얼어붙을 때까지 그 자리에 있는다.

  ── 순서를 정하는 기준: lane_priority (1 이 가장 먼저) ──────────────────
  로더는 하나이고 두 대가 거의 같은 순간에 PICK 을 마친다. "상대가 지금 차선을
  쓰는가" 만 보면 둘 다 "안 쓴다" 로 읽고 같이 출발한다. 그래서 상대가 아직
  차선 판단 전(start · scan · pick, CONTENDING_STAGES)인데 나보다 선순위면
  후순위 쪽이 우회로 간다. 선순위는 상대가 우회(hold_back · approach · wait)에
  있는 것을 보고 직행한다.
  ★ 교착이 안 생기는 근거는 "양보는 한쪽만" 이다 — 우선순위는 전순서라 둘이
    서로를 선순위로 볼 수 없다. 한쪽이라도 prio 가 없으면(0) 이 장치는 꺼지고
    "지금 차선을 쓰는가" 만으로 판단한다.
  우선순위는 /orchestrator/state 의 prio= 토큰으로 서로 알린다.

  ── 양보 목록에 스택 단계가 들어 있는 이유 ──────────────────────────────
  스택 자리(포장 출력 선반)와 검사 스테이션이 로더 곁이라, 상대가 스택을
  나르는 동안 로더로 들어가면 nav_server(회피 없음)가 그 경로를 가로지른다.
  그래서 PEER_BUSY_STAGES 에 스택 단계가 전부 들어 있고, 후순위 로봇의 WAIT 는
  상대가 dock 으로 넘어간 뒤에야 풀린다.
  ★ approach · wait · hold_back 은 넣지 않는다 — 줄 서 있는 상태를 양보
    대상으로 만들면 양쪽이 서로의 대기를 기다린다. isaacpjt/tools/
    test_peer_yield.py 가 그걸 검사한다.

  ── 출발 게이트가 필요한 이유 ───────────────────────────────────────────
  두 로봇이 도크에 0.98 m 간격으로 나란히 서 있는데 base_link 뒤쪽 끝이
  0.607 m 라 제자리회전 스윕이 0.656 m 다. 북쪽 로봇(robot1)이 먼저 돌면
  꼬리가 남쪽 로봇을 친다. 후순위 로봇은 선순위 상대가 자기 도크에서
  start_clearance_m 밖으로 나갈 때까지 서 있는다(상대 위치는 peer_pose_topic).
  상대 위치를 못 받으면 START_PEER_GRACE_S 만 기다려 보고 출발한다.

상태 저장
  "지금 어느 단계인가" 는 저장하지 않는다. 어느 잎이 RUNNING 인지가 답이고,
  트리를 찍으면 그게 실제다.

  "이번 미션의 데이터" 는 블랙보드(py_trees 의 공유 저장소)에 둔다.
      kind         QR 원문. 이 씬에서는 "1"/"2"(매거진) 또는 "F3-STKO-1"(스택)
      variant      kind 를 grasp.yaml/place.yaml 키로 푼 것
      carrier_id   QR 원문 그대로 = 개체 ID. 로그·표시용
      run_id       미션 1회를 묶는 UUID. SCAN 성공 때 발행한다
      qr_pose      PickCarrier 가 플랜지를 찾을 탐색 창 prior
      fail_stage   어느 단계에서 멈췄나
      fail_reason  왜 멈췄나

사용하는 인터페이스
  호출  perception/carrier_scan      CarrierScan.srv
  액션  navigation/navigate_to       NavigateTo.action
  액션  manipulation/pick_carrier    PickCarrier.action
  액션  manipulation/place_carrier   PlaceCarrier.action
  발행  orchestrator/state           std_msgs/String
  구독  <peer_state_topic>           std_msgs/String  ★ 절대이름
  구독  <peer_pose_topic>            PoseWithCovarianceStamped ★ 절대이름
  발행  /trace/event                 TraceEvent.msg   ★ 절대이름
  서비스 orchestrator/resume         std_srvs/SetBool (얼어붙은 단계 재시도)

★ 이름 앞에 / 가 없는 것은 전부 상대이름이고, 노드가 뜬 네임스페이스가 앞에
  붙는다. robot1 로 띄우면 /robot1/navigation/navigate_to 가 된다. 로봇별 값
  (좌표 · 우선순위 · 상대 토픽)은 mission_nodes.launch.py 의 표에서 온다.
  아래 DEFAULT_* 는 robot1 값이다.

★ 좌표는 실측이 아니라 기하 계산이다
  선반 앞 정차 관계(전면에서 0.4167 m, frames.yaml)와 nav_server 의 직선
  주행·후진 규칙에서 뽑았고, 점유격자 위에서 차체 사각형을 실제로 놓아
  정차점·구간·회전 여유를 확인했다. Isaac 에서 한 번 돌려 보고 어긋나면
  launch 의 표에서 고친다.

실패하면
  그 자리에서 정지한다(Freeze). 자동 복귀도 재시도도 하지 않는다. 어느
  단계에서 왜 멈췄는지는 에러 로그와 /orchestrator/state 에 남고,
  /orchestrator/resume 에 true 를 보내면 그 단계부터 다시 시도한다.
  ★ SCAN 도 얼린다. 순찰이 없어서 "못 읽었으니 다음 바퀴에 다시" 가 없다.

알려진 갭
  - 스택 관측 자세가 따로 없다. carrier_scan 을 받는 쪽(carrier_code_reader)
    이 매거진용 자세로 폴백하는데, 스택 QR 은 40 mm 이고 매거진보다 0.11 m
    낮게 붙어 있어 화면에서 80 px 남짓이다(디코드 실측 하한 70, 권장 100).
    STACK_SCAN 이 가장 먼저 멈출 자리다.
  - 스택 pick/place 는 grasp.yaml/place.yaml 의 tray_* 항목을 쓰는데 그 값이
    아직 미실측이다(에셋 기하에서 계산).
  - 배터리·도킹 선점 가지는 없다. DOCK 은 그냥 좌표 주행이다.
  - dock 단계는 DB 에 기록하지 않는다(docs/DB구성.md 미결 #2).
"""

import math
import sys
import time
import uuid
from pathlib import Path

import py_trees
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from py_trees.common import Access, Status
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool

from cobot3_interfaces.action import NavigateTo, PickCarrier, PlaceCarrier
from cobot3_interfaces.msg import TraceEvent
from cobot3_interfaces.srv import CarrierScan

# ══════════════════════════════════════════════════════════════════════════
#  좌표
#
#  형식은 전부 (x, y, yaw_deg) 를 평탄화한 것이다.
#      x, y      map 프레임 기준 위치, 단위 m
#      yaw_deg   그 자리에서 로봇이 바라볼 방향, 단위 도. +x 가 0도, 반시계 +
#  ★ 평탄화해서 받는 이유: rclpy 파라미터에 double[][] 타입이 없어서 중첩
#    리스트를 못 넘긴다. 세 개씩 끊어 읽는다(_unflatten_route).
# ══════════════════════════════════════════════════════════════════════════

# place 하러 갈 목적지 — 포장 투입 컨베이어 앞 주차 위치.
#
# ★ 2026-09-22: y 를 0.0 에서 4.60 으로 옮겼다. 씬의 PackagingZone 프림에
#   xformOp:translate = (0, 4.6, 0) 이 걸려 있어 포장 투입 벨트가 월드
#   y 4.05~5.15 에 있는데, 이 상수는 그 이동 전 값(로컬 y=0)이라 그대로
#   남아 있었다. 그래서 place 목표 (4.10, 0.00, 0.600) 이 허공이었고 매거진이
#   벨트가 아니라 바닥으로 떨어졌다. sim_backend 주석의 "ConveyorFrame
#   (x>=4.2) 바로 앞" 도 그 프림의 **로컬** x 를 가리킨 것이다.
#
# 지금 값의 근거 (전부 씬에서 직접 잰 값이다)
#   포장 투입 벨트  x 4.05~8.75, y 4.05~5.15, 윗면 z 0.5407, +x 로 0.2 m/s
#   x=3.75 인 이유  점유격자에서 이 자리의 제자리회전 여유가 25도다. 3.80
#                   이면 대기 자리에서 들어오는 각(13.7도)에서 차체 앞모서리가
#                   벨트 끝에 4 mm 모자란다.
#   y=4.60 인 이유  벨트 폭의 한가운데. 양옆 SideRail 까지 0.6 m 넘게 남는다.
#
# ★ 벨트가 실제로 물건을 끌고 간다(PhysxSurfaceVelocityAPI, 0.2 m/s).
#   그래서 두 로봇이 같은 점에 놓아도 된다 — 먼저 놓은 매거진은 다음 로봇이
#   올 때쯤 몇 미터 밖으로 실려 나가 있다. 배치 자리를 로봇마다 가를 필요가
#   없는 이유가 이것이다.
#
# ★ place.yaml 의 배치 자세가 이 좌표에 도착했다는 전제로 만든 base_link
#   상대 오프셋이고, stations.yaml PKG-01 place_pose 도 같은 값이어야 한다
#   (isaacpjt/tools/check_staging_poses.py 가 둘이 같은지 검사한다).
TEST_LOADER = (3.75, 4.60, 0.0)

# 도크 → 스캔 자리. 첫 점은 자기 줄을 따라 서쪽으로 곧장(회전 없음), 둘째 점은
# 선반 줄 동쪽 끝의 진입점, 셋째가 스캔 자리다. 둘째 점의 yaw 를 스캔 자리와
# 같게 두어 마지막 구간은 회전 없이 들어간다. 매거진은 씬에서 선반 동쪽 끝
# 슬롯(x=-0.641)에 있다 — 도크에서 가장 가까운 자리다.
DEFAULT_SCAN_ROUTE = [
    3.5, -6.591, 180.0,
    1.0, 2.036, 0.0,
    -0.641, 2.036, 0.0,
]

# 스캔 자리 → 대기 자리. 우회 가지의 APPROACH 가 이 경로를 그대로 쓰고,
# 직행 가지의 NAV 는 여기에 로더 한 점을 덧붙여 쓴다.
#
# ★ 직행도 대기 자리를 지나간다. 스캔 자리에서 로더로 직선을 그으면 선반
#   모서리를 스치고(선반 앞면이 정차점에서 0.42 m 밖에 안 떨어져 있다),
#   로더 도착각도 48도가 되어 제자리회전 여유 25도를 넘는다. 대기 자리를
#   경유하면 13.7도로 떨어진다. 덤으로 "직행을 고른 뒤 상대가 차선에 들어오면
#   로더에서 마주친다" 는 사각지대가 없어진다 — 판단 지점이 늘 로더 몇 초
#   앞이다. 직행과 우회의 차이는 "대기 자리에서 기다리느냐" 하나로 좁아진다.
#
# 첫 점은 선반 줄을 벗어나는 동쪽 이탈점이다. 이 점 없이 바로 북상하면
# 차체가 선반을 친다.
DEFAULT_APPROACH_ROUTE = [
    1.50, 2.036, 0.0,
    1.50, 5.15, 0.0,
]

# 로더 → 스택 자리. 스택은 포장 출력 선반(ShelfDeck, x 3.694~4.044,
# y 0.271~1.671, 윗면 z 0.54) 위 (3.83, 1.23) 에 yaw -90 도로 놓여 있다.
# 로봇은 그 서쪽 (3.31, 1.28) 에 남향으로 서고, 팔이 왼쪽(+x)을 보므로
# 손목캠이 스택의 -x 면 QR 을 마주본다.
#
# 첫 점은 로더에서 서쪽으로 빠지는 이탈점이다. 로더에서 스택으로 바로 그으면
# 선반 남서 모서리에 차체가 닿는다. 둘째 점에서 남향으로 돌고(선반 북쪽이라
# 사방이 비어 있다) 마지막을 선반과 나란히 내려간다 — 비스듬히 들어가면
# 뒷모서리(뒤 0.607 m)가 선반에 닿는다.
DEFAULT_STACK_PICK_ROUTE = [
    2.60, 4.60, 180.0,
    3.10, 2.60, -90.0,
    3.31, 1.28, -90.0,
]

# 스택 자리 → 검사 스테이션. 선반과 나란히 북쪽으로 빠져나온 뒤 남하하고,
# 마지막에 동향으로 돌아 벨트를 마주본다. 검사 벨트는 x 4.05~8.75,
# y -3.26~-2.16, 윗면 z 0.5407 이고 +x 로 흐른다. 마지막 점 x=3.70 은
# 점유격자에서 제자리회전 여유가 98도인 가장 동쪽 자리다 — 3.80 이면
# 여유가 25도로 떨어져 도크로 나갈 때(67도 필요) 장애물을 친다.
# place.yaml 의 tray_* 배치 자세가 이 점 기준이다.
DEFAULT_STACK_DELIVER_ROUTE = [
    3.10, 2.60, -90.0,
    3.20, -2.705, -90.0,
    3.70, -2.705, 0.0,
]

# 스택을 안 맡은 로봇이 로더에서 도크로 가기 전에 먼저 빠지는 점. 로더에서
# 도크로 바로 그으면 포장 출력 선반을 스친다.
DEFAULT_LOADER_EXIT_ROUTE = [
    2.60, 4.60, 180.0,
]

# 로더(또는 검사 스테이션) → 도크. 첫 점까지 남하하고, 둘째 점에서 180 도로
# 돌아(이웃 로봇 줄과 0.98 m 떨어져 있어 꼬리 스윕 0.656 m 가 안 닿는다)
# 마지막은 후진으로 도크에 들어간다 — 주차 방향(yaw 180)이 그대로 나온다.
# 마지막 점이 곧 도크이고 씬의 시작 자세와 같다.
DEFAULT_DOCK_ROUTE = [
    3.0, -4.6, 90.0,
    3.0, -6.3, 180.0,
    5.5, -6.591, 180.0,
]

# ── 단계 이름 ─────────────────────────────────────────────────────────────
# 상태가 아니라 라벨이다 — 트리의 어느 노드인지, 그리고 실패 기록(fail_stage)에
# 어느 단계였는지 적는 데만 쓴다. 이 문자열이 그대로 /orchestrator/state 의
# state= 값이 되고, 상대 로봇이 그걸 읽어 차선을 양보할지 정한다.
START = "start"
SCAN = "scan"
PICK = "pick"
NAV = "nav"                      # 직행으로 로더까지
PLACE = "place"
# 우회 경로(로더 차선 조율)에서만 지나가는 단계들.
HOLD_BACK = "hold_back"          # 스캔 자리에서 상대가 로더에 멈추기를 기다린다
APPROACH = "approach"            # 대기 자리로 주행
WAIT = "wait"                    # 대기 자리에서 차선이 비기를 기다린다
PUSH = "push"                    # 차선이 비면 대기 자리에서 로더로
# 스택 가지.
STACK_NAV = "stack_nav"
STACK_SCAN = "stack_scan"
STACK_PICK = "stack_pick"
STACK_DELIVER = "stack_deliver"
STACK_PLACE = "stack_place"
DOCK = "dock"                    # 도크로 복귀
DONE = "done"                    # 시나리오 끝. 도크에 서 있다

# ── 생산 트래킹 (docs/DB구성.md) ──────────────────────────────────────────
# DB 에 행이 남는 단계. scan 은 없다 — 판독 실패는 미션이 시작되지도 않은
# 것이라 남길 행이 없고, 판독 성공 시각은 pick 행의 started_at 이 곧 그것이다.
# 우회 네 단계도 넣는다 — 대기 시간이 두 대 시연의 핵심 지표다.
LOGGED_STAGES = (PICK, HOLD_BACK, APPROACH, WAIT, PUSH, NAV, PLACE,
                 STACK_PICK, STACK_DELIVER, STACK_PLACE)

# 스택 단계 → DB 어휘. stack_log 표는 magazine_log 와 컬럼이 같고 stage 도
# "pick | nav | place | return" 이다(docs/DB구성.md §4). run_id 가 스택마다
# 새로 나오므로 UNIQUE(run_id, stage, attempt) 와 부딪히지 않는다.
# stack_nav 와 stack_scan 은 기록하지 않는다 — 미션이 아직 시작 전이다.
TRACE_STAGE = {STACK_PICK: PICK, STACK_DELIVER: NAV, STACK_PLACE: PLACE}

# 실패 사유는 '단계' 가 정한다. DB 의 fail_reason ENUM 네 값과 같아야 한다.
STAGE_TO_REASON = {PICK: "pick_error", NAV: "nav_error", PLACE: "place_error",
                   # 우회 네 단계는 전부 주행/대기라 nav_error 로 모인다.
                   HOLD_BACK: "nav_error", APPROACH: "nav_error",
                   WAIT: "nav_error", PUSH: "nav_error", START: "nav_error",
                   STACK_NAV: "nav_error", STACK_PICK: "pick_error",
                   STACK_DELIVER: "nav_error", STACK_PLACE: "place_error",
                   DOCK: "dock_error"}

# 어디서 일어난 일인가.
PORT_BY_STAGE = {PLACE: "pkg_loader", PUSH: "pkg_loader", NAV: "pkg_loader"}
# 스택을 내려놓는 자리 이름. stack_place_port 파라미터 기본값.
DEFAULT_STACK_PLACE_PORT = "test_station"

# ── 타임아웃 (초) ─────────────────────────────────────────────────────────
# ★ SCAN 이 넉넉한 이유: 이 판에는 POSE 단계가 없어서 관측 자세를 잡는 일까지
#   carrier_scan 서비스 한 번 안에 들어간다. 게다가 sim_backend 는 RPC 를 한
#   번에 하나씩 처리하고 뷰포트도 하나라, 두 로봇의 스캔이 줄을 선다. GUI
#   렌더에서 한 번이 10~20초이므로 재시도까지 이 안에 들어가려면 크게 잡아야
#   한다. 옛 값(40초)은 한 대 기준이었다.
SCAN_TIMEOUT_S = 180.0
NAV_TIMEOUT_S = 300.0
PICK_TIMEOUT_S = 120.0
PLACE_TIMEOUT_S = 120.0
SERVER_WAIT_S = 5.0

# SCAN 이 found=false 를 받아도 바로 포기하지 않고 이 횟수만큼 carrier_scan 을
# 다시 부른다(최초 시도 포함 총 SCAN_RETRIES+1 번). 매 시도가 팔을 다시
# 정렬하고 카메라를 새로 캡처하므로, 렌더링 워밍업 부족이나 그 한 프레임의
# 일시적 디코드 실패를 재시도로 걸러낸다.
SCAN_RETRIES = 2

# 트리를 이 주기로 tick 한다. 잎이 블로킹하지 않으므로 한 tick 은 수십 마이크로초다.
TICK_PERIOD_S = 0.1
# 상태 발행 주기. 이 값이 상대가 보는 정보의 최대 지연이다.
STATE_PUBLISH_PERIOD_S = 0.2

# ── 로더 차선 조율 ───────────────────────────────────────────────────────
# 상대가 차선을 쓰고 있다고 보는 단계들. 여기 들어 있으면 기다린다.
# launch 의 LANE_STAGES 와 같은 값이어야 한다(test_peer_yield.py 가 검사).
#
#   nav · push    로더로 들어가는 중
#   place         로더에 붙어서 내려놓는 중
#   stack_*       스택 자리와 검사 스테이션이 로더 곁이라, 그 사이 로더로
#                 들어가면 상대의 경로를 가로지른다
#
# ★ approach · wait · hold_back 은 넣지 않는다. 줄 서 있는 상태를 양보 대상으로
#   만들면 양쪽이 서로의 대기를 기다려 아무도 안 움직인다.
DEFAULT_PEER_BUSY_STAGES = [NAV, PUSH, PLACE,
                            STACK_NAV, STACK_SCAN, STACK_PICK,
                            STACK_DELIVER, STACK_PLACE]

# 상대가 아직 차선 판단 전이라 곧 로더로 올 단계들. 후순위 로봇은 선순위
# 상대가 여기 있으면 우회로 간다(PeerClear · HOLD_BACK 의 yield_to_contending).
CONTENDING_STAGES = [START, SCAN, PICK]

# HOLD_BACK 이 스캔 자리에서 붙잡고 있는 상대 단계 — 상대가 로더 구역 안에서
# '움직이고 있는' 동안이다. 대기 자리가 로더 접근선 위라 거기로 가는 것 자체가
# 그 구역에 들어가는 것이다.
# ★ place 는 일부러 뺐다. 상대가 로더에 '멈춰' 있는 유일한 구간이고, 바로
#   그때가 이쪽이 대기 자리까지 올라가기에 안전한 때다.
HOLD_BACK_STAGES = [NAV, PUSH, STACK_NAV]

# 상대가 로더에서 '멀어지는 중' 인 단계. 여기서만 거리로 판정한다 — 반경을
# 벗어난 순간 진입해도 되므로 단계가 끝나기를 기다리지 않는다. 나머지 단계는
# 거리가 멀어도 곧 들어오므로 거리를 보면 오답이 나온다.
PEER_LEAVING_STAGES = [STACK_NAV]

# 먼저 place 한 로봇이 스택을 맡는다. 상대가 이 단계 중 하나면 상대가 이미
# 로더를 지났다는 뜻이라 나는 나중이다 — 도킹으로 간다(StackFirst).
PEER_PAST_LOADER_STAGES = [STACK_NAV, STACK_SCAN, STACK_PICK, STACK_DELIVER,
                           STACK_PLACE, DOCK, DONE]

# 대기를 푸는 기준: 로더 주차점 중심 반경.
# ★ 시간으로 재지 않는 이유 — 이 파일의 타임아웃은 전부 벽시계인데, Isaac 을
#   GUI 렌더로 돌리면 시뮬이 실시간보다 느려서 벽시계로 본 이동 속도가 머신·
#   렌더 설정마다 다르다. "상대가 나간 뒤 N 초" 의 N 을 정할 근거가 없다.
DEFAULT_LOADER_CLEAR_RADIUS_M = 1.0
# 대기 자리에서 이만큼 기다려도 안 비면 실패로 본다. "성능" 문턱이 아니라
# "아무도 안 온다" 문턱이라 넉넉히 둔다 — 상대가 얼어붙은 경우는 이 타임아웃이
# 아니라 PEER_FROZEN 이 먼저 잡는다.
PEER_WAIT_TIMEOUT_S = 1800.0
# 조건이 풀린 뒤 이만큼 더 서 있는다. 지금은 형식적인 값이다 — 풀리는 경로가
# 전부 위치나 완료 상태를 보기 때문에 SUCCESS 를 낼 때 상대는 이미 반경 밖이다.
PEER_CLEAR_DWELL_S = 2.0

# 로더 차선 순서. 1 이 가장 먼저이고 0 이면 순서를 정하지 않는다.
DEFAULT_LANE_PRIORITY = 0
# 대기 자리 — approach_route 의 마지막 점과 같아야 한다.
#   robot1 (1.50, 5.15)  도착각 -13.7도
#   robot2 (0.10, 5.15)  도착각  -8.5도
# 이탈 궤적이 로더에서 남서로 흐르므로 둘 다 차선 북쪽에 둔다.
# 검증은 isaacpjt/tools/check_staging_poses.py.
DEFAULT_STAGING_POSE = [1.50, 5.15, 0.0]

# 출발 게이트. 후순위 로봇은 선순위 상대가 자기 도크(dock_route 의 마지막 점)
# 에서 이만큼 밖으로 나가야 출발한다. 두 도크가 0.98 m 떨어져 있고 꼬리 스윕이
# 0.656 m 라 1.5 m 면 상대가 회전 반경 밖이다.
DEFAULT_START_CLEARANCE_M = 1.5
# 상대 상태·위치를 아직 한 번도 못 받았으면 이만큼만 기다려 보고 출발한다.
START_PEER_GRACE_S = 3.0
# 게이트 상한. 상대가 안 움직여도(얼었거나 amcl 이 안 나와도) 이 뒤엔 출발한다.
START_GATE_TIMEOUT_S = 60.0


def _find_ws_root():
    p = Path(__file__).resolve()
    for _ in range(10):
        if (p / "isaacpjt").is_dir():
            return p
        if p.parent == p:
            break
        p = p.parent
    raise RuntimeError("cobot3_ws 를 못 찾았다")


WS_ROOT = _find_ws_root()

# isaacpjt 는 ament 패키지가 아니라 그냥으로는 import 되지 않는다. carrier_code.py 는
# 의존성 없는 순수 파이썬이고 QR 코드 규칙의 유일한 주인이라, 복사본을 만드는 대신
# 경로를 열어 그 파일 하나를 쓴다.
sys.path.insert(0, str(WS_ROOT / "isaacpjt" / "assets"))
import carrier_code  # noqa: E402

# ── QR 원문 -> grasp.yaml / place.yaml 의 variant 키 ──────────────────────
# 옛 씬의 매거진 QR 은 "1" 또는 "2" 한 글자만 담았다. 16종 에셋으로 바꾸면
# "F1-MGZB-1" 이 온다. 씬 교체가 끝나기 전까지 둘 다 받는다. 스택은 이미 새
# 형식(F3-STKO-1)이라 carrier_code 가 푼다.
NUMERIC_TO_VARIANT = {"1": "magazine_1_orange", "2": "magazine_2_blue"}


def _variant_of(payload):
    """QR 원문 -> variant 키. 못 풀면 None.

    새 페이로드는 carrier_code 가 푼다. 거기서 나오는 base_asset 의 확장자만
    떼면 grasp.yaml / place.yaml 의 키와 그대로 맞는다:
        "F3-STKO-1" -> base_asset "tray_1_orange.usda" -> "tray_1_orange"
    """
    info = carrier_code.parse_code(payload)
    if info is not None:
        return info.base_asset[:-len(".usda")]
    return NUMERIC_TO_VARIANT.get(payload)          # 옛 매거진 QR 폴백


def _family_of(payload):
    """QR 원문 -> "magazine" | "stack" | None. 옛 숫자 QR 은 매거진이다."""
    info = carrier_code.parse_code(payload)
    if info is not None:
        return info.family
    return "magazine" if payload in NUMERIC_TO_VARIANT else None


def _reason_name(result_cls, code):
    """액션 result 의 fail_reason 코드를 .action 에 적힌 상수명으로 바꾼다."""
    for name in dir(result_cls):
        if not name.isupper():
            continue
        value = getattr(result_cls, name, None)
        if isinstance(value, int) and value == code:
            return f"{name}({code})"
    return str(code)


def _unflatten_route(flat):
    """[x0,y0,yaw0, x1,y1,yaw1, ...] → [(x0,y0,yaw0), (x1,y1,yaw1), ...].

    ★ 잘못된 입력에는 예외를 던진다 — 빈 리스트를 돌려주지 않는다. 이 값이
      어그러질 수 있는 경로는 "launch 가 넘긴 숫자 표가 틀렸다" 하나뿐이고,
      그건 런타임 상황이 아니라 설정 실수다. 미션 도중에 멈추는 것보다 노드가
      뜰 때 죽는 편이 낫다 — 런치 로그에 바로 보이고, 좌표를 모르는 로봇이
      주행 goal 을 내는 일도 없다.
    """
    vals = [float(v) for v in flat]
    if not vals or len(vals) % 3 != 0:
        raise ValueError(
            f"경로의 길이가 {len(vals)} 다 — 비어 있지 않고 3 의 배수여야 한다. "
            f"정차점 하나가 (x, y, yaw_deg) 세 칸이다. "
            f"mission_nodes.launch.py 의 이 로봇 항목을 확인해라.")
    return [tuple(vals[i:i + 3]) for i in range(0, len(vals), 3)]


def _to_pose(xy_yaw_deg):
    """(x, y, yaw_deg) -> NavigateTo goal 의 PoseStamped(map)."""
    x, y, yaw_deg = xy_yaw_deg
    yaw = math.radians(yaw_deg)
    ps = PoseStamped()
    ps.header.frame_id = "map"
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.orientation.z = math.sin(yaw / 2.0)
    ps.pose.orientation.w = math.cos(yaw / 2.0)
    return ps


def _blackboard(name, keys):
    bb = py_trees.blackboard.Client(name=name)
    for key in keys:
        bb.register_key(key=key, access=Access.WRITE)
    return bb


# ══════════════════════════════════════════════════════════════════════════
#  잎 — 실제로 뭔가 하는 노드들.
#
#  공통 규칙 하나: 어떤 잎도 블로킹하지 않는다. future 를 기다리지 말고
#  done() 을 확인하고 RUNNING 을 돌려준다.
# ══════════════════════════════════════════════════════════════════════════


class ActionLeaf(py_trees.behaviour.Behaviour):
    """액션 goal 하나. NavigateTo · PickCarrier · PlaceCarrier 가 전부 이걸 쓴다.

    셋 다 result 가 success · fail_reason 모양이라 하나로 된다.
    make_goal 은 initialise() 에서 한 번만 불린다.
    """

    def __init__(self, name, node, client, server_name, result_cls, make_goal,
                 timeout_s, feedback_cb=None):
        super().__init__(name)
        self.node = node
        self.client = client
        self.server_name = server_name
        self.result_cls = result_cls
        self.make_goal = make_goal
        self.timeout_s = timeout_s
        self.feedback_cb = feedback_cb
        self.goal_handle = None
        self.send_future = None
        self.result_future = None

    def initialise(self):
        self.goal_handle = None
        self.send_future = None
        self.result_future = None
        self.goal = self.make_goal()
        now = time.monotonic()
        self.server_deadline = now + SERVER_WAIT_S
        self.deadline = now + self.timeout_s
        # 생산 트래킹 — 이 단계가 시작된 시각. Freeze 가 끝날 때 읽어 간다.
        # 시뮬과 벽시계를 둘 다 찍는 이유는 docs/DB구성.md §4-2.
        self.started_stamp, self.started_wall = self.node.now_pair()

    def update(self):
        if time.monotonic() > self.deadline:
            self.feedback_message = f"TIMEOUT({self.timeout_s:.0f}s)"
            return Status.FAILURE

        # 1) 서버를 기다린다. 블로킹하지 않고 tick 마다 확인만 한다.
        if self.send_future is None:
            if not self.client.server_is_ready():
                if time.monotonic() > self.server_deadline:
                    self.feedback_message = f"SERVER_UNAVAILABLE({self.server_name})"
                    return Status.FAILURE
                self.feedback_message = "서버 대기"
                return Status.RUNNING
            self.send_future = self.client.send_goal_async(
                self.goal, feedback_callback=self.feedback_cb)

        # 2) goal 이 수락되기를 기다린다.
        if self.goal_handle is None:
            if not self.send_future.done():
                self.feedback_message = "goal 수락 대기"
                return Status.RUNNING
            self.goal_handle = self.send_future.result()
            if not self.goal_handle.accepted:
                self.feedback_message = "GOAL_REJECTED"
                return Status.FAILURE
            self.result_future = self.goal_handle.get_result_async()

        # 3) result 를 기다린다.
        if not self.result_future.done():
            return Status.RUNNING
        result = self.result_future.result().result
        if result.success:
            return Status.SUCCESS
        self.feedback_message = _reason_name(self.result_cls, result.fail_reason)
        return Status.FAILURE

    def terminate(self, new_status):
        """실패했으면 진행 중인 goal 을 거둔다.

        최상위가 Sequence 라 선점(INVALID)은 이 판에서 일어나지 않지만,
        py_trees 가 트리를 정리할 때는 부를 수 있으므로 같이 받는다.
        """
        if new_status not in (Status.INVALID, Status.FAILURE):
            self.goal_handle = None
            return
        if self.goal_handle is not None and self.result_future is not None \
                and not self.result_future.done():
            self.node.begin_cancel(self.goal_handle, self.name)
        self.goal_handle = None


class ScanLeaf(py_trees.behaviour.Behaviour):
    """carrier_scan 서비스를 호출해 QR 정보를 받아 블랙보드에 넣는다.

    멈춘 뒤에 읽어야 정확하다 (05 §12-1 정지 상태에서 근접 판독). 앞 단계가
    주행 goal 의 result 를 받은 뒤에 오므로 여기 올 때는 이미 서 있다.

    ★ 받는 쪽(carrier_code_reader)이 이 호출 안에서 관측 자세까지 잡는다.
      순찰 판에 있던 POSE 단계(observe_pose 서비스)를 없앤 자리다.

    ★ 실패는 전부 Freeze 다. 순찰이 없어서 "못 읽었으니 다음 바퀴에 다시" 가
      없다 — 사람이 보고 /orchestrator/resume 으로 다시 돌린다.

    expect_family 로 종류를 검사한다. 스택 자리에서 매거진이 읽히면 자리가
    틀린 것이라 재시도해도 같다 — 바로 얼린다.
    """

    def __init__(self, name, node, expect_family):
        super().__init__(name)
        self.node = node
        self.expect_family = expect_family
        self.bb = _blackboard(name, ("kind", "variant", "carrier_id", "qr_pose"))
        self.future = None
        self.attempt = 0

    def initialise(self):
        self.future = None
        self.attempt = 0
        now = time.monotonic()
        self.server_deadline = now + SERVER_WAIT_S
        self.deadline = now + SCAN_TIMEOUT_S
        self.started_stamp, self.started_wall = self.node.now_pair()

    def update(self):
        if time.monotonic() > self.deadline:
            self.feedback_message = f"TIMEOUT({SCAN_TIMEOUT_S:.0f}s)"
            return Status.FAILURE

        if self.future is None:
            if not self.node.carrier_scan.service_is_ready():
                if time.monotonic() > self.server_deadline:
                    self.feedback_message = "SERVICE_UNAVAILABLE(perception/carrier_scan)"
                    return Status.FAILURE
                self.feedback_message = "서비스 대기"
                return Status.RUNNING
            self.future = self.node.carrier_scan.call_async(CarrierScan.Request())

        if not self.future.done():
            return Status.RUNNING

        res = self.future.result()
        if not res.found:
            self.attempt += 1
            if self.attempt <= SCAN_RETRIES:
                # 재시도 — 받는 쪽이 팔을 재정렬하고 카메라를 새로 캡처한다.
                self.node.get_logger().info(
                    f"SCAN 미판독 — 재시도 {self.attempt}/{SCAN_RETRIES}")
                self.future = None
                return Status.RUNNING
            self.feedback_message = f"NOT_FOUND(found=false) x{self.attempt}"
            return Status.FAILURE

        variant = _variant_of(res.payload)
        if variant is None:
            self.feedback_message = f"UNKNOWN_PAYLOAD({res.payload!r})"
            return Status.FAILURE
        family = _family_of(res.payload)
        if family != self.expect_family:
            self.feedback_message = (
                f"WRONG_FAMILY({res.payload!r} 은 {family}, 기대 {self.expect_family})")
            return Status.FAILURE

        qr_pose = PoseStamped()
        qr_pose.header = res.header
        qr_pose.pose = res.qr_pose
        self.bb.kind = res.payload
        self.bb.variant = variant
        self.bb.carrier_id = res.payload      # 페이로드 자체가 개체 ID 다
        self.bb.qr_pose = qr_pose
        self.node.get_logger().info(
            f"scan — 종류={self.bb.kind} variant={self.bb.variant}")
        self.feedback_message = f"{self.bb.variant}"
        return Status.SUCCESS


class PeerClear(py_trees.behaviour.Behaviour):
    """상대가 로더 차선을 쓰고 있지 않은가. 배송 경로를 고르는 분기 조건이다.

        SUCCESS  직행 가지 — 대기 자리를 거쳐 로더로 바로 간다
        FAILURE  우회 가지 — 대기 자리에서 차선이 비기를 기다린다

    "쓰고 있다" 의 기준은 node.peer_busy() 가 정한다. 거기에 더해, 상대가 아직
    차선 판단 전(CONTENDING_STAGES)인데 나보다 선순위면 양보한다 — 둘이 동시에
    PICK 을 마치는 이 시나리오에서 순서를 정하는 유일한 장치다.

    FAILURE 가 에러가 아니라 경로 선택이라 Freeze 로 감싸지 않는다. 감싸면
    로봇이 얼어붙는다.
    """

    def __init__(self, name, node):
        super().__init__(name)
        self.node = node

    def update(self):
        busy, why = self.node.peer_busy()
        if not busy and self.node.peer_contending() and self.node.peer_outranks_me():
            busy = True
            why = f"state={self.node.peer_stage()} (선순위 prio={self.node.peer_prio()})"
        if busy:
            self.node.get_logger().info(f"로더 차선 사용 중({why}) — 대기 자리로 우회한다")
            self.feedback_message = f"우회 — 상대 {why}"
            return Status.FAILURE
        self.feedback_message = f"직행 — {why}"
        return Status.SUCCESS


class WaitForPeer(py_trees.behaviour.Behaviour):
    """상대가 차선을 비우기를 기다린다. 액션도 스레드도 future 도 없는 조건 잎이다.

    실패는 둘이다. 어느 쪽이든 Freeze 가 받아서 그 자리에 세운다.
      PEER_FROZEN       상대가 차선 안에서 얼어붙었다. 물리적으로 비켜지지
                        않으므로 기다려 봐야 소용없다.
      PEER_WAIT_TIMEOUT PEER_WAIT_TIMEOUT_S 를 넘겼다.

    풀리는 조건은 둘 중 먼저 오는 것이다.
      상태 변화  상대가 양보 목록을 벗어나면 PEER_CLEAR_DWELL_S 뒤에 진입한다.
      반경       상대가 로더에서 멀어지는 중인 단계(PEER_LEAVING_STAGES)에
                 있고 loader_clear_radius_m 밖으로 나갔으면 아직 그 단계여도
                 진입한다. 상대 위치를 못 받으면 상태 변화만 기다린다.
    어느 경로든 상대가 로더 반경 안에 있으면 먼저 막힌다 — 상태와 무관하다.

    상대를 한 번도 본 적이 없으면 즉시 통과한다 — 한 대만 띄웠을 때 영원히
    기다리는 걸 막는다.
    """

    def __init__(self, name, node, stages=None, use_radius=True, arrive_log="",
                 yield_to_contending=False):
        """stages      기다릴 상대 단계. None 이면 peer_busy_stages 전체.
        use_radius  로더 반경으로도 판정할지. 로더에서 멀리 떨어져 기다리는
                    자리(스캔 자리)에서는 반경이 의미가 없어서 끈다.
        arrive_log  이 잎에 처음 들어갈 때 남길 로그 한 줄.
        yield_to_contending  선순위 상대가 아직 출발 전이어도 붙잡는다.
        """
        super().__init__(name)
        self.node = node
        self.stages = stages
        self.use_radius = use_radius
        self.arrive_log = arrive_log
        self.yield_to_contending = yield_to_contending

    def _busy(self):
        """(기다려야 하나, 이유). stages 가 있으면 그 목록으로만 본다."""
        if self.yield_to_contending and self.node.peer_contending() \
                and self.node.peer_outranks_me():
            return True, f"state={self.node.peer_stage()} (선순위 상대가 곧 로더로 온다)"
        if self.stages is None:
            return self.node.peer_busy()
        stage = self.node.peer_stage()
        if stage is None:
            return False, "상대 없음"
        if stage in self.stages:
            return True, f"state={stage}"
        return False, f"state={stage}"

    def initialise(self):
        self.deadline = time.monotonic() + PEER_WAIT_TIMEOUT_S
        self._clear_since = None
        # ★ Freeze 가 DB 행을 만들 때 getattr 로 읽어 간다. 안 넣으면 조용히
        #   now 로 대체되어 duration_sec 이 0 으로 찍힌다(에러는 안 난다).
        self.started_stamp, self.started_wall = self.node.now_pair()
        # 즉시 알린다 — 상대가 이 값을 보고 자기 차례를 판단한다.
        self.node.publish_state()
        if self.arrive_log:
            self.node.get_logger().info(self.arrive_log)

    def update(self):
        if self.node.peer_frozen():
            self.feedback_message = f"PEER_FROZEN(state={self.node.peer_stage()})"
            return Status.FAILURE
        if time.monotonic() > self.deadline:
            self.feedback_message = f"PEER_WAIT_TIMEOUT({PEER_WAIT_TIMEOUT_S:.0f}s)"
            return Status.FAILURE

        now = time.monotonic()
        stage = self.node.peer_stage()
        busy, why = self._busy()
        dist = self.node.peer_dist_to_loader() if self.use_radius else None
        need = self.node.loader_clear_radius_m

        # ① 위치가 먼저다. 상대가 로더 반경 안에 있으면 상태와 무관하게 기다린다.
        if dist is not None and dist < need:
            self._clear_since = None
            self.feedback_message = f"대기 — 상대가 로더 {dist:.2f}/{need:.2f} m"
            return Status.RUNNING

        # ② 상대가 로더에서 멀어지는 중이면 반경을 벗어난 것으로 충분하다.
        if busy and stage in PEER_LEAVING_STAGES:
            if dist is None:
                self.feedback_message = f"대기 — 상대 {why} (위치 모름)"
                return Status.RUNNING
            busy, why = False, f"로더 {dist:.2f} m 밖"

        # ③ 상대가 로더로 다가오는 중이면 거리와 무관하게 기다린다.
        if busy:
            self._clear_since = None
            self.feedback_message = f"대기 — 상대 {why}"
            return Status.RUNNING

        if self._clear_since is None:
            self._clear_since = now
            self.node.get_logger().info(f"로더가 비었다 — {why}")
        if now - self._clear_since < PEER_CLEAR_DWELL_S:
            self.feedback_message = f"로더 비었음 — 여유 대기 ({why})"
            return Status.RUNNING

        self.feedback_message = f"로더 비었음 — 진입 ({why})"
        return Status.SUCCESS


class WaitPeerAway(py_trees.behaviour.Behaviour):
    """출발 게이트. 후순위 로봇은 선순위 상대가 자기 도크 곁을 떠날 때까지 선다.

    왜 필요한지는 모듈 독스트링 "출발 게이트가 필요한 이유". 거리는 상대의
    amcl_pose 와 내 도크(anchor)로 잰다 — 이 노드는 자기 위치를 모르지만
    출발 전이라 도크에 있다는 것은 안다.

    통과 조건 (먼저 오는 것):
      선순위다 / 상대가 없다      바로 간다. 상대 정보가 아직 없으면
                                  START_PEER_GRACE_S 만 기다려 본다
      상대가 clearance 밖         회전 스윕 밖이다
      START_GATE_TIMEOUT_S 경과   상대가 얼었거나 위치를 못 받는다. 경고하고 간다
    """

    def __init__(self, name, node, anchor, clearance_m):
        super().__init__(name)
        self.node = node
        self.anchor = anchor
        self.clearance_m = float(clearance_m)

    def initialise(self):
        now = time.monotonic()
        self.grace_deadline = now + START_PEER_GRACE_S
        self.deadline = now + START_GATE_TIMEOUT_S
        self.node.publish_state()      # 상대도 내 prio 를 봐야 한다

    def update(self):
        now = time.monotonic()
        if not self.node.peer_seen():
            if now < self.grace_deadline:
                self.feedback_message = "상대 상태 대기"
                return Status.RUNNING
            self.feedback_message = "상대 없음 — 출발"
            return Status.SUCCESS
        if not self.node.peer_outranks_me():
            self.feedback_message = "선순위 — 출발"
            return Status.SUCCESS
        if now > self.deadline:
            self.node.get_logger().warning(
                f"출발 게이트 {START_GATE_TIMEOUT_S:.0f}s 초과 — 상대(state="
                f"{self.node.peer_stage()})가 비켜 주지 않는다. 그냥 출발한다")
            return Status.SUCCESS
        xy = self.node.peer_xy()
        if xy is None:
            self.feedback_message = "상대 위치 대기"
            return Status.RUNNING
        dist = math.hypot(xy[0] - self.anchor[0], xy[1] - self.anchor[1])
        if dist < self.clearance_m:
            self.feedback_message = f"상대가 도크 곁 {dist:.2f}/{self.clearance_m:.2f} m"
            return Status.RUNNING
        self.node.get_logger().info(f"상대가 {dist:.2f} m 밖으로 나갔다 — 출발")
        return Status.SUCCESS


class StackFirst(py_trees.behaviour.Behaviour):
    """내가 먼저 place 했나. 스택 가지의 문지기다.

    SUCCESS 면 스택을 맡고, FAILURE 면 나중 로봇이라 도킹으로 간다. 기준은
    "상대가 이미 로더를 지났는가"(PEER_PAST_LOADER_STAGES). 상대가 없으면
    (한 대) 당연히 내 몫이다. 상대가 얼어 있어도 로더 전이면 내가 맡는다 —
    그 상대는 스택을 나를 수 없다.

    ★ 순서가 겹칠 수 없는 이유: 로더는 하나이고 배송 가지가 한 대씩만
      들여보낸다. 먼저 놓은 쪽이 stack_nav 로 넘어가 상태를 즉시 발행하고,
      나중 쪽은 그로부터 한 사이클 뒤에나 이 잎에 온다.

    FAILURE 가 에러가 아니라 분기라 Freeze 로 감싸지 않는다.
    """

    def __init__(self, name, node):
        super().__init__(name)
        self.node = node

    def update(self):
        stage = self.node.peer_stage()
        if not self.node.peer_seen():
            why = "상대 없음"
        elif stage in PEER_PAST_LOADER_STAGES:
            self.node.get_logger().info(
                f"상대가 먼저 place 했다(state={stage}) — 스택은 상대 몫, 도킹으로")
            self.feedback_message = f"나중 — 상대 {stage}"
            return Status.FAILURE
        else:
            why = f"상대 state={stage}"
        self.node.get_logger().info(f"내가 먼저 place 했다({why}) — 스택을 가지러 간다")
        self.feedback_message = f"먼저 — {why}"
        return Status.SUCCESS


class CycleDone(py_trees.behaviour.Behaviour):
    """캐리어 하나가 끝났다. 손이 비었다고 확정하고 블랙보드를 비운다."""

    def __init__(self, name, node, next_label):
        super().__init__(name)
        self.node = node
        self.next_label = next_label
        self.bb = _blackboard(name, ("kind", "variant", "carrier_id", "qr_pose", "run_id"))

    def update(self):
        self.node.get_logger().info(
            f"사이클 완료 (carrier={self.bb.carrier_id}) — {self.next_label}")
        self.bb.kind = ""
        self.bb.variant = ""
        self.bb.carrier_id = ""
        self.bb.qr_pose = None
        self.bb.run_id = ""          # 다음 미션은 새 run_id 를 받는다
        return Status.SUCCESS


class Idle(py_trees.behaviour.Behaviour):
    """시나리오 끝. 영원히 RUNNING — 로봇은 도크에 서 있고 state=done 이다."""

    def __init__(self, name, node):
        super().__init__(name)
        self.node = node

    def initialise(self):
        self.node.get_logger().info("시나리오 완료 — 도크에서 대기한다 (state=done)")
        self.node.publish_state()

    def update(self):
        self.feedback_message = "도크 대기"
        return Status.RUNNING


# ══════════════════════════════════════════════════════════════════════════
#  데코레이터
# ══════════════════════════════════════════════════════════════════════════


class Freeze(py_trees.decorators.Decorator):
    """자식이 FAILURE 면 그 자리에서 멈춘다 — 그 뒤로는 tick 마다 RUNNING.

    "로봇을 멈추고 그 자리에서 정지한다. 자동 복귀도 재시도도 하지 않는다."
    RUNNING 을 돌려주므로 위쪽 Sequence 는 이 가지를 붙들고 있고, 로봇은
    아무 데도 가지 않는다. /orchestrator/resume 이 frozen 을 풀면 자식이
    initialise() 부터 다시 돌아 goal 이 새로 나간다.
    """

    def __init__(self, name, child, node, stage):
        super().__init__(name=name, child=child)
        self.node = node
        self.stage = stage
        self.frozen = False

    def tick(self):
        """얼어붙은 뒤에는 자식을 절대 다시 tick 하지 않는다.

        py_trees 의 Decorator.tick() 은 무조건 자식을 tick 하는데, FAILURE 로
        끝난 잎을 다시 tick 하면 Behaviour.tick() 이 initialise() 를 다시 부른다
        — 액션 잎이면 goal 이 새로 나간다. 정지해 있어야 할 로봇이 다시 움직인다.
        """
        if self.frozen:
            self.status = Status.RUNNING
            yield self
            return
        yield from super().tick()

    def update(self):
        child = self.decorated

        if child.status == Status.SUCCESS:
            # ★ 생산 트래킹의 발행 지점이다. 기록하는 단계가 전부 이 데코레이터를
            #   지나가므로 여기 한 곳만 고치면 다 걸린다.
            if self.stage in (SCAN, STACK_SCAN):
                # 페이로드를 처음 확보한 순간 = 미션 하나의 시작. run_id 를 발행한다.
                self.node.new_run()
            elif self.stage in LOGGED_STAGES:
                self.node.emit_trace(self.stage, child, ok=True)
            self.feedback_message = child.feedback_message
            return child.status

        if child.status != Status.FAILURE:
            self.feedback_message = child.feedback_message
            return child.status

        reason = child.feedback_message or "UNKNOWN"
        if self.stage in LOGGED_STAGES:
            self.node.emit_trace(self.stage, child, ok=False, fail_detail=reason)

        self.frozen = True
        # self 를 넘기는 이유: /orchestrator/resume 이 얼어붙은 이 잎을 찾아
        # frozen=False 로 되돌려야 한다 (TaskManager._on_resume).
        self.node.on_freeze(self.stage, reason, self)
        self.feedback_message = f"FROZEN: {reason}"
        return Status.RUNNING


# ══════════════════════════════════════════════════════════════════════════
#  트리
# ══════════════════════════════════════════════════════════════════════════


def build_tree(node):
    """이 모듈 맨 위 독스트링의 트리를 조립한다."""
    bb = _blackboard("build_tree", ("variant", "qr_pose"))

    def nav_leaf(stage, pose):
        return Freeze(stage.upper(), ActionLeaf(
            stage, node, node.nav, "navigation/navigate_to", NavigateTo.Result,
            make_goal=lambda p=pose: NavigateTo.Goal(pose=_to_pose(p)),
            timeout_s=NAV_TIMEOUT_S), node, stage)

    def route(stage, points, label):
        """경로의 점 하나가 NavigateTo goal 하나다.

        잎 이름이 같아도 된다 — current_stage() 는 RUNNING 인 잎의 이름만 보므로
        state= 에는 stage 가 그대로 나간다. 점을 나눠 두는 이유는 nav_server 가
        직선으로만 몰기 때문이다: 중간 점이 없으면 선반이나 컨베이어를 가로지른다.
        """
        return py_trees.composites.Sequence(
            label, memory=True, children=[nav_leaf(stage, pt) for pt in points])

    dock_pose = node.dock_route[-1]
    gate = WaitPeerAway(START, node, anchor=dock_pose,
                        clearance_m=node.start_clearance_m)
    start = route(START, node.scan_route, "출발")

    scan = Freeze("SCAN", ScanLeaf(SCAN, node, expect_family="magazine"), node, SCAN)
    pick = Freeze("PICK", ActionLeaf(
        PICK, node, node.pick, "manipulation/pick_carrier", PickCarrier.Result,
        make_goal=lambda: PickCarrier.Goal(variant=bb.variant, qr_pose=bb.qr_pose),
        timeout_s=PICK_TIMEOUT_S,
        feedback_cb=node.log_phase("PICK")), node, PICK)

    # ── 배송 — 직행과 우회. 어느 쪽이든 approach_route 를 지나간다 ──────────
    # Selector memory=True 가 중요하다 — 한 번 고른 가지를 그 가지가 끝날 때까지
    # 들고 간다. memory=False 면 매 tick 상대를 다시 보고, 주행 중에 상대가
    # 차선에 들어오는 순간 직행 가지가 무효화된다. 그러면 미션 Sequence 가
    # memory=True 라 PICK 부터 다시 시작해 버린다 — 이미 캐리어를 들고 있는데
    # 또 집으려 든다.
    direct = py_trees.composites.Sequence(
        "직행", memory=True,
        children=[PeerClear("상대 한가?", node),
                  route(NAV, list(node.approach_route) + [TEST_LOADER], "로더로")])

    # ★ 대기 자리가 로더 접근선 위라, 거기로 가는 것 자체가 로더 구역에 들어가는
    #   것이다. 그 구역에서 상대가 움직이고 있으면 둘이 같은 공간에서 엇갈린다.
    #   그래서 스캔 자리에서 먼저 기다린다 — 상대가 로더에 '멈춰 있는' 동안
    #   (place)에만 올라간다.
    hold_back = Freeze("HOLD_BACK", WaitForPeer(
        HOLD_BACK, node, stages=HOLD_BACK_STAGES, use_radius=False,
        yield_to_contending=True,
        arrive_log="상대가 로더 구역에서 움직이는 중 — 스캔 자리에서 기다린다"),
        node, HOLD_BACK)
    approach = route(APPROACH, node.approach_route, "대기 자리로")
    # WAIT 의 양보 목록은 peer_busy_stages 전체라 상대의 스택 사이클 내내
    # 기다린다 — 모듈 독스트링 "양보 목록에 스택 단계가 들어 있는 이유".
    wait = Freeze("WAIT", WaitForPeer(
        WAIT, node, arrive_log="대기 자리 도착 — 상대가 로더 곁을 비우기를 기다린다"),
        node, WAIT)
    push = Freeze("PUSH", ActionLeaf(
        PUSH, node, node.nav, "navigation/navigate_to", NavigateTo.Result,
        make_goal=lambda: NavigateTo.Goal(pose=_to_pose(TEST_LOADER)),
        timeout_s=NAV_TIMEOUT_S), node, PUSH)
    detour = py_trees.composites.Sequence(
        "우회", memory=True, children=[hold_back, approach, wait, push])
    to_loader = py_trees.composites.Selector(
        "배송", memory=True, children=[direct, detour])

    # 놓을 자리는 종류로 정해진다 — 좌표를 넘기지 않는다(place.yaml).
    place = Freeze("PLACE", ActionLeaf(
        PLACE, node, node.place, "manipulation/place_carrier", PlaceCarrier.Result,
        make_goal=lambda: PlaceCarrier.Goal(variant=bb.variant),
        timeout_s=PLACE_TIMEOUT_S,
        feedback_cb=node.log_phase("PLACE")), node, PLACE)

    # ── 스택 — 먼저 place 한 쪽만 ─────────────────────────────────────────
    stack_scan = Freeze("STACK_SCAN", ScanLeaf(
        STACK_SCAN, node, expect_family="stack"), node, STACK_SCAN)
    stack_pick = Freeze("STACK_PICK", ActionLeaf(
        STACK_PICK, node, node.pick, "manipulation/pick_carrier", PickCarrier.Result,
        make_goal=lambda: PickCarrier.Goal(variant=bb.variant, qr_pose=bb.qr_pose),
        timeout_s=PICK_TIMEOUT_S,
        feedback_cb=node.log_phase("STACK_PICK")), node, STACK_PICK)
    stack_place = Freeze("STACK_PLACE", ActionLeaf(
        STACK_PLACE, node, node.place, "manipulation/place_carrier",
        PlaceCarrier.Result,
        make_goal=lambda: PlaceCarrier.Goal(variant=bb.variant),
        timeout_s=PLACE_TIMEOUT_S,
        feedback_cb=node.log_phase("STACK_PLACE")), node, STACK_PLACE)
    stack = py_trees.composites.Sequence(
        "스택", memory=True,
        children=[StackFirst("먼저 place?", node),
                  route(STACK_NAV, node.stack_pick_route, "스택으로"),
                  stack_scan, stack_pick,
                  route(STACK_DELIVER, node.stack_deliver_route, "검사 스테이션으로"),
                  stack_place,
                  CycleDone("스택 완료", node, next_label="도킹")])
    # 나중에 place 한 쪽은 스택을 건너뛰고 바로 도킹한다. 그래도 로더에서
    # 서쪽으로 한 번 빠져 줘야 한다 — 로더에서 도크로 바로 그으면 포장 출력
    # 선반을 스친다(DEFAULT_LOADER_EXIT_ROUTE).
    after_place = py_trees.composites.Selector(
        "다음 일", memory=True,
        children=[stack, route(DOCK, node.loader_exit_route, "나중 — 로더 이탈")])

    return py_trees.composites.Sequence(
        "고정 시나리오", memory=True,
        children=[gate, start, scan, pick, to_loader, place,
                  CycleDone("매거진 완료", node, next_label="스택/도킹 판단"),
                  after_place,
                  route(DOCK, node.dock_route, "도킹"),
                  Idle(DONE, node)])


def current_stage(root):
    """지금 RUNNING 인 잎의 이름.

    저장해 둔 값이 아니라 트리에서 꺼낸 값이라 실제와 어긋날 수 없다.
    """
    running = [b.name for b in root.iterate()
               if b.status == Status.RUNNING and not b.children]
    return running[-1] if running else "-"


# ══════════════════════════════════════════════════════════════════════════
#  노드 — ROS 배선만 한다. 미션 순서는 전부 트리에 있다.
# ══════════════════════════════════════════════════════════════════════════


class TaskManager(Node):

    def __init__(self):
        super().__init__("task_manager")

        self.failed = False
        self._last_snapshot = ""

        self.bb = _blackboard("task_manager", (
            "kind", "variant", "carrier_id", "qr_pose",
            "fail_stage", "fail_reason", "run_id"))
        self.bb.kind = ""
        self.bb.variant = ""
        self.bb.carrier_id = ""
        self.bb.qr_pose = None
        self.bb.run_id = ""
        self.bb.fail_stage = ""
        self.bb.fail_reason = ""

        # 콜백 그룹도 스레드도 없다. 잎이 블로킹하지 않아서 단일 스레드로 충분하다.
        self.carrier_scan = self.create_client(CarrierScan, "perception/carrier_scan")
        self.nav = ActionClient(self, NavigateTo, "navigation/navigate_to")
        self.pick = ActionClient(self, PickCarrier, "manipulation/pick_carrier")
        self.place = ActionClient(self, PlaceCarrier, "manipulation/place_carrier")
        self._state_pub = self.create_publisher(String, "orchestrator/state", 10)

        # ── 생산 트래킹 (docs/DB구성.md §9) ──────────────────────────────
        # /trace/event 만 절대이름이다. event_logger 는 전역 1개라, 상대이름이면
        # 로봇마다 다른 토픽이 되어 로봇을 늘릴 때마다 로거를 고쳐야 한다.
        self.robot_id = self.get_namespace().strip("/") or "robot1"
        self._trace_pub = self.create_publisher(TraceEvent, "/trace/event", 50)
        # use_sim_time 이 켜져 있어도 벽시계를 따로 읽는다 — 둘 다 DB 에 들어간다
        self._wall_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self._attempts = {}          # stage -> 시도 횟수. new_run() 이 비운다
        self._frozen_node = None     # 얼어붙은 Freeze. _on_resume 이 푼다
        self.create_service(SetBool, "orchestrator/resume", self._on_resume)

        # ── 좌표 (근거는 각 DEFAULT_* 상수 주석) ─────────────────────────
        # ★ 트리 조립보다 먼저다 — build_tree 가 이 값들을 읽는다.
        self.declare_parameter("scan_route", DEFAULT_SCAN_ROUTE)
        self.declare_parameter("approach_route", DEFAULT_APPROACH_ROUTE)
        self.declare_parameter("loader_exit_route", DEFAULT_LOADER_EXIT_ROUTE)
        self.declare_parameter("stack_pick_route", DEFAULT_STACK_PICK_ROUTE)
        self.declare_parameter("stack_deliver_route", DEFAULT_STACK_DELIVER_ROUTE)
        self.declare_parameter("dock_route", DEFAULT_DOCK_ROUTE)
        self.declare_parameter("stack_place_port", DEFAULT_STACK_PLACE_PORT)
        self.declare_parameter("lane_priority", DEFAULT_LANE_PRIORITY)
        self.declare_parameter("start_clearance_m", DEFAULT_START_CLEARANCE_M)
        self.scan_route = _unflatten_route(self.get_parameter("scan_route").value)
        self.approach_route = _unflatten_route(
            self.get_parameter("approach_route").value)
        self.loader_exit_route = _unflatten_route(
            self.get_parameter("loader_exit_route").value)
        self.stack_pick_route = _unflatten_route(
            self.get_parameter("stack_pick_route").value)
        self.stack_deliver_route = _unflatten_route(
            self.get_parameter("stack_deliver_route").value)
        self.dock_route = _unflatten_route(self.get_parameter("dock_route").value)
        self.stack_place_port = self.get_parameter("stack_place_port").value
        self.lane_priority = int(self.get_parameter("lane_priority").value)
        self.start_clearance_m = float(self.get_parameter("start_clearance_m").value)

        # ── 로더 차선 조율 (두 대가 같은 로더로 갈 때) ────────────────────
        self.declare_parameter("peer_state_topic", "")
        self.declare_parameter("peer_pose_topic", "")
        self.declare_parameter("loader_clear_radius_m", DEFAULT_LOADER_CLEAR_RADIUS_M)
        self.declare_parameter("peer_busy_stages", DEFAULT_PEER_BUSY_STAGES)
        self.loader_clear_radius_m = float(
            self.get_parameter("loader_clear_radius_m").value)
        # 빈 문자열은 걸러낸다 — rclpy 는 빈 리스트의 타입을 못 정해서 [""] 로
        # 넘기는 경우가 있고, 그게 그대로 들어오면 아무 단계에도 안 맞는다.
        self._peer_busy_stages = tuple(
            x for x in self.get_parameter("peer_busy_stages").value if x)
        self._peer_stage = None       # 마지막으로 본 상대 단계
        self._peer_xy = None          # 상대 베이스 위치 (map). 없으면 None
        self._peer_failed = False     # 상대가 얼어붙었나 (FAILED 토큰)
        self._peer_prio = 0           # 상대의 lane_priority. 0 = 없음
        self._peer_last_rx = 0.0      # 마지막 수신 시각. 0 = 한 번도 못 받음
        self._peer_topic = self.get_parameter("peer_state_topic").value
        if self._peer_topic:
            # ★ 절대이름이다. 상대의 orchestrator/state 를 그대로 읽는다.
            #   상대이름으로 두면 자기 자신을 구독한다.
            self.create_subscription(
                String, self._peer_topic, self._on_peer_state, 10)
            self.get_logger().info(
                f"로더 차선 조율 켜짐 — 구독 {self._peer_topic}, "
                f"양보 대상 {list(self._peer_busy_stages)}, "
                f"이탈 판정 거리 {self.loader_clear_radius_m:.2f} m")
        # ★ 이 노드가 위치를 구독하는 유일한 자리다. 원래 task_manager 는 로봇
        #   위치를 모른다(기하는 navigation·manipulation 담당). 예외를 둔 이유는
        #   "상대가 차선을 비켰나" 와 "상대가 도크를 떠났나" 를 시뮬 속도와
        #   무관하게 판정하려면 시간이 아니라 거리를 봐야 하기 때문이다.
        peer_pose_topic = self.get_parameter("peer_pose_topic").value
        if peer_pose_topic:
            self.create_subscription(
                PoseWithCovarianceStamped, peer_pose_topic, self._on_peer_pose, 10)
            self.get_logger().info(f"상대 위치 구독 {peer_pose_topic}")
        elif self._peer_topic:
            self.get_logger().warning(
                "peer_pose_topic 이 비어 있다 — 상대가 차선을 비웠는지, 도크를 "
                "떠났는지를 거리로 못 잰다. 출발 게이트는 시간으로, 대기는 상태 "
                "변화로만 푼다. 안전하지만 느리다.")
        else:
            self.get_logger().info(
                "peer_state_topic 이 비어 있다 — 조율 없이 혼자 돈다 "
                "(한 대만 띄울 때의 기본값)")

        self.tree = py_trees.trees.BehaviourTree(build_tree(self))
        self.tree.add_post_tick_handler(self._on_post_tick)
        self.tree.setup()

        self.create_timer(TICK_PERIOD_S, self.tree.tick)
        self.create_timer(STATE_PUBLISH_PERIOD_S, self._publish_state)

        self.get_logger().info("task_manager ready — 행동트리 tick 시작 (순찰 없음)")
        self._log_routes()

    def _log_routes(self):
        """시작 로그 — 어느 좌표로 도는지 한 번에 보인다."""
        def fmt(route):
            return " → ".join(f"({x:.3f}, {y:.3f}, {yaw:.1f}°)" for x, y, yaw in route)
        self.get_logger().info(
            f"경로 — 출발 {fmt(self.scan_route)}"
            f" | 로더행 {fmt(self.approach_route)} → "
            f"({TEST_LOADER[0]:.2f}, {TEST_LOADER[1]:.2f}, {TEST_LOADER[2]:.1f}°)"
            f" | 스택 {fmt(self.stack_pick_route)}"
            f" | 검사 {fmt(self.stack_deliver_route)}"
            f" | 이탈 {fmt(self.loader_exit_route)} | 도킹 {fmt(self.dock_route)}")
        self.get_logger().info(
            f"조율 — lane_priority={self.lane_priority or '없음'} (1 이 먼저) · "
            f"출발 여유 {self.start_clearance_m:.2f} m · "
            f"대기 자리 {tuple(round(v, 3) for v in self.approach_route[-1])}")
        if self.lane_priority == 0 and self._peer_topic:
            self.get_logger().warning(
                "lane_priority 가 0 이다 — 두 대가 동시에 PICK 을 마치면 둘 다 "
                "직행을 골라 로더에서 만난다. launch 의 LANE_PRIORITY_BY_ROBOT 참고.")

    # ── 트리가 부르는 것들 ────────────────────────────────────────────────
    def on_freeze(self, stage, reason, node=None):
        """Freeze 가 얼어붙을 때 한 번 부른다."""
        self.failed = True
        self._frozen_node = node
        self.bb.fail_stage = stage
        self.bb.fail_reason = reason
        self.get_logger().error(
            f"[실패] {stage} 단계에서 멈췄다. 이유: {reason} — "
            f"그 자리에서 정지한다. 자동 복귀하지 않는다. 복구하려면 "
            f"/{self.robot_id}/orchestrator/resume 에 true 를 보낸다.")
        self._publish_state()

    # ── 생산 트래킹 (docs/DB구성.md §4-7 · §9) ────────────────────────────
    def now_pair(self):
        """(시뮬 시각, 벽시계) 한 쌍. 둘 다 builtin_interfaces/Time.

        use_sim_time 이 켜져 있으면 get_clock() 은 /clock 을 따른다 — GPU 머신의
        Isaac 이 발행하는 값이다. 두 머신 사이 DDS 가 안 뚫려 /clock 이 안 오면
        0 이 나오고, DB 의 duration_sec 이 전부 0 이 된다. 에러는 안 난다.
        """
        return (self.get_clock().now().to_msg(), self._wall_clock.now().to_msg())

    def new_run(self):
        """SCAN 이 성공한 순간 = 미션 하나의 시작. run_id 를 새로 발행한다."""
        self.bb.run_id = str(uuid.uuid4())
        self._attempts = {}          # 미션이 바뀌면 attempt 도 1 로 돌아간다
        self.get_logger().info(
            f"미션 시작 run={self.bb.run_id[:8]} carrier={self.bb.kind}")
        return self.bb.run_id

    def attempt_of(self, stage):
        return self._attempts.get(stage, 1)

    def _status_of(self, stage, ok):
        """그 시점 '물체' 의 상태. 단계의 성패와는 다른 값이다."""
        stage = TRACE_STAGE.get(stage, stage)
        if not ok:
            return "FAILED"
        return "COMPLETED" if stage == PLACE else "IN_TRANSIT"

    def emit_trace(self, stage, leaf, ok, fail_detail=""):
        """단계 하나가 끝날 때마다 한 건. Freeze 가 부른다."""
        sim_now, wall_now = self.now_pair()
        msg = TraceEvent()
        msg.started_stamp = getattr(leaf, "started_stamp", sim_now)
        msg.stamp = sim_now
        msg.started_wall = getattr(leaf, "started_wall", wall_now)
        msg.wall_stamp = wall_now
        msg.carrier_id = self.bb.kind or ""          # QR 원문 그대로
        msg.robot_id = self.robot_id
        msg.run_id = self.bb.run_id or ""
        msg.stage = TRACE_STAGE.get(stage, stage)    # 스택 단계는 DB 어휘로
        msg.attempt = self.attempt_of(stage)
        msg.success = bool(ok)
        msg.status = self._status_of(stage, ok)
        msg.fail_reason = "" if ok else STAGE_TO_REASON.get(stage, "nav_error")
        msg.fail_detail = "" if ok else (fail_detail or "UNKNOWN")
        msg.port = (self.stack_place_port if stage == STACK_PLACE
                    else PORT_BY_STAGE.get(stage, ""))
        self._trace_pub.publish(msg)

    def _on_resume(self, req, res):
        """웹 복구 — 얼어붙은 그 단계를 다시 시도한다.

        새 미션이 아니다. Freeze 가 얼어 있는 동안 RUNNING 을 돌려주므로
        memory=True 인 Sequence 가 그 잎을 붙들고 있고, frozen 을 풀면 자식이
        initialise() 부터 다시 돌아 goal 이 새로 나간다 — 앞 단계는 재실행되지
        않는다. 그래서 run_id 는 그대로 두고 attempt 만 올린다.
        """
        if not req.data:
            res.success = False
            res.message = "포기(false)는 미구현 — true 로 재개만 된다"
            return res
        node = self._frozen_node
        if node is None:
            res.success = False
            res.message = "얼어붙은 단계가 없다"
            return res

        self._attempts[node.stage] = self.attempt_of(node.stage) + 1
        node.frozen = False
        self._frozen_node = None
        self.failed = False
        self.bb.fail_stage = ""
        self.bb.fail_reason = ""
        self._publish_state()
        res.success = True
        res.message = f"{node.stage} 재개 (attempt={self.attempt_of(node.stage)})"
        self.get_logger().warning(f"[복구] {res.message}")
        return res

    def begin_cancel(self, goal_handle, who):
        """진행 중인 goal 을 거둔다. 취소를 보내기만 한다."""
        self.get_logger().info(f"진행 중인 goal 취소 ({who})")
        goal_handle.cancel_goal_async()

    def log_phase(self, label):
        return lambda fb: self.get_logger().info(f"  {label} phase={fb.feedback.phase}")

    # ── 표시 ──────────────────────────────────────────────────────────────
    def _on_post_tick(self, tree):
        snapshot = py_trees.display.unicode_tree(tree.root, show_status=True)
        if snapshot != self._last_snapshot:
            self._last_snapshot = snapshot
            self.get_logger().info("\n" + snapshot)

    # ── 로더 차선 조율 ────────────────────────────────────────────────────
    def _on_peer_state(self, msg):
        """상대 task_manager 의 orchestrator/state 를 읽는다.

        문자열 형식은 _publish_state 가 만드는 것과 같다. 모르는 필드는 무시한다.
        """
        stage, failed, prio = None, False, 0
        for part in msg.data.split("|"):
            part = part.strip()
            if part.startswith("state="):
                stage = part[len("state="):]
            elif part == "FAILED":
                failed = True
            elif part.startswith("prio="):
                try:
                    prio = int(part[len("prio="):])
                except ValueError:
                    prio = 0
        self._peer_stage = stage
        self._peer_failed = failed
        self._peer_prio = prio
        self._peer_last_rx = time.monotonic()

    def _on_peer_pose(self, msg):
        self._peer_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def peer_stage(self):
        return self._peer_stage

    def peer_seen(self):
        """상대 상태를 한 번이라도 받았나."""
        return self._peer_last_rx != 0.0

    def peer_xy(self):
        return self._peer_xy

    def peer_prio(self):
        return self._peer_prio

    def peer_outranks_me(self):
        """상대가 나보다 선순위인가 (lane_priority 는 1 이 가장 먼저).

        어느 한쪽이라도 우선순위가 없으면(0) False 다 — 그러면 "지금 차선을
        쓰는가" 규칙만으로 판단한다.
        """
        return (self.lane_priority > 0 and self._peer_prio > 0
                and self._peer_prio < self.lane_priority)

    def peer_contending(self):
        """상대가 아직 차선 판단 전이라 곧 로더로 오는가 (CONTENDING_STAGES)."""
        return self.peer_seen() and self._peer_stage in CONTENDING_STAGES

    def peer_dist_to_loader(self):
        """상대 베이스와 로더 주차점의 거리. 위치를 모르면 None."""
        if self._peer_xy is None:
            return None
        return math.hypot(self._peer_xy[0] - TEST_LOADER[0],
                          self._peer_xy[1] - TEST_LOADER[1])

    def peer_frozen(self):
        """상대가 차선 안에서 얼어붙었나.

        얼어붙은 자리가 차선 안이면 기다려도 안 비켜진다 — Freeze 는 tick 마다
        RUNNING 만 돌려주고 자동 복귀가 없으므로 사람이 상대를 풀어야 한다.
        그래서 기다리지 않고 이쪽도 실패로 올려 같이 웹에 뜨게 한다.
        """
        return bool(self._peer_failed
                    and self._peer_stage in self._peer_busy_stages)

    def peer_busy(self):
        """(양보해야 하나, 사람이 읽을 이유) 한 쌍.

        한 번도 못 받았으면 상대가 안 떠 있다고 보고 통과시킨다 — 한 대만
        띄우고 영원히 기다리는 걸 막는다.

        ★ 소식이 끊겨도 마지막으로 들은 단계를 그대로 쓴다. 위험한 방향은 이미
          막혀 있다 — 상대가 차선 단계에서 죽으면 마지막 메시지가 그 단계이므로
          계속 양보한다. 반대로 차선 밖에서 죽은 상대까지 양보 대상으로 만들면
          죽은 로봇 때문에 살아 있는 로봇이 대기 자리에서 얼어붙는다.
        """
        if not self._peer_busy_stages or self._peer_last_rx == 0.0:
            return False, "상대 없음"
        age = time.monotonic() - self._peer_last_rx
        if self._peer_stage in self._peer_busy_stages:
            return True, f"state={self._peer_stage} ({age:.1f}s 전)"
        return False, f"state={self._peer_stage} ({age:.1f}s 전)"

    def publish_state(self):
        """잎이 자기 상태를 즉시 알려야 할 때 부른다.

        다음 주기를 기다리면 그 사이 상대가 묵은 값으로 출발 판단을 한다.
        """
        self._publish_state()

    def _publish_state(self):
        # 얼어붙었으면 그 단계가 상태다. Freeze 는 데코레이터라 잎이 아니어서
        # current_stage() 에 안 잡힌다.
        stage = self.bb.fail_stage if self.failed else current_stage(self.tree.root)
        msg = String()
        parts = [f"state={stage}"]
        if self.lane_priority:
            # 순서 장치. 상대의 PeerClear · HOLD_BACK · 출발 게이트가 읽는다.
            parts.append(f"prio={self.lane_priority}")
        if self.bb.carrier_id:
            parts.append(f"carrier={self.bb.carrier_id}")
        if self.bb.run_id:
            # 사람이 보는 용도. DB 의 run_id 앞 8자와 같아서 로그와 표를 맞춰본다
            parts.append(f"run={self.bb.run_id[:8]}")
        if self.bb.variant:
            parts.append(f"variant={self.bb.variant}")
        if self.failed:
            parts.append("FAILED")
            parts.append(f"fail_stage={self.bb.fail_stage}")
            parts.append(f"fail_reason={self.bb.fail_reason}")
        msg.data = " | ".join(parts)
        self._state_pub.publish(msg)


def main():
    rclpy.init()
    node = TaskManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
