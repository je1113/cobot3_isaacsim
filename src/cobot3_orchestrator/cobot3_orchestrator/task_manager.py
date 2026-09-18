"""
task_manager — orchestrator 행동트리(Behavior Tree).

docs/08_ROS2_NODE_Graph.html 의 확정안(01-03)이 기준이다. 노드 5개 중
"미션 어휘는 전부 여기" 에 해당하는 노드이고, 나머지 셋은 좌표 하나 또는
종류 하나만 받는다.

    ros2 run cobot3_orchestrator task_manager

이전 판은 상태기계였다. 상태(patrol/hold/scan/pick/nav/place)를 문자열로 들고
워커 스레드에서 단계 함수를 차례로 불렀다. 이 판은 같은 순서를 행동트리로 쓴다.
바깥에서 보이는 것(토픽·서비스·액션 이름, goal 내용, 실패 시 거동)은 그대로다.

행동트리가 뭔가 — 이 파일을 읽는 데 필요한 만큼만
  트리를 일정 주기로 뿌리부터 "tick" 한다(여기서는 10 Hz). tick 을 받은 노드는
  SUCCESS / FAILURE / RUNNING 셋 중 하나를 돌려준다. RUNNING 이 핵심이다 —
  "아직 하는 중" 을 상태 변수에 저장하는 대신 잎이 tick 마다 RUNNING 을 돌려주는
  것으로 표현한다. 그래서 어떤 잎도 블로킹하면 안 된다. 기다리지 말고 future 를
  확인하고 바로 RUNNING 을 돌려줘야 한다.

    Sequence [→]  왼쪽부터. SUCCESS 면 다음, 하나라도 FAILURE 면 즉시 FAILURE.
    Selector [?]  왼쪽부터. FAILURE 면 다음, SUCCESS/RUNNING 이면 거기서 멈춤.
                  = 우선순위. 위에 있을수록 먼저 기회를 받는다.

이 노드의 트리

    [?] 우선순위                      Selector, memory=False
     ├─ [→] 캐리어 처리                Sequence, memory=True
     │   ├─ detected?                  QR 이 보인다는 신호가 와 있나
     │   ├─ HOLD                       로봇이 실제로 설 때까지 기다린다
     │   ├─ SCAN                       carrier_scan 서비스 → 종류·위치
     │   ├─ PICK                       멈춘 그 자리에서 집는다
     │   ├─ NAV                        목적지로 주행
     │   ├─ PLACE                      배치
     │   ├─ RETURN                     순찰 시작 좌표로 복귀
     │   └─ 사이클 완료
     └─ [→] 순찰 가지                  Sequence, memory=True
         ├─ START (1회)                시작 자리 → 순찰 첫 정차점. 평생 한 번만
         │                             나간다 (OneShot). 노드가 뜬 자리는 순찰
         │                             경로 위가 아니라서 필요하다.
         └─ 순찰                       정차점을 순서대로 돈다. 끝나지 않는다.

  최상위가 memory=False 인 이유: tick 마다 맨 위부터 다시 검사하라는 뜻이다.
  그래서 순찰이 RUNNING 인 중에도 "detected?" 가 매 tick 재검사된다. 신호가 오는
  순간 Selector 가 캐리어 처리 가지를 고르고, 밀려난 순찰 잎에 terminate(INVALID)
  가 불린다 — 거기서 진행 중인 NavigateTo goal 을 취소한다. 이게 HOLD 다.
  "감지되면 주행을 멈춰라" 를 어디에도 적지 않았는데 구조에서 나온다.

  신호를 언제 받나: carrier_detected 는 "순찰" 이 RUNNING 일 때만 받는다.
  이전 판의 "if self.state != PATROL: return" 과 같은 규칙이고, 그래서 START 로
  이동하는 중이나 작업 중에 들어온 신호는 버려진다. 물건은 그 자리에 그대로
  있으므로 다음 순찰에 다시 보인다.

  캐리어 처리가 memory=True 인 이유: PICK 이 RUNNING 인 동안 앞의 HOLD·SCAN 을
  다시 tick 하면 안 된다. memory=True 면 지난 tick 에 RUNNING 이던 자식부터
  이어친다. 위의 최상위 Selector 와 조합하면 "픽 하는 중에도 우선순위는 매 tick
  보되, 이미 끝난 스캔은 다시 하지 않는다" 가 된다.

상태 저장
  "지금 어느 단계인가" 는 저장하지 않는다. 어느 잎이 RUNNING 인지가 답이고,
  트리를 찍으면 그게 실제다. 이전 판의 self._state 는 따로 갱신해 줘야 하는
  값이라 실제와 어긋날 수 있었다 (실패 후 상태는 'scan' 인데 실제로는 아무것도
  하지 않는 상태였다).

  "이번 미션의 데이터" 는 블랙보드(py_trees 의 공유 저장소)에 둔다.
      detected     carrier_detected 신호가 와 있나
      kind         QR 원문. 이 씬에서는 "1" 또는 "2" — 물체의 종류
      variant      kind 를 grasp.yaml(어떻게집나)/place.yaml(어디에놓나) 키로 푼 것
      carrier_id   carriers.yaml(어느개체가어느종류인가) 조회 결과. 로그·표시용
      qr_pose      PickCarrier 가 플랜지를 찾을 탐색 창 prior
      fail_stage   어느 단계에서 멈췄나
      fail_reason  왜 멈췄나
      patrol_target 지금 향하는 PATROL_ROUTE 인덱스 — 아래 계약
      scan_fail_streak SCAN 이 연속 몇 번 미판독이었나

    ros2 run py_trees_ros_tutorials 없이도 아래로 볼 수 있다:
        ros2 topic echo /orchestrator/state

사용하는 인터페이스 — 08 문서에서 확정된 것만 쓴다. 이전 판과 동일하다.
  구독  /perception/carrier_detected  std_msgs/Bool
  호출  /perception/carrier_scan      CarrierScan.srv
  액션  /navigation/navigate_to       NavigateTo.action
  액션  /manipulation/pick_carrier    PickCarrier.action
  액션  /manipulation/place_carrier   PlaceCarrier.action
  발행  /orchestrator/state           std_msgs/String  (상태 · 실패 단계 확인용)

★ /orchestrator/state 는 디버그 토픽이 아니라 계약이다
  cobot3_perception/carrier_code_reader 가 이걸 구독해서 두 가지를 판정한다.
  문자열 모양을 바꾸면 그 노드가 깨진다.

    state=<단계>         그 노드의 _detect_tick 은 이 값이 정확히 "patrol" 일
                         때만 QR 폴링을 돈다. _on_orchestrator_state 는 값이
                         "patrol" 이 아니게 되면 팔 자세를 다시 잡도록
                         disarm 한다. 그래서 순찰 중에는 정확히 "patrol" 이어야
                         한다 — 트리의 순찰 잎 이름을 PATROL("patrol") 로 둔
                         이유가 이것이다. start · hold · scan · pick · nav ·
                         place · return 은 전부 "patrol 아님" 으로 취급된다.
    patrol_target=<0|1>  그 노드의 POSE_BY_PATROL_TARGET 이 이걸로 층별 관측
                         자세를 고른다 — 끝점(1)으로 가는 중이면 2층, 시작점(0)
                         으로 돌아가는 중이면 1층. 순찰 잎이 정차점을 고르는
                         즉시 발행한다(set_patrol_target).

  나머지(detected · carrier · variant · scan_fail · FAILED · fail_stage ·
  fail_reason)는 사람이 보는 용도다. 트리가 바뀔 때마다 트리 그림도 로그로
  남는다. std_msgs/String 이라 cobot3_interfaces 에는 파일이 생기지 않는다.

실패하면
  로봇을 멈추고 (진행 중인 goal 을 취소) 그 자리에서 정지한다. 자동 복귀도
  재시도도 하지 않는다. 이전 판과 같은 정책이고, 얼어붙는 자리가 "상태 문자열"
  이 아니라 "트리의 그 노드" 라는 것만 다르다 — Freeze 데코레이터가 그 뒤로
  tick 마다 RUNNING 만 돌려준다. 어느 단계에서 왜 멈췄는지는 에러 로그와
  /orchestrator/state 토픽 두 곳에서 확인한다.

  예외가 하나 있다. SCAN 의 found=false 는 멈추지 않고 patrol 로 돌아간다.
  대신 SCAN_COOLDOWN_S 동안 carrier_detected 를 받지 않는다 — 안 그러면 다음
  순찰에 같은 자리에 같은 이유로 또 서서 무한히 반복한다. 연속
  SCAN_FAIL_WARN 회 실패하면 경고를 낸다(멈추지는 않는다).
  CarrierScan.srv 응답 주석이 정한 거동이다("세우는 사이 라벨이 시야에서
  벗어났거나 다수결을 못 채운 경우다. orchestrator 는 이때 HOLD 를 풀고 patrol
  로 돌아간다"). 그 외 스캔 실패(SERVICE_UNAVAILABLE · TIMEOUT ·
  UNKNOWN_PAYLOAD)는 다른 단계와 똑같이 그 자리에서 정지한다.

붙어 있는 상대 노드
  /navigation/navigate_to     cobot3_navigation/nav_server
                              선반 구역은 Nav2 대신 cmd_vel 로 직접 몬다. 취소는
                              양쪽 경로 다 받는다(_on_cancel ACCEPT, 직접주행
                              루프의 is_cancel_requested) — 그래서 순찰 중 선점이
                              안전하다.
  /perception/carrier_detected  carrier_code_reader 가 SHELF_ZONE 안에서
                              state=patrol 인 동안 폴링하다가 디코드에 성공한
                              그 순간 한 번 발행한다. 즉 신호가 온 자리가 읽히는
                              자리다 — 그래서 이 노드는 그 자리에서 바로 세운다.
                              정차점까지 더 가고 싶으면 아래 상수 하나를 끈다.

알려진 갭
  - 이 씬의 QR 은 숫자 하나("1"/"2")만 담아서 같은 종류의 여러 개체를 구분하지
    못한다. carriers.yaml 에서 그 종류의 첫 항목을 쓴다.
  - 순찰 중이 아닐 때(START · scan · pick · nav · place · return) 들어온
    carrier_detected 는 버린다. 물건은 그 자리에 그대로 있으므로 다음 순찰에
    다시 보인다.
  - TraceEvent 발행과 배터리·도킹 선점 가지는 아직 없다. 둘 다 트리에 가지
    하나씩 더하는 자리가 이미 나 있다 (build_tree 참고).
"""

import math
import time
from pathlib import Path

import py_trees
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from py_trees.common import Access, Status
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String

from cobot3_interfaces.action import NavigateTo, PickCarrier, PlaceCarrier
from cobot3_interfaces.srv import CarrierScan

# ══════════════════════════════════════════════════════════════════════════
#  좌표 — 아직 안 정해졌다. 아래 두 곳을 채워 넣어야 로봇이 실제로 움직인다.
#
#  좌표 형식은 둘 다 같다:   (x, y, yaw_deg)
#      x, y      map 프레임 기준 위치, 단위 m
#      yaw_deg   그 자리에서 로봇이 바라볼 방향, 단위 도(degree).
#                +x 축이 0도이고 반시계 방향이 +. 예: 90.0 이면 +y 를 본다.
#  세 값 모두 float 이다. 아래 _to_pose() 가 이걸 NavigateTo goal 의
#  PoseStamped(frame_id="map") 로 바꿔서 보낸다.
# ══════════════════════════════════════════════════════════════════════════

# 순찰 경로 — patrol 상태에서 이 좌표들을 순서대로 돌고, 끝까지 가면 다시
# 처음으로 돌아가 반복한다. QR 은 0.28 m 안에서만 읽히므로(05 §2-4) 각 정차점은
# 선반 라벨 가까이에, 그리고 멈춘 자리에서 팔이 매거진에 닿는 거리에 잡아야 한다
# — 픽하러 따로 이동하지 않고 선 그 자리에서 집기 때문이다(M0609 도달거리 900 mm).
#
# 개활지 회전 웨이포인트를 넣은 순환 경로(이전 버전)는 그 전환 구간 자체가
# 깔끔하게 안 돌아서(실측: 방향이 몇 초 사이 수십 도씩 흔들림) 걷어냈다.
# 대신 실제로 RViz/Isaac 에서 로봇을 직접 움직여 확인한 좌표 두 개로 단순
# 직선 왕복을 쓴다 — 계산으로 추정한 좌표보다 이게 더 믿을 만하다.
#   시작점: 2026-09-18 실측 (-2.674, 1.613)
#   끝점:   2026-09-18 실측, 선반1 관측 자세 근처 (-6.582, 1.344)
# 두 점 다 yaw=0 으로 고정 — frames.yaml 의 관측/파지 자세(observation_poses)가
# 그 자세로 티칭돼 있어서다. 왕복 중 반대 방향으로 갈 때 정차점에서 제자리
# 회전이 필요한 문제(footprint 스윕 0.66m vs 선반 여유 0.42~0.48m)는 아직
# 해결 안 됐다 — nav_server 의 저속 프로파일 + 도착후 미세정렬로 완화만 됐다.
PATROL_ROUTE = [
    (-2.674, 1.613, 0.0),
    (-6.582, 1.344, 0.0),
]

# place 하러 갈 목적지 — 테스트 스테이션 로더 앞 주차 위치.
# 지금은 매거진 1 · 2 를 전부 여기로 가져다 놓는다. (08 문서의 "매거진은 패키징
# 로더로" 규칙은 지금 적용하지 않는다 — pkg_loader 는 나중에 추가한다.)
TEST_LOADER = (4.0, 0.0, 0.0)

# ── 단계 이름 ─────────────────────────────────────────────────────────────
# 이전 판의 "상태" 다. 지금은 상태가 아니라 라벨이다 — 트리의 어느 노드인지,
# 그리고 실패 기록(fail_stage)에 어느 단계였는지 적는 데만 쓴다.
START = "start"
PATROL = "patrol"
HOLD = "hold"
SCAN = "scan"
PICK = "pick"
NAV = "nav"
PLACE = "place"
RETURN = "return"

# QR payload -> grasp.yaml / place.yaml 의 variant 키.
# 이 씬의 QR 은 "1" 또는 "2" 한 글자만 담고 있다.
NUMERIC_TO_VARIANT = {"1": "magazine_1_orange", "2": "magazine_2_blue"}

# 각 단계를 이만큼 기다려도 안 끝나면 실패로 본다. 단위 초.
SCAN_TIMEOUT_S = 10.0
NAV_TIMEOUT_S = 300.0
PICK_TIMEOUT_S = 120.0
PLACE_TIMEOUT_S = 120.0
SERVER_WAIT_S = 5.0

# SCAN 이 found=false 로 끝난 뒤 이만큼은 carrier_detected 를 받지 않는다.
# 라벨이 판독거리 밖에 서게 되는 자리는 기하로 정해져 있어서, 그냥 patrol 로
# 돌려보내면 다음 순찰에 같은 자리에 같은 이유로 또 선다 — 무한히 반복한다.
# task_manager 는 로봇 위치를 모르므로(odom·TF 구독 없음, 기하는 manipulation
# 담당) "그 자리" 를 좌표로 기억하지 못한다. 시간으로 대신한다.
SCAN_COOLDOWN_S = 30.0
# 연속 이만큼 실패하면 경고를 낸다. 쿨다운만 두면 로봇이 조용히 계속 도는데,
# 그건 "안 보이는 실패" 라 더 나쁘다. 멈추지는 않는다 — 다른 라벨은 처리해야 한다.
SCAN_FAIL_WARN = 3

# carrier_detected 가 오면 주행을 그 자리에서 끊을지, 정차점까지 가고 나서
# 처리할지.
#   True  — 그 자리에서 끊는다. carrier_code_reader 는 디코드에 성공한 순간
#           발행하므로 신호가 온 자리가 곧 읽히는 자리다. 더 가면 라벨이
#           판독거리(0.28 m, 05 §2-4) 밖으로 나간다. nav_server 가 Nav2 경로와
#           선반구역 cmd_vel 직접주행 양쪽에서 취소를 받으므로 안전하다.
#   False — 436dbe5 "주행 중 QR 감시 제거 — 정차점 도착 후 처리" 의 거동.
#           신호는 받아 두고 정차점에 선 뒤에 처리한다.
PREEMPT_WHILE_DRIVING = True

# 트리를 이 주기로 tick 한다. 잎이 블로킹하지 않으므로 한 tick 은 수십 마이크로초다.
# 반응 지연의 상한이 이 값이다 — carrier_detected 가 와서 주행 취소가 나가기까지
# 최대 0.1 초.
TICK_PERIOD_S = 0.1


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
CARRIERS_YAML = WS_ROOT / "src/cobot3_bringup/config/carriers.yaml"


def _reason_name(result_cls, code):
    # 액션 result 의 fail_reason 코드를 .action 에 적힌 상수명(사람이 이해할 수
    # 있는 버전)으로 바꾼다.
    for name in dir(result_cls):
        if not name.isupper():
            continue
        value = getattr(result_cls, name, None)
        if isinstance(value, int) and value == code:
            return f"{name}({code})"
    return str(code)


def _to_pose(xy_yaw_deg):
    # 입력해준 목적지 (x, y, yaw_deg) -변환-> NavigateTo goal 의 PoseStamped(map)
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
#  done() 을 확인하고 RUNNING 을 돌려준다. 이전 판의 _wait() 가 하던
#  done.wait(timeout=...) 이 여기서는 금지다 — 그게 블로킹하는 동안에는
#  트리가 위쪽 우선순위를 검사하지 못하기 때문이다.
# ══════════════════════════════════════════════════════════════════════════


class ActionLeaf(py_trees.behaviour.Behaviour):
    """액션 goal 하나. NavigateTo · PickCarrier · PlaceCarrier 가 전부 이걸 쓴다.

    셋 다 result 가 success · fail_reason 모양이라 하나로 된다.

    make_goal 은 initialise() 에서 한 번만 불린다. 순찰 잎이 이걸로 정차점을
    하나씩 전진시킨다.

    ok_fail_reasons 에 든 fail_reason 은 실패로 치지 않고 SUCCESS 로 넘긴다.
    이전 판 _patrol_step() 의 "CANCELED = 의도된 중단이라 조용히 넘어간다" 가
    이것이다.
    """

    def __init__(self, name, node, client, server_name, result_cls, make_goal,
                 timeout_s, feedback_cb=None, ok_fail_reasons=(), moves_base=False):
        super().__init__(name)
        self.node = node
        self.client = client
        self.server_name = server_name
        self.result_cls = result_cls
        self.make_goal = make_goal
        self.timeout_s = timeout_s
        self.feedback_cb = feedback_cb
        self.ok_fail_reasons = tuple(ok_fail_reasons)
        self.moves_base = moves_base
        self.soft = False
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

    def update(self):
        if time.monotonic() > self.deadline:
            self.feedback_message = f"TIMEOUT({self.timeout_s:.0f}s)"
            return Status.FAILURE

        # 1) 서버를 기다린다. 이전 판의 wait_for_server(timeout_sec=5) 인데,
        #    블로킹하지 않고 tick 마다 확인만 한다.
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
            if self.moves_base:
                # Hold 가 "베이스가 아직 움직이나" 를 이걸로 판단한다.
                self.node.set_driving(self.result_future)

        # 3) result 를 기다린다.
        if not self.result_future.done():
            return Status.RUNNING
        result = self.result_future.result().result
        if result.success:
            return Status.SUCCESS
        reason = _reason_name(self.result_cls, result.fail_reason)
        if result.fail_reason in self.ok_fail_reasons:
            self.feedback_message = f"{reason} — 실패로 치지 않는다"
            return Status.SUCCESS
        self.feedback_message = reason
        return Status.FAILURE

    def terminate(self, new_status):
        """끊겼거나 실패했으면 진행 중인 goal 을 거둔다.

        INVALID 는 우선순위 선점이다 — 순찰 중에 carrier_detected 가 와서
        Selector 가 캐리어 처리 가지를 고르면 여기가 불린다. 이전 판의
        _step_hold() → _cancel_active_goal() 이 하던 일이고, 이제는 "멈춰라" 를
        누가 말해주지 않아도 트리가 가지를 바꾸는 것만으로 나간다.

        FAILURE 는 이전 판 _fail() 이 _cancel_active_goal() 을 부르던 자리다.
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

    멈춘 뒤에 읽어야 정확하다 (05 §12-1 정지 상태에서 근접 판독). 앞의 HOLD 잎이
    로봇이 실제로 설 때까지 RUNNING 을 돌려주므로 여기 올 때는 이미 서 있다.

    found=false 는 soft 실패다 — Freeze 가 얼리지 않고 FAILURE 를 그대로 올려서
    캐리어 처리 가지가 통째로 FAILURE 가 되고, Selector 가 순찰로 넘어간다.
    CarrierScan.srv 응답 주석이 정한 거동이다.
    """

    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.bb = _blackboard(name, ("kind", "variant", "carrier_id", "qr_pose"))
        self.soft = False
        self.future = None

    def initialise(self):
        self.future = None
        self.soft = False
        now = time.monotonic()
        self.server_deadline = now + SERVER_WAIT_S
        self.deadline = now + SCAN_TIMEOUT_S

    def update(self):
        if time.monotonic() > self.deadline:
            self.feedback_message = f"TIMEOUT({SCAN_TIMEOUT_S:.0f}s)"
            return Status.FAILURE

        if self.future is None:
            if not self.node.carrier_scan.service_is_ready():
                if time.monotonic() > self.server_deadline:
                    self.feedback_message = "SERVICE_UNAVAILABLE(/perception/carrier_scan)"
                    return Status.FAILURE
                self.feedback_message = "서비스 대기"
                return Status.RUNNING
            self.future = self.node.carrier_scan.call_async(CarrierScan.Request())

        if not self.future.done():
            return Status.RUNNING

        res = self.future.result()
        if not res.found:
            # 세우는 사이 라벨이 시야에서 벗어났거나 다수결을 못 채웠다.
            # 멈추지 않고 순찰로 돌아간다 — 물건은 그 자리에 그대로 있다.
            self.soft = True
            self.node.on_scan_not_found()
            self.feedback_message = "NOT_FOUND(found=false) — patrol 로 돌아간다"
            return Status.FAILURE

        variant = NUMERIC_TO_VARIANT.get(res.payload)
        if variant is None:
            self.feedback_message = f"UNKNOWN_PAYLOAD({res.payload!r})"
            return Status.FAILURE

        qr_pose = PoseStamped()
        qr_pose.header = res.header          # header 정보
        qr_pose.pose = res.qr_pose           # 위치 및 방향 정보
        self.bb.kind = res.payload
        self.bb.variant = variant
        self.bb.carrier_id = self.node.lookup_carrier_id(variant)
        self.bb.qr_pose = qr_pose
        self.node.on_scan_ok()
        self.node.get_logger().info(
            f"scan — 종류={self.bb.kind} variant={self.bb.variant} "
            f"carrier={self.bb.carrier_id}")
        self.feedback_message = f"{self.bb.variant}"
        return Status.SUCCESS


class Detected(py_trees.behaviour.Behaviour):
    """QR 이 보인다는 신호가 와 있나. 캐리어 처리 가지의 문지기다.

    이 조건이 매 tick 재검사되는 것이 이전 판과의 가장 큰 차이다. 이전 판은
    _patrol_step() 이 도착까지 블로킹해서, 그 사이에 온 신호가 도착한 뒤에야
    처리됐다 — 라벨을 지나쳐 세우고 스캔에 실패했다.
    """

    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.bb = _blackboard(name, ("detected",))

    def update(self):
        if self.node.failed:
            # 이전 판 _on_carrier_detected() 의 "if self._failed.is_set(): return".
            self.feedback_message = "정지 상태"
            return Status.FAILURE
        if not self.bb.detected:
            return Status.FAILURE
        if not PREEMPT_WHILE_DRIVING and self.node.driving():
            # 신호는 들고 있되 정차점에 설 때까지 기다린다. 베이스가 서면
            # driving() 이 False 가 되고 다음 tick 에 이 가지가 선택된다.
            self.feedback_message = "감지됨 — 정차점 도착 대기"
            return Status.FAILURE
        return Status.SUCCESS


class Hold(py_trees.behaviour.Behaviour):
    """로봇이 실제로 설 때까지 기다린다.

    "멈춰라" 를 여기서 보내지 않는다. 우선순위 Selector 가 순찰 가지를 밀어내면
    순찰 잎의 terminate(INVALID) 에서 취소가 나간다.

    보는 것이 "취소가 걸렸나" 가 아니라 "베이스가 아직 움직이나" 인 이유:
    한 tick 안에서 Sequence 가 이 잎을 먼저 통과시킨 뒤에야 Selector 가 순찰
    가지를 무효화한다. 즉 이 잎이 도는 시점에는 아직 취소가 나가기 전이다.
    "취소 대기 중인가" 로 물으면 아무것도 대기 중이 아니어서 그냥 통과하고,
    다음 잎(SCAN)이 아직 구르는 로봇 위에서 QR 을 읽는다.

    기다려야 하는 이유: 멈춰서 근접 판독해야 정확하다 (05 §12-1). 움직이는 중에
    읽으면 모션 블러와 판독 시각 오차가 qr_pose 에 그대로 실린다.

    이전 판 _cancel_active_goal() 과 같이 5 초까지만 기다리고, 응답이 없으면
    경고만 남기고 그대로 진행한다.
    """

    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.bb = _blackboard(name, ("detected",))

    def initialise(self):
        self.deadline = time.monotonic() + SERVER_WAIT_S

    def update(self):
        if self.node.driving():
            if time.monotonic() > self.deadline:
                self.node.get_logger().warning("goal 취소 응답이 없다 — 그대로 진행한다")
                self.node.clear_driving()
            else:
                self.feedback_message = "감속 대기"
                return Status.RUNNING
        # 신호를 여기서 지운다. 이 뒤로 들어오는 carrier_detected 는 작업 중에 온
        # 것이라 버린다 (TaskManager._on_carrier_detected 참고).
        self.bb.detected = False
        return Status.SUCCESS


class CycleDone(py_trees.behaviour.Behaviour):
    """배치와 복귀까지 끝났다. 손이 비었다고 확정하고 순찰로 돌아간다."""

    def __init__(self, name, node, waypoints):
        super().__init__(name)
        self.node = node
        self.waypoints = waypoints
        self.bb = _blackboard(name, ("kind", "variant", "carrier_id", "qr_pose"))

    def update(self):
        self.node.get_logger().info(
            f"사이클 완료 (carrier={self.bb.carrier_id}) — patrol 로 복귀")
        self.bb.kind = ""
        self.bb.variant = ""
        self.bb.carrier_id = ""
        self.bb.qr_pose = None
        # RETURN 이 PATROL_ROUTE[0] 까지 데려다 놨다. 순찰은 그 다음 점부터
        # 이어가면 된다 — 되감지 않으면 이미 서 있는 자리로 goal 을 한 번 더 보낸다.
        self.waypoints.idx = 1
        return Status.SUCCESS


# ══════════════════════════════════════════════════════════════════════════
#  데코레이터
# ══════════════════════════════════════════════════════════════════════════


class Freeze(py_trees.decorators.Decorator):
    """자식이 FAILURE 면 그 자리에서 멈춘다 — 그 뒤로는 tick 마다 RUNNING.

    이전 판의 실패 정책 그대로다: "로봇을 멈추고 그 자리에서 정지한다. 자동
    복귀도 재시도도 하지 않는다." 다른 점은 얼어붙는 자리가 상태 문자열이 아니라
    트리의 이 노드라는 것이다. RUNNING 을 돌려주므로 Selector 는 이 가지를 계속
    고르고, 로봇은 아무 데도 가지 않는다.

    나중에 "3번까지 재시도" 로 바꾸고 싶으면 이 클래스 하나만 고치면 된다.
    py_trees.decorators.Retry 로 갈아끼워도 된다.

    soft 실패(자식이 self.soft = True 로 표시한 것)는 얼리지 않고 FAILURE 를
    그대로 올린다. SCAN 의 found=false 가 그렇다.
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
        if child.status != Status.FAILURE:
            self.feedback_message = child.feedback_message
            return child.status

        reason = child.feedback_message or "UNKNOWN"
        if getattr(child, "soft", False):
            self.feedback_message = reason
            return Status.FAILURE

        self.frozen = True
        self.node.on_freeze(self.stage, reason)
        self.feedback_message = f"FROZEN: {reason}"
        return Status.RUNNING


# ══════════════════════════════════════════════════════════════════════════
#  트리
# ══════════════════════════════════════════════════════════════════════════


class _NextWaypoint:
    """순찰 정차점을 하나씩 전진시킨다. 이전 판의 self._patrol_idx.

    고른 인덱스를 node 에 알린다 — carrier_code_reader 가 /orchestrator/state 의
    patrol_target 을 보고 층별 관측 자세를 고르기 때문이다(그 노드의
    POSE_BY_PATROL_TARGET). 끝점(1)로 가는 중이면 2층, 시작점(0)으로 돌아가는
    중이면 1층이다.
    """

    def __init__(self, node, start_idx=0):
        self.node = node
        self.idx = start_idx

    def __call__(self):
        target_idx = self.idx % len(PATROL_ROUTE)
        self.idx += 1
        self.node.set_patrol_target(target_idx)
        return NavigateTo.Goal(pose=_to_pose(PATROL_ROUTE[target_idx]))


def build_tree(node):
    """이 모듈 맨 위 docstring 의 트리를 조립한다."""
    bb = _blackboard("build_tree", ("variant", "qr_pose"))
    # START 와 RETURN 이 둘 다 PATROL_ROUTE[0] 에 데려다 놓으므로, 순찰은 그
    # 다음 점부터 잇는다 — 안 그러면 이미 서 있는 자리로 goal 을 한 번 더 보낸다.
    waypoints = _NextWaypoint(node, start_idx=1)

    scan = Freeze("SCAN", ScanLeaf(SCAN, node), node, SCAN)

    pick = Freeze("PICK", ActionLeaf(
        PICK, node, node.pick, "/manipulation/pick_carrier", PickCarrier.Result,
        make_goal=lambda: PickCarrier.Goal(variant=bb.variant, qr_pose=bb.qr_pose),
        timeout_s=PICK_TIMEOUT_S,
        feedback_cb=node.log_phase("PICK")), node, PICK)

    # 목적지 좌표로 주행. 지금은 매거진 1 · 2 전부 test_loader 로 간다.
    nav = Freeze("NAV", ActionLeaf(
        NAV, node, node.nav, "/navigation/navigate_to", NavigateTo.Result,
        make_goal=lambda: NavigateTo.Goal(pose=_to_pose(TEST_LOADER)),
        timeout_s=NAV_TIMEOUT_S, moves_base=True), node, NAV)

    # 놓을 자리는 종류로 정해진다 — 좌표를 넘기지 않는다.
    place = Freeze("PLACE", ActionLeaf(
        PLACE, node, node.place, "/manipulation/place_carrier", PlaceCarrier.Result,
        make_goal=lambda: PlaceCarrier.Goal(variant=bb.variant),
        timeout_s=PLACE_TIMEOUT_S,
        feedback_cb=node.log_phase("PLACE")), node, PLACE)

    # 배치를 마친 자리(TEST_LOADER)는 순찰 경로에서 멀다. 순찰 잎이 어차피
    # 다음 정차점으로 goal 을 내기는 하지만, 복귀를 단계로 세워 두면 어디서
    # 실패했는지가 갈리고(TraceEvent.msg 의 stage 목록에도 RETURN 이 있다),
    # 나중에 배터리 검사를 끼울 자리가 생긴다 — NavigateTo.action 의
    # "넘으면 다음 RETURN 에서 dock_pad 로" 가 여기다.
    ret = Freeze("RETURN", ActionLeaf(
        RETURN, node, node.nav, "/navigation/navigate_to", NavigateTo.Result,
        make_goal=lambda: NavigateTo.Goal(pose=_to_pose(PATROL_ROUTE[0])),
        timeout_s=NAV_TIMEOUT_S, moves_base=True), node, RETURN)

    mission = py_trees.composites.Sequence(
        "캐리어 처리", memory=True,
        children=[Detected("detected?", node), Hold(HOLD, node),
                  scan, pick, nav, place, ret,
                  CycleDone("사이클 완료", node, waypoints)])

    # 순찰 — 도착(SUCCESS)을 RUNNING 으로 바꿔서 끝나지 않게 만든다. 다음 tick 에
    # 잎이 다시 initialise() 되면서 _NextWaypoint 가 다음 정차점을 낸다.
    # SuccessIsRunning 덕분에 이 Freeze 는 정차점에 도착하는 순간에도 RUNNING 을
    # 유지한다 — _on_carrier_detected 가 "지금 순찰 중인가" 를 이걸로 판단하므로
    # 도착하는 tick 에 신호를 흘리지 않으려면 그래야 한다.
    # CANCELED 는 의도된 중단이라 실패로 치지 않는다 (NavigateTo.action 주석).
    patrol = Freeze("PATROL", py_trees.decorators.SuccessIsRunning(
        name="순찰", child=ActionLeaf(
            PATROL, node, node.nav, "/navigation/navigate_to",
            NavigateTo.Result, make_goal=waypoints, timeout_s=NAV_TIMEOUT_S,
            ok_fail_reasons=(NavigateTo.Result.CANCELED,),
            moves_base=True)), node, PATROL)

    # 시작 자리에서 순찰 첫 정차점까지. 노드가 뜬 자리는 순찰 경로 위가 아니다.
    # OneShot 이라 성공하면 그 뒤로는 자식을 tick 하지 않는다 — goal 은 평생 한 번
    # 나간다. 성공 전에 끊기면(그럴 일은 아래 참고) 기억하지 않으므로 다시 시도한다.
    start = py_trees.decorators.OneShot(
        "START(1회)",
        child=Freeze("START", ActionLeaf(
            START, node, node.nav, "/navigation/navigate_to", NavigateTo.Result,
            make_goal=lambda: NavigateTo.Goal(pose=_to_pose(PATROL_ROUTE[0])),
            timeout_s=NAV_TIMEOUT_S, moves_base=True), node, START),
        policy=py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION)

    patrol_branch = py_trees.composites.Sequence(
        "순찰 가지", memory=True, children=[start, patrol])

    # 바깥에서 "지금 순찰 중인가" 를 물을 수 있게 해 둔다. START 중에는 patrol 이
    # 아직 tick 되지 않아 INVALID 다 — 그래서 START 중에 들어온 carrier_detected 는
    # 이전 판이 "상태가 patrol 이 아니면 버린다" 로 하던 것과 같이 버려진다.
    node.patrol_node = patrol

    # 가지를 더한다면 여기다. 위에 있을수록 먼저 기회를 받는다 —
    # 배터리 선점(NavigateTo.action ★ hard_threshold_s)은 mission 위에,
    # 외부 작업 지시(RegisterTask.srv) 처리는 mission 과 patrol_branch 사이에 온다.
    return py_trees.composites.Selector(
        "우선순위", memory=False, children=[mission, patrol_branch])


def current_stage(root):
    """지금 RUNNING 인 잎의 이름. 이전 판의 self._state 를 대신한다.

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
        self.carriers = yaml.safe_load(CARRIERS_YAML.read_text(encoding="utf-8"))

        self.failed = False
        self.patrolling = False
        self._driving = None
        self._scan_cooldown_until = 0.0
        self._scan_fail_streak = 0
        self._last_snapshot = ""

        self.bb = _blackboard("task_manager", (
            "detected", "kind", "variant", "carrier_id", "qr_pose",
            "fail_stage", "fail_reason", "scan_fail_streak", "patrol_target"))
        self.bb.detected = False
        self.bb.kind = ""
        self.bb.variant = ""
        self.bb.carrier_id = ""
        self.bb.qr_pose = None
        self.bb.fail_stage = ""
        self.bb.fail_reason = ""
        self.bb.scan_fail_streak = 0
        self.bb.patrol_target = None

        # 콜백 그룹도 스레드도 없다. 잎이 블로킹하지 않아서 단일 스레드로 충분하다.
        self.carrier_scan = self.create_client(CarrierScan, "/perception/carrier_scan")
        self.nav = ActionClient(self, NavigateTo, "/navigation/navigate_to")
        self.pick = ActionClient(self, PickCarrier, "/manipulation/pick_carrier")
        self.place = ActionClient(self, PlaceCarrier, "/manipulation/place_carrier")
        self.create_subscription(
            Bool, "/perception/carrier_detected", self._on_carrier_detected, 10)
        self._state_pub = self.create_publisher(String, "/orchestrator/state", 10)

        self.patrol_node = None          # build_tree 가 채운다
        self.tree = py_trees.trees.BehaviourTree(build_tree(self))
        self.tree.add_post_tick_handler(self._on_post_tick)
        self.tree.setup()

        self.create_timer(TICK_PERIOD_S, self.tree.tick)
        self.create_timer(1.0, self._publish_state)

        self.get_logger().info("task_manager ready — 행동트리 tick 시작")
        if not PATROL_ROUTE:
            self.get_logger().warning(
                "PATROL_ROUTE 가 비어 있다 — 순찰 잎이 정차점을 못 낸다. "
                "task_manager.py 상단에 좌표를 넣어라.")
        if TEST_LOADER is None:
            self.get_logger().warning(
                "TEST_LOADER 가 비어 있다 — pick 까지는 되지만 nav 단계에서 멈춘다. "
                "task_manager.py 상단에 좌표를 넣어라.")

    # ── 트리가 부르는 것들 ────────────────────────────────────────────────
    def on_freeze(self, stage, reason):
        """Freeze 가 얼어붙을 때 한 번 부른다."""
        self.failed = True
        self.bb.fail_stage = stage
        self.bb.fail_reason = reason
        self.get_logger().error(
            f"[실패] {stage} 단계에서 멈췄다. 이유: {reason} — "
            f"그 자리에서 정지한다. 자동 복귀하지 않는다.")
        self._publish_state()

    def begin_cancel(self, goal_handle, who):
        """진행 중인 goal 을 거둔다.

        취소를 보내기만 한다. 로봇이 실제로 섰는지는 goal 의 result 가 오는지로
        판단하고, 그건 Hold 가 driving() 으로 본다.
        """
        self.get_logger().info(f"진행 중인 goal 취소 ({who})")
        goal_handle.cancel_goal_async()

    def set_driving(self, result_future):
        """베이스를 움직이는 goal 이 떴다. 그 result future 를 들고 있는다."""
        self._driving = result_future

    def driving(self):
        """베이스가 아직 움직이고 있나. result 가 오면(성공·실패·취소) 끝난 것이다."""
        if self._driving is None:
            return False
        if self._driving.done():
            self._driving = None
            return False
        return True

    def clear_driving(self):
        self._driving = None

    def set_patrol_target(self, idx):
        """순찰 잎이 다음 정차점을 고를 때마다 부른다.

        carrier_code_reader 가 /orchestrator/state 의 patrol_target 으로 층별
        관측 자세를 고르므로, 바뀌는 즉시 알려야 한다 — 다음 1초 주기 발행을
        기다리면 그 사이 엉뚱한 층을 본다.
        """
        self.bb.patrol_target = idx
        self._publish_state()

    def on_scan_not_found(self):
        """SCAN 이 found=false 로 끝났다. 잠시 감지를 받지 않는다.

        같은 자리에 또 서서 또 실패하는 무한 반복을 끊는다. 멈추지는 않는다 —
        다른 라벨은 계속 처리해야 하고, 라벨이 시야에서 벗어난 것뿐이라면
        다음 기회에 성공할 수도 있다.
        """
        self._scan_fail_streak += 1
        self.bb.scan_fail_streak = self._scan_fail_streak
        self._scan_cooldown_until = time.monotonic() + SCAN_COOLDOWN_S
        if self._scan_fail_streak >= SCAN_FAIL_WARN:
            self.get_logger().warning(
                f"SCAN 연속 {self._scan_fail_streak}회 미판독 — 정지 위치가 판독거리"
                f"(05 §2-4, 0.28 m) 밖일 수 있다. 순찰 정차점이나 감지 임계를 "
                f"확인해라. 순찰은 계속한다.")
        else:
            self.get_logger().info(
                f"SCAN 미판독 — {SCAN_COOLDOWN_S:.0f}초간 carrier_detected 를 받지 않는다")

    def on_scan_ok(self):
        self._scan_fail_streak = 0
        self.bb.scan_fail_streak = 0

    def lookup_carrier_id(self, variant):
        """carriers.yaml 에서 그 variant 의 첫 항목.

        QR 이 종류만 담아서 개체는 못 가린다.
        """
        for cid, info in self.carriers.items():
            if isinstance(info, dict) and info.get("variant") == variant:
                return cid
        return ""

    def log_phase(self, label):
        return lambda fb: self.get_logger().info(f"  {label} phase={fb.feedback.phase}")

    # ── 구독 ──────────────────────────────────────────────────────────────
    def _on_carrier_detected(self, msg):
        if not msg.data:
            return
        if self.failed:
            return
        if time.monotonic() < self._scan_cooldown_until:
            # 방금 그 라벨을 못 읽었다. 같은 자리에 또 서지 않는다.
            self.get_logger().debug("carrier_detected 무시 (스캔 실패 쿨다운)")
            return
        if not self.patrolling:
            # 순찰 중일 때만 받는다. 이전 판의 "if self.state != PATROL: return" 과
            # 같은 규칙이다 — 작업 중(scan/pick/nav/place/return)이나 START 로
            # 이동하는 중에 들어온 신호는 버린다. 물건은 그 자리에 그대로 있으므로
            # 다음 순찰에 다시 보인다.
            self.get_logger().debug(
                f"carrier_detected 무시 (지금 {current_stage(self.tree.root)})")
            return
        if not self.bb.detected:
            self.get_logger().info("carrier_detected — QR 이 보인다")
            self.bb.detected = True

    # ── 표시 ──────────────────────────────────────────────────────────────
    def _on_post_tick(self, tree):
        self.patrolling = self.patrol_node.status == Status.RUNNING
        snapshot = py_trees.display.unicode_tree(tree.root, show_status=True)
        if snapshot != self._last_snapshot:
            self._last_snapshot = snapshot
            self.get_logger().info("\n" + snapshot)

    def _publish_state(self):
        # 얼어붙었으면 그 단계가 상태다 — 이전 판의 "상태는 실패한 그 상태 그대로
        # 둔다. pick 에서 실패했으면 상태는 pick 이다" 와 같다. Freeze 는
        # 데코레이터라 잎이 아니어서 current_stage() 에 안 잡힌다.
        stage = self.bb.fail_stage if self.failed else current_stage(self.tree.root)
        msg = String()
        # state= 값은 carrier_code_reader 와의 계약이다. 그 노드는
        #   _detect_tick:   state == "patrol" 일 때만 QR 폴링을 돈다
        #   _on_orchestrator_state: state != "patrol" 이면 팔 자세를 다시 잡도록 disarm
        # 이라 순찰 중에는 정확히 "patrol" 이어야 한다. 트리의 순찰 잎 이름을
        # PATROL("patrol")로 둔 이유가 이것이다.
        parts = [f"state={stage}", f"detected={self.bb.detected}"]
        if self.bb.patrol_target is not None:
            # carrier_code_reader 가 이걸 보고 층별 관측 자세(observe_pose
            # pose_name)를 고른다 — PATROL_ROUTE[1](끝점)로 가는 중이면 2층,
            # PATROL_ROUTE[0](시작점)으로 돌아가는 중이면 1층. 확정안 밖의
            # 추가분(§03 note)이라 자유롭게 확장 가능하다.
            parts.append(f"patrol_target={self.bb.patrol_target}")
        if self.bb.scan_fail_streak:
            parts.append(f"scan_fail={self.bb.scan_fail_streak}")
        if self.bb.carrier_id:
            parts.append(f"carrier={self.bb.carrier_id}")
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
