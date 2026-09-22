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
     │   ├─ [?] 배송                   Selector, memory=True — 경로를 고른다
     │   │   ├─ [→] 직행               상대 state 가 아래 넷이 아니면 이쪽
     │   │   │   ├─ 상대 한가?         상대 state 가 nav · push · place ·
     │   │   │   │                     return 중 하나인가. 아래 "배송 경로를
     │   │   │   │                     고르는 기준" 참고
     │   │   │   └─ NAV                로더로 바로 주행
     │   │   └─ [→] 우회               상대 state 가 그 넷 중 하나면 이쪽
     │   │       ├─ HOLD_BACK          픽업존에서 — 상대가 로더에 멈출
     │   │       │                     때까지. 대기 장소가 로더 앞이라 그
     │   │       │                     구역에 둘이 같이 들어가면 안 된다
     │   │       ├─ APPROACH           대기 장소로 주행
     │   │       ├─ WAIT               대기 장소에서 — 상대가 로더를 비울 때까지
     │   │       └─ PUSH               로더로 주행
     │   ├─ PLACE                      배치
     │   ├─ RETURN                     순찰 시작 좌표로 복귀
     │   └─ 사이클 완료
     └─ (guard) 작업 있나?             EternalGuard — 웹 작업(ExecuteTask)의
         │                             문지기. wait_for_task 면 goal 이 올 때까지
         │                             아래 가지를 닫아 둔다. 아래 "웹 작업 지시"
         └─ [→] 순찰 가지              Sequence, memory=True
             ├─ START (1회)            시작 자리 → 순찰 시작점. 평생 한 번만
             │                         나간다 (OneShot). 노드가 뜬 자리는 순찰
             │                         경로 위가 아니라서 필요하다.
             ├─ POSE                   팔을 관측 자세로. 여기서 기다린다
             └─ 순찰                   정차점 사이를 왕복한다. 끝나지 않는다.

  순찰 가지의 순서가 곧 요구사항이다 — "시작점으로 이동하고, 자세를 취한 뒤,
  순찰을 시작한다". START 는 평생 한 번이지만 POSE 는 가지에 다시 들어올 때마다
  돈다. 그래서 RETURN 이 patrol_route[0] 로 복귀한 뒤에도 같은 순서가 성립한다 —
  잎 하나가 START 경로와 RETURN 경로를 둘 다 덮는다. 근거는 ObservePoseLeaf
  독스트링의 ★ 항목에 있다.

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

  ── 배송 경로를 고르는 기준 ────────────────────────────────────────────
  기준은 "상대가 로더 접근 차선을 쓰고 있는가" 하나다. 쓰고 있다고 보는 단계가
  peer_busy_stages 파라미터이고, 기본값은 DEFAULT_PEER_BUSY_STAGES 의 넷이다
  (launch 의 LANE_STAGES 와 같은 값이어야 한다. 단위테스트가 검사한다).

      nav     직행 가지로 로더로 바로 가는 중
      push    대기 자리에서 로더로 들어가는 중 — 이름만 다른 "목적지로 가는 중"
      place   로더에 붙어서 내려놓는 중
      return  로더에서 후진으로 나오는 중 — 이탈선이 접근선과 같은 선이라,
              이걸 빼면 대기하던 로봇이 출발하는 순간 정면으로 만난다

  목록에 없는 단계는 전부 "안 쓴다" 로 본다. patrol · hold · scan · pick 은 로더와
  무관하고, approach(대기 자리로 가는 중)와 wait(대기 자리에 정차)은 차선 밖이다.
  ★ approach 와 wait 을 목록에 넣으면 교착이다 — 두 로봇이 같은 목록을 쓰므로
    양쪽이 서로의 대기를 기다리고 아무도 움직이지 않는다. 이 둘이 목록 밖에
    있다는 것이 교착 부재의 근거이고, test_peer_yield.py 가 그걸 검사한다.

  ★ 우회 가지는 두 번 기다린다. 대기 장소가 로더 바로 앞이라 거기로 가는 것
    자체가 로더 구역에 들어가는 것이기 때문이다.
        HOLD_BACK  픽업존에서. 상대가 로더 구역 안에서 움직이는 중
                   (nav · push · return)이면 움직이지 않는다. 상대가 로더에
                   멈춰 있는 동안(place)에만 대기 장소로 올라간다.
        WAIT       대기 장소에서. 상대가 로더를 비울 때까지 기다린다.
    앞의 것이 구역 진입을 막고, 뒤의 것이 로더 진입을 막는다.

  예외 하나: 상대 상태를 한 번도 못 받았으면 상대가 안 떠 있다고 보고 직행한다
  (peer_state_topic 이 비었거나 한 대만 띄운 경우). 한 번이라도 받은 뒤 소식이
  끊기면 마지막으로 들은 단계를 그대로 쓴다 — 수신 시각으로 "묵었으면 양보" 하지
  않는 이유는 peer_busy() 독스트링에 있다.

  ★ 이 판단은 PICK 직후 한 번이다. 배송 Selector 가 memory=True 라 고른 가지를
    끝까지 들고 가므로, 직행을 고른 뒤에 상대가 차선에 들어오면 로더에서
    마주친다. 아래 "알려진 갭" 참고.

상태 저장
  "지금 어느 단계인가" 는 저장하지 않는다. 어느 잎이 RUNNING 인지가 답이고,
  트리를 찍으면 그게 실제다. 이전 판의 self._state 는 따로 갱신해 줘야 하는
  값이라 실제와 어긋날 수 있었다 (실패 후 상태는 'scan' 인데 실제로는 아무것도
  하지 않는 상태였다).

웹 작업 지시 (ExecuteTask.action — /{robot}/orchestrator/execute_task)
  웹이 가져가는 것은 "다음에 어느 선반" 하나뿐이다(.action 머리주석). 그래서
  새 가지가 아니라 순찰 가지의 문지기(EternalGuard)와 순찰 경로 교체로 받는다.
    goal 수락     target_ref 로 shelves.yaml 의 선반을 찾아 waypoint_start →
                  waypoint_end 를 순찰 경로로 대기시킨다(_pending_route). 다른
                  선반을 돌던 중이면 순찰 가지를 한 tick 끊어 새 경로로 다시
                  들어간다. 미션 순서는 그대로다 — 순찰 → carrier_detected →
                  HOLD → SCAN → PICK → 배송 → PLACE → RETURN.
    feedback      pick · nav · place · return 에 들어설 때 한 번씩(_task_track).
                  run_id 는 SCAN 이 발행한 것이라 첫 feedback 부터 실린다.
    result        사이클 완료 → success. Freeze 로 얼면 그 단계의 *_FAIL 로 닫고
                  트리는 얼어 있는 채로 둔다(복구는 지금처럼 /orchestrator/resume).
                  경로를 empty_sweeps 바퀴 돌아도 감지가 없으면 NO_CARRIER —
                  실패가 아니다. SKIP/STOP(cancel_goal)은 미션 전이면 바로 세우고,
                  미션 중이면 RETURN 까지 마친 뒤 CANCELED 로 돌려준다.
    아직 없는 것  RECOVER(스테이션 회수) goal · resume_progress 재개 ·
                  more_at_target 판정(항상 false) · RobotCommand 서비스(PAUSE 등).
                  wait_for_task 기본값이 False 라 웹 없이 띄우던 순찰이 그대로다.

  "이번 미션의 데이터" 는 블랙보드(py_trees 의 공유 저장소)에 둔다.
      detected     carrier_detected 신호가 와 있나
      kind         QR 원문. 이 씬에서는 "1" 또는 "2" — 물체의 종류
      variant      kind 를 grasp.yaml(어떻게집나)/place.yaml(어디에놓나) 키로 푼 것
      carrier_id   QR 원문 그대로 = 개체 ID. 로그·표시용. DB 의 qr_payload 와 같은 값
      run_id       미션 1회를 묶는 UUID. SCAN 성공 때 발행해 사이클 끝까지 들고 간다.
                   DB 의 magazine_log.run_id 와 같은 값이다 (docs/DB구성.md §4)
      qr_pose      PickCarrier 가 플랜지를 찾을 탐색 창 prior
      fail_stage   어느 단계에서 멈췄나
      fail_reason  왜 멈췄나
      patrol_target 지금 향하는 PATROL_ROUTE 인덱스 — 아래 계약
      scan_fail_streak SCAN 이 연속 몇 번 미판독이었나

    ros2 run py_trees_ros_tutorials 없이도 아래로 볼 수 있다:
        ros2 topic echo /orchestrator/state

사용하는 인터페이스 — 08 문서에서 확정된 것만 쓴다. 메시지 타입은 이전 판과
동일하고, 이름만 상대이름이 됐다.
  구독  perception/carrier_detected  std_msgs/Bool
  호출  perception/carrier_scan      CarrierScan.srv
  호출  perception/observe_pose      std_srvs/Trigger — 순찰 전 관측 자세.
                                     어느 자세인지는 받는 쪽이 자기 파라미터로
                                     안다(DEFAULT_OBSERVE_POSE_SERVICE 참고)
  액션  navigation/navigate_to       NavigateTo.action
  액션  manipulation/pick_carrier    PickCarrier.action
  액션  manipulation/place_carrier   PlaceCarrier.action
  발행  orchestrator/state           std_msgs/String  (상태 · 실패 단계 확인용)
  구독  <peer_state_topic>           std_msgs/String  ★ 절대이름. 상대 로봇의
                                     orchestrator/state 를 그대로 읽는다. 로더
                                     접근 차선을 한 대만 쓰게 하려고 본다.
                                     파라미터가 비어 있으면 구독하지 않는다.
  발행  /trace/event                 TraceEvent.msg   ★ 절대이름. event_logger 가
                                     전역 1개라 로봇이 몇 대든 여기로 모인다
  서비스 orchestrator/resume         std_srvs/SetBool (웹 복구 — 얼어붙은 단계 재시도)
  서비스 config/reload               ReloadConfig.srv — 웹이 설정을 저장한 직후
                                     부른다. scope 가 shelves · all 이면 순찰
                                     경로를 다시 읽는다. 이름이 문서와 구현이
                                     갈라져 있다 — DEFAULT_RELOAD_SERVICE 참고
  파일  shelves.yaml                 순찰 정차점. 뜰 때 한 번 읽고, 그 뒤로는
                                     config/reload 를 받을 때 다시 읽는다.
                                     주인은 웹 관제 UI 「설정 > 선반」 탭이다

★ 이름 앞에 / 가 없다 — 전부 상대이름이고, 노드가 뜬 네임스페이스가 앞에
  붙는다. robot1 로 띄우면 /robot1/navigation/navigate_to 가 된다. 그래서 이
  노드는 자기가 어느 로봇인지 모르고, 알 필요도 없다 — /robot1 의 task_manager
  에게는 /robot1 의 서버만 보인다. "본 놈이 가는 것" 이 코드가 아니라 배선으로
  보장된다(docs/02 §2). 로봇을 늘릴 때 이 파일에서 고칠 것은 없다 — 순찰 경로는
  shelves.yaml 의 assigned_robot 으로 배정하고(웹 화면이 그 파일의 주인이다),
  나머지 로봇별 값(대기 자리 · 상대 토픽)은 mission_nodes.launch.py 의 표에서
  파라미터로 온다. 아래 좌표 상수는 둘 다 없을 때의 폴백일 뿐이다.

  예외가 하나 생길 예정이다: docking_server 는 도크가 공용 자원이라 전역 1개로
  두므로 /docking/dock 만 절대이름이고, 대신 Dock.action 의 robot_name 필드로
  어느 로봇인지 말한다(docs/02 §2). 아직 미구현.

★ /orchestrator/state 는 디버그 토픽이 아니라 계약이다
  cobot3_perception/carrier_code_reader 가 이걸 구독해서 두 가지를 판정한다.
  문자열 모양을 바꾸면 그 노드가 깨진다.

    state=<단계>         그 노드의 _detect_tick 은 이 값이 정확히 "patrol" 일
                         때만 QR 폴링을 돈다. _on_orchestrator_state 는 값이
                         "patrol" 이 아니게 되면 팔 자세를 다시 잡도록
                         disarm 한다. 그래서 순찰 중에는 정확히 "patrol" 이어야
                         한다 — 트리의 순찰 잎 이름을 PATROL("patrol") 로 둔
                         이유가 이것이다. start · pose · hold · scan · pick ·
                         nav · place · return 은 전부 "patrol 아님" 으로
                         취급된다. pose 가 그 목록에 있는 것은 의도다 — 팔이
                         관측 자세로 움직이는 동안은 폴링할 이유가 없다.
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
  - 씬이 아직 옛 에셋이라 QR 이 숫자 하나("1"/"2")만 담는다. 그 동안에도 돌도록
    _variant_of() 가 새 페이로드(F1-MGZB-1)와 옛 숫자를 둘 다 받는다.
    씬을 16종으로 바꾸면 NUMERIC_TO_VARIANT 와 그 폴백을 지운다.
  - 순찰 중이 아닐 때(START · POSE · scan · pick · nav · place · return) 들어온
    carrier_detected 는 버린다. 물건은 그 자리에 그대로 있으므로 다음 순찰에
    다시 보인다.
  - 순찰 경로 반영은 "저장 즉시" 가 아니라 "다음 순찰 사이클부터" 다.
    config/reload 를 받으면 새 좌표를 대기시켜 두고 POSE 잎이 커밋한다
    (_on_reload_config ★). 주행 중인 목표를 바꾸지 않으려는 것이라 의도된
    지연이지만, 로봇이 긴 미션 한 판을 도는 동안은 옛 좌표로 순찰한다.
  - 순찰 중 경로가 바뀌면 START 는 다시 나가지 않는다(OneShot 이라 평생 한 번).
    commit_pending_route 가 정차점 인덱스를 0 으로 되감아 새 경로의 첫 점부터
    가게 하지만, "시작 자리에서 순찰 경로로 진입" 을 다시 하지는 않는다.
    두 선반이 멀리 떨어져 있으면 그 사이를 순찰 잎이 직접 주행한다.
  - 관측 자세는 순찰 한 바퀴 내내 하나로 고정이다. 왕복 방향마다 층을 바꾸던
    방식(carrier_code_reader 의 POSE_BY_PATROL_TARGET)은 "시작점에서 한 번 잡고
    그대로 왕복한다" 로 바뀌었다. 그래서 아래 patrol_target 계약은 이제 이
    노드에서는 쓸 일이 없다 — 받는 쪽이 고정 자세로 넘어가면 발행도 지울 수
    있다. 호환을 위해 지금은 그대로 싣는다.
  - 배터리·도킹 선점 가지는 아직 없다. 트리에 가지 하나 더하는 자리가 이미
    나 있다 (build_tree 참고).
  - TraceEvent 발행은 붙었다 — Freeze.update() 한 곳에서 pick·nav·place·return
    넷을 전부 낸다. 스키마와 그 근거는 docs/DB구성.md.
  - 로더 차선 조율은 출발 시점의 판단으로 커밋한다. PICK 직후 상대를 한 번 보고
    직행/우회를 고르면 그 가지를 끝까지 들고 가므로, 직행을 고른 뒤에 상대가
    차선에 들어오면 로더에서 마주친다. 대기 자리를 무조건 경유하게 만들면
    판단 지점이 항상 로더 몇 초 앞이 되어 이 사각지대가 없어지는데, 로더가
    비어 있는 흔한 경우의 정차 비용을 아끼는 쪽을 골랐다. 사각지대를 줄이려면
    peer_busy_stages 를 넓힌다(예: 상대의 pick 도 넣으면 상대가 출발하기까지
    SCAN·PICK 을 거쳐야 하므로 겹칠 틈이 줄어든다).
  - 조율은 로봇 두 대를 전제로 한 배선이다. 상대가 하나라고 보고 토픽 하나를
    구독하고, 양쪽이 같은 규칙을 쓴다. 그래서 순서를 정하는 장치가 없다 —
    둘이 같은 순간에 대기 자리를 떠나면 차선에서 만난다. 막으려면 판단 지점을
    대기 자리 한 곳으로 모으고(무조건 경유) 거기서 순서를 정해야 한다.
    세 대 이상이면 도크처럼 전역 조정 노드를 두는 편이 낫다
    (docs/02 §2 의 docking_server 논리와 같다).
  - 대기 자리 좌표가 아직 launch 상수다. pkg_loader 가 씬에 들어오면 로더마다
    대기 자리가 따로 필요하고, 그때 frames.yaml 이 아니라 사람이 관리하는
    설정 파일로 옮겨야 한다(docs/02 §5 의 좌표 하드코딩 갭과 같은 자리).
"""

import hashlib
import math
import sys
import threading
import time
import uuid
from pathlib import Path

import py_trees
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from py_trees.common import Access, Status
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from cobot3_interfaces.action import ExecuteTask, NavigateTo, PickCarrier, PlaceCarrier
from cobot3_interfaces.msg import TraceEvent
from cobot3_interfaces.srv import CarrierScan, ReloadConfig

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
# 대신 두 점 사이 단순 직선 왕복을 쓴다. 두 점의 yaw 를 같게 잡으면 끝점에서
# 제자리 회전이 필요 없다 — nav_server 가 목표 지점이 뒤쪽(±90도 밖)이면
# 그대로 후진하고(_execute 의 direction = -1.0), 도착 yaw 가 이미 맞으니
# _fine_align 도 거의 돌지 않는다(허용오차 5도). 그래서 "전진/후진 반복" 이
# 순찰의 기본 거동이고, 선반 여유가 좁아도 성립한다.
#
# ★ 아래는 마지막 폴백일 뿐이다. 실제 값의 주인은 shelves.yaml 이고, 그 파일의
#   주인은 웹 관제 UI 「설정 > 선반」 탭이다. 이 노드는 뜰 때 자기 네임스페이스와
#   같은 assigned_robot 을 가진 선반을 찾아 waypoint_start → waypoint_end 를
#   순찰 경로로 쓴다. 고르는 규칙 전체는 _resolve_patrol_route 독스트링에 있다.
#   로봇마다 다른 선반을 돌아야 해서 모듈 상수로 두면 두 대가 같은 경로를 돌다
#   선반 앞에서 부딪힌다.
#
# ★ 파라미터는 평탄화해서 받는다 — rclpy 파라미터에 double[][] 타입이 없어서
#   중첩 리스트를 못 넘긴다. [x0, y0, yaw0, x1, y1, yaw1, ...] 로 주고 세 개씩
#   끊어 읽는다(_unflatten_route). staging_pose 가 flat list 인 것과 같은 이유다.
#
# ★ 이 폴백 좌표는 옛 레이아웃(선반이 x≈-6.5 에 있던 시절)의 2026-09-18
#   실측값이라 지금 씬에는 맞지 않는다 — simple_factory_layout.usda 의
#   Shelf_01 중심은 (-1.741, +2.953), Shelf_02 중심은 (-1.741, -1.973) 이다.
#   파라미터 없이 띄우면 로봇이 씬에 없는 자리로 가므로 __init__ 이 경고한다.
DEFAULT_PATROL_ROUTE = [
    -3.1, 1.9, 0.0,
    0.1, 1.9, 0.0,
]

# 순찰 시작 전에 "관측 자세를 잡아라" 고 말할 서비스. 상대이름이라 네임스페이스가
# 앞에 붙는다 — /robot1/perception/observe_pose.
#
# ★ std_srvs/Trigger 다 — 요청에 필드가 없다. 어느 자세를 잡을지는 받는 쪽이
#   자기 파라미터로 안다. 이 노드는 "네 순찰 자세를 잡아라" 만 말한다.
#   이유는 두 가지다:
#     1) 자세는 로봇마다 하나로 고정이다. 왕복 방향마다 층을 바꾸던 이전
#        방식(carrier_code_reader 의 POSE_BY_PATROL_TARGET)과 달리, 시작점에서
#        한 번 잡고 그대로 왕복한다. 그래서 매번 이름을 실어 보낼 이유가 없다.
#     2) docs/08 확정안의 "미션 어휘는 task_manager 에만, 나머지 노드는 좌표
#        하나 또는 종류 하나만 받는다" 와 맞는다. 자세 이름은 기하이고, 기하는
#        perception 담당이다. cobot3_interfaces 에 새 .srv 를 만들지 않아도
#        되는 것은 덤이다.
#   자세 이름을 task_manager 가 정해야 할 일이 생기면 그때 .srv 를 만들고
#   ObservePoseLeaf 의 요청 한 줄만 바꾸면 된다.
#
# ★ 빈 문자열이면 이 단계를 건너뛴다(잎은 트리에 남고 즉시 SUCCESS).
#   받는 쪽이 아직 없을 때 순찰만 먼저 돌려보기 위한 것이다 — 트리 모양이
#   바뀌지 않으므로 로그의 트리 그림과 current_stage 가 그대로 유지된다.
DEFAULT_OBSERVE_POSE_SERVICE = "perception/observe_pose"

# 웹이 설정을 저장한 직후 "다시 읽어라" 를 보내는 서비스.
#
# ★ 이름이 문서와 구현이 갈라져 있다. ReloadConfig.srv 머리주석은 "/{robot}/
#   config/reload — 노드마다 자기 이름 아래" 라 적혀 있어 노드 이름이 들어가는
#   것처럼 읽히는데, 실제로 웹 백엔드가 만드는 클라이언트는
#   web/backend/app/services/rosbridge.py 의
#       node.create_client(ReloadConfig, f"/{robot}/config/reload")
#   로 로봇당 하나다. 지금 붙는 쪽(웹)에 맞춰 상대이름 "config/reload" 를
#   기본값으로 둔다 — 네임스페이스가 붙어 /robot1/config/reload 가 된다.
#
#   ☞ 같은 네임스페이스의 다른 노드(carrier_code_reader · nav_server ·
#     pick_place_server)도 설정 yaml 을 읽으므로, 그쪽까지 이 서비스를 열면
#     이름이 겹친다. 로봇당 하나로 갈지(이 노드가 대표로 받고 나머지는 각자
#     다음 미션에 다시 읽는다) 노드마다 따로 둘지는 아직 정해지지 않았다
#     (ReloadConfig.srv §11 #3 "WBS 0.5 는 아직 열려 있다"). 노드별로 가기로
#     하면 이 파라미터만 바꾸면 된다.
DEFAULT_RELOAD_SERVICE = "config/reload"

# 웹이 작업을 시키는 액션(ExecuteTask.action). 상대이름이라 네임스페이스가 붙어
# /robot1/orchestrator/execute_task 가 된다 — web/backend/app/services/rosbridge.py
# 의 ActionClient 가 정확히 그 이름을 본다. 빈 문자열이면 서버를 열지 않는다.
DEFAULT_EXECUTE_TASK_ACTION = "orchestrator/execute_task"

# 작업(goal) 없이도 순찰할지.
#   False  지금까지처럼 assigned_robot 선반을 알아서 돈다. goal 은 "다음에 어느
#          선반" 만 바꾼다 — 웹 없이 띄우던 시험 방식이 그대로 된다.
#   True   goal 이 올 때까지 순찰 가지를 닫아 두고 그 자리에서 기다린다.
#          ExecuteTask.action 머리주석의 "goal 대기" 가 이것이다. 순찰 순서를
#          웹이 들 때 켠다.
DEFAULT_WAIT_FOR_TASK = False

# 빈 선반 판정. goal 의 경로를 적용한 뒤 선반을 이만큼 왕복(시작점→끝점→시작점)
# 했는데도 carrier_detected 가 한 번도 없으면 NO_CARRIER 로 끝낸다 — 실패가
# 아니다(ExecuteTask.action result 주석, docs/DB구성.md §8-4).
DEFAULT_EMPTY_SWEEPS = 1

# place 하러 갈 목적지 — 테스트 스테이션 로더 앞 주차 위치.
# 지금은 매거진 1 · 2 를 전부 여기로 가져다 놓는다. (08 문서의 "매거진은 패키징
# 로더로" 규칙은 지금 적용하지 않는다 — pkg_loader 는 나중에 추가한다.)
# ★ isaacpjt/M0609/lula_ik/12_place_test.py 의 WP_PLACE 와 반드시 같은 값이어야
# 한다 — place.yaml 의 PLACE_SLOT_POSE_BASE_LINK(그 노드가 여기 도착했다는
# 전제로 만든 base_link 상대 오프셋)가 이 좌표 기준이다. 원래 4.0 이었는데
# 컨베이어 회전-여유 문제로 3.85 로 뺐다(nav_server.py 조향 버그 이력 참고).
TEST_LOADER = (3.85, 0.0, 0.0)

# ── 로더 접근 차선 — 두 대가 같은 로더로 갈 때 ───────────────────────────
# ★ 로더 주차점에서는 몸을 거의 못 돌린다.
#   base_link 가 차체 앞쪽에 있어서 제자리회전 스윕 반경이 앞 0.286 m, 뒤
#   0.656 m 로 비대칭이다(robot1_nav2_params.yaml 의 footprint). 점유격자에서
#   주차점(stations.yaml PKG-01 place_pose) 최근접 장애물이 0.30 m 이므로,
#   footprint 사각형을 실제로 돌려 보면 yaw 오차 9도에서 뒷모서리가 닿는다.
#
#   즉 로더에는 거의 정렬된 상태로 들어가야 한다. 도착 yaw 오차가 남으면
#   nav_server 의 _fine_align 이 그 자리에서 몸을 돌리다 장애물을 친다.
#   그리고 place.yaml 의 배치 자세가 base_link 기준 앞으로 0.25 m 라, yaw 가
#   틀어진 채로 PLACE 하면 배치 목표 자체가 엉뚱한 곳으로 간다.
#
# ★ 이탈선이 접근선과 겹친다. 배치를 마친 로봇은 후진으로 나오는데, nav_server
#   가 ALIGN_MAX_W(0.15 rad/s) 상한으로 천천히 돌기 때문에 로더 뒤 1.35 m
#   가량은 접근선을 그대로 따라 나오고 그 뒤부터 순찰 시작점 쪽으로 흐른다.
#   그래서 대기 자리는 그 흐름의 반대쪽으로 비껴서 잡는다.
#
# 아래 기본값은 이 세 조건을 전부 통과한 점이다.
#       robot1 ( 0.80, -0.40)  도착각 +7.5도 · 로더 3.08 m · 이탈여유 0.32 m
#       robot2 (-0.80,  0.00)  도착각 +0.0도 · 로더 4.65 m · 이탈여유 0.32 m
#
# ★ 로봇마다 달라야 한다 — 같은 점을 쓰면 대기 자리에서 둘이 부딪힌다.
#   mission_nodes.launch.py 가 로봇별로 넘긴다.
#
# ★ 지도를 새로 만들면 반드시 다시 검증해라.
#       python3 isaacpjt/tools/check_staging_poses.py
#   좌표를 점유격자에서 골랐는데 레이아웃을 손보면 지도가 바뀐다. 실제로 한 번
#   바뀌면서(원점이 x -10.075 에서 -6.575 로 이동) 그전에 고른 대기 자리 하나가
#   장애물 안으로 들어갔고, 로더를 막고 있던 것도 동쪽 컨베이어에서 북쪽
#   스테이션으로 바뀌었다. 그 도구가 지도·launch·stations.yaml 을 직접 읽으므로
#   값을 두 곳에 적어 둘 필요가 없다.
DEFAULT_STAGING_POSE = [0.80, -0.40, 0.0]

# 순찰 없이 SCAN 부터 바로 돌리는 임시 스위치. 순찰 가지를 주석처리해 뒀던
# 동안 쓰던 것이고, 순찰을 되살리면서 False 로 되돌렸다 — carrier_detected 를
# 정상 경로(순찰 중 감지)로 받는다. 순찰 없이 pick 부터 단위 테스트하고 싶으면
# 다시 True 로 두면 되지만, 그때는 로봇이 이미 pick 자리에 서 있어야 한다.
START_DETECTED_FOR_TEST = False

# ── 단계 이름 ─────────────────────────────────────────────────────────────
# 이전 판의 "상태" 다. 지금은 상태가 아니라 라벨이다 — 트리의 어느 노드인지,
# 그리고 실패 기록(fail_stage)에 어느 단계였는지 적는 데만 쓴다.
START = "start"
POSE = "pose"           # 순찰 시작 전 관측 자세 잡기. 아래 ObservePoseLeaf 참고
PATROL = "patrol"
HOLD = "hold"
SCAN = "scan"
PICK = "pick"
NAV = "nav"
PLACE = "place"
RETURN = "return"
# 우회 경로(로더 차선 조율)에서만 지나가는 단계들. 직행이면 NAV 하나로 끝난다.
HOLD_BACK = "hold_back" # 픽업존에서 상대가 로더에 도착하기를 기다린다
APPROACH = "approach"   # 대기 장소로 주행
WAIT = "wait"           # 대기 자리에서 상대가 차선을 비우기를 기다린다
PUSH = "push"           # 차선이 비면 대기 자리에서 로더로 주행

# ── 생산 트래킹 (docs/DB구성.md) ──────────────────────────────────────────
# DB 에 행이 남는 단계. scan 은 없다 — 판독 실패는 미션이 시작되지도 않은 것이라
# 남길 행이 없고, 판독 성공 시각은 pick 행의 started_at 이 곧 그것이다 (§4-6).
# 우회 경로의 네 단계도 넣는다 — 대기 시간이 두 대 시연의 핵심 지표이고,
# docs/DB구성.md §4 가 stage 를 TEXT 로 둔 이유가 "가지가 늘어날 자리" 다.
# 값을 더하는 데 마이그레이션이 필요 없다.
LOGGED_STAGES = (PICK, HOLD_BACK, APPROACH, WAIT, PUSH, NAV, PLACE, RETURN)

# 실패 사유는 '단계' 가 정한다. return 은 같은 NavigateTo 액션이라 nav_error 다.
# 그래서 액션 enum 에 없는 실패(TIMEOUT · SERVER_UNAVAILABLE · GOAL_REJECTED)도
# 갈 곳이 있다. DB 의 fail_reason ENUM 네 값과 같아야 한다.
STAGE_TO_REASON = {PICK: "pick_error", NAV: "nav_error",
                   PLACE: "place_error", RETURN: "nav_error",
                   # 우회 세 단계는 전부 주행/대기라 nav_error 로 모인다.
                   HOLD_BACK: "nav_error", APPROACH: "nav_error",
                   WAIT: "nav_error", PUSH: "nav_error"}

# 어디서 일어난 일인가. 순찰 중 발견 방식이라 슬롯 번호를 모르므로 place 만 채워진다.
PORT_BY_STAGE = {PLACE: "test_loader", PUSH: "test_loader"}

# ── 웹 작업(ExecuteTask) 보고 ────────────────────────────────────────────
# feedback.stage 는 .action 이 정한 넷(pick · nav · place · return)뿐이다.
# 우회 네 단계는 전부 주행/대기라 nav 로 올린다 — STAGE_TO_REASON 이 nav_error
# 로 모으는 것과 같은 기준이다. 목록에 없는 단계(scan · hold · 순찰)는 보고하지
# 않는다: scan 은 run_id 가 아직 없고, 순찰은 작업 밖이다.
TASK_STAGE = {PICK: "pick", NAV: "nav", HOLD_BACK: "nav", APPROACH: "nav",
              WAIT: "nav", PUSH: "nav", PLACE: "place", RETURN: "return"}

# feedback.progress — goal.resume_progress 와 같은 축(0.0 처음부터 … 1.0 복귀
# 끝)이라 단계마다 고정값이다. 화면이 막대로 그릴 뿐 로봇은 안 읽는다.
TASK_PROGRESS = {PICK: 0.3, NAV: 0.5, HOLD_BACK: 0.4, APPROACH: 0.45,
                 WAIT: 0.45, PUSH: 0.5, PLACE: 0.7, RETURN: 0.9}

# Freeze 가 얼어붙은 단계 → result.fail_reason. 하위 액션의 실패를 그대로
# 올리는 것이라(.action 주석) 단계가 곧 사유다. 목록 밖(우회·순찰·시작·자세)
# 은 전부 주행이라 NAV_FAIL 이다.
TASK_FAIL_REASON = {SCAN: ExecuteTask.Result.SCAN_FAIL,
                    PICK: ExecuteTask.Result.PICK_FAIL,
                    PLACE: ExecuteTask.Result.PLACE_FAIL}

# 이 단계들 중 하나가 RUNNING 이면 미션이 시작된 것이다 — 취소가 와도 캐리어를
# 놓지 않고 RETURN 까지 마친다(_execute_task).
MISSION_STAGES = (HOLD, SCAN, PICK, HOLD_BACK, APPROACH, WAIT, PUSH, NAV, PLACE, RETURN)


# 각 단계를 이만큼 기다려도 안 끝나면 실패로 본다. 단위 초.
# SCAN 은 재시도(SCAN_RETRIES)까지 포함해서 이 시간 안에 끝나야 한다 — 시도
# 한 번(observe_pose 재정렬 + n_frames 캡처)이 GUI/렌더 모드에서 몇 초씩
# 걸리므로 재시도 여유를 넉넉히 둔다.
# 관측 자세로 팔이 움직이는 데 주는 시간. sim_backend 의 observe_pose 는
# 관절을 옮기고 SETTLE_STEPS 만큼 시뮬을 돌린 뒤 카메라까지 덥히므로
# GUI 모드에서 몇 초 걸린다. SCAN 과 같은 여유를 준다.
OBSERVE_POSE_TIMEOUT_S = 40.0
SCAN_TIMEOUT_S = 40.0
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

# SCAN 이 found=false 를 받아도 바로 patrol 로 돌아가지 않고 이 횟수만큼
# carrier_scan 을 다시 부른다(최초 시도 포함 총 SCAN_RETRIES+1 번). 매 시도가
# observe_pose 로 팔을 다시 정렬하고 카메라를 새로 캡처하므로, 렌더링 워밍업
# 부족이나 그 한 프레임의 일시적 디코드 실패 같은 걸 재시도로 걸러낸다.
# 그래도 계속 실패하면(라벨이 진짜 시야 밖이거나 판독거리 밖) 기존 정책대로
# soft 실패로 patrol 로 돌아간다.
SCAN_RETRIES = 2

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

# ── 로더 차선 조율 ───────────────────────────────────────────────────────
# 상대가 차선을 쓰고 있다고 보는 단계들. 여기 들어 있으면 기다린다.
#   nav     직행 가지로 로더로 바로 가는 중
#   push    대기 자리에서 로더로 들어가는 중
#   place   로더에 붙어서 내려놓는 중
#   return  로더에서 후진으로 차선을 빠져나오는 중 — 접근선과 같은 선이다
# 'approach'(대기 자리로 가는 중)와 'wait'(대기 자리에 정차)는 넣지 않는다.
# 둘 다 차선 밖이라 방해되지 않고, ★ 넣으면 교착이다 — 양쪽이 서로의 대기를
# 기다리면 아무도 안 움직인다. 두 로봇이 이 목록을 똑같이 쓰므로 이 규칙이
# 곧 교착 부재의 근거다. isaacpjt/tools/test_peer_yield.py 가 이걸 검사한다.
DEFAULT_PEER_BUSY_STAGES = [NAV, PUSH, PLACE, RETURN]

# ── 대기를 푸는 기준: 로더 반경 ───────────────────────────────────────────
# ★ 시간으로 재지 않는 이유
#   이 파일의 타임아웃은 전부 time.monotonic() 즉 벽시계다. 그런데 Isaac 을 GUI
#   렌더로 돌리면 시뮬이 실시간보다 느려서, 벽시계로 본 이동 속도가 nav_server 의
#   지령 상한(개활지 1.2 m/s)과 전혀 다르고 머신·렌더 설정마다 또 다르다.
#   TraceEvent.msg §4-2 가 사이클 타임을 시뮬 시각으로 재는 이유가 같은 것이다.
#   그래서 "상대가 나간 뒤 N 초" 로 두면 N 을 정할 근거가 없다. 대신 상대가 로더
#   주차점에서 얼마나 떨어져 있는지를 보면 속도와 무관하다.
#
# 판정은 로더 주차점(stations.yaml PKG-01 place_pose, 코드에서는 TEST_LOADER)을
# 중심으로 한 반경이다. 상대의 amcl_pose 와 그 점의 거리를 잰다.
DEFAULT_LOADER_CLEAR_RADIUS_M = 1.0
# ★ 파라미터다 — 시뮬에서 보면서 조정할 값이라 소스 상수로 두지 않았다.
#       ros2 param set /robot2/task_manager loader_clear_radius_m 2.0
#
# ★ 1.0 은 시작값이고 낙관적인 쪽이다. 상대가 로더에서 후진해 나오는 경로가
#   이쪽 진입 경로와 같은 선이라, 기하로 따지면 더 커야 한다.
#       3.05 m  상대가 이쪽 대기 자리의 x 를 지나가는 거리
#       4.15 m  이탈 궤적이 접근선을 1.35 m 따라 나온 뒤 14도로 흐르는 것을
#               감안해, 이쪽 진입선과의 횡간격이 차체 폭 합(0.50 m) + 여유
#               0.20 m 를 넘는 지점까지의 거리
#   1.0 m 시점의 상대는 아직 접근선 위(y≈0)에 있다. 그때 출발하면 둘이 중간에서
#   마주칠 수 있다. 시뮬에서 볼 첫 증상이 그것이고, 그러면 이 값을 올린다.
#   ★ 위 3.05 · 4.15 도 nav_server 코드에서 유도한 이탈 궤적 근사의 산물이다
#     (실측 아님). 절대값으로 믿지 말 것.

# 대기 자리에서 이만큼 기다려도 안 비면 실패로 본다. 넘으면 다른 실패와 똑같이
# Freeze 되어 웹에 뜬다.
#
# ★ 이건 "성능" 문턱이 아니라 "아무도 안 온다" 문턱이다. 정상 대기의 상한은 상대
#   한 사이클인데 시뮬 속도에 따라 몇 분이 될 수 있으므로 넉넉히 둔다. 상대가
#   얼어붙은 경우는 이 타임아웃이 아니라 PEER_FROZEN 이 먼저 잡으므로, 이 값이
#   커도 실패가 늦게 드러나지는 않는다.
PEER_WAIT_TIMEOUT_S = 1800.0

# 조건이 풀린 뒤 이만큼 더 서 있는다.
#
# ★ 지금은 형식적인 값이다. 풀리는 경로가 전부 위치나 완료 상태를 보기 때문에,
#   이 잎이 SUCCESS 를 낼 때 상대는 이미 반경 밖이다. 상태가 예상 밖으로 건너뛸
#   때의 최소 여유로만 남긴다.
PEER_CLEAR_DWELL_S = 2.0

# ── 상대 단계를 두 종류로 가른다 ─────────────────────────────────────────
# 반경만으로 전부 판정할 수는 없다. 거리가 같아도 방향이 다르기 때문이다.
#
#   다가오는 중 (nav · push · place)
#       상대가 로더로 향하고 있거나 붙어 있다. 지금 반경 밖이어도 곧 들어온다.
#       거리를 보면 "멀다" 는 오답이 나온다. 그래서 단계가 끝날 때까지 기다린다.
#
#   떠나는 중 (return)  ← PEER_LEAVING_STAGES
#       상대가 로더에서 멀어지고 있다. 여기서만 거리가 답을 준다. 반경을 벗어난
#       순간 진입해도 되므로, return 이 순찰 시작점까지 6.5 m 를 다 가는 것을
#       기다리지 않는다.
#
# ★ 앞으로 들어올 자리다. docs/03 의 "매거진 place 후 스택 있으면 가지러 감" 이
#   붙으면 그 단계도 갈라 넣어야 한다 — 다만 그건 "머무는" 동작이라 다가오는
#   쪽도 떠나는 쪽도 아니다. 반경으로는 안 풀리고 별도 처리가 필요하다.
PEER_LEAVING_STAGES = [RETURN]

# HOLD_BACK 이 픽업존에서 붙잡고 있는 상대 단계 — 상대가 로더 구역 안에서
# '움직이고 있는' 동안이다.
#
# ★ 대기 장소가 로더 바로 앞이라 거기로 가는 것 자체가 로더 구역에 들어가는
#   것이다. 그 구역에서 상대가 움직이고 있으면 둘이 같은 공간에서 엇갈린다.
#   nav_server 에 회피가 없으므로 그게 곧 충돌이다.
#
#       nav · push  상대가 로더로 들어오는 중
#       return      상대가 로더에서 빠져나가는 중 — 들어올 때와 같은 공간을 쓴다
#
#   ★ place 는 일부러 뺐다. 상대가 로더에 '멈춰' 있는 유일한 구간이고, 바로
#     그때가 이쪽이 대기 장소까지 올라가기에 안전한 때다. 여기서 올라가 둬야
#     상대가 비키는 순간 바로 들어갈 수 있다 — 그게 대기 장소를 두는 이유다.
#
#   그래서 우회 가지가 두 번 기다린다. HOLD_BACK 이 구역 진입을 막고,
#   WAIT 이 로더 진입을 막는다.
HOLD_BACK_STAGES = [NAV, PUSH, RETURN]

# 상태 발행 주기. 이 값이 상대가 보는 정보의 최대 지연이다 — 1 초로 두면
# 상대가 1 초 묵은 값으로 출발 판단을 한다. 메시지가 짧은 문자열이라
# 올려도 비용이 없다. (carrier_code_reader 는 변화만 보므로 영향 없다.)
STATE_PUBLISH_PERIOD_S = 0.2


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

# 순찰 정차점의 주인. 웹 관제 UI 「설정 > 선반」 탭이 읽고 쓰는 파일이고
# (그 파일 머리주석과 docs/DB구성.md §10-1), cobot3_perception/
# carrier_code_reader.py 도 같은 파일에서 관측 자세를 읽는다.
DEFAULT_SHELVES_YAML = WS_ROOT / "src/cobot3_bringup/config/shelves.yaml"

# isaacpjt 는 ament 패키지가 아니라 그냥으로는 import 되지 않는다. carrier_code.py 는
# 의존성 없는 순수 파이썬이고 QR 코드 규칙의 유일한 주인이라, 복사본을 만드는 대신
# 경로를 열어 그 파일 하나를 쓴다 — cobot3_perception/carrier_code_reader.py 가
# sim_client 를 집어 오는 방식과 같은 관례다.
sys.path.insert(0, str(WS_ROOT / "isaacpjt" / "assets"))
import carrier_code  # noqa: E402

# ── QR 원문 -> grasp.yaml / place.yaml 의 variant 키 ──────────────────────
# 옛 씬의 QR 은 "1" 또는 "2" 한 글자만 담았다. 16종 에셋으로 바꾸면
# "F1-MGZB-1" 이 온다. 씬 교체가 끝나기 전까지 둘 다 받는다 — 그래야 씬을
# 언제 바꾸든 로봇이 멈추지 않는다. 교체가 끝나면 이 표와 아래 폴백을 지운다.
NUMERIC_TO_VARIANT = {"1": "magazine_1_orange", "2": "magazine_2_blue"}


def _variant_of(payload):
    """QR 원문 -> variant 키. 못 풀면 None.

    새 페이로드는 carrier_code 가 푼다. 거기서 나오는 base_asset 의 확장자만
    떼면 지금 grasp.yaml / place.yaml 의 키와 그대로 맞는다:

        "F1-MGZB-1" -> base_asset "magazine_2_blue.usda" -> "magazine_2_blue"

    덕분에 yaml 키 이름 바꾸기를 씬 교체와 분리할 수 있다. 나중에 yaml 키를
    carrier_type(magazine_blue)으로 바꾸면 아래 한 줄만 고치면 된다:

        return f"{info.family}_{info.color}"

    자리를 보지 않고 아는 품목 코드를 찾는 방식이라, 로트 날짜가 끼어도
    (F1-260921-MGZO-1) 그대로 읽힌다.
    """
    info = carrier_code.parse_code(payload)
    if info is not None:
        return info.base_asset[:-len(".usda")]
    return NUMERIC_TO_VARIANT.get(payload)          # 옛 씬용 폴백


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


def _unflatten_route(flat):
    """[x0,y0,yaw0, x1,y1,yaw1, ...] → [(x0,y0,yaw0), (x1,y1,yaw1), ...].

    평탄화돼 오는 이유는 위 DEFAULT_PATROL_ROUTE 주석 참고.

    ★ 잘못된 입력에는 예외를 던진다 — 빈 리스트를 돌려주지 않는다.
      이 값이 비거나 어그러질 수 있는 경로는 "launch 가 넘긴 숫자 표가 틀렸다"
      하나뿐이고(파라미터를 아예 안 주면 declare_parameter 의 기본값이 들어온다),
      그건 런타임 상황이 아니라 설정 실수다. 미션 도중에 멈추는 것보다 노드가
      뜰 때 죽는 편이 낫다 — 런치 로그에 바로 보이고, 좌표를 모르는 로봇이
      주행 goal 을 내는 일도 없다. 반쯤 읽은 좌표로 움직이는 것이 제일 나쁘다.
    """
    vals = [float(v) for v in flat]
    if not vals or len(vals) % 3 != 0:
        raise ValueError(
            f"patrol_route 의 길이가 {len(vals)} 다 — 비어 있지 않고 3 의 배수여야 "
            f"한다. 정차점 하나가 (x, y, yaw_deg) 세 칸이다. "
            f"mission_nodes.launch.py 의 이 로봇 항목을 확인해라.")
    return [tuple(vals[i:i + 3]) for i in range(0, len(vals), 3)]


def _shelf_route(shelf):
    """shelves.yaml 의 선반 하나 → 순찰 경로 [(x, y, yaw_deg), ...].

    waypoint_start → waypoint_end 두 점이다. 선반 하나에 정차점이 정확히 둘이라
    지금의 전진/후진 왕복(두 점 사이 직선)과 그대로 맞는다 — 정차점 개수를 늘릴
    일이 생기면 그때 이 함수만 고치면 된다.

    ★ 단위가 다르다. shelves.yaml 은 meta.units 가 "m, rad" 라 theta 가
      라디안인데, 이 파일의 좌표는 yaw_deg 다. 변환을 빼먹으면 SHELF-B 의
      theta=3.14159 가 3.14 도가 되어 로봇이 선반을 등진 채 선다.

    좌표가 하나라도 비어 있으면 None 을 돌려준다. 화면의 미입력은 null 로
    저장되는데(web/backend/app/shapes.py 의 to_number), 반쯤 채워진 좌표로
    주행 goal 을 내는 것이 제일 나쁘다.
    """
    route = []
    for key in ("waypoint_start", "waypoint_end"):
        wp = shelf.get(key)
        if not isinstance(wp, dict):
            return None
        try:
            x, y, theta = (float(wp[k]) for k in ("x", "y", "theta"))
        except (KeyError, TypeError, ValueError):
            return None
        route.append((x, y, math.degrees(theta)))
    return route


def _shelf_is_taught(shelf):
    """web/backend/app/shapes.py 의 shelf_is_taught 와 **같은 규칙** — 층이 하나
    이상 있고 모든 층의 arm_teach_pose 여섯 칸이 다 채워져 있을 것.

    웹이 배정 때 422 로 걸러야 하는 조건이지만(ExecuteTask.action NOT_TAUGHT),
    파일이 그 사이 바뀔 수 있어 goal 을 받을 때 한 번 더 본다.
    """
    passes = [p for p in (shelf.get("scan_passes") or []) if isinstance(p, dict)]
    if not passes:
        return False
    for p in passes:
        joints = p.get("arm_teach_pose")
        if not isinstance(joints, (list, tuple)) or len(joints) < 6:
            return False
        if any(j is None or (isinstance(j, str) and not j.strip()) for j in joints[:6]):
            return False
    return True


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
        # 생산 트래킹 — 이 단계가 시작된 시각. Freeze 가 끝날 때 읽어 간다.
        # 시뮬과 벽시계를 둘 다 찍는 이유는 docs/DB구성.md §4-2.
        self.started_stamp, self.started_wall = self.node.now_pair()

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

    found=false 는 SCAN_RETRIES 번 재시도한 뒤에도 안 되면 soft 실패다 — Freeze
    가 얼리지 않고 FAILURE 를 그대로 올려서 캐리어 처리 가지가 통째로 FAILURE
    가 되고, Selector 가 순찰로 넘어간다. CarrierScan.srv 응답 주석이 정한
    거동이다.
    """

    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.bb = _blackboard(name, ("kind", "variant", "carrier_id", "qr_pose"))
        self.soft = False
        self.future = None
        self.attempt = 0

    def initialise(self):
        self.future = None
        # ★ SCAN 은 어떤 실패도 로봇을 얼리지 않는다 — soft 를 처음부터 True 로 둔다.
        #   판독이 안 되면 무조건 순찰로 돌아간다. SCAN 은 에러가 아니다.
        #   (docs/DB구성.md §4-6 — 그래서 DB 에도 scan 행이 없다)
        self.soft = True
        self.attempt = 0
        now = time.monotonic()
        self.server_deadline = now + SERVER_WAIT_S
        self.deadline = now + SCAN_TIMEOUT_S

    def update(self):
        if time.monotonic() > self.deadline:
            self.node.on_scan_not_found()      # 쿨다운. 없으면 곧바로 또 시도한다
            self.feedback_message = f"TIMEOUT({SCAN_TIMEOUT_S:.0f}s)"
            return Status.FAILURE

        if self.future is None:
            if not self.node.carrier_scan.service_is_ready():
                if time.monotonic() > self.server_deadline:
                    self.node.on_scan_not_found()
                    self.feedback_message = "SERVICE_UNAVAILABLE(/perception/carrier_scan)"
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
                # 재시도 — observe_pose 부터 다시 걸어 팔을 재정렬하고 카메라를
                # 새로 캡처한다. future 를 비우면 다음 tick 에 새 요청이 나간다.
                self.node.get_logger().info(
                    f"SCAN 미판독 — 재시도 {self.attempt}/{SCAN_RETRIES}")
                self.future = None
                return Status.RUNNING
            # 재시도까지 다 썼다 — 세우는 사이 라벨이 시야에서 벗어났거나
            # 다수결을 못 채웠다고 본다. 멈추지 않고 순찰로 돌아간다 — 물건은
            # 그 자리에 그대로 있다.
            self.soft = True
            self.node.on_scan_not_found()
            self.feedback_message = (
                f"NOT_FOUND(found=false) x{self.attempt} — patrol 로 돌아간다")
            return Status.FAILURE

        variant = _variant_of(res.payload)
        if variant is None:
            self.node.on_scan_not_found()
            self.feedback_message = f"UNKNOWN_PAYLOAD({res.payload!r})"
            return Status.FAILURE

        qr_pose = PoseStamped()
        qr_pose.header = res.header          # header 정보
        qr_pose.pose = res.qr_pose           # 위치 및 방향 정보
        self.bb.kind = res.payload
        self.bb.variant = variant
        self.bb.carrier_id = res.payload      # 페이로드 자체가 개체 ID 다
        self.bb.qr_pose = qr_pose
        self.node.on_scan_ok()
        self.node.get_logger().info(
            f"scan — 종류={self.bb.kind} variant={self.bb.variant} "
            f"carrier={self.bb.carrier_id}")
        self.feedback_message = f"{self.bb.variant}"
        return Status.SUCCESS


class ObservePoseLeaf(py_trees.behaviour.Behaviour):
    """순찰을 시작하기 전에 팔을 관측 자세로 세운다.

    왜 트리에 이 단계가 있나
      이전에는 이 일을 아무도 "단계" 로 두지 않았다. carrier_code_reader 가
      선반 구역에 들어온 것을 보고 자기 타이머 안에서 observe_pose 를 불렀다
      (그 노드의 _armed 플래그). 그러면 베이스는 이미 순찰 goal 을 받아
      움직이는 중인데 팔이 그와 동시에 올라간다 — 순서가 보장되지 않는다.
      요구는 "시작점에 서고 → 자세를 잡고 → 그 다음 순찰" 이므로, 기다릴 수
      있는 자리, 즉 트리의 잎이어야 한다.

    ★ 이 잎 하나가 START 경로와 RETURN 경로를 둘 다 덮는다.
      순찰 가지가 Sequence(memory=True)[START(OneShot), POSE, PATROL] 이고,
      최상위 Selector 가 memory=False 라 캐리어 처리 가지에 선점당하면 순찰
      가지가 INVALID 로 끊긴다. 끊긴 Sequence 는 다음에 다시 tick 될 때
      RUNNING 이 아니므로 첫 자식부터 다시 시작한다 — START 는 OneShot 이라
      캐시된 SUCCESS 를 그냥 돌려주고, POSE 가 다시 돈다. 즉 RETURN 이
      patrol_route[0] 에 데려다 놓은 뒤 자동으로 "자세 → 순찰" 순서가 된다.
      (py_trees 2.6.0 에서 이 거동을 직접 확인했다: START 1회, POSE 2회)

    ★ /orchestrator/state 가 이 단계 동안 "pose" 다 — "patrol" 이 아니다.
      carrier_code_reader 는 정확히 "patrol" 일 때만 QR 을 폴링하므로, 팔이
      움직이는 동안은 폴링하지 않는다. 의도한 것이다.

    ★ 받는 쪽에 거는 조건 (cobot3_perception 담당)
      - perception/observe_pose (std_srvs/Trigger) 서버를 연다.
      - 자기 순찰 자세 이름을 파라미터로 들고, 요청을 받으면 그 자세로 간다.
      - 팔이 자세에 도착한 뒤에 응답한다. 먼저 응답하면 이 잎이 기다리는
        의미가 없어지고 순서 보장이 사라진다.
      - 자기 타이머에서 observe_pose 를 부르던 자동 arming 은 걷어내야 한다.
        안 그러면 두 노드가 같은 팔에 명령을 보낸다.
    """

    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.future = None

    def initialise(self):
        # ★ 순찰 경로 갈아끼우기의 유일한 지점이다. 여기서 하는 이유는
        #   TaskManager._on_reload_config 독스트링 참고.
        self.node.commit_pending_route()
        self.future = None
        now = time.monotonic()
        self.server_deadline = now + SERVER_WAIT_S
        self.deadline = now + OBSERVE_POSE_TIMEOUT_S

    def update(self):
        if self.node.observe_pose is None:
            # 서비스 이름이 비어 있다 — 이 단계를 끈 것이다.
            self.feedback_message = "건너뜀(observe_pose_service 가 비었다)"
            return Status.SUCCESS

        if time.monotonic() > self.deadline:
            self.feedback_message = f"TIMEOUT({OBSERVE_POSE_TIMEOUT_S:.0f}s)"
            return Status.FAILURE

        if self.future is None:
            if not self.node.observe_pose.service_is_ready():
                if time.monotonic() > self.server_deadline:
                    self.feedback_message = (
                        f"SERVICE_UNAVAILABLE({self.node.observe_pose_name})")
                    return Status.FAILURE
                self.feedback_message = "서비스 대기"
                return Status.RUNNING
            self.future = self.node.observe_pose.call_async(Trigger.Request())

        if not self.future.done():
            return Status.RUNNING

        res = self.future.result()
        if not res.success:
            self.feedback_message = f"REJECTED({res.message or 'UNKNOWN'})"
            return Status.FAILURE

        self.node.get_logger().info(
            f"관측 자세 완료 — 순찰 시작 ({res.message})" if res.message
            else "관측 자세 완료 — 순찰 시작")
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


class PeerClear(py_trees.behaviour.Behaviour):
    """상대 로봇이 로더 차선을 쓰고 있지 않은가. 배송 경로를 고르는 분기 조건이다.

    "쓰고 있다" 의 기준은 node.peer_busy() 가 정한다 — 상대 state 가
    peer_busy_stages(기본 nav · push · place · return) 중 하나인가. 왜 그 넷이고
    왜 approach · wait 은 아닌지는 모듈 독스트링의 "배송 경로를 고르는 기준" 에
    적어 뒀다.

        SUCCESS  직행 가지 — 지금까지처럼 로더로 바로 간다
        FAILURE  우회 가지 — 차선 밖 대기 자리로 가서 기다린다

    FAILURE 가 에러가 아니라 경로 선택이라 Freeze 로 감싸지 않는다. 감싸면
    로봇이 얼어붙는다.

    ★ PICK 직후 한 번만 평가된다. 감싸는 Selector 가 memory=True 라 한 번 고른
      가지를 끝까지 들고 가기 때문이다. 즉 출발 시점의 판단으로 커밋한다 —
      직행을 고른 뒤에 상대가 차선에 들어오면 로더에서 마주친다. 이 사각지대를
      없애려면 대기 자리를 무조건 경유해야 하는데(그러면 판단 지점이 항상 로더
      몇 초 앞이 된다), 지금은 로더가 비어 있는 흔한 경우의 정차 비용을 아끼는
      쪽을 골랐다. 사각지대를 줄이려면 peer_busy_stages 를 넓히면 된다.
    """

    def __init__(self, name, node):
        super().__init__(name)
        self.node = node

    def update(self):
        busy, why = self.node.peer_busy()
        if busy:
            self.node.get_logger().info(f"로더 차선 사용 중({why}) — 대기 자리로 우회한다")
            self.feedback_message = f"우회 — 상대 {why}"
            return Status.FAILURE
        self.feedback_message = f"직행 — {why}"
        return Status.SUCCESS


class WaitForPeer(py_trees.behaviour.Behaviour):
    """차선 밖 대기 자리에서 상대가 차선을 비우기를 기다린다.

    액션 클라이언트도 스레드도 future 도 없다. 캐시된 상대 상태를 읽어
    RUNNING 이나 SUCCESS 를 돌려주는 조건 잎이다.

    실패는 둘이다. 어느 쪽이든 Freeze 가 받아서 그 자리에 세우고 웹으로 보낸다.
      PEER_FROZEN       상대가 차선 안에서 얼어붙었다. 물리적으로 비켜지지
                        않으므로 기다려 봐야 소용없다. 사람이 상대를 풀어야 한다.
      PEER_WAIT_TIMEOUT PEER_WAIT_TIMEOUT_S 를 넘겼다.

    풀리는 조건은 둘 중 먼저 오는 것이다.
      상태 변화  상대가 양보 목록을 벗어나면 PEER_CLEAR_DWELL_S 뒤에 진입한다.
                 상한 단계로 이미 세고 있었으면 그 시각을 기준으로 쓰므로,
                 상대가 상한보다 빨리 끝내면 추가 대기 없이 바로 간다.
      반경       상대가 로더에서 멀어지는 중인 단계(PEER_LEAVING_STAGES)에
                 있고, 로더 주차점에서 loader_clear_radius_m 밖으로 나갔으면
                 아직 그 단계여도 진입한다. 상대 위치를 못 받으면 이 경로는
                 쓰지 않고 상태 변화만 기다린다.
    어느 경로든 상대가 로더 반경 안에 있으면 먼저 막힌다 — 상태와 무관하다.

    상대를 한 번도 본 적이 없으면(peer_busy 가 "상대 없음") 즉시 통과한다 —
    한 대만 띄웠을 때 영원히 기다리는 걸 막는다.
    """

    def __init__(self, name, node, stages=None, use_radius=True, arrive_log=""):
        """stages      기다릴 상대 단계. None 이면 peer_busy_stages 전체.
        use_radius  로더 반경으로도 판정할지. 로더에서 멀리 떨어져 기다리는
                    자리(픽업존)에서는 반경이 의미가 없어서 끈다.
        arrive_log  이 잎에 처음 들어갈 때 남길 로그 한 줄.
        """
        super().__init__(name)
        self.node = node
        self.stages = stages
        self.use_radius = use_radius
        self.arrive_log = arrive_log
        self.soft = False

    def _busy(self):
        """(기다려야 하나, 이유). stages 가 있으면 그 목록으로만 본다."""
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
        # set_patrol_target 이 같은 이유로 즉시 발행하는 것과 같다.
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
        #    상태 목록이 못 잡는 경우(엉뚱한 단계인데 물리적으로 로더에 붙어 있다)
        #    까지 덮는 안전망이고, 실제로 로더에 들어가려는 이 순간이 그 안전망이
        #    의미를 갖는 유일한 자리다.
        if dist is not None and dist < need:
            self._clear_since = None
            self.feedback_message = f"대기 — 상대가 로더 {dist:.2f}/{need:.2f} m"
            return Status.RUNNING

        # ② 상대가 로더에서 멀어지는 중이면, 반경을 벗어난 것으로 충분하다.
        #    단계가 끝날 때까지(return 이면 6.5 m 를 다 갈 때까지) 기다리지 않는다.
        if busy and stage in PEER_LEAVING_STAGES:
            if dist is None:
                # 위치를 못 받는다. 거리 판정을 포기하고 상태 변화를 기다린다.
                self.feedback_message = f"대기 — 상대 {why} (위치 모름)"
                return Status.RUNNING
            busy, why = False, f"로더 {dist:.2f} m 밖"

        # ③ 상대가 로더로 다가오는 중이면 거리와 무관하게 기다린다. 지금 멀어도
        #    곧 들어오므로, 거리를 보면 "멀다" 는 오답이 나온다.
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


class CycleDone(py_trees.behaviour.Behaviour):
    """배치와 복귀까지 끝났다. 손이 비었다고 확정하고 순찰로 돌아간다."""

    def __init__(self, name, node, waypoints):
        super().__init__(name)
        self.node = node
        self.waypoints = waypoints
        self.bb = _blackboard(name, ("kind", "variant", "carrier_id", "qr_pose", "run_id"))

    def update(self):
        self.node.get_logger().info(
            f"사이클 완료 (carrier={self.bb.carrier_id}) — patrol 로 복귀")
        # 웹 작업이었다면 여기가 끝이다 — run_id 를 지우기 전에 알려야 result 에 실린다.
        self.node.on_task_cycle_done()
        self.bb.kind = ""
        self.bb.variant = ""
        self.bb.carrier_id = ""
        self.bb.qr_pose = None
        self.bb.run_id = ""          # 다음 미션은 새 run_id 를 받는다
        # RETURN 이 patrol_route[0] 까지 데려다 놨다. 순찰은 그 다음 점부터
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

        if child.status == Status.SUCCESS:
            # ★ 생산 트래킹의 발행 지점이다. pick · nav · place · return 넷이 전부
            #   이 데코레이터를 지나가므로 여기 한 곳만 고치면 네 단계가 다 걸린다.
            if self.stage == SCAN:
                # 페이로드를 처음 확보한 순간 = 미션 하나의 시작. run_id 를 발행한다.
                # (scan 자체는 행을 만들지 않는다 — LOGGED_STAGES 에 없다)
                self.node.new_run()
            elif self.stage in LOGGED_STAGES:
                self.node.emit_trace(self.stage, child, ok=True)
            self.feedback_message = child.feedback_message
            return child.status

        if child.status != Status.FAILURE:
            self.feedback_message = child.feedback_message
            return child.status

        reason = child.feedback_message or "UNKNOWN"
        if getattr(child, "soft", False):
            self.feedback_message = reason
            return Status.FAILURE

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
        route = self.node.patrol_route
        target_idx = self.idx % len(route)
        self.idx += 1
        self.node.set_patrol_target(target_idx)
        return NavigateTo.Goal(pose=_to_pose(route[target_idx]))


def build_tree(node):
    """이 모듈 맨 위 docstring 의 트리를 조립한다."""
    bb = _blackboard("build_tree", ("variant", "qr_pose"))
    # START 와 RETURN 이 둘 다 patrol_route[0] 에 데려다 놓으므로, 순찰은 그
    # 다음 점부터 잇는다 — 안 그러면 이미 서 있는 자리로 goal 을 한 번 더 보낸다.
    waypoints = _NextWaypoint(node, start_idx=1)

    scan = Freeze("SCAN", ScanLeaf(SCAN, node), node, SCAN)

    pick = Freeze("PICK", ActionLeaf(
        PICK, node, node.pick, "manipulation/pick_carrier", PickCarrier.Result,
        make_goal=lambda: PickCarrier.Goal(variant=bb.variant, qr_pose=bb.qr_pose),
        timeout_s=PICK_TIMEOUT_S,
        feedback_cb=node.log_phase("PICK")), node, PICK)

    # ── 배송 — PICK 직후 상대를 한 번 보고 두 경로 중 하나를 고른다 ──────
    # 직행: 상대가 로더 차선을 안 쓰고 있다. 지금까지와 같이 로더로 바로 간다.
    # 우회: 상대가 로더로 가고 있거나 내려놓고 있다. 차선 밖 대기 자리로 가서
    #       차선이 비기를 기다린 뒤 들어간다.
    #
    # Selector memory=True 가 중요하다 — 한 번 고른 가지를 그 가지가 끝날 때까지
    # 들고 간다. memory=False 면 매 tick 상대를 다시 보고, 주행 중에 상대가
    # 차선에 들어오는 순간 직행 가지가 무효화된다. 그러면 미션 Sequence 가
    # memory=True 라 PICK 부터 다시 시작해 버린다 — 이미 캐리어를 들고 있는데
    # 또 집으려 든다(docs/02 §3-4 가 복구 가지에서 지적한 것과 같은 함정).
    #
    # 그 대신 사각지대가 생긴다. 직행을 고른 뒤 상대가 차선에 들어오면 로더에서
    # 마주친다. 없애려면 대기 자리를 무조건 경유해 판단 지점을 항상 로더 몇 초
    # 앞에 두면 되는데, 로더가 비어 있는 흔한 경우의 정차 비용을 아끼는 쪽을
    # 골랐다. peer_busy_stages 를 넓히면 사각지대가 줄어든다.
    #
    # ★ nav 와 push 는 목적지가 같지만 잎 객체를 따로 만든다. py_trees 잎은
    #   트리에서 한 자리만 차지한다(docs/02 §4-5). 그리고 stage 가 갈려 있어야
    #   DB 의 UNIQUE(run_id, stage, attempt) 에서 두 행이 안 부딪힌다 —
    #   같은 이름이면 뒤 행이 ON CONFLICT DO NOTHING 으로 조용히 사라진다.
    nav = Freeze("NAV", ActionLeaf(
        NAV, node, node.nav, "navigation/navigate_to", NavigateTo.Result,
        make_goal=lambda: NavigateTo.Goal(pose=_to_pose(TEST_LOADER)),
        timeout_s=NAV_TIMEOUT_S, moves_base=True), node, NAV)

    direct = py_trees.composites.Sequence(
        "직행", memory=True,
        children=[PeerClear("상대 한가?", node), nav])

    approach = Freeze("APPROACH", ActionLeaf(
        APPROACH, node, node.nav, "navigation/navigate_to", NavigateTo.Result,
        make_goal=lambda: NavigateTo.Goal(pose=_to_pose(node.staging_pose)),
        timeout_s=NAV_TIMEOUT_S, moves_base=True), node, APPROACH)

    # ★ 대기 장소가 로더 바로 앞이라, 거기로 가는 것 자체가 로더 구역에 들어가는
    #   것이다. 그 구역에서 상대가 움직이고 있으면(들어오든 나가든) 둘이 같은
    #   공간에서 엇갈린다. 그래서 픽업존에서 먼저 기다린다 — 상대가 로더에
    #   '멈춰 있는' 동안(place)에만 올라간다.
    hold_back = Freeze("HOLD_BACK", WaitForPeer(
        HOLD_BACK, node, stages=HOLD_BACK_STAGES, use_radius=False,
        arrive_log="상대가 로더로 오는 중 — 픽업존에서 기다린다"),
        node, HOLD_BACK)

    wait = Freeze("WAIT", WaitForPeer(
        WAIT, node, arrive_log="대기 장소 도착 — 로더가 비기를 기다린다"),
        node, WAIT)

    push = Freeze("PUSH", ActionLeaf(
        PUSH, node, node.nav, "navigation/navigate_to", NavigateTo.Result,
        make_goal=lambda: NavigateTo.Goal(pose=_to_pose(TEST_LOADER)),
        timeout_s=NAV_TIMEOUT_S, moves_base=True), node, PUSH)

    detour = py_trees.composites.Sequence(
        "우회", memory=True, children=[hold_back, approach, wait, push])

    to_loader = py_trees.composites.Selector(
        "배송", memory=True, children=[direct, detour])

    # 놓을 자리는 종류로 정해진다 — 좌표를 넘기지 않는다.
    place = Freeze("PLACE", ActionLeaf(
        PLACE, node, node.place, "manipulation/place_carrier", PlaceCarrier.Result,
        make_goal=lambda: PlaceCarrier.Goal(variant=bb.variant),
        timeout_s=PLACE_TIMEOUT_S,
        feedback_cb=node.log_phase("PLACE")), node, PLACE)

    # 배치를 마친 자리(TEST_LOADER)는 순찰 경로에서 멀다. 순찰 잎이 어차피
    # 다음 정차점으로 goal 을 내기는 하지만, 복귀를 단계로 세워 두면 어디서
    # 실패했는지가 갈리고(TraceEvent.msg 의 stage 목록에도 RETURN 이 있다),
    # 나중에 배터리 검사를 끼울 자리가 생긴다 — NavigateTo.action 의
    # "넘으면 다음 RETURN 에서 dock_pad 로" 가 여기다.
    ret = Freeze("RETURN", ActionLeaf(
        RETURN, node, node.nav, "navigation/navigate_to", NavigateTo.Result,
        make_goal=lambda: NavigateTo.Goal(pose=_to_pose(node.patrol_route[0])),
        timeout_s=NAV_TIMEOUT_S, moves_base=True), node, RETURN)

    mission = py_trees.composites.Sequence(
        "캐리어 처리", memory=True,
        children=[Detected("detected?", node), Hold(HOLD, node),
                  scan, pick, to_loader, place, ret,
                  CycleDone("사이클 완료", node, waypoints)])

    # ── 순찰 가지 ────────────────────────────────────────────────────────
    # 순서가 곧 요구사항이다: 시작점으로 이동 → 관측 자세 → 순찰.
    #
    # ★ START 는 평생 한 번이지만 POSE 는 재진입마다 다시 돈다.
    #   캐리어 처리 가지에 선점당하면 이 Sequence 가 INVALID 로 끊기고, 다음에
    #   다시 tick 될 때 RUNNING 이 아니므로 첫 자식부터 시작한다. START 는
    #   OneShot 이라 캐시된 SUCCESS 를 그냥 돌려주고 주행 goal 을 새로 내지
    #   않으며, POSE 가 다시 돈다. 그래서 RETURN 이 patrol_route[0] 에 데려다
    #   놓은 뒤에도 "자세 → 순찰" 순서가 저절로 성립한다 — 잎 하나가 START
    #   경로와 RETURN 경로를 둘 다 덮는다(ObservePoseLeaf 독스트링 참고).
    patrol = Freeze("PATROL", py_trees.decorators.SuccessIsRunning(
        name="순찰", child=ActionLeaf(
            PATROL, node, # ★ PATROL만 direct driving
            node.patrol_nav, "navigation/patrol_to",
            NavigateTo.Result, make_goal=waypoints,
            timeout_s=NAV_TIMEOUT_S,
            ok_fail_reasons=(NavigateTo.Result.CANCELED,),moves_base=True,),),
            node,PATROL,)

    start = py_trees.decorators.OneShot(
        "START(1회)", 
        child=Freeze("START", ActionLeaf(
            START, node,
            # Nav2
            node.nav, "navigation/navigate_to",
            NavigateTo.Result, make_goal=lambda: NavigateTo.Goal(
            pose=_to_pose(node.patrol_route[0])),
                        timeout_s=NAV_TIMEOUT_S, moves_base=True,),
                        node, START,),
        policy=(py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION),)
    
    pose = Freeze("POSE", ObservePoseLeaf(POSE, node), node, POSE)

    patrol_branch = py_trees.composites.Sequence(
        "순찰 가지", memory=True, children=[start, pose, patrol])

    # ── 웹 작업 지시 (ExecuteTask.action) ────────────────────────────────
    # 별도 가지가 아니라 순찰 가지의 문지기다. 웹이 가져가는 것은 "다음에 어느
    # 선반" 하나뿐이라(.action 머리주석) 미션 순서는 그대로 두고, 순찰이
    # 돌아도 되는지(goal 이 있는지)와 어느 선반을 도는지만 바깥에서 정한다.
    #
    # EternalGuard 는 tick 마다 조건을 다시 본다. 조건이 닫히면 자식을 INVALID
    # 로 끊고 FAILURE 를 돌려준다 — 순찰 잎의 terminate(INVALID) 가 진행 중인
    # NavigateTo goal 을 거둔다(ActionLeaf.terminate). 그래서 STOP 이 "멈춰라"
    # 를 따로 보내지 않아도 서고, goal 이 선반을 바꾸면 한 tick 닫았다 열어
    # START(캐시) → POSE → 순찰로 새 경로에 다시 들어간다(task_allows_patrol).
    guarded_patrol = py_trees.decorators.EternalGuard(
        name="작업 있나?", child=patrol_branch, condition=node.task_allows_patrol)

    node.patrol_node = patrol
    # 경로가 바뀌면 다음 정차점 인덱스를 되감아야 한다 (commit_pending_route).
    node.patrol_waypoints = waypoints

    # 가지를 더한다면 여기다. 위에 있을수록 먼저 기회를 받는다 —
    # 배터리 선점(NavigateTo.action ★ hard_threshold_s)은 mission 위에 온다.
    return py_trees.composites.Selector(
        "우선순위", memory=False, children=[mission, guarded_patrol])


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


class _Task:
    """웹이 시킨 작업 하나 (ExecuteTask goal). 수락된 순간부터 result 를 낼 때까지.

    두 스레드가 본다 — 액션 execute 콜백(기다리는 쪽)과 트리 tick(끝내는 쪽).
    끝내는 쪽은 finish() 하나로만 쓰고, 기다리는 쪽은 done 만 본다. 먼저 끝낸
    쪽이 이긴다(취소와 사이클 완료가 같은 순간 겹쳐도 result 는 하나다).
    """

    def __init__(self, goal_handle):
        goal = goal_handle.request
        self.handle = goal_handle
        self.task_id = int(goal.task_id)
        self.kind = goal.kind
        self.target_ref = goal.target_ref
        self.done = threading.Event()
        self.result = None            # ExecuteTask.Result. finish() 가 한 번만 채운다
        self.canceled = False         # 웹이 SKIP/STOP 을 걸었다 (cancel_goal)
        self.mission_started = False  # 미션 단계에 들어섰다 — 취소해도 캐리어를 놓지 않는다
        self.route_committed = False  # 이 작업의 선반 경로가 순찰에 적용됐다
        self.end_legs = 0             # 경로 적용 뒤 끝점을 향해 출발한 횟수 (빈 선반 판정)
        self.last_feedback = None     # 마지막으로 올린 (stage, run_id, carrier_id)
        self._lock = threading.Lock()

    def finish(self, result):
        """result 를 확정한다. 이미 끝났으면 무시한다(먼저 끝낸 쪽이 이긴다)."""
        with self._lock:
            if self.result is not None:
                return False
            self.result = result
        self.done.set()
        return True


class TaskManager(Node):

    def __init__(self):
        super().__init__("task_manager")

        self.failed = False
        self.patrolling = False
        self._driving = None
        self._scan_cooldown_until = 0.0
        self._scan_fail_streak = 0
        self._last_snapshot = ""

        self.bb = _blackboard("task_manager", (
            "detected", "kind", "variant", "carrier_id", "qr_pose",
            "fail_stage", "fail_reason", "scan_fail_streak", "patrol_target",
            "run_id"))
        self.bb.detected = START_DETECTED_FOR_TEST
        self.bb.kind = ""
        self.bb.variant = ""
        self.bb.carrier_id = ""
        self.bb.qr_pose = None
        self.bb.run_id = ""
        self.bb.fail_stage = ""
        self.bb.fail_reason = ""
        self.bb.scan_fail_streak = 0
        self.bb.patrol_target = None

        # 콜백 그룹도 스레드도 없다. 잎이 블로킹하지 않아서 단일 스레드로 충분하다.
        self.carrier_scan = self.create_client(CarrierScan, "perception/carrier_scan")
        self.nav = ActionClient(self, NavigateTo, "navigation/navigate_to")
        self.patrol_nav = ActionClient(self, NavigateTo, "navigation/patrol_to",)
        self.pick = ActionClient(self, PickCarrier, "manipulation/pick_carrier")
        self.place = ActionClient(self, PlaceCarrier, "manipulation/place_carrier")
        self.create_subscription(
            Bool, "perception/carrier_detected", self._on_carrier_detected, 10)
        self._state_pub = self.create_publisher(String, "orchestrator/state", 10)

        # ── 생산 트래킹 (docs/DB구성.md §9) ──────────────────────────────
        # /trace/event 만 절대이름이다. event_logger 는 전역 1개라, 상대이름이면
        # 로봇마다 다른 토픽이 되어 로봇을 늘릴 때마다 로거를 고쳐야 한다.
        self.robot_id = self.get_namespace().strip("/") or "robot1"
        self._trace_pub = self.create_publisher(TraceEvent, "/trace/event", 50)
        # use_sim_time 이 켜져 있어도 벽시계를 따로 읽는다 — 둘 다 DB 에 들어간다(§4-2)
        self._wall_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self._attempts = {}          # stage -> 시도 횟수. new_run() 이 비운다
        self._frozen_node = None     # 얼어붙은 Freeze. _on_resume 이 푼다
        self.create_service(SetBool, "orchestrator/resume", self._on_resume)

        # ── 순찰 경로 (로봇마다 다른 선반을 돈다) ─────────────────────────
        # ★ 트리 조립보다 먼저다 — 잎들이 self.patrol_route 를 읽는다.
        #   출처를 고르는 규칙은 _resolve_patrol_route 독스트링에 있다.
        self.declare_parameter("shelves_yaml", str(DEFAULT_SHELVES_YAML))
        self.declare_parameter("patrol_shelf", "")
        self.declare_parameter("patrol_route", DEFAULT_PATROL_ROUTE)
        self.patrol_route, self.patrol_route_source = self._resolve_patrol_route()
        # 웹이 좌표를 고쳤을 때 갈아끼울 값. 바로 반영하지 않는 이유는
        # _on_reload_config 독스트링 ★ 참고 — POSE 잎이 커밋한다.
        self._pending_route = None
        self.patrol_waypoints = None     # build_tree 가 채운다
        self.declare_parameter("reload_service", DEFAULT_RELOAD_SERVICE)
        reload_name = self.get_parameter("reload_service").value
        if reload_name:
            self.create_service(ReloadConfig, reload_name, self._on_reload_config)

        # ── 웹 작업 지시 (ExecuteTask.action) ────────────────────────────
        # 액션 서버만 별도 콜백 그룹이다. rclpy 액션의 execute 콜백은 result 를
        # 돌려줄 때까지 블로킹하는 계약이라(_execute_task), 트리 tick 과 같은
        # 그룹에 두면 작업 하나가 도는 내내 트리가 멈춘다. 트리·구독·서비스는
        # 전부 기본(상호배제) 그룹에 남아 서로 겹치지 않는다 — "잎이 블로킹하지
        # 않아서 단일 스레드로 충분하다" 는 트리 쪽에서는 여전히 참이다.
        # 두 스레드가 같이 만지는 것은 self._task(핸드오프)와 _pending_route ·
        # _task_bounce(플래그)뿐이고, 트리와 블랙보드는 tick 스레드만 바꾼다.
        self.declare_parameter("execute_task_action", DEFAULT_EXECUTE_TASK_ACTION)
        self.declare_parameter("wait_for_task", DEFAULT_WAIT_FOR_TASK)
        self.declare_parameter("empty_sweeps", DEFAULT_EMPTY_SWEEPS)
        self.wait_for_task = bool(self.get_parameter("wait_for_task").value)
        self.empty_sweeps = int(self.get_parameter("empty_sweeps").value)
        self._task = None             # 진행 중인 _Task. 없으면 None
        self._task_lock = threading.Lock()
        self._task_claimed = False    # goal 수락 ~ _task 생성 사이의 자리표시
        self._task_bounce = False     # 선반이 바뀌었다 — 순찰 가지를 한 tick 끊어라
        self._task_reject_last = ("", 0.0)   # (사유, 시각) — 거절 경고 간격 조절
        self._task_group = ReentrantCallbackGroup()
        self._task_server = None
        action_name = self.get_parameter("execute_task_action").value
        if action_name:
            self._task_server = ActionServer(
                self, ExecuteTask, action_name,
                execute_callback=self._execute_task,
                goal_callback=self._on_task_goal,
                cancel_callback=self._on_task_cancel,
                callback_group=self._task_group)
            self.get_logger().info(
                f"ExecuteTask 액션 서버 {action_name} — "
                f"{'goal 대기' if self.wait_for_task else '순찰하며 goal 을 받는다'}")
        elif self.wait_for_task:
            self.get_logger().warning(
                "wait_for_task 가 켜져 있는데 execute_task_action 이 비어 있다 — "
                "goal 이 올 곳이 없어 순찰 가지가 영원히 닫힌다.")

        # 순찰을 시작하기 전에 팔을 세울 자세. 빈 이름이면 그 단계를 건너뛴다 —
        # 이유와 받는 쪽 조건은 DEFAULT_OBSERVE_POSE_SERVICE 주석과
        # ObservePoseLeaf 독스트링에 있다.
        self.declare_parameter("observe_pose_service", "")
        self.observe_pose_name = self.get_parameter("observe_pose_service").value
        self.observe_pose = (self.create_client(Trigger, self.observe_pose_name)
                             if self.observe_pose_name else None)
        if self.observe_pose is None:
            self.get_logger().warning(
                "observe_pose_service 가 비어 있다 — 순찰 전 관측 자세 단계를 "
                "건너뛴다. 팔이 주행 자세 그대로라 QR 이 안 보일 수 있다.")

        # ── 로더 차선 조율 (두 대가 같은 로더로 갈 때) ────────────────────
        # ★ build_tree 가 self.staging_pose 를 읽으므로 트리 조립보다 먼저다.
        self.declare_parameter("peer_state_topic", "")
        self.declare_parameter("peer_pose_topic", "")
        self.declare_parameter("loader_clear_radius_m", DEFAULT_LOADER_CLEAR_RADIUS_M)
        self.declare_parameter("peer_busy_stages", DEFAULT_PEER_BUSY_STAGES)
        self.declare_parameter("staging_pose", DEFAULT_STAGING_POSE)
        self.staging_pose = tuple(self.get_parameter("staging_pose").value)
        self.loader_clear_radius_m = float(self.get_parameter("loader_clear_radius_m").value)
        # 빈 문자열은 걸러낸다 — rclpy 는 빈 리스트의 타입을 못 정해서 [""] 로
        # 넘기는 경우가 있고, 그게 그대로 들어오면 아무 단계에도 안 맞는다.
        self._peer_busy_stages = tuple(
            x for x in self.get_parameter("peer_busy_stages").value if x)
        self._peer_stage = None       # 마지막으로 본 상대 단계
        self._peer_xy = None          # 상대 베이스 위치 (map). 없으면 None
        self._peer_failed = False     # 상대가 얼어붙었나 (FAILED 토큰)
        self._peer_last_rx = 0.0      # 마지막 수신 시각 (monotonic). 0 = 한 번도 못 받음
        peer_topic = self.get_parameter("peer_state_topic").value
        if peer_topic:
            # ★ 절대이름이다. 상대의 orchestrator/state 를 그대로 읽는다.
            #   상대이름으로 두면 자기 자신을 구독한다.
            self.create_subscription(String, peer_topic, self._on_peer_state, 10)
            self.get_logger().info(
                f"로더 차선 조율 켜짐 — 구독 {peer_topic}, "
                f"양보 대상 {list(self._peer_busy_stages)}, "
                f"대기 자리 {tuple(round(v, 3) for v in self.staging_pose)}, "
                f"이탈 판정 거리 {self.loader_clear_radius_m:.2f} m")
        # ★ 이 노드가 위치를 구독하는 유일한 자리다. 원래 task_manager 는 로봇
        #   위치를 모른다(기하는 navigation·manipulation 담당). 예외를 둔 이유는
        #   "상대가 차선을 비켰나" 를 시뮬 속도와 무관하게 판정하려면 시간이 아니라
        #   거리를 봐야 하고, 그 거리를 알 방법이 이것뿐이기 때문이다.
        #   못 받으면 거리 판정을 포기하고 상태 변화만 기다린다 — 느리지만 안전하다.
        peer_pose_topic = self.get_parameter("peer_pose_topic").value
        if peer_pose_topic:
            self.create_subscription(
                PoseWithCovarianceStamped, peer_pose_topic, self._on_peer_pose, 10)
            self.get_logger().info(f"상대 위치 구독 {peer_pose_topic}")
        elif peer_topic:
            self.get_logger().warning(
                "peer_pose_topic 이 비어 있다 — 상대가 차선을 비웠는지 거리로 못 "
                "재므로 return 이 끝날 때까지 기다린다. 안전하지만 느리다.")
        else:
            self.get_logger().info(
                "peer_state_topic 이 비어 있다 — 조율 없이 항상 직행한다 "
                "(한 대만 띄울 때의 기본값)")

        self.patrol_node = None          # build_tree 가 채운다
        self.tree = py_trees.trees.BehaviourTree(build_tree(self))
        self.tree.add_post_tick_handler(self._on_post_tick)
        self.tree.setup()

        self.create_timer(TICK_PERIOD_S, self.tree.tick)
        # 이 주기가 상대가 보는 정보의 최대 지연이다 — 상수 주석 참고.
        self.create_timer(STATE_PUBLISH_PERIOD_S, self._publish_state)

        self.get_logger().info("task_manager ready — 행동트리 tick 시작")
        pts = " → ".join(f"({x:.3f}, {y:.3f}, {yaw:.1f}°)"
                         for x, y, yaw in self.patrol_route)
        if _unflatten_route(DEFAULT_PATROL_ROUTE) == self.patrol_route:
            self.get_logger().warning(
                f"순찰 경로가 폴백값 그대로다 [{self.patrol_route_source}] — "
                f"이건 옛 레이아웃(선반 x≈-6.5) 좌표라 지금 씬에는 맞지 않는다. "
                f"로봇이 씬에 없는 자리로 간다. 위 경고에 왜 shelves.yaml 을 "
                f"못 썼는지 적혀 있다.")
        else:
            self.get_logger().info(
                f"순찰 경로 [{self.patrol_route_source}] {pts} — 두 점의 yaw 가 "
                f"같으면 제자리 회전 없이 전진/후진 왕복한다")
        if TEST_LOADER is None:
            self.get_logger().warning(
                "TEST_LOADER 가 비어 있다 — pick 까지는 되지만 nav 단계에서 멈춘다. "
                "task_manager.py 상단에 좌표를 넣어라.")

    # ── 순찰 경로 고르기 ──────────────────────────────────────────────────
    def _resolve_patrol_route(self):
        """이 로봇이 돌 순찰 경로와 그 출처를 정한다. (경로, 출처설명) 을 준다.

        출처는 셋이고 이 순서로 고른다. 어느 것이 이겼는지는 시작 로그에 반드시
        찍는다 — 두 출처가 조용히 경쟁하면 "왜 저기로 가지" 를 추적할 수 없다.

            1) shelves.yaml 에서 이 로봇에게 배정된 선반. 배정은 선반의
               assigned_robot 이 네임스페이스(self.robot_id)와 같은 것으로 본다.
               좌표의 주인은 웹 화면이다 — 「설정 > 선반」 탭이 이 파일을 읽고
               쓴다(그 파일 머리주석, docs/DB구성.md §10-1).
            2) patrol_route 파라미터. launch 가 평탄화 좌표를 직접 준 경우다.
            3) DEFAULT_PATROL_ROUTE 폴백. 옛 레이아웃 좌표라 크게 경고한다.

        patrol_shelf 파라미터를 주면 1) 에서 배정을 무시하고 그 shelf_id 를
        쓴다 — 시험용이고, assigned_robot 이 화면에 붙기 전까지의 다리이기도
        하다. shelves_yaml 을 빈 문자열로 두면 1) 을 통째로 건너뛴다.

        ★ 1) 이 실패해도 예외를 던지지 않고 2) 로 내려간다. 대신 왜 실패했는지
          반드시 경고로 남긴다. 조용히 옛 좌표로 도는 것이 제일 나쁘다.

        ★★ assigned_robot 은 웹이 지울 수 있었다 — 알고 있어야 한다.
          web/backend/app/shapes.py 의 shelf_in() 이 고정된 키 목록으로 dict 를
          새로 만들어서, 그 목록에 없는 필드는 화면에서 「설정 > 선반」 을 한 번
          저장하는 순간 조용히 사라졌다. 지금은 shelf_out 이 이 키를 화면에
          내주고 routers/settings.py 가 화면이 안 보낸 키를 파일 값으로 지키므로
          저장해도 남는다. 사람이 에디터로 지우는 경우는 여전히 있어서 아래에서
          그 경우를 따로 짚어 경고한다.
        """
        path = self.get_parameter("shelves_yaml").value or ""
        if path:
            route, source = self._route_from_shelves(
                Path(path), self.get_parameter("patrol_shelf").value or "")
            if route:
                return route, source

        route = _unflatten_route(self.get_parameter("patrol_route").value)
        if route == _unflatten_route(DEFAULT_PATROL_ROUTE):
            return route, "DEFAULT_PATROL_ROUTE 폴백"
        return route, "patrol_route 파라미터"

    def _route_from_shelves(self, path, want_shelf):
        """shelves.yaml 에서 이 로봇의 선반을 찾아 경로로 바꾼다.

        못 찾거나 좌표가 덜 찼으면 (None, "") 을 주고, 왜인지는 경고로 남긴다.
        """
        try:
            doc = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            shelves = [sh for sh in (doc.get("shelves") or []) if isinstance(sh, dict)]
        except (OSError, yaml.YAMLError) as e:
            self.get_logger().warning(f"shelves.yaml({path}) 을 못 읽었다: {e}")
            return None, ""

        ids = [str(sh.get("shelf_id")) for sh in shelves]
        if want_shelf:
            picked = [sh for sh in shelves if str(sh.get("shelf_id")) == want_shelf]
            how = f"patrol_shelf={want_shelf}"
            if not picked:
                self.get_logger().warning(
                    f"shelves.yaml 에 shelf_id={want_shelf} 가 없다 — 있는 것: {ids}")
                return None, ""
        else:
            picked = [sh for sh in shelves
                      if str(sh.get("assigned_robot") or "") == self.robot_id]
            how = f"assigned_robot={self.robot_id}"
            if not picked:
                if not any("assigned_robot" in sh for sh in shelves):
                    self.get_logger().warning(
                        "shelves.yaml 의 어느 선반에도 assigned_robot 이 없다. "
                        "선반 항목에 assigned_robot: robot1 처럼 적어야 이 로봇이 "
                        "자기 선반을 안다. 웹 저장은 이제 이 키를 지키지만"
                        "(routers/settings.py) 에디터로 지웠을 수 있다 "
                        "(_resolve_patrol_route 독스트링 ★★).")
                else:
                    assigned = {str(sh.get("shelf_id")): sh.get("assigned_robot")
                                for sh in shelves}
                    self.get_logger().warning(
                        f"shelves.yaml 에 assigned_robot={self.robot_id} 인 선반이 "
                        f"없다. 지금 배정: {assigned}")
                return None, ""

        if len(picked) > 1:
            self.get_logger().warning(
                f"{how} 에 맞는 선반이 {len(picked)} 개다 "
                f"({[str(sh.get('shelf_id')) for sh in picked]}) — 첫 번째를 쓴다. "
                f"한 로봇에 두 선반을 배정하면 어느 쪽을 도는지 알 수 없다.")

        shelf_id = str(picked[0].get("shelf_id"))
        route = _shelf_route(picked[0])
        if route is None:
            self.get_logger().warning(
                f"shelves.yaml 의 {shelf_id} 에 waypoint 좌표가 덜 찼다 — "
                f"waypoint_start · waypoint_end 의 x · y · theta 여섯 칸이 "
                f"모두 숫자여야 한다. 화면에서 채워라.")
            return None, ""
        return route, f"shelves.yaml {shelf_id} ({how})"

    def _on_reload_config(self, req, res):
        """웹이 설정을 저장한 직후 부른다 (ReloadConfig.srv).

        web/backend/app/routers/settings.py 의 _save() 가 저장할 때마다 로봇
        전부에게 보낸다. 이 노드가 관심 있는 scope 는 shelves(순찰 정차점)와
        all 뿐이고, 나머지는 조용히 성공시킨다 — "자기와 무관한 scope 를 받은
        노드는 reloaded=true, revision=\"\" 로 조용히 성공시킨다" 가 그 파일이
        정한 규칙이다. 웹이 노드마다 어떤 파일을 읽는지 알 필요가 없게 하려는
        것이다.

        ★ 받자마자 self.patrol_route 를 갈아끼우지 않는다.
          지금 순찰 중이면 진행 중인 NavigateTo goal 이 옛 좌표로 가는 중이고,
          _NextWaypoint.idx 도 옛 경로를 기준으로 돌고 있다. 새 값을
          대기시켜 뒀다가 POSE 잎이 커밋한다 — 그 잎은 순찰에 재진입할 때마다
          반드시 지나가고(START 경로든 RETURN 경로든), 그 시점엔 베이스가 서
          있어서 목표가 바뀌어도 안전하다. 그래서 "저장 즉시" 가 아니라
          "다음 순찰 사이클부터" 다.

        ★ shelves.yaml 에서 이 로봇 경로를 못 찾으면 반영하지 않는다.
          _resolve_patrol_route 는 못 찾으면 파라미터나 폴백으로 내려가는데,
          reload 에서 그걸 받아들이면 잘 돌던 로봇이 저장 한 번에 옛 레이아웃
          좌표로 떨어진다. 특히 웹이 assigned_robot 을 지우는 경우가 그렇다
          (_resolve_patrol_route ★★). 그때는 reloaded=false 로 거절하고 지금
          경로를 그대로 쓴다 — 웹이 그 사유를 로그에 남긴다.
        """
        if req.scope not in (ReloadConfig.Request.ALL, ReloadConfig.Request.SHELVES):
            res.reloaded = True
            res.revision = ""
            return res

        try:
            route, source = self._resolve_patrol_route()
        except Exception as e:                       # noqa: BLE001 — 사유를 웹에 돌려준다
            res.reloaded = False
            res.rejected_because = f"순찰 경로를 다시 읽지 못했다: {e}"
            self.get_logger().warning(f"{res.rejected_because} — 지금 경로를 유지한다")
            return res

        if not source.startswith("shelves.yaml"):
            res.reloaded = False
            res.rejected_because = (
                f"shelves.yaml 에서 {self.robot_id} 의 순찰 경로를 못 찾았다 "
                f"(대신 고른 것: {source}). 지금 경로를 유지한다 — "
                f"assigned_robot 이 지워지지 않았는지 확인해라.")
            self.get_logger().warning(res.rejected_because)
            return res

        res.revision = self._shelves_revision()
        res.reloaded = True
        if route == self.patrol_route:
            self.get_logger().info(f"reload — 순찰 경로가 그대로다 [{source}]")
            return res

        self._pending_route = (route, source)
        self.get_logger().info(
            f"reload — 새 순찰 경로를 받았다 [{source}]. 다음 순찰 시작"
            f"(POSE 단계)에 적용한다. 지금 주행 중인 목표는 건드리지 않는다.")
        return res

    def _shelves_revision(self):
        """shelves.yaml 내용의 sha256 앞 12자. 웹이 반영 여부를 판정한다."""
        path = self.get_parameter("shelves_yaml").value or ""
        try:
            return hashlib.sha256(Path(path).read_bytes()).hexdigest()[:12]
        except OSError:
            return ""

    def commit_pending_route(self):
        """대기 중인 순찰 경로를 지금 적용한다. POSE 잎이 부른다.

        정차점 인덱스를 0 으로 되감는다. 평소(RETURN 직후)에는 CycleDone 이
        1 로 둔다 — 이미 patrol_route[0] 에 서 있으니 그 다음 점부터 잇는
        것이다. 그런데 경로가 바뀌면 로봇이 서 있는 곳은 옛 경로의 시작점이라
        새 경로의 시작점이 아니다. 그래서 새 경로의 첫 점부터 다시 간다.
        """
        if self._pending_route is None:
            return
        route, source = self._pending_route
        self._pending_route = None
        old = self.patrol_route
        self.patrol_route = route
        self.patrol_route_source = source
        if self.patrol_waypoints is not None:
            self.patrol_waypoints.idx = 0
        task = self._task
        if task is not None:
            # 이 작업의 선반 경로가 적용됐다. 빈 선반 판정은 여기서부터 센다.
            task.route_committed = True
            task.end_legs = 0
        self.get_logger().info(
            "순찰 경로 적용 [" + source + "] "
            + " → ".join(f"({x:.3f}, {y:.3f}, {yaw:.1f}°)" for x, y, yaw in route)
            + f" (이전 {len(old)} 점 → 새 {len(route)} 점)")

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
        self._task_on_freeze(stage, reason)
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
        """그 시점 '물체' 의 상태. 단계의 성패(success)와는 다른 값이다.

        return 은 place 뒤라 실패해도 물체는 이미 배달됐다 — 그래서
        success=false 인데 status=COMPLETED 인 행이 나온다(docs/DB구성.md §4-3).
        """
        if stage == RETURN:
            return "COMPLETED"
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
        msg.carrier_id = self.bb.kind or ""          # QR 원문 그대로. 가공하지 않는다
        msg.robot_id = self.robot_id
        msg.run_id = self.bb.run_id or ""
        msg.stage = stage
        msg.attempt = self.attempt_of(stage)
        msg.success = bool(ok)
        msg.status = self._status_of(stage, ok)
        msg.fail_reason = "" if ok else STAGE_TO_REASON.get(stage, "nav_error")
        msg.fail_detail = "" if ok else (fail_detail or "UNKNOWN")
        msg.port = PORT_BY_STAGE.get(stage, "")
        self._trace_pub.publish(msg)

    def _on_resume(self, req, res):
        """웹 복구 — 얼어붙은 그 단계를 다시 시도한다.

        새 미션이 아니다. Freeze 가 얼어 있는 동안 RUNNING 을 돌려주므로
        memory=True 인 mission Sequence 가 그 잎을 붙들고 있고, frozen 을 풀면
        자식이 initialise() 부터 다시 돌아 goal 이 새로 나간다 — pick·nav 는
        재실행되지 않는다. 그래서 run_id 는 그대로 두고 attempt 만 올린다
        (docs/DB구성.md §4-7).
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
        self._task_count_sweep(idx)

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

    def log_phase(self, label):
        return lambda fb: self.get_logger().info(f"  {label} phase={fb.feedback.phase}")

    # ── 웹 작업 지시 (ExecuteTask.action) ────────────────────────────────
    # 스레드 경계: _on_task_goal · _on_task_cancel · _execute_task 는 액션
    # 콜백 스레드(_task_group)에서, 나머지(task_allows_patrol · _task_track ·
    # _task_count_sweep · on_task_cycle_done · _task_on_freeze)는 tick 스레드
    # 에서 돈다. 트리와 블랙보드는 tick 스레드만 바꾼다.

    def _on_task_goal(self, request):
        """goal 을 받을지. 거절하면 웹이 큐로 되돌리고 alert 를 띄운다(dispatcher)."""
        why = None
        if request.kind != "SCAN":
            why = (f"kind={request.kind} 는 아직 못 받는다 — SCAN 만 된다 "
                   f"(RECOVER 는 스테이션으로 가는 경로가 아직 없다)")
        elif self.failed:
            why = (f"{self.bb.fail_stage} 단계에서 얼어붙어 있다 — "
                   f"/{self.robot_id}/orchestrator/resume 으로 먼저 푼다")
        else:
            with self._task_lock:
                if self._task is not None or self._task_claimed:
                    why = ("작업이 이미 진행 중이다 — 로봇 하나에 하나씩이다 "
                           "(표 5 task_one_running_per_robot)")
                else:
                    self._task_claimed = True
        if why:
            # 거절된 goal 은 웹이 큐로 되돌리고 2초 뒤에 다시 보낸다(dispatcher 의
            # _IDLE_POLL_SEC). 얼어 있는 동안 같은 사유가 계속 오므로, 같은 사유는
            # 30초에 한 번만 경고로 남기고 나머지는 debug 로 내린다.
            now = time.monotonic()
            last_why, last_at = self._task_reject_last
            if why != last_why or now - last_at > 30.0:
                self._task_reject_last = (why, now)
                self.get_logger().warning(
                    f"ExecuteTask 거절 task={request.task_id} {request.target_ref}: {why}")
            else:
                self.get_logger().debug(f"ExecuteTask 거절 task={request.task_id}: {why}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_task_cancel(self, goal_handle):
        """화면의 SKIP / STOP. 받기만 하고, 실제 처리는 _execute_task 의 대기 루프가 한다."""
        return CancelResponse.ACCEPT

    def _execute_task(self, goal_handle):
        """goal 하나를 끝까지. 트리가 끝낼 때까지 여기서 기다린다(액션 스레드).

        하는 일은 셋이다 — 선반을 찾아 순찰 경로로 대기시키고, 트리가 result 를
        채울 때까지 기다리고, 취소 요청을 살핀다. 미션 자체는 기존 트리가 돈다:
        순찰 → carrier_detected → HOLD → SCAN → PICK → 배송 → PLACE → RETURN.
        """
        goal = goal_handle.request
        task = _Task(goal_handle)

        route, code, detail = self._route_for_target(goal.target_ref)
        if route is None:
            with self._task_lock:
                self._task_claimed = False
            self.get_logger().error(
                f"ExecuteTask task={task.task_id} {goal.target_ref}: {detail}")
            goal_handle.abort()
            return self._task_result(success=False, fail_reason=code, fail_detail=detail)

        if goal.resume_progress > 0.0:
            # 재개는 아직 없다 — 끊긴 지점이 아니라 scan 부터 다시 한다
            # (ExecuteTask.action: run_id 를 이어갈지도 미정 §11 #2).
            self.get_logger().warning(
                f"ExecuteTask task={task.task_id}: resume_progress="
                f"{goal.resume_progress:.2f} 는 아직 못 쓴다 — 처음부터 한다")

        with self._task_lock:
            self._task = task
        if route == self.patrol_route:
            task.route_committed = True
            how = "지금 순찰 경로 그대로"
        else:
            self._pending_route = (
                route, f"shelves.yaml {goal.target_ref} (ExecuteTask task={task.task_id})")
            if self.patrolling:
                # 다른 선반을 돌던 중이다. 순찰 가지를 한 tick 끊어 새 경로로
                # 다시 들어가게 한다. 미션 중이면 끊지 않는다 — task_allows_patrol
                # 이 미리 커밋해 두므로 RETURN 이 새 선반의 시작점으로 간다.
                self._task_bounce = True
            how = "새 경로 대기 (다음 순찰 진입에 적용)"
        self.get_logger().info(
            f"ExecuteTask 수락 task={task.task_id} {goal.kind} {goal.target_ref} — {how}")

        # 트리가 끝낼 때까지. result 는 tick 스레드가 _Task.finish 로 채운다.
        while not task.done.wait(timeout=0.2):
            if goal_handle.is_cancel_requested and not task.canceled:
                task.canceled = True
                if task.mission_started:
                    # 캐리어를 들고 있을 수 있다. 미션은 끝까지 돌리고 그 result
                    # 를 취소로 돌려준다 — 웹은 CANCELED 를 실패로 치지 않는다.
                    self.get_logger().warning(
                        f"ExecuteTask task={task.task_id} 취소 요청 — 미션이 이미 "
                        f"돌고 있어 RETURN 까지 마친 뒤 닫는다")
                else:
                    self.get_logger().info(
                        f"ExecuteTask task={task.task_id} 취소 — 순찰을 세운다")
                    task.finish(self._task_result(
                        success=False, fail_reason=ExecuteTask.Result.CANCELED,
                        fail_detail="웹이 취소했다(SKIP/STOP)"))

        with self._task_lock:
            self._task = None
            self._task_claimed = False
        result = task.result
        if task.canceled:
            goal_handle.canceled()
        elif result.success or result.fail_reason == ExecuteTask.Result.NO_CARRIER:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        self.get_logger().info(
            f"ExecuteTask 종료 task={task.task_id} success={result.success} "
            f"reason={_reason_name(ExecuteTask.Result, result.fail_reason)} "
            f"run={result.run_id[:8] or '-'}")
        return result

    @staticmethod
    def _task_result(success, fail_reason=ExecuteTask.Result.NONE, fail_detail="",
                     run_id="", more_at_target=False):
        r = ExecuteTask.Result()
        r.success = bool(success)
        r.more_at_target = bool(more_at_target)
        r.run_id = run_id or ""
        r.fail_reason = int(fail_reason)
        r.fail_detail = fail_detail or ""
        return r

    def _route_for_target(self, target_ref):
        """goal 의 target_ref(shelf_id) → 순찰 경로. 못 쓰면 (None, 사유코드, 설명).

        좌표를 goal 에 싣지 않는 이유는 .action 주석 — 자세의 주인은 파일이다.
        """
        path = self.get_parameter("shelves_yaml").value or ""
        try:
            doc = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
            shelves = [sh for sh in (doc.get("shelves") or []) if isinstance(sh, dict)]
        except (OSError, yaml.YAMLError) as e:
            return None, ExecuteTask.Result.NO_TARGET, f"shelves.yaml({path}) 을 못 읽었다: {e}"
        shelf = next((sh for sh in shelves if str(sh.get("shelf_id")) == target_ref), None)
        if shelf is None:
            ids = [str(sh.get("shelf_id")) for sh in shelves]
            return (None, ExecuteTask.Result.NO_TARGET,
                    f"shelves.yaml 에 {target_ref} 가 없다 — 있는 것: {ids}")
        route = _shelf_route(shelf)
        if route is None:
            return (None, ExecuteTask.Result.NO_TARGET,
                    f"{target_ref} 의 waypoint 좌표가 덜 찼다 — 화면에서 채워라")
        if not _shelf_is_taught(shelf):
            return (None, ExecuteTask.Result.NOT_TAUGHT,
                    f"{target_ref} 에 티칭된 자세가 없다 — 웹이 422 로 걸렀어야 한다")
        return route, ExecuteTask.Result.NONE, ""

    def task_allows_patrol(self):
        """순찰 가지 EternalGuard 의 조건. tick 마다 불린다(tick 스레드).

        False 를 돌려주면 순찰 가지가 INVALID 로 끊기고 진행 중인 주행 goal 이
        거둬진다(build_tree 의 guarded_patrol 주석). 두 경우다:
          - 선반이 바뀌어 한 tick 끊는다(_task_bounce). 다음 tick 에 다시 열린다.
          - wait_for_task 인데 goal 이 없다 — 그 자리에서 기다린다.
        열려 있는 동안, 작업의 새 경로가 대기 중이고 순찰 잎이 돌고 있지 않으면
        여기서 바로 적용한다. POSE 가 커밋하기 전에 START 가 옛 경로의 시작점
        으로 나가는 것을 막고, 미션 중이면 RETURN 이 새 시작점으로 가게 한다.
        (작업이 없을 때의 reload 는 지금처럼 POSE 가 커밋한다.)
        """
        if self._task_bounce:
            self._task_bounce = False
            return False
        task = self._task
        if task is not None and self._pending_route is not None and not self.patrolling:
            self.commit_pending_route()
        if task is None and self.wait_for_task:
            return False
        return True

    def _task_track(self, tree):
        """tick 뒤에 한 번. 미션 단계가 바뀌면 feedback 을 올린다(tick 스레드)."""
        task = self._task
        if task is None:
            return
        stage = current_stage(tree.root)
        if stage in MISSION_STAGES and not task.mission_started:
            task.mission_started = True
        fb_stage = TASK_STAGE.get(stage)
        if fb_stage is None:
            return
        key = (fb_stage, self.bb.run_id, self.bb.carrier_id)
        if key == task.last_feedback:
            return
        task.last_feedback = key
        fb = ExecuteTask.Feedback()
        fb.stage = fb_stage
        fb.progress = float(TASK_PROGRESS.get(stage, 0.0))
        fb.run_id = self.bb.run_id or ""
        fb.carrier_id = self.bb.carrier_id or ""
        task.handle.publish_feedback(fb)

    def _task_count_sweep(self, idx):
        """순찰 정차점이 바뀔 때마다(set_patrol_target). 빈 선반 판정.

        끝점(1)을 향해 두 번째로 출발한다는 것은 시작점→끝점→시작점 한 바퀴를
        감지 없이 돌았다는 뜻이다. empty_sweeps 바퀴가 차면 NO_CARRIER 로 닫는다.
        """
        task = self._task
        if task is None or not task.route_committed or task.mission_started or idx != 1:
            return
        task.end_legs += 1
        sweeps = task.end_legs - 1
        if sweeps >= self.empty_sweeps:
            self.get_logger().info(
                f"ExecuteTask task={task.task_id} {task.target_ref}: "
                f"{sweeps}바퀴 돌았는데 캐리어가 없다 — NO_CARRIER")
            task.finish(self._task_result(
                success=False, fail_reason=ExecuteTask.Result.NO_CARRIER,
                fail_detail=f"{sweeps}바퀴 순찰, 감지 없음"))

    def on_task_cycle_done(self):
        """CycleDone 이 부른다 — RETURN 까지 끝났다. 웹 작업이면 성공으로 닫는다."""
        task = self._task
        if task is None:
            return
        # more_at_target 은 아직 못 본다. 복귀한 자리가 선반 앞이지만 상태가
        # patrol 이 아니라 carrier_code_reader 가 폴링하지 않는다 — 확신이 없으면
        # false 가 안전하다(.action 주석). 남은 캐리어는 다음 goal 의 순찰이 본다.
        task.finish(self._task_result(
            success=True, run_id=self.bb.run_id, more_at_target=False))

    def _task_on_freeze(self, stage, reason):
        """얼어붙었다(on_freeze). 웹 작업이면 그 단계의 실패로 닫는다.

        트리는 그대로 얼어 있다. 웹이 /orchestrator/resume 으로 풀면 트리는 그
        단계부터 이어가지만 작업은 이미 닫혔으므로 웹이 다시 배차한다 — 얼어
        있는 동안 오는 goal 은 거절된다(_on_task_goal).
        """
        task = self._task
        if task is None:
            return
        task.finish(self._task_result(
            success=False,
            fail_reason=TASK_FAIL_REASON.get(stage, ExecuteTask.Result.NAV_FAIL),
            fail_detail=reason, run_id=self.bb.run_id))

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
        # 순찰 잎이 RUNNING 인 동안만 carrier_detected 를 받는다. POSE 로
        # 팔을 세우는 중이나 작업 중에는 patrol_node 가 RUNNING 이 아니므로
        # 자연히 닫힌다 (_on_carrier_detected 참고).
        self.patrolling = (self.patrol_node is not None
                            and self.patrol_node.status == Status.RUNNING)
        # 웹 작업 보고 — 미션 단계가 바뀔 때만 feedback 을 낸다.
        self._task_track(tree)

        # ※ 순찰 가지가 없던 동안 "완전 정지" 를 감지해 detected 를 다시 세우던
        #   임시 블록이 여기 있었다. 순찰을 되살리면서 지웠다 — SCAN 이 soft
        #   실패하면 이제 Selector 가 순찰 가지로 빠지고, 순찰하다 다시
        #   carrier_detected 를 받는다. 그게 원래 설계한 경로다.

        snapshot = py_trees.display.unicode_tree(tree.root, show_status=True)
        if snapshot != self._last_snapshot:
            self._last_snapshot = snapshot
            self.get_logger().info("\n" + snapshot)

    # ── 로더 차선 조율 ────────────────────────────────────────────────────
    def _on_peer_state(self, msg):
        """상대 task_manager 의 orchestrator/state 를 읽는다.

        문자열 형식은 _publish_state 가 만드는 것과 같고, 파싱도
        cobot3_perception/carrier_code_reader._on_orchestrator_state 와 같다.
        그 노드와 마찬가지로 모르는 필드는 그냥 무시한다.
        """
        stage, failed = None, False
        for part in msg.data.split("|"):
            part = part.strip()
            if part.startswith("state="):
                stage = part[len("state="):]
            elif part == "FAILED":
                failed = True
        self._peer_stage = stage
        self._peer_failed = failed
        self._peer_last_rx = time.monotonic()

    def _on_peer_pose(self, msg):
        self._peer_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def peer_stage(self):
        return self._peer_stage

    def peer_dist_to_loader(self):
        """상대 베이스와 로더 주차점의 거리. 위치를 모르면 None.

        중심이 TEST_LOADER 다 — stations.yaml PKG-01 place_pose 와 같은 점이고,
        로봇이 place 하려고 서는 자리다. WaitForPeer 가 이 거리를
        loader_clear_radius_m 과 비교한다.
        """
        if self._peer_xy is None:
            return None
        return math.hypot(self._peer_xy[0] - TEST_LOADER[0],
                          self._peer_xy[1] - TEST_LOADER[1])

    def peer_frozen(self):
        """상대가 차선 안에서 얼어붙었나.

        얼어붙은 자리가 차선 안이면 기다려도 안 비켜진다 — Freeze 는 tick 마다
        RUNNING 만 돌려주고 자동 복귀가 없으므로, 사람이 상대를 풀어야 한다.
        그래서 기다리지 않고 이쪽도 실패로 올려 같이 웹에 뜨게 한다.
        """
        return bool(self._peer_failed
                    and self._peer_stage in self._peer_busy_stages)

    def peer_busy(self):
        """(양보해야 하나, 사람이 읽을 이유) 한 쌍.

        판단 두 갈래다.
          한 번도 못 받았다  상대가 안 떠 있다고 보고 통과시킨다. 한 대만 띄우고
                            영원히 기다리는 걸 막는다.
          단계가 목록에 있다 양보한다.

        ★ 소식이 끊겨도 마지막으로 들은 단계를 그대로 쓴다. 수신 시각으로
          "묵었으면 양보" 규칙을 두려다 접었다. 위험한 방향은 이미 막혀 있다 —
          상대가 차선 단계에서 죽으면 마지막 메시지가 그 단계이므로 계속 양보한다.
          반대로 순찰 중에 죽은 상대(개활지에 서 있다)까지 양보 대상으로 만들면,
          죽은 로봇 때문에 살아 있는 로봇이 대기 자리에서 얼어붙는다.
          남는 구멍은 "링크가 끊긴 채 상대가 멀쩡히 차선에 진입" 하나인데, 그건
          수십 초짜리 무음이 필요해서 짧은 문턱으로는 못 잡는다.

        age 는 판단에 안 쓰고 로그에만 남긴다 — 얼마나 신선한 값으로 정했는지가
        나중에 원인 추적에 필요하다.
        """
        if not self._peer_busy_stages or self._peer_last_rx == 0.0:
            return False, "상대 없음"
        age = time.monotonic() - self._peer_last_rx
        if self._peer_stage in self._peer_busy_stages:
            return True, f"state={self._peer_stage} ({age:.1f}s 전)"
        return False, f"state={self._peer_stage} ({age:.1f}s 전)"

    def publish_state(self):
        """잎이 자기 상태를 즉시 알려야 할 때 부른다.

        set_patrol_target 이 즉시 발행하는 것과 같은 이유다 — 다음 주기를
        기다리면 그 사이 상대가 묵은 값으로 출발 판단을 한다.
        """
        self._publish_state()

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
        if self.bb.run_id:
            # 사람이 보는 용도. DB 의 run_id 앞 8자와 같아서 로그와 표를 맞춰볼 수 있다
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
    # 스레드가 둘 이상이어야 한다 — ExecuteTask 의 execute 콜백이 작업이 끝날
    # 때까지 블로킹하는 동안에도 트리가 tick 되어야 하기 때문이다. 그룹 나누기와
    # 스레드 경계는 TaskManager.__init__ 의 액션 서버 주석에 있다.
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
