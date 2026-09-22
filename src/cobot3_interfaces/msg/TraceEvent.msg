# ══════════════════════════════════════════════════════════════════════
#  보내는 노드 : cobot3_orchestrator / task_manager
#  받는 노드   : cobot3_orchestrator / event_logger
#  종류        : 토픽  /trace/event   (Reliable, depth 50)  ★ 절대이름이다
#  역할        : 미션 단계가 하나 끝날 때마다 결과를 한 건 낸다. event_logger 는 그대로
#                PostgreSQL 의 magazine_log / stack_log 에 append 한다 — 판단하지 않는다.
#  전달 내용   : 아래 필드가 DB 한 행과 거의 1:1 이다 (docs/DB구성.md §4)
# ══════════════════════════════════════════════════════════════════════

# ★ 왜 절대이름인가
#   task_manager 는 로봇마다 하나씩 뜨고 나머지 배선은 전부 상대이름이지만,
#   event_logger 는 전역 1개다. 상대이름이면 /robot1/trace/event, /robot2/trace/event
#   를 따로 구독해야 해서 로봇을 늘릴 때마다 로거를 고쳐야 한다.
#   docking_server 의 /docking/dock 이 절대이름인 것과 같은 논리다.
#
# ★ 발행 지점은 Freeze 데코레이터 한 곳이다
#   pick · nav · place · return 넷이 전부 Freeze 로 감싸져 있고(build_tree),
#   그 데코레이터가 자식의 SUCCESS 도 FAILURE 도 다 보며 stage 를 이미 들고 있다.
#   scan 은 Freeze 로 감싸져 있지만 행을 만들지 않는다 — 판독 실패는 미션이
#   시작되지도 않은 것이라 남길 행이 없다(docs/DB구성.md §4-6).
#
# ★ event_logger 가 declare_parameter 로 정의해야 하는 것
#   db_dsn      (string)  기본값은 환경변수 COBOT3_DB_DSN. 비밀번호는 ~/.pgpass 가 갖는다
#   spool_path  (string)  DB 가 죽었을 때 흘려 둘 파일 (runs/trace_spool.jsonl)
#   → run_id · robot_id · 벽시계가 이제 메시지에 실려 오므로, 로거는 자기 파라미터로
#     붙일 것이 없다. 이전 판의 amr_id · run_id 파라미터는 사라졌다.


# ── 시각 4개 ──────────────────────────────────────────────────────────
#  시뮬과 벽시계를 둘 다 담는다. Isaac 을 GUI 렌더로 돌리면 실시간보다 느려서
#  벽시계로 잰 사이클 타임은 물리적으로 틀린 값이 된다 (docs/DB구성.md §4-2).
#    사이클 타임 분석  → 시뮬 (duration_sec 이 이걸로 계산되는 생성열이다)
#    "몇 시에 배달됐나" → 벽시계
#
#  넷 다 task_manager 가 찍는다. 로거의 now() 를 쓰면 큐 지연·스풀 재적재분이
#  전부 그때로 찍힌다.
builtin_interfaces/Time started_stamp   # 시뮬 · 그 단계를 시작한 시각  → started_sim
builtin_interfaces/Time stamp           # 시뮬 · 그 단계가 끝난 시각    → ended_sim
builtin_interfaces/Time started_wall    # 벽시계 · 시작                 → started_at
builtin_interfaces/Time wall_stamp      # 벽시계 · 끝                   → ended_at
                                        #   ended_at 이 화면에 표시하는 값이다

# ── 누구의 무엇인가 ───────────────────────────────────────────────────
string carrier_id       # QR 원문 그대로. "F1-MGZB-1"          → qr_payload
                        #   가공하지 않는다. 파싱 규칙이 바뀌어도(로트 날짜 추가)
                        #   과거 행을 다시 해석할 수 있어야 한다.
                        #   kind_code 는 싣지 않는다 — 로거가 carrier_kind 표에
                        #   원문을 대조해 푼다(코드 목록이 DB 에만 있게).
string robot_id         # "robot1". task_manager 가 자기 네임스페이스에서 얻는다.
                        #   전역 로거 하나가 두 로봇 이벤트를 받으므로 메시지에 있어야 한다
string run_id           # 미션 1회를 묶는 UUID. scan 이 성공한 순간 발행해
                        #   cycle 이 끝날 때까지 들고 간다. 던지고 잊는 토픽에서
                        #   같은 미션의 네 행을 잇는 키가 이것 말고 없다

# ── 어느 단계가 어떻게 됐나 ───────────────────────────────────────────
string stage            # "pick" | "nav" | "place" | "return"  ← BT 잎 상수 그대로(소문자)
uint8  attempt 1        # 같은 (run_id, stage) 의 몇 번째 시도인가.
                        #   웹 복구(/orchestrator/resume)로 재시도하면 2, 3 ...
                        #   DB 제약이 UNIQUE(run_id, stage, attempt) 라 이게 없으면
                        #   재시도 행이 막혀 기록을 잃는다 (docs/DB구성.md §4-7)
bool   success          # 그 '단계' 의 성패                     → succeeded
string status           # 그 시점 '물체' 의 상태                → status
                        #   IN_TRANSIT | COMPLETED | FAILED
                        #
                        #   ★ 로거가 추론하지 않고 여기 실어 보내는 이유
                        #     return 은 place 뒤라 실패해도 물체는 이미 배달됐다.
                        #     success=false 인데 status=COMPLETED 인 행이 나온다.
                        #     "마지막 단계인가" 를 로거가 추론하면 거기서 틀린다.
                        #
                        #     pick·nav 성공 → IN_TRANSIT
                        #     place 성공     → COMPLETED
                        #     pick·nav·place 실패 → FAILED
                        #     return         → 성패 무관 COMPLETED

# ── 왜 실패했나 ───────────────────────────────────────────────────────
string fail_reason      # 넷 중 하나. 성공이면 빈 문자열       → fail_reason (ENUM)
                        #   pick_error | nav_error | place_error | dock_error
                        #   단계가 사유를 정한다 — return 은 같은 NavigateTo 액션이라
                        #   nav_error 다. 그래서 액션 enum 에 없는 실패(타임아웃,
                        #   서버 없음)도 갈 곳이 있다.
string fail_detail      # 액션 result 의 상수명 그대로          → fail_detail
                        #   "NO_ATTACH(4)" · "PORT_OCCUPIED(3)" · "TIMEOUT(120s)" ·
                        #   "SERVER_UNAVAILABLE(...)" · "GOAL_REJECTED"
                        #   fail_reason 넷은 집계용이라 "PICK 에서 실패" 까지만 말한다.
                        #   흡착이 안 붙은 건지 IK 가 없는 건지는 이 칸에만 남는다.
string port             # 어디서 — "test_loader" | "pkg_loader" | "shelf_1_s3"
                        #   지금은 place 행만 채워진다. NAV 목표가 TEST_LOADER 상수
                        #   하나뿐이고 순찰 중 발견 방식이라 슬롯 번호를 지정받지 않는다.
