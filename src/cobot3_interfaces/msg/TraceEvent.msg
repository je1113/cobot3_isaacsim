# ─────────────────────────────────────────────────────────────────────
#  TraceEvent — 추적 이벤트 1건
#  task_manager  →  event_logger      토픽 /trace/event
#
#  목적: 생산 트래킹의 원천. SQLite `event` 테이블 1행과 1:1 대응한다.
#        "MAG-A17 이 언제 어디서 무엇에 성공/실패했나" 가 여기서 나온다.
#
#  공장 · 라인 · 로트는 여기에 없다. carrier_id 로 `carrier` 테이블(지금은
#  carriers.yaml)을 조인하면 나온다. 같은 정보를 두 곳에 쓰지 않는다.
#
#  event 테이블은 append-only 다 (05 §2-6). 정정이 필요하면 UPDATE 가 아니라
#  정정 이벤트를 하나 더 낸다. 추적 데이터를 고치는 순간 증거 능력이 사라진다.
#
#  출처: docs/05_QR_Traceability.md §2-6 그대로.
# ─────────────────────────────────────────────────────────────────────

std_msgs/Header header

string  carrier_id              # "MAG-A17". 미판독(NO_READ)이면 빈 문자열
string  amr_id                  # 단일 AMR 이라 지금은 항상 같은 값. 필드는 유지.

string  event_type              # PICK_TRY | PICK_OK | TRANSIT | PLACE_TRY | PLACE_OK
                                # | NO_READ | ID_MISMATCH | TYPE_MISMATCH
string  zone                    # STOCKER | PACKAGING | TEST
string  port                    # 슬롯 번호 또는 포트 번호   예: "shelf_1_s3"
string  stage                   # 인식 | 검증 | 파지 | 주행 | 배치  — 실패 사유 분류축
string  fail_reason             # 실패 시에만. 액션 result 의 fail_reason 상수명을
                                # 문자열로 옮겨 적는다 (예: "NO_ATTACH", "OFF_FLANGE")

bool    success

float64 sim_time                # 시뮬 시각 (use_sim_time). 사이클 타임 계산은 이걸로.
builtin_interfaces/Time wall_time   # 벽시계. 시뮬을 몇 번 재시작해도 순서가 보존된다.
string  run_id                  # 반복 시행 회차 — 회차별 성공률 집계 키
