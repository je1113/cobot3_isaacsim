# task_manager  →  event_logger        토픽 /trace/event
#
# 기능: 미션 단계 하나가 끝날 때마다 한 건. event_logger 는 받은 그대로
#       SQLite event 테이블에 1행 append 한다 — 판단하지 않는다.
#
# 공통 정보(run_id · amr_id · 벽시계)는 메시지에 없다. 로거가 INSERT 할 때
# 자기 파라미터와 datetime('now') 로 붙인다. 시뮬 시각은 header.stamp 다.
# 공장·라인·로트는 carrier_id 로 carrier 테이블(지금은 carriers.yaml) 조인.
#
# 05 §2-6 의 event_type(PICK_TRY / PICK_OK / ...)은 stage + success 로 풀었다.
# "단계별 실패율" 이 GROUP BY stage WHERE success=0 으로 바로 나온다.
#
# ★ event_logger 가 declare_parameter 로 정의해야 하는 것
#   db_path   (string)            SQLite 파일 경로
#   run_id    (string)            반복 시행 회차. 런치마다 지정, 모든 행에 붙인다
#   amr_id    (string, "amr_1")   단일 AMR
#   → 메시지에 없는 이 셋과 벽시계(datetime('now'))는 INSERT 때 로거가 붙인다.

std_msgs/Header header          # stamp = 시뮬 시각 (use_sim_time)

string carrier_id               # "MAG-A17". 미판독이면 빈 문자열
string stage                    # SCAN | PICK | TRANSIT | PLACE | RETURN | DOCK
bool   success
string fail_reason              # 실패 시만. 액션 result 의 상수명 그대로 ("NO_ATTACH")
string port                     # 어디서 — "shelf_1_s3" | "pkg_loader" | "test_loader" | "dock_pad_1"
