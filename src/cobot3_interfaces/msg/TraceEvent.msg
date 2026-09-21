# ══════════════════════════════════════════════════════════════════════
#  보내는 노드 : cobot3_orchestrator / task_manager   (Freeze.update() 한 곳)
#  받는 노드   : cobot3_orchestrator / event_logger   (전역 1개)
#  종류        : 토픽  /trace/event   (절대이름, Reliable, depth 50)
#  역할        : 미션 단계가 하나 끝날 때마다 결과를 한 건 낸다.
#                로거는 그대로 magazine_log / stack_log 에 append 한다 — 판단하지 않는다.
#  전달 내용   : run_id · robot_id · carrier_id · stage · 두 벌 시계 · success · status · 실패 사유
# ══════════════════════════════════════════════════════════════════════

# ★ docs/DB구성.md §9 가 요구한 4필드(robot_id · run_id · status · 벽시계)를 반영한 판이다.
#   이전 판은 stamp 하나 + carrier_id + stage + success + fail_reason 뿐이었고,
#   공통 정보를 로거가 자기 파라미터로 붙이는 구조였다. 그 구조는 AMR 2대에서 깨진다 —
#   로거가 전역 1개인데 robot_id 를 파라미터로 들고 있으면 두 로봇을 구분할 수 없다.
#
# ★ 이 메시지의 필드는 DB 컬럼과 1:1 이다. 로거는 옮겨 담기만 한다.
#   예외가 둘:
#     · kind_code  — 로거가 carrier_code.parse_code(carrier_id) 로 푼다(§8-3).
#                    파서는 carrier_code.py 하나뿐이어야 해서 메시지에 싣지 않는다.
#     · 표 선택    — 같은 파서의 family 가 magazine 이면 표 2, stack 이면 표 3.
#
# ★ 시계를 두 벌 싣는 이유는 §4-2. 사이클 타임은 시뮬 시각이 물리적으로 옳고
#   ("몇 시에 배달됐나" 는 벽시계), duration_sec 은 DB 의 생성열이라 여기 없다.
#
# ★ started_* 를 task_manager 가 찍는 이유는 §4-1. 로거의 now() 를 쓰면
#   큐 지연분과 스풀 재적재분이 전부 '그때' 로 찍힌다.

string run_id                   # 미션 1회. ScanLeaf 성공 순간 uuid4() 발행 → CycleDone 에서 폐기.
                                # UUID 전용 타입이 ROS 에 없어 문자열이다. 로거가 UUID 로 캐스팅한다.
string robot_id                 # "robot1". self.get_namespace().strip('/')

string carrier_id               # QR 원문 그대로 — "F1-MGZB-1". 가공하지 않는다(§4-1 ①).
                                # 미판독이면 빈 문자열이고, 그런 이벤트는 애초에 발행되지 않는다(§4-6).

string stage                    # ★ 소문자다: pick | nav | place | return
                                # DB 의 stage 컬럼과 UNIQUE(run_id, stage) 가 이 값을 그대로 쓴다.
                                # 대문자 상수(PICK/NAV/...)는 task_manager 내부 라벨이라
                                # 발행할 때 .lower() 한다. 섞이면 같은 단계가 두 행이 된다.
                                # TEXT 인 이유는 §2 — 트리에 가지가 늘어날 자리가 나 있다.

builtin_interfaces/Time started_sim   # 잎의 initialise() 시각 (use_sim_time)
builtin_interfaces/Time ended_sim     # SUCCESS·FAILURE 반환 시각 (use_sim_time)
builtin_interfaces/Time started_wall  # 같은 두 순간의 벽시계.
builtin_interfaces/Time ended_wall    # ended_wall 이 화면에 "몇 시에" 로 나가는 값이다.

bool   success                  # 그 '단계' 의 성패
string status                   # 그 시점 '물체' 의 상태 — IN_TRANSIT | COMPLETED | FAILED
                                # ★ 둘을 따로 두는 이유는 §4-3. return 은 succeeded=false 라도
                                #   status=COMPLETED 다(물체는 이미 배달됐다). 한 칸으로 합치면
                                #   둘 중 하나가 거짓말이 된다.
                                # ★ 이 판단은 task_manager 가 한다. 로거는 하지 않는다 —
                                #   로거가 "마지막 단계인가" 를 추론하면 return 행에서 틀린다.

string fail_reason              # 성공이면 빈 문자열. pick_error | nav_error | place_error | dock_error
                                # DB 의 ENUM 4개와 같은 값. 다섯 번째를 보내면 INSERT 가 거부된다.
string fail_detail              # "PORT_OCCUPIED(3)" — _reason_name() 이 만든 "{상수명}({코드})".
                                # 이게 없으면 "pick_error 12건" 을 봐도 고칠 게 뭔지 모른다(§4-1).

string port                     # 어디서 — "shelf_01" | "test_loader" | "dock_pad_1".
                                # 지금 task_manager 가 대부분 모른다(§4-5). 모르면 빈 문자열.
