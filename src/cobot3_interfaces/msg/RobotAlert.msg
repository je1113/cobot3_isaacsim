# ══════════════════════════════════════════════════════════════════════
#  보내는 노드 : cobot3_orchestrator / task_manager
#  받는 노드   : web backend (cobot3_web) — WebSocket 의 alert 채널로 흘린다
#  종류        : 토픽  /trace/alert   (절대이름, Reliable, depth 20)
#  역할        : **캐리어가 없는 사건.** QR 을 못 읽어서 payload 가 없거나,
#                애초에 캐리어와 무관한 일(일시정지 · 전체정지 · 서비스 끊김).
#  전달 내용   : robot_id · severity · code · detail · stage · 벽시계
# ══════════════════════════════════════════════════════════════════════

# ★ 이 메시지가 왜 필요한가 — docs/DB구성.md §4-6 의 구멍이다.
#
#   표 2·3 의 행은 키가 QR 원문(qr_payload)이다. 그런데 SCAN 의 hard 실패 3종은
#   원문이 없거나(TIMEOUT · SERVICE_UNAVAILABLE) 종류를 못 풀어서(UNKNOWN_PAYLOAD)
#   매거진·스택 중 어느 표에 넣을지조차 알 수 없다. 그래서 **로봇이 얼어붙는데
#   DB 에는 흔적이 없다.** 일시정지 · 전체정지도 같은 이유로 갈 곳이 없다.
#
#   §11 #1 이 그 해법으로 robot_log 표를 예고해 뒀지만 아직 미정이다.
#   그때까지 이 토픽이 **유일한 경로**다 — 웹이 받아서 화면에 띄우고, 사라진다.
#
# ★ 그래서 이 메시지는 '기록' 이 아니라 '알림' 이다.
#   받는 쪽이 없으면 그냥 없어진다(event_logger 는 구독하지 않는다).
#   robot_log 가 생기면 로거도 이걸 구독해서 append 하면 되고, 메시지는 안 바뀐다.
#   필드를 DB 컬럼처럼 생긴 모양으로 둔 이유가 그것이다.
#
# ★ TraceEvent 와 합치지 않은 이유:
#   TraceEvent 는 run_id 와 carrier_id 가 NOT NULL 인 세계다. 여기에 빈 값을
#   허용하기 시작하면 "판독 실패" 와 "아직 판독 전" 이 구분되지 않는다.
#   §8-2 가 표를 둘로 나눈 것과 같은 판단이다 — NULL 이 '아직' 인지 '영영' 인지
#   구분되지 않는 칸을 만들지 않는다.

uint8 INFO=0                    # 알아두면 좋은 것. 화면에 조용히 쌓인다.
uint8 WARN=1                    # soft 실패 — 로봇은 계속 돈다. 예: NOT_FOUND 재시도
uint8 ERROR=2                   # hard 실패 — 로봇이 그 자리에서 얼었다. 사람이 필요하다.

string robot_id                 # "robot1"
uint8  severity

string code                     # 기계가 분기하는 값. 상수처럼 대문자·고정 어휘로 쓴다.
                                #   SCAN_TIMEOUT · SCAN_SERVICE_UNAVAILABLE · UNKNOWN_PAYLOAD
                                #   PAUSED · RESUMED · STOPPED · SKIPPED
                                #   TASK_REJECTED · CONFIG_RELOAD_FAILED
                                # ★ ENUM 이 아니라 문자열인 이유는 TraceEvent.stage 와 같다 —
                                #   가지가 늘어날 자리가 나 있고(§2), 여기는 DB 제약도 없다.

string detail                   # 사람이 읽는 한 줄. "40s 안에 CarrierScan 응답 없음"
                                # 또는 "{상수명}({코드})" 형태를 그대로 올려도 된다.
string stage                    # 그때 어느 단계였나 — scan | pick | nav | place | return.
                                # 정지 명령처럼 단계와 무관하면 빈 문자열.

builtin_interfaces/Time stamp   # 벽시계. 화면에 시각으로 찍히는 값이라 시뮬 시각이 아니다.
                                # (TraceEvent 가 두 벌을 싣는 것과 다르다 — 여기서
                                #  '소요 시간' 을 재는 일이 없어 시뮬 시각이 쓰이지 않는다.)
