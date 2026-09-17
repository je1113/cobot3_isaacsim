# ─────────────────────────────────────────────────────────────────────
#  CarrierScan — QR 판독 결과 1건
#  carrier_code_reader  →  task_manager      토픽 /perception/carrier_scan
#
#  목적: "앞에 어떤 캐리어가 있다" 를 orchestrator 에 알린다.
#        task_manager 는 이걸 받으면 순회 주행을 취소하고 SCAN 상태로 간다.
#
#  QR 에는 개체 ID 만 들어 있다 (05 §2-1: "C3.MAG.A17.9").
#  변형(오렌지/블루) · 공장 · 라인 · 로트는 여기에 없다 — carrier_id 로
#  carriers.yaml(나중엔 DB) 을 조회해서 얻는다. 그래야 QR 을 다시 안 찍고도
#  추적 정보를 늘릴 수 있고, 페이로드가 짧아 판독 거리가 유리하다.
#
#  발행 시점: 매 프레임이 아니다. 스키마 정규식 + mod-36 체크문자 검증을
#  통과하고 N-of-M 다수결까지 넘긴 판독만 낸다. 스트림이 아니라 이벤트다.
#  → QoS Reliable, depth 5.  (BestEffort 면 그 한 건이 증발한다)
#
#  출처: docs/05_QR_Traceability.md §2-6.  carrier_type(string) 만
#        carrier_kind(uint8) 로 바꿨다 — CarrierPose · PlaceCarrier 와 같은
#        타입으로 비교하기 위해. 05 레퍼런스 코드의 out.carrier_type 두 줄만
#        고치면 된다.
# ─────────────────────────────────────────────────────────────────────

std_msgs/Header header          # stamp = 판독한 프레임 시각, frame_id = 카메라

# carrier_kind
uint8 MAGAZINE=0
uint8 TRAY_STACK=1

string  raw_payload             # 디코드 원문          예: "C3.MAG.A17.9"
string  symbology               # 심볼 종류            예: "QRCode"

string  carrier_id              # 개체 고유 ID         예: "MAG-A17"
                                # 페이로드의 <kind>-<serial>. 이 값으로 변형과
                                # 생산 정보를 조회한다.
uint8   carrier_kind            # MAGAZINE | TRAY_STACK — 페이로드의 MAG/STK 에서
                                # 목적지 분기용 (매거진→패키징 로더, 스택→테스트 로더)

string  reader_id               # 어느 카메라가 읽었나   예: "wrist_camera"

bool    payload_valid           # 스키마 정규식 + mod-36 체크문자 통과 여부
                                # false 면 발행하지 않으므로 사실상 항상 true.
                                # 필드를 남기는 이유: 디버그 모드에서 실패 건도
                                # 흘려 보고 싶을 때를 위해.
float32 decode_latency_ms       # 디코드에 걸린 시간 — 전처리 단계 튜닝용
uint16  frame_votes             # N-of-M 다수결에서 이 ID 가 얻은 표 수
float32 range_m                 # 심볼 화면 크기로 역산한 거리 (Z = fx·W/px)
                                # 거리별 판독률 곡선을 뽑기 위한 값 (05 §2-12)
