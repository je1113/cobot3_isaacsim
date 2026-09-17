# carrier_code_reader  →  task_manager        토픽 /perception/carrier_scan
#
# 기능: 라벨을 읽고 "누구(carrier_id)가 어디(qr_pose)에 있다" 를 알린다.
#       task_manager 는 이걸 받아 순회를 멈추고, carrier_id 로 carriers.yaml →
#       frames.yaml 을 조회해 PickCarrier.goal.flange_pose 를 계산한다.
#
# 발행 조건: 스키마 정규식 + mod-36 체크문자 + N-of-M 다수결 통과 시만.
#            스트림이 아니라 이벤트다 → Reliable, depth 5.
#
# 여기 없는 것(raw_payload · symbology · frame_votes · decode_latency_ms ·
# range_m · reader_id · payload_valid)은 튜닝·분석 지표라 노드 로그로 뺐다.
# 페이로드 형식 "C3.MAG.A17.9" 는 05 §2-1.
#
# ★ carrier_code_reader 가 declare_parameter 로 정의해야 하는 것
#   decode_hz       (double, 5.0)    30 Hz 전부 디코드하지 않는다      05 §2-5
#   roi_fraction    (double, 0.6)    중앙 크롭 — 옆 슬롯 라벨 오검출 차단  05 §2-5
#   vote_window     (int, 5)         N-of-M 의 M                        05 §2-5
#   vote_required   (int, 3)         N-of-M 의 N                        05 §2-5
#   output_frame    (string)         qr_pose 의 frame_id — map / base_link 미정
#   publish_debug   (bool, false)    크롭 이미지를 /perception/carrier_scan_debug 로
#
# ★ task_manager 가 정의해야 하는 것 (이 메시지를 쓰려면)
#   carriers_yaml   (string)         carrier_id → variant        cobot3_bringup/config
#   frames_yaml     (string)         variant → 라벨→플랜지 오프셋  cobot3_bringup/config

std_msgs/Header header          # stamp = 판독한 프레임 시각

uint8 MAGAZINE=0
uint8 TRAY_STACK=1

string carrier_id               # "MAG-A17" — 페이로드의 <kind>-<serial>
uint8  carrier_kind             # MAGAZINE | TRAY_STACK — 페이로드의 MAG/STK.
                                # carrier_id 앞글자로도 알 수 있지만, 받는 쪽이
                                # 페이로드 문법을 몰라도 되게 따로 둔다.
geometry_msgs/PoseStamped qr_pose
                                # 라벨 중심 자세. +Z = 라벨 법선(바깥), +X = 라벨 가로.
                                # frame_id 는 header 에 명시 — map / base_link 미정.
                                # reader 가 프레임 시각의 TF 로 변환해 낸다.
