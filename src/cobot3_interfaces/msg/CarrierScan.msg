# cobot3_perception (carrier_code_reader)  →  cobot3_orchestrator (task_manager)
#                                             토픽 /perception/carrier_scan
#
# 기능: 카메라 토픽을 받아 QR 을 읽고, "무엇이(payload) 어디에(qr_pose)" 를 알린다.
#       perception 의 일은 여기까지다 — 이 ID 가 어느 변형인지, 어디로 가야 하는지는
#       모른다. payload 해석도 orchestrator 가 한다.
#
# orchestrator 는 이걸 받으면 명령 큐에 한 건 넣는다. patrol 상태인 로봇이 있으면
# 큐에서 빼서 qr_pose 에 상대좌표를 적용해 NavigateTo 목표를 만든다.
#
# 발행 조건: 스키마 정규식 + mod-36 체크문자 + N-of-M 다수결을 통과한 판독만.
#            스트림이 아니라 이벤트다 → Reliable, depth 5.
#
# ★ carrier_code_reader 가 declare_parameter 로 정의해야 하는 것
#   decode_hz       (double, 5.0)    30 Hz 전부 디코드하지 않는다        05 §2-5
#   roi_fraction    (double, 0.6)    중앙 크롭 — 옆 슬롯 라벨 오검출 차단  05 §2-5
#   vote_window     (int, 5)         N-of-M 의 M                        05 §2-5
#   vote_required   (int, 3)         N-of-M 의 N                        05 §2-5
#   output_frame    (string, "map")  qr_pose 를 어느 좌표계로 낼지
#   publish_debug   (bool, false)    크롭 이미지 디버그 토픽

std_msgs/Header header          # stamp   = 판독한 프레임 시각
                                # frame_id = qr_pose 의 좌표계 (output_frame).
                                #            이 메시지에서 좌표계를 말하는 곳은 여기 하나다.

string payload                  # QR 이 담고 있는 정보 원문. 예: "C3.MAG.A17.9"
                                # 파싱(<kind>-<serial> 분해, 변형 조회)은 orchestrator 가 한다.

geometry_msgs/Pose qr_pose      # 라벨 중심의 위치 + 방향.
                                # 방향이 필요한 이유: orchestrator 가 "라벨 정면 0.5 m" 같은
                                # 상대좌표를 적용해 로봇이 설 자리와 보는 방향까지 정한다.
                                # +Z = 라벨 법선(바깥), +X = 라벨 이미지의 오른쪽.
                                #   (grasp.yaml qr_frame_convention 과 같은 규약)
                                # 이 값은 파지 목표가 아니라 대략값(prior)이다 — 실제 파지점은
                                # manipulation 이 플랜지를 위에서 재서 정한다. 이유는
                                # grasp.yaml flange_vision.why (라벨이 옆벽이라 yaw 1σ 2.3도).
