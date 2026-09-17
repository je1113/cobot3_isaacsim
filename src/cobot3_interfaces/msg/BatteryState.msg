# ─────────────────────────────────────────────────────────────────────
#  BatteryState — 배터리 상태 방송
#  docking_server  →  task_manager      토픽 /battery_state
#
#  목적: "충전하러 가야 하나" 를 task_manager 가 판단할 근거.
#        판단은 docking_server 가 하지 않는다. 언제 끊을지는 미션 상태
#        (PICK 중인가, RETURN 중인가)를 아는 쪽만 알기 때문이다. (07 §5)
#
#  시뮬에는 배터리가 없다. docking_server 가 마지막 충전 이후 경과 시간을
#  세고 그걸 SoC 로 환산한다. 실기로 옮길 때는 이 노드만 바뀌고 받는 쪽은
#  그대로다.
#
#  QoS: Reliable + TRANSIENT_LOCAL, 1 Hz.
#       task_manager 가 늦게 떠도 마지막 값을 즉시 받는다.
# ─────────────────────────────────────────────────────────────────────

std_msgs/Header header

float32 soc                     # 0.0 ~ 1.0. 지금은 elapsed_s 의 선형 환산
float32 elapsed_s               # 마지막 충전 완료 이후 경과 시간

float32 soft_threshold_s        # 넘으면 needs_charge. 다음 RETURN 에서 이탈한다.
                                # "임계값 + 최악의 남은 사이클 < 방전" 이 되게 잡는다.
float32 hard_threshold_s        # 넘으면 critical. 그 자리에서 중단하고 캐리어를
                                # 가장 가까운 안전 지점에 내려놓은 뒤 도킹.
                                # 0 이면 미사용 (지금은 soft 하나로 시작).
bool    needs_charge            # elapsed_s > soft_threshold_s
bool    critical                # elapsed_s > hard_threshold_s

bool    charging                # 접점이 붙어 충전 중. true 인 동안 elapsed_s 정지·리셋.
