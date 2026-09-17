# ─────────────────────────────────────────────────────────────────────
#  CarrierPose — 캐리어 자세 + 실측 치수
#  pose_resolver  →  task_manager      토픽 /carrier_pose
#
#  목적: "어디에 흡착해야 하는가" 와 "실제로 얼마만 한가" 를 준다.
#
#  ★ 이 메시지는 실측값만 담는다. 카탈로그(frames.yaml)에서 오는 기대값은
#    task_manager 가 PickCarrier.goal 을 만들 때 따로 붙인다.
#    그래야 "실측 vs 기대" 가 한 goal 안에 나란히 놓여 대조와 로그가 된다.
#
#  그리퍼가 흡착(Ø50 컵)이므로 "개폐 폭" 같은 개념은 없다.
#  흡착 목표는 플랜지 판 윗면 중심 한 점이고, 그게 flange_pose 다.
#
#  출처: docs/06_System_Architecture.html §6.  flange_width_m("그리퍼 개폐
#        목표")은 흡착과 무관하므로 뺐다.
# ─────────────────────────────────────────────────────────────────────

std_msgs/Header header          # frame_id = base_link (팔 베이스 기준으로 변환 완료)

# carrier_kind
uint8 MAGAZINE=0
uint8 TRAY_STACK=1

string  carrier_id              # 이 측정을 촉발한 CarrierScan 의 ID 를 그대로 복사.
                                # task_manager 가 scan 과 pose 를 짝지을 때 쓴다.
uint8   carrier_kind            # MAGAZINE | TRAY_STACK

geometry_msgs/Pose flange_pose  # ★ 흡착 목표점 — 플랜지 판 윗면 중심.
                                #   position 은 컵이 닿을 자리, orientation 은
                                #   판의 법선(+Z 가 위). 흡착면을 이 Z 에
                                #   grip_gaps_m 만큼 띄워 내린다.

float32 body_width_m            # 실측 몸체 폭   — 매거진 변형 구분축 (250 vs 300)
float32 stack_height_m          # 실측 전체 높이 — 스택 변형 구분축 (77.7 vs 97.0)
                                # 두 값은 카탈로그 기대값과 대조해 품종 혼입을 잡는다.
float32 tilt_rad                # 판이 수평에서 기운 각. 크면 컵이 한쪽만 닿는다.
float32 confidence              # 0~1. 낮으면 task_manager 가 재관측을 시킨다.
