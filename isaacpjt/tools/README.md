# isaacpjt/tools

씬(`worlds/simple_factory_layout.usda`)에서 숫자를 직접 뽑아내는 도구들.
손으로 잰 값이나 도면 값을 쓰지 않는 게 원칙이다.

결과의 최종 정착지는 `src/cobot3_bringup/config/frames.yaml` 한 곳이다.

## 실행 순서

```bash
# 0. 실측 — 선반/매거진/로봇/그리퍼 치수와 좌표를 전부 뽑는다
isaac_python isaacpjt/tools/measure_layout.py
#    -> out/layout_measured.yaml

# 1. 허용 오차 산출 — 흡착 컵과 플랜지 치수에서 역산 (Isaac 불필요)
python3 isaacpjt/tools/grasp_tolerance.py

# 2. 좌표계 검증 — frames.yaml 이 씬과 몇 mm 차이인지. 완료 기준 5 mm
isaac_python isaacpjt/tools/verify_frames.py

# 3. 손목 카메라 ROS2 발행 — image + camera_info
ros_set && isaac_ros
isaac_python isaacpjt/tools/wrist_camera_ros.py
#    다른 터미널: ros2 topic echo /wrist_camera/color/camera_info --once

# 4. 스캔 거리/도달성 계산 (관절값은 참고용일 뿐)
isaac_python isaacpjt/tools/find_scan_poses.py
#    -> out/scan_poses.yaml

# 4b. QR 이 실제로 몇 미터까지 디코드되는지 렌더해서 확인
QR_POSE=shelf_1_top_scan isaac_python isaacpjt/tools/qr_decode_range.py
#    -> out/qr_decode_range.yaml, out/qr_frames/*.png
#    실측: 0.456 m(라벨 70 px, 모듈당 2.40) 까지 디코드,
#          0.556 m(57 px, 1.97) 부터 실패. 검출은 0.656 m 까지.

# 5. 자세 티칭 캡처 — 실제로 쓸 관절값은 여기서 뜬다
#    한 선반당 정차 2회(a, b). 각 정차에서 8 개 중 6 개가 사거리에 들고
#    두 정차의 합집합이 8 개 전부를 덮는다. 시작할 때 표적 목록을 찍어 준다.
CAPTURE_BASE=shelf_1_a isaac_python isaacpjt/tools/capture_pose.py   # 기본값
CAPTURE_BASE=shelf_1_b isaac_python isaacpjt/tools/capture_pose.py
CAPTURE_BASE=shelf_2_a isaac_python isaacpjt/tools/capture_pose.py
CAPTURE_BASE=shelf_2_b isaac_python isaacpjt/tools/capture_pose.py
#    뷰포트 옆 "Teach Pose" 패널의 슬라이더로 자세를 만들고 Capture
#    터미널로도 된다:
#        echo "0 -30 90 0 60 0"  > /tmp/capture_jog
#        echo "shelf_1_top_scan" > /tmp/capture_pose
#    -> out/taught_poses.yaml

# 6. QR 자세추정 정확도 — GT 는 매 표본 리지드바디 pose 로 만든다
isaac_python isaacpjt/tools/eval_qr_pose.py
#    -> out/qr_pose_eval.yaml

# 6b. 오차가 이상하면 꼭짓점 재투영으로 GT / 카메라 / K 중 뭐가 틀렸는지 가른다
isaac_python isaacpjt/tools/diag_qr_reproj.py
#    -> out/qr_reproj_diag.yaml, out/qr_reproj_frames/*.png

# 7. 비전 파지 — QR 대략값 -> 플랜지 상방 관측 -> 파지 (grasp.yaml flange_vision)
PICK_VISION=1 PICK_TRIALS=10 PICK_HEADLESS=1 \
    isaac_python isaacpjt/M0609/lula_ik/12_pick_test.py
#    매거진을 ±10 mm / ±5도 밀어 두고, prior 에 ≤5 mm / ±7도 오차를 넣는다.
#    PICK_DEPTH_NOISE_MM=2 로 깊이 노이즈를 얹어 볼 수 있다.
#    프레임: isaacpjt/M0609/lula_ik/out/vision_frames/*.png

# Isaac Sim 메뉴(Tools > Robotics > ...)는 찾아도 없다. isaac_python 이 띄우는
# isaacsim.exp.base.python.kit 에는 메뉴 확장이 아예 안 올라온다.
# 그래서 조작 UI 를 capture_pose.py 안에 직접 넣었다.
```

## 왜 4 와 5 가 따로인가

`find_scan_poses.py` 의 Lula IK 는 자기 충돌만 보고 선반/매거진과의 충돌은
안 본다. 해가 여러 개인 자세에서 팔꿈치가 뒤집힌 해를 고르기도 한다.
그래서 4 는 "얼마나 떨어져야 QR 이 몇 픽셀로 보이는가 / 애초에 닿기는 하는가"
까지만 쓰고, 실제 관절값은 5 로 티칭해서 뜬다.

## ROS 쪽

```bash
colcon build --packages-select cobot3_bringup
source install/setup.bash

ros2 launch cobot3_bringup tf.launch.py use_fake_joints:=true
ros2 run tf2_tools view_frames
```

## QR 은 ID 와 대략 위치만, 파지 목표는 플랜지를 위에서 찍어 정한다

`eval_qr_pose.py` 로 QR 자세추정을 재 보면 위치는 평균 1.6 mm 로 쓸 만한데
yaw 는 1σ 2.3도, 최대 4.6도다. 파지 허용치(1.05도)의 네 배다.

라벨이 옆벽에 서 있어서 매거진 yaw 가 카메라에서는 '화면 밖으로 기우는'
회전이 되기 때문이다. 36 mm 코드를 273 mm 에서 보면 1도에 좌우 변 길이 차가
0.2 px 인데, `diag_qr_reproj.py` 로 잰 cv2 꼭짓점 잔차는 RMS 2.1 px
(오른쪽 아래 꼭짓점은 finder pattern 이 없어 3.5 px) 다. 필터로는 못 줄인다.

그래서 근거리에서 플랜지를 위에서 찍는다
(`src/cobot3_perception/cobot3_perception/flange_topview.py`).
yaw 가 화면 안의 회전이 되고, 컵이 닿을 면을 직접 재므로 QR -> 플랜지
레버암(114/153 mm)을 타고 오차가 커지지도 않는다.

### 함정: 물리 전 저작값을 GT 로 쓰지 마라

매거진이 상판 위 4.6 mm 떠 있게 저작돼 있다. `layout_measured.yaml` 의 좌표는
물리 전 값이라, 그걸 정답으로 쓰면 QR 이 4.58 mm 아래에 있는 걸 오차로 센다.
이전 `eval_qr_pose.py` 의 "횡오차 4.4 mm" 는 전부 이것이었다
(이미지 v 로 10.2 px, GT 를 고치자 0.55 mm). 카메라 pose · TF · K 는 맞았다.
