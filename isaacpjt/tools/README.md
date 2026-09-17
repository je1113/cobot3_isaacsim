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

# 5. 자세 티칭 캡처 — 실제로 쓸 관절값은 여기서 뜬다
isaac_python isaacpjt/tools/capture_pose.py                 # shelf_1 앞
CAPTURE_BASE=shelf_2 isaac_python isaacpjt/tools/capture_pose.py
#    뷰포트 옆 "Teach Pose" 패널의 슬라이더로 자세를 만들고 Capture
#    터미널로도 된다:
#        echo "0 -30 90 0 60 0"  > /tmp/capture_jog
#        echo "shelf_1_top_scan" > /tmp/capture_pose
#    -> out/taught_poses.yaml

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
