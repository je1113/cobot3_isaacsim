#!/usr/bin/env bash
# sim_backend.py 를 시스템 ROS 와 격리해서 띄운다.
#
# ★ 왜 필요한가
#   시스템 ROS 2 Jazzy 는 파이썬 3.12 용이고 Isaac 의 kit 파이썬은 3.11 이다.
#   ROS 를 source 한 셸에서 python.sh 를 부르면 그 셸의 PYTHONPATH(3.12
#   site-packages)와 LD_LIBRARY_PATH(/opt/ros/jazzy/lib 이 Isaac 브릿지 lib
#   보다 앞)를 그대로 물려받아, Isaac 이 플러그인을 로드하다 심볼 충돌로
#   죽는다. 증상은 SimulationApp(...) 생성자에서 Segmentation fault 이고
#   우리 코드는 한 줄도 실행되지 않는다(실측 2026-09-22).
#
#   isaac-sim.sh 로 띄우면 멀쩡한 이유가 이것이다 — 그쪽은 자체 환경을 쓴다.
#
#   sim_backend.py 자체는 rclpy 를 import 하지 않는다(순수 stdlib JSON-RPC).
#   그런데도 ROS 환경이 문제가 되는 건 sim_backend.py 가
#   enable_extension("isaacsim.ros2.bridge") 를 켜기 때문이다 — 씬의 OmniGraph
#   가 Nav2 용 /clock·odom·lidar 를 내보내려면 그 확장이 필요하고, 그 확장은
#   /opt/ros/jazzy/lib(3.12 용)이 아니라 Isaac 이 번들한 3.11 용 브릿지 lib 을
#   찾아야 한다. 그래서 "ROS 환경을 빼는" 게 아니라 "브릿지 환경으로 바꿔치기"
#   한다. ros_set 된 터미널에서 그대로 쳐도 된다 — 아래에서 직접 걷어낸다.
#
#   같은 격리를 M0609/lula_ik/run_color_sort.sh 가 이미 하고 있다.
#   이 파일은 그것을 sim_backend 용으로 옮긴 것이다.
#
# 사용법 (워크스페이스 루트에서)
#   ./isaacpjt/ros_bridge/run_sim_backend.sh                 # GUI
#   SIM_HEADLESS=1 ./isaacpjt/ros_bridge/run_sim_backend.sh  # 창 없이
#     ☞ 헤드리스는 world.step(render=False) 라 라이다/카메라가 안 나온다.
#       시험에는 GUI 를 써라(sim_backend.py 메인 루프 참고).
#   ./isaacpjt/ros_bridge/run_sim_backend.sh --check         # Isaac 안 띄우고 환경만 검사
#   SIM_WORLD_USD=isaacpjt/worlds/simple_factory_layout_test.usda \
#       ./isaacpjt/ros_bridge/run_sim_backend.sh             # 씬 지정(static_test 시나리오)
#   ISAAC_SCRIPT=isaacpjt/tools/wrist_camera_ros.py \
#       ./isaacpjt/ros_bridge/run_sim_backend.sh             # 같은 격리가 필요한 다른 스크립트
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ws_root="$(cd -- "$script_dir/../.." && pwd)"
isaac_root="${ISAAC_SIM_ROOT:-$HOME/isaacsim}"
bridge_root="$isaac_root/exts/isaacsim.ros2.bridge/jazzy"

# python.sh 만 봐서는 부족하다 — 브릿지 디렉터리가 없으면 아래 PYTHONPATH/
# LD_LIBRARY_PATH 가 허공을 가리키는데, 그 상태로도 Isaac 은 일단 뜬다(ros2
# bridge 확장만 조용히 실패해서 /clock 이 안 나오는 걸로만 보인다). 여기서
# 미리 막는다. run_color_sort.sh 도 같은 검사를 한다.
if [[ ! -x "$isaac_root/python.sh" || ! -d "$bridge_root/rclpy" || ! -d "$bridge_root/lib" ]]; then
    echo "Isaac Sim Jazzy 브릿지를 못 찾았다: $bridge_root — ISAAC_SIM_ROOT 로 알려줘라." >&2
    exit 1
fi

# ── 시스템 ROS 흔적을 걷어낸다 ────────────────────────────────────────
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH
unset PYTHONHOME VIRTUAL_ENV CONDA_PREFIX CONDA_DEFAULT_ENV
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export PYTHONPATH="$bridge_root/rclpy"
export PYTHONNOUSERSITE=1
export LD_LIBRARY_PATH="$bridge_root/lib"

# ── ROS 설정 — 이 PC 의 ROS 노드들과 같은 도메인이어야 통신된다 ───────
export ROS_DISTRO=jazzy
export ROS_VERSION=2
export ROS_PYTHON_VERSION=3
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
# ★ 기본 102. 처음 커밋(d933cda)은 103 으로 두고 "이 팀 설정" 이라 적었는데
#   근거가 없었다 — 저장소의 다른 모든 곳이 102 다: run_color_sort.sh,
#   7_pick_place_color.py(setdefault 102), 그리고 log/build_*/events.log 에
#   찍힌 실제 셸 환경(ROS_DOMAIN_ID='102'). 도메인이 어긋나면 Isaac 은 멀쩡히
#   뜨는데 Nav2/AMCL 이 /clock·odom·lidar 를 못 받아 "떴는데 아무것도 안
#   움직이는" 증상만 남는다. ros_set 셸이 export 한 값이 있으면 그걸 따른다.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-102}"

# ── 씬 ────────────────────────────────────────────────────────────────
# ★ 여기서 기본 씬을 정하지 않는다. 처음 커밋은 simple_factory_layout_test.usda
#   를 기본으로 박아 뒀는데, sim_backend.py 의 WORLD_USD 기본은 simple_factory_
#   layout.usda 라 런처를 거치면 씬이 몰래 바뀌었다(상태 스냅샷 경로도 씬 이름
#   을 따라가서 같이 갈린다 — sim_backend.py STATE_SNAPSHOT_PATH). 기본값의
#   출처는 sim_backend.py 하나여야 한다. 바꾸고 싶으면 SIM_WORLD_USD 를 넘겨라.
if [[ -n "${SIM_WORLD_USD:-}" ]]; then
    # 상대 경로는 워크스페이스 루트 기준으로 받는다(아래 cd 와 맞춘다)
    [[ "$SIM_WORLD_USD" = /* ]] || SIM_WORLD_USD="$ws_root/$SIM_WORLD_USD"
    export SIM_WORLD_USD
    if [[ ! -f "$SIM_WORLD_USD" ]]; then
        echo "씬 파일이 없다: $SIM_WORLD_USD" >&2
        exit 1
    fi
    scene_label="$SIM_WORLD_USD"
else
    scene_label="(미지정 → sim_backend.py 의 기본 씬)"
fi
export SIM_HEADLESS="${SIM_HEADLESS:-0}"
# GUI 렌더를 N 스텝마다 한 번만(물리는 매 스텝). 1 = 매 스텝 렌더.
export SIM_RENDER_EVERY="${SIM_RENDER_EVERY:-2}"

# ── 띄울 스크립트 ─────────────────────────────────────────────────────
# 기본은 sim_backend.py. isaacpjt/tools/wrist_camera_ros.py 처럼 ROS2 브릿지를
# 켜는 다른 Isaac 스크립트도 똑같은 격리가 필요하다 — ISAAC_SCRIPT 로 바꿔 띄운다.
target="${ISAAC_SCRIPT:-isaacpjt/ros_bridge/sim_backend.py}"

cd "$ws_root"

# ── --check: Isaac 을 띄우지 않고 환경만 본다 ─────────────────────────
# 크래시가 SimulationApp() 안에서 나서 파이썬 Traceback 이 안 남는다. 그래서
# 그 전에 "격리가 제대로 걸렸나" 만 따로 본다:
#   · 인터프리터가 kit 의 3.11 인가, sys.path 에 3.12 site-packages 가 남아 있나
#   · rclpy 가 Isaac 브릿지 번들에서 잡히나(시스템 ROS 의 3.12 것이 아니라)
#   · 그 rclpy 의 C 확장과 타입서포트 .so 가 실제로 로드되나 — LD_LIBRARY_PATH
#     가 브릿지 lib 을 가리켜야 통과한다. /opt/ros/jazzy/lib 이 앞서면 여기서
#     ImportError 로 잡힌다(세그폴트 대신 읽을 수 있는 에러로).
#   · sim_backend.py 가 import 하는 numpy/yaml/cv2 가 Isaac 번들에 있나
# 이게 통과해도 Isaac 자체 문제(GPU/드라이버)까지 보장하진 않는다.
if [[ "${1:-}" == "--check" ]]; then
    check_py='
import sys
print("python  :", sys.version.split()[0], "(", sys.executable, ")")
bad = [p for p in sys.path if "python3.12" in p or "/opt/ros/" in p]
if bad:
    sys.exit("FAIL: sys.path 에 시스템 ROS(3.12) 경로가 남아 있다: " + ", ".join(bad))
import rclpy
print("rclpy   :", rclpy.__file__)
if "isaacsim.ros2.bridge" not in rclpy.__file__:
    sys.exit("FAIL: rclpy 가 Isaac 브릿지 번들이 아닌 곳에서 잡혔다")
import rclpy.impl.implementation_singleton          # C 확장(_rclpy_pybind11) → librcl 등 로드
from rosidl_generator_py import import_type_support
import_type_support("std_msgs")                    # 타입서포트 .so 로드
print("bridge  : rclpy C 확장 + std_msgs 타입서포트 로드 OK")
import numpy, yaml, cv2
print("bundle  : numpy", numpy.__version__, "/ yaml", yaml.__version__, "/ cv2", cv2.__version__)
print("OK — 이 환경이면 SimulationApp() 까지는 ROS 충돌 없이 간다")
'
    exec "$isaac_root/python.sh" -c "$check_py"
fi

echo "  스크립트   $target"
echo "  씬        $scene_label"
echo "  헤드리스   $SIM_HEADLESS"
echo "  렌더 주기  ${SIM_RENDER_EVERY} 스텝마다"
echo "  도메인     $ROS_DOMAIN_ID"
echo

exec "$isaac_root/python.sh" "$target" "$@"
