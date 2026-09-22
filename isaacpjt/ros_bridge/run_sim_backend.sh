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
#   같은 격리를 M0609/lula_ik/run_color_sort.sh 가 이미 하고 있다.
#   이 파일은 그것을 sim_backend 용으로 옮긴 것이다.
#
# 사용법
#   ./isaacpjt/ros_bridge/run_sim_backend.sh                 # GUI
#   SIM_HEADLESS=1 ./isaacpjt/ros_bridge/run_sim_backend.sh  # 창 없이
#     ☞ 헤드리스는 world.step(render=False) 라 라이다/카메라가 안 나온다.
#       시험에는 GUI 를 써라(sim_backend.py 메인 루프 참고).
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ws_root="$(cd -- "$script_dir/../.." && pwd)"
isaac_root="${ISAAC_SIM_ROOT:-$HOME/isaacsim}"
bridge_root="$isaac_root/exts/isaacsim.ros2.bridge/jazzy"

if [[ ! -x "$isaac_root/python.sh" ]]; then
    echo "Isaac Sim 을 못 찾았다: $isaac_root — ISAAC_SIM_ROOT 로 알려줘라." >&2
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
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-103}"

# ── 씬 ────────────────────────────────────────────────────────────────
export SIM_WORLD_USD="${SIM_WORLD_USD:-$ws_root/isaacpjt/worlds/simple_factory_layout_test.usda}"
export SIM_HEADLESS="${SIM_HEADLESS:-0}"

if [[ ! -f "$SIM_WORLD_USD" ]]; then
    echo "씬 파일이 없다: $SIM_WORLD_USD" >&2
    exit 1
fi

echo "  씬        $SIM_WORLD_USD"
echo "  헤드리스   $SIM_HEADLESS"
echo "  도메인     $ROS_DOMAIN_ID"
echo

cd "$ws_root"
exec "$isaac_root/python.sh" isaacpjt/ros_bridge/sim_backend.py "$@"
