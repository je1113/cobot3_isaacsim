#!/usr/bin/env bash
# PC A: isolate Isaac Sim's Python 3.11 Jazzy from sourced system ROS.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
isaac_root="${ISAAC_SIM_ROOT:-$HOME/isaacsim}"
bridge_root="$isaac_root/exts/isaacsim.ros2.bridge/jazzy"
if [[ ! -x "$isaac_root/python.sh" || ! -d "$bridge_root/rclpy" ]]; then
    echo "Isaac Sim Jazzy libraries not found: $bridge_root; set ISAAC_SIM_ROOT." >&2
    exit 1
fi
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH
unset PYTHONHOME VIRTUAL_ENV CONDA_PREFIX CONDA_DEFAULT_ENV
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export PYTHONPATH="$bridge_root/rclpy"
export PYTHONNOUSERSITE=1
export LD_LIBRARY_PATH="$bridge_root/lib"
export ROS_DISTRO=jazzy
export ROS_VERSION=2
export ROS_PYTHON_VERSION=3
export ROS_DOMAIN_ID=102
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
if [[ "${1:-}" == "--check" ]]; then
    exec "$isaac_root/python.sh" -c 'import sys, rclpy; from sensor_msgs.msg import Image; from std_msgs.msg import Int32; from rosidl_generator_py import import_type_support; import_type_support("sensor_msgs"); import_type_support("std_msgs"); print("Python:", sys.version); print("rclpy:", rclpy.__file__); print("ROS message type support: OK")'
fi
exec "$isaac_root/python.sh" "$script_dir/7_pick_place_color.py" "$@"
