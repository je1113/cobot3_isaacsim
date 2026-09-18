"""
frames.yaml 에 적힌 정적 변환을 그대로 TF 로 발행한다.

    ros2 launch cobot3_bringup tf_static.launch.py

값을 코드에 박지 않는다. config/frames.yaml 이 단일 출처이고, 그 값은
isaacpjt/tools/measure_layout.py 가 USD 에서 뽑은 것이다.

회전은 quat(xyzw) 를 쓴다. tool0->camera_link 는 pitch 가 -90도 근처라
rpy 로 넘기면 짐벌락 근처에서 수치가 흔들린다.

인자:
    robot_ns:=<네임스페이스>   프레임 이름 앞에 붙일 접두사 (기본 없음)
    only:=a,b,c               발행할 변환 키만 골라서 (기본 전부)
"""

import re
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


CONFIG_NAME = "frames.yaml"


def _node_name(key):
    """frames.yaml 키 -> 쓸 수 있는 노드 이름"""
    name = re.sub(r"[^A-Za-z0-9_]", "_", f"stf_{key}")
    name = re.sub(r"_{2,}", "_", name)
    return name


def _load(context):
    share = Path(get_package_share_directory("cobot3_bringup"))
    cfg = yaml.safe_load((share / "config" / CONFIG_NAME).read_text(encoding="utf-8"))

    ns = LaunchConfiguration("robot_ns").perform(context).strip("/")
    only = [k for k in LaunchConfiguration("only").perform(context).split(",") if k]

    prefix = f"{ns}/" if ns else ""
    nodes = []

    for key, tf in cfg["static_transforms"].items():
        if only and key not in only:
            continue
        qx, qy, qz, qw = tf["quat_xyzw"]
        x, y, z = tf["xyz"]
        nodes.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            # ROS 2 노드 이름 규칙: [a-zA-Z_][a-zA-Z0-9_]* 이고 밑줄 두 개가
            # 연달아 오면 안 된다. frames.yaml 의 키는 parent__child 라 그대로
            # 쓰면 거부당한다.
            name=_node_name(key),
            output="screen",
            arguments=[
                "--x", str(x), "--y", str(y), "--z", str(z),
                "--qx", str(qx), "--qy", str(qy), "--qz", str(qz), "--qw", str(qw),
                "--frame-id", prefix + tf["parent"],
                "--child-frame-id", prefix + tf["child"],
            ],
        ))

    if not nodes:
        raise RuntimeError(
            f"발행할 변환이 없다. only={only!r} 가 "
            f"{list(cfg['static_transforms'])} 중 아무것도 못 골랐다.")
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_ns", default_value="",
            description="프레임 이름 접두사. 카터 두 대를 동시에 띄울 때 쓴다."),
        DeclareLaunchArgument(
            "only", default_value="",
            description="발행할 static_transforms 키를 쉼표로. 비우면 전부."),
        OpaqueFunction(function=_load),
    ])
