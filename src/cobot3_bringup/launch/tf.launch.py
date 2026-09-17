"""
TF 트리 전체를 세운다 — 정적 변환 + m0609 robot_state_publisher.

    ros2 launch cobot3_bringup tf.launch.py

    # Isaac 없이 트리 연결만 확인할 때 (가짜 관절값을 쏜다)
    ros2 launch cobot3_bringup tf.launch.py use_fake_joints:=true

    # 다른 터미널에서
    ros2 run tf2_tools view_frames

이 launch 가 책임지는 구간은 base_link 아래뿐이다.
map->odom->base_link 는 Nav2 와 Isaac 의 odometry 퍼블리셔가 채운다.

m0609 URDF 의 루트 링크 이름이 'base_link' 라 Carter 의 base_link 와 부딪히므로
frame_prefix 로 'm0609_' 를 붙인다. 값은 config/frames.yaml 의 description 에 있다.
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _repo_root(share):
    """install/ 아래 share 경로에서 워크스페이스 소스 루트를 되짚는다.

    frames.yaml 의 URDF 경로가 워크스페이스 상대경로라서 필요하다.
    install/<pkg>/share/<pkg> -> 워크스페이스 루트는 네 단계 위.
    """
    return Path(share).resolve().parents[3]


def _setup(context):
    share = get_package_share_directory("cobot3_bringup")
    cfg = yaml.safe_load(
        (Path(share) / "config" / "frames.yaml").read_text(encoding="utf-8"))
    desc = cfg["description"]

    urdf = _repo_root(share) / desc["m0609_urdf"]
    if not urdf.exists():
        raise RuntimeError(
            f"URDF 를 못 찾았다: {urdf}\n"
            f"frames.yaml 의 description.m0609_urdf 는 워크스페이스 루트 기준 "
            f"상대경로여야 한다.")

    fake = LaunchConfiguration("use_fake_joints")

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="m0609_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": urdf.read_text(encoding="utf-8"),
                "frame_prefix": desc["frame_prefix"],
            }],
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="m0609_fake_joints",
            output="screen",
            condition=IfCondition(fake),
            # 파라미터 없이 두면 모든 관절을 0 으로 발행한다. 트리 연결 확인에는
            # 그걸로 충분하다. (빈 리스트를 파라미터로 넘기면 launch_ros 가
            # 타입을 못 정해서 죽는다.)
        ),
    ]
    return nodes


def generate_launch_description():
    share = get_package_share_directory("cobot3_bringup")
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_fake_joints", default_value="false",
            description="Isaac 없이 트리만 확인할 때 joint_state_publisher 로 "
                        "관절값을 채운다."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(share) / "launch" / "tf_static.launch.py"))),
        OpaqueFunction(function=_setup),
    ])
