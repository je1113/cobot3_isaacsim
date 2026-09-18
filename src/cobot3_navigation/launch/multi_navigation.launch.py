import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription

from launch.conditions import IfCondition

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    # ==========================================================
    # PACKAGE PATH
    # ==========================================================

    package_dir = get_package_share_directory(
        "cobot3_navigation"
    )

    nav2_bringup_dir = get_package_share_directory(
        "nav2_bringup"
    )


    # ==========================================================
    # COMMON FILES
    # ==========================================================

    map_file = os.path.join(
        package_dir,
        "maps",
        "simple_factory_layout.yaml"
    )

    robot1_params = os.path.join(
        package_dir,
        "params",
        "robot1_nav2_params.yaml"
    )

    robot2_params = os.path.join(
        package_dir,
        "params",
        "robot2_nav2_params.yaml"
    )

    # ----------------------------------------------------------
    # RVIZ CONFIG FILES
    # ----------------------------------------------------------

    robot1_rviz_config = os.path.join(
        package_dir,
        "rviz",
        "robot1_nav2.rviz"
    )

    robot2_rviz_config = os.path.join(
        package_dir,
        "rviz",
        "robot2_nav2.rviz"
    )


    # ==========================================================
    # NAV2 BRINGUP
    # ==========================================================

    nav2_bringup_launch = os.path.join(
        nav2_bringup_dir,
        "launch",
        "bringup_launch.py"
    )


    # ==========================================================
    # LAUNCH CONFIGURATION
    # ==========================================================

    use_sim_time = LaunchConfiguration(
        "use_sim_time"
    )

    use_rviz = LaunchConfiguration(
        "use_rviz"
    )


    # ==========================================================
    # ROBOT 1
    #
    # Isaac Sim expected topics:
    #
    # /robot1/chassis/odom
    # /robot1/cmd_vel
    # /robot1/front_3d_lidar/lidar_points
    # /robot1/tf
    # ==========================================================

    robot1_pointcloud_to_scan = Node(

        package="pointcloud_to_laserscan",

        executable="pointcloud_to_laserscan_node",

        namespace="robot1",

        name="pointcloud_to_laserscan",

        output="screen",

        remappings=[

            # PointCloud input
            (
                "cloud_in",
                "front_3d_lidar/lidar_points"
            ),

            # LaserScan output
            (
                "scan",
                "scan"
            ),

            # Robot1 TF
            (
                "/tf",
                "tf"
            ),

            (
                "/tf_static",
                "tf_static"
            ),
        ],

        parameters=[{

            "target_frame": "front_3d_lidar",

            "transform_tolerance": 0.01,

            # 기존 NVIDIA 설정 유지
            "min_height": -0.4,
            "max_height": 1.5,

            # 전방 180도
            "angle_min": -1.5708,
            "angle_max": 1.5708,

            # 약 0.5도
            "angle_increment": 0.0087,

            "scan_time": 0.3333,

            "range_min": 0.05,
            "range_max": 100.0,

            "use_inf": True,
            "inf_epsilon": 1.0,

            "use_sim_time": True,
        }],
    )


    robot1_nav2 = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(
            nav2_bringup_launch
        ),

        launch_arguments={

            "namespace": "robot1",

            "use_namespace": "True",

            "map": map_file,

            "use_sim_time": use_sim_time,

            "params_file": robot1_params,

            "autostart": "True",

        }.items(),
    )


    # ==========================================================
    # ROBOT 1 RVIZ
    #
    # robot1_nav2.rviz 내부:
    #
    # Fixed Frame = map
    # Map       = /robot1/map
    # LaserScan = /robot1/scan
    # ==========================================================

    robot1_rviz = Node(

        package="rviz2",

        executable="rviz2",

        name="rviz2_robot1",

        output="screen",

        arguments=[
            "-d",
            robot1_rviz_config
        ],

        remappings=[

            (
                "/tf",
                "/robot1/tf"
            ),

            (
                "/tf_static",
                "/robot1/tf_static"
            ),
        ],

        parameters=[{
            "use_sim_time": True
        }],

        condition=IfCondition(
            use_rviz
        ),
    )


    # ==========================================================
    # ROBOT 2
    #
    # Isaac Sim expected topics:
    #
    # /robot2/chassis/odom
    # /robot2/cmd_vel
    # /robot2/front_3d_lidar/lidar_points
    # /robot2/tf
    # ==========================================================

    robot2_pointcloud_to_scan = Node(

        package="pointcloud_to_laserscan",

        executable="pointcloud_to_laserscan_node",

        namespace="robot2",

        name="pointcloud_to_laserscan",

        output="screen",

        remappings=[

            # PointCloud input
            (
                "cloud_in",
                "front_3d_lidar/lidar_points"
            ),

            # LaserScan output
            (
                "scan",
                "scan"
            ),

            # Robot2 TF
            (
                "/tf",
                "tf"
            ),

            (
                "/tf_static",
                "tf_static"
            ),
        ],

        parameters=[{

            "target_frame": "front_3d_lidar",

            "transform_tolerance": 0.01,

            "min_height": -0.4,
            "max_height": 1.5,

            "angle_min": -1.5708,
            "angle_max": 1.5708,

            "angle_increment": 0.0087,

            "scan_time": 0.3333,

            "range_min": 0.05,
            "range_max": 100.0,

            "use_inf": True,
            "inf_epsilon": 1.0,

            "use_sim_time": True,
        }],
    )


    robot2_nav2 = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(
            nav2_bringup_launch
        ),

        launch_arguments={

            "namespace": "robot2",

            "use_namespace": "True",

            "map": map_file,

            "use_sim_time": use_sim_time,

            "params_file": robot2_params,

            "autostart": "True",

        }.items(),
    )


    # ==========================================================
    # ROBOT 2 RVIZ
    #
    # robot2_nav2.rviz 내부:
    #
    # Fixed Frame = map
    # Map       = /robot2/map
    # LaserScan = /robot2/scan
    # ==========================================================

    robot2_rviz = Node(

        package="rviz2",

        executable="rviz2",

        name="rviz2_robot2",

        output="screen",

        arguments=[
            "-d",
            robot2_rviz_config
        ],

        remappings=[

            (
                "/tf",
                "/robot2/tf"
            ),

            (
                "/tf_static",
                "/robot2/tf_static"
            ),
        ],

        parameters=[{
            "use_sim_time": True
        }],

        condition=IfCondition(
            use_rviz
        ),
    )


    # ==========================================================
    # LAUNCH
    # ==========================================================

    return LaunchDescription([

        # ------------------------------------------------------
        # Launch arguments
        # ------------------------------------------------------

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="True",
            description="Use Isaac Sim simulation clock"
        ),

        DeclareLaunchArgument(
            "use_rviz",
            default_value="True",
            description="Automatically launch RViz for robot1 and robot2"
        ),


        # ------------------------------------------------------
        # Robot 1
        # ------------------------------------------------------

        robot1_pointcloud_to_scan,
        robot1_nav2,
        robot1_rviz,


        # ------------------------------------------------------
        # Robot 2
        # ------------------------------------------------------

        robot2_pointcloud_to_scan,
        robot2_nav2,
        robot2_rviz,
    ])