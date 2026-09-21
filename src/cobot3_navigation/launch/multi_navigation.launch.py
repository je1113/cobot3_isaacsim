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
    # LAUNCH CONFIGURATION
    # ==========================================================

    use_sim_time = LaunchConfiguration(
        "use_sim_time"
    )

    use_rviz = LaunchConfiguration(
        "use_rviz"
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

    workspace_dir = os.path.expanduser("~/cobot3_ws")

    robot_urdf_file = os.path.join(
        workspace_dir,
        "isaacpjt",
        "cobot3",
        "urdf",
        "cobot3.urdf"
    )

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

    nav2_bringup_launch = os.path.join(
        nav2_bringup_dir,
        "launch",
        "bringup_launch.py"
    )


    # ==========================================================
    # ROBOT DESCRIPTION
    # ==========================================================

    with open(robot_urdf_file, "r") as urdf_file:
        robot_description = urdf_file.read()


    # ==========================================================
    # ROBOT 1 STATE PUBLISHER
    #
    # robot_description
    # -> /robot1/robot_description
    #
    # TF
    # -> /robot1/tf
    # -> /robot1/tf_static
    # ==========================================================

    robot1_state_publisher = Node(

        package="robot_state_publisher",

        executable="robot_state_publisher",

        namespace="robot1",

        name="robot_state_publisher",

        output="screen",

        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time,
        }],


        remappings=[
            (
                "/tf",
                "tf"
            ),
            (
                "/tf_static",
                "tf_static"
            ),
        ],
    )


    # ==========================================================
    # ROBOT 1 JOINT STATE PUBLISHER
    # ==========================================================

    # robot1_joint_state_publisher = Node(
    #     package="joint_state_publisher",
    #     executable="joint_state_publisher",

    #     namespace="robot1",

    #     name="joint_state_publisher",

    #     output="screen",

    #     parameters=[{
    #         "robot_description": robot_description,
    #         "use_sim_time": use_sim_time,
    #     }],
    # )


    # ==========================================================
    # ROBOT 1 base_link -> chassis_link
    # ==========================================================

    robot1_base_to_chassis = Node(
        package="tf2_ros",
        executable="static_transform_publisher",

        namespace="robot1",

        name="base_to_chassis",

        output="screen",

        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", "base_link",
            "--child-frame-id", "chassis_link",
        ],

        remappings=[
            (
                "/tf_static",
                "tf_static"
            ),
        ],
    )

    # ==========================================================
    # ROBOT 2 STATE PUBLISHER
    #
    # robot_description
    # -> /robot2/robot_description
    #
    # TF
    # -> /robot2/tf
    # -> /robot2/tf_static
    # ==========================================================

    robot2_state_publisher = Node(

        package="robot_state_publisher",

        executable="robot_state_publisher",

        namespace="robot2",

        name="robot_state_publisher",

        output="screen",

        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time,
        }],

        remappings=[
            (
                "/tf",
                "tf"
            ),
            (
                "/tf_static",
                "tf_static"
            ),
        ],
    )

    # ==========================================================
    # ROBOT 2 JOINT STATE PUBLISHER
    # ==========================================================

    # robot2_joint_state_publisher = Node(
    #     package="joint_state_publisher",
    #     executable="joint_state_publisher",

    #     namespace="robot2",

    #     name="joint_state_publisher",

    #     output="screen",

    #     parameters=[{
    #         "robot_description": robot_description,
    #         "use_sim_time": use_sim_time,
    #     }],
    # )


    # ==========================================================
    # ROBOT 2 base_link -> chassis_link
    # ==========================================================

    robot2_base_to_chassis = Node(
        package="tf2_ros",
        executable="static_transform_publisher",

        namespace="robot2",

        name="base_to_chassis",

        output="screen",

        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", "base_link",
            "--child-frame-id", "chassis_link",
        ],

        remappings=[
            (
                "/tf_static",
                "tf_static"
            ),
        ],
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

            "use_sim_time": use_sim_time,
        }],
    )


    # ==========================================================
    # ROBOT 1 NAV2
    # ==========================================================

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
    # robot1_nav2.rviz:
    #
    # Fixed Frame = map
    # Map = /robot1/map
    # LaserScan = /robot1/scan
    # RobotModel = /robot1/robot_description
    # ==========================================================

    robot1_rviz = Node(

        package="rviz2",

        executable="rviz2",
        namespace="robot1",
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
            "use_sim_time": use_sim_time
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

            "use_sim_time": use_sim_time,
        }],
    )


    # ==========================================================
    # ROBOT 2 NAV2
    # ==========================================================

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
    # robot2_nav2.rviz:
    #
    # Fixed Frame = map
    # Map = /robot2/map
    # LaserScan = /robot2/scan
    # RobotModel = /robot2/robot_description
    # ==========================================================

    robot2_rviz = Node(

        package="rviz2",

        executable="rviz2",
        namespace="robot2",
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
            "use_sim_time": use_sim_time
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

        robot1_state_publisher,

        # robot1_joint_state_publisher,

        robot1_base_to_chassis,

        robot1_nav2,

        robot1_rviz,

        # ------------------------------------------------------
        # Robot 2
        # ------------------------------------------------------

        robot2_pointcloud_to_scan,

        robot2_state_publisher,

        # robot2_joint_state_publisher,

        robot2_base_to_chassis,

        robot2_nav2,

        robot2_rviz,

    ])