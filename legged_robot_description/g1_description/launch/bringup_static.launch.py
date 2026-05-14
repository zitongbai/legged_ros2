import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="g1_description",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="g1_29dof_rev_1_0.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_config",
            default_value="static.yaml",
            description="Controller configuration file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "main_loop_config",
            default_value="main_loop/static.yaml",
            description="Main loop configuration file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_lowlevel_write",
            default_value="true",
            description="Enable low-level command writing, useful in debugging or testing scenarios. \
                        If set to true, the robot will receive low-level commands from the controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rqt_cm",
            default_value="false",
            description="Start rqt_controller_manager automatically with this launch file.",
        )
    )

    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    controller_config = LaunchConfiguration("controller_config")
    main_loop_config = LaunchConfiguration("main_loop_config")
    enable_lowlevel_write = LaunchConfiguration("enable_lowlevel_write")
    use_rviz = LaunchConfiguration("use_rviz")
    use_rqt_cm = LaunchConfiguration("use_rqt_cm")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "enable_lowlevel_write:=",
            enable_lowlevel_write,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    controller_config_path = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "config",
            "ros2_control",
            controller_config,
        ]
    )

    main_loop_config_path = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "config",
            main_loop_config,
        ]
    )

    main_loop_node = Node(
        package="legged_ros2_control",
        executable="g1_main_loop",
        parameters=[controller_config_path, robot_description, main_loop_config_path],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # -----------------------------------------------------------------------
    # RVIZ
    # -----------------------------------------------------------------------
    pkg_share = get_package_share_directory("g1_description")
    rviz_config_file = os.path.join(pkg_share, "rviz2", "g1.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(use_rviz),
    )

    rqt_controller_manager = Node(
        package="rqt_controller_manager",
        executable="rqt_controller_manager",
        condition=IfCondition(use_rqt_cm),
    )

    # -----------------------------------------------------------------------
    # Spawners
    # -----------------------------------------------------------------------
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    imu_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    stand_static_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["stand_static_controller", "-c", "/controller_manager", "--inactive"],
    )

    sit_static_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["sit_static_controller", "-c", "/controller_manager", "--inactive"],
    )

    delay_after_stand_static_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=stand_static_controller_spawner,
            on_exit=[
                joint_state_broadcaster_spawner,
                imu_state_broadcaster_spawner,
                sit_static_controller_spawner,
                rviz_node,
                rqt_controller_manager,
            ],
        )
    )

    nodes = [
        main_loop_node,
        robot_state_pub_node,
        stand_static_controller_spawner,
        delay_after_stand_static_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
