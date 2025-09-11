
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
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
            default_value="go2_description",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="robot.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_config", 
            default_value="rand_traj_controller.yaml",
            description="Controller configuration file.",
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
            default_value="true",
            description="Start rqt_controller_manager automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "network_interface",
            default_value='lo',
            description="network_interface for Unitree SDK2",
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

    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    controller_config = LaunchConfiguration("controller_config")
    use_rviz = LaunchConfiguration("use_rviz")
    use_rqt_cm = LaunchConfiguration("use_rqt_cm")
    prefix = LaunchConfiguration("prefix")
    network_interface = LaunchConfiguration("network_interface")
    enable_lowlevel_write = LaunchConfiguration("enable_lowlevel_write")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "xacro", description_file]
            ),
            " ", 
            "prefix:=", 
            prefix,
            " ", 
            "enable_sim:=",
            "false", 
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    controller_params = {
        "network_interface": network_interface,
        "enable_lowlevel_write": enable_lowlevel_write
    }

    controller_config_path = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "config",
            controller_config
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz2", "go2.rviz"]
    )

    control_node = Node(
        package="legged_ros2_control",
        executable="go2_node",
        parameters=[controller_config_path, robot_description, controller_params],
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

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(use_rviz),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    rand_traj_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rand_traj_controller", "-c", "/controller_manager", "--inactive"],
    )

    rqt_controller_manager = Node(
        package="rqt_controller_manager",
        executable="rqt_controller_manager",
        condition=IfCondition(use_rqt_cm),
    )

    # rqt_robot_steering = Node(
    #     package="rqt_robot_steering",
    #     executable="rqt_robot_steering",
    # )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of joint_state_broadcaster and rl_controller after rand_traj_controller_spawner
    delay_after_rand_traj_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rand_traj_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        rand_traj_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_after_rand_traj_controller_spawner,
        rqt_controller_manager,
        # rqt_robot_steering
    ]

    return LaunchDescription(declared_arguments + nodes)




