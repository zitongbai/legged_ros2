
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
            default_value="g1_description",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="g1_29dof_lock_waist_rev_1_0.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mjcf_file", 
            default_value="g1_29dof_lock_waist_rev_1_0.xml",
            description="MJCF file for the robot."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rl_policy", 
            default_value="policy.pt",
            description="RL policy file. This file is exported by IsaacLab automatically \
                        when playing the policy.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
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

    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    mjcf_file = LaunchConfiguration("mjcf_file")
    rl_policy = LaunchConfiguration("rl_policy")
    gui = LaunchConfiguration("gui")
    prefix = LaunchConfiguration("prefix")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ", 
            "prefix:=", 
            prefix,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    mjcf_file_path = {
        "mujoco_model_path": PathJoinSubstitution(
            [FindPackageShare(description_package), "mjcf", mjcf_file]
        )
    }

    rl_policy_path = {
        "rl_policy_path": PathJoinSubstitution(
            [FindPackageShare(description_package), "config", "rl", rl_policy]
        )
    }

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "config",
            "rl_controller.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz2", "g1.rviz"]
    )

    control_node = Node(
        package="legged_ros2_control",
        executable="mujoco_node",
        parameters=[robot_controllers, mjcf_file_path, rl_policy_path, robot_description],
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

    gui = LaunchConfiguration("gui")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rl_controller", "-c", "/controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of joint_state_broadcaster after `robot_controller`
    # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        robot_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner, 
    ]

    return LaunchDescription(declared_arguments + nodes)




