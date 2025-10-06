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
            "mjcf_file", 
            default_value="scene.xml",
            description="MJCF file for the robot."
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
            "controller_config_file", 
            default_value="ros2_controller.yaml",
            description="ROS2 controller configuration file."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "teleop_keyboard",
            default_value="true",
            description="Start teleop keyboard node.",
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
    controller_config_file = LaunchConfiguration("controller_config_file")
    gui = LaunchConfiguration("gui")
    teleop_keyboard = LaunchConfiguration("teleop_keyboard")
    prefix = LaunchConfiguration("prefix")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    mjcf_file_path = {
        "mujoco_model_path": PathJoinSubstitution(
            [FindPackageShare(description_package), "mjcf", mjcf_file]
        )
    }

    controller_config = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "config",
            "mujoco_direct",
            controller_config_file
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz2", "go2.rviz"]
    )

    control_node = Node(
        package="legged_ros2_control",
        executable="mujoco_node",
        parameters=[controller_config, mjcf_file_path, robot_description],
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
    
    teleop_keyboard_node = Node(
        package="legged_ros2_control",
        executable="legged_teleop_keyboard.py",
        name="teleop_keyboard",
        output="screen",
        condition=IfCondition(teleop_keyboard),
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        rviz_node,
        teleop_keyboard_node
    ]

    return LaunchDescription(declared_arguments + nodes)




