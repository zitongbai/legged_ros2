
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
            "config",
            default_value="config_g1.yaml",
            description="Configuration file for the simulation."
        )
    )
    
    config = LaunchConfiguration("config")
    config_file = {
        "config_file": PathJoinSubstitution(
            [FindPackageShare("unitree_mujoco"), "config", config]
        )
    }

    unitree_mujoco_node = Node(
        package="unitree_mujoco",
        executable="unitree_mujoco",
        output="screen",
        parameters=[config_file]
    )

    nodes = [
        unitree_mujoco_node,
    ]

    return LaunchDescription(declared_arguments + nodes)