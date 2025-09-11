import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory("go2_description")
    sdk_launch_path = os.path.join(pkg_share, "launch", "sdk.launch.py")

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "network_interface",
            default_value='lo',
            description="network_interface for Unitree SDK2",
        )
    )

    network_interface = LaunchConfiguration("network_interface")

    nodes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sdk_launch_path),
            launch_arguments={
                "rl_policy": "policy_jump_gym.onnx",
                "controller_config": "jump_controller_gym.yaml",
                "network_interface": network_interface,
            }.items(),
        )
    ]

    return LaunchDescription(
        declared_arguments + nodes
    )

