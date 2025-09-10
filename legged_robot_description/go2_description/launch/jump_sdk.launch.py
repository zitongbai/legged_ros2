import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("go2_description")
    sdk_launch_path = os.path.join(pkg_share, "launch", "sdk.launch.py")

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sdk_launch_path),
                launch_arguments={
                    "rl_policy": "policy_jump_gym.onnx",
                    "controller_config": "jump_controller_gym.yaml"
                }.items(),
            )
        ]
    )

