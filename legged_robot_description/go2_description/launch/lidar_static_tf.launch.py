from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lidar_static_tf_params = PathJoinSubstitution(
        [
            FindPackageShare("go2_description"),
            "config",
            "mapping",
            "lidar_static_tf.yaml",
        ]
    )

    lidar_static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("legged_mapping"),
                    "launch",
                    "lidar_static_tf.launch.py",
                ]
            )
        ),
        launch_arguments={
            "params_file": lidar_static_tf_params,
            "use_sim_time": "false",
        }.items(),
    )

    return LaunchDescription([lidar_static_tf_launch])
