from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
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

    builtin_lidar_frame_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="utlidar_frame_static_tf",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            "utlidar_link",
            "--child-frame-id",
            "utlidar_lidar",
        ],
        output="screen",
    )

    return LaunchDescription([lidar_static_tf_launch, builtin_lidar_frame_tf])
