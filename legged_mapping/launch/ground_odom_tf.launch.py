from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("legged_mapping"),
                        "config",
                        "ground_odom_tf.yaml",
                    ]
                ),
                description="Path to the ground odom TF parameter file.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time for the ground odom TF node.",
            ),
            Node(
                package="legged_mapping",
                executable="ground_odom_tf_node",
                name="ground_odom_tf_node",
                output="screen",
                parameters=[
                    params_file,
                    {"use_sim_time": use_sim_time},
                ],
            ),
        ]
    )
