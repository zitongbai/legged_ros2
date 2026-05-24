from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mocap_type = LaunchConfiguration("mocap_type")
    hostname = LaunchConfiguration("hostname")
    frame_id = LaunchConfiguration("frame_id")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mocap_type",
                default_value="nokov",
                description="Motion capture backend type.",
            ),
            DeclareLaunchArgument(
                "hostname",
                default_value="192.168.50.95",
                description="Motion capture server hostname or IP address.",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="world",
                description="Frame ID used in published PoseStamped messages.",
            ),
            Node(
                package="legged_mocap",
                executable="mocap_pose_node",
                name="mocap_pose_node",
                output="screen",
                parameters=[
                    {
                        "mocap_type": mocap_type,
                        "hostname": hostname,
                        "frame_id": frame_id,
                    }
                ],
            ),
        ]
    )
