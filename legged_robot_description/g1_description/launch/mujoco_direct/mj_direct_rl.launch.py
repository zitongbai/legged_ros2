from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetParameter
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
            "controller_config_file", 
            default_value="ros2_controller_rl.yaml",
            description="ROS2 controller configuration file."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rl_policy_file",
            default_value="policy.onnx",
            description="RL policy file. This file is exported by IsaacLab automatically \
                        when playing the policy.",
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
            "teleop_keyboard",
            default_value="true",
            description="Start teleop keyboard node.",
        )
    )
    
    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    mjcf_file = LaunchConfiguration("mjcf_file")
    controller_config_file = LaunchConfiguration("controller_config_file")
    rl_policy_file = LaunchConfiguration("rl_policy_file")
    use_rviz = LaunchConfiguration("use_rviz")
    use_rqt_cm = LaunchConfiguration("use_rqt_cm")
    teleop_keyboard = LaunchConfiguration("teleop_keyboard")

    rl_policy_file_param = SetParameter(
        name="rl_policy_path", 
        value=PathJoinSubstitution(
            [FindPackageShare(description_package), "config", "rl", rl_policy_file]
        )
    )
    
    mujoco_direct_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("legged_ros2_control"), "launch", "mujoco_direct.launch.py"]
            )
        ),
        launch_arguments={
            "description_package": description_package,
            "description_file": description_file,
            "mjcf_file": mjcf_file,
            "gui": use_rviz,
            "controller_config_file": controller_config_file,
            "teleop_keyboard": teleop_keyboard,
        }.items(),
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    static_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["static_controller", "-c", "/controller_manager", "--inactive"],
    )

    rl_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rl_controller", "-c", "/controller_manager", "--inactive"],
    )
    
    rqt_controller_manager = Node(
        package="rqt_controller_manager",
        executable="rqt_controller_manager",
        condition=IfCondition(use_rqt_cm),
    )
    
    
    params = [rl_policy_file_param]
    
    nodes = [
        mujoco_direct_launch,
        joint_state_broadcaster_spawner,
        static_controller_spawner,
        rl_controller_spawner,
        rqt_controller_manager,
    ]
    
    return LaunchDescription(declared_arguments + params + nodes)
    