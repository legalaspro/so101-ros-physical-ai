from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    
    # --- Launch arguments ---
    hardware_type = LaunchConfiguration("hardware_type")  # real|mock|mujoco 
    leader_ns = LaunchConfiguration("leader_namespace")
    follower_ns = LaunchConfiguration("follower_namespace")

    leader_usb = LaunchConfiguration("leader_usb_port")
    follower_usb = LaunchConfiguration("follower_usb_port")

    leader_joint_cfg = LaunchConfiguration("leader_joint_config_file")
    follower_joint_cfg = LaunchConfiguration("follower_joint_config_file")

    leader_ctrl_cfg = LaunchConfiguration("leader_controller_config_file")
    follower_ctrl_cfg = LaunchConfiguration("follower_controller_config_file")

    leader_rviz = LaunchConfiguration("leader_rviz")
    follower_rviz = LaunchConfiguration("follower_rviz")

    arm_controller = LaunchConfiguration("arm_controller")  # arm_trajectory_controller|arm_forward_controller

    teleop_params_file = LaunchConfiguration("teleop_params_file")
    teleop_delay_s = LaunchConfiguration("teleop_delay_s")

    # --- Include leader bringup ---
    leader_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("so101_bringup"), "launch", "leader.launch.py"])
        ),
        launch_arguments={
            "namespace": leader_ns,
            "hardware_type": hardware_type,
            "usb_port": leader_usb,
            "joint_config_file": leader_joint_cfg,
            "controller_config_file": leader_ctrl_cfg,
            "use_rviz": leader_rviz,
        }.items(),
    )

    # --- Include follower bringup ---
    follower_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("so101_bringup"), "launch", "follower_split.launch.py"])
        ),
        launch_arguments={
            "namespace": follower_ns,
            "hardware_type": hardware_type,
            "usb_port": follower_usb,
            "joint_config_file": follower_joint_cfg,
            "controller_config_file": follower_ctrl_cfg,
            "use_rviz": follower_rviz,
            "arm_controller": arm_controller,
        }.items(),
    )

    # --- Include teleop launch ---
    teleop_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("so101_teleop"), "launch", "teleop_split.launch.py"])
        ),
        launch_arguments={
            "leader_namespace": leader_ns,
            "follower_namespace": follower_ns,
            "arm_controller": arm_controller,
            # optionally pass params_file if you want
            "params_file": teleop_params_file,
        }.items(),
    )

    teleop_start = TimerAction(
        period=teleop_delay_s,
        actions=[teleop_include],
    )

    # --- Defaults for files ---
    default_leader_joint_cfg = PathJoinSubstitution(
        [FindPackageShare("so101_bringup"), "config", "hardware", "leader_joints.yaml"]
    )
    default_follower_joint_cfg = PathJoinSubstitution(
        [FindPackageShare("so101_bringup"), "config", "hardware", "follower_joints.yaml"]
    )
    default_leader_ctrl_cfg = PathJoinSubstitution(
        [FindPackageShare("so101_bringup"), "config", "ros2_control", "leader_controllers.yaml"]
    )
    default_follower_ctrl_cfg = PathJoinSubstitution(
        [FindPackageShare("so101_bringup"), "config", "ros2_control", "follower_split_controllers.yaml"]
    )
    default_teleop_params = PathJoinSubstitution(
        [FindPackageShare("so101_teleop"), "config", "teleop.yaml"]
    )

    return LaunchDescription([
        DeclareLaunchArgument("hardware_type", default_value="real"),
        DeclareLaunchArgument("leader_namespace", default_value="leader"),
        DeclareLaunchArgument("follower_namespace", default_value="follower"),

        DeclareLaunchArgument("leader_usb_port", default_value="/dev/so101_leader"),
        DeclareLaunchArgument("follower_usb_port", default_value="/dev/so101_follower"),

        DeclareLaunchArgument("leader_joint_config_file", default_value=default_leader_joint_cfg),
        DeclareLaunchArgument("follower_joint_config_file", default_value=default_follower_joint_cfg),

        DeclareLaunchArgument("leader_controller_config_file", default_value=default_leader_ctrl_cfg),
        DeclareLaunchArgument("follower_controller_config_file", default_value=default_follower_ctrl_cfg),

        DeclareLaunchArgument("leader_rviz", default_value="false"),
        DeclareLaunchArgument("follower_rviz", default_value="false"),

        DeclareLaunchArgument("arm_controller", default_value="arm_trajectory_controller"),

        DeclareLaunchArgument("teleop_params_file", default_value=default_teleop_params),
        DeclareLaunchArgument("teleop_delay_s", default_value="2.0"),

        leader_launch,
        follower_launch,
        teleop_start,
    ])
