"""Recording session: arms + cameras + headless episode recorder."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable
from launch.actions import ExecuteProcess

def generate_launch_description():
    
    # --- Launch arguments ---
    hardware_type = LaunchConfiguration("hardware_type")  # real|mock|mujoco 
    leader_ns = LaunchConfiguration("leader_namespace")
    follower_ns = LaunchConfiguration("follower_namespace")
    leader_frame_prefix = LaunchConfiguration("leader_frame_prefix")
    follower_frame_prefix = LaunchConfiguration("follower_frame_prefix")
    
    leader_usb = LaunchConfiguration("leader_usb_port")
    follower_usb = LaunchConfiguration("follower_usb_port")

    leader_joint_cfg = LaunchConfiguration("leader_joint_config_file")
    follower_joint_cfg = LaunchConfiguration("follower_joint_config_file")

    leader_ctrl_cfg = LaunchConfiguration("leader_controller_config_file")
    follower_ctrl_cfg = LaunchConfiguration("follower_controller_config_file")

    arm_controller = LaunchConfiguration("arm_controller")  # trajectory_controller|forward_controller
    cameras_config_file = LaunchConfiguration("cameras_config_file")

    teleop_params_file = LaunchConfiguration("teleop_params_file")
    teleop_delay_s = LaunchConfiguration("teleop_delay_s")

    recording_config_file = LaunchConfiguration("recording_config_file")
    root_dir = LaunchConfiguration("root_dir")
    experiment_name = LaunchConfiguration("experiment_name")

    use_rerun = LaunchConfiguration("use_rerun")
    rerun_env_dir = LaunchConfiguration("rerun_env_dir")
    rerun_delay_s = LaunchConfiguration("rerun_delay_s")

    # --- Include leader bringup ---
    leader_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("so101_bringup"), "launch", "leader.launch.py"])
        ),
        launch_arguments={
            "namespace": leader_ns,
            "hardware_type": hardware_type,
            "usb_port": leader_usb,
            "frame_prefix": leader_frame_prefix,
            "joint_config_file": leader_joint_cfg,
            "controller_config_file": leader_ctrl_cfg,
            "use_rviz": 'false',
        }.items(),
    )

    # --- Include follower bringup ---
    follower_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("so101_bringup"), "launch", "follower.launch.py"])
        ),
        launch_arguments={
            "namespace": follower_ns,
            "hardware_type": hardware_type,
            "usb_port": follower_usb,
            "frame_prefix": follower_frame_prefix,
            "joint_config_file": follower_joint_cfg,
            "controller_config_file": follower_ctrl_cfg,
            "use_rviz": 'false',
            "arm_controller": arm_controller,
        }.items(),
    )

    # --- Include cameras launch ---
    cameras_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("so101_bringup"), "launch", "cameras.launch.py"])
        ),
        launch_arguments={
            "cameras_config": cameras_config_file,
        }.items(),
    )

    # --- Include teleop launch ---
    teleop_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("so101_teleop"), "launch", "teleop.launch.py"])
        ),
        launch_arguments={
            "leader_namespace": leader_ns,
            "follower_namespace": follower_ns,
            "arm_controller": arm_controller,
            "params_file": teleop_params_file,
        }.items(),
    )

    teleop_start = TimerAction(
        period=teleop_delay_s,
        actions=[teleop_include],
    )


    # --- Include Headless Recorder launch ---
    recorder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("episode_recorder"), "launch", "recorder.launch.py"])
        ),
        launch_arguments={
            "params_file": recording_config_file,
            "root_dir": root_dir,
            "experiment_name": experiment_name,
        }.items(),
    )

    # --- Launch Rerun 
    rerun_bridge_proc = ExecuteProcess(
        cmd=[
            "pixi", "run", "bridge", "--",
            # "--wrist", rerun_wrist,
            # "--overhead", rerun_overhead,
            # "--joint-states", rerun_joint_states,
            # "--forward-commands", rerun_forward_cmds,
            # "--joint-trajectory", rerun_joint_traj,
        ],
        cwd=rerun_env_dir,
        additional_env={"PYTHONUNBUFFERED": "1"},
        condition=IfCondition(use_rerun),
        output="screen",
    )

    rerun_start = TimerAction(
        period=rerun_delay_s,
        actions=[rerun_bridge_proc],
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
        [FindPackageShare("so101_bringup"), "config", "ros2_control", "follower_controllers.yaml"]
    )
    default_cameras_cfg = PathJoinSubstitution(
        [FindPackageShare("so101_bringup"), "config", "cameras", "so101_cameras.yaml"]
    )
    default_teleop_params = PathJoinSubstitution(
        [FindPackageShare("so101_teleop"), "config", "teleop.yaml"]
    )
    default_recording_cfg = PathJoinSubstitution(
        [FindPackageShare("so101_bringup"), "config", "recording", "episode_recorder_so101.yaml"]
    )
    default_root_dir =  PathJoinSubstitution([
        EnvironmentVariable("ROS_HOME", default_value=PathJoinSubstitution([EnvironmentVariable("HOME"), ".ros"])),
        "so101_episodes",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("hardware_type", default_value="real"),
        DeclareLaunchArgument("leader_namespace", default_value="leader"),
        DeclareLaunchArgument("follower_namespace", default_value="follower"),

        DeclareLaunchArgument("leader_frame_prefix", default_value="leader/"),
        DeclareLaunchArgument("follower_frame_prefix", default_value="follower/"),

        DeclareLaunchArgument("leader_usb_port", default_value="/dev/so101_leader"),
        DeclareLaunchArgument("follower_usb_port", default_value="/dev/so101_follower"),

        DeclareLaunchArgument("leader_joint_config_file", default_value=default_leader_joint_cfg),
        DeclareLaunchArgument("follower_joint_config_file", default_value=default_follower_joint_cfg),

        DeclareLaunchArgument("leader_controller_config_file", default_value=default_leader_ctrl_cfg),
        DeclareLaunchArgument("follower_controller_config_file", default_value=default_follower_ctrl_cfg),

        DeclareLaunchArgument("arm_controller", default_value="forward_controller"),
        DeclareLaunchArgument("cameras_config_file", default_value=default_cameras_cfg),

        DeclareLaunchArgument("teleop_params_file", default_value=default_teleop_params),
        DeclareLaunchArgument("teleop_delay_s", default_value="2.0"),

        DeclareLaunchArgument("recording_config_file", default_value=default_recording_cfg),
        DeclareLaunchArgument("root_dir", default_value=default_root_dir),
        DeclareLaunchArgument("experiment_name", default_value="pick_and_place"),

        DeclareLaunchArgument("use_rerun", default_value="false"),
        DeclareLaunchArgument(
            "rerun_env_dir",
            # Best: set env var once, no need to pass each run:
            # export SO101_RERUN_ENV_DIR=/abs/path/to/tools/rerun_env
            default_value=EnvironmentVariable("SO101_RERUN_ENV_DIR", default_value=""),
        ),
        DeclareLaunchArgument("rerun_delay_s", default_value=teleop_delay_s),

        leader_launch,
        follower_launch,
        cameras_launch,
        recorder_launch,
        rerun_start,
        teleop_start
    ])
