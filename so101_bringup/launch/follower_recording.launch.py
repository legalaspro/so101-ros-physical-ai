"""Follower arm + cameras + headless episode recorder + optional rerun.

Generic data-collection / perception / recorder stack.
Task-specific launch files (training, teleop, etc.) should include this.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Launch arguments ─────────────────────────────────────────
    hardware_type = LaunchConfiguration("hardware_type")
    follower_ns = LaunchConfiguration("follower_namespace")
    follower_frame_prefix = LaunchConfiguration("follower_frame_prefix")
    follower_usb = LaunchConfiguration("follower_usb_port")
    follower_joint_cfg = LaunchConfiguration("follower_joint_config_file")
    follower_ctrl_cfg = LaunchConfiguration("follower_controller_config_file")
    arm_controller = LaunchConfiguration("arm_controller")
    cameras_config_file = LaunchConfiguration("cameras_config_file")

    recording_config_file = LaunchConfiguration("recording_config_file")
    root_dir = LaunchConfiguration("root_dir")
    experiment_name = LaunchConfiguration("experiment_name")
    task = LaunchConfiguration("task")

    use_rerun = LaunchConfiguration("use_rerun")
    rerun_env_dir = LaunchConfiguration("rerun_env_dir")
    rerun_delay_s = LaunchConfiguration("rerun_delay_s")

    # ── Follower arm + cameras ───────────────────────────────────
    follower_vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("so101_bringup"), "launch", "follower_vision.launch.py"]
            )
        ),
        launch_arguments={
            "hardware_type": hardware_type,
            "follower_namespace": follower_ns,
            "follower_frame_prefix": follower_frame_prefix,
            "follower_usb_port": follower_usb,
            "follower_joint_config_file": follower_joint_cfg,
            "follower_controller_config_file": follower_ctrl_cfg,
            "arm_controller": arm_controller,
            "cameras_config_file": cameras_config_file,
            "use_rviz": "false",
        }.items(),
    )

    # ── Episode recorder (headless) ──────────────────────────────
    recorder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("episode_recorder"), "launch", "recorder.launch.py"]
            )
        ),
        launch_arguments={
            "params_file": recording_config_file,
            "root_dir": root_dir,
            "experiment_name": experiment_name,
            "task": task,
        }.items(),
    )

    # ── Optional rerun bridge ────────────────────────────────────
    rerun_bridge_proc = ExecuteProcess(
        cmd=["pixi", "run", "bridge", "--"],
        cwd=rerun_env_dir,
        additional_env={"PYTHONUNBUFFERED": "1"},
        condition=IfCondition(use_rerun),
        output="screen",
    )

    rerun_start = TimerAction(
        period=rerun_delay_s,
        actions=[rerun_bridge_proc],
    )

    # ── Defaults ─────────────────────────────────────────────────
    default_follower_joint_cfg = ""
    default_follower_ctrl_cfg = PathJoinSubstitution(
        [
            FindPackageShare("so101_bringup"),
            "config",
            "ros2_control",
            "follower_controllers.yaml",
        ]
    )
    default_cameras_cfg = PathJoinSubstitution(
        [FindPackageShare("so101_bringup"), "config", "cameras", "so101_cameras.yaml"]
    )
    default_recording_cfg = PathJoinSubstitution(
        [
            FindPackageShare("so101_bringup"),
            "config",
            "recording",
            "episode_recorder_so101.yaml",
        ]
    )
    default_root_dir = PathJoinSubstitution(
        [
            EnvironmentVariable(
                "ROS_HOME",
                default_value=PathJoinSubstitution([EnvironmentVariable("HOME"), ".ros"]),
            ),
            "so101_episodes",
        ]
    )

    return LaunchDescription(
        [
            # Arm + cameras
            DeclareLaunchArgument("hardware_type", default_value="real"),
            DeclareLaunchArgument("follower_namespace", default_value="follower"),
            DeclareLaunchArgument("follower_frame_prefix", default_value="follower/"),
            DeclareLaunchArgument("follower_usb_port", default_value="/dev/so101_follower"),
            DeclareLaunchArgument("follower_joint_config_file", default_value=default_follower_joint_cfg),
            DeclareLaunchArgument("follower_controller_config_file", default_value=default_follower_ctrl_cfg),
            DeclareLaunchArgument("arm_controller", default_value="forward_controller"),
            DeclareLaunchArgument("cameras_config_file", default_value=default_cameras_cfg),
            # Recorder
            DeclareLaunchArgument("recording_config_file", default_value=default_recording_cfg),
            DeclareLaunchArgument("root_dir", default_value=default_root_dir),
            DeclareLaunchArgument("experiment_name", default_value="pick_and_place"),
            DeclareLaunchArgument("task", default_value=""),
            # Rerun
            DeclareLaunchArgument("use_rerun", default_value="false"),
            DeclareLaunchArgument(
                "rerun_env_dir",
                default_value=EnvironmentVariable("SO101_RERUN_ENV_DIR", default_value=""),
            ),
            DeclareLaunchArgument("rerun_delay_s", default_value="3.0"),
            # Actions
            follower_vision_launch,
            recorder_launch,
            rerun_start,
        ]
    )
