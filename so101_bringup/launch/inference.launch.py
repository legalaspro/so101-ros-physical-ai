"""Inference: follower arm + cameras."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import EnvironmentVariable
from launch.actions import ExecuteProcess


def generate_launch_description():
    # --- Launch arguments ---
    hardware_type = LaunchConfiguration("hardware_type")  # real|mock|mujoco
    follower_ns = LaunchConfiguration("follower_namespace")
    follower_frame_prefix = LaunchConfiguration("follower_frame_prefix")
    follower_usb = LaunchConfiguration("follower_usb_port")

    follower_joint_cfg = LaunchConfiguration("follower_joint_config_file")
    follower_ctrl_cfg = LaunchConfiguration("follower_controller_config_file")

    arm_controller = LaunchConfiguration(
        "arm_controller"
    )  # trajectory_controller|forward_controller
    cameras_config_file = LaunchConfiguration("cameras_config_file")

    # Inference toggles + policy params
    # use_inference = LaunchConfiguration("use_inference")
    # inference_delay_s = LaunchConfiguration("inference_delay_s")
    # policy_repo_id = LaunchConfiguration("policy_repo_id")
    # policy_device = LaunchConfiguration("policy_device")

    use_rerun = LaunchConfiguration("use_rerun")
    rerun_env_dir = LaunchConfiguration("rerun_env_dir")
    rerun_delay_s = LaunchConfiguration("rerun_delay_s")

    # --- Include follower bringup ---
    follower_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("so101_bringup"), "launch", "follower.launch.py"]
            )
        ),
        launch_arguments={
            "namespace": follower_ns,
            "hardware_type": hardware_type,
            "usb_port": follower_usb,
            "frame_prefix": follower_frame_prefix,
            "joint_config_file": follower_joint_cfg,
            "controller_config_file": follower_ctrl_cfg,
            "use_rviz": "false",
            "arm_controller": arm_controller,
        }.items(),
    )

    # --- Include cameras launch ---
    cameras_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("so101_bringup"), "launch", "cameras.launch.py"]
            )
        ),
        launch_arguments={"cameras_config": cameras_config_file}.items(),
    )

    # --- Inference node ---
    # inference_node = Node(
    #     package="so101_inference",
    #     executable="lerobot_inference_node",
    #     name="lerobot_inference_node",
    #     namespace="",
    #     output="screen",
    #     parameters=[
    #         {
    #             "repo_id": policy_repo_id,
    #             "checkpoint": policy_checkpoint,
    #             "device": policy_device,
    #             # Later you can add topic params here, e.g.:
    #             # "follower_namespace": follower_ns,
    #         }
    #     ],
    #     condition=IfCondition(use_inference),
    # )

    # inference_start = TimerAction(
    #     period=inference_delay_s,
    #     actions=[inference_node],
    #     condition=IfCondition(use_inference),
    # )

    # --- Launch Rerun
    rerun_bridge_proc = ExecuteProcess(
        cmd=[
            "pixi",
            "run",
            "bridge",
            "--",
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
    default_follower_joint_cfg = PathJoinSubstitution(
        [
            FindPackageShare("so101_bringup"),
            "config",
            "hardware",
            "follower_joints.yaml",
        ]
    )
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

    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware_type", default_value="real"),
            DeclareLaunchArgument("follower_namespace", default_value="follower"),
            DeclareLaunchArgument("follower_frame_prefix", default_value="follower/"),
            DeclareLaunchArgument(
                "follower_usb_port", default_value="/dev/so101_follower"
            ),
            DeclareLaunchArgument(
                "follower_joint_config_file", default_value=default_follower_joint_cfg
            ),
            DeclareLaunchArgument(
                "follower_controller_config_file",
                default_value=default_follower_ctrl_cfg,
            ),
            DeclareLaunchArgument("arm_controller", default_value="forward_controller"),
            DeclareLaunchArgument(
                "cameras_config_file", default_value=default_cameras_cfg
            ),
            # inference knobs
            DeclareLaunchArgument("use_inference", default_value="false"),
            DeclareLaunchArgument("inference_delay_s", default_value="2.0"),
            DeclareLaunchArgument("policy_repo_id", default_value=""),
            DeclareLaunchArgument("policy_device", default_value="cpu"),
            DeclareLaunchArgument("use_rerun", default_value="true"),
            DeclareLaunchArgument(
                "rerun_env_dir",
                # Best: set env var once, no need to pass each run:
                # export SO101_RERUN_ENV_DIR=/abs/path/to/so101-ros-physical-ai
                default_value=EnvironmentVariable(
                    "SO101_RERUN_ENV_DIR", default_value=""
                ),
            ),
            DeclareLaunchArgument("rerun_delay_s", default_value="2.0"),
            follower_launch,
            cameras_launch,
            rerun_start,
            # inference_start,
        ]
    )
