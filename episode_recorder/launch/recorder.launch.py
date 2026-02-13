"""Headless launch: EpisodeRecorder lifecycle node auto-configure + auto-activate."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
import lifecycle_msgs.msg


def generate_launch_description():

    default_params = PathJoinSubstitution(
        [FindPackageShare("episode_recorder"), "config", "default_config.yaml"]
    )
    default_root_dir =  PathJoinSubstitution([
        EnvironmentVariable("ROS_HOME", default_value=PathJoinSubstitution([EnvironmentVariable("HOME"), ".ros"])),
        "so101_episodes",
    ])

    # --- Launch arguments ---
    params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="YAML with episode_recorder parameters (topics, storage_id, etc.)",
    )

    root_dir = DeclareLaunchArgument(
        "root_dir",
        default_value=default_root_dir,
        description="Root directory for episodes",
    )

    experiment_name = DeclareLaunchArgument(
        "experiment_name",
        default_value="",
        description="Optional subfolder under root_dir (handled by the recorder node)",
    )

    recorder_ns = DeclareLaunchArgument(
        "recorder_ns",
        default_value="",
        description="Namespace for recorder node (optional)",
    )


    recorder = LifecycleNode(
        package="episode_recorder",
        executable="episode_recorder_node",
        name="episode_recorder",
        namespace=LaunchConfiguration("recorder_ns"),
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "root_dir": LaunchConfiguration("root_dir"),
                "experiment_name": LaunchConfiguration("experiment_name"),
            },
        ],
    )

    # Configure once the process starts (avoids race)
    configure_on_start = RegisterEventHandler(
        OnProcessStart(
            target_action=recorder,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(recorder),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )
                )
            ],
        )
    )

    # Activate after it reaches INACTIVE
    activate_after_configure = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=recorder,
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(recorder),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )


    return LaunchDescription(
        [
            params_file,
            root_dir,
            experiment_name,
            recorder_ns,
            recorder,
            configure_on_start,
            activate_after_configure,
        ]
    )
