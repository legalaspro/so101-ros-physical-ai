"""
Launch: hardware bringup + MoveItPy motion test.

For visualisation, run the Rerun bridge separately:
  pixi run bridge        # or bridge-3d

Usage:
  ros2 launch so101_bringup moveit_py_test.launch.py
  ros2 launch so101_bringup moveit_py_test.launch.py hardware_type:=mock
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    hardware_type = LaunchConfiguration("hardware_type")
    namespace = LaunchConfiguration("namespace")
    joint_config_file = LaunchConfiguration("joint_config_file")

    use_sim_time = PythonExpression(["'", hardware_type, "' == 'mujoco'"])

    xacro_path = os.path.join(
        get_package_share_directory("so101_description"),
        "urdf",
        "so101_arm.urdf.xacro",
    )

    moveit_config = (
        MoveItConfigsBuilder("so101_arm", package_name="so101_moveit_config")
        .robot_description(
            file_path=xacro_path,
            mappings={"variant": "follower", "use_ros2_control": "false"},
        )
        .robot_description_semantic()
        .robot_description_kinematics()
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .pilz_cartesian_limits(
            file_path="config/pilz_cartesian_limits.yaml"
        )
        .joint_limits()
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .moveit_cpp(
            file_path=os.path.join(
                get_package_share_directory("so101_moveit_config"),
                "config",
                "moveit_py_config.yaml",
            )
        )
        .to_moveit_configs()
    )

    # 1) Hardware bringup (ros2_control + rsp + spawners)
    follower_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("so101_bringup"),
                "launch",
                "follower_split.launch.py",
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "hardware_type": hardware_type,
            "joint_config_file": joint_config_file,
            "use_rviz": "false",
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # 2) Static TF: world → base_link (required by MoveIt's virtual joint)
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    # 3) MoveItPy test node
    #    NOTE: joint_states remapping is done inside the script via MoveItPy's
    #    remappings= kwarg, because MoveItPy creates its own internal C++ node
    #    that doesn't inherit launch-level remappings.
    moveit_py_node = Node(
        name="moveit_py",
        package="so101_moveit_config",
        executable="so101_moveit_test.py",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware_type", default_value="real"),
            DeclareLaunchArgument("namespace", default_value="follower"),
            DeclareLaunchArgument("joint_config_file", default_value=""),
            follower_bringup,
            static_tf,
            moveit_py_node,
        ]
    )

