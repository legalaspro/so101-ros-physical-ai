# Copyright 2026 Dmitri Manajev
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch handeye_calibration_node + cartesian_motion_node.

handeye_calibration_node detects a ChArUco board, publishes the
board→camera TF, and provides a Viser UI with:
- IK gizmo (Manual EE mode) → publishes to /servo_target
- Hand-eye calibration buttons (Take Sample, Compute, Save)
  using cv2.calibrateHandEye directly.

cartesian_motion_node performs IK when the gizmo is dragged.

Assumes the follower arm stack (ros2_control + controllers) is
already running, e.g. via follower_vision.launch.py.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # ── Launch arguments ──
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    joint_states_topic = LaunchConfiguration("joint_states_topic")
    cmd_topic = LaunchConfiguration("cmd_topic")
    base_frame = LaunchConfiguration("base_frame")
    servo_target_topic = LaunchConfiguration("servo_target_topic")
    robot_effector_frame = LaunchConfiguration("robot_effector_frame")
    calibration_poses_file = LaunchConfiguration("calibration_poses_file")

    # ── Nodes ──
    handeye_node = Node(
        package="so101_camera_calibration",
        executable="handeye_calibration_node",
        name="handeye_calibration_node",
        output="screen",
        parameters=[{
            "image_topic": image_topic,
            "camera_info_topic": camera_info_topic,
            "joint_states_topic": joint_states_topic,
            "base_frame": base_frame,
            "robot_effector_frame": robot_effector_frame,
            "servo_target_topic": servo_target_topic,
            "calibration_poses_file": calibration_poses_file,
        }],
    )

    cartesian_motion_node = Node(
        package="so101_kinematics",
        executable="cartesian_motion_node",
        name="cartesian_motion_node",
        output="screen",
        parameters=[{
            "joints_topic": joint_states_topic,
            "cmd_topic": cmd_topic,
            "base_frame": base_frame,
        }],
        remappings=[
            ("servo_target", servo_target_topic),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "image_topic",
            default_value="/static_camera/image_raw",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="/static_camera/camera_info",
        ),
        DeclareLaunchArgument(
            "joint_states_topic",
            default_value="/follower/joint_states",
        ),
        DeclareLaunchArgument(
            "cmd_topic",
            default_value="/follower/forward_controller/commands",
        ),
        DeclareLaunchArgument(
            "base_frame",
            default_value="follower/base_link",
        ),
        DeclareLaunchArgument(
            "robot_effector_frame",
            default_value="follower/gripper_frame_link",
        ),
        DeclareLaunchArgument(
            "servo_target_topic",
            default_value="/servo_target",
        ),
        DeclareLaunchArgument(
            "calibration_poses_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("so101_camera_calibration"),
                "config",
                "calibration_poses.yaml",
            ]),
            description="YAML file with joint poses for auto-calibration",
        ),
        handeye_node,
        cartesian_motion_node,
    ])
