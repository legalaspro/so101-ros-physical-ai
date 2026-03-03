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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # -- Declare all launch arguments with defaults matching ros2_node.py --
    args = [
        DeclareLaunchArgument("transport_type", default_value="zmq"),
        DeclareLaunchArgument("server_address", default_value="127.0.0.1:8090"),
        DeclareLaunchArgument("policy_type", default_value="act"),
        DeclareLaunchArgument("repo_id", default_value="legalaspro/act_so101_pnp_microsanity_20_50hz_v0"),
        DeclareLaunchArgument("policy_device", default_value="cuda"),
        DeclareLaunchArgument("client_device", default_value="cpu"),
        DeclareLaunchArgument("actions_per_chunk", default_value="100"),
        DeclareLaunchArgument("chunk_size_threshold", default_value="0.5"),
        DeclareLaunchArgument("fps", default_value="50.0"),
        DeclareLaunchArgument("max_age_s", default_value="0.2"),
        DeclareLaunchArgument("task", default_value="Put the green cube in the cup."),
        DeclareLaunchArgument("aggregate_fn_name", default_value="weighted_average"),
        # Topics
        DeclareLaunchArgument("fwd_topic", default_value="/follower/forward_controller/commands"),
        DeclareLaunchArgument("joints_topic", default_value="/follower/joint_states"),
        DeclareLaunchArgument("top_camera_topic", default_value="/static_camera/image_raw"),
        DeclareLaunchArgument("wrist_camera_topic", default_value="/follower/image_raw"),
        # Camera names as the policy expects them in observation keys
        DeclareLaunchArgument("camera_top_name", default_value="top"),
        DeclareLaunchArgument("camera_wrist_name", default_value="wrist"),
    ]

    node = Node(
        package="so101_inference",
        executable="async_inference_node",
        name="async_ros2_inference_client",
        parameters=[
            {
                "transport_type": LaunchConfiguration("transport_type"),
                "server_address": LaunchConfiguration("server_address"),
                "policy_type": LaunchConfiguration("policy_type"),
                "repo_id": LaunchConfiguration("repo_id"),
                "policy_device": LaunchConfiguration("policy_device"),
                "client_device": LaunchConfiguration("client_device"),
                "actions_per_chunk": LaunchConfiguration("actions_per_chunk"),
                "chunk_size_threshold": LaunchConfiguration("chunk_size_threshold"),
                "fps": LaunchConfiguration("fps"),
                "max_age_s": LaunchConfiguration("max_age_s"),
                "task": LaunchConfiguration("task"),
                "aggregate_fn_name": LaunchConfiguration("aggregate_fn_name"),
                "fwd_topic": LaunchConfiguration("fwd_topic"),
                "joints_topic": LaunchConfiguration("joints_topic"),
                "top_camera_topic": LaunchConfiguration("top_camera_topic"),
                "wrist_camera_topic": LaunchConfiguration("wrist_camera_topic"),
                "camera_top_name": LaunchConfiguration("camera_top_name"),
                "camera_wrist_name": LaunchConfiguration("camera_wrist_name"),
            }
        ],
        output="screen",
    )

    return LaunchDescription(args + [node])
