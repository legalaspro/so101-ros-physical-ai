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
    args = [
        DeclareLaunchArgument("repo_id", default_value="legalaspro/act_so101_pnp_microsanity_20_50hz_v0"),
        DeclareLaunchArgument("fps", default_value="50.0"),
        DeclareLaunchArgument("max_age_s", default_value="0.2"),
        # Topics
        DeclareLaunchArgument("fwd_topic", default_value="/follower/forward_controller/commands"),
        DeclareLaunchArgument("joints_topic", default_value="/follower/joint_states"),
        DeclareLaunchArgument("top_camera_topic", default_value="/static_camera/image_raw"),
        DeclareLaunchArgument("wrist_camera_topic", default_value="/follower/image_raw"),
    ]

    node = Node(
        package="so101_inference",
        executable="lerobot_inference_node",
        name="lerobot_inference_node",
        parameters=[
            {
                "repo_id": LaunchConfiguration("repo_id"),
                "fps": LaunchConfiguration("fps"),
                "max_age_s": LaunchConfiguration("max_age_s"),
                "fwd_topic": LaunchConfiguration("fwd_topic"),
                "joints_topic": LaunchConfiguration("joints_topic"),
                "top_camera_topic": LaunchConfiguration("top_camera_topic"),
                "wrist_camera_topic": LaunchConfiguration("wrist_camera_topic"),
            }
        ],
        output="screen",
    )

    return LaunchDescription(args + [node])
