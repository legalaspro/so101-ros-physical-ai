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

"""
SO101 LeRobot ROS2 Inference Node
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray


import torch
import numpy as np
from copy import copy

from so101_inference.utils import ros2_image_to_numpy

from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.factory import make_pre_post_processors


class LeRobotInferenceNode(Node):
    def __init__(self) -> None:
        super().__init__("lerobot_inference_node")

        # --------------------
        # Parameters
        # --------------------
        self.declare_parameter(
            "repo_id",
            "legalaspro/act_so101_pnp_microsanity_20_50hz_v0",
        )
        self.declare_parameter("fps", 50.0)
        self.declare_parameter("max_age_s", 0.2)

        self.declare_parameter("fwd_topic", "/follower/forward_controller/commands")
        self.declare_parameter("joints_topic", "/follower/joint_states")
        self.declare_parameter("top_camera_topic", "/static_camera/image_raw")
        self.declare_parameter("wrist_camera_topic", "/follower/image_raw")

        self.declare_parameter(
            "arm_joints",
            [
                "shoulder_pan",
                "shoulder_lift",
                "elbow_flex",
                "wrist_flex",
                "wrist_roll",
                "gripper",
            ],
        )

        # Read parameters
        self.repo_id = str(self.get_parameter("repo_id").value)
        self.fps = float(self.get_parameter("fps").value)
        self.max_age_s = float(self.get_parameter("max_age_s").value)

        self.fwd_topic = str(self.get_parameter("fwd_topic").value)
        self.joints_topic = str(self.get_parameter("joints_topic").value)
        self.top_camera_topic = str(self.get_parameter("top_camera_topic").value)
        self.wrist_camera_topic = str(self.get_parameter("wrist_camera_topic").value)

        self.arm_joints = list(self.get_parameter("arm_joints").value)

        if self.fps <= 0:
            self.get_logger().warn(f"Invalid fps={self.fps}; forcing 30.0")
            self.fps = 30.0

        # --------------------
        # Policy Setup
        # --------------------
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"🚀 Using device: {self.device}")
        self.get_logger().info(f"Loading LeRobot policy from repo_id: {self.repo_id}")
        config = PreTrainedConfig.from_pretrained(self.repo_id)
        # config.n_action_steps = 50
        # config.temporal_ensemble_coeff = 0.01
        self.policy = ACTPolicy.from_pretrained(self.repo_id, config=config).to(self.device)
        self.policy.eval()
        self.policy.reset()

        # Load preprocessor/postprocessor (contains normalization stats from training)
        self.preprocessor, self.postprocessor = make_pre_post_processors(
            policy_cfg=self.policy.config,
            pretrained_path=self.repo_id,
            preprocessor_overrides={"device_processor": {"device": str(self.device)}},
            postprocessor_overrides={"device_processor": {"device": "cpu"}},
        )

        # --------------------
        # Runtime state
        # --------------------

        self._latest_top_img: Image | None = None
        self._latest_wrist_img: Image | None = None
        self._latest_joints_msg: JointState | None = None
        self._rx_top = None
        self._rx_wrist = None
        self._rx_joints = None

        # Joint ordering cache
        self._joint_idx: list[int] | None = None
        self._joint_idx_ready = False
        self._latest_joints_vec: np.ndarray | None = None  # ordered float32

        # --------------------
        #  ROS2 Subscribers, Publishers, Timers
        # --------------------
        self.create_subscription(Image, self.top_camera_topic, self._on_top_image_cb, qos_profile_sensor_data)
        self.create_subscription(
            Image,
            self.wrist_camera_topic,
            self._on_wrist_image_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(JointState, self.joints_topic, self._on_joints_cb, qos_profile_sensor_data)

        self.forward_pub = self.create_publisher(Float64MultiArray, self.fwd_topic, 10)

        # TIMER LOOP
        period = 1.0 / self.fps
        self.create_timer(period, self.inference_loop)

        # Startup logs
        self.get_logger().info("LeRobotInferenceNode READY")
        self.get_logger().info(f"  top_camera_topic:   {self.top_camera_topic}")
        self.get_logger().info(f"  wrist_camera_topic: {self.wrist_camera_topic}")
        self.get_logger().info(f"  joints_topic:       {self.joints_topic}")
        self.get_logger().info(f"  fwd_topic:          {self.fwd_topic}")
        self.get_logger().info(f"  fps:                {self.fps:.1f}")
        self.get_logger().info(f"  max_age_s:          {self.max_age_s:.3f}")

    def _on_top_image_cb(self, msg: Image):
        self._latest_top_img = msg
        self._rx_top = self.get_clock().now()

    def _on_wrist_image_cb(self, msg: Image):
        self._latest_wrist_img = msg
        self._rx_wrist = self.get_clock().now()

    def _on_joints_cb(self, msg: JointState):
        # Initialize mapping once (or retry until it works)
        if not self._joint_idx_ready:
            if not self._initialize_joint_indices(msg):
                return

        # Cache ordered joints
        pos = msg.position
        self._latest_joints_vec = np.array([pos[i] for i in self._joint_idx], dtype=np.float32)
        self._latest_joints_msg = msg
        self._rx_joints = self.get_clock().now()

    def _initialize_joint_indices(self, msg: JointState) -> bool:
        name_to_idx = {name: i for i, name in enumerate(msg.name)}

        idx = []
        missing = []
        for j in self.arm_joints:
            if j not in name_to_idx:
                missing.append(j)
            else:
                idx.append(name_to_idx[j])

        if missing:
            self.get_logger().error(f"JointState missing joints: {missing}. Available: {list(msg.name)}")
            return False

        self._joint_idx = idx
        self._joint_idx_ready = True
        self.get_logger().info(f"Initialized joint indices for {len(idx)} joints: {self.arm_joints}")
        return True

    def _data_ready(self) -> bool:
        return (
            self._latest_top_img is not None
            and self._latest_wrist_img is not None
            and self._latest_joints_vec is not None
            and self._rx_top is not None
            and self._rx_wrist is not None
            and self._rx_joints is not None
        )

    def _is_data_fresh(self) -> bool:
        now = self.get_clock().now()

        def age_s(t) -> float:
            return (now - t).nanoseconds * 1e-9

        return (
            age_s(self._rx_top) <= self.max_age_s
            and age_s(self._rx_wrist) <= self.max_age_s
            and age_s(self._rx_joints) <= self.max_age_s
        )

    def _build_observation(self) -> dict:
        """
        Return a dict with raw numpy images (uint8 RGB) and ordered joint state.
        """

        top_rgb = ros2_image_to_numpy(self._latest_top_img)  # HxWx3 uint8 RGB
        wrist_rgb = ros2_image_to_numpy(self._latest_wrist_img)  # HxWx3 uint8 RGB
        return {
            "observation.state": self._latest_joints_vec,  # (6,) float32
            "observation.images.top": top_rgb,
            "observation.images.wrist": wrist_rgb,
        }

    def inference_loop(self):
        # Require data
        if not self._data_ready():
            return
        # Require data fresh
        if not self._is_data_fresh():
            return

        observation = self._build_observation()

        with torch.inference_mode():
            # Convert numpy → tensor, normalize images, add batch dim, move to device
            obs = copy(observation)
            for name in obs:
                obs[name] = torch.from_numpy(obs[name])
                if "image" in name:
                    obs[name] = obs[name].float() / 255.0
                    obs[name] = obs[name].permute(2, 0, 1).contiguous()
                # obs[name] = obs[name].unsqueeze(0).to(self.device) # done in preprocessor

            obs = self.preprocessor(obs)
            action = self.policy.select_action(obs)
            action = self.postprocessor(action)

        # Remove batch dimension and convert to numpy
        action = action.squeeze(0).cpu().numpy()

        msg = Float64MultiArray()
        msg.data = action
        self.forward_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LeRobotInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
