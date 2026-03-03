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
Thin ROS2 node for SO101 async inference.

Wires ROS2 subscriptions/publishers/timers and delegates all inference
logic to :class:`AsyncInferenceClient`.
"""

from __future__ import annotations

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray

from so101_inference.async_client import AsyncInferenceClient, ClientCfg
from so101_inference.transport.grpc_transport import GrpcTransport
from so101_inference.utils import ros2_image_to_numpy


def _build_lerobot_features(cam_top: str, cam_wrist: str) -> dict:
    """Build the feature spec dict with the given camera names."""
    return {
        "observation.state": {
            "dtype": "float32",
            "shape": (6,),
            "names": [
                "shoulder_pan.pos",
                "shoulder_lift.pos",
                "elbow_flex.pos",
                "wrist_flex.pos",
                "wrist_roll.pos",
                "gripper.pos",
            ],
        },
        f"observation.images.{cam_top}": {
            "dtype": "image",
            "shape": (480, 640, 3),
            "names": ["height", "width", "channels"],
        },
        f"observation.images.{cam_wrist}": {
            "dtype": "image",
            "shape": (480, 640, 3),
            "names": ["height", "width", "channels"],
        },
    }


class AsyncRos2InferenceClient(Node):
    """Thin ROS2 wrapper that delegates inference to ``AsyncInferenceClient``."""

    def __init__(self) -> None:
        super().__init__("async_ros2_inference_client")

        # --------------------
        # Parameters
        # --------------------
        self.declare_parameter("transport_type", "zmq")
        self.declare_parameter("server_address", "127.0.0.1:8090")
        self.declare_parameter("policy_type", "act")
        self.declare_parameter(
            "repo_id",
            "legalaspro/act_so101_pnp_microsanity_20_50hz_v0",
        )
        self.declare_parameter("policy_device", "cuda")
        self.declare_parameter("client_device", "cpu")
        self.declare_parameter("actions_per_chunk", 100)
        self.declare_parameter("chunk_size_threshold", 0.5)
        self.declare_parameter("fps", 50.0)
        self.declare_parameter("max_age_s", 0.2)
        self.declare_parameter("task", "Put the green cube in the cup.")

        self.declare_parameter("fwd_topic", "/follower/forward_controller/commands")
        self.declare_parameter("joints_topic", "/follower/joint_states")
        self.declare_parameter("top_camera_topic", "/static_camera/image_raw")
        self.declare_parameter("wrist_camera_topic", "/follower/image_raw")

        # Camera names as the policy expects them in observation keys
        self.declare_parameter("camera_top_name", "top")
        self.declare_parameter("camera_wrist_name", "wrist")

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

        cfg = ClientCfg(
            server_address=str(self.get_parameter("server_address").value),
            policy_type=str(self.get_parameter("policy_type").value),
            repo_id=str(self.get_parameter("repo_id").value),
            policy_device=str(self.get_parameter("policy_device").value),
            client_device=str(self.get_parameter("client_device").value),
            actions_per_chunk=int(self.get_parameter("actions_per_chunk").value),
            chunk_size_threshold=float(self.get_parameter("chunk_size_threshold").value),
            fps=float(self.get_parameter("fps").value),
            max_age_s=float(self.get_parameter("max_age_s").value),
            task=str(self.get_parameter("task").value),
        )

        self.fwd_topic = str(self.get_parameter("fwd_topic").value)
        self.joints_topic = str(self.get_parameter("joints_topic").value)
        self.top_camera_topic = str(self.get_parameter("top_camera_topic").value)
        self.wrist_camera_topic = str(self.get_parameter("wrist_camera_topic").value)
        self.arm_joints = list(self.get_parameter("arm_joints").value)

        self._cam_top = str(self.get_parameter("camera_top_name").value)
        self._cam_wrist = str(self.get_parameter("camera_wrist_name").value)

        if cfg.fps <= 0:
            self.get_logger().warn(f"Invalid fps={cfg.fps}; forcing 30.0")
            cfg.fps = 30.0

        self.cfg = cfg
        self._log = self.get_logger()

        # --------------------
        # Transport + Client
        # --------------------
        transport_type = str(self.get_parameter("transport_type").value)
        if transport_type == "zmq":
            from so101_inference.transport.zmq_transport import ZmqTransport

            host, port_str = cfg.server_address.rsplit(":", 1)
            zmq_port = int(port_str)
            transport = ZmqTransport(host, port=zmq_port, logger=self.get_logger())
        else:
            transport = GrpcTransport(cfg.server_address, cfg.fps, logger=self.get_logger())

        lerobot_features = _build_lerobot_features(self._cam_top, self._cam_wrist)
        self.client = AsyncInferenceClient(
            transport=transport,
            cfg=cfg,
            lerobot_features=lerobot_features,
            logger=self._log,
        )
        self.client.start()

        # --------------------
        # ROS Runtime state
        # --------------------
        self._latest_top_img: Image | None = None
        self._latest_wrist_img: Image | None = None
        self._latest_joints_msg: JointState | None = None
        self._rx_top = None
        self._rx_wrist = None
        self._rx_joints = None

        self._joint_idx: list[int] | None = None
        self._joint_idx_ready = False
        self._latest_joints_vec: np.ndarray | None = None

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

        period = 1.0 / self.cfg.fps
        self.create_timer(period, self.control_loop)

        # Startup logs
        self._log.info("=" * 60)
        self._log.info("  AsyncRos2InferenceClient READY")
        self._log.info("=" * 60)
        self._log.info(f"  server:             {self.cfg.server_address}")
        self._log.info(f"  policy:             {self.cfg.policy_type} | {self.cfg.repo_id}")
        self._log.info(
            f"  device:             server={self.cfg.policy_device}  client={self.cfg.client_device}"
        )
        self._log.info(f"  cam_top:            {self._cam_top} ({self.top_camera_topic})")
        self._log.info(f"  cam_wrist:          {self._cam_wrist} ({self.wrist_camera_topic})")
        self._log.info(f"  joints_topic:       {self.joints_topic}")
        self._log.info(f"  fwd_topic:          {self.fwd_topic}")
        self._log.info(f"  fps:                {self.cfg.fps:.1f}  (period={period * 1000:.1f}ms)")
        self._log.info(f"  actions/chunk:      {self.cfg.actions_per_chunk}")
        self._log.info(f"  chunk_threshold:    {self.cfg.chunk_size_threshold}")
        self._log.info(f"  max_age_s:          {self.cfg.max_age_s}")
        self._log.info(f"  task:               {self.cfg.task}")
        self._log.info("=" * 60)

    # ---------------------------------
    #   ROS Callbacks
    # ---------------------------------

    def _on_top_image_cb(self, msg: Image):
        self._latest_top_img = msg
        self._rx_top = self.get_clock().now()

    def _on_wrist_image_cb(self, msg: Image):
        self._latest_wrist_img = msg
        self._rx_wrist = self.get_clock().now()

    def _on_joints_cb(self, msg: JointState):
        if not self._joint_idx_ready:
            if not self._initialize_joint_indices(msg):
                return
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
            self._log.error(f"JointState missing joints: {missing}. Available: {list(msg.name)}")
            return False
        self._joint_idx = idx
        self._joint_idx_ready = True
        self._log.info(f"Joint mapping initialized: {self.arm_joints} → indices {idx}")
        return True

    # ---------------------------------
    #   Control Loop Helpers
    # ---------------------------------

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
            age_s(self._rx_top) <= self.cfg.max_age_s
            and age_s(self._rx_wrist) <= self.cfg.max_age_s
            and age_s(self._rx_joints) <= self.cfg.max_age_s
        )

    def _get_data_ages(self) -> dict[str, float]:
        """Return age in ms for each sensor stream."""
        now = self.get_clock().now()
        ages = {}
        if self._rx_top is not None:
            ages[self._cam_top] = (now - self._rx_top).nanoseconds * 1e-6
        if self._rx_wrist is not None:
            ages[self._cam_wrist] = (now - self._rx_wrist).nanoseconds * 1e-6
        if self._rx_joints is not None:
            ages["joints"] = (now - self._rx_joints).nanoseconds * 1e-6
        return ages

    def _build_raw_observation(self) -> dict:
        top_rgb = ros2_image_to_numpy(self._latest_top_img)
        wrist_rgb = ros2_image_to_numpy(self._latest_wrist_img)
        j = self._latest_joints_vec

        joints_str = " ".join(f"{v:+.4f}" for v in j)
        self._log.debug(f"  obs joints: [{joints_str}] | top_img={top_rgb.shape} wrist_img={wrist_rgb.shape}")

        return {
            "shoulder_pan.pos": float(j[0]),
            "shoulder_lift.pos": float(j[1]),
            "elbow_flex.pos": float(j[2]),
            "wrist_flex.pos": float(j[3]),
            "wrist_roll.pos": float(j[4]),
            "gripper.pos": float(j[5]),
            self._cam_top: top_rgb,
            self._cam_wrist: wrist_rgb,
            "task": self.cfg.task,
        }

    # ---------------------------------
    #   Control Loop
    # ---------------------------------

    def control_loop(self):
        loop_start = time.perf_counter()
        self.client.increment_control_loop()

        # Require data
        if not self._data_ready():
            if self.client._control_loop_count % 100 == 0:
                self._log.warn(
                    f"⏳ Waiting for sensor data... "
                    f"top={'✓' if self._latest_top_img else '✗'} "
                    f"wrist={'✓' if self._latest_wrist_img else '✗'} "
                    f"joints={'✓' if self._latest_joints_vec is not None else '✗'}"
                )
            return

        # Require data fresh
        if not self._is_data_fresh():
            ages = self._get_data_ages()
            ages_str = " ".join(f"{k}={v:.0f}ms" for k, v in ages.items())
            self._log.warn(f"⚠️ Stale sensor data (max_age={self.cfg.max_age_s * 1000:.0f}ms) | {ages_str}")
            return

        # (1) execute next action if any
        if self.client.actions_available():
            action_np = self.client.pop_action()
            if action_np is not None:
                msg = Float64MultiArray()
                msg.data = action_np.tolist()
                self.forward_pub.publish(msg)
        else:
            if self.client._control_loop_count % 30 == 0:
                self._log.debug("⏸️ No actions in queue to execute")

        # (2) send observations when queue is "low enough"
        if self.client.ready_to_send():
            ages = self._get_data_ages()
            ages_str = " ".join(f"{k}={v:.0f}ms" for k, v in ages.items())
            self._log.info(f"  sensor ages: {ages_str}")

            raw_obs = self._build_raw_observation()
            self.client.submit_observation(raw_obs)

        # (3) periodic summary
        self.client.maybe_log_summary()

        loop_ms = (time.perf_counter() - loop_start) * 1000
        if loop_ms > (1000.0 / self.cfg.fps) * 1.5:
            self._log.warn(f"⚠️ Control loop SLOW: {loop_ms:.1f}ms (budget={1000.0 / self.cfg.fps:.1f}ms)")

    # ---------------------------------
    #   Shutdown
    # ---------------------------------

    def destroy_node(self):
        self._log.info("🛑 Shutting down AsyncRos2InferenceClient...")
        self.client.stop()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AsyncRos2InferenceClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
