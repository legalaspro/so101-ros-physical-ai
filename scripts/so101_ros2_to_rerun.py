#!/usr/bin/env python3
from __future__ import annotations

import argparse
import threading
from dataclasses import dataclass
from typing import Optional

import numpy as np
import rclpy
import rerun as rr
import rerun.blueprint as rrb
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Image, JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory

# LeRobot-style constants
OBS_STR = "observation"
ACTION_STR = "action"


def stamp_to_datetime64(stamp) -> np.datetime64:
    t = Time.from_msg(stamp)
    return np.datetime64(t.nanoseconds, "ns")


def time_to_datetime64(t: Time) -> np.datetime64:
    return np.datetime64(t.nanoseconds, "ns")


def media_type_from_compressed_format(fmt: str) -> Optional[str]:
    f = (fmt or "").lower()
    # Common ROS compressed formats look like "jpeg", "png" or "jpeg compressed bgr8"
    if "jpeg" in f or "jpg" in f:
        return "image/jpeg"
    if "png" in f:
        return "image/png"
    return None


def rgb8_to_numpy(img: Image) -> np.ndarray:
    arr = np.frombuffer(img.data, dtype=np.uint8)
    return arr.reshape(img.height, img.width, 3)  # RGB


def log_scalar(path: str, value: float) -> None:
    rr.log(path, rr.Scalars(value))


@dataclass
class Topics:
    wrist: str
    overhead: str
    joint_states: str
    forward_commands: Optional[str] = None
    joint_trajectory: Optional[str] = None


class So101Ros2ToRerun(Node):
    def __init__(
        self,
        topics: Topics,
        cmd_joint_order: list[str],
        clear_state_gap_s: float = 2.0,
    ) -> None:
        super().__init__("so101_ros2_to_rerun")
        self._cmd_joint_order = list(cmd_joint_order)
        # Timestamp reference for unstamped messages (Float64MultiArray).
        # Stamped callbacks write here; the action callback reads it.
        # Protected by _time_lock for thread-safety.
        self._time_lock = threading.Lock()
        self._last_ros_time: np.datetime64 | None = None
        self._last_action_time = np.datetime64(0, "ns")
        self._clear_state_gap = np.timedelta64(max(0, int(clear_state_gap_s * 1e9)), "ns")

        # Separate callback groups so heavy-ish callbacks don't block each other.
        self._cg_img_wrist = ReentrantCallbackGroup()
        self._cg_img_over = ReentrantCallbackGroup()
        self._cg_joints = ReentrantCallbackGroup()
        self._cg_cmd = ReentrantCallbackGroup()
        self._cg_traj = ReentrantCallbackGroup()

        if self._is_compressed(topics.wrist):
            self.create_subscription(
                CompressedImage,
                topics.wrist,
                self._on_wrist_img,
                qos_profile_sensor_data,
                callback_group=self._cg_img_wrist,
            )
        else:
            self.create_subscription(
                Image,
                topics.wrist,
                self._on_wrist_img_raw,
                qos_profile_sensor_data,
                callback_group=self._cg_img_wrist,
            )

        if self._is_compressed(topics.overhead):
            self.create_subscription(
                CompressedImage,
                topics.overhead,
                self._on_overhead_img,
                qos_profile_sensor_data,
                callback_group=self._cg_img_over,
            )
        else:
            self.create_subscription(
                Image,
                topics.overhead,
                self._on_overhead_img_raw,
                qos_profile_sensor_data,
                callback_group=self._cg_img_over,
            )

        self.create_subscription(
            JointState,
            topics.joint_states,
            self._on_joint_states,
            qos_profile_sensor_data,
            callback_group=self._cg_joints,
        )

        if topics.forward_commands:
            qos_cmd = QoSProfile(depth=10)
            self.create_subscription(
                Float64MultiArray,
                topics.forward_commands,
                self._on_forward_commands,
                qos_cmd,
                callback_group=self._cg_cmd,
            )

        if topics.joint_trajectory:
            self.create_subscription(
                JointTrajectory,
                topics.joint_trajectory,
                self._on_joint_trajectory,
                qos_profile_sensor_data,
                callback_group=self._cg_traj,
            )

        self.get_logger().info("Rerun bridge started.")
        self.get_logger().info(f"State clear gap threshold: {clear_state_gap_s:.3f}s")

    def _next_action_time(self) -> tuple[np.datetime64, np.datetime64] | None:
        with self._time_lock:
            if self._last_ros_time is None:
                return None
            ts = self._last_ros_time
            prev_action_ts = self._last_action_time
            if ts <= prev_action_ts:
                ts = prev_action_ts + np.timedelta64(1, "ns")
            self._last_action_time = ts
            return ts, prev_action_ts

    def _on_wrist_img(self, msg: CompressedImage) -> None:
        rr.set_time("ros_time", timestamp=stamp_to_datetime64(msg.header.stamp))
        mt = media_type_from_compressed_format(msg.format) or "image/jpeg"
        rr.log(
            "cameras/cam_wrist",
            rr.EncodedImage(contents=bytes(msg.data), media_type=mt),
        )

    def _on_wrist_img_raw(self, img: Image) -> None:
        rr.set_time("ros_time", timestamp=stamp_to_datetime64(img.header.stamp))
        # img_cv = self.cv_bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")  # usually RGB
        # rr.log("cameras/cam_wrist", rr.Image(cv_img, color_model="RGB"))
        rr.log("cameras/cam_wrist", rr.Image(rgb8_to_numpy(img), color_model="RGB"))

    def _on_overhead_img(self, msg: CompressedImage) -> None:
        rr.set_time("ros_time", timestamp=stamp_to_datetime64(msg.header.stamp))
        mt = media_type_from_compressed_format(msg.format) or "image/jpeg"
        rr.log(
            "cameras/cam_overhead",
            rr.EncodedImage(contents=bytes(msg.data), media_type=mt),
        )

    def _on_overhead_img_raw(self, img: Image) -> None:
        rr.set_time("ros_time", timestamp=stamp_to_datetime64(img.header.stamp))
        # img_cv = self.cv_bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
        # rr.log("cameras/cam_overhead", rr.Image(cv_img, color_model="RGB"))
        rr.log("cameras/cam_overhead", rr.Image(rgb8_to_numpy(img), color_model="RGB"))

    def _on_joint_states(self, msg: JointState) -> None:
        ts = stamp_to_datetime64(msg.header.stamp)
        with self._time_lock:
            self._last_ros_time = ts
        rr.set_time("ros_time", timestamp=ts)
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                log_scalar(f"state/position/{name}", float(msg.position[i]))

    def _on_forward_commands(self, msg: Float64MultiArray) -> None:
        # Float64MultiArray has no header stamp. Derive time from the latest
        # stamped ROS message so action/state plots stay aligned.
        action_time = self._next_action_time()
        if action_time is None:
            return
        ts, prev_action_ts = action_time
        rr.set_time("ros_time", timestamp=ts)

        gap_s: float | None = None
        if (
            self._clear_state_gap > np.timedelta64(0, "ns")
            and ts - prev_action_ts > self._clear_state_gap
        ):
            gap_s = float((ts - prev_action_ts) / np.timedelta64(1, "ms")) / 1000.0
            self.get_logger().info(
                f"Clearing state/position after command gap of {gap_s:.3f}s"
            )
            rr.log("action/position", rr.Clear(recursive=True))
            rr.log("state/position", rr.Clear(recursive=True))

        data = list(msg.data)
        if not self._cmd_joint_order:
            # If you didn't pass joint names, log by index.
            for i, v in enumerate(data):
                log_scalar(f"action/forward_commands/idx_{i}", float(v))
            return

        # Controller expects commands in the same order as its configured "joints" list.
        n = min(len(self._cmd_joint_order), len(data))
        for i in range(n):
            jn = self._cmd_joint_order[i]
            log_scalar(f"action/position/{jn}", float(data[i]))

    def _on_joint_trajectory(self, msg: JointTrajectory) -> None:
        rr.set_time("ros_time", timestamp=stamp_to_datetime64(msg.header.stamp))

        if not msg.points:
            return

        # For live viewing, log the first point (the "next commanded setpoint").
        p0 = msg.points[0]
        n = min(len(msg.joint_names), len(p0.positions))
        for i in range(n):
            jn = msg.joint_names[i]
            log_scalar(f"action/trajectory/position/{jn}", float(p0.positions[i]))

    def _is_compressed(self, topic: str) -> bool:
        return topic.endswith("/compressed")


def main() -> None:
    p = argparse.ArgumentParser(description="SO-101 ROS2 to Rerun bridge")
    p.add_argument("--wrist", default="/follower/image_raw/compressed")
    p.add_argument("--overhead", default="/static_camera/image_raw/compressed")
    p.add_argument("--joint-states", default="/follower/joint_states")
    p.add_argument("--forward-commands", default="/follower/forward_controller/commands")
    p.add_argument(
        "--cmd-joints",
        nargs="*",
        default=[
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll",
            "gripper",
        ],
        help="Joint name order matching controller 'joints' param",
    )
    p.add_argument(
        "--joint-trajectory",
        default="",
        help="e.g. /follower/trajectory_controller/joint_trajectory",
    )
    p.add_argument(
        "--clear-state-gap-s",
        type=float,
        default=2.0,
        help="Clear state/position when commands resume after this many seconds; <=0 disables.",
    )
    p.add_argument(
        "--viewer",
        choices=["native", "web"],
        default="web",
        help="Launch web viewer (default) or native desktop app.",
    )
    p.add_argument(
        "--rerun-memory-limit",
        default="512MiB",
        help="Rerun gRPC server memory limit, e.g. 128MiB, 256MiB, 1GiB.",
    )

    args, unknownargs = p.parse_known_args()

    # Initialise Rerun recording.
    rr.init("so101_ros2_live")

    if args.viewer == "native":
        rr.spawn()
    else:
        server_uri = rr.serve_grpc(server_memory_limit=args.rerun_memory_limit)
        rr.serve_web_viewer(connect_to=server_uri)

    # ──  # Blueprint: cameras left, plots right (state + action)
    blueprint = rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(
                rrb.Spatial2DView(name="Wrist Camera", origin="cameras/cam_wrist"),
                rrb.Spatial2DView(name="Overhead Camera", origin="cameras/cam_overhead"),
                row_shares=[1, 1],
            ),
            rrb.Vertical(
                rrb.TimeSeriesView(
                    name="State (Joint Positions)", origin="state/position"
                ),
                rrb.TimeSeriesView(name="Action (Commands)", origin="action"),
                row_shares=[1, 1],
            ),
            column_shares=[1, 1],
        ),
        auto_layout=False,
        auto_views=False,
    )
    rr.send_blueprint(blueprint)

    rclpy.init(args=unknownargs)
    topics = Topics(
        wrist=args.wrist,
        overhead=args.overhead,
        joint_states=args.joint_states,
        forward_commands=args.forward_commands or None,
        joint_trajectory=args.joint_trajectory or None,
    )

    node = So101Ros2ToRerun(
        topics,
        cmd_joint_order=args.cmd_joints,
        clear_state_gap_s=args.clear_state_gap_s,
    )

    exec_ = MultiThreadedExecutor()
    exec_.add_node(node)
    try:
        exec_.spin()
    except KeyboardInterrupt:
        pass
    finally:
        exec_.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
