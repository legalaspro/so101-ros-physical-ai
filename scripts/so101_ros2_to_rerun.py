#!/usr/bin/env python3
from __future__ import annotations

import argparse
from dataclasses import dataclass
from typing import Optional

import numpy as np
import rclpy
import rerun as rr
import rerun.blueprint as rrb
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, JointState, Image
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
    def __init__(self, topics: Topics, cmd_joint_order: list[str]) -> None:
        super().__init__("so101_ros2_to_rerun")
        self._cmd_joint_order = list(cmd_joint_order)

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
                callback_group=self._cg_img_wrist)
        else:
            self.create_subscription(
                Image, 
                topics.wrist, 
                self._on_wrist_img_raw, 
                qos_profile_sensor_data,
                callback_group=self._cg_img_wrist)
        
        if self._is_compressed(topics.overhead):    
            self.create_subscription(
                CompressedImage,
                topics.overhead,
                self._on_overhead_img,
                qos_profile_sensor_data,
                callback_group=self._cg_img_over)
        else:
            self.create_subscription(
                Image, 
                topics.overhead, 
                self._on_overhead_img_raw, 
                qos_profile_sensor_data,
                callback_group=self._cg_img_over)
        

        self.create_subscription(
            JointState,
            topics.joint_states,
            self._on_joint_states,
            qos_profile_sensor_data,
            callback_group=self._cg_joints)

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
        rr.set_time("ros_time", timestamp=stamp_to_datetime64(msg.header.stamp))
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                log_scalar(f"state/position/{name}", float(msg.position[i]))
    
    def _on_forward_commands(self, msg: Float64MultiArray) -> None:
        # Float64MultiArray has no header stamp. Use node clock (ROS time if use_sim_time=true).
        now = self.get_clock().now()
        rr.set_time("ros_time", timestamp=time_to_datetime64(now))

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
        default=["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"],
        help="Joint name order matching controller 'joints' param",
    )   
    p.add_argument("--joint-trajectory", default="", help="e.g. /follower/trajectory_controller/joint_trajectory")

    args, unknownargs = p.parse_known_args()

    # Initialise Rerun recording (no sinks yet — just buffering).
    rr.init("so101_ros2_live")

    # Start gRPC data server, then web viewer that connects to it.
    # This is the official pattern from the Rerun docs:
    #   https://ref.rerun.io/docs/python/0.29.1/common/initialization_functions/#rerun.serve_web_viewer
    server_uri = rr.serve_grpc()
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
                rrb.TimeSeriesView(name="State (Joint Positions)", origin="state/position"),
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

    node = So101Ros2ToRerun(topics, cmd_joint_order=args.cmd_joints)

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
