#!/usr/bin/env python3
"""SO-101 ROS 2 → Rerun bridge: single 3-D view with URDF, TF & camera frustums."""
from __future__ import annotations

import argparse
import re
import threading
from dataclasses import dataclass, field
from functools import partial
from typing import Optional

import numpy as np
import rclpy
import rerun as rr
import rerun.blueprint as rrb
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Float64MultiArray, String
from tf2_msgs.msg import TFMessage
import xml.etree.ElementTree as ET


def stamp_to_datetime64(stamp) -> np.datetime64:
    t = Time.from_msg(stamp)
    return np.datetime64(t.nanoseconds, "ns")


def time_to_datetime64(t: Time) -> np.datetime64:
    return np.datetime64(t.nanoseconds, "ns")


def media_type_from_compressed_format(fmt: str) -> Optional[str]:
    f = (fmt or "").lower()
    if "jpeg" in f or "jpg" in f:
        return "image/jpeg"
    if "png" in f:
        return "image/png"
    return None

def log_scalar(path: str, value: float) -> None:
    rr.log(path, rr.Scalars(value))


@dataclass
class CameraCfg:
    """Per-camera configuration."""
    name: str               # short label, e.g. "wrist"
    image_topic: str        # ROS compressed image topic
    tf_frame: str           # TF frame name the camera lives in
    width: int = 640        # default resolution
    height: int = 480
    focal_length: float = 300.0  # default focal length px
    image_plane_distance: float = 0.05  # default image plane distance (m)


@dataclass
class Topics:
    cameras: list[CameraCfg] = field(default_factory=list)
    joint_states: str = "/follower/joint_states"
    robot_description: str = "/follower/robot_description"
    tf: str = "/tf"
    tf_static: str = "/tf_static"
    forward_commands: Optional[str] = None


class So101Ros2ToRerun3DBridge(Node):
    """ROS 2 node that bridges sensor data + URDF + TF into Rerun (single 3D view)."""

    def __init__(
        self,
        topics: Topics,
        cmd_joint_order: list[str],
        tf_prefix: str = "follower/",
        tf_root_frame: str = "world",
        clear_state_gap_s: float = 2.0,
    ) -> None:
        super().__init__("so101_ros2_rerun_3d_bridge")
        self._cmd_joint_order = list(cmd_joint_order)
        self._tf_prefix = tf_prefix
        self._tf_root_frame = tf_root_frame
        self._time_lock = threading.Lock()
        self._last_ros_time: np.datetime64 | None = None
        self._last_action_time = np.datetime64(0, "ns")
        self._clear_state_gap = np.timedelta64(max(0, int(clear_state_gap_s * 1e9)), "ns")
        self._camera_cfgs: dict[str, CameraCfg] = {}  # name → cfg

        # Shared callback groups
        self._cg_joints = ReentrantCallbackGroup()
        self._cg_cmd = ReentrantCallbackGroup()
        self._cg_urdf = ReentrantCallbackGroup()
        self._cg_tf = ReentrantCallbackGroup()

        # ── Camera subscriptions (dynamic, based on CameraCfg list) ──
        for cam in topics.cameras:
            self._camera_cfgs[cam.name] = cam
            cg = ReentrantCallbackGroup()
            # Image subscription (compressed only)
            self.create_subscription(
                CompressedImage, cam.image_topic,
                partial(self._on_camera_compressed, cam_name=cam.name),
                qos_profile_sensor_data, callback_group=cg)
            # Static pinhole frustum tied to camera's TF frame
            # rr.log(
            #     f"cameras/{cam.name}/camera_info",
            #     rr.Pinhole(
            #         focal_length=cam.focal_length,
            #         width=cam.width,
            #         height=cam.height,
            #         camera_xyz=rr.ViewCoordinates.FLU,
            #         image_plane_distance=cam.image_plane_distance,
            #         parent_frame=cam.tf_frame,
            #         child_frame=cam.tf_frame + "_image_plane",
            #     ),
            #     static=True,
            # )
            self.get_logger().info(
                f"Camera '{cam.name}': image={cam.image_topic}  tf_frame={cam.tf_frame}")

        # ── Joint states ──
        self.create_subscription(JointState, topics.joint_states, self._on_joint_states,
                                 qos_profile_sensor_data, callback_group=self._cg_joints)

        # ── URDF (robot_description) ──
        qos_urdf = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(String, topics.robot_description, self._on_robot_description,
                                 qos_urdf, callback_group=self._cg_urdf)

        # ── TF ──
        self.create_subscription(TFMessage, topics.tf, self._on_tf,
                                 qos_profile_sensor_data, callback_group=self._cg_tf)
        qos_tf_static = QoSProfile(
            depth=100,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(TFMessage, topics.tf_static, self._on_tf_static,
                                 qos_tf_static, callback_group=self._cg_tf)

        # ── Forward commands ──
        if topics.forward_commands:
            qos_cmd = QoSProfile(depth=10)
            self.create_subscription(Float64MultiArray, topics.forward_commands,
                                     self._on_forward_commands, qos_cmd, callback_group=self._cg_cmd)

        self.get_logger().info("Rerun 3D bridge started.")

    # ── helpers ──────────────────────────────────────────────────────
    def _next_action_time(self):
        with self._time_lock:
            if self._last_ros_time is None:
                return None
            ts = self._last_ros_time
            prev = self._last_action_time
            if ts <= prev:
                ts = prev + np.timedelta64(1, "ns")
            self._last_action_time = ts
            return ts, prev

    # ── URDF helpers ───────────────────────────────────────────────
    @staticmethod
    def _prefix_urdf_frames(urdf_xml: str, prefix: str) -> str:
        """Add *prefix* to every link and joint frame reference in the URDF XML.

        This makes the URDF link names (e.g. ``base_link``) match the TF frame
        names published by ``robot_state_publisher`` when it uses the same
        ``frame_prefix`` (e.g. ``follower/base_link``).
        """
        if not prefix:
            return urdf_xml
        # Prefix link names in <link name="...">, <parent link="...">, <child link="...">
        urdf_xml = re.sub(
            r'(<(?:link|parent|child)\s+(?:[^>]*?\s)?(?:name|link)\s*=\s*")([^"]+)(")',
            lambda m: m.group(1) + prefix + m.group(2) + m.group(3),
            urdf_xml,
        )
        return urdf_xml

    # ── URDF callback ────────────────────────────────────────────────
    def _on_robot_description(self, msg: String) -> None:
        self.get_logger().info("Received robot_description – logging URDF to Rerun")
        urdf_xml = msg.data
        # Prefix URDF link names so they match TF frame names (e.g. follower/base_link)
        urdf_xml = self._prefix_urdf_frames(urdf_xml, self._tf_prefix)
        rr.log_file_from_contents(
            file_path="robot.urdf",
            file_contents=urdf_xml.encode("utf-8"),
            entity_path_prefix="urdf",
            static=True,
        )
        # Log coordinate axes + labels for every link in the URDF
        self._log_axes_from_urdf(urdf_xml)

    def _log_axes_from_urdf(self, urdf_xml: str) -> None:
        """Log small coordinate axes attached to each URDF link via TF frames."""
        try:
            root = ET.fromstring(urdf_xml)
        except ET.ParseError as e:
            self.get_logger().warn(f"Could not parse URDF for axes: {e}")
            return
        link_names: set[str] = set()
        for joint in root.findall("joint"):
            parent = joint.find("parent")
            child = joint.find("child")
            if parent is not None and parent.get("link"):
                link_names.add(parent.get("link"))
            if child is not None and child.get("link"):
                link_names.add(child.get("link"))
        prefix = self._tf_prefix
        ee_frame = prefix + "gripper_frame_link"
        for link_name in sorted(link_names):
            rr.log(f"axes/{link_name}",
                   rr.CoordinateFrame(frame=link_name),
                   rr.TransformAxes3D(0.06), static=True)
            is_ee = link_name == ee_frame
            short_name = link_name.removeprefix(prefix)
            rr.log(f"axes/{link_name}/label",
                   rr.CoordinateFrame(frame=link_name),
                   rr.Points3D([[0, 0, 0]],
                                radii=0.01 if is_ee else 0.004,
                                labels=[short_name],
                                colors=[[0, 255, 0]] if is_ee else [[255, 255, 255]]),
                    static=True)

    # ── TF callbacks ─────────────────────────────────────────────────
    def _log_tf_transforms(self, tf_msg: TFMessage) -> None:
        for transform in tf_msg.transforms:
            time_ns = Time.from_msg(transform.header.stamp)
            rr.set_time("ros_time", timestamp=np.datetime64(time_ns.nanoseconds, "ns"))
            t = transform.transform.translation
            r = transform.transform.rotation
            rr.log(
                "transforms",
                rr.Transform3D(
                    translation=[t.x, t.y, t.z],
                    rotation=rr.Quaternion(xyzw=[r.x, r.y, r.z, r.w]),
                    parent_frame=transform.header.frame_id,
                    child_frame=transform.child_frame_id,
                ),
            )

    def _on_tf(self, msg: TFMessage) -> None:
        self._log_tf_transforms(msg)

    def _on_tf_static(self, msg: TFMessage) -> None:
        self._log_tf_transforms(msg)

    # ── Camera callbacks (generic, bound via functools.partial) ───────
    def _on_camera_compressed(self, msg: CompressedImage, *, cam_name: str) -> None:
        rr.set_time("ros_time", timestamp=stamp_to_datetime64(msg.header.stamp))
        mt = media_type_from_compressed_format(msg.format) or "image/jpeg"
        cam = self._camera_cfgs[cam_name]
        rr.log(f"cameras/{cam_name}/image",
               rr.EncodedImage(contents=bytes(msg.data), media_type=mt))
        # Image plane frame must match Pinhole's child_frame
        # rr.log(f"cameras/{cam_name}/image",
        #        rr.CoordinateFrame(frame=cam.tf_frame + "_image_plane"))

    # ── Joint-state callback ─────────────────────────────────────────
    def _on_joint_states(self, msg: JointState) -> None:
        ts = stamp_to_datetime64(msg.header.stamp)
        with self._time_lock:
            self._last_ros_time = ts
        rr.set_time("ros_time", timestamp=ts)
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                log_scalar(f"state/position/{name}", float(msg.position[i]))

    # ── Forward commands callback ────────────────────────────────────
    def _on_forward_commands(self, msg: Float64MultiArray) -> None:
        action_time = self._next_action_time()
        if action_time is None:
            return
        ts, prev_action_ts = action_time
        rr.set_time("ros_time", timestamp=ts)

        if (
            self._clear_state_gap > np.timedelta64(0, "ns")
            and ts - prev_action_ts > self._clear_state_gap
        ):
            rr.log("action/position", rr.Clear(recursive=True))
            rr.log("state/position", rr.Clear(recursive=True))

        data = list(msg.data)
        if not self._cmd_joint_order:
            for i, v in enumerate(data):
                log_scalar(f"action/forward_commands/idx_{i}", float(v))
            return

        n = min(len(self._cmd_joint_order), len(data))
        for i in range(n):
            log_scalar(f"action/position/{self._cmd_joint_order[i]}", float(data[i]))



# ═════════════════════════════════════════════════════════════════════
# main
# ═════════════════════════════════════════════════════════════════════

def main() -> None:
    p = argparse.ArgumentParser(description="SO-101 ROS2→Rerun 3D bridge")
    # Camera args
    p.add_argument("--wrist-image", default="/follower/image_raw/compressed",
                   help="Wrist camera image topic")
    p.add_argument("--wrist-tf-frame", default="follower/wrist_camera_link",
                   help="TF frame name for wrist camera")
    p.add_argument("--overhead-image", default="/static_camera/image_raw/compressed",
                   help="Overhead camera image topic")
    p.add_argument("--overhead-tf-frame", default="follower/static_camera_link",
                   help="TF frame name for overhead camera")
    # Other topics
    p.add_argument("--joint-states", default="/follower/joint_states")
    p.add_argument("--robot-description", default="/follower/robot_description",
                   help="Topic publishing the URDF XML string")
    p.add_argument("--tf", default="/tf")
    p.add_argument("--tf-static", default="/tf_static")
    p.add_argument("--forward-commands", default="/follower/forward_controller/commands")
    p.add_argument("--cmd-joints", nargs="*", default=[
        "shoulder_pan", "shoulder_lift", "elbow_flex",
        "wrist_flex", "wrist_roll", "gripper",
    ], help="Joint name order matching controller 'joints' param")
    p.add_argument("--tf-prefix", default="follower/",
                   help="TF frame prefix used by robot_state_publisher. Use '' for MoveIt/follower_split.")
    p.add_argument("--tf-root-frame", default="world",
                   help="Root frame of the TF tree")
    p.add_argument("--clear-state-gap-s", type=float, default=2.0)
    args, unknownargs = p.parse_known_args()

    # ── Rerun init (always web) ──
    rr.init("so101_ros2_3d")
    server_uri = rr.serve_grpc()
    rr.serve_web_viewer(connect_to=server_uri)

    # Connect the Rerun view root "/" to the TF tree root frame
    rr.log("/", rr.CoordinateFrame(frame=args.tf_root_frame), static=True)

    # ── Build camera configs ──
    cameras = [
        CameraCfg(name="cam_wrist", image_topic=args.wrist_image,
                  tf_frame=args.wrist_tf_frame),
        CameraCfg(name="cam_overhead", image_topic=args.overhead_image,
                  tf_frame=args.overhead_tf_frame),
    ]

    # ── Blueprint: single 3D view + camera 2D views + time-series plots ──
    blueprint = rrb.Blueprint(
        rrb.Vertical(
            rrb.Horizontal(
                rrb.Spatial3DView(name="3D Scene", origin="/"),
                rrb.Vertical(
                    rrb.Spatial2DView(name="Wrist", origin="cameras/cam_wrist/image"),
                    rrb.Spatial2DView(name="Overhead", origin="cameras/cam_overhead/image"),
                    row_shares=[1, 1],
                ),
                column_shares=[2, 1],
            ),
            rrb.Horizontal(
                rrb.TimeSeriesView(name="State (Joint Positions)", origin="state/position"),
                rrb.TimeSeriesView(name="Action (Commands)", origin="action"),
                column_shares=[1, 1],
            ),
            row_shares=[3, 1],
        ),
        auto_layout=False,
        auto_views=False,
    )
    rr.send_blueprint(blueprint)

    # ── ROS 2 init ──
    rclpy.init(args=unknownargs)
    topics = Topics(
        cameras=cameras,
        joint_states=args.joint_states,
        robot_description=args.robot_description,
        tf=args.tf,
        tf_static=args.tf_static,
        forward_commands=args.forward_commands or None,
    )

    node = So101Ros2ToRerun3DBridge(
        topics,
        cmd_joint_order=args.cmd_joints,
        tf_prefix=args.tf_prefix,
        tf_root_frame=args.tf_root_frame,
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
