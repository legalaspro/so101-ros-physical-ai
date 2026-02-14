#!/usr/bin/env python3
"""SO-101 Episode Browser — Rerun + ROS2 bag playback + Gradio UI."""

from __future__ import annotations

import argparse
import atexit
import signal
import subprocess
import sys
import threading
from dataclasses import dataclass
from pathlib import Path

import gradio as gr
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import CompressedImage, Image, JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory
import rerun as rr
from rclpy.time import Time

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def media_type_from_compressed_format(fmt: str) -> str | None:
    f = (fmt or "").lower()
    if "jpeg" in f or "jpg" in f:
        return "image/jpeg"
    if "png" in f:
        return "image/png"
    return None


def rgb8_to_numpy(img: Image) -> np.ndarray:
    return np.frombuffer(img.data, dtype=np.uint8).reshape(img.height, img.width, 3)


def stamp_to_datetime64(stamp) -> np.datetime64:
    t = Time.from_msg(stamp)
    return np.datetime64(t.nanoseconds, "ns")


def time_to_datetime64(t: Time) -> np.datetime64:
    return np.datetime64(t.nanoseconds, "ns")


# ---------------------------------------------------------------------------
# Dataclasses
# ---------------------------------------------------------------------------

@dataclass
class Topics:
    wrist: str = "/follower/image_raw"
    overhead: str = "/static_camera/image_raw"
    joint_states: str = "/follower/joint_states"
    forward_commands: str | None = "/follower/forward_controller/commands"
    joint_trajectory: str | None = None


DEFAULT_CMD_JOINTS: list[str] = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


# ---------------------------------------------------------------------------
# Rerun blueprint
# ---------------------------------------------------------------------------

def make_blueprint() -> rr.blueprint.Blueprint:
    return rr.blueprint.Blueprint(
        rr.blueprint.Horizontal(
            rr.blueprint.Vertical(
                rr.blueprint.Spatial2DView(name="Wrist", origin="cameras/wrist"),
                rr.blueprint.Spatial2DView(name="Overhead", origin="cameras/overhead"),
            ),
            rr.blueprint.Vertical(
                rr.blueprint.TimeSeriesView(name="Joint States", origin="state"),
                rr.blueprint.TimeSeriesView(name="Actions", origin="action"),
            ),
        ),
        collapse_panels=True,
    )


blueprint = make_blueprint()
GRPC_URI: str = ""


# ---------------------------------------------------------------------------
# ROS2 Node — So101Ros2ToRerun
# ---------------------------------------------------------------------------

class So101Ros2ToRerun(Node):
    """Subscribe to SO-101 topics and log data to Rerun."""

    def __init__(self, topics: Topics, cmd_joint_order: list[str]) -> None:
        super().__init__("so101_ros2_to_rerun")
        self._cmd_joint_order = list(cmd_joint_order)
        self._cg_img_wrist = ReentrantCallbackGroup()
        self._cg_img_over = ReentrantCallbackGroup()
        self._cg_joints = ReentrantCallbackGroup()
        self._cg_cmd = ReentrantCallbackGroup()
        self._cg_traj = ReentrantCallbackGroup()

        if self._is_compressed(topics.wrist):
            self.create_subscription(
                CompressedImage, topics.wrist, self._on_wrist_img,
                qos_profile_sensor_data, callback_group=self._cg_img_wrist,
            )
        else:
            self.create_subscription(
                Image, topics.wrist, self._on_wrist_img_raw,
                qos_profile_sensor_data, callback_group=self._cg_img_wrist,
            )

        if self._is_compressed(topics.overhead):
            self.create_subscription(
                CompressedImage, topics.overhead, self._on_overhead_img,
                qos_profile_sensor_data, callback_group=self._cg_img_over,
            )
        else:
            self.create_subscription(
                Image, topics.overhead, self._on_overhead_img_raw,
                qos_profile_sensor_data, callback_group=self._cg_img_over,
            )

        self.create_subscription(
            JointState, topics.joint_states, self._on_joint_states,
            qos_profile_sensor_data, callback_group=self._cg_joints,
        )

        if topics.forward_commands:
            qos_cmd = QoSProfile(depth=10)
            self.create_subscription(
                Float64MultiArray, topics.forward_commands, self._on_forward_commands,
                qos_cmd, callback_group=self._cg_cmd,
            )

        if topics.joint_trajectory:
            self.create_subscription(
                JointTrajectory, topics.joint_trajectory, self._on_joint_trajectory,
                qos_profile_sensor_data, callback_group=self._cg_traj,
            )

        self.get_logger().info("Rerun bridge started.")

    # -- helpers --
    @staticmethod
    def _is_compressed(topic_name: str) -> bool:
        return "compressed" in topic_name.lower()

    @staticmethod
    def _stamp_secs(stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9

    # -- image callbacks --
    def _on_wrist_img(self, msg: CompressedImage) -> None:
        rr.set_time("ros_time", timestamp=stamp_to_datetime64(msg.header.stamp))
        mt = media_type_from_compressed_format(msg.format) or "image/jpeg"
        rr.log("cameras/wrist", rr.EncodedImage(contents=bytes(msg.data), media_type=mt))

    def _on_wrist_img_raw(self, msg: Image) -> None:
        rr.set_time("ros_time", timestamp=stamp_to_datetime64(msg.header.stamp))
        rr.log("cameras/wrist", rr.Image(rgb8_to_numpy(msg), color_model="RGB"))

    def _on_overhead_img(self, msg: CompressedImage) -> None:
        rr.set_time("ros_time", timestamp=stamp_to_datetime64(msg.header.stamp))
        mt = media_type_from_compressed_format(msg.format) or "image/jpeg"
        rr.log("cameras/overhead", rr.EncodedImage(contents=bytes(msg.data), media_type=mt))

    def _on_overhead_img_raw(self, msg: Image) -> None:
        rr.set_time("ros_time", timestamp=stamp_to_datetime64(msg.header.stamp))
        rr.log("cameras/overhead", rr.Image(rgb8_to_numpy(msg), color_model="RGB"))

    # -- joint state callback --
    def _on_joint_states(self, msg: JointState) -> None:
        rr.set_time("ros_time", timestamp=stamp_to_datetime64(msg.header.stamp))
        for name, pos in zip(msg.name, msg.position):
            rr.log(f"state/position/{name}", rr.Scalars(float(pos)))

    # -- forward commands callback --
    def _on_forward_commands(self, msg: Float64MultiArray) -> None:
        now = self.get_clock().now()
        rr.set_time("ros_time", timestamp=time_to_datetime64(now))
        for name, val in zip(self._cmd_joint_order, msg.data):
            rr.log(f"action/position/{name}", rr.Scalars(float(val)))

    # -- joint trajectory callback --
    def _on_joint_trajectory(self, msg: JointTrajectory) -> None:
        rr.set_time("ros_time", timestamp=stamp_to_datetime64(msg.header.stamp))
        if msg.points:
            pt = msg.points[0]
            for name, pos in zip(msg.joint_names, pt.positions):
                rr.log(f"action/position/{name}", rr.Scalars(float(pos)))


# ---------------------------------------------------------------------------
# Episode indexing
# ---------------------------------------------------------------------------

def index_episodes(root: Path) -> list[tuple[str, str]]:
    """Find all *.mcap files under *root*, return [(label, path_str), ...]."""
    mcaps = sorted(root.rglob("*.mcap"))
    choices: list[tuple[str, str]] = []
    for p in mcaps:
        label = str(p.relative_to(root))
        choices.append((label, str(p)))
    return choices


# ---------------------------------------------------------------------------
# Bag playback management
# ---------------------------------------------------------------------------

BAG_PROC: subprocess.Popen | None = None
BAG_LOCK = threading.Lock()


def stop_playback() -> None:
    global BAG_PROC
    with BAG_LOCK:
        if BAG_PROC is not None:
            try:
                BAG_PROC.send_signal(signal.SIGINT)
                BAG_PROC.wait(timeout=3)
            except (subprocess.TimeoutExpired, OSError):
                try:
                    BAG_PROC.kill()
                    BAG_PROC.wait(timeout=2)
                except OSError:
                    pass
            BAG_PROC = None


def start_playback(
    target: str,
    rate: float = 1.0,
    clock_hz: int = 100,
    loop: bool = False,
    read_ahead: int = 12000,
) -> subprocess.Popen:
    global BAG_PROC
    stop_playback()

    # Determine if target is a file or directory
    target_path = Path(target)
    if target_path.is_file():
        bag_dir = str(target_path.parent)
    else:
        bag_dir = str(target_path)

    cmd = [
        "ros2", "bag", "play",
        "-s", "mcap",
        bag_dir,
        "--clock", str(int(clock_hz)),
        "--rate", str(float(rate)),
        "--read-ahead-queue-size", str(int(read_ahead)),
        "--disable-keyboard-controls",
    ]
    if loop:
        cmd.append("--loop")

    with BAG_LOCK:
        BAG_PROC = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    return BAG_PROC


# ---------------------------------------------------------------------------
# Rerun clear / reset
# ---------------------------------------------------------------------------

def clear_and_reset() -> None:
    """Clear all logged data and re-send the blueprint."""
    rr.log("/", rr.Clear(recursive=True))
    rr.send_blueprint(blueprint)


# ---------------------------------------------------------------------------
# Gradio UI builder
# ---------------------------------------------------------------------------

def build_ui(
    episodes: list[tuple[str, str]],
    viewer_url: str,
) -> gr.Blocks:
    """Build the Gradio Blocks interface."""

    episode_labels = [label for label, _ in episodes]
    episode_map = {label: path for label, path in episodes}

    def on_play(episode_label: str) -> str:
        if not episode_label:
            return "⚠️ No episode selected."
        target = episode_map.get(episode_label, "")
        if not target:
            return f"⚠️ Episode not found: {episode_label}"
        clear_and_reset()
        start_playback(target, rate=1.0, clock_hz=100, loop=False, read_ahead=12000)
        return f"▶️ Playing: {episode_label}"

    with gr.Blocks(title="SO-101 Episode Browser", theme=gr.themes.Soft()) as demo:
        gr.Markdown("# 🤖 SO-101 Episode Browser")

        with gr.Row():
            # -- Left column: controls --
            with gr.Column(scale=1, min_width=300):
                episode_radio = gr.Radio(
                    choices=episode_labels,
                    label="Episodes",
                    info=f"{len(episodes)} episode(s) found",
                )

                status_md = gr.Markdown("Ready.")

            # -- Right column: Rerun viewer --
            with gr.Column(scale=3):
                gr.HTML(
                    f'<iframe src="{viewer_url}" '
                    f'style="width:100%;height:80vh;border:none;"></iframe>'
                )

        # -- Events --
        episode_radio.change(
            fn=on_play,
            inputs=[episode_radio],
            outputs=[status_md],
        )

    return demo


# ---------------------------------------------------------------------------
# CLI argument parser
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="SO-101 Episode Browser — Rerun + ROS2 bag + Gradio",
    )
    p.add_argument(
        "--episodes_root", type=str, required=True,
        help="Root directory containing MCAP episode files",
    )
    p.add_argument(
        "--wrist", type=str, default="/follower/image_raw",
        help="Wrist camera topic",
    )
    p.add_argument(
        "--overhead", type=str, default="/static_camera/image_raw",
        help="Overhead camera topic",
    )
    p.add_argument(
        "--joint-states", type=str, default="/follower/joint_states",
        help="Joint states topic",
    )
    p.add_argument(
        "--forward-commands", type=str,
        default="/follower/forward_controller/commands",
        help="Forward commands topic (set empty to disable)",
    )
    p.add_argument(
        "--joint-trajectory", type=str, default=None,
        help="Joint trajectory topic (optional)",
    )
    p.add_argument(
        "--cmd-joints", type=str, nargs="+",
        default=DEFAULT_CMD_JOINTS,
        help="Joint names for forward commands (order matters)",
    )
    p.add_argument("--server_name", type=str, default="127.0.0.1")
    p.add_argument("--server_port", type=int, default=7860)
    return p.parse_args()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    args = parse_args()
    episodes_root = Path(args.episodes_root).expanduser().resolve()
    if not episodes_root.is_dir():
        print(f"Error: episodes_root does not exist: {episodes_root}", file=sys.stderr)
        sys.exit(1)

    # -- Index episodes --
    episodes = index_episodes(episodes_root)
    print(f"Found {len(episodes)} MCAP episode(s) under {episodes_root}")
    for label, _ in episodes:
        print(f"  • {label}")

    # -- Init Rerun (proven pattern) --
    rr.init("so101_episode_browser")
    server_uri = rr.serve_grpc()
    rr.serve_web_viewer(connect_to=server_uri, open_browser=False)
    rr.send_blueprint(blueprint)
    viewer_url = "http://127.0.0.1:9090?url=rerun%2Bhttp%3A%2F%2F127.0.0.1%3A9876%2Fproxy"
    print(f"Rerun web viewer: {viewer_url}")

    # -- Init ROS2 --
    rclpy.init()
    topics = Topics(
        wrist=args.wrist,
        overhead=args.overhead,
        joint_states=args.joint_states,
        forward_commands=args.forward_commands or None,
        joint_trajectory=args.joint_trajectory or None,
    )
    node = So101Ros2ToRerun(topics, args.cmd_joints)
    node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    print("ROS2 bridge node running in background thread.")

    # -- Cleanup --
    def cleanup() -> None:
        print("\nShutting down...")
        stop_playback()
        try:
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass

    atexit.register(cleanup)

    # -- Build and launch Gradio --
    demo = build_ui(episodes, viewer_url)
    print(f"Launching Gradio on {args.server_name}:{args.server_port}")
    demo.launch(
        server_name=args.server_name,
        server_port=args.server_port,
        share=False,
        inbrowser=True,
        prevent_thread_lock=False,
    )


if __name__ == "__main__":
    main()