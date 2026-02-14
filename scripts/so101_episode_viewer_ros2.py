#!/usr/bin/env python3
"""SO-101 Episode Browser — Rerun RecordingStream + ROS2 bag playback + Gradio UI."""

from __future__ import annotations

import argparse
import atexit
import signal
import subprocess
import sys
import threading
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterator

import gradio as gr
import numpy as np
import rclpy
import rerun as rr
from gradio_rerun import Rerun
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image, JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory

# ---------------------------------------------------------------------------
# Constants & Styling
# ---------------------------------------------------------------------------

APP_ID = "so101_episode_browser"

CSS = """
#episode_list_wrap {
  height: 750px;
  overflow-y: auto;
  border: 1px solid var(--border-color-primary);
  border-radius: 8px;
  padding: 8px;
}
"""

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
    return np.datetime64(stamp.sec * 1_000_000_000 + stamp.nanosec, "ns")



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


# ---------------------------------------------------------------------------
# ROS2 Node — So101Ros2ToRerun (stream-aware)
# ---------------------------------------------------------------------------


class So101Ros2ToRerun(Node):
    """Subscribe to SO-101 topics and log data to a swappable RecordingStream."""

    def __init__(self, topics: Topics, cmd_joint_order: list[str]) -> None:
        super().__init__("so101_ros2_to_rerun")
        self._cmd_joint_order = list(cmd_joint_order)

        # -- Swappable RecordingStream --
        self._rec: rr.RecordingStream | None = None

        # Timestamp reference for unstamped messages (Float64MultiArray).
        # Stamped callbacks write here; the action callback reads it.
        # Protected by _time_lock for thread-safety.
        self._time_lock = threading.Lock()
        self._last_ros_time: np.datetime64 | None = None
        self._last_action_time: np.datetime64 | None = None

        # -- Callback groups --
        self._cg_img_wrist = ReentrantCallbackGroup()
        self._cg_img_over = ReentrantCallbackGroup()
        self._cg_joints = ReentrantCallbackGroup()
        self._cg_cmd = ReentrantCallbackGroup()
        self._cg_traj = ReentrantCallbackGroup()

        # -- Subscriptions --
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

        self.get_logger().info("Rerun bridge started (stream-aware).")

    # -- public API --

    def set_recording(self, rec: rr.RecordingStream | None) -> None:
        """Hot-swap the target RecordingStream."""
        self._rec = rec
        with self._time_lock:
            self._last_ros_time = None
            self._last_action_time = None

    def _get_rec(self) -> rr.RecordingStream | None:
        return self._rec

    # -- helpers --

    @staticmethod
    def _is_compressed(topic_name: str) -> bool:
        return "compressed" in topic_name.lower()

    # -- timestamp helpers --

    def _next_action_time(self) -> np.datetime64 | None:
        """Return a strictly-increasing timestamp for unstamped action msgs.

        Derives from the last stamped reference time. Tracks its own
        monotonic counter so consecutive actions never share a timestamp,
        even if the stamped reference hasn't changed.

        Returns None when no stamped reference exists yet.
        """
        with self._time_lock:
            if self._last_ros_time is None:
                return None
            ts = self._last_ros_time
            prev = self._last_action_time
            if prev is not None and ts <= prev:
                ts = prev + np.timedelta64(1, "ns")
            self._last_action_time = ts
            return ts

    # -- image callbacks --

    def _on_wrist_img(self, msg: CompressedImage) -> None:
        rec = self._get_rec()
        if rec is None:
            return
        mt = media_type_from_compressed_format(msg.format) or "image/jpeg"
        rec.set_time("ros_time", timestamp=stamp_to_datetime64(msg.header.stamp))
        rec.log("cameras/wrist", rr.EncodedImage(contents=bytes(msg.data), media_type=mt))

    def _on_wrist_img_raw(self, msg: Image) -> None:
        rec = self._get_rec()
        if rec is None:
            return
        rec.set_time("ros_time", timestamp=stamp_to_datetime64(msg.header.stamp))
        rec.log("cameras/wrist", rr.Image(rgb8_to_numpy(msg), color_model="RGB"))

    def _on_overhead_img(self, msg: CompressedImage) -> None:
        rec = self._get_rec()
        if rec is None:
            return
        mt = media_type_from_compressed_format(msg.format) or "image/jpeg"
        rec.set_time("ros_time", timestamp=stamp_to_datetime64(msg.header.stamp))
        rec.log("cameras/overhead", rr.EncodedImage(contents=bytes(msg.data), media_type=mt))

    def _on_overhead_img_raw(self, msg: Image) -> None:
        rec = self._get_rec()
        if rec is None:
            return
        rec.set_time("ros_time", timestamp=stamp_to_datetime64(msg.header.stamp))
        rec.log("cameras/overhead", rr.Image(rgb8_to_numpy(msg), color_model="RGB"))

    # -- joint state callback --

    def _on_joint_states(self, msg: JointState) -> None:
        rec = self._get_rec()
        if rec is None:
            return
        ts = stamp_to_datetime64(msg.header.stamp)
        # Cache as the reference clock for unstamped action messages.
        with self._time_lock:
            self._last_ros_time = ts
        rec.set_time("ros_time", timestamp=ts)
        for name, pos in zip(msg.name, msg.position):
            rec.log(f"state/position/{name}", rr.Scalars(float(pos)))

    # -- forward commands callback --

    def _on_forward_commands(self, msg: Float64MultiArray) -> None:
        rec = self._get_rec()
        if rec is None:
            return
        # Float64MultiArray has no header — derive a monotonic timestamp
        # from the last known stamped time.
        ts = self._next_action_time()
        if ts is None:
            return  # no stamped reference yet, skip
        rec.set_time("ros_time", timestamp=ts)
        for name, val in zip(self._cmd_joint_order, msg.data):
            rec.log(f"action/position/{name}", rr.Scalars(float(val)))

    # -- joint trajectory callback --

    def _on_joint_trajectory(self, msg: JointTrajectory) -> None:
        rec = self._get_rec()
        if rec is None:
            return
        rec.set_time("ros_time", timestamp=stamp_to_datetime64(msg.header.stamp))
        if msg.points:
            pt = msg.points[0]
            for name, pos in zip(msg.joint_names, pt.positions):
                rec.log(f"action/position/{name}", rr.Scalars(float(pos)))



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
    loop: bool = False,
    read_ahead: int = 12000,
) -> subprocess.Popen:
    global BAG_PROC
    stop_playback()

    target_path = Path(target)
    if target_path.is_file():
        bag_dir = str(target_path.parent)
    else:
        bag_dir = str(target_path)

    cmd = [
        "ros2", "bag", "play",
        "-s", "mcap",
        bag_dir,
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
# Streaming logic (mirrors MCAP viewer pattern)
# ---------------------------------------------------------------------------


def stream_episode(
    episode_label: str,
    episodes_state: dict[str, str],
    node: So101Ros2ToRerun,
    current_recording_id: str,
) -> Iterator[tuple[Any, Any, str]]:
    """Generator that creates a new RecordingStream per episode, starts bag
    playback, and yields binary chunks to the Gradio Rerun component."""

    if not episode_label:
        yield gr.skip(), "⚠️ No episode selected.", current_recording_id
        return

    target = episodes_state.get(episode_label, "")
    if not target:
        yield gr.skip(), f"⚠️ Episode not found: {episode_label}", current_recording_id
        return

    # -- Create a fresh RecordingStream --
    new_recording_id = str(uuid.uuid4())
    rec = rr.RecordingStream(application_id=APP_ID, recording_id=new_recording_id)
    stream = rec.binary_stream()

    # Send blueprint on this stream
    rec.send_blueprint(make_blueprint())

    # Hot-swap the recording on the ROS2 node
    node.set_recording(rec)

    # Start bag playback (publishes to ROS2 topics → node callbacks → rec)
    stop_playback()
    proc = start_playback(target, rate=1.0, loop=False, read_ahead=12000)

    yield gr.skip(), f"▶️ Loading `{episode_label}`...", new_recording_id

    try:
        while proc.poll() is None:
            chunk = stream.read()
            if chunk:
                yield chunk, gr.skip(), new_recording_id
            else:
                time.sleep(0.01)

        # Flush remaining data after playback ends
        while True:
            chunk = stream.read()
            if not chunk:
                break
            yield chunk, gr.skip(), new_recording_id

        yield gr.skip(), f"✅ `{episode_label}` complete", new_recording_id

    except GeneratorExit:
        stop_playback()
    finally:
        node.set_recording(None)
        rr.disconnect(recording=rec)


# ---------------------------------------------------------------------------
# Gradio UI builder
# ---------------------------------------------------------------------------


def build_ui(
    episodes: list[tuple[str, str]],
    node: So101Ros2ToRerun,
) -> gr.Blocks:
    """Build the Gradio Blocks interface with embedded Rerun streaming viewer."""

    episode_labels = [label for label, _ in episodes]
    episode_map = {label: path for label, path in episodes}

    with gr.Blocks(title="SO-101 Episode Browser", theme=gr.themes.Soft(), css=CSS) as demo:

        # --- State ---
        episodes_state = gr.State(episode_map)
        recording_id = gr.State("")

        # --- Layout ---
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

            # -- Right column: Rerun streaming viewer --
            with gr.Column(scale=3):
                viewer = Rerun(
                    streaming=True,
                    height=800,
                    panel_states={
                        "blueprint": "hidden",
                        "selection": "hidden",
                        "time": "collapsed",
                    },
                )

        # -- Events --
        # We need to pass the node to stream_episode; wrap in a closure
        def _stream(episode_label, ep_state, rec_id):
            yield from stream_episode(episode_label, ep_state, node, rec_id)

        episode_radio.change(
            fn=_stream,
            inputs=[episode_radio, episodes_state, recording_id],
            outputs=[viewer, status_md, recording_id],
            concurrency_limit=1,
        )

    return demo


# ---------------------------------------------------------------------------
# CLI argument parser
# ---------------------------------------------------------------------------


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="SO-101 Episode Browser — Rerun RecordingStream + ROS2 bag + Gradio",
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
    p.add_argument("--server_name", type=str, default="0.0.0.0")
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
    demo = build_ui(episodes, node)
    print(f"Launching Gradio on {args.server_name}:{args.server_port}")
    demo.queue().launch(
        server_name=args.server_name,
        server_port=args.server_port,
        share=False,
        inbrowser=True,
        prevent_thread_lock=False,
    )


if __name__ == "__main__":
    main()