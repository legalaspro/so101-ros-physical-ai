# rosbag_to_lerobot

Convert ROS 2 rosbag episodes (MCAP) into [LeRobot v3.0](https://github.com/huggingface/lerobot) datasets — locally or pushed straight to the Hugging Face Hub.

## How It Works

The converter reads bags sequentially using a **reference-topic-driven** approach:
every message on the reference topic (e.g. the wrist camera) emits one dataset frame.
Other features (joint state, action, additional cameras) are sampled as-of the reference timestamp with a configurable freshness window (`max_age_s`). Frames where any feature is missing or stale are automatically dropped, and per-episode synchronization diagnostics are logged.

### Alignment Guarantee

Non-reference features are sampled as-of the reference timestamp using the latest message **≤ reference time**. This prevents "future leakage" (e.g. pairing an observation with an action that happened after it).

## Prerequisites

This package runs inside the **`lerobot` Pixi environment** shipped with the repo (see `pixi.toml` at the repo root). The environment bundles LeRobot, ffmpeg, and all Python dependencies so nothing else needs to be installed manually.

```bash
# Make sure Pixi is installed — https://pixi.sh
# All commands below are run from the repo root (so101-ros-physical-ai/)
```

> **Note:** The ROS 2 environment must be sourceable (and your workspace built if you rely on custom message types), because the converter uses ROS 2 Python libraries (`rosbag2_py`, message definitions, `rclpy.serialization`) at runtime.

---

## Quick Start — Local Conversion

No Hugging Face account required. Use `local/` as the repo-id prefix to keep everything on disk:

```bash
pixi run -e lerobot convert -- \
  --input-dir  ~/.ros/so101_episodes/pick_and_place_2 \
  --config     ~/ros2_ws/src/so101-ros-physical-ai/rosbag_to_lerobot/config/so101.yaml \
  --repo-id    local/so101_test
```

If `--output-dir` is omitted, LeRobot writes to its default cache location (typically `~/.cache/huggingface/lerobot/<repo-id>/`).

### Useful flags

| Flag | Description |
|------|-------------|
| `--overwrite` | Delete any existing dataset at the target path before writing |
| `--sync-p95` | Collect p95 sync latency stats (slightly more overhead) |
| `--vcodec <codec>` | Video codec (`libsvtav1` default, `libx264`, `h264_nvenc`, …) |
| `--use-videos` / `--no-use-videos` | MP4 video (default) vs individual images |

---

## Publishing to Hugging Face Hub

### 1. Authenticate

Pick **one** of:

```bash
# Interactive browser login
pixi run -e lerobot -- hf auth login

# Or export your token directly
export HF_TOKEN="hf_..."
```

Verify you're logged in:

```bash
pixi run -e lerobot -- hf auth whoami
```

### 2. Convert & Push

```bash
pixi run -e lerobot convert -- \
  --input-dir  ~/.ros/so101_episodes/pick_and_place_2 \
  --config     ~/ros2_ws/src/so101-ros-physical-ai/rosbag_to_lerobot/config/so101.yaml \
  --repo-id    <hf-username>/so101-pick-and-place \
  --push-hub
```

The `--push-hub` flag finalizes the dataset and uploads it to `https://huggingface.co/datasets/<repo-id>`.

---

## Visualization

### Local (LeRobot CLI)

Enter the lerobot shell, then use the built-in dataset visualizer:

```bash
pixi shell -e lerobot
lerobot-dataset-viz --repo-id local/so101_test --episode-index 0
```

### Online (Hugging Face Space)

After pushing a dataset to the Hub, visualize it at:

👉 **<https://huggingface.co/spaces/lerobot/visualize_dataset>**

Enter your `repo-id` (e.g. `<hf-username>/so101-pick-and-place`) and browse episodes interactively.

---

## Configuration

Conversion is driven by a YAML file (see [`config/so101.yaml`](config/so101.yaml) for the default). Key fields:

| Field | Purpose |
|-------|---------|
| `robot_type` | Robot identifier written into the dataset metadata |
| `fps` | Target frame rate |
| `reference_topic` | ROS topic whose messages drive frame emission |
| `task` | Task label written to every frame |
| `default_max_age_s` | Default freshness window for feature sampling |
| `features[]` | List of features — each entry has: `key`, `topic`, `msg_type`, `stamp_src`, `shape`, `names`, `max_age_s` |

### `stamp_src` — Timestamp Source

Each feature declares a `stamp_src` that controls which timestamp is used for synchronization:

| Value | Behaviour |
|-------|-----------|
| `bag` **(default)** | Use the bag-level receive timestamp (from `read_next()`). Recommended for real hardware. |
| `header` | Use the message's `header.stamp`. Useful in simulation where clocks are perfectly synced. Falls back to `bag` if the message type has no header. |

We default to `bag` because it reflects the moment the message was actually written to the bag, which gives the most consistent alignment across topics on real hardware. Camera drivers in particular report header timestamps that lag behind the true capture time by varying amounts — after benchmarking several ROS 2 camera drivers (`v4l2_camera`, `usb_cam`, `camera_ros`/libcamera, `gscam`) we found that `camera_ros` and `gscam` performed best in terms of stable Hz and low header-to-bag delay; we selected `gscam` as slightly better overall (~30-35 ms delay). Even so, the bag timestamp remains the most reliable common reference for cross-topic synchronization. In simulation, where all clocks are perfectly synchronized, `header` can be the better choice.

### Supported message types

`sensor_msgs/msg/Image`, `sensor_msgs/msg/CompressedImage`, `sensor_msgs/msg/JointState`, `std_msgs/msg/Float64MultiArray`.

---

## CLI Reference

```
pixi run -e lerobot convert -- --help
```

```
usage: convert --input-dir DIR --config FILE --repo-id ID [options]

  --input-dir       Directory containing episode bag subdirectories
  --config          Path to YAML config file
  --repo-id         HuggingFace repo ID (e.g. user/dataset_name or local/name)
  --output-dir      Override default output location
  --use-videos      Store as MP4 video (default) / --no-use-videos for images
  --vcodec          Video codec (libsvtav1 | libx264 | h264 | hevc | h264_nvenc)
  --push-hub        Push final dataset to HuggingFace Hub
  --sync-p95        Collect p95 sync stats
  --overwrite       Delete existing dataset directory before writing
```

---

## License

Apache License 2.0 — see [LICENSE](../LICENSE) for details.

