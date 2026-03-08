# SO-101 ROS Physical AI

> ROS 2 Jazzy · ros2_control · MoveIt 2 · Rerun

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue?logo=ros)](https://docs.ros.org/en/jazzy/)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache%202.0-green.svg)](LICENSE)
[![MoveIt 2](https://img.shields.io/badge/MoveIt%202-Motion%20Planning-orange)](https://moveit.ros.org/)
[![Rerun](https://img.shields.io/badge/Rerun-Visualization-purple)](https://www.rerun.io/)

Complete ROS 2 stack for the SO-101 robot arm in a leader/follower configuration. Feetech STS3215 servo driver via ros2_control, leader-to-follower teleoperation, MoveIt 2 motion planning, multi-camera support, episode recording for imitation learning, conversion to LeRobot datasets, policy training, policy inference, and live Rerun visualization — all on real hardware.

> Sync inference supports ACT and SmolVLA on-device; async inference supports any LeRobot policy via `policy_server`. PRs and issues welcome.

<p>
  <a href="https://www.youtube.com/watch?v=l6kWDoHxczc">
    <img src="https://img.youtube.com/vi/l6kWDoHxczc/hqdefault.jpg" alt="SO-101 async inference demo" width="720" />
  </a>
</p>

**Hero demo:** run async policy inference on the real SO-101 arm with a ROS 2 client on the robot and the policy hosted on a remote GPU server.

## What you can do with this repo

| Teleoperate the robot | Record imitation learning episodes | Run local or remote inference |
| --- | --- | --- |
| [<img src="docs/assets/gifs/so101_teleop.gif" alt="SO-101 teleop demo" width="240" />](so101_teleop/README.md) | [<img src="docs/assets/gifs/ros2_episode_recorder.gif" alt="SO-101 episode recorder demo" width="240" />](episode_recorder/README.md) | [<img src="https://img.youtube.com/vi/l6kWDoHxczc/hqdefault.jpg" alt="SO-101 async inference demo" width="240" />](so101_inference/README.md) |
| **[Teleop](so101_teleop/README.md)**<br>Mirror the leader arm to the follower with the default and recommended `forward_controller`. | **[Episode recorder](episode_recorder/README.md)**<br>Save synchronized robot + camera episodes with keyboard controls and optional live Rerun. | **[Inference](so101_inference/README.md)**<br>Run ACT and SmolVLA on-device, or offload any LeRobot policy to a remote GPU via [`policy_server`](policy_server/README.md). |

**End-to-end workflow:** [teleoperate the robot](so101_teleop/README.md) → [record episodes](episode_recorder/README.md) → [convert rosbags to LeRobot datasets](rosbag_to_lerobot/README.md) → [train policies](#training-lerobot) → [run learned policies](so101_inference/README.md)

## Best first things to try

After the [hardware setup guide](docs/hardware.md) and [installation](#installation), these are the fastest ways to get a feel for the repo:

1. **Teleoperate the robot**

   ```bash
   ros2 launch so101_bringup teleop.launch.py
   ```

2. **Record a demonstration episode**

   ```bash
   ros2 launch so101_bringup recording_session.launch.py \
     experiment_name:=pick_and_place \
     task:="Pick up the cube and place it in the container." \
     use_rerun:=true
   ```

3. **Convert recorded episodes into a LeRobot dataset**

   ```bash
   pixi run -e lerobot convert -- \
     --input-dir ~/.ros/so101_episodes/pick_and_place \
     --config ~/ros2_ws/src/so101-ros-physical-ai/rosbag_to_lerobot/config/so101.yaml \
     --repo-id local/so101_test
   ```

4. **Then dive deeper**

   - [Teleop guide](so101_teleop/README.md)
   - [Episode recorder guide](episode_recorder/README.md)
   - [LeRobot dataset conversion](rosbag_to_lerobot/README.md)
   - [Training with LeRobot](#training-lerobot)
   - [Policy inference](so101_inference/README.md)
   - [Remote GPU policy server](policy_server/README.md)

---

## Features

| Feature                           | Description                                                                                  |
| --------------------------------- | -------------------------------------------------------------------------------------------- |
| **Leader/Follower Teleop**        | Real-time joint mirroring from leader arm to follower arm (forward or trajectory controller) |
| **ros2_control + Feetech Driver** | Hardware interface for STS3215 servos with configurable joint limits and calibration         |
| **MoveIt 2 Integration**          | OMPL-based motion planning, joint limits, kinematics (KDL) for the follower arm              |
| **Multi-Camera Pipeline**         | USB cameras and RealSense D400 series with configurable TF placement                         |
| **Episode Recording**             | Record joint states + camera frames into timestamped episodes for imitation learning         |
| **Rerun Visualization**           | Live visualization of observations, actions, and camera feeds via ROS-to-Rerun bridge (Pixi) |
| **Policy Inference**              | Sync: ACT & SmolVLA on-device. Async: any LeRobot policy (ACT, SmolVLA, π₀, …) offloaded to a remote GPU server via ZMQ/gRPC |
| **URDF/Xacro Model**              | Full SO-101 description with STL meshes, separate leader/follower end-effectors              |

---

## Packages

| Package               | Language        | Description                                                                                      |
| --------------------- | --------------- | ------------------------------------------------------------------------------------------------ |
| `so101_bringup`       | Python (launch) | Top-level launch files, hardware configs, ros2_control, cameras, recording, TF layout            |
| `so101_description`   | Xacro/URDF      | Robot model, STL meshes, RViz configs, ros2_control hardware interface macros                    |
| `so101_teleop`        | C++             | Leader-to-follower teleoperation node (forward and trajectory controller modes)                  |
| `so101_moveit_config` | YAML/Python     | MoveIt 2 config: SRDF, OMPL planning, joint limits, kinematics, controllers                      |
| `episode_recorder`    | C++             | Minimalistic rosbag (MCAP) recorder with configurable topics and keyboard-driven episode control |
| `rosbag_to_lerobot`   | Python          | Convert rosbag episodes to [LeRobot](https://github.com/huggingface/lerobot) v3.0 datasets (local or Hub) — runs in Pixi `lerobot` env |
| `so101_inference`     | Python          | Policy inference — sync (ACT, SmolVLA on-device) and async (any LeRobot policy via remote GPU server). See [so101_inference README](so101_inference/README.md) |
| `policy_server`       | Python          | GPU-side inference server — loads any LeRobot policy and serves actions over ZMQ or gRPC. Runs on a remote machine (e.g. vast.ai). See [policy_server README](policy_server/README.md) |
| `feetech_ros2_driver` | C++             | **Submodule** — Feetech STS3215 ros2_control hardware interface                                  |
| `scripts/`            | Python          | `so101_ros2_to_rerun.py` — ROS 2 to Rerun bridge (runs inside Pixi env)                          |

---

## Repository Structure

```
so101-ros-physical-ai/
├── so101_bringup/
│   ├── launch/              # teleop, recording_session, leader, follower, cameras ...
│   ├── config/
│   │   ├── hardware/        # leader/follower joint configs + LeRobot calibration refs
│   │   ├── ros2_control/    # controller YAML (forward, trajectory, joint_state)
│   │   ├── cameras/         # USB cam + RealSense configs
│   │   └── recording/       # episode recorder params
│   └── rviz/                # leader, follower, teleop RViz configs
├── so101_description/
│   ├── urdf/                # Xacro: arm, end-effectors (leader/follower), ros2_control
│   ├── meshes/              # STL meshes for all links
│   └── launch/
├── so101_teleop/
│   ├── src/                 # teleop.cpp, teleop_split.cpp
│   ├── config/              # teleop.yaml, teleop_split.yaml
│   └── launch/
├── so101_moveit_config/
│   └── config/              # SRDF, OMPL, joint_limits, kinematics, controllers
├── episode_recorder/
│   ├── src/                 # episode_recorder.cpp, teleop_episode_keyboard.cpp
│   ├── config/              # default_config.yaml
│   └── launch/
├── rosbag_to_lerobot/
│   ├── rosbag_to_lerobot/   # Python package (bag_reader, converter, decoders, cli)
│   ├── config/
│   │   └── so101.yaml       # Default conversion config (topics, features, sync)
│   └── test/
├── so101_inference/
│   └── so101_inference/     # LeRobot policy inference node + utils (runs in Pixi lerobot env)
├── feetech_ros2_driver/     # (submodule) Feetech ros2_control plugin
├── scripts/
│   └── so101_ros2_to_rerun.py
├── docs/
│   ├── hardware.md          # Full hardware setup guide (udev, calibration, cameras)
│   └── assets/
│       └── 99-so101.rules.example  # Example udev rules template
├── pixi.toml                # Pixi envs: default (Rerun bridge) + lerobot (dataset conversion + inference)
└── LICENSE
```

---

## Requirements

- **Ubuntu 24.04** + **ROS 2 Jazzy**
- Two SO-101 arms (leader + follower) with Feetech STS3215 servos, connected via USB
- `rosdep`, `colcon`
- (Optional) USB cameras / Intel RealSense for vision
- (Optional) [Pixi](https://pixi.sh/) — required for Rerun visualization, dataset conversion, and policy inference

### Hardware Setup & Calibration

> **Warning**
> Before running on real hardware you **must**:
>
> 1. Complete LeRobot motor setup + calibration and update `so101_bringup/config/hardware/*_joints.yaml` with your values.
> 2. Set up udev rules so your devices appear as `/dev/so101_leader`, `/dev/so101_follower`, `/dev/cam_wrist`, `/dev/cam_overhead`.
>
> **[→ Full hardware setup guide (docs/hardware.md)](docs/hardware.md)**

Follow the [LeRobot SO-101 guide](https://huggingface.co/docs/lerobot/so101) to assemble and configure the arms, then complete these steps before launching ROS:

| Step                          | Action                                                                                               | Notes                                        |
| ----------------------------- | ---------------------------------------------------------------------------------------------------- | -------------------------------------------- |
| **1. Motor setup**            | Run the LeRobot "setup motors" routine (writes servo IDs and baudrate to EEPROM)                     | One-time per arm                             |
| **2. Calibrate**              | Run LeRobot calibration for both arms (`calibrate` command for follower and leader)                  | Required per robot — produces a JSON per arm |
| **3. Transfer to ROS config** | Copy `id`, `homing_offset`, `range_min`, `range_max` from calibration JSON into the YAML files below | See reference JSONs                          |
| **4. Udev rules**             | Create stable device symlinks using the [example template](docs/assets/99-so101.rules.example)       | See [docs/hardware.md](docs/hardware.md)     |
| **5. Launch ROS**             | `ros2 launch so101_bringup teleop.launch.py`                                                         | Only after steps 1-4                         |

**Config files the driver reads at launch:**

```
so101_bringup/config/hardware/
├── leader_joints.yaml          # ← edit with YOUR calibration values
├── follower_joints.yaml        # ← edit with YOUR calibration values
├── lerobot_leader_arm.json     # reference: LeRobot calibration output example
└── lerobot_follower_arm.json   # reference: LeRobot calibration output example
```

The YAML files use a `joints:` top-level key with per-joint parameters: `id`, `homing_offset`, `range_min`, `range_max`, `return_delay_time`, `acceleration` (follower also supports `p_coefficient`, `i_coefficient`, `d_coefficient`, and torque limits for the gripper). The included `lerobot_*.json` files show the raw LeRobot calibration output for reference.

---

## Installation

```bash
# Clone (includes submodules)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/legalaspro/so101-ros-physical-ai.git

# Install dependencies and build
cd ~/ros2_ws
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

## Usage

### Teleop (Leader to Follower)

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch so101_bringup teleop.launch.py
```

Common overrides:

```bash
# Use trajectory controller instead of forward controller
ros2 launch so101_bringup teleop.launch.py arm_controller:=trajectory_controller

# Disable cameras
ros2 launch so101_bringup teleop.launch.py use_cameras:=false use_camera_tf:=false

# Disable RViz
ros2 launch so101_bringup teleop.launch.py use_teleop_rviz:=false
```

### Episode Recording

Record teleoperation episodes for imitation learning. In one terminal, launch the recording session:

```bash
ros2 launch so101_bringup recording_session.launch.py experiment_name:=pick_and_place task:="Pick up the cube and place it in the container." use_rerun:=true
```

In a second terminal, run the interactive keyboard controller:

```bash
ros2 run episode_recorder teleop_episode_keyboard
```

Keys: **r** — start recording, **s** — save & stop, **d** / Backspace — discard episode, **q** — quit, **h** — help. Episodes are saved to `~/.ros/so101_episodes/` by default.

After recording, you can review your episodes with the built-in episode viewer powered by [Gradio](https://www.gradio.app/) and [gradio-rerun](https://pypi.org/project/gradio-rerun/), which makes it easy to browse, select, and visualize episodes in the browser:

```bash
# pixi run python scripts/so101_episode_viewer_ros2.py --episodes_root ~/.ros/so101_episodes/pick_and_place
pixi run python scripts/so101_episode_viewer_mcap.py --episodes_root ~/.ros/so101_episodes/pick_and_place
```

> **Note:** A ROS 2 variant (`so101_episode_viewer_ros2.py`) also exists — it replays bags through ROS 2 to correctly visualize `Float64MultiArray` action messages that the MCAP reader doesn't natively decode. In a future Rerun release, MCAP will be fully supported and the ROS 2 variant will no longer be needed.

### LeRobot Dataset Conversion

The repo ships a second Pixi environment (`lerobot`) that bundles [LeRobot](https://github.com/huggingface/lerobot), ffmpeg, and all dependencies needed to convert recorded rosbag episodes into LeRobot v3.0 datasets. See the full [rosbag_to_lerobot README](rosbag_to_lerobot/README.md) for details.

**Local conversion** (no Hugging Face account needed):

```bash
pixi run -e lerobot convert -- \
  --input-dir  ~/.ros/so101_episodes/pick_and_place \
  --config     ~/ros2_ws/src/so101-ros-physical-ai/rosbag_to_lerobot/config/so101.yaml \
  --repo-id    local/so101_test
```

**Push to Hugging Face Hub:**

```bash
# Authenticate (once)
pixi run -e lerobot -- hf auth login
# or: export HF_TOKEN="hf_..."

# Verify
pixi run -e lerobot -- hf auth whoami

# Convert & push
pixi run -e lerobot convert -- \
  --input-dir  ~/.ros/so101_episodes/pick_and_place \
  --config     ~/ros2_ws/src/so101-ros-physical-ai/rosbag_to_lerobot/config/so101.yaml \
  --repo-id    <hf-username>/so101-pick-and-place \
  --push-hub
```

**Visualize** the converted dataset:

```bash
# Local — enter the lerobot shell and use the built-in visualizer
pixi shell -e lerobot
lerobot-dataset-viz --repo-id local/so101_test --episode-index 0

# Online — after pushing to the Hub, open:
# https://huggingface.co/spaces/lerobot/visualize_dataset
```

### Training (LeRobot)

Once you have a LeRobot dataset (local or on the Hub), train a policy using the [LeRobot](https://github.com/huggingface/lerobot) training CLI:

```bash
pixi shell -e lerobot

lerobot-train \
  --dataset.repo_id=<hf-username>/so101-pick-and-place \
  --policy.type=act \
  --output_dir=outputs/train/act_so101_pick_place \
  --job_name=act_so101_pick_place \
  --policy.device=cuda
```

Optional flags: `--wandb.enable=true`, `--policy.repo_id=<hf-username>/<policy-name>` (auto-push checkpoint to Hub). See the [LeRobot training docs](https://huggingface.co/docs/lerobot/il_robots#train-a-policy) for the full list of options.

### Policy Inference (LeRobot)

Deploy trained LeRobot policies on the real SO-101 follower arm. Two inference modes are available — **synchronous** (ACT, SmolVLA on-device) and **asynchronous** (any LeRobot policy — ACT, SmolVLA, π₀, and others — offloaded to a remote GPU server via ZMQ/gRPC). See the full [so101_inference README](so101_inference/README.md) for all parameters and details.

**Terminal 1 — bring up the follower arm + cameras (+ optional Rerun):**

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch so101_bringup inference.launch.py
```

**Terminal 2 — run the policy (Pixi `lerobot` env):**

```bash
cd ~/ros2_ws/src/so101-ros-physical-ai

# Synchronous — ACT policy on-device
pixi run -e lerobot infer -- --ros-args \
    -p repo_id:="legalaspro/act_so101_pnp_crosslane_showcase_60_50hz_v0"

# Synchronous — SmolVLA on-device
pixi run -e lerobot infer -- --ros-args \
    -p repo_id:="legalaspro/smolvla_so101_pnp_crosslane_showcase_60_50hz_v0" \
    -p policy_type:=smolvla \
    -p camera_top_name:=camera1 -p camera_wrist_name:=camera2

# Asynchronous — SmolVLA offloaded to a remote GPU server
pixi run -e lerobot async_infer -- --ros-args \
    -p repo_id:="legalaspro/smolvla_so101_pnp_crosslane_showcase_60_50hz_v0" \
    -p policy_type:=smolvla \
    -p server_address:=192.168.1.100:8090 \
    -p fps:=50.0 -p actions_per_chunk:=50 -p chunk_size_threshold:=0.6 \
    -p camera_top_name:=camera1 -p camera_wrist_name:=camera2
```

Common overrides:

```bash
# Disable Rerun in the launch file
ros2 launch so101_bringup inference.launch.py use_rerun:=false
```

### Rerun (Live Visualization)

The repo ships a [Pixi](https://pixi.sh/) environment with `bridge` and `viewer` tasks. Rerun can be added to both teleop and recording sessions:

```bash
# Set once
export SO101_RERUN_ENV_DIR=~/ros2_ws/src/so101-ros-physical-ai

# Teleop with Rerun instead of RViz
ros2 launch so101_bringup teleop.launch.py use_rerun:=true use_teleop_rviz:=false

# Recording session with Rerun
ros2 launch so101_bringup recording_session.launch.py experiment_name:=pick_and_place use_rerun:=true
```

Or run the bridge standalone:

```bash
cd ~/ros2_ws/src/so101-ros-physical-ai
pixi run viewer   # in one terminal
pixi run bridge   # in another (after sourcing ROS)
```

### MoveIt 2 (Follower)

MoveIt 2 is configured with dedicated ros2_control controllers for the follower arm: a `FollowJointTrajectory` controller for the 5-DOF arm and a `ParallelGripperCommand` controller for the gripper, enabling OMPL-based motion planning with independent gripper control.

```bash
ros2 launch so101_bringup follower_moveit_demo.launch.py
```

---

## Configuration

### Launch Arguments (teleop.launch.py)

| Argument            | Default               | Description                                            |
| ------------------- | --------------------- | ------------------------------------------------------ |
| `hardware_type`     | `real`                | `real` or `mock` (`mujoco` planned, not yet supported) |
| `arm_controller`    | `forward_controller`  | `forward_controller` or `trajectory_controller`        |
| `use_cameras`       | `true`                | Enable USB / RealSense cameras                         |
| `use_camera_tf`     | `true`                | Publish camera TF frames                               |
| `use_teleop_rviz`   | `true`                | Launch RViz with teleop config                         |
| `use_rerun`         | `false`               | Launch Rerun bridge                                    |
| `leader_usb_port`   | `/dev/so101_leader`   | Leader arm USB device                                  |
| `follower_usb_port` | `/dev/so101_follower` | Follower arm USB device                                |

### Launch Arguments (inference.launch.py)

| Argument                       | Default               | Description                                            |
| ------------------------------ | --------------------- | ------------------------------------------------------ |
| `hardware_type`                | `real`                | `real` or `mock`                                       |
| `arm_controller`               | `forward_controller`  | `forward_controller` or `trajectory_controller`        |
| `follower_usb_port`            | `/dev/so101_follower` | Follower arm USB device                                |
| `use_rerun`                    | `true`                | Launch Rerun bridge alongside hardware                 |
| `rerun_env_dir`                | `$SO101_RERUN_ENV_DIR`| Path to repo root (for Pixi Rerun env)                 |

### Hardware Configs

- `so101_bringup/config/hardware/` — joint names, IDs, limits, calibration
- `so101_bringup/config/ros2_control/` — controller parameters (forward, trajectory, joint_state_broadcaster)
- `so101_bringup/config/cameras/` — camera topics, resolution, frame rates

---

## Author

Dmitri Manajev

## License

This project is licensed under the Apache License 2.0 — see [LICENSE](LICENSE) for details.
