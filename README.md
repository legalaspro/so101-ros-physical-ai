# SO-101 ROS Physical AI

> ROS 2 Jazzy · ros2_control · MoveIt 2 · Rerun

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue?logo=ros)](https://docs.ros.org/en/jazzy/)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache%202.0-green.svg)](LICENSE)
[![MoveIt 2](https://img.shields.io/badge/MoveIt%202-Motion%20Planning-orange)](https://moveit.ros.org/)
[![Rerun](https://img.shields.io/badge/Rerun-Visualization-purple)](https://www.rerun.io/)

Complete ROS 2 stack for the SO-101 robot arm in a leader/follower configuration. Feetech STS3215 servo driver via ros2_control, leader-to-follower teleoperation, MoveIt 2 motion planning, multi-camera support, episode recording for imitation learning, and live Rerun visualization — all on real hardware.

> **Status:** actively developed — teleop, episode recording, and visualization are working. Next up: LeRobot dataset conversion, training pipeline, and inference/deployment. PRs and issues welcome.

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
├── feetech_ros2_driver/     # (submodule) Feetech ros2_control plugin
├── scripts/
│   └── so101_ros2_to_rerun.py
├── pixi.toml                # Pixi env for Rerun bridge + viewer
└── LICENSE
```

---

## Requirements

- **Ubuntu 24.04** + **ROS 2 Jazzy**
- Two SO-101 arms (leader + follower) with Feetech STS3215 servos, connected via USB
- `rosdep`, `colcon`
- (Optional) USB cameras / Intel RealSense for vision
- (Optional) [Pixi](https://pixi.sh/) for Rerun visualization

### Hardware Setup & Calibration

> **Warning**
> Do not run on real hardware until you have calibrated your specific robot and updated
> `so101_bringup/config/hardware/leader_joints.yaml` and `follower_joints.yaml`.
> The files shipped in this repo are examples from one robot and **will not match yours**.

Follow the [LeRobot SO-101 guide](https://huggingface.co/docs/lerobot/so101) to assemble the arms, then complete these steps before launching ROS:

| Step                          | Action                                                                                               | Notes                                        |
| ----------------------------- | ---------------------------------------------------------------------------------------------------- | -------------------------------------------- |
| **1. Motor setup**            | Run the LeRobot "setup motors" routine (writes servo IDs and baudrate to EEPROM)                     | One-time per arm                             |
| **2. Calibrate**              | Run LeRobot calibration for both arms (`calibrate` command for follower and leader)                  | Required per robot — produces a JSON per arm |
| **3. Transfer to ROS config** | Copy `id`, `homing_offset`, `range_min`, `range_max` from calibration JSON into the YAML files below | See reference JSONs                          |
| **4. Launch ROS**             | `ros2 launch so101_bringup teleop.launch.py`                                                         | Only after steps 1-3                         |

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
ros2 launch so101_bringup recording_session.launch.py experiment_name:=pick_and_place
```

In a second terminal, run the interactive keyboard controller:

```bash
ros2 run episode_recorder teleop_episode_keyboard
```

Keys: **r** — start recording, **s** — save & stop, **d** / Backspace — discard episode, **q** — quit, **h** — help. Episodes are saved to `~/.ros/so101_episodes/` by default.

### MoveIt 2 (Follower)

```bash
ros2 launch so101_bringup follower_moveit_demo.launch.py
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

### Hardware Configs

- `so101_bringup/config/hardware/` — joint names, IDs, limits, calibration
- `so101_bringup/config/ros2_control/` — controller parameters (forward, trajectory, joint_state_broadcaster)
- `so101_bringup/config/cameras/` — camera topics, resolution, frame rates

---

## Author

Dmitri Manajev

## License

This project is licensed under the Apache License 2.0 — see [LICENSE](LICENSE) for details.
