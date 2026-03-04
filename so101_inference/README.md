# so101_inference

ROS 2 inference package for the SO-101 robot arm. Runs [LeRobot](https://github.com/huggingface/lerobot) policies (ACT, SmolVLA) directly on the robot, consuming camera images and joint states and publishing joint commands at high frequency.

## Features

- **Synchronous (local) inference** — loads the policy on the robot's own GPU/CPU and runs forward passes in a tight control loop. Supports ACT and SmolVLA policies.
- **Asynchronous (remote) inference** — offloads the policy to a remote server via ZeroMQ or gRPC, while the robot keeps executing actions from a local queue. Supports any LeRobot policy (ACT, SmolVLA, π₀, and others) since inference runs server-side. Ideal for large VLA models that don't fit on the robot's device.
- **Configurable camera names** — different policies expect different camera observation keys (e.g. ACT may use `top`/`wrist` while VLAs like SmolVLA typically use `camera1`/`camera2`). The `camera_top_name` and `camera_wrist_name` parameters let you match whatever names the policy was trained with, without changing topics or re-training.
- **Action chunking & aggregation** — the async node manages an action queue with configurable chunk sizes, refill thresholds, and aggregation strategies (`weighted_average`, `latest_only`, `average`, `conservative`).
- **Compressed image support** — the async node can subscribe to `CompressedImage` topics and forward raw JPEG bytes to the server (decoded server-side) for lower bandwidth usage.
- **Stale-data gating** — observations older than `max_age_s` are automatically discarded to prevent the robot from acting on outdated sensor data.
- **Pluggable transports** — ZeroMQ (`zmq`) and gRPC (`grpc`) transport backends, selected via a single parameter.
- **Built-in telemetry** — periodic summaries of FPS, queue depth, actions executed, observations sent/dropped, and round-trip latency.

## Nodes

| Node | Executable | Description |
|------|-----------|-------------|
| `lerobot_inference_node` | `lerobot_inference_node` | Synchronous local inference — loads the policy on-device |
| `async_ros2_inference_client` | `async_inference_node` | Asynchronous remote inference — offloads policy to a server |

## Quick Start

### Prerequisites

The robot hardware stack (follower arm + cameras) must already be running and publishing on the expected ROS 2 topics. Build the workspace and source it, or use the Pixi environment:

```bash
# Build (if not using pixi)
cd ~/ros2_ws && colcon build --packages-select so101_inference
source install/setup.bash
```

### Synchronous Inference (on-device)

Run a locally-loaded ACT policy:

```bash
pixi run -e lerobot infer -- --ros-args \
    -p repo_id:="legalaspro/act_so101_pnp_crosslane_showcase_60_50hz_v0"
```

Run a SmolVLA policy locally (VLAs typically use different camera names like `camera1`/`camera2`):

```bash
pixi run -e lerobot infer -- --ros-args \
    -p repo_id:="legalaspro/smolvla_so101_pnp_crosslane_showcase_60_50hz_v0" \
    -p policy_type:=smolvla \
    -p fps:=50.0 \
    -p camera_top_name:=camera1 \
    -p camera_wrist_name:=camera2
```

### Asynchronous Inference (remote server)

Run a SmolVLA policy offloaded to a remote GPU server:

```bash
pixi run -e lerobot async_infer -- --ros-args \
    -p repo_id:="legalaspro/smolvla_so101_pnp_crosslane_showcase_60_50hz_v0" \
    -p policy_type:=smolvla \
    -p server_address:=192.168.1.100:8090 \
    -p fps:=50.0 \
    -p actions_per_chunk:=50 \
    -p chunk_size_threshold:=0.6 \
    -p camera_top_name:=camera1 \
    -p camera_wrist_name:=camera2
```

ACT policy with ZeroMQ transport (default):

```bash
pixi run -e lerobot async_infer -- --ros-args \
    -p repo_id:="legalaspro/act_so101_pnp_crosslane_showcase_60_50hz_v0" \
    -p server_address:=10.0.0.42:8090 \
    -p actions_per_chunk:=100 \
    -p chunk_size_threshold:=0.5
```

Using gRPC transport instead of ZeroMQ:

```bash
pixi run -e lerobot async_infer -- --ros-args \
    -p transport_type:=grpc \
    -p server_address:=10.0.0.42:50051 \
    -p repo_id:="legalaspro/act_so101_pnp_crosslane_showcase_60_50hz_v0"
```

## Parameters

### Synchronous Node (`infer`)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `repo_id` | string | `legalaspro/act_so101_pnp_microsanity_20_50hz_v0` | HuggingFace model repo ID |
| `policy_type` | string | `act` | Policy architecture: `act` or `smolvla` |
| `task` | string | `Put the green cube in the cup.` | Task description (used by VLA models) |
| `fps` | float | `50.0` | Control loop frequency |
| `max_age_s` | float | `0.2` | Max sensor data age before it's considered stale |
| `camera_top_name` | string | `top` | Camera name for the top/overhead camera as expected by the policy |
| `camera_wrist_name` | string | `wrist` | Camera name for the wrist camera as expected by the policy |
| `fwd_topic` | string | `/follower/forward_controller/commands` | Topic to publish joint commands |
| `joints_topic` | string | `/follower/joint_states` | Topic to subscribe for joint states |
| `top_camera_topic` | string | `/static_camera/image_raw` | Topic for overhead camera images |
| `wrist_camera_topic` | string | `/follower/image_raw` | Topic for wrist camera images |

### Asynchronous Node (`async_infer`)

All parameters from the synchronous node plus:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `transport_type` | string | `zmq` | Transport backend: `zmq` or `grpc` |
| `server_address` | string | `127.0.0.1:8090` | Policy server `host:port` |
| `policy_device` | string | `cuda` | Device for policy inference on the server |
| `client_device` | string | `cpu` | Device for pre/post-processing on the robot |
| `actions_per_chunk` | int | `100` | Number of actions requested per inference call |
| `chunk_size_threshold` | float | `0.5` | Queue fill ratio below which a new observation is sent (0.0–1.0) |
| `aggregate_fn_name` | string | `weighted_average` | Action aggregation strategy: `weighted_average`, `latest_only`, `average`, `conservative` |
| `use_compressed` | bool | `false` | Subscribe to `CompressedImage` topics and send JPEG bytes to the server |

## Architecture

```
 cameras + joints ──► inference node ──► arm commands
                           │
                      sync: on-device policy (ACT/SmolVLA)
                      async: ZMQ/gRPC ──► remote GPU server
```

## License

Apache-2.0 — see [LICENSE](../LICENSE).

