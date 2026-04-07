# so101_kinematics

ROS2 package for inverse-kinematics control of the SO-101 arm using
[robokin](https://github.com/dimitriBarber/robokin) (Placo backend) with a
[Viser](https://viser.studio/) 3D browser UI.

## Prerequisites

- ROS2 (Humble or later)
- The [so101-ros-physical-ai](https://github.com/dimitriBarber/so101-ros-physical-ai) stack built and sourced

## Installation

```bash
# 1. Install pip
sudo apt install -y python3-pip

# 2. Install robokin with placo and viser extras
pip install /home/so101/ros2_ws/src/robokin[placo,viser] --break-system-packages

# 3. Remove the pip-installed numpy 2.x — ROS2 ships numpy 1.x and
#    other packages (placo, viser, etc.) work fine with it.
pip uninstall numpy --break-system-packages
```

## Nodes

### `so101_ik_control_node`

Interactive IK control with a Viser gizmo.  Drag the gizmo to move the
end-effector in real time; the solver streams joint commands to the arm.

<video src="https://github.com/user-attachments/assets/ca9ddc47-adbd-4dd0-8c6e-e57ba177ee6b" controls width="100%"></video>

```bash
ros2 launch so101_bringup follower_vision.launch.py
ros2 run so101_kinematics so101_ik_control_node
```

**Parameters:**

| Parameter             | Default                                  | Description                  |
|-----------------------|------------------------------------------|------------------------------|
| `joints_topic`        | `/follower/joint_states`                 | Joint-state feedback topic   |
| `cmd_topic`           | `/follower/forward_controller/commands`  | Joint-command output topic   |
| `use_cameras`         | `false`                                  | Show camera feeds in Viser   |
| `cam_wrist_topic`     | `/follower/image_raw`                    | Wrist camera image topic     |
| `cam_overhead_topic`  | `/static_camera/image_raw`               | Overhead camera image topic  |

### `so101_planned_control_node`

Two-mode control node: **servo mode** for live gizmo dragging (Cartesian IK
every tick) and **planned mode** for smooth, pre-computed trajectories
(joint-quintic or Cartesian-interpolated).

```bash
ros2 launch so101_bringup follower_vision.launch.py
ros2 run so101_kinematics so101_planned_control_node
```

### `robokin_test_node`

Minimal test node for verifying the robokin + Viser setup without a real arm.

```bash
ros2 run so101_kinematics robokin_test_node
```

## Library modules

- **`motion_planner.py`** — Thin planning layer with cartesian and
  joint-quintic strategies (wraps robokin solver).
- **`trajectory_executor.py`** — Feeds pre-computed `(ts, qs)` trajectories
  to the arm at the correct rate.

## License

Apache-2.0 — see [LICENSE](../../LICENSE).
