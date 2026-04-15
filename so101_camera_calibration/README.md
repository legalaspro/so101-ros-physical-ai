# so101_camera_calibration

Camera intrinsic and hand-eye calibration tools for the SO-101 arm.
Both tools expose a **Viser web UI** at `http://localhost:8080`, so
calibration can be done headlessly without a desktop GUI.

> **Intrinsic calibration** (section 1) is the useful part — standard
> ChArUco-based OpenCV calibration, and the output YAML drops straight into
> `camera_info_url` for any ROS 2 camera driver.
>
> ⚠️ **Hand-eye calibration** (section 2) is experimental. On a low-cost
> arm like the SO-101, servo repeatability (~1–2°) sets a hard accuracy
> floor, and the workflow here is a first pass rather than a validated
> pipeline. Treat the result as a starting point, not ground truth.
> If you work in computer vision / robotics and can spot issues or suggest
> improvements for cheap arms like the SO-101, contributions are very
> welcome.

> **Frame naming.** In this README `base_link` refers to the robot base
> frame; in the running system this is typically `follower/base_link`.
> The camera frames form a chain `static_camera_link → static_camera_optical_frame`
> where the optical frame is the one OpenCV (and the calibration output)
> talks about.

## Prerequisites

Before running either calibration, bring up the follower arm with its
camera stack in a separate terminal:

```bash
ros2 launch so101_bringup follower_vision.launch.py
```

This publishes `/static_camera/image_raw`, `/static_camera/camera_info`,
`/follower/joint_states`, and the TF tree that the calibration nodes
consume.

You also need a printed ChArUco board — see
[Printing a Target](#printing-a-target).

## 1. Camera Intrinsic Calibration

Calibrates focal lengths, principal point, and distortion.

<video src="https://github.com/user-attachments/assets/d333c7da-e61c-4738-8384-039c7fbca979" controls width="720"></video>


```bash
ros2 run so101_camera_calibration camera_intrinsic_calibration_node \
  --ros-args -p image_topic:=/static_camera/image_raw
```

Open `http://localhost:8080`. Move the board around and vary tilt:

- cover center, corners, and edges
- rotate and tilt the board
- fill as much of the coverage grid as possible

Outputs:

- `/tmp/camera_cal.yaml` — ROS `camera_info` format
- `/tmp/camera_cal.npz` — NumPy archive

## 2. Hand-Eye (Extrinsic) Calibration

Estimates the transform `base_link → static_camera_optical_frame`.

```bash
ros2 launch so101_camera_calibration handeye_calibration.launch.py
```

### Manual Calibration (Recommended)

<video src="https://github.com/user-attachments/assets/cf9792f5-6f94-478e-8a59-db8f60bd06aa" controls width="720"></video>

1. Toggle **Manual EE: ON** to enable the IK gizmo
2. Drag the gripper to a pose where the ChArUco board is clearly visible
3. Click **📷 Take Sample**
4. Repeat for **20–30 diverse poses** — vary position *and* tilt
5. Click **🧮 Compute Calibration**
6. Verify **Park** and **Horaud** agree closely (for example within ~1 cm)
7. Click **✅ Save Calibration**

Saved to `~/.ros2/robokin_calibrations/so101_eye_on_base.yaml`.

> **Tip:** Rotational variety is critical. Pure translations give degenerate
> solutions — always tilt the gripper at different angles between samples.
> Only take samples when ChArUco detection looks clean and reprojection
> error is low.

### Auto-Calibration

Drives the arm through preconfigured joint target poses from
`config/calibration_poses.yaml` and captures a sample at each one. Useful
as a **sanity check**. It is typically less accurate than manual collection
because of servo repeatability and lighting sensitivity. Click
**🤖 Auto-Calibrate** in the UI — make sure the workspace is clear first.

### Applying the Result

The calibration outputs the optical-frame transform. The URDF chain is:

```
base_link ──(A)──▸ static_camera_link ──(B)──▸ static_camera_optical_frame
```

- **(A)** = `cam_static_xyz / cam_static_rpy` (what you pass to the launch file)
- **(B)** = fixed URDF rotation `rpy=(-π/2, 0, -π/2)`, **zero translation**

Calibration gives `A × B`. To recover `A`:

```python
import numpy as np
from scipy.spatial.transform import Rotation

# Replace with your calibration result (optical frame)
t = [0.1752, 0.0255, 0.5618]
rpy_deg = [-176.0, -1.0, -87.2]

T_cal = np.eye(4)
T_cal[:3, :3] = Rotation.from_euler('xyz', rpy_deg, degrees=True).as_matrix()
T_cal[:3, 3] = t

T_opt = np.eye(4)
T_opt[:3, :3] = Rotation.from_euler('xyz', [-np.pi/2, 0, -np.pi/2]).as_matrix()

T_link = T_cal @ np.linalg.inv(T_opt)
print(f'cam_static_xyz:="{T_link[0,3]:.4f} {T_link[1,3]:.4f} {T_link[2,3]:.4f}"')
rpy = Rotation.from_matrix(T_link[:3,:3]).as_euler('xyz')
print(f'cam_static_rpy:="{rpy[0]:.4f} {rpy[1]:.4f} {rpy[2]:.4f}"')
```

Because `T_opt` has **zero translation**, this conversion changes only the
rotation — the `xyz` is identical before and after.

Then pass to the launch file:

```bash
ros2 launch so101_bringup follower_vision.launch.py \
  cam_static_xyz:="0.1752 0.0255 0.5618" \
  cam_static_rpy:="-0.2452 1.4988 -0.1957"
```

#### Gimbal Lock Note

The `static_camera_link` pitch is near 90°, which causes **gimbal lock** —
roll and yaw Euler angles become unstable across runs even when the actual
rotation is nearly identical. Example from two calibration runs:

| Run             | Optical-frame rpy   | `static_camera_link` rpy | Rotation diff |
|-----------------|---------------------|--------------------------|---------------|
| v1 (27 samples) | (-176°, -1°, -87°)  | (-14°, 86°, -11°)        | —             |
| v2 (24 samples) | (-176°,  2°, -90°)  | ( 20°, 86°,  20°)        | **3.6°**      |

Roll and yaw swing by ~30° but the underlying rotations differ by only 3.6°.
The optical-frame rpy is near pitch = 0° and stays stable, while the
`static_camera_link` rpy is near pitch ≈ 90° and is not. TF uses quaternions
internally, so gimbal lock does not affect runtime behaviour.

> Use the result from a single good calibration run. **Do not average
> Euler angles near pitch ≈ 90°.**

## Printing a Target

Two generators are included — one per calibration type.

**Hand-eye target** (small, attaches to the gripper):

```bash
python3 scripts/gen_charuco_handeye.py
```

Creates `/tmp/charuco_handeye_A4.png` + `.pdf` (A4, 300 DPI, 4×5 squares at
**15 mm**, `DICT_4X4_50`). Attach to the gripper facing the camera.

**Intrinsic target** (larger and denser, covers the image better):

```bash
python3 scripts/gen_charuco_intrinsic.py
```

Creates `/tmp/charuco_intrinsic_A4.png` + `.pdf` (A4, 300 DPI, 8×6 squares at
**25 mm** (18 mm markers), `DICT_5X5_250`). Use it handheld — move it around
the image to cover corners/edges and vary tilt.

Both: print at **actual size** (no scale-to-fit). Verify with a ruler that
one square matches the expected millimetres. The node parameters
(`squares_x/y`, `square_length`, `marker_length`, `aruco_dict`) must match
the generated board.

## Package Structure

```
so101_camera_calibration/
├── config/
│   ├── calibration_poses.yaml          # Joint poses for auto-calibrate
│   ├── so101_eye_on_base.yaml          # Reference calibration result
│   └── so101_eye_on_base.samples.yaml  # Reference samples
├── launch/
│   └── handeye_calibration.launch.py
├── scripts/
│   ├── gen_charuco_handeye.py
│   └── gen_charuco_intrinsic.py
└── so101_camera_calibration/
    ├── camera_intrinsic_calibration_node.py
    └── handeye_calibration_node.py
```

`cartesian_motion_node` (the IK/trajectory service used by the hand-eye
launch file) now lives in [`so101_kinematics`](../so101_kinematics), with
its services in [`so101_kinematics_msgs`](../so101_kinematics_msgs).

## Tips

- **Lighting matters.** Use consistent artificial light. Matte-print the
  ChArUco board to avoid specular reflections.
- **Rotation variety > position variety.** Tilt the gripper at different
  angles between samples.
- **Trust Park and Horaud.** Tsai-Lenz often gets depth wrong; Andreff and
  Daniilidis can be unstable. If Park and Horaud agree within ~1 cm, the
  calibration is usually in good shape.
- **Typical values** for a camera ~60 cm above the base:
  `z ≈ 0.55–0.60 m`, `x ≈ 0.15–0.20 m` (mount-dependent).

## Troubleshooting

- **No green corners detected** — improve lighting, flatten the print, or
  move the board closer.
- **Coverage grid stays incomplete** — push the board into the image corners
  and vary tilt more aggressively.
- **Park and Horaud disagree strongly** — collect more samples with more
  rotational diversity.
- **Auto-calibration results are inconsistent** — switch to manual calibration;
  servo repeatability is the likely culprit.
