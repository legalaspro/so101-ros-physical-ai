# Hardware Setup

> **Required before running on real hardware.**
> The config files shipped in this repo are examples from one specific robot and **will not match yours**.

This document focuses on **ROS-side hardware integration**:

- stable device naming (serial + cameras),
- permissions,
- LeRobot motor setup and calibration as prerequisites,
- optional ROS-side joint overrides when you explicitly want them.

---

## 1. Motor Setup (One-Time per Arm)

Each servo must have a unique ID and correct baudrate written to EEPROM.
Follow the official [LeRobot SO-101 guide](https://huggingface.co/docs/lerobot/so101) for:

- **setup motors** (IDs / baudrate)
- **calibration** (offsets / limits)

> This repo assumes your servos are already configured and responding correctly.

---

## 2. Identify Devices (Quick)

Plug in the devices and confirm the kernel sees them:

```bash
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true
ls -l /dev/video* 2>/dev/null || true
```

---

## 3. Udev Rules (Recommended)

This stack assumes stable device symlinks created by udev:

| Device          | Default path          |
| --------------- | --------------------- |
| Leader arm      | `/dev/so101_leader`   |
| Follower arm    | `/dev/so101_follower` |
| Wrist camera    | `/dev/cam_wrist`      |
| Overhead camera | `/dev/cam_overhead`   |

### 3.1 Query Device Properties

```bash
# Arms — replace /dev/ttyACM0 with the device you see:
udevadm info --query=property --name=/dev/ttyACM0 | \
  egrep 'ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT|ID_PATH'
```

```bash
# Cameras:
ls -l /dev/v4l/by-id/
# pick the node you want, then:
udevadm info --query=property --name=/dev/videoX | \
  egrep 'ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT|ID_PATH'
```

> If `ID_SERIAL_SHORT` is missing (some devices), match on `ID_PATH` instead.

### 3.2 Edit the Example Rules File

Template: [`docs/assets/99-so101.rules.example`](assets/99-so101.rules.example).
Replace placeholders with values from 3.1.

**Arms (tty):**

```
SUBSYSTEM=="tty", ENV{ID_VENDOR_ID}=="XXXX", ENV{ID_MODEL_ID}=="YYYY", ENV{ID_SERIAL_SHORT}=="SERIAL_LEADER",   SYMLINK+="so101_leader",   GROUP="dialout", MODE="0660"
SUBSYSTEM=="tty", ENV{ID_VENDOR_ID}=="XXXX", ENV{ID_MODEL_ID}=="YYYY", ENV{ID_SERIAL_SHORT}=="SERIAL_FOLLOWER", SYMLINK+="so101_follower", GROUP="dialout", MODE="0660"
```

**Cameras (video4linux):**

```
ACTION=="add|change", SUBSYSTEM=="video4linux", KERNEL=="video*", ENV{ID_SERIAL_SHORT}=="SERIAL_WRIST",    ATTR{index}=="0", SYMLINK+="cam_wrist",    GROUP="video", MODE="0660"
ACTION=="add|change", SUBSYSTEM=="video4linux", KERNEL=="video*", ENV{ID_SERIAL_SHORT}=="SERIAL_OVERHEAD", ATTR{index}=="0", SYMLINK+="cam_overhead", GROUP="video", MODE="0660"
```

> Many USB cameras expose multiple `/dev/video*` devices (video + metadata).
> `ATTR{index}=="0"` selects the main video stream.

### 3.3 Install and Reload

```bash
sudo cp docs/assets/99-so101.rules.example /etc/udev/rules.d/99-so101.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Verify:

```bash
ls -l /dev/so101_leader /dev/so101_follower
ls -l /dev/cam_wrist /dev/cam_overhead   # if you added camera rules
```

---

## 4. Permissions (dialout + video)

```bash
sudo usermod -aG dialout,video $USER
```

Log out / in (or reboot), then verify:

```bash
groups | grep -E 'dialout|video'
```

> `dialout` is needed for serial ports (arms), `video` for cameras.
> With the udev rules above (`GROUP="dialout"` / `GROUP="video"`, `MODE="0660"`),
> you should not need `sudo` or `chmod` hacks.

---

## 5. Calibration, EEPROM, and Optional Joint Config Overrides

Before using the real arms from ROS, complete the LeRobot motor setup and calibration steps for both arms. Those steps write the required persistent values to the servo motors, including IDs and calibration-related settings stored in EEPROM.

Servo motors keep persistent values in EEPROM, so those settings survive power cycles and only change when a tool explicitly writes new ones. That means calibration values already stored on the motors can remain the source of truth instead of being duplicated into multiple ROS config files.

### Parameter precedence

**Precedence:** `joint_config_file` overrides `URDF/Xacro` defaults.

On initialization, the driver writes the resulting values for supported parameters to the motors. If you provide a `joint_config_file`, those values override the URDF/Xacro defaults and replace the older stored values for the same registers.

If you use a `joint_config_file`, each joint entry must:

- include the correct `id`
- match a joint defined in the `ros2_control` / xacro description

Common parameters:

- `id`: motor ID on the bus
- `homing_offset`: joint zero alignment, written to the servo EEPROM
- `range_min` / `range_max`: joint travel limits, written to the servo EEPROM
- `p_coefficient` / `i_coefficient` / `d_coefficient`, `return_delay_time`, `max_torque_limit`, `protection_current`, `overload_torque`: optional tuning and protection settings written to the servo EEPROM
- `acceleration`: optional motion parameter used by the driver and not part of LeRobot calibration output

For the follower gripper, this project sets these protection values by default in `so101_description/urdf/ros2_control/so101_ros2_control.xacro` to reduce the risk of overloading or damaging the motor:

- `max_torque_limit: 500`
- `protection_current: 250`
- `overload_torque: 25`

LeRobot calibration does not produce these safety values. If needed, you can still override them per robot through `joint_config_file`.

Use `joint_config_file` when you want explicit per-robot overrides, to reapply known-good values during bringup, or to set extra tuning/protection parameters not produced by LeRobot.

Optional override examples live here:

```text
so101_bringup/config/hardware/
├── leader_joints.yaml
├── follower_joints.yaml
├── lerobot_leader_arm.json
└── lerobot_follower_arm.json
```

The YAML files in this repo are examples of override files, while the included `lerobot_*.json` files show raw LeRobot calibration output for reference.

---

## 6. Camera Configuration

Camera config: `so101_bringup/config/cameras/so101_usb_cam.yaml`

Set `video_device` to your udev symlink:

```yaml
/follower/cam_wrist:
  ros__parameters:
    video_device: "/dev/cam_wrist"
```

---

## 7. Verification Checklist

Before launching teleop:

- [ ] LeRobot motor setup and calibration were completed for both arms before first ROS use
- [ ] Leader / follower udev symlinks exist
- [ ] User is in `dialout` and `video` groups
- [ ] If using `joint_config_file`, it points to the intended override YAML with correct joint IDs
- [ ] Camera `video_device` paths are correct (if using cameras)

Sanity checks:

```bash
ls -l /dev/so101_leader /dev/so101_follower 2>/dev/null || true
ls -l /dev/cam_wrist /dev/cam_overhead 2>/dev/null || true
```

Then run:

```bash
ros2 launch so101_bringup teleop.launch.py hardware_type:=real
```
