# Hardware Setup

> **Required before running on real hardware.**
> The config files shipped in this repo are examples from one specific robot and **will not match yours**.

This document focuses on **ROS-side hardware integration**:

- stable device naming (serial + cameras),
- permissions,
- transferring LeRobot calibration into this stack.

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

## 5. Calibration → ROS Hardware YAML (Required per Robot)

LeRobot calibration produces one JSON file per arm containing per-joint:
`id`, `homing_offset`, `range_min`, `range_max`.

Transfer these into:

| Source (LeRobot JSON)     | Target (ROS YAML)                                    |
| ------------------------- | ---------------------------------------------------- |
| leader calibration JSON   | `so101_bringup/config/hardware/leader_joints.yaml`   |
| follower calibration JSON | `so101_bringup/config/hardware/follower_joints.yaml` |

LeRobot JSON output (one joint shown):

```json
"shoulder_pan": {
    "id": 1,
    "drive_mode": 0,
    "homing_offset": -732,
    "range_min": 821,
    "range_max": 3267
}
```

ROS YAML schema:

```yaml
joints:
  shoulder_pan:
    id: 1
    homing_offset: -732
    range_min: 821
    range_max: 3267
    return_delay_time: 0
    acceleration: 254
```

The YAML files in this repo are **examples**. Replace values with your calibration output.

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

- [ ] Leader / follower udev symlinks exist
- [ ] User is in `dialout` and `video` groups
- [ ] `leader_joints.yaml` and `follower_joints.yaml` contain **your** calibration values
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
