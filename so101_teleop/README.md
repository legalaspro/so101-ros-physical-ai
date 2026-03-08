# so101_teleop

**Video:** https://www.youtube.com/watch?v=wLBEH63x_nA

<p>
  <a href="https://www.youtube.com/watch?v=wLBEH63x_nA">
    <img src="../docs/assets/gifs/so101_teleop.gif" alt="SO-101 teleop demo" height="240" />
  </a>
</p>

Leader-to-follower teleoperation package for the SO-101 arm. It subscribes to the leader `/joint_states` topic and sends follower commands either as a `JointTrajectory` or as forward position commands.

## Quick start

Recommended full-stack launch:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch so101_bringup teleop.launch.py
```

This uses `forward_controller` (ROS 2 `ForwardCommandController`) by default, and that is the recommended mode.

Optional override if you want trajectory commands instead:

```bash
ros2 launch so101_bringup teleop.launch.py arm_controller:=trajectory_controller
```

## Package-only launch

Use this only if the leader and follower stacks are already running:

```bash
ros2 launch so101_teleop teleop.launch.py
```

Optional split arm/gripper variant:

```bash
ros2 launch so101_teleop teleop_split.launch.py
```

## Main files

- `launch/teleop.launch.py` — standard teleop node
- `launch/teleop_split.launch.py` — split arm + gripper teleop node
- `config/teleop.yaml` — publish rate, stale timeout, joint list
- `config/teleop_split.yaml` — split arm/gripper parameters
- `src/teleop.cpp` — standard follower command relay
- `src/teleop_split.cpp` — arm teleop + gripper action client

## Useful launch args

- `leader_namespace` — default: `leader`
- `follower_namespace` — default: `follower`
- `arm_controller` — default and recommended: `forward_controller` (ROS 2 `ForwardCommandController`); optional: `trajectory_controller`
- `params_file` — custom teleop parameter file