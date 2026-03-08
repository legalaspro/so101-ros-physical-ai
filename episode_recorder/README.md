# episode_recorder

**Video:** https://www.youtube.com/watch?v=lXkloZll4PA

<p>
  <a href="https://www.youtube.com/watch?v=lXkloZll4PA">
    <img src="../docs/assets/gifs/ros2_episode_recorder.gif" alt="ROS 2 episode recorder demo" height="240" />
  </a>
</p>

Minimalistic ROS 2 episode recorder for imitation learning. It records configurable topics into rosbag2 episodes (MCAP by default) and supports keyboard-driven start / stop / discard control.

## Quick start

Recommended full-stack launch:

```bash
ros2 launch so101_bringup recording_session.launch.py \
  experiment_name:=pick_and_place \
  task:="Pick up the cube and place it in the container." \
  use_rerun:=true
```

`use_rerun:=true` is recommended if you want to see the recording live during the session.

In a second terminal, run the keyboard controller:

```bash
ros2 run episode_recorder teleop_episode_keyboard
```

Episodes are saved under `~/.ros/so101_episodes/` by default.

## Package-only launch

Use this if the robot stack is already running and you only want the recorder:

```bash
ros2 launch episode_recorder recorder.launch.py \
  experiment_name:=pick_and_place \
  task:="Pick up the cube and place it in the container."
```

## Keyboard controls

- `r` or `→` — start recording
- `s` or `←` — stop and save
- `d` or `Backspace` — discard current episode
- `t` — edit the recorder `task` parameter
- `h` — help
- `q` — quit

## Main files

- `launch/recorder.launch.py` — lifecycle recorder launch with auto-configure and auto-activate
- `config/default_config.yaml` — topics, storage backend, default experiment settings
- `src/episode_recorder.cpp` — recorder lifecycle node and bag writing logic
- `src/teleop_episode_keyboard.cpp` — interactive keyboard client for start / stop / discard

## Useful launch args

- `params_file` — YAML config file for topics and storage settings
- `root_dir` — default output root, usually `~/.ros/so101_episodes`
- `experiment_name` — subfolder under `root_dir`
- `task` — task label stored in rosbag metadata
- `recorder_ns` — optional recorder namespace