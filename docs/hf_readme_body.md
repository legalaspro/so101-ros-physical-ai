## Dataset Description

- **Homepage:** https://github.com/legalaspro/so101-ros-physical-ai
- **License:** apache-2.0

SO-101 pick-and-place demonstration episodes recorded in ROS 2 Jazzy and stored as rosbag2 (MCAP).  
Converted into LeRobot format using the `rosbag_to_lerobot` converter from:
https://github.com/legalaspro/so101-ros-physical-ai

### Data includes
- Wrist camera video: `observation.images.wrist`
- Top/static camera video: `observation.images.top`
- Joint positions: `observation.state` (6-DoF, `.pos`)
- Position commands: `action` (6-DoF position commands)

### Intended use

Behavior cloning / imitation learning baselines, dataset tooling tests, and ROS→LeRobot conversion examples.

