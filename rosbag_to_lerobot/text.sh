ros2 run rosbag_to_lerobot convert \
  --input-dir ~/.ros/so101_episodes/pick_and_place \
  --output-dir ~/.lerobot/so101_lerobot_ds \
  --config ~/ros2_ws/src/so101-ros-physical-ai/config/so101.yaml \
  --repo-id local/so101_test \
  --no-push-hub

python -m rosbag_to_lerobot.convert \
  --input-dir ~/.ros/so101_episodes/pick_and_place \
  --output-dir ~/.lerobot/so101_lerobot_ds \
  --config ~/ros2_ws/src/so101-ros-physical-ai/config/so101.yaml \
  --repo-id local/so101_test \
  --no-push-hub


pixi run -e lerobot -- python -m rosbag_to_lerobot.convert \
  --input-dir ~/.ros/so101_episodes/pick_and_place \
  --output-dir ~/.lerobot/so101_lerobot_ds \
  --config ~/ros2_ws/src/so101-ros-physical-ai/config/so101.yaml \
  --repo-id local/so101_test \
  --no-push-hub

  pixi run -e lerobot -- python -c "
from lerobot.datasets.lerobot_dataset import LeRobotDataset
ds = LeRobotDataset('~/.lerobot/so101_lerobot_ds')
print(ds)
print('✅ Cameras:', list(ds.meta['cameras'].keys()))
print('✅ Episodes:', ds.num_episodes)
"


python -m rosbag_to_lerobot.cli  \
  --input-dir  /home/dmitri/.ros/so101_episodes/pick_and_place_2 \
  --output-dir  /home/dmitri/.lerobot/so101_lerobot_ds \
  --config ~/ros2_ws/src/so101-ros-physical-ai/rosbag_to_lerobot/config/so101.yaml \
  --repo-id local/so101_test