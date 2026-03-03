cd ~/ros2_ws/ && colcon build && source install/setup.bash && cd ~/ros2_ws/src/so101-ros-physical-ai

pixi run -e lerobot infer
pixi run -e lerobot async_infer -- --ros-args -p server_address:=145.236.164.238:24410 -p fps:=50.0 \
    -p chunk_size_threshold:=0.5 -p aggregate_fn_name:=weighted_average


