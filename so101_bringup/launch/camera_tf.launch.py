from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    root = "world"

    return LaunchDescription(
        [
            # Overhead camera pose relative to follower base
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_overhead_cam",
                arguments=[
                    "-0.15", "0.0", "0.80",   # x y z (meters)
                    "0.0", "1.0", "0.0",   # yaw pitch roll (radians)
                    root,
                    "static_camera/cam_overhead",
                ],
            ),

            # Wrist camera pose relative to end-effector link
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_wrist_cam",
                arguments=[
                    "0.00", "0.0", "-0.02",
                    "-1.57", "0.0", "-1.57", 
                    "follower/moving_jaw_so101_v1_link", 
                    "follower/cam_wrist",
                ],
            ),
        ]
    )
