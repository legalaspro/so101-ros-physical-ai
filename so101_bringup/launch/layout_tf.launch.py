from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # positions in meters
    leader_x = LaunchConfiguration("leader_x")
    leader_y = LaunchConfiguration("leader_y")
    leader_z = LaunchConfiguration("leader_z")

    follower_x = LaunchConfiguration("follower_x")
    follower_y = LaunchConfiguration("follower_y")
    follower_z = LaunchConfiguration("follower_z")

    world_frame = LaunchConfiguration("world_frame")

    return LaunchDescription([
        DeclareLaunchArgument("world_frame", default_value="world"),

        DeclareLaunchArgument("follower_x", default_value="0.0"),
        DeclareLaunchArgument("follower_y", default_value="0.0"),
        DeclareLaunchArgument("follower_z", default_value="0.0"),

        DeclareLaunchArgument("leader_x", default_value="-0.5"),
        DeclareLaunchArgument("leader_y", default_value="-0.5"),
        DeclareLaunchArgument("leader_z", default_value="0.0"),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_follower_base",
            arguments=[
                follower_x, follower_y, follower_z, 
                "0.0", "0.0", "0.0",
                world_frame, 
                "follower/base_link"],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_leader_base",
            arguments=[
                leader_x, leader_y, leader_z, 
                "1.57", "0.0", "0.0",
                world_frame, 
                "leader/base_link"],
        ),
    ])

# x y z  yaw pitch roll  parent_frame  child_frame
