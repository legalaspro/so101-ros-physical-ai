"""
Live xacro editing for both follower and leader arms side by side.

Usage:
  ros2 launch so101_description display_live_duo.launch.py

Then edit xacro files in src/so101_description/urdf/ and save - RViz updates automatically!
"""
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def process_xacro(xacro_path: str, variant: str) -> str:
    """Process xacro file with variant argument."""
    result = subprocess.run(
        ['xacro', xacro_path, f'variant:={variant}'],
        capture_output=True,
        text=True,
        check=True,
    )
    return result.stdout


def launch_setup(context):
    pkg_share = get_package_share_directory('so101_description')
    ros_ws = os.environ.get('ROS_WS', '/ros2_ws')

    # Use source xacro path for watching
    src_xacro_path = os.path.join(
        ros_ws, 'src', 'so101_description', 'urdf', 'so101_arm.urdf.xacro'
    )
    installed_xacro_path = os.path.join(pkg_share, 'urdf', 'so101_arm.urdf.xacro')
    xacro_path = src_xacro_path if os.path.exists(src_xacro_path) else installed_xacro_path

    nodes = []

    for variant, offset_y in [('follower', 0.0), ('leader', 0.4)]:
        # Process xacro to get robot_description
        robot_description = process_xacro(xacro_path, variant)

        # Robot State Publisher with namespace
        nodes.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=variant,
            parameters=[{
                'robot_description': robot_description,
                'frame_prefix': f'{variant}/',
            }],
            output='screen',
        ))

        # Joint State Publisher GUI with namespace
        nodes.append(Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            namespace=variant,
            name=f'{variant}_joint_state_publisher_gui',
        ))

        # Live xacro reload node for this arm
        nodes.append(Node(
            package='so101_description',
            executable='live_urdf_reload.py',
            namespace=variant,
            name=f'{variant}_live_reload',
            parameters=[{
                'urdf_path': xacro_path,
                'xacro_args': f'variant:={variant}',
                'watch_interval': 0.5,
                'target_node': f'/{variant}/robot_state_publisher',
            }],
            output='screen',
        ))

        # Static transform to offset the robots
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace=variant,
            name=f'{variant}_world_tf',
            arguments=['0', str(offset_y), '0', '0', '0', '0', 'world', f'{variant}/base_link'],
        ))

    # RViz (single instance)
    nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'display_duo.rviz')],
    ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])

