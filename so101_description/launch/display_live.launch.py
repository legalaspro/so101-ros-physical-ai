"""
Live URDF/xacro editing with auto-reload in RViz.

Usage:
  ros2 launch so101_description display_live.launch.py variant:=follower
  ros2 launch so101_description display_live.launch.py variant:=leader

Then edit xacro files in src/so101_description/urdf/ and save - RViz updates automatically!
"""
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
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
    variant = LaunchConfiguration('variant').perform(context)
    ros_ws = os.environ.get('ROS_WS', '/ros2_ws')

    # Use source xacro path for watching
    src_xacro_path = os.path.join(
        ros_ws, 'src', 'so101_description', 'urdf', 'so101_arm.urdf.xacro'
    )
    installed_xacro_path = os.path.join(pkg_share, 'urdf', 'so101_arm.urdf.xacro')
    xacro_path = src_xacro_path if os.path.exists(src_xacro_path) else installed_xacro_path

    # Process xacro to get initial robot_description
    robot_description = process_xacro(xacro_path, variant)

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # Joint State Publisher GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'display.rviz')],
    )

    # Live URDF reload node - watches xacro files and republishes on changes
    live_reload_node = Node(
        package='so101_description',
        executable='live_urdf_reload.py',
        parameters=[{
            'urdf_path': xacro_path,
            'xacro_args': f'variant:={variant}',
            'watch_interval': 0.5,
            'target_node': '/robot_state_publisher',
        }],
        output='screen',
    )

    return [
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
        live_reload_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'variant',
            default_value='follower',
            choices=['follower', 'leader'],
            description='Which robot variant to edit live',
        ),
        OpaqueFunction(function=launch_setup),
    ])

