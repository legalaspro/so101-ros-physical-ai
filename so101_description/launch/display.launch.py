import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    so101_description_share_dir = get_package_share_directory('so101_description')

    # Declare arguments
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Use joint_state_publisher_gui instead of joint_state_publisher',
    )

    display_config_arg = DeclareLaunchArgument(
        'display_config',
        default_value=os.path.join(so101_description_share_dir, 'rviz', 'display.rviz'),
        description='Path to the RViz display config file',
    )

    gui = LaunchConfiguration('gui')
    display_config = LaunchConfiguration('display_config')

    robot_desc_path = os.path.join(
        so101_description_share_dir, "urdf", "so101_new_calib.urdf"
    )
    robot_desc = xacro.process_file(robot_desc_path)
    robot_description_xml = robot_desc.toxml()

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description_xml}],
        output="screen",
    )

    # Joint State Publisher (non-GUI) - used when gui:=false
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(gui),
    )

    # Joint State Publisher GUI - used when gui:=true
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(gui),
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', display_config],
    )

    return LaunchDescription(
        [
            gui_arg,
            display_config_arg,
            robot_state_publisher_node,
            joint_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node,
        ]
    )
