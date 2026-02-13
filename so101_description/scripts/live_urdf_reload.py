#!/usr/bin/env python3
"""
Live URDF Reload Node - watches URDF/xacro files and republishes robot_description on changes.

Usage:
  ros2 run so101_description live_urdf_reload.py --ros-args -p urdf_path:=/path/to/robot.urdf
  ros2 run so101_description live_urdf_reload.py --ros-args -p urdf_path:=/path/to/robot.xacro -p xacro_args:="variant:=leader"

Or with launch file:
  ros2 launch so101_description display_live.launch.py variant:=leader
"""
import os
import subprocess

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


class LiveUrdfReload(Node):
    def __init__(self):
        super().__init__('live_urdf_reload')

        # Parameters
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('xacro_args', '')  # e.g. "variant:=leader prefix:=left_"
        self.declare_parameter('watch_interval', 0.5)  # seconds
        self.declare_parameter('target_node', '/robot_state_publisher')

        self.urdf_path = self.get_parameter('urdf_path').get_parameter_value().string_value
        self.xacro_args = self.get_parameter('xacro_args').get_parameter_value().string_value
        self.watch_interval = self.get_parameter('watch_interval').get_parameter_value().double_value
        self.target_node = self.get_parameter('target_node').get_parameter_value().string_value

        # Watch directory for included xacro files
        self.watch_dir = os.path.dirname(self.urdf_path)

        if not self.urdf_path:
            self.get_logger().error('urdf_path parameter is required!')
            return

        if not os.path.exists(self.urdf_path):
            self.get_logger().error(f'URDF/xacro file not found: {self.urdf_path}')
            return

        self.get_logger().info(f'Watching: {self.urdf_path}')
        if self.xacro_args:
            self.get_logger().info(f'Xacro args: {self.xacro_args}')
        self.get_logger().info(f'Target node: {self.target_node}')

        # Track file modification times for all xacro/urdf files in directory
        self.last_mtime = self._get_latest_mtime()
        self.last_content = self._read_urdf()

        # Service client to set parameters on robot_state_publisher
        self.param_client = self.create_client(
            SetParameters,
            f'{self.target_node}/set_parameters'
        )

        # Timer to check for file changes
        self.timer = self.create_timer(self.watch_interval, self.check_for_changes)

        self.get_logger().info('Live URDF reload ready! Edit and save your xacro/URDF to see changes in RViz.')

    def _get_latest_mtime(self) -> float:
        """Get the latest modification time of all xacro/urdf files in watch directory."""
        latest = 0.0
        for root, _, files in os.walk(self.watch_dir):
            for f in files:
                if f.endswith(('.xacro', '.urdf')):
                    mtime = os.path.getmtime(os.path.join(root, f))
                    latest = max(latest, mtime)
        return latest

    def _read_urdf(self) -> str:
        """Process file with xacro (works for both .urdf and .xacro files)."""
        try:
            cmd = ['xacro', self.urdf_path]
            # Add xacro arguments if provided
            if self.xacro_args:
                cmd.extend(self.xacro_args.split())

            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                check=True,
            )
            return result.stdout
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'xacro failed: {e.stderr}')
            return ''
        except Exception as e:
            self.get_logger().error(f'Failed to process URDF: {e}')
            return ''

    def check_for_changes(self):
        """Check if any xacro/URDF file has been modified."""
        try:
            current_mtime = self._get_latest_mtime()
            if current_mtime > self.last_mtime:
                self.last_mtime = current_mtime

                # Read new content
                new_content = self._read_urdf()
                if new_content and new_content != self.last_content:
                    self.last_content = new_content
                    self.get_logger().info('Xacro/URDF changed, reloading...')
                    self.reload_urdf(new_content)
        except Exception as e:
            self.get_logger().warn(f'Error checking file: {e}')

    def reload_urdf(self, urdf_content: str):
        """Send new robot_description to robot_state_publisher."""
        if not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'Service {self.target_node}/set_parameters not available')
            return

        # Create parameter request
        param = Parameter()
        param.name = 'robot_description'
        param.value = ParameterValue()
        param.value.type = ParameterType.PARAMETER_STRING
        param.value.string_value = urdf_content

        request = SetParameters.Request()
        request.parameters = [param]

        future = self.param_client.call_async(request)
        future.add_done_callback(self._reload_callback)

    def _reload_callback(self, future):
        """Handle reload result."""
        try:
            result = future.result()
            if result.results[0].successful:
                self.get_logger().info('✓ URDF reloaded successfully!')
            else:
                self.get_logger().error(f'Failed to reload: {result.results[0].reason}')
        except Exception as e:
            self.get_logger().error(f'Reload error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LiveUrdfReload()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

