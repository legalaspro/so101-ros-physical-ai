import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _spawn_cameras(context):
    pkg = LaunchConfiguration("bringup_pkg").perform(context)
    cameras_cfg = LaunchConfiguration("cameras_config").perform(context)

    pkg_share = get_package_share_directory(pkg)

    with open(cameras_cfg, "r") as f:
        cfg = yaml.safe_load(f) or {}

    nodes = []
    for cam in cfg.get("cameras", []):
        name = cam["name"]
        ns = cam.get("namespace", "")
        cam_type = cam["camera_type"]
        param_path = cam["param_path"]

        param_file = (
            param_path
            if os.path.isabs(param_path)
            else os.path.join(pkg_share, "config", "cameras", param_path)
        )

        if cam_type == "v4l2_camera":
            nodes.append(
                Node(
                    package="v4l2_camera",
                    executable="v4l2_camera_node",
                    name=name,
                    namespace=ns,
                    parameters=[param_file, {"use_sim_time": False}],
                    output="screen",
                )
            )
        elif cam_type == "libcam":
            nodes.append(
                Node(
                    package="camera_ros",
                    executable="camera_node",
                    name=name,
                    namespace=ns,
                    parameters=[param_file, {"use_sim_time": False}],
                    output="screen",
                    remappings=[
                        ("~/image_raw", "image_raw"),
                        ("~/camera_info", "camera_info"),
                        ("~/image_raw/compressed", "image_raw/compressed"),
                    ],
                )
            )
        elif cam_type == "gscam":
            nodes.append(
                Node(
                    package="gscam",
                    executable="gscam_node",
                    name=name,
                    namespace=ns,
                    parameters=[param_file, {"use_sim_time": False}],
                    output="screen",
                    remappings=[
                        ("camera/image_raw", "image_raw"),
                        ("camera/camera_info", "camera_info"),
                        ("camera/image_raw/compressed", "image_raw/compressed"),
                    ],
                )
            )
        elif cam_type == "usb_camera":
            nodes.append(
                Node(
                    package="usb_cam",
                    executable="usb_cam_node_exe",
                    name=name,
                    namespace=ns,
                    parameters=[param_file, {"use_sim_time": False}],
                    output="screen",
                )
            )
        elif cam_type == "realsense2_camera":
            nodes.append(
                Node(
                    package="realsense2_camera",
                    executable="realsense2_camera_node",
                    name=name,
                    namespace=ns,
                    parameters=[param_file, {"use_sim_time": False}],
                    output="screen",
                )
            )
        else:
            raise RuntimeError(f"Unsupported camera_type: {cam_type}")

    return nodes


def generate_launch_description():
    bringup_pkg = "so101_bringup"

    return LaunchDescription(
        [
            DeclareLaunchArgument("bringup_pkg", default_value=bringup_pkg),
            DeclareLaunchArgument(
                "cameras_config",
                default_value=os.path.join(
                    get_package_share_directory(bringup_pkg),
                    "config",
                    "cameras",
                    "so101_cameras.yaml",
                ),
            ),
            OpaqueFunction(function=_spawn_cameras),
        ]
    )
