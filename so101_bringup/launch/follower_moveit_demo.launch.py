import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    hardware_type = LaunchConfiguration("hardware_type")
    namespace = LaunchConfiguration("namespace")

    use_sim_time = PythonExpression(["'", hardware_type, "' == 'mujoco'"])

    # 1) Bringup (ros2_control + rsp + spawners) - your existing follower.launch.py
    follower_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("so101_bringup"),
                "launch",
                "follower_split.launch.py",
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "hardware_type": hardware_type,
            "use_rviz": "false",  # MoveIt RViz is launched separately below
            "use_sim_time": use_sim_time,  
        }.items(),
    )

    # 2) Move group (pure MoveIt)
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("so101_moveit_config"),
                "launch",
                "move_group.launch.py",
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "variant": "follower",
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # 3) MoveIt RViz 
    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("so101_moveit_config"),
                "launch",
                "moveit_rviz.launch.py",
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "variant": "follower",
        }.items()
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware_type", default_value="real"),  # real|mock_components|mujoco
            DeclareLaunchArgument("namespace", default_value="follower"),
            follower_bringup,
            move_group,
            moveit_rviz,
        ]
    )
