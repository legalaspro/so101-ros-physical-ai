import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    variant = LaunchConfiguration("variant")

    xacro_path = os.path.join(
        get_package_share_directory("so101_description"),
        "urdf",
        "so101_arm.urdf.xacro",
    )

    moveit_config = (
        MoveItConfigsBuilder("so101_arm", package_name="so101_moveit_config")
        .robot_description(
            file_path=xacro_path,
            mappings={
                "variant": variant,
                "use_ros2_control": "false",
            },
        )
        .robot_description_semantic() # uses your SRDF in so101_moveit_config
        .robot_description_kinematics()
        .planning_pipelines(pipelines=["ompl"]) # loads ompl_planning.yaml
        .joint_limits()
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False, # don't let MoveIt switch controllers
        )
        .to_moveit_configs()
    )

    # Run in root namespace (MoveIt + namespaces is buggy, all reference
    # projects do this).  Remap joint_states so move_group finds the
    # topic published inside the controller namespace.
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time}],
        remappings=[("joint_states", ["/", namespace, "/joint_states"])],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="follower"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("variant", default_value="follower"),
            move_group_node,
        ]
    )