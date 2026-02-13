from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    leader_ns = LaunchConfiguration("leader_namespace")
    follower_ns = LaunchConfiguration("follower_namespace")
    arm_controller = LaunchConfiguration("arm_controller")  
    params_file = LaunchConfiguration("params_file")

    default_params = PathJoinSubstitution(
        [FindPackageShare("so101_teleop"), "config", "teleop_split.yaml"]
    )

    # Derived topics from namespaces
    leader_topic = PythonExpression(["'/' + '", leader_ns, "' + '/joint_states'"])
    jtc_topic = PythonExpression(["'/' + '", follower_ns, "' + '/arm_trajectory_controller/joint_trajectory'"])
    fwd_topic = PythonExpression(["'/' + '", follower_ns, "' + '/arm_forward_controller/commands'"])
    gripper_action = PythonExpression(["'/' + '", follower_ns, "' + '/gripper_controller/gripper_cmd'"])

    # Map controller choice to node param
    arm_mode = PythonExpression([
        "'joint_trajectory' if '", arm_controller, "' == 'arm_trajectory_controller' else 'forward_position'"
    ])

    return LaunchDescription([
        DeclareLaunchArgument("leader_namespace", default_value="leader"),
        DeclareLaunchArgument("follower_namespace", default_value="follower"),
        DeclareLaunchArgument("arm_controller", default_value="arm_trajectory_controller"),  # arm_trajectory_controller|arm_forward_controller
        DeclareLaunchArgument("params_file", default_value=default_params),

        Node(
            package="so101_teleop",
            executable="teleop_split",
            name="arm_gripper_teleop",
            output="screen",
            parameters=[
                params_file,
                {
                    "arm_mode": arm_mode,
                    "leader_topic": leader_topic,
                    "jtc_topic": jtc_topic,
                    "fwd_topic": fwd_topic,
                    "gripper_action": gripper_action,
                },
            ],
        ),
    ])
