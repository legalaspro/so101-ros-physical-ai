from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    # --- Launch arguments ---
    namespace = LaunchConfiguration("namespace")
    usb_port = LaunchConfiguration("usb_port")
    frame_prefix = LaunchConfiguration("frame_prefix")
    joint_config_file = LaunchConfiguration("joint_config_file")
    hardware_type = LaunchConfiguration("hardware_type")
    controllers = LaunchConfiguration("controller_config_file")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    # --- Paths ---
    xacro_file = PathJoinSubstitution([
        FindPackageShare("so101_description"),
        "urdf",
        "so101_arm.urdf.xacro",
    ])

    robot_description = Command([
        "xacro ", xacro_file,
        " variant:=leader",
        " use_ros2_control:=true",
        " hardware_type:=", hardware_type,
        " usb_port:=", usb_port,
        " joint_config_file:=", joint_config_file,
    ])

    # --- Nodes ---
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        parameters=[{"robot_description": robot_description,
             "frame_prefix": frame_prefix}],
        output="screen",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,
        parameters=[controllers],
        output="screen",
        emulate_tty=True,
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    # --- LaunchDescription ---
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="leader"),
            DeclareLaunchArgument(
                "hardware_type", default_value="real", description="real | mock"
            ),
            DeclareLaunchArgument("usb_port", default_value="/dev/so101_leader"),
            DeclareLaunchArgument(
                "frame_prefix",
                default_value="",
                description="TF frame prefix for robot_state_publisher, e.g. 'leader/'",
            ),
            DeclareLaunchArgument(
                "joint_config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("so101_bringup"),
                        "config",
                        "hardware",
                        "leader_joints.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "controller_config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("so101_bringup"),
                        "config",
                        "ros2_control",
                        "leader_controllers.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("so101_bringup"), "rviz", "leader.rviz"]
                ),
            ),
            rsp,
            ros2_control_node,
            joint_state_spawner,
            rviz_node,
        ]
    )