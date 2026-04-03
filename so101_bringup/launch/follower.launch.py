from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # --- Launch arguments ---
    namespace = LaunchConfiguration("namespace")
    frame_prefix = LaunchConfiguration("frame_prefix")
    hardware_type = LaunchConfiguration("hardware_type")
    usb_port = LaunchConfiguration("usb_port")
    joint_config_file = LaunchConfiguration("joint_config_file")
    controller_config_file = LaunchConfiguration("controller_config_file")
    arm_controller = LaunchConfiguration("arm_controller")

    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    # --- Camera TF arguments ---
    enable_static_cam = LaunchConfiguration("enable_static_cam")
    enable_wrist_cam = LaunchConfiguration("enable_wrist_cam")
    cam_static_xyz = LaunchConfiguration("cam_static_xyz")
    cam_static_rpy = LaunchConfiguration("cam_static_rpy")
    cam_wrist_xyz = LaunchConfiguration("cam_wrist_xyz")
    cam_wrist_rpy = LaunchConfiguration("cam_wrist_rpy")

    # --- Paths ---
    xacro_file = PathJoinSubstitution([FindPackageShare("so101_description"), "urdf", "so101_arm.urdf.xacro"])

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                xacro_file,
                " variant:=follower",
                " use_ros2_control:=true",
                " hardware_type:=",
                hardware_type,
                " usb_port:=",
                usb_port,
                " joint_config_file:=",
                joint_config_file,
                " enable_static_cam:=",
                enable_static_cam,
                " enable_wrist_cam:=",
                enable_wrist_cam,
                " cam_static_xyz:='",
                cam_static_xyz,
                "' cam_static_rpy:='",
                cam_static_rpy,
                "' cam_wrist_xyz:='",
                cam_wrist_xyz,
                "' cam_wrist_rpy:='",
                cam_wrist_rpy,
                "'",
            ]
        ),
        value_type=str,
    )

    # --- Nodes ---
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        parameters=[{"robot_description": robot_description, "frame_prefix": frame_prefix}],
        output="screen",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,
        parameters=[controller_config_file],
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

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=[arm_controller],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="follower"),
            DeclareLaunchArgument("hardware_type", default_value="real"),  # real | mock
            DeclareLaunchArgument("usb_port", default_value="/dev/so101_follower"),
            DeclareLaunchArgument(
                "frame_prefix",
                default_value="",
                description="TF frame prefix for robot_state_publisher, e.g. 'follower/'",
            ),
            DeclareLaunchArgument(
                "joint_config_file",
                default_value="",
            ),
            DeclareLaunchArgument(
                "controller_config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("so101_bringup"),
                        "config",
                        "ros2_control",
                        "follower_controllers.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "arm_controller",
                default_value="forward_controller",
                description="Arm controller to use: trajectory_controller or forward_controller",
            ),
            # --- Camera TF arguments ---
            DeclareLaunchArgument(
                "enable_static_cam",
                default_value="false",
                description="Enable static (overhead) camera frame in URDF",
            ),
            DeclareLaunchArgument(
                "enable_wrist_cam",
                default_value="false",
                description="Enable wrist camera frame in URDF",
            ),
            DeclareLaunchArgument(
                "cam_static_xyz",
                default_value="0.2 0.0 0.60",
                description="Static camera position relative to base_link (x y z meters)",
            ),
            DeclareLaunchArgument(
                "cam_static_rpy",
                default_value="0.0 1.5708 0.0",
                description="Static camera orientation relative to base_link (roll pitch yaw radians)",
            ),
            DeclareLaunchArgument(
                "cam_wrist_xyz",
                default_value="0.0 0.0 -0.02",
                description="Wrist camera position relative to end-effector link (x y z meters)",
            ),
            DeclareLaunchArgument(
                "cam_wrist_rpy",
                default_value="-1.5708 0.0 -1.5708",
                description="Wrist camera orientation relative to end-effector link (roll pitch yaw radians)",
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("so101_bringup"), "rviz", "follower.rviz"]
                ),
            ),
            rsp,
            ros2_control_node,
            joint_state_spawner,
            arm_controller_spawner,
            rviz_node,
        ]
    )
