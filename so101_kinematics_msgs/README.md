# so101_kinematics_msgs

Service interfaces consumed by [`so101_kinematics`](../so101_kinematics).

| Service       | File                  | Consumer                                   |
|---------------|-----------------------|--------------------------------------------|
| `GoToPose`    | `srv/GoToPose.srv`    | `so101_kinematics.cartesian_motion_node`   |
| `GoToJoints`  | `srv/GoToJoints.srv`  | `so101_kinematics.cartesian_motion_node`   |

Kept as a sibling `ament_cmake` package because ROS 2 requires
`rosidl_generate_interfaces` to live in an `ament_cmake` package, while
`so101_kinematics` is `ament_python`.
