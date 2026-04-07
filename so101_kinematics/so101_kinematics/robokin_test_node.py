# Copyright 2026 Dmitri Manajev
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""ROS2 node: Placo IK solver with Viser 3D visualization."""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import viser
import yourdfpy
from robokin.placo import PlacoKinematics, PlacoConfig
from robokin.robot_model import load_robot_description
from robokin.ui.viser_app import ViserRobotUI


EE_FRAME = "gripper_frame_link"
DT = 1.0 / 50.0

POS_DEADZONE = 0.001
ROT_DEADZONE = 0.005


class IKViserNode(Node):
    def __init__(self):
        super().__init__("robokin_test_node")

        # Load robot model
        model = load_robot_description("so_arm101_description")
        urdf_path = str(model.urdf_path)
        urdf = yourdfpy.URDF.load(urdf_path)

        # Placo IK solver
        self.solver = PlacoKinematics(
            urdf_path=urdf_path,
            ee_frame=EE_FRAME,
            cfg=PlacoConfig(dt=DT),
        )
        self.joint_names = self.solver.joint_names

        # Initial joint config
        q_init = self.solver.make_configuration({
            "shoulder_pan": 0.0,
            "shoulder_lift": -np.pi / 2,
            "elbow_flex": np.pi / 2,
            "wrist_flex": np.deg2rad(42.97),
            "wrist_roll": 0.0,
        })
        self.solver.set_joint_state(q_init)
        T_init = self.solver.current_pose()
        self._last_gizmo_T = T_init.copy()

        # Viser UI
        self.server = viser.ViserServer()
        self.ui = ViserRobotUI(
            server=self.server,
            urdf=urdf,
            solver_joint_names=self.joint_names,
            gripper_joint_name="gripper",
        )
        self.ui.build(
            initial_q=q_init,
            initial_T=T_init,
            enable_joint_sliders=True,
            enable_gripper=True,
            enable_gizmo=True,
        )

        # ROS2 publisher for joint states
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)

        # Timer at 50 Hz
        self.timer = self.create_timer(DT, self.control_loop)

        self.get_logger().info(
            f"IK Viser node started — open http://localhost:8080"
        )

    def control_loop(self):
        if self.ui.is_manual_joint_mode():
            q = self.ui.get_joint_values()
            self.solver.set_joint_state(q)
            T = self.solver.current_pose()
            self.ui.update_robot_from_joint_values(q)
            self.ui.update_ee_display(T)
            self.ui.set_target_pose(T)
        else:
            # IK: only re-solve when user dragged the gizmo
            T_target = self.ui.get_target_pose()
            if self._gizmo_moved(T_target):
                q_current = self.solver.get_joint_state()
                q = self.solver.servo_step(q_current, T_target)
                self.solver.set_joint_state(q)
                # Snap gizmo to reachable pose
                self.ui.sync_from_solver(self.solver, move_gizmo=True)
                self._last_gizmo_T = self.solver.current_pose().copy()

        # Publish joint state to ROS2
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = self.solver.get_joint_state().tolist()
        self.joint_pub.publish(msg)


    def _gizmo_moved(self, T_new: np.ndarray) -> bool:
        pos_delta = np.linalg.norm(T_new[:3, 3] - self._last_gizmo_T[:3, 3])
        rot_delta = np.linalg.norm(T_new[:3, :3] - self._last_gizmo_T[:3, :3])
        return pos_delta > POS_DEADZONE or rot_delta > ROT_DEADZONE


def main(args=None):
    rclpy.init(args=args)
    node = IKViserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
