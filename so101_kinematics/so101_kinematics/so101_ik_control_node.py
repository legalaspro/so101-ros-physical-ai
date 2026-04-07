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

"""ROS2 node: Placo IK + Viser for SO-101 follower arm.

Integrates with so101-ros-physical-ai stack:
  - Subscribes to /follower/joint_states for real arm feedback
  - Subscribes to camera topics for display in Viser
  - Publishes to /follower/forward_controller/commands to move the arm

Launch the follower arm first:
  ros2 launch so101_bringup follower.launch.py

Then run this node:
  ros2 run so101_kinematics so101_ik_control_node
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Float64MultiArray

import time
from scipy.spatial.transform import Rotation
import viser
import yourdfpy
from robokin.placo import PlacoKinematics, PlacoConfig
from robokin.robot_model import load_robot_description
from robokin.transformations import (
    compute_segment_steps_from_speed,
    ease_in_out_sine,
    interpolate_pose,
)
from robokin.ui.viser_app import ViserRobotUI


EE_FRAME = "gripper_frame_link"
DT = 1.0 / 50.0
DWELL_TIME = 0.3

# Deadzone thresholds to avoid gizmo→solver feedback loop
POS_DEADZONE = 0.001   # 1 mm
ROT_DEADZONE = 0.005   # ~0.3 deg


def make_pose(pos_mm, rotvec_rad):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(np.asarray(rotvec_rad, dtype=float)).as_matrix()
    T[:3, 3] = np.asarray(pos_mm, dtype=float) / 1000.0
    return T


class IKViserSO101Node(Node):
    def __init__(self):
        super().__init__("so101_ik_control_node")

        # -- Parameters --
        self.declare_parameter("joints_topic", "/follower/joint_states")
        self.declare_parameter("cmd_topic", "/follower/forward_controller/commands")
        self.declare_parameter("use_cameras", False)
        self.declare_parameter("cam_wrist_topic", "/follower/image_raw")
        self.declare_parameter("cam_overhead_topic", "/static_camera/image_raw")
        joints_topic = self.get_parameter("joints_topic").value
        cmd_topic = self.get_parameter("cmd_topic").value
        self.use_cameras = self.get_parameter("use_cameras").value
        cam_wrist_topic = self.get_parameter("cam_wrist_topic").value
        cam_overhead_topic = self.get_parameter("cam_overhead_topic").value

        # -- Load robot model --
        model = load_robot_description("so_arm101_description")
        urdf_path = str(model.urdf_path)
        urdf = yourdfpy.URDF.load(urdf_path)

        # -- Placo IK solver --
        self.solver = PlacoKinematics(
            urdf_path=urdf_path,
            ee_frame=EE_FRAME,
            cfg=PlacoConfig(dt=DT),
        )
        self.joint_names = self.solver.joint_names

        # Rest configuration
        self.Q_REST = self.solver.make_configuration({
            "shoulder_pan": 0.0,
            "shoulder_lift": -np.pi / 2,
            "elbow_flex": np.pi / 2,
            "wrist_flex": np.deg2rad(42.97),
            "wrist_roll": 0.0,
        })
        self.solver.set_joint_state(self.Q_REST)
        q_init = self.Q_REST.copy()
        T_init = self.solver.current_pose()
        self.T_REST = T_init.copy()

        # Preset poses
        T_home = self.solver.fk(self.solver.make_configuration({}))
        T_down = make_pose([136.4, 0.0, 62.0], [0.0, np.pi, 0.0])
        T_left = make_pose([136.4, 100.0, 62.0], [0.0, np.pi, 0.0])
        T_right = make_pose([136.4, -100.0, 62.0], [0.0, np.pi, 0.0])
        self._pose_loop_list = [
            ("Home", T_home),
            ("Down", T_down),
            ("Left", T_left),
            ("Right", T_right),
        ]

        # Segment trajectory state
        self._segment_active = False
        self._segment_T_start = None
        self._segment_T_goal = None
        self._segment_t0 = 0.0
        self._segment_duration = 0.0

        # Loop state
        self._loop_active = False
        self._loop_pose_idx = 0
        self._loop_dwelling = False
        self._loop_dwell_t0 = 0.0

        # -- Viser UI --
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

        # Buttons
        self._loop_btn = self.server.gui.add_button("Run loop")
        rest_btn = self.server.gui.add_button("Rest")

        @self._loop_btn.on_click
        def _(event):
            if self._loop_active:
                self._loop_active = False
                self._loop_btn.label = "Run loop"
            else:
                self._loop_active = True
                self._loop_pose_idx = 0
                self._loop_dwelling = False
                self._loop_btn.label = "End loop"
                name, T_goal = self._pose_loop_list[0]
                self.get_logger().info(f"loop -> {name}")
                self._start_segment(T_goal)

        @rest_btn.on_click
        def _(event):
            self._loop_active = False
            self._loop_btn.label = "Run loop"
            self._start_segment(self.T_REST)

        # Camera image panels in Viser
        self._cam_wrist_handle = None
        self._cam_overhead_handle = None
        if self.use_cameras:
            self.get_logger().info(f"  Cameras enabled: {cam_wrist_topic}, {cam_overhead_topic}")

        # -- Latest arm state from hardware --
        self.q_measured = q_init.copy()
        self.has_arm_feedback = False
        self._initialized_from_arm = False

        # -- Gizmo deadzone tracking --
        self._last_gizmo_T = T_init.copy()

        # -- ROS2 subscribers --
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.joint_sub = self.create_subscription(
            JointState, joints_topic, self._joint_state_cb, sensor_qos
        )

        if self.use_cameras:
            self.cam_wrist_sub = self.create_subscription(
                Image, cam_wrist_topic, self._cam_wrist_cb, sensor_qos
            )
            self.cam_overhead_sub = self.create_subscription(
                Image, cam_overhead_topic, self._cam_overhead_cb, sensor_qos
            )

        # -- ROS2 publisher for arm commands --
        self.cmd_pub = self.create_publisher(Float64MultiArray, cmd_topic, 10)

        # -- Control loop timer --
        self.timer = self.create_timer(DT, self.control_loop)

        self.get_logger().info("Waiting for first joint state from arm...")

    def _joint_state_cb(self, msg: JointState):
        """Map incoming JointState to solver joint order."""
        q = np.zeros(len(self.joint_names))
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                q[i] = msg.position[idx]
        self.q_measured = q
        self.has_arm_feedback = True

        if not self._initialized_from_arm:
            self.solver.set_joint_state(q)
            T = self.solver.current_pose()
            self.ui.sync_from_solver(self.solver, move_gizmo=True)
            self._last_gizmo_T = T.copy()
            self._initialized_from_arm = True
            self.get_logger().info("Initialized from arm — http://localhost:8080")

    def _cam_wrist_cb(self, msg: Image):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, -1
        )
        if self._cam_wrist_handle is None:
            self._cam_wrist_handle = self.server.gui.add_image(
                img, label="Wrist Camera", format="jpeg", jpeg_quality=75,
            )
        else:
            self._cam_wrist_handle.image = img

    def _cam_overhead_cb(self, msg: Image):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, -1
        )
        if self._cam_overhead_handle is None:
            self._cam_overhead_handle = self.server.gui.add_image(
                img, label="Overhead Camera", format="jpeg", jpeg_quality=75,
            )
        else:
            self._cam_overhead_handle.image = img

    def _start_segment(self, T_goal: np.ndarray):
        """Start a smooth trajectory from current pose to T_goal."""
        q_meas = self.q_measured if self.has_arm_feedback else self.solver.get_joint_state()
        T_start = self.solver.fk(q_meas)
        n_steps = compute_segment_steps_from_speed(
            T_start=T_start,
            T_goal=T_goal,
            dt=DT,
            linear_speed_mps=self.solver.cfg.linear_speed_mps,
            angular_speed_radps=self.solver.cfg.angular_speed_radps,
        )
        self._segment_T_start = T_start
        self._segment_T_goal = T_goal
        self._segment_duration = n_steps * DT
        self._segment_t0 = time.perf_counter()
        self._segment_active = True
        self.ui.set_target_pose(T_goal)

    def control_loop(self):
        if not self._initialized_from_arm:
            return

        if self._segment_active:
            # Trajectory segment in progress
            elapsed = time.perf_counter() - self._segment_t0
            alpha = ease_in_out_sine(
                1.0 if self._segment_duration <= 0 else elapsed / self._segment_duration
            )
            T_ref = interpolate_pose(self._segment_T_start, self._segment_T_goal, alpha)
            q_seed = self.q_measured if self.has_arm_feedback else self.solver.get_joint_state()
            q = self.solver.servo_step(q_seed, T_ref)
            self.solver.set_joint_state(q)
            self.ui.sync_from_solver(self.solver, move_gizmo=False)

            if elapsed >= self._segment_duration:
                self._segment_active = False
                self.ui.set_target_pose(self._segment_T_goal)
                self._last_gizmo_T = self.solver.current_pose().copy()
                # Start dwell if looping
                if self._loop_active:
                    self._loop_dwelling = True
                    self._loop_dwell_t0 = time.perf_counter()

        elif self._loop_active and self._loop_dwelling:
            # Dwell between loop segments
            if time.perf_counter() - self._loop_dwell_t0 >= DWELL_TIME:
                self._loop_dwelling = False
                self._loop_pose_idx = (self._loop_pose_idx + 1) % len(self._pose_loop_list)
                name, T_goal = self._pose_loop_list[self._loop_pose_idx]
                self.get_logger().info(f"loop -> {name}")
                self._start_segment(T_goal)

        elif self.ui.is_manual_joint_mode():
            # Manual slider mode
            q = self.ui.get_joint_values()
            self.solver.set_joint_state(q)
            T = self.solver.current_pose()
            self.ui.update_robot_from_joint_values(q)
            self.ui.update_ee_display(T)
            self.ui.set_target_pose(T)
        else:
            # IK gizmo mode — only re-solve when user actually dragged the gizmo
            T_target = self.ui.get_target_pose()
            if self._gizmo_moved(T_target):
                q_seed = self.q_measured if self.has_arm_feedback else self.solver.get_joint_state()
                q = self.solver.servo_step(q_seed, T_target)
                self.solver.set_joint_state(q)
                self.ui.sync_from_solver(self.solver, move_gizmo=True)
                self._last_gizmo_T = self.solver.current_pose().copy()
            else:
                # Still update the robot mesh so gripper slider changes show
                self.ui.update_robot_from_joint_values(self.solver.get_joint_state())

        # Send commands to the real arm
        q_cmd = self.solver.get_joint_state().copy()
        # Patch gripper value from UI slider (IK doesn't control it)
        if self.ui.gripper_slider is not None:
            q_cmd[self.joint_names.index("gripper")] = float(
                self.ui.gripper_slider.value
            )
        cmd = Float64MultiArray()
        cmd.data = q_cmd.tolist()
        self.cmd_pub.publish(cmd)

    def _gizmo_moved(self, T_new: np.ndarray) -> bool:
        """Check if gizmo moved beyond deadzone thresholds."""
        pos_delta = np.linalg.norm(T_new[:3, 3] - self._last_gizmo_T[:3, 3])
        rot_delta = np.linalg.norm(T_new[:3, :3] - self._last_gizmo_T[:3, :3])
        return pos_delta > POS_DEADZONE or rot_delta > ROT_DEADZONE

    def destroy_node(self):
        self.server.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IKViserSO101Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
