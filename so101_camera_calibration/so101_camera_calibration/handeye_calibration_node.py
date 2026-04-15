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

"""ChArUco target pose node for hand-eye calibration.

Subscribes to a calibrated camera (image + camera_info), detects a ChArUco
board, runs `solvePnP` to estimate board pose in the camera optical frame,
and broadcasts the resulting transform on TF as
`<camera_frame> -> <target_frame>` (default: `cam_overhead -> handeye_target`).

Viser UI on :8080 provides:
- Live detection overlay with charuco corner visualization
- 3D URDF robot model updated from /follower/joint_states
- **Manual EE** toggle — shows an IK gizmo and publishes PoseStamped
  to ``/servo_target`` on every gizmo drag. The external
  ``cartesian_motion_node`` performs the actual IK.
- **Hand-eye calibration** — Take Sample, Compute, Save buttons using
  ``cv2.calibrateHandEye`` directly. Saves to
  ``~/.ros2/robokin_calibrations/``.
"""

from packaging.version import Version

import os
import pathlib
import threading
import time as _time
import cv2
import cv2.aruco as aruco
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time, Duration
from sensor_msgs.msg import Image, CameraInfo, JointState
from geometry_msgs.msg import TransformStamped, PoseStamped
from cv_bridge import CvBridge
import tf2_ros
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from tf_transformations import quaternion_from_matrix
from scipy.spatial.transform import Rotation
from so101_kinematics_msgs.srv import GoToJoints

import viser
import yourdfpy
from robokin.robot_model import load_robot_description
from robokin.ui.viser_app import ViserRobotUI


EE_FRAME = "gripper_frame_link"

# Deadzone to avoid gizmo→solver feedback loop
POS_DEADZONE = 0.001   # 1 mm
ROT_DEADZONE = 0.005   # ~0.3 deg

# Calibration data directory
_CALIB_DIR = pathlib.Path(os.path.expanduser("~/.ros2/robokin_calibrations"))

_CV_NEW_API = Version(cv2.__version__) >= Version("4.8.0")

_DICT_MAP = {
    "DICT_4X4_50": aruco.DICT_4X4_50,
    "DICT_4X4_100": aruco.DICT_4X4_100,
    "DICT_4X4_250": aruco.DICT_4X4_250,
    "DICT_5X5_50": aruco.DICT_5X5_50,
    "DICT_5X5_100": aruco.DICT_5X5_100,
    "DICT_5X5_250": aruco.DICT_5X5_250,
}


def _make_board(squares_x, squares_y, square_m, marker_m, dict_name):
    aruco_dict = aruco.getPredefinedDictionary(_DICT_MAP[dict_name])
    if _CV_NEW_API:
        return aruco_dict, aruco.CharucoBoard(
            (squares_x, squares_y), square_m, marker_m, aruco_dict
        )
    return aruco_dict, aruco.CharucoBoard_create(
        squares_x, squares_y, square_m, marker_m, aruco_dict
    )


def _build_detector(board, aruco_dict, min_markers):
    """Build a detector once. Returns (detector, detector_params, charuco_params)
    for new API, or (None, detector_params, None) for legacy. Charuco params'
    cameraMatrix / distCoeffs must be set externally when CameraInfo arrives."""
    if _CV_NEW_API:
        det_params = aruco.DetectorParameters()
        ch_params = aruco.CharucoParameters()
        ch_params.minMarkers = int(min_markers)
        ch_params.tryRefineMarkers = False
        detector = aruco.CharucoDetector(board, ch_params, det_params)
        return detector, det_params, ch_params
    det_params = aruco.DetectorParameters_create()
    return None, det_params, None


def _detect(gray, detector, det_params, board, aruco_dict, K, D, min_markers):
    """Returns (char_corners, char_ids) or (None, None)."""
    if _CV_NEW_API:
        char_corners, char_ids, _, _ = detector.detectBoard(gray)
        return char_corners, char_ids
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=det_params)
    if ids is None or len(ids) == 0:
        return None, None
    n, char_corners, char_ids = aruco.interpolateCornersCharuco(
        corners, ids, gray, board,
        minMarkers=int(min_markers),
    )
    if n is None or n < 1:
        return None, None
    return char_corners, char_ids


def _solve_pose(char_corners, char_ids, board, K, D):
    """Returns (rvec, tvec, reproj_err_px) or (None, None, None)."""
    if char_corners is None or len(char_corners) < 4:
        return None, None, None
    if _CV_NEW_API:
        obj_pts, img_pts = board.matchImagePoints(char_corners, char_ids)
        if obj_pts is None or len(obj_pts) < 4:
            return None, None, None
        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, D,
                                      flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return None, None, None
        proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, K, D)
        err = float(np.linalg.norm(proj.reshape(-1, 2) - img_pts.reshape(-1, 2),
                                   axis=1).mean())
        return rvec, tvec, err
    # Legacy path
    ok, rvec, tvec = aruco.estimatePoseCharucoBoard(
        char_corners, char_ids, board, K, D, None, None
    )
    if not ok:
        return None, None, None
    obj_pts = board.chessboardCorners[char_ids.flatten()]
    proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, K, D)
    err = float(np.linalg.norm(proj.reshape(-1, 2) - char_corners.reshape(-1, 2),
                               axis=1).mean())
    return rvec, tvec, err


class HandeyeCalibrationNode(Node):
    def __init__(self):
        super().__init__("handeye_calibration_node")

        # ── Parameters ──
        self.declare_parameter("image_topic", "/static_camera/image_raw")
        self.declare_parameter("camera_info_topic", "/static_camera/camera_info")
        # Leave camera_frame empty to adopt the frame_id from incoming
        # CameraInfo/Image headers (recommended). Set a string to override.
        self.declare_parameter("camera_frame", "")
        self.declare_parameter("target_frame", "handeye_target")
        self.declare_parameter("squares_x", 4)
        self.declare_parameter("squares_y", 5)
        self.declare_parameter("square_length_m", 0.015)
        self.declare_parameter("marker_length_m", 0.011)
        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("min_corners", 6)
        self.declare_parameter("min_markers", 1)
        self.declare_parameter("max_reproj_px", 2.0)
        self.declare_parameter("robot_description", "so_arm101_description")
        self.declare_parameter("joint_states_topic", "/follower/joint_states")
        self.declare_parameter("base_frame", "follower/base_link")
        self.declare_parameter("robot_effector_frame", "follower/gripper_frame_link")
        self.declare_parameter("servo_target_topic", "/servo_target")
        self.declare_parameter("calibration_name", "so101_eye_on_base")
        self.declare_parameter("go_to_joints_service", "/go_to_joints")
        self.declare_parameter("auto_settle_seconds", 2.0)
        self.declare_parameter("calibration_poses_file", "")

        self._camera_frame = self.get_parameter("camera_frame").value
        self._target_frame = self.get_parameter("target_frame").value
        self._min_corners = int(self.get_parameter("min_corners").value)
        self._min_markers = int(self.get_parameter("min_markers").value)
        self._max_reproj = float(self.get_parameter("max_reproj_px").value)
        self._robot_base_frame = str(self.get_parameter("base_frame").value)
        self._robot_effector_frame = str(
            self.get_parameter("robot_effector_frame").value)
        self._calib_name = str(self.get_parameter("calibration_name").value)

        self._aruco_dict, self._board = _make_board(
            int(self.get_parameter("squares_x").value),
            int(self.get_parameter("squares_y").value),
            float(self.get_parameter("square_length_m").value),
            float(self.get_parameter("marker_length_m").value),
            str(self.get_parameter("dictionary").value),
        )
        self._detector, self._det_params, self._ch_params = _build_detector(
            self._board, self._aruco_dict,
            int(self.get_parameter("min_markers").value),
        )

        # ── Charuco state ──
        self._K = None
        self._D = None
        self._latest_bgr = None
        self._latest_stamp = None

        self._bridge = CvBridge()
        self._tf_b = TransformBroadcaster(self)

        # ── TF listener for calibration sampling ──
        self._tf_buf = Buffer(cache_time=Duration(seconds=2), node=self)
        self._tf_listener = TransformListener(
            self._tf_buf, self, spin_thread=True)

        # ── Calibration sample storage ──
        self._calib_samples: list[dict] = []  # [{robot: 4x4, tracking: 4x4}, ...]
        self._last_calibration: np.ndarray | None = None

        # ── Robot model (FK only via yourdfpy — IK lives in cartesian_motion_node) ──
        model = load_robot_description(
            str(self.get_parameter("robot_description").value)
        )
        self._urdf = yourdfpy.URDF.load(str(model.urdf_path))
        self.joint_names = list(self._urdf.actuated_joint_names)

        # ── Manual-mode state ──
        self._manual_override = False
        self._last_gizmo_T = np.eye(4)
        self._last_ee_T = np.eye(4)
        self._last_q: np.ndarray | None = None
        self._initialized_from_arm = False
        self._auto_running = False
        self._charuco_detected = False

        # ── GoToJoints service client (for auto-calibration) ──
        goto_srv = str(self.get_parameter("go_to_joints_service").value)
        self._goto_client = self.create_client(GoToJoints, goto_srv)

        # ── Viser UI + URDF viz ──
        self.server = viser.ViserServer()
        self.ui = ViserRobotUI(
            server=self.server,
            urdf=self._urdf,
            gripper_joint_name="gripper",
        )
        n_joints = len(self.joint_names)
        q_init = np.zeros(n_joints)
        T_init = self._ee_fk(q_init)
        self.ui.build(
            initial_q=q_init,
            initial_T=T_init,
            enable_gizmo=True,
            enable_joint_sliders=False,
            enable_gripper=False,
        )
        # Gizmo starts hidden — only shown in manual mode
        self.ui.ik_target.visible = False

        gui = self.server.gui
        gui.set_panel_label("ChArUco Target Pose")
        gui.add_markdown(
            f"Board **{int(self.get_parameter('squares_x').value)}×"
            f"{int(self.get_parameter('squares_y').value)}**  "
            f"sq={self.get_parameter('square_length_m').value * 1000:.0f}mm  "
            f"mk={self.get_parameter('marker_length_m').value * 1000:.0f}mm  "
            f"dict={self.get_parameter('dictionary').value}"
        )
        self._gui_img = gui.add_image(
            np.zeros((240, 320, 3), dtype=np.uint8), label="Detection"
        )
        self._gui_info = gui.add_markdown("Waiting for image + camera_info…")

        # Manual EE toggle button
        self._manual_btn = gui.add_button("Manual EE: OFF")

        @self._manual_btn.on_click
        def _(event):
            self._manual_override = not self._manual_override
            self._manual_btn.label = (
                f"Manual EE: {'ON' if self._manual_override else 'OFF'}"
            )
            if self._manual_override:
                # Show first, then snap — viser may ignore position
                # updates on hidden controls.
                self.ui.ik_target.visible = True
                T_ee = self._last_ee_T.copy()
                self.ui.set_target_pose(T_ee)
                self._last_gizmo_T = T_ee.copy()
            else:
                self.ui.ik_target.visible = False

        # ── Hand-Eye Calibration UI ──
        gui.add_markdown("---\n### Hand-Eye Calibration")
        self._calib_status = gui.add_markdown("Samples: **0**")

        btn_take = gui.add_button("📷 Take Sample")
        btn_remove = gui.add_button("🗑️ Remove Last")
        btn_save_smp = gui.add_button("💾 Save Samples")
        btn_load_smp = gui.add_button("📂 Load Samples")
        btn_compute = gui.add_button("🧮 Compute Calibration")
        btn_save_cal = gui.add_button("✅ Save Calibration")
        gui.add_markdown(
            "---\n*Manual calibration recommended. "
            "Auto-calibrate drives the arm through preconfigured joint "
            "target poses from the calibration file — "
            "make sure workspace is clear.*"
        )
        btn_auto = gui.add_button("🤖 Auto-Calibrate")

        @btn_take.on_click
        def _(event):
            self._take_sample()

        @btn_remove.on_click
        def _(event):
            self._remove_last_sample()

        @btn_save_smp.on_click
        def _(event):
            self._save_samples()

        @btn_load_smp.on_click
        def _(event):
            self._load_samples()

        @btn_compute.on_click
        def _(event):
            self._compute_calibration()

        @btn_save_cal.on_click
        def _(event):
            self._save_calibration()

        @btn_auto.on_click
        def _(event):
            if self._auto_running:
                self._auto_running = False
                self._calib_status.content = "⏹️ Auto-calibration stopping…"
                return
            threading.Thread(
                target=self._auto_calibrate, daemon=True).start()

        # ── ROS I/O ──
        img_topic = self.get_parameter("image_topic").value
        info_topic = self.get_parameter("camera_info_topic").value
        self.create_subscription(Image, img_topic, self._on_image, 1)
        self.create_subscription(CameraInfo, info_topic, self._on_info, 1)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self._on_joints, sensor_qos,
        )

        servo_topic = str(self.get_parameter("servo_target_topic").value)
        self._servo_pub = self.create_publisher(PoseStamped, servo_topic, 10)

        self.create_timer(0.1, self._tick)
        parent = self._camera_frame or "<from image header>"
        self.get_logger().info(
            f"handeye_calibration_node up — will publish TF "
            f"{parent} -> {self._target_frame}  "
            f"servo_target={servo_topic}"
        )

    # ── Calibration methods (self-contained, no easy_handeye2) ──

    @staticmethod
    def _tf_to_matrix(tf_msg) -> np.ndarray:
        """Convert a geometry_msgs/Transform to a 4×4 matrix."""
        t = tf_msg.translation
        q = tf_msg.rotation
        T = np.eye(4)
        T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T[:3, 3] = [t.x, t.y, t.z]
        return T

    def _take_sample(self):
        """Snapshot the robot and tracking TF pairs."""
        if not self._camera_frame:
            self._calib_status.content = "⚠️ No camera frame yet"
            return
        try:
            t = Time()  # latest available
            # eye-on-base: robot = effector→base
            robot_tf = self._tf_buf.lookup_transform(
                self._robot_effector_frame,
                self._robot_base_frame,
                t, Duration(seconds=1))
            # tracking: camera→marker
            tracking_tf = self._tf_buf.lookup_transform(
                self._camera_frame,
                self._target_frame,
                t, Duration(seconds=1))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"TF lookup failed: {e}")
            self._calib_status.content = f"❌ TF lookup failed: {e}"
            return

        sample = {
            "robot": self._tf_to_matrix(robot_tf.transform),
            "tracking": self._tf_to_matrix(tracking_tf.transform),
        }
        if self._last_q is not None:
            sample["joint_values"] = self._last_q.copy()
            sample["joint_names"] = list(self.joint_names)
        self._calib_samples.append(sample)
        n = len(self._calib_samples)
        rt = sample["robot"][:3, 3]
        tt = sample["tracking"][:3, 3]
        self._calib_status.content = f"✅ Sample taken — **{n}** total"
        self.get_logger().info(
            f"[calib] Sample {n}: "
            f"robot t=({rt[0]:+.4f}, {rt[1]:+.4f}, {rt[2]:+.4f})  "
            f"tracking t=({tt[0]:+.4f}, {tt[1]:+.4f}, {tt[2]:+.4f})")

    def _remove_last_sample(self):
        if self._calib_samples:
            self._calib_samples.pop()
        n = len(self._calib_samples)
        self._calib_status.content = f"🗑️ Removed — **{n}** remaining"

    # ── Auto-calibration (preconfigured joint target poses) ──

    def _go_to_joints(
        self, joint_names: list[str], positions: list[float],
    ) -> tuple[bool, str]:
        """Call the /go_to_joints service synchronously (from a background thread).

        Returns (success, message).
        """
        if not self._goto_client.wait_for_service(timeout_sec=2.0):
            return False, "go_to_joints service not available"

        req = GoToJoints.Request()
        req.joint_names = list(joint_names)
        req.positions = [float(p) for p in positions]

        future = self._goto_client.call_async(req)
        while not future.done():
            _time.sleep(0.05)
        result = future.result()
        if result is None:
            return False, "Service call returned None"
        return result.success, result.message

    def _auto_calibrate(self):
        """Replay preconfigured joint target poses and take a sample at each.

        Runs on a background thread so the UI stays responsive. Reads the
        preconfigured joint targets from `calibration_poses_file` (a YAML
        list of {joint_names, joint_values} entries shipped in the package
        config) and drives the arm through them via the GoToJoints service.
        """
        # Load reference poses
        poses_file = str(
            self.get_parameter("calibration_poses_file").value)
        if not poses_file:
            self._calib_status.content = (
                "❌ Set `calibration_poses_file` parameter")
            return
        filepath = pathlib.Path(poses_file)
        if not filepath.exists():
            self._calib_status.content = (
                f"❌ Poses file not found: {filepath}")
            return
        with open(filepath) as f:
            poses = yaml.safe_load(f)
        if not poses or not isinstance(poses, list):
            self._calib_status.content = "❌ Poses file is empty or invalid"
            return

        settle_s = float(self.get_parameter("auto_settle_seconds").value)

        self._auto_running = True
        self._calib_samples.clear()
        total = len(poses)
        self._calib_status.content = (
            f"🤖 Auto-calibration starting — **{total}** poses")
        self.get_logger().info(
            f"[auto-calib] Starting with {total} poses, settle={settle_s}s")

        for i, pose in enumerate(poses):
            if not self._auto_running:
                self._calib_status.content = (
                    f"⏹️ Stopped after **{i}** / {total}")
                self.get_logger().info(f"[auto-calib] Stopped by user at {i}")
                return

            jnames = list(pose["joint_names"])
            jvals = [float(v) for v in pose["joint_values"]]

            self._calib_status.content = (
                f"🤖 Moving to pose **{i+1}** / {total} …")
            self.get_logger().info(
                f"[auto-calib] Pose {i+1}/{total}: calling go_to_joints")

            ok, msg = self._go_to_joints(jnames, jvals)
            if not ok:
                self.get_logger().warn(
                    f"[auto-calib] Pose {i+1} failed: {msg}")
                self._calib_status.content = (
                    f"⚠️ Pose {i+1} failed: {msg} — skipping")
                continue

            if not self._auto_running:
                return

            # Wait for charuco detection to stabilize, then verify
            _time.sleep(settle_s)

            if not self._charuco_detected:
                # Poll for up to 5 more seconds
                waited = 0.0
                while not self._charuco_detected and waited < 5.0:
                    _time.sleep(0.5)
                    waited += 0.5
                if not self._charuco_detected:
                    self.get_logger().warn(
                        f"[auto-calib] Pose {i+1}: charuco not detected — skipping")
                    self._calib_status.content = (
                        f"⚠️ Pose {i+1}: no charuco — skipping")
                    continue

            self._take_sample()

        self._auto_running = False
        n = len(self._calib_samples)
        self._calib_status.content = (
            f"🤖 Auto-calibration done — **{n}** / {total} samples taken")
        self.get_logger().info(
            f"[auto-calib] Complete: {n}/{total} samples")

    _HANDEYE_METHODS = {
        "Tsai-Lenz": cv2.CALIB_HAND_EYE_TSAI,
        "Park": cv2.CALIB_HAND_EYE_PARK,
        "Horaud": cv2.CALIB_HAND_EYE_HORAUD,
        "Andreff": cv2.CALIB_HAND_EYE_ANDREFF,
        "Daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }

    def _compute_calibration(self):
        n = len(self._calib_samples)
        if n < 3:
            self._calib_status.content = (
                f"❌ Need ≥3 samples (have {n})")
            return

        R_gripper2base = []
        t_gripper2base = []
        R_target2cam = []
        t_target2cam = []
        for s in self._calib_samples:
            R_gripper2base.append(s["robot"][:3, :3])
            t_gripper2base.append(s["robot"][:3, 3].reshape(3, 1))
            R_target2cam.append(s["tracking"][:3, :3])
            t_target2cam.append(s["tracking"][:3, 3].reshape(3, 1))

        # Try all algorithms, pick the best, and log all for comparison
        self.get_logger().info(
            f"[calib] Computing with {n} samples, all methods:")
        results = {}
        for name, method in self._HANDEYE_METHODS.items():
            try:
                R, t = cv2.calibrateHandEye(
                    R_gripper2base, t_gripper2base,
                    R_target2cam, t_target2cam,
                    method=method)
                T = np.eye(4)
                T[:3, :3] = R
                T[:3, 3] = t.flatten()
                tv = t.flatten()
                rpy = Rotation.from_matrix(R).as_euler("xyz", degrees=True)
                self.get_logger().info(
                    f"  {name:12s}: t=({tv[0]:+.4f}, {tv[1]:+.4f}, "
                    f"{tv[2]:+.4f})  rpy=({rpy[0]:+.1f}, {rpy[1]:+.1f}, "
                    f"{rpy[2]:+.1f})°")
                results[name] = T
            except Exception as e:
                self.get_logger().warn(f"  {name}: FAILED — {e}")

        if not results:
            self._calib_status.content = "❌ All methods failed"
            return

        # Use Park as default (generally most robust for eye-on-base)
        best_name = "Park" if "Park" in results else next(iter(results))
        self._last_calibration = results[best_name]
        tv = self._last_calibration[:3, 3]
        rpy = Rotation.from_matrix(
            self._last_calibration[:3, :3]).as_euler("xyz", degrees=True)

        status_lines = [f"🧮 **{best_name}** ({n} samples)",
                        f"t=({tv[0]:.4f}, {tv[1]:.4f}, {tv[2]:.4f})",
                        f"rpy=({rpy[0]:.1f}, {rpy[1]:.1f}, {rpy[2]:.1f})°",
                        "", "All methods:"]
        for name, T in results.items():
            t = T[:3, 3]
            r = Rotation.from_matrix(T[:3, :3]).as_euler("xyz", degrees=True)
            marker = "→" if name == best_name else " "
            status_lines.append(
                f"{marker} **{name}**: t=({t[0]:.3f}, {t[1]:.3f}, "
                f"{t[2]:.3f})")
        self._calib_status.content = "\n\n".join(status_lines)
        self.get_logger().info(
            f"[calib] Selected {best_name}:\n{self._last_calibration}")

    def _save_calibration(self):
        if self._last_calibration is None:
            self._calib_status.content = "❌ No calibration to save — compute first"
            return
        _CALIB_DIR.mkdir(parents=True, exist_ok=True)
        filepath = _CALIB_DIR / f"{self._calib_name}.yaml"

        t = self._last_calibration[:3, 3]
        q = Rotation.from_matrix(
            self._last_calibration[:3, :3]).as_quat()  # xyzw

        data = {
            "calibration_type": "eye_on_base",
            "tracking_base_frame": self._camera_frame,
            "tracking_marker_frame": self._target_frame,
            "robot_base_frame": self._robot_base_frame,
            "robot_effector_frame": self._robot_effector_frame,
            "transform": {
                "translation": {"x": float(t[0]),
                                "y": float(t[1]),
                                "z": float(t[2])},
                "rotation": {"x": float(q[0]), "y": float(q[1]),
                             "z": float(q[2]), "w": float(q[3])},
            },
        }
        with open(filepath, "w") as f:
            yaml.dump(data, f, default_flow_style=False)
        self._calib_status.content = f"✅ Saved to `{filepath}`"
        self.get_logger().info(f"[calib] Saved to {filepath}")

    def _save_samples(self):
        _CALIB_DIR.mkdir(parents=True, exist_ok=True)
        filepath = _CALIB_DIR / f"{self._calib_name}.samples.yaml"
        data = []
        for s in self._calib_samples:
            entry = {
                "robot": s["robot"].tolist(),
                "tracking": s["tracking"].tolist(),
            }
            if "joint_values" in s:
                entry["joint_values"] = s["joint_values"].tolist()
                entry["joint_names"] = s["joint_names"]
            data.append(entry)
        with open(filepath, "w") as f:
            yaml.dump(data, f, default_flow_style=False)
        n = len(self._calib_samples)
        self._calib_status.content = f"💾 Saved **{n}** samples"
        self.get_logger().info(f"[calib] Saved {n} samples to {filepath}")

    def _load_samples(self):
        filepath = _CALIB_DIR / f"{self._calib_name}.samples.yaml"
        if not filepath.exists():
            self._calib_status.content = f"❌ No saved samples at `{filepath}`"
            return
        with open(filepath) as f:
            data = yaml.safe_load(f)
        self._calib_samples = []
        for s in data:
            sample = {
                "robot": np.array(s["robot"]),
                "tracking": np.array(s["tracking"]),
            }
            if "joint_values" in s:
                sample["joint_values"] = np.array(s["joint_values"])
                sample["joint_names"] = s["joint_names"]
            self._calib_samples.append(sample)
        n = len(self._calib_samples)
        self._calib_status.content = f"📂 Loaded **{n}** samples"
        self.get_logger().info(f"[calib] Loaded {n} samples from {filepath}")

    # ── FK helper (no IK solver needed — uses yourdfpy directly) ──

    def _ee_fk(self, q: np.ndarray) -> np.ndarray:
        """Forward kinematics: joint values → 4×4 EE pose via yourdfpy."""
        self._urdf.update_cfg(q)
        return np.array(self._urdf.get_transform(EE_FRAME), dtype=float)

    # ── callbacks ──

    def _on_image(self, msg: Image):
        self._latest_bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self._latest_stamp = msg.header.stamp
        # Adopt the image's frame_id as the TF parent unless an explicit
        # camera_frame param was set.
        if not self._camera_frame and msg.header.frame_id:
            self._camera_frame = msg.header.frame_id
            self.get_logger().info(
                f"adopted camera_frame from image header: "
                f"{self._camera_frame} -> {self._target_frame}"
            )

    def _on_joints(self, msg: JointState):
        cfg = {n: p for n, p in zip(msg.name, msg.position)}
        q = np.array([cfg.get(n, 0.0) for n in self.joint_names])
        self._last_q = q.copy()
        T_ee = self._ee_fk(q)

        if not self._initialized_from_arm:
            self._initialized_from_arm = True
            self._last_gizmo_T = T_ee.copy()
            self.get_logger().info("Initialized from arm joint states")

        self._last_ee_T = T_ee.copy()

        # Update the URDF viz from measured joints
        self.ui.update_robot_from_joint_values(q)
        self.ui.update_ee_display(T_ee)

        # When NOT in manual mode, keep the gizmo tracking the gripper
        # so it's ready at the EE when manual is toggled ON.
        if not self._manual_override:
            self.ui.set_target_pose(T_ee)
            self._last_gizmo_T = T_ee.copy()

    def _on_info(self, msg: CameraInfo):
        if not self._camera_frame and msg.header.frame_id:
            self._camera_frame = msg.header.frame_id
        if not (len(msg.k) == 9 and msg.k[0] != 0.0):
            return
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        D = np.array(msg.d, dtype=np.float64).reshape(-1)
        if (self._K is not None
                and np.array_equal(self._K, K)
                and np.array_equal(self._D, D)):
            return
        self._K, self._D = K, D
        if self._ch_params is not None:
            self._ch_params.cameraMatrix = self._K
            self._ch_params.distCoeffs = self._D
            self._detector = aruco.CharucoDetector(
                self._board, self._ch_params, self._det_params
            )

    # ── Manual-mode helpers ──

    def _gizmo_moved(self, T_new: np.ndarray) -> bool:
        pos_delta = np.linalg.norm(T_new[:3, 3] - self._last_gizmo_T[:3, 3])
        rot_delta = np.linalg.norm(T_new[:3, :3] - self._last_gizmo_T[:3, :3])
        return pos_delta > POS_DEADZONE or rot_delta > ROT_DEADZONE

    def _publish_servo_target(self, T: np.ndarray):
        """Convert a 4×4 pose matrix to PoseStamped and publish to /servo_target."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._robot_base_frame
        pos = T[:3, 3]
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        quat = Rotation.from_matrix(T[:3, :3]).as_quat()  # xyzw
        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])
        self._servo_pub.publish(msg)

    # ── Charuco detection tick ──

    def _tick(self):
        # Manual mode: publish gizmo pose to /servo_target when dragged
        if self._manual_override and self._initialized_from_arm:
            T_target = self.ui.get_target_pose()
            if self._gizmo_moved(T_target):
                self._publish_servo_target(T_target)
                self._last_gizmo_T = T_target.copy()

        if self._latest_bgr is None:
            return
        if self._K is None:
            self._gui_info.content = "Waiting for camera_info with non-zero K…"
            return

        bgr = self._latest_bgr
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        char_corners, char_ids = _detect(
            gray, self._detector, self._det_params,
            self._board, self._aruco_dict, self._K, self._D,
            self._min_markers,
        )
        display = bgr.copy()

        n = 0 if char_ids is None else len(char_ids)
        if n < self._min_corners:
            self._charuco_detected = False
            self._gui_img.image = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)
            self._gui_info.content = f"Detected {n}/{self._min_corners} corners — insufficient"
            return

        rvec, tvec, err = _solve_pose(char_corners, char_ids, self._board, self._K, self._D)
        if rvec is None:
            self._charuco_detected = False
            self._gui_info.content = f"Pose solve failed ({n} corners)"
            return

        if err > self._max_reproj:
            self._charuco_detected = False
            aruco.drawDetectedCornersCharuco(display, char_corners, char_ids, (0, 0, 255))
            self._gui_img.image = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)
            self._gui_info.content = (
                f"Reproj err {err:.2f} px > {self._max_reproj:.2f} — rejected"
            )
            return

        self._charuco_detected = True

        # Broadcast TF
        if self._camera_frame:
            self._publish_tf(rvec, tvec)

        aruco.drawDetectedCornersCharuco(display, char_corners, char_ids, (0, 255, 0))
        cv2.drawFrameAxes(display, self._K, self._D, rvec, tvec,
                          float(self.get_parameter("square_length_m").value) * 2)
        self._gui_img.image = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)
        t = tvec.flatten()
        self._gui_info.content = (
            f"**Corners:** {n}  **Reproj:** {err:.2f} px\n\n"
            f"**tvec (m):** x={t[0]:+.3f}  y={t[1]:+.3f}  z={t[2]:+.3f}"
        )

    def _publish_tf(self, rvec, tvec):
        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4)
        T[:3, :3] = R
        t = tvec.flatten()
        T[:3, 3] = t
        q = quaternion_from_matrix(T)

        msg = TransformStamped()
        msg.header.stamp = self._latest_stamp or self.get_clock().now().to_msg()
        msg.header.frame_id = self._camera_frame
        msg.child_frame_id = self._target_frame
        msg.transform.translation.x = float(t[0])
        msg.transform.translation.y = float(t[1])
        msg.transform.translation.z = float(t[2])
        msg.transform.rotation.x = q[0]
        msg.transform.rotation.y = q[1]
        msg.transform.rotation.z = q[2]
        msg.transform.rotation.w = q[3]
        self._tf_b.sendTransform(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HandeyeCalibrationNode()
    # MultiThreadedExecutor so background-thread service calls
    # (auto-calibrate → GoToJoints) can be processed while ROS spins.
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
