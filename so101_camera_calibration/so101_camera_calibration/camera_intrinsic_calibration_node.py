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

"""Headless ChArUco camera intrinsic calibrator with Viser web UI.

Same ROS-style diversity / goodenough / outlier filter as the chessboard
calibrator, but uses a ChArUco board via cv2.aruco.CharucoDetector. ChArUco
corners can be detected even when the board is partially off-frame or
heavily tilted, so coverage at the image edges (where distortion matters
most) is easier to fill.

Open http://localhost:8080 once running. Outputs to /tmp/camera_cal.npz
and /tmp/camera_cal.yaml (ROS camera_info format), same as chessboard node.

Parameters:
    image_topic     Camera topic (default: /static_camera/image_raw).
    squares_x       ChArUco grid squares along X (default: 8).
    squares_y       ChArUco grid squares along Y (default: 6).
    square_length   Physical square size in metres (default: 0.025).
    marker_length   Embedded ArUco marker size in metres (default: 0.018).
    aruco_dict      Dictionary name (default: DICT_5X5_250).
    auto_capture    Auto-capture when the pose is novel (default: false).
    min_param_dist  L1 distance in ROS param space [x, y, size, skew] that a
                    new capture must beat against every existing capture.
    max_motion_px   Reject frames where the board moved more than this per frame.
    min_corners     Minimum ChArUco corners required per capture (default: 12).
    min_markers     Min adjacent ArUco markers for a ChArUco corner to be
                    interpolated. 1 = permissive (matches ROS image_pipeline),
                    2 = OpenCV new-API default.
"""

import math

import numpy as np
import cv2
import cv2.aruco as aruco
from packaging.version import Version

_CV_NEW_API = Version(cv2.__version__) >= Version("4.8.0")
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import viser
import yaml


PARAM_RANGES = (0.7, 0.7, 0.4, 0.5)
PARAM_LABELS = ("X", "Y", "Size", "Skew")


def _largest_rectangle_corners(char_corners, char_ids, xdim, ydim):
    """Largest fully-detected axis-aligned rectangle on the ChArUco inner-corner grid.

    Args:
        char_corners: np.ndarray (N, 1, 2) float — subpixel pixel coords (u, v)
            of the N detected ChArUco inner corners.
        char_ids:     np.ndarray (N, 1) int — ID per corner; ID k lies at
            (row=k//xdim, col=k%xdim) on the board.
        xdim: int — inner corners per row = squares_x - 1.
        ydim: int — inner corner rows    = squares_y - 1.

    Returns:
        (tl, tr, br, bl) tuple of length-2 pixel arrays, or None if no 2×2-or-
        larger rectangle has all four corners detected.
    """
    ids_flat = char_ids.ravel().astype(int)
    id_to_pixel = {int(i): char_corners[k, 0] for k, i in enumerate(ids_flat)}
    visible = {int(i) for i in ids_flat}

    best_area = 0
    best = None
    for y1 in range(ydim):
        for y2 in range(y1, ydim):
            for x1 in range(xdim):
                for x2 in range(x1, xdim):
                    area = (x2 - x1 + 1) * (y2 - y1 + 1)
                    if area <= best_area:
                        continue
                    if (y1 * xdim + x1 in visible
                            and y1 * xdim + x2 in visible
                            and y2 * xdim + x1 in visible
                            and y2 * xdim + x2 in visible):
                        best_area = area
                        best = (x1, x2, y1, y2)
    if best is None or best_area < 4:
        return None
    x1, x2, y1, y2 = best
    tl = id_to_pixel[y1 * xdim + x1]
    tr = id_to_pixel[y1 * xdim + x2]
    br = id_to_pixel[y2 * xdim + x2]
    bl = id_to_pixel[y2 * xdim + x1]
    return tl, tr, br, bl


def _quad_area(tl, tr, br, bl):
    """Shoelace area in pixels² of the quad with corners given clockwise from top-left.

    Args:
        tl, tr, br, bl: length-2 pixel arrays [u, v].
    Returns:
        float — polygon area in pixels².
    """
    p = np.array([tl, tr, br, bl], dtype=np.float64)
    x = p[:, 0]
    y = p[:, 1]
    return 0.5 * abs(
        x[0] * y[1] - x[1] * y[0]
        + x[1] * y[2] - x[2] * y[1]
        + x[2] * y[3] - x[3] * y[2]
        + x[3] * y[0] - x[0] * y[3]
    )


def _quad_skew(tl, tr, br):
    """Board tilt as deviation of the tr corner's angle from 90°, normalized to [0, 1].

    Args:
        tl, tr, br: length-2 pixel arrays. Only the angle at tr (edges tr→tl
            and tr→br) is used; bl is unneeded.
    Returns:
        float in [0, 1]. 0 = right angle (fronto-parallel); 1 = 45° off or more.
    """
    def angle(a, b, c):
        ab = a - b
        cb = c - b
        cos = np.dot(ab, cb) / (np.linalg.norm(ab) * np.linalg.norm(cb) + 1e-12)
        return math.acos(max(-1.0, min(1.0, cos)))
    return min(1.0, 2.0 * abs((math.pi / 2.0) - angle(tl, tr, br)))


def _board_params_charuco(char_corners, char_ids, image_size, xdim, ydim):
    """ROS-style (X, Y, Size, Skew) descriptor in [0, 1] from a ChArUco detection.

    Uses the largest fully-visible sub-rectangle of the inner-corner grid so
    area / skew / position stay meaningful on partial board views.

    Args:
        char_corners: np.ndarray (N, 1, 2) float — detected corner pixel coords.
        char_ids:     np.ndarray (N, 1) int — corner IDs on the board grid.
        image_size:   (width, height) tuple of ints, pixels — used to normalize.
        xdim, ydim:   int — inner-corner grid dimensions (squares - 1).

    Returns:
        [x, y, size, skew] list of floats in [0, 1], or None if no valid
        rectangle found.
    """
    rect = _largest_rectangle_corners(char_corners, char_ids, xdim, ydim)
    if rect is None:
        return None
    tl, tr, br, bl = rect
    width, height = image_size
    area = _quad_area(tl, tr, br, bl)
    border = math.sqrt(area)
    cx = float(np.mean([tl[0], tr[0], br[0], bl[0]]))
    cy = float(np.mean([tl[1], tr[1], br[1], bl[1]]))
    x = (cx - border / 2) / max(width - border, 1.0)
    y = (cy - border / 2) / max(height - border, 1.0)
    size = math.sqrt(area / (width * height))
    skew = _quad_skew(tl, tr, br)
    return [
        float(np.clip(x, 0.0, 1.0)),
        float(np.clip(y, 0.0, 1.0)),
        float(np.clip(size, 0.0, 1.0)),
        float(skew),
    ]


def _param_distance(a, b):
    """L1 distance between two param vectors (typically 4-float [x, y, size, skew]).

    Args:
        a, b: equal-length iterables of floats.
    Returns:
        float — sum(|aᵢ - bᵢ|).
    """
    return sum(abs(x - y) for x, y in zip(a, b))


def _progress_bar(fraction, width=16):
    """Unicode progress bar for the web UI.

    Args:
        fraction: float, clamped to [0, 1].
        width:    int, total character length (default 16).
    Returns:
        str like "████████░░░░░░░░".
    """
    fraction = max(0.0, min(1.0, fraction))
    filled = int(round(fraction * width))
    return "█" * filled + "░" * (width - filled)


def _resolve_aruco_dict(name: str):
    """Look up an OpenCV predefined ArUco dictionary by name.

    Args:
        name: str, e.g. "DICT_5X5_250" or "5X5_250" (prefix auto-added).
    Returns:
        aruco.Dictionary.
    Raises:
        ValueError if the name does not exist in cv2.aruco.
    """
    attr = name if name.startswith("DICT_") else f"DICT_{name}"
    if not hasattr(aruco, attr):
        raise ValueError(f"Unknown ArUco dictionary: {name}")
    return aruco.getPredefinedDictionary(getattr(aruco, attr))


class CameraIntrinsicCalibrationNode(Node):
    def __init__(self):
        super().__init__("camera_intrinsic_calibration_node")

        self.declare_parameter("image_topic", "/static_camera/image_raw")
        self.declare_parameter("squares_x", 8)
        self.declare_parameter("squares_y", 6)
        self.declare_parameter("square_length", 0.025)
        self.declare_parameter("marker_length", 0.018)
        self.declare_parameter("aruco_dict", "DICT_5X5_250")
        self.declare_parameter("auto_capture", False)
        self.declare_parameter("min_param_dist", 0.2)
        self.declare_parameter("max_motion_px", 4.0)
        self.declare_parameter("min_corners", 12)
        # OpenCV new-API default: 2 (more conservative, fewer noisy corners on
        # partial views). Set to 1 to match ROS image_pipeline's legacy call
        # interpolateCornersCharuco(..., minMarkers=1) for max coverage.
        self.declare_parameter("min_markers", 2)
        # alpha for cv2.getOptimalNewCameraMatrix used to build P in the
        # ROS camera_info YAML. 0.0 = crop to all-valid pixels (ROS default),
        # 1.0 = keep full FOV with black borders.
        self.declare_parameter("rectify_alpha", 0.0)

        self.squares_x = int(self.get_parameter("squares_x").value)
        self.squares_y = int(self.get_parameter("squares_y").value)
        self.square_length = float(self.get_parameter("square_length").value)
        self.marker_length = float(self.get_parameter("marker_length").value)
        self.dict_name = str(self.get_parameter("aruco_dict").value)
        self.max_motion_px = float(self.get_parameter("max_motion_px").value)
        self.min_corners = int(self.get_parameter("min_corners").value)
        self.min_markers = int(self.get_parameter("min_markers").value)

        self.aruco_dict = _resolve_aruco_dict(self.dict_name)
        if _CV_NEW_API:
            self.board = aruco.CharucoBoard(
                (self.squares_x, self.squares_y),
                self.square_length,
                self.marker_length,
                self.aruco_dict,
            )
            self._aruco_params = aruco.DetectorParameters()
            # CharucoDetector uses homography without intrinsics. Subpixel marker
            # refinement deviations propagate into the interpolated ChArUco
            # corners, so disable it (OpenCV tutorial recommendation).
            self._aruco_params.cornerRefinementMethod = aruco.CORNER_REFINE_NONE
            cp = aruco.CharucoParameters()
            cp.minMarkers = self.min_markers
            self.detector = aruco.CharucoDetector(
                self.board, cp, self._aruco_params
            )
        else:
            self.board = aruco.CharucoBoard_create(
                self.squares_x, self.squares_y,
                self.square_length, self.marker_length,
                self.aruco_dict,
            )
            self._aruco_params = aruco.DetectorParameters_create()
            self._aruco_params.cornerRefinementMethod = aruco.CORNER_REFINE_NONE
            self.detector = None

        self.bridge = CvBridge()
        self._latest_bgr = None
        self._latest_detection = None  # (char_corners, char_ids, params)
        self._prev_frame_corners = None
        self._prev_frame_ids = None
        self._latest_motion = None
        self.all_corners = []   # list of Nx1x2 float arrays (subpixel pixel coords)
        self.all_ids = []       # list of Nx1 int arrays
        self._captured_params = []
        self.image_size = None

        topic = self.get_parameter("image_topic").value
        self.create_subscription(Image, topic, self._image_cb, 1)
        self.create_timer(0.1, self._process)

        # ── Viser UI ──
        self.server = viser.ViserServer()
        gui = self.server.gui
        gui.set_panel_label("ChArUco Calibrator")

        gui.add_markdown(
            f"**Board:** {self.squares_x}×{self.squares_y} squares, "
            f"{self.square_length*1000:.1f}mm square / "
            f"{self.marker_length*1000:.1f}mm marker · `{self.dict_name}`"
        )
        gui.add_markdown(f"**Topic:** `{topic}`")

        self._auto_cb = gui.add_checkbox(
            "Auto-capture", bool(self.get_parameter("auto_capture").value)
        )
        self._min_param_dist = gui.add_slider(
            "Min param distance", 0.05, 0.5, 0.01,
            float(self.get_parameter("min_param_dist").value),
        )
        self._max_motion = gui.add_slider(
            "Max motion (px/frame)", 0.5, 10.0, 0.1, self.max_motion_px,
        )

        self._capture_btn = gui.add_button("Capture pose")
        self._capture_btn.on_click(lambda _: self._capture())

        self._calibrate_btn = gui.add_button("Calibrate")
        self._calibrate_btn.on_click(lambda _: self._calibrate())

        self._reset_btn = gui.add_button("Reset captures")
        self._reset_btn.on_click(lambda _: self._reset())

        self._save_btn = gui.add_button("Save to /tmp/camera_cal.npz")
        self._save_btn.on_click(lambda _: self._save())

        gui.add_markdown("### Status")
        self._status = gui.add_markdown("Waiting for image...")

        gui.add_markdown("### Coverage")
        self._coverage = gui.add_markdown("Capture poses to build coverage.")

        gui.add_markdown("### Camera")
        self._gui_img = gui.add_image(
            np.zeros((240, 320, 3), dtype=np.uint8), label="Live"
        )

        gui.add_markdown("### Result")
        self._result = gui.add_markdown("Not calibrated yet.")

        self._K = None
        self._D = None
        self._err = None

        self.get_logger().info(
            "ChArUco calibrator started — open http://localhost:8080"
        )

    # ── image path ──────────────────────────────────────────────────────

    def _image_cb(self, msg: Image):
        self._latest_bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def _process(self):
        if self._latest_bgr is None:
            return
        bgr = self._latest_bgr
        self.image_size = (bgr.shape[1], bgr.shape[0])

        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        char_corners, char_ids = self._detect_charuco(gray)

        display = bgr.copy()
        n_captures = len(self.all_corners)
        status_lines = [
            f"**Captures:** {n_captures}",
            f"**Image size:** {bgr.shape[1]}×{bgr.shape[0]}",
        ]
        xdim = self.squares_x - 1
        ydim = self.squares_y - 1
        params = None
        if char_corners is not None and len(char_corners) >= self.min_corners:
            params = _board_params_charuco(
                char_corners, char_ids, self.image_size, xdim, ydim
            )
        if params is not None:
            aruco.drawDetectedCornersCharuco(display, char_corners, char_ids)
            motion = self._compute_motion(char_corners, char_ids)
            self._latest_motion = motion
            accepted, reason = self._accept_reason(params, motion)
            self._latest_detection = (char_corners, char_ids, params)
            self._prev_frame_corners = char_corners.copy()
            self._prev_frame_ids = char_ids.copy()
            status_lines.append(
                f"**ChArUco corners:** {len(char_corners)} / "
                f"{(self.squares_x - 1) * (self.squares_y - 1)}"
            )
            status_lines.append(
                "  ".join(f"**{lbl}:** {v:.2f}" for lbl, v in zip(PARAM_LABELS, params))
            )
            motion_str = "—" if motion is None else f"{motion:.2f}px"
            status_lines.append(f"**Motion:** {motion_str}")
            status_lines.append(
                "**Guidance:** " + ("novel pose — capture" if accepted else reason)
            )
            if self._auto_cb.value and accepted:
                self._capture(source="auto", log_rejection=False)
        else:
            self._latest_detection = None
            self._prev_frame_corners = None
            self._prev_frame_ids = None
            self._latest_motion = None
            have = 0 if char_corners is None else len(char_corners)
            status_lines.append(
                f"**ChArUco corners:** {have} (need ≥{self.min_corners})"
            )
            status_lines.append("**Guidance:** keep more of the board visible")

        self._status.content = "  \n".join(status_lines)
        self._coverage.content = self._coverage_markdown()
        self._gui_img.image = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)

    def _detect_charuco(self, gray):
        if _CV_NEW_API:
            char_corners, char_ids, _m_corners, _m_ids = self.detector.detectBoard(gray)
        else:
            m_corners, m_ids, _ = aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self._aruco_params
            )
            if m_ids is None or len(m_ids) == 0:
                return None, None
            _n, char_corners, char_ids = aruco.interpolateCornersCharuco(
                m_corners, m_ids, gray, self.board, minMarkers=self.min_markers
            )
        if char_corners is None or char_ids is None:
            return None, None
        if len(char_corners) < 4:
            return None, None
        # Reject if any corner is within BORDER=8 px of the image edge.
        h, w = gray.shape
        border = 8
        pts = char_corners.reshape(-1, 2)
        if (
            pts[:, 0].min() < border or pts[:, 0].max() > w - border
            or pts[:, 1].min() < border or pts[:, 1].max() > h - border
        ):
            return None, None
        return char_corners, char_ids

    # ── ROS-style diversity and goodenough ──────────────────────────────

    def _compute_motion(self, corners, ids):
        """Mean per-corner pixel motion for IDs shared with the previous frame."""
        prev_c = self._prev_frame_corners
        prev_i = self._prev_frame_ids
        if prev_c is None or prev_i is None:
            return None
        cur_map = {int(i): corners[k, 0] for k, i in enumerate(ids.ravel())}
        prev_map = {int(i): prev_c[k, 0] for k, i in enumerate(prev_i.ravel())}
        shared = set(cur_map) & set(prev_map)
        if not shared:
            return None
        deltas = np.array([cur_map[i] - prev_map[i] for i in shared])
        return float(np.mean(np.linalg.norm(deltas, axis=1)))

    def _accept_reason(self, params, motion):
        if motion is None:
            return False, "hold still (building motion baseline)"
        if motion > self._max_motion.value:
            return False, (
                f"hold still (motion {motion:.2f}px > "
                f"{self._max_motion.value:.2f}px)"
            )
        if not self._captured_params:
            return True, ""
        nearest = min(_param_distance(params, p) for p in self._captured_params)
        if nearest < self._min_param_dist.value:
            return False, (
                f"pose too similar (param Δ={nearest:.2f} < "
                f"{self._min_param_dist.value:.2f})"
            )
        return True, ""

    def _goodenough_progress(self):
        if not self._captured_params:
            return [0.0] * 4, 0.0
        arr = np.asarray(self._captured_params)
        lo = arr.min(axis=0).tolist()
        hi = arr.max(axis=0).tolist()
        spans = [hi[0] - lo[0], hi[1] - lo[1], hi[2], hi[3]]
        progress = [min(span / r, 1.0) for span, r in zip(spans, PARAM_RANGES)]
        return progress, float(np.mean(progress))

    def _coverage_markdown(self):
        progress, overall = self._goodenough_progress()
        lines = []
        for label, p in zip(PARAM_LABELS, progress):
            lines.append(f"`{label:<5}` `{_progress_bar(p)}` {p * 100:3.0f}%")
        n = len(self._captured_params)
        ready = n >= 40 or all(p >= 1.0 for p in progress)
        tail = "**Good enough — click Calibrate.**" if ready else (
            f"Overall: {overall * 100:.0f}% · {n}/40 captures, or fill all bars."
        )
        lines.append("")
        lines.append(tail)
        return "  \n".join(lines)

    # ── capture / calibrate / reset / save ──────────────────────────────

    def _capture(self, source="manual", log_rejection=True):
        if self._latest_detection is None:
            if log_rejection:
                self.get_logger().warn("No board detected — cannot capture")
            return
        corners, ids, params = self._latest_detection
        accepted, reason = self._accept_reason(params, self._latest_motion)
        if not accepted:
            if log_rejection:
                self.get_logger().warn(f"{source.capitalize()} capture rejected: {reason}")
            return

        self.all_corners.append(corners.copy())
        self.all_ids.append(ids.copy())
        self._captured_params.append(params)
        self.get_logger().info(
            f"Captured {len(self.all_corners)} ({source}) "
            f"[{len(corners)} corners · x={params[0]:.2f} y={params[1]:.2f} "
            f"size={params[2]:.2f} skew={params[3]:.2f}]"
        )

    def _run_calibration(self, all_corners, all_ids):
        err, K, D, rvecs, tvecs = aruco.calibrateCameraCharuco(
            charucoCorners=all_corners,
            charucoIds=all_ids,
            board=self.board,
            imageSize=self.image_size,
            cameraMatrix=None,
            distCoeffs=None,
            flags=0,
        )
        board_obj = (
            self.board.getChessboardCorners()
            if _CV_NEW_API
            else np.asarray(self.board.chessboardCorners)
        )
        view_errors = []
        for corners, ids, rvec, tvec in zip(all_corners, all_ids, rvecs, tvecs):
            obj = board_obj[ids.ravel()]
            projected, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
            diff = corners.reshape(-1, 2) - projected.reshape(-1, 2)
            view_err = float(np.sqrt(np.mean(np.sum(diff ** 2, axis=1))))
            view_errors.append(view_err)
        return float(err), K, D, view_errors

    def _calibrate_with_outlier_filter(self):
        all_corners = list(self.all_corners)
        all_ids = list(self.all_ids)
        original_indices = list(range(len(all_corners)))
        dropped = []
        err, K, D, view_errors = self._run_calibration(all_corners, all_ids)
        while len(view_errors) > 12:
            median_err = float(np.median(view_errors))
            worst_idx = int(np.argmax(view_errors))
            worst_err = view_errors[worst_idx]
            if worst_err <= max(0.75, median_err * 2.5):
                break
            trial_c = [p for i, p in enumerate(all_corners) if i != worst_idx]
            trial_i = [p for i, p in enumerate(all_ids) if i != worst_idx]
            trial_err, trial_K, trial_D, trial_view = self._run_calibration(
                trial_c, trial_i
            )
            if trial_err > err:
                break
            original_idx = original_indices.pop(worst_idx)
            dropped.append(original_idx)
            all_corners = trial_c
            all_ids = trial_i
            err, K, D, view_errors = trial_err, trial_K, trial_D, trial_view
            self.get_logger().info(
                f"Excluded capture {original_idx + 1} (err {worst_err:.3f}px) "
                f"— now {len(all_corners)} views, overall {err:.3f}px"
            )
        return err, K, D, view_errors, dropped

    def _calibrate(self):
        if len(self.all_corners) < 10:
            self.get_logger().warn(
                f"Need at least 10 captures, have {len(self.all_corners)}"
            )
            return
        err, K, D, view_errors, dropped = self._calibrate_with_outlier_filter()
        self._err = float(err)
        self._K = K
        self._D = D.ravel()
        median_view = float(np.median(view_errors))
        worst_view = float(np.max(view_errors))
        quality = "good" if err < 0.5 else "acceptable" if err < 1.0 else "poor"
        self._result.content = (
            f"**Reprojection error:** {err:.4f} ({quality})  \n\n"
            f"**Median per-view:** {median_view:.4f}px  "
            f"**Worst view:** {worst_view:.4f}px  \n"
            f"**Excluded outliers:** {len(dropped)}  \n\n"
            f"**fx:** {K[0,0]:.2f}  **fy:** {K[1,1]:.2f}  \n"
            f"**cx:** {K[0,2]:.2f}  **cy:** {K[1,2]:.2f}  \n\n"
            f"**Distortion (k1, k2, p1, p2, k3):**  \n"
            f"{np.array2string(self._D, precision=5, separator=', ')}"
        )
        self.get_logger().info(f"Calibration done — reprojection error {err:.4f}")

    def _reset(self):
        self.all_corners = []
        self.all_ids = []
        self._captured_params = []
        self._K = None
        self._D = None
        self._err = None
        self._latest_detection = None
        self._prev_frame_corners = None
        self._prev_frame_ids = None
        self._latest_motion = None
        self._result.content = "Not calibrated yet."
        self.get_logger().info("Captures reset")

    def _save(self):
        if self._K is None:
            self.get_logger().warn("Run Calibrate first")
            return
        npz_path = "/tmp/camera_cal.npz"
        yaml_path = "/tmp/camera_cal.yaml"
        np.savez(
            npz_path,
            camera_matrix=self._K,
            dist_coeffs=self._D,
            reprojection_error=self._err,
            image_size=self.image_size,
        )
        # ROS mono rectification: R = I; P[:3,:3] = getOptimalNewCameraMatrix(alpha);
        # P[:,3] = 0. Matches image_pipeline/mono_calibrator.py.
        alpha = float(self.get_parameter("rectify_alpha").value)
        K_new, _ = cv2.getOptimalNewCameraMatrix(
            self._K, self._D, self.image_size, alpha
        )
        R = np.eye(3, dtype=np.float64)
        P = np.zeros((3, 4), dtype=np.float64)
        P[:3, :3] = K_new
        info = {
            "image_width": int(self.image_size[0]),
            "image_height": int(self.image_size[1]),
            "camera_name": "cam_overhead",
            "camera_matrix": {
                "rows": 3, "cols": 3,
                "data": self._K.flatten().tolist(),
            },
            "distortion_model": "plumb_bob",
            "distortion_coefficients": {
                "rows": 1, "cols": int(len(self._D)),
                "data": self._D.tolist(),
            },
            "rectification_matrix": {
                "rows": 3, "cols": 3,
                "data": R.flatten().tolist(),
            },
            "projection_matrix": {
                "rows": 3, "cols": 4,
                "data": P.flatten().tolist(),
            },
        }
        with open(yaml_path, "w") as f:
            yaml.safe_dump(info, f, default_flow_style=None)
        self.get_logger().info(f"Saved {npz_path} and {yaml_path}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraIntrinsicCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
