#!/usr/bin/env python3
"""Touch Anything observation pipeline.

The node derives a guarded pre-touch-far joint preview after operator
confirmation.  Real execution is configuration-locked by default and, when
explicitly enabled, is sent only through the existing Arm command gateway.
SAM 2.1 is imported and loaded only after a frozen-image click.
"""
from __future__ import annotations

from collections import deque
import gc
import importlib.util
import json
import math
from pathlib import Path
import sys
import threading
import time
from typing import Any
import uuid

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Twist
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool, Float32, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))
from touch_kinematics import (
    JOINT_NAMES as REACH_JOINT_NAMES,
    solve_near_tip_alignment,
    solve_near_forward_press,
    solve_pre_touch,
)
from approach_geometry import (
    base_yz_pixel_alignment_delta,
    monocular_lateral_target,
    next_approach_waypoint,
    pixel_inside_mask,
    project_camera_point,
    validate_tracking_continuity,
)


# Keep third-party code and its small Python dependencies local to this
# workspace. PyTorch/OpenCV/ROS remain the audited system installations.
_WORKSPACE = Path("/home/matsunaga-h/robot_ws")
for _extra_path in (
    _WORKSPACE / "third_party/sam2_runtime_deps",
    _WORKSPACE / "third_party/sam2",
):
    if _extra_path.is_dir() and str(_extra_path) not in sys.path:
        sys.path.insert(0, str(_extra_path))


def stamp_seconds(message: Any) -> float:
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def jsonable(value: Any) -> Any:
    """Convert NumPy-derived evidence without silently allowing NaN."""
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(key): jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(item) for item in value]
    return value


def depth_to_meters(depth: np.ndarray, encoding: str, scale: float) -> np.ndarray:
    """Convert an aligned ROS depth image without silently guessing its units."""
    if encoding in {"16UC1", "mono16"}:
        return depth.astype(np.float32) * float(scale)
    if encoding == "32FC1":
        return depth.astype(np.float32)
    raise ValueError(f"unsupported depth encoding: {encoding}")


def deproject_pixels(
    uv: np.ndarray,
    depths_m: np.ndarray,
    camera_matrix: np.ndarray,
    distortion_model: str = "",
    distortion: np.ndarray | None = None,
) -> np.ndarray:
    fx, fy = float(camera_matrix[0, 0]), float(camera_matrix[1, 1])
    cx, cy = float(camera_matrix[0, 2]), float(camera_matrix[1, 2])
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("CameraInfo focal length is invalid")
    corrected_uv = uv.astype(np.float64)
    coefficients = (
        np.asarray(distortion, dtype=np.float64).reshape(-1)
        if distortion is not None else np.zeros(0, dtype=np.float64)
    )
    if coefficients.size and np.any(np.abs(coefficients) > 1.0e-12):
        if distortion_model not in {"plumb_bob", "rational_polynomial"}:
            raise ValueError(
                f"unsupported nonzero CameraInfo distortion model: {distortion_model}"
            )
        corrected_uv = cv2.undistortPoints(
            corrected_uv.reshape(-1, 1, 2),
            camera_matrix.astype(np.float64), coefficients,
            P=camera_matrix.astype(np.float64),
        ).reshape(-1, 2)
    z = depths_m.astype(np.float64)
    x = (corrected_uv[:, 0] - cx) * z / fx
    y = (corrected_uv[:, 1] - cy) * z / fy
    return np.column_stack((x, y, z))


def robust_surface_target(
    mask: np.ndarray,
    depth_m: np.ndarray,
    click_uv: tuple[int, int],
    camera_matrix: np.ndarray,
    *,
    radius_px: int = 20,
    minimum_points: int = 30,
    distortion_model: str = "",
    distortion: np.ndarray | None = None,
) -> dict[str, Any]:
    """Estimate a local surface point/normal from mask-constrained depth."""
    height, width = depth_m.shape
    u, v = click_uv
    yy, xx = np.ogrid[:height, :width]
    local = (xx - u) ** 2 + (yy - v) ** 2 <= radius_px ** 2
    valid = mask.astype(bool) & local & np.isfinite(depth_m) & (depth_m > 0.0)
    pixels_vu = np.argwhere(valid)
    if len(pixels_vu) < minimum_points:
        raise ValueError(
            f"only {len(pixels_vu)} valid mask/depth pixels near the selected point; "
            f"need {minimum_points}"
        )

    values = depth_m[pixels_vu[:, 0], pixels_vu[:, 1]].astype(np.float64)
    median = float(np.median(values))
    mad = float(np.median(np.abs(values - median)))
    robust_limit = max(0.008, 3.5 * 1.4826 * mad)
    keep = np.abs(values - median) <= robust_limit
    pixels_vu = pixels_vu[keep]
    values = values[keep]
    if len(values) < minimum_points:
        raise ValueError(
            f"only {len(values)} depth inliers remain after robust filtering; need {minimum_points}"
        )

    # Bound the plane fit cost for large masks while preserving deterministic output.
    if len(values) > 2500:
        step = int(math.ceil(len(values) / 2500.0))
        pixels_vu = pixels_vu[::step]
        values = values[::step]
    uv = np.column_stack((pixels_vu[:, 1], pixels_vu[:, 0]))
    points = deproject_pixels(
        uv, values, camera_matrix,
        distortion_model=distortion_model,
        distortion=distortion,
    )
    centroid = np.median(points, axis=0)
    _, _, vh = np.linalg.svd(points - centroid, full_matrices=False)
    normal = vh[-1]
    normal /= max(float(np.linalg.norm(normal)), 1.0e-12)
    # In the optical frame, the camera is at the origin. Point the surface
    # normal toward it so the pre-touch approach direction is unambiguous.
    if float(np.dot(normal, centroid)) > 0.0:
        normal = -normal
    residuals = np.abs((points - centroid) @ normal)
    return {
        "point": centroid,
        "normal": normal,
        "valid_points": int(len(points)),
        "median_depth_m": median,
        "depth_mad_m": mad,
        "plane_residual_median_m": float(np.median(residuals)),
        "plane_residual_p95_m": float(np.percentile(residuals, 95)),
    }


def quaternion_matrix_xyzw(q: np.ndarray) -> np.ndarray:
    # Never modify a view backed by saved TF evidence.
    q = np.array(q, dtype=np.float64, copy=True)
    norm = float(np.dot(q, q))
    if norm < 1.0e-15:
        return np.eye(3)
    q *= math.sqrt(2.0 / norm)
    q = np.outer(q, q)
    return np.array(
        [
            [1.0 - q[1, 1] - q[2, 2], q[0, 1] - q[2, 3], q[0, 2] + q[1, 3]],
            [q[0, 1] + q[2, 3], 1.0 - q[0, 0] - q[2, 2], q[1, 2] - q[0, 3]],
            [q[0, 2] - q[1, 3], q[1, 2] + q[0, 3], 1.0 - q[0, 0] - q[1, 1]],
        ]
    )


class LazySam21:
    def __init__(self, config: str, checkpoint: str, cpu_threads: int = 2) -> None:
        self.config = config
        self.checkpoint = Path(checkpoint).expanduser()
        self.cpu_threads = max(1, int(cpu_threads))
        self.predictor = None
        self.current_image_key: str | None = None
        self.last_use_monotonic: float | None = None

    def availability(self) -> tuple[bool, str]:
        if importlib.util.find_spec("sam2") is None:
            return False, "Python module sam2 is not installed"
        if not self.checkpoint.is_file():
            return False, f"SAM 2.1 checkpoint not found: {self.checkpoint}"
        if self.predictor is not None:
            return True, "SAM 2.1 is loaded and ready"
        return True, "SAM 2.1 module and checkpoint are present; model is not loaded"

    def load(self) -> None:
        if self.predictor is not None:
            return
        available, reason = self.availability()
        if not available:
            raise RuntimeError(reason)
        import torch
        from sam2.build_sam import build_sam2
        from sam2.sam2_image_predictor import SAM2ImagePredictor

        torch.set_num_threads(self.cpu_threads)
        try:
            torch.set_num_interop_threads(1)
        except RuntimeError:
            # PyTorch permits setting this only before inter-op work starts.
            pass
        model = build_sam2(self.config, str(self.checkpoint), device="cpu")
        model.eval()
        self.predictor = SAM2ImagePredictor(model)
        self.last_use_monotonic = time.monotonic()

    def predict(
        self,
        rgb: np.ndarray,
        points: list[tuple[int, int]],
        labels: list[int],
        image_key: str,
    ) -> tuple[np.ndarray, float, dict[str, float]]:
        self.load()
        import torch

        embedding_start = time.monotonic()
        with torch.inference_mode():
            if self.current_image_key != image_key:
                self.predictor.set_image(rgb)
                self.current_image_key = image_key
            embedding_sec = time.monotonic() - embedding_start
            decode_start = time.monotonic()
            masks, scores, _ = self.predictor.predict(
                point_coords=np.asarray(points, dtype=np.float32),
                point_labels=np.asarray(labels, dtype=np.int32),
                multimask_output=True,
            )
            decode_sec = time.monotonic() - decode_start
        index = int(np.argmax(scores))
        self.last_use_monotonic = time.monotonic()
        return masks[index].astype(bool), float(scores[index]), {
            "embedding_sec": embedding_sec,
            "mask_decode_sec": decode_sec,
        }

    def unload_if_idle(self, idle_seconds: float) -> bool:
        if self.predictor is None or self.last_use_monotonic is None:
            return False
        if time.monotonic() - self.last_use_monotonic < idle_seconds:
            return False
        self.predictor = None
        self.current_image_key = None
        self.last_use_monotonic = None
        gc.collect()
        return True

    def unload(self) -> bool:
        if self.predictor is None:
            return False
        self.predictor = None
        self.current_image_key = None
        self.last_use_monotonic = None
        gc.collect()
        return True


class TouchObservationNode(Node):
    def __init__(self) -> None:
        super().__init__("touch_observation_node")
        workspace = Path("/home/matsunaga-h/robot_ws")
        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter(
            "depth_topic", "/camera/camera/aligned_depth_to_color/image_raw"
        )
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("arm_color_topic", "/arm_camera/color/image_raw")
        self.declare_parameter("arm_camera_info_topic", "/arm_camera/color/camera_info")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("depth_scale_m_per_unit", 0.001)
        self.declare_parameter("maximum_sync_skew_sec", 0.10)
        self.declare_parameter("maximum_tf_fallback_skew_sec", 0.05)
        self.declare_parameter("minimum_target_depth_m", 0.15)
        self.declare_parameter("maximum_target_depth_m", 1.50)
        self.declare_parameter("maximum_depth_mad_m", 0.025)
        self.declare_parameter("maximum_plane_p95_m", 0.020)
        self.declare_parameter("sam_config", "configs/sam2.1/sam2.1_hiera_t.yaml")
        self.declare_parameter(
            "sam_checkpoint",
            str(workspace / "models/sam2/sam2.1_hiera_tiny.pt"),
        )
        self.declare_parameter("sam_idle_unload_sec", 180.0)
        self.declare_parameter("sam_cpu_threads", 2)
        self.declare_parameter("touch_camera_pan_deg", 267.0)
        self.declare_parameter("touch_camera_tilt_deg", 40.0)
        self.declare_parameter("camera_motion_settle_sec", 0.75)
        self.declare_parameter("d435_reacquire_max_target_shift_m", 0.06)
        self.declare_parameter("pre_touch_far_distance_m", 0.08)
        self.declare_parameter("pre_touch_near_distance_m", 0.03)
        # Real FAR evidence settled 5 raw ticks beyond a goal placed exactly
        # on the effective shoulder_lift limit; retain physical headroom.
        self.declare_parameter("ik_joint_limit_margin_rad", 0.03)
        self.declare_parameter("approach_step_distance_m", 0.005)
        self.declare_parameter("approach_large_max_lateral_step_m", 0.060)
        self.declare_parameter("approach_aligned_press_step_distance_m", 0.025)
        self.declare_parameter("approach_press_entry_tolerance_px", 20.0)
        self.declare_parameter("approach_too_close_mask_fraction", 0.15)
        self.declare_parameter("approach_too_close_retreat_m", 0.005)
        self.declare_parameter("approach_near_advance_distance_m", 0.035)
        self.declare_parameter("approach_max_press_past_target_m", 0.100)
        self.declare_parameter("approach_alignment_tolerance_m", 0.003)
        self.declare_parameter("approach_max_lateral_step_m", 0.005)
        self.declare_parameter("approach_max_ik_error_m", 0.003)
        self.declare_parameter("touch_gripper_position_rad", 1.045)
        self.declare_parameter("automatic_approach_after_confirmation", True)
        self.declare_parameter("automatic_approach_max_steps", 24)
        self.declare_parameter("automatic_approach_timeout_sec", 180.0)
        self.declare_parameter("automatic_approach_settle_sec", 0.75)
        self.declare_parameter("automatic_arm_target_max_shift_px", 90.0)
        self.declare_parameter("automatic_arm_target_min_area_ratio", 0.35)
        self.declare_parameter("automatic_arm_target_max_area_ratio", 3.0)
        self.declare_parameter("approach_cartesian_coarse_max_error_m", 0.040)
        self.declare_parameter("approach_cartesian_press_max_error_m", 0.020)
        self.declare_parameter("approach_cartesian_min_progress_ratio", 0.30)
        self.declare_parameter("approach_cartesian_correctable_max_error_m", 0.060)
        self.declare_parameter("approach_cartesian_max_correction_retries", 3)
        self.declare_parameter("arm_sam_minimum_mask_pixels", 40)
        self.declare_parameter("arm_sam_minimum_score", 0.40)
        self.declare_parameter("arm_sam_large_mask_fraction", 0.25)
        self.declare_parameter("arm_sam_large_mask_minimum_score", 0.15)
        self.declare_parameter("arm_sam_press_minimum_score", 0.05)
        self.declare_parameter("touch_success_lab_delta", 60.0)
        self.declare_parameter("touch_success_changed_fraction", 0.65)
        self.declare_parameter("touch_success_max_x_remaining_m", 0.010)
        self.declare_parameter("touch_success_max_center_error_px", 15.0)
        self.declare_parameter("touch_success_required_observations", 2)
        self.declare_parameter("touch_success_minimum_step", 1)
        self.declare_parameter("arm_projection_image_margin_px", 8)
        # Use the retained D435 3-D target projection as the first wrist-camera
        # SAM prompt.  The operator still confirms the resulting mask and can
        # replace the prompt with a click when the automatic hand-off is wrong.
        self.declare_parameter("arm_projection_auto_prompt", True)
        # Initial normalized coordinate of the rigid lower-jaw tip in the raw
        # wrist-camera image, measured from the operator-provided 2026-09-04
        # frame.  Keep it configurable for a pixel-level confirmation capture.
        self.declare_parameter("arm_fixed_tip_u_ratio", 0.496)
        self.declare_parameter("arm_fixed_tip_v_ratio", 0.735)
        self.declare_parameter("approach_pixel_alignment_tolerance_px", 8.0)
        self.declare_parameter("approach_min_pixel_progress_px", -12.0)
        self.declare_parameter("approach_nonconverging_step_limit", 6)
        # Midpoint between the measured FRONT (~0.03 m) and lateral
        # (~0.24 m) prepare-pose tip bands. Classification is performed in
        # base_link, where positive Y is LEFT and negative Y is RIGHT.
        self.declare_parameter("prepare_front_half_width_m", 0.12)
        # Applied only after the operator has confirmed the wrist-camera SAM
        # mask. This is the total Y/Z plausibility envelope. FAR corrections
        # remain capped by approach_max_lateral_step_m; after NEAR is latched,
        # the full camera-derived Y/Z correction may be commanded in one step.
        self.declare_parameter("maximum_arm_lateral_correction_m", 0.20)
        self.declare_parameter("allow_pre_touch_far_execution", False)
        self.declare_parameter("allow_approach_execution", False)
        self.declare_parameter(
            "artifact_root", "/home/matsunaga-h/robot_ws/debug-output/touch-anything"
        )

        self._bridge = CvBridge()
        self._lock = threading.RLock()
        self._color: deque[Image] = deque(maxlen=12)
        self._depth: deque[Image] = deque(maxlen=12)
        self._info: deque[CameraInfo] = deque(maxlen=12)
        self._arm_color: deque[Image] = deque(maxlen=4)
        self._arm_info: deque[CameraInfo] = deque(maxlen=4)
        self._frozen: dict[str, Any] | None = None
        self._prompts: list[tuple[int, int]] = []
        self._labels: list[int] = []
        self._mask: np.ndarray | None = None
        self._target: dict[str, Any] | None = None
        self._reach_plan: dict[str, Any] | None = None
        self._reach_preview_request_id: str | None = None
        self._reach_execute_request_id: str | None = None
        self._arm_frozen: dict[str, Any] | None = None
        self._arm_prompts: list[tuple[int, int]] = []
        self._arm_labels: list[int] = []
        self._arm_mask: np.ndarray | None = None
        self._arm_target: dict[str, Any] | None = None
        self._arm_target_confirmed = False
        self._arm_handoff_complete = False
        self._overlay_stage = "D435"
        self._approach_plan: dict[str, Any] | None = None
        self._approach_preview_request_id: str | None = None
        self._approach_execute_request_id: str | None = None
        self._automatic_approach = False
        self._automatic_approach_started: float | None = None
        self._automatic_approach_steps = 0
        self._automatic_reacquire_due: float | None = None
        self._automatic_tracking_reference: dict[str, Any] | None = None
        self._automatic_reacquire_pending = False
        self._automatic_last_pixel_error: float | None = None
        self._automatic_nonconverging_steps = 0
        self._last_executed_x_remaining_m: float | None = None
        self._cartesian_correction_retries = 0
        self._press_phase_active = False
        self._approach_far_tip_x_m: float | None = None
        self._near_phase_latched = False
        self._far_arm_acquire_due: float | None = None
        self._motor_samples: deque[dict[str, Any]] = deque(maxlen=600)
        self._far_motor_baseline_started: float | None = None
        self._far_motor_baseline: dict[str, Any] | None = None
        self._active_step_motor_started: float | None = None
        self._motor_step_evidence: list[dict[str, Any]] = []
        self._arm_appearance_reference: dict[str, Any] | None = None
        # Immutable within one Touch session.  Later operator corrections may
        # replace the SAM tracking prompt/mask, but must not silently move the
        # before-Touch colour baseline.  Success is always compared with the
        # first arm-camera region explicitly confirmed by the operator.
        self._arm_appearance_reference_id: str | None = None
        self._touch_success_change_streak = 0
        self._home_preview_request_id: str | None = None
        self._home_execute_request_id: str | None = None
        self._active = False
        self._state = "INACTIVE"
        self._message = "Select Touch Anything from the Arm Dashboard"
        self._arm_state: dict[str, Any] = {}
        self._arm_state_received: float | None = None
        self._robot_mode = "UNKNOWN"
        self._last_nonzero_cmd: float | None = None
        self._pan_raw: float | None = None
        self._tilt_raw: float | None = None
        self._pan_received: float | None = None
        self._tilt_received: float | None = None
        self._camera_adjustment_requested_at: float | None = None
        self._prepare_camera_goal: dict[str, Any] | None = None
        self._d435_reacquire_reference: dict[str, Any] | None = None
        self._arm_joints: dict[str, float] = {}
        self._lift_position_mm: float | None = None
        self._lift_state: dict[str, Any] = {}
        self._last_camera_motion: float | None = None
        self._last_mechanism_motion: float | None = None
        self._job_id = 0
        self._sam_lock = threading.Lock()
        self._sam = LazySam21(
            str(self.get_parameter("sam_config").value),
            str(self.get_parameter("sam_checkpoint").value),
            int(self.get_parameter("sam_cpu_threads").value),
        )

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_subscription(
            Image, str(self.get_parameter("color_topic").value), self._on_color,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image, str(self.get_parameter("depth_topic").value), self._on_depth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo, str(self.get_parameter("camera_info_topic").value), self._on_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image, str(self.get_parameter("arm_color_topic").value), self._on_arm_color,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo, str(self.get_parameter("arm_camera_info_topic").value),
            self._on_arm_info, qos_profile_sensor_data,
        )
        self.create_subscription(String, "/touch_anything/command", self._on_command, 20)
        self.create_subscription(String, "/arm/dashboard_state", self._on_arm_state, 10)
        self.create_subscription(String, "/robot_mode", self._on_robot_mode, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(
            Float32, "/chokudomotor/angle",
            lambda msg: self._on_axis("pan", float(msg.data)), 10,
        )
        self.create_subscription(
            Float32, "/cameraswingmotor/angle",
            lambda msg: self._on_axis("tilt", float(msg.data)), 10,
        )
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 20)
        self.create_subscription(Float32, "/lift/position", self._on_lift_position, 10)
        self.create_subscription(String, "/lift/state", self._on_lift_state, 10)

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._state_pub = self.create_publisher(String, "/touch_anything/state", state_qos)
        self._lock_pub = self.create_publisher(Bool, "/touch_anything/base_lock", state_qos)
        self._nav_mode_pub = self.create_publisher(String, "/nav_mode/command", 10)
        self._arm_command_pub = self.create_publisher(String, "/arm/dashboard_command", 10)
        self._pan_target_pub = self.create_publisher(
            Float32, "/chokudomotor/target_angle", 10
        )
        self._tilt_target_pub = self.create_publisher(
            Float32, "/cameraswingmotor/target_angle", 10
        )
        self._overlay_pub = self.create_publisher(
            Image, "/touch_anything/overlay_image", qos_profile_sensor_data
        )
        self._point_pub = self.create_publisher(
            PointStamped, "/touch_anything/target_point", state_qos
        )
        self._pose_pub = self.create_publisher(
            PoseStamped, "/touch_anything/target_pose", state_qos
        )
        self._marker_pub = self.create_publisher(
            MarkerArray, "/touch_anything/markers", state_qos
        )
        self.create_timer(0.5, self._publish_state)
        # Keep the arm-camera TF marker useful while the operator moves or
        # previews the arm.  This is visualization-only and publishes no
        # mechanism command.
        self.create_timer(0.5, self._refresh_visualization)
        self.create_timer(0.1, self._automatic_approach_tick)
        self.create_timer(0.1, self._far_arm_handoff_tick)
        self.create_timer(0.1, self._prepare_camera_tick)
        self.create_timer(5.0, self._maintenance)
        self.get_logger().info(
            "Touch Anything ready; pre-touch preview enabled, real execution locked by default"
        )

    def _on_color(self, message: Image) -> None:
        with self._lock:
            self._color.append(message)

    def _on_depth(self, message: Image) -> None:
        with self._lock:
            self._depth.append(message)

    def _on_info(self, message: CameraInfo) -> None:
        with self._lock:
            self._info.append(message)

    def _on_arm_color(self, message: Image) -> None:
        with self._lock:
            self._arm_color.append(message)

    def _on_arm_info(self, message: CameraInfo) -> None:
        with self._lock:
            self._arm_info.append(message)

    def _on_arm_state(self, message: String) -> None:
        try:
            state = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        now_monotonic = time.monotonic()
        motor_health = state.get("motor_health", {}) if isinstance(state, dict) else {}
        currents = motor_health.get("current_raw", {}) if isinstance(motor_health, dict) else {}
        with self._lock:
            if isinstance(currents, dict) and currents:
                self._motor_samples.append({
                    "monotonic_sec": now_monotonic,
                    "wall_time_sec": time.time(),
                    "current_raw": {
                        str(name): int(value) for name, value in currents.items()
                    },
                })
            self._arm_state = state if isinstance(state, dict) else {}
            self._arm_state_received = time.monotonic()
            execution = self._arm_state.get("execution", {})
            if (
                self._home_preview_request_id is not None
                and execution.get("last_request_id") == self._home_preview_request_id
            ):
                candidate = self._arm_state.get("candidate", {})
                preview_request_id = self._home_preview_request_id
                self._home_preview_request_id = None
                if bool(candidate.get("valid")) and candidate.get("id"):
                    execute_request_id = f"touch-home-execute-{uuid.uuid4()}"
                    self._home_execute_request_id = execute_request_id
                    self._arm_command_pub.publish(String(data=json.dumps({
                        "mode": str(self._arm_state.get("mode", "REAL_DASHBOARD_TEST")),
                        "request_id": execute_request_id,
                        "command": "execute_candidate",
                        "candidate_id": str(candidate["id"]),
                        "duration_sec": 2.0,
                    }, ensure_ascii=False, allow_nan=False)))
                    self._set_state(
                        "TOUCH_SUCCESS_RETURNING_HOME",
                        "Touch success confirmed; guarded home candidate accepted and executing",
                    )
                else:
                    errors = candidate.get("errors", [])
                    reason = "; ".join(str(item) for item in errors) or str(
                        execution.get("message", "Arm gateway rejected home preview")
                    )
                    self._set_state(
                        "TOUCH_SUCCESS_HOME_FAILED",
                        f"Touch succeeded but guarded home preview failed: {reason}; "
                        f"request={preview_request_id}",
                    )
            if (
                self._reach_plan is not None
                and self._reach_preview_request_id is not None
                and self._arm_state.get("execution", {}).get("last_request_id")
                == self._reach_preview_request_id
            ):
                candidate = self._arm_state.get("candidate", {})
                if bool(candidate.get("valid")) and candidate.get("id"):
                    self._reach_plan["candidate_id"] = str(candidate["id"])
                    if self._state == "PRETOUCH_PREVIEW_REQUESTED":
                        self._start_prepare_camera_adjustment()
                elif self._state == "PRETOUCH_PREVIEW_REQUESTED":
                    errors = candidate.get("errors", [])
                    reason = "; ".join(str(item) for item in errors) or str(
                        self._arm_state.get("execution", {}).get(
                            "message", "Arm gateway rejected the preview"
                        )
                    )
                    self._set_state("PRETOUCH_GATEWAY_REJECTED", reason)
            if (
                self._approach_plan is not None
                and self._approach_preview_request_id is not None
                and self._arm_state.get("execution", {}).get("last_request_id")
                == self._approach_preview_request_id
            ):
                candidate = self._arm_state.get("candidate", {})
                if bool(candidate.get("valid")) and candidate.get("id"):
                    self._approach_plan["candidate_id"] = str(candidate["id"])
                    self._approach_preview_request_id = None
                    if self._state == "APPROACH_PREVIEW_REQUESTED":
                        self._set_state(
                            "APPROACH_STEP_PLANNED",
                            "Guarded visual-approach step accepted by Arm gateway",
                        )
                        if self._automatic_approach:
                            self._execute_approach_step(automatic=True)
                elif self._state == "APPROACH_PREVIEW_REQUESTED":
                    self._approach_preview_request_id = None
                    errors = candidate.get("errors", [])
                    reason = "; ".join(str(item) for item in errors) or str(
                        self._arm_state.get("execution", {}).get(
                            "message", "Arm gateway rejected the approach preview"
                        )
                    )
                    if self._automatic_approach:
                        self._pause_automatic_approach(
                            "APPROACH_GATEWAY_REJECTED",
                            f"Automatic approach paused: {reason}",
                        )
                    else:
                        self._set_state("APPROACH_GATEWAY_REJECTED", reason)
            execution = self._arm_state.get("execution", {})
            if (
                self._home_execute_request_id is not None
                and execution.get("last_request_id") == self._home_execute_request_id
            ):
                execution_state = str(execution.get("state", "UNKNOWN"))
                if execution_state == "SUCCEEDED":
                    self._home_execute_request_id = None
                    with self._sam_lock:
                        self._sam.unload()
                    self._set_state(
                        "TOUCH_SUCCEEDED_HOME",
                        "Arm-camera appearance change confirmed Touch success; home reached",
                    )
                elif execution_state in {"FAILED", "CANCELLED", "BLOCKED"}:
                    self._home_execute_request_id = None
                    self._set_state(
                        "TOUCH_SUCCESS_HOME_FAILED",
                        "Touch succeeded but return-home execution failed: "
                        + str(execution.get("message", execution_state)),
                    )
            if (
                self._reach_execute_request_id is not None
                and execution.get("last_request_id") == self._reach_execute_request_id
            ):
                execution_state = str(execution.get("state", "UNKNOWN"))
                if execution_state == "SUCCEEDED":
                    self._reach_execute_request_id = None
                    self._far_arm_acquire_due = (
                        time.monotonic()
                        + float(self.get_parameter("automatic_approach_settle_sec").value)
                    )
                    self._motor_samples.clear()
                    self._far_motor_baseline_started = time.monotonic()
                    self._far_motor_baseline = None
                    self._motor_step_evidence.clear()
                    self._set_state(
                        "PRETOUCH_FAR_REACHED",
                        "Arm gateway reports FAR trajectory completed; waiting for stability "
                        "before automatic arm-camera SAM acquisition",
                    )
                elif execution_state in {"FAILED", "CANCELLED", "BLOCKED"}:
                    self._reach_execute_request_id = None
                    self._set_state(
                        "PRETOUCH_EXECUTION_FAILED",
                        str(execution.get("message", execution_state)),
                    )
            if (
                self._approach_execute_request_id is not None
                and execution.get("last_request_id") == self._approach_execute_request_id
            ):
                execution_state = str(execution.get("state", "UNKNOWN"))
                if execution_state == "SUCCEEDED":
                    self._approach_execute_request_id = None
                    cartesian_feedback = self._record_completed_motor_step(execution)
                    acceptance = (
                        {} if cartesian_feedback is None
                        else cartesian_feedback.get("acceptance", {})
                    )
                    if not bool(acceptance.get("accepted", False)):
                        reason = str(
                            acceptance.get(
                                "reason",
                                "measured touch-tip endpoint is unavailable",
                            )
                        )
                        correctable = bool(acceptance.get("correctable", False))
                        maximum_retries = int(self.get_parameter(
                            "approach_cartesian_max_correction_retries"
                        ).value)
                        if not correctable or self._cartesian_correction_retries >= maximum_retries:
                            self._pause_automatic_approach(
                                "APPROACH_CARTESIAN_NOT_REACHED",
                                "Arm joint trajectory completed, but the measured "
                                f"Cartesian endpoint was not accepted: {reason}; "
                                f"correction retries={self._cartesian_correction_retries}/"
                                f"{maximum_retries}",
                            )
                            return
                        self._cartesian_correction_retries += 1
                    else:
                        self._cartesian_correction_retries = 0
                    # Never advance the visual/contact state from the commanded
                    # waypoint.  Use measured TF feedback so the next IK solve
                    # starts from the pose the physical arm actually reached.
                    completed_tip = np.asarray(
                        (cartesian_feedback or {}).get("completed_tip_base_m", []),
                        dtype=np.float64,
                    )
                    refined = np.asarray(
                        (self._approach_plan or {}).get("refined_point_base_m", []),
                        dtype=np.float64,
                    )
                    if completed_tip.shape == (3,) and refined.shape == (3,):
                        self._last_executed_x_remaining_m = float(
                            refined[0] - completed_tip[0]
                        )
                    elif self._approach_plan is not None:
                        self._last_executed_x_remaining_m = float(
                            self._approach_plan.get("remaining_after_m", math.inf)
                        )
                    if self._arm_target is not None:
                        self._automatic_tracking_reference = dict(self._arm_target)
                    if self._automatic_approach:
                        self._automatic_approach_steps += 1
                    self._clear_arm_observation()
                    if self._automatic_approach:
                        self._automatic_reacquire_due = (
                            time.monotonic() + float(self.get_parameter(
                                "automatic_approach_settle_sec"
                            ).value)
                        )
                        self._set_state(
                            (
                                "AUTO_APPROACH_CARTESIAN_CORRECTING"
                                if not bool(acceptance.get("accepted", False))
                                else "AUTO_APPROACH_SETTLING"
                            ),
                            f"Step {self._automatic_approach_steps} measured; "
                            f"Cartesian residual will be re-observed and re-planned "
                            f"from actual touch-tip TF (correction "
                            f"{self._cartesian_correction_retries})",
                        )
                    else:
                        self._set_state(
                            "ARM_REACQUIRE_REQUIRED",
                            "Approach step reached; freeze a new arm-camera frame and "
                            "re-segment before planning another step",
                        )
                elif execution_state in {"FAILED", "CANCELLED", "BLOCKED"}:
                    self._approach_execute_request_id = None
                    reason = str(execution.get("message", execution_state))
                    cartesian_feedback = self._record_completed_motor_step(execution)
                    acceptance = (
                        {} if cartesian_feedback is None
                        else cartesian_feedback.get("acceptance", {})
                    )
                    # The gateway may report FAILED solely because one motor
                    # missed its HOLD-entry tick tolerance after motion ended.
                    # If measured touch-tip TF is nevertheless inside the
                    # accepted/correctable Cartesian envelope, continue the
                    # stop-observe loop from reality instead of discarding the
                    # completed step. Other gateway failures remain blocking.
                    hold_entry_only = "Final error exceeds per-motor HOLD-entry tolerance" in reason
                    measured_usable = bool(
                        acceptance.get("accepted", False)
                        or acceptance.get("correctable", False)
                    )
                    if self._automatic_approach and hold_entry_only and measured_usable:
                        if self._automatic_approach:
                            self._automatic_approach_steps += 1
                        if self._arm_target is not None:
                            self._automatic_tracking_reference = dict(self._arm_target)
                        self._clear_arm_observation()
                        self._automatic_reacquire_due = (
                            time.monotonic() + float(self.get_parameter(
                                "automatic_approach_settle_sec"
                            ).value)
                        )
                        self._set_state(
                            "AUTO_APPROACH_CARTESIAN_CORRECTING",
                            "Gateway HOLD-entry tolerance was missed, but measured "
                            "touch-tip TF remains inside the bounded Cartesian "
                            "correction envelope; re-observing before the next step",
                        )
                        return
                    if self._automatic_approach:
                        self._pause_automatic_approach(
                            "APPROACH_EXECUTION_FAILED",
                            f"Automatic approach paused: {reason}",
                        )
                    else:
                        self._set_state("APPROACH_EXECUTION_FAILED", reason)
            if self._active and self._arm_state.get("control_mode") != "code2real&sim":
                self._deactivate("Arm left code2real&sim; Touch Mode cancelled")

    def _on_robot_mode(self, message: String) -> None:
        self._robot_mode = message.data.strip().upper()

    def _on_cmd_vel(self, message: Twist) -> None:
        values = (
            message.linear.x, message.linear.y, message.linear.z,
            message.angular.x, message.angular.y, message.angular.z,
        )
        if any(abs(float(value)) > 1.0e-5 for value in values):
            self._last_nonzero_cmd = time.monotonic()

    def _on_axis(self, axis: str, value: float) -> None:
        now = time.monotonic()
        previous = self._pan_raw if axis == "pan" else self._tilt_raw
        if previous is not None and abs(value - previous) > 0.2:
            self._last_camera_motion = now
            self._last_mechanism_motion = now
        if axis == "pan":
            self._pan_raw = value
            self._pan_received = now
        else:
            self._tilt_raw = value
            self._tilt_received = now

    def _on_joint_state(self, message: JointState) -> None:
        observed = {
            str(name): float(value)
            for name, value in zip(message.name, message.position)
            if str(name).startswith(("shoulder_", "elbow_", "wrist_", "gripper_"))
        }
        if not observed:
            return
        common = set(observed).intersection(self._arm_joints)
        if (
            common
            and max(abs(observed[name] - self._arm_joints[name]) for name in common) > 0.003
        ):
            self._last_mechanism_motion = time.monotonic()
        self._arm_joints = observed

    def _on_lift_position(self, message: Float32) -> None:
        value = float(message.data)
        if self._lift_position_mm is not None and abs(value - self._lift_position_mm) > 0.5:
            self._last_mechanism_motion = time.monotonic()
        self._lift_position_mm = value

    def _on_lift_state(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            self._lift_state = payload if isinstance(payload, dict) else {"raw": message.data}
        except json.JSONDecodeError:
            self._lift_state = {"raw": message.data}

    def _arm_gate(self) -> tuple[bool, str]:
        if self._arm_state_received is None or time.monotonic() - self._arm_state_received > 0.75:
            return False, "Arm Dashboard state is missing or stale"
        if self._arm_state.get("control_mode") != "code2real&sim":
            return False, "Arm must be in code2real&sim mode"
        if not bool(self._arm_state.get("control_mode_synced")):
            return False, "Arm control mode is not synchronized with the hardware server"
        if self._arm_state.get("health") in {
            "STALE", "STARTING", "RECOVERY_REQUIRED", "HOLD_FAULT", "CODE_TORQUE_INCOMPLETE"
        }:
            return False, f"Arm health blocks Touch Mode: {self._arm_state.get('health')}"
        return True, "Arm code mode and fresh hardware feedback are available"

    def _on_command(self, message: String) -> None:
        try:
            command = json.loads(message.data)
        except (json.JSONDecodeError, TypeError) as exc:
            self._set_state("BLOCKED", f"Invalid command JSON: {exc}")
            return
        name = str(command.get("command", "")).strip().lower()
        if name == "enter":
            allowed, reason = self._arm_gate()
            if not allowed:
                self._set_state("BLOCKED", reason)
                return
            sam_available, sam_reason = self._sam.availability()
            if not sam_available:
                self._set_state("BLOCKED", sam_reason)
                return
            # MANUAL forces the authoritative mode switch to publish zero and
            # cancel any NavigateToPose goal. We never restore AUTO on exit;
            # doing so could resume motion without an operator decision.
            self._nav_mode_pub.publish(String(data="MANUAL"))
            with self._lock:
                self._active = True
                self._frozen = None
                self._prompts.clear()
                self._labels.clear()
                self._mask = None
                self._target = None
                self._reach_plan = None
                self._reach_preview_request_id = None
                self._reach_execute_request_id = None
                self._prepare_camera_goal = None
                self._d435_reacquire_reference = None
                self._arm_handoff_complete = False
                self._clear_arm_observation()
                self._approach_plan = None
                self._approach_preview_request_id = None
                self._approach_execute_request_id = None
                self._reset_automatic_approach()
                self._motor_samples.clear()
                self._far_motor_baseline_started = None
                self._far_motor_baseline = None
                self._active_step_motor_started = None
                self._motor_step_evidence.clear()
                self._overlay_stage = "D435"
            requested_pan, requested_tilt = self._publish_one_shot_camera_adjustment()
            self._set_state(
                "READY_TO_FREEZE",
                "MANUAL/base lock active; one-shot camera request sent for "
                f"pan={requested_pan:.1f}, tilt={requested_tilt:.1f} deg; "
                "adjust further if needed, then freeze when stable",
            )
        elif name in {"exit", "cancel"}:
            self._deactivate("Touch Mode exited; base lock released")
        elif name == "freeze":
            self._freeze()
        elif name == "clear_prompts":
            if self._overlay_stage == "ARM":
                self._clear_arm_prompts()
                return
            with self._lock:
                self._prompts.clear()
                self._labels.clear()
                self._mask = None
                self._target = None
                self._reach_plan = None
                self._reach_preview_request_id = None
            self._republish_overlay()
            self._set_state("FRAME_FROZEN", "Prompts cleared; click the target object")
        elif name == "add_prompt":
            self._add_prompt(command)
        elif name == "confirm_preview":
            if self._state != "TARGET_READY" or self._target is None:
                self._set_state("BLOCKED", "A verified 3-D target is required before confirmation")
                return
            with self._lock:
                self._frozen["operator_confirmed"] = True
                self._save_artifact()
            self._set_state(
                "PREVIEW_CONFIRMED",
                "3-D target accepted; automatically planning guarded FAR preview",
            )
            # Operator confirmation is the only human authorization required for
            # FAR.  Planning, camera-reference verification and execution remain
            # individually gated, but advance automatically when each gate passes.
            self._plan_pre_touch_far()
        elif name == "plan_pre_touch_far":
            self._plan_pre_touch_far()
        elif name == "execute_pre_touch_far":
            self._execute_pre_touch_far()
        elif name == "acquire_arm_target":
            self._acquire_arm_target()
        elif name == "confirm_arm_target":
            self._confirm_arm_target()
        elif name == "plan_approach_step":
            self._plan_approach_step()
        elif name == "execute_approach_step":
            self._execute_approach_step()
        else:
            self._set_state("BLOCKED", f"Unsupported Touch command: {name or '<empty>'}")

    def _freeze(self) -> None:
        if not self._active:
            self._set_state("BLOCKED", "Activate Touch Anything from the Arm Dashboard first")
            return
        allowed, reason = self._arm_gate()
        if not allowed:
            self._set_state("BLOCKED", reason)
            return
        camera_ready, camera_reason = self._camera_feedback_ready()
        if not camera_ready:
            self._set_state("BLOCKED", camera_reason)
            return
        if self._last_nonzero_cmd is not None and time.monotonic() - self._last_nonzero_cmd < 0.75:
            self._set_state("BLOCKED", "Robot motion was observed within 0.75 s; wait until stable")
            return
        if (
            self._last_mechanism_motion is not None
            and time.monotonic() - self._last_mechanism_motion < 0.75
        ):
            self._set_state(
                "BLOCKED",
                "Pan/tilt, lift, or Arm motion was observed within 0.75 s; wait until stable",
            )
            return
        with self._lock:
            if not self._color or not self._depth or not self._info:
                self._set_state("BLOCKED", "D435 color, aligned depth, or CameraInfo is missing")
                return
            color = self._color[-1]
            color_stamp = stamp_seconds(color)
            depth = min(self._depth, key=lambda item: abs(stamp_seconds(item) - color_stamp))
            info = min(self._info, key=lambda item: abs(stamp_seconds(item) - color_stamp))
            depth_skew = abs(stamp_seconds(depth) - color_stamp)
            info_skew = abs(stamp_seconds(info) - color_stamp)
            maximum_skew = float(self.get_parameter("maximum_sync_skew_sec").value)
            if depth_skew > maximum_skew or info_skew > maximum_skew:
                self._set_state(
                    "BLOCKED",
                    f"D435 synchronization rejected: depth skew={depth_skew:.3f}s, "
                    f"CameraInfo skew={info_skew:.3f}s, limit={maximum_skew:.3f}s",
                )
                return
            rgb = self._bridge.imgmsg_to_cv2(color, desired_encoding="rgb8").copy()
            raw_depth = self._bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough").copy()
            depth_m = depth_to_meters(
                raw_depth, depth.encoding,
                float(self.get_parameter("depth_scale_m_per_unit").value),
            )
            if rgb.shape[:2] != depth_m.shape[:2]:
                self._set_state(
                    "BLOCKED",
                    f"Aligned depth size {depth_m.shape[::-1]} does not match color {rgb.shape[1::-1]}",
                )
                return
            try:
                tf_snapshot = self._lookup_tf_snapshot(color.header.frame_id, color_stamp)
            except ValueError as exc:
                self._set_state("BLOCKED", str(exc))
                return
            self._frozen = {
                "id": str(uuid.uuid4()),
                "rgb": rgb,
                "raw_depth": raw_depth,
                "depth_encoding": depth.encoding,
                "depth_m": depth_m,
                "camera_matrix": np.asarray(info.k, dtype=np.float64).reshape(3, 3),
                "distortion_model": info.distortion_model,
                "distortion": list(info.d),
                "stamp": color_stamp,
                "stamp_msg": color.header.stamp,
                "frame_id": color.header.frame_id,
                "tf_snapshot": tf_snapshot,
                "depth_skew_sec": depth_skew,
                "camera_info_skew_sec": info_skew,
                "mechanism_evidence": {
                    "pan_raw": self._pan_raw,
                    "tilt_raw": self._tilt_raw,
                    "arm_joint_positions_rad": dict(self._arm_joints),
                    "lift_position_mm": self._lift_position_mm,
                    "lift_state": dict(self._lift_state),
                },
                "operator_confirmed": False,
            }
            self._prompts.clear()
            self._labels.clear()
            self._mask = None
            self._target = None
            self._reach_plan = None
            self._reach_preview_request_id = None
            self._reach_execute_request_id = None
            self._save_artifact()
        self._republish_overlay()
        self._set_state("FRAME_FROZEN", "D435 evidence frozen; click the object in the RViz panel")

    def _add_prompt(self, command: dict[str, Any]) -> None:
        if self._overlay_stage == "ARM":
            self._add_arm_prompt(command)
            return
        if not self._active or self._frozen is None:
            self._set_state("BLOCKED", "Freeze a D435 frame before adding a point prompt")
            return
        try:
            u, v = int(command["u"]), int(command["v"])
            label = int(command.get("label", 1))
        except (KeyError, TypeError, ValueError):
            self._set_state("BLOCKED", "Point prompt requires integer u, v, and label")
            return
        height, width = self._frozen["rgb"].shape[:2]
        if not (0 <= u < width and 0 <= v < height) or label not in {0, 1}:
            self._set_state("BLOCKED", "Point prompt is outside the frozen image or has invalid label")
            return
        with self._lock:
            if label == 0 and not any(item == 1 for item in self._labels):
                self._set_state("BLOCKED", "Add a positive target point before negative prompts")
                return
            self._prompts.append((u, v))
            self._labels.append(label)
            self._job_id += 1
            job_id = self._job_id
            rgb = self._frozen["rgb"].copy()
            points = list(self._prompts)
            labels = list(self._labels)
            self._save_artifact()
        self._set_state("SAM_LOADING" if self._sam.predictor is None else "SAM_RUNNING",
                        "SAM 2.1 Tiny is processing the frozen image on CPU")
        threading.Thread(
            target=self._segment_worker,
            args=(job_id, rgb, points, labels),
            daemon=True,
            name="sam21-touch-worker",
        ).start()

    def _segment_worker(
        self, job_id: int, rgb: np.ndarray,
        points: list[tuple[int, int]], labels: list[int],
    ) -> None:
        try:
            with self._sam_lock:
                with self._lock:
                    if job_id != self._job_id:
                        return
                    image_key = str(self._frozen["id"]) if self._frozen else "stale"
                mask, score, timing = self._sam.predict(
                    rgb, points, labels, image_key=image_key
                )
            with self._lock:
                if job_id != self._job_id or self._frozen is None:
                    return
                self._mask = mask
                positive_indices = [i for i, label in enumerate(labels) if label == 1]
                if not positive_indices:
                    raise ValueError("at least one positive prompt is required")
                click = points[positive_indices[-1]]
                target = robust_surface_target(
                    mask, self._frozen["depth_m"], click,
                    self._frozen["camera_matrix"],
                    distortion_model=self._frozen["distortion_model"],
                    distortion=np.asarray(self._frozen["distortion"], dtype=np.float64),
                )
                target["sam_score"] = score
                target["sam_timing"] = timing
                target["click_uv"] = list(click)
                target["mask_pixels"] = int(np.count_nonzero(mask))
                mask_pixels_vu = np.argwhere(mask)
                if len(mask_pixels_vu) > 0:
                    min_v, min_u = np.min(mask_pixels_vu, axis=0)
                    max_v, max_u = np.max(mask_pixels_vu, axis=0)
                    bbox_width_px = int(max_u - min_u + 1)
                    bbox_height_px = int(max_v - min_v + 1)
                    fx = float(self._frozen["camera_matrix"][0, 0])
                    fy = float(self._frozen["camera_matrix"][1, 1])
                    median_depth = float(target["median_depth_m"])
                    target["d435_mask_extent"] = {
                        "bbox_px": [bbox_width_px, bbox_height_px],
                        "approx_width_m": median_depth * bbox_width_px / fx,
                        "approx_height_m": median_depth * bbox_height_px / fy,
                        "role": "AUXILIARY_CROSS_CAMERA_SCALE_NOT_CONTROL_GATE",
                    }
                minimum_depth = float(self.get_parameter("minimum_target_depth_m").value)
                maximum_depth = float(self.get_parameter("maximum_target_depth_m").value)
                max_mad = float(self.get_parameter("maximum_depth_mad_m").value)
                max_plane = float(self.get_parameter("maximum_plane_p95_m").value)
                depth = float(target["median_depth_m"])
                if not minimum_depth <= depth <= maximum_depth:
                    raise ValueError(
                        f"target depth {depth:.3f}m outside verified observation range "
                        f"[{minimum_depth:.3f}, {maximum_depth:.3f}]m"
                    )
                if float(target["depth_mad_m"]) > max_mad:
                    raise ValueError(
                        f"local depth MAD {target['depth_mad_m']:.4f}m exceeds {max_mad:.4f}m"
                    )
                if float(target["plane_residual_p95_m"]) > max_plane:
                    raise ValueError(
                        f"surface plane residual p95 {target['plane_residual_p95_m']:.4f}m "
                        f"exceeds {max_plane:.4f}m"
                    )
                target.update(self._transform_target(target))
                reacquire = self._d435_reacquire_reference
                if reacquire is not None:
                    original = np.asarray(
                        reacquire["target"]["point_target"], dtype=np.float64
                    )
                    observed = np.asarray(target["point_target"], dtype=np.float64)
                    shift = float(np.linalg.norm(observed - original))
                    maximum_shift = float(
                        self.get_parameter("d435_reacquire_max_target_shift_m").value
                    )
                    if shift > maximum_shift:
                        raise ValueError(
                            f"reacquired D435 point shifted {shift * 1000.0:.1f}mm "
                            f"(limit {maximum_shift * 1000.0:.1f}mm)"
                        )
                    # The validated FAR candidate was derived from the original
                    # operator-confirmed metric target.  Reacquisition proves
                    # continuity only; it must not silently alter that plan.
                    self._target = dict(reacquire["target"])
                    self._reach_plan = dict(reacquire["plan"])
                    self._target["reacquisition"] = {
                        "observed_point_target": observed.tolist(),
                        "shift_m": shift,
                        "sam_score": float(score),
                        "projected_uv": list(reacquire["projected_uv"]),
                    }
                    self._d435_reacquire_reference = None
                    final_state = "D435_TARGET_REACQUIRED"
                    final_message = (
                        f"Same D435 target reacquired at the measured reference camera pose; "
                        f"3-D shift={shift * 1000.0:.1f}mm, SAM score={score:.3f}; "
                        "automatically executing the already validated FAR candidate"
                    )
                else:
                    self._target = target
                    final_state = "TARGET_READY"
                    final_message = (
                        f"SAM score={score:.3f}; depth={target['median_depth_m']:.3f}m; "
                        "3-D preview only—no arm command was sent"
                    )
                self._save_artifact()
            self._republish_overlay()
            self._publish_target()
            self._set_state(final_state, final_message)
            if final_state == "D435_TARGET_REACQUIRED":
                self._execute_pre_touch_far()
        except Exception as exc:
            with self._lock:
                if job_id != self._job_id:
                    return
                reacquiring = self._d435_reacquire_reference is not None
                if reacquiring:
                    saved = self._d435_reacquire_reference
                    self._target = dict(saved["target"])
                    self._reach_plan = dict(saved["plan"])
                    self._d435_reacquire_reference = None
                self._save_artifact()
            self.get_logger().error(f"Touch segmentation/geometry rejected: {exc}")
            self._set_state(
                "D435_TARGET_REACQUIRE_REQUIRED" if reacquiring else "REJECTED",
                (f"Automatic same-target verification failed: {exc}; "
                 "adjust/freeze and select the same object again") if reacquiring else str(exc),
            )

    def _clear_arm_observation(self) -> None:
        with self._lock:
            self._arm_frozen = None
            self._arm_prompts.clear()
            self._arm_labels.clear()
            self._arm_mask = None
            self._arm_target = None
            self._arm_target_confirmed = False
            self._approach_plan = None
            self._approach_preview_request_id = None
            self._overlay_stage = "ARM"

    def _reset_automatic_approach(self) -> None:
        self._automatic_approach = False
        self._automatic_approach_started = None
        self._automatic_approach_steps = 0
        self._automatic_reacquire_due = None
        self._automatic_tracking_reference = None
        self._automatic_reacquire_pending = False
        self._automatic_last_pixel_error = None
        self._automatic_nonconverging_steps = 0
        self._last_executed_x_remaining_m = None
        self._cartesian_correction_retries = 0
        self._press_phase_active = False
        self._approach_far_tip_x_m = None
        self._near_phase_latched = False
        self._far_arm_acquire_due = None
        self._arm_appearance_reference = None
        self._arm_appearance_reference_id = None
        self._touch_success_change_streak = 0
        self._home_preview_request_id = None
        self._home_execute_request_id = None

    @staticmethod
    def _summarize_motor_samples(samples: list[dict[str, Any]]) -> dict[str, Any]:
        names = sorted({
            name for sample in samples for name in sample.get("current_raw", {})
        })
        motors: dict[str, Any] = {}
        for name in names:
            values = np.asarray([
                sample["current_raw"][name]
                for sample in samples if name in sample.get("current_raw", {})
            ], dtype=np.float64)
            if values.size == 0:
                continue
            median = float(np.median(values))
            motors[name] = {
                "sample_count": int(values.size),
                "median_raw": median,
                "mad_raw": float(np.median(np.abs(values - median))),
                "median_abs_raw": float(np.median(np.abs(values))),
                "minimum_raw": int(np.min(values)),
                "maximum_raw": int(np.max(values)),
                "maximum_abs_raw": int(np.max(np.abs(values))),
            }
        return {
            "sample_count": len(samples),
            "duration_sec": (
                0.0 if len(samples) < 2 else
                float(samples[-1]["monotonic_sec"] - samples[0]["monotonic_sec"])
            ),
            "motors": motors,
        }

    def _capture_far_motor_baseline(self) -> None:
        started = self._far_motor_baseline_started
        if started is None or self._far_motor_baseline is not None:
            return
        samples = [
            sample for sample in self._motor_samples
            if float(sample["monotonic_sec"]) >= started
        ]
        summary = self._summarize_motor_samples(samples)
        summary.update({
            "source": "/arm/dashboard_state motor_health.current_raw",
            "underlying_source": "/arm/hardware_status direct DYNAMIXEL Present_Current registers",
            "captured_at_wall_time_sec": time.time(),
            "phase": "PRETOUCH_FAR_SETTLED",
            "motor_health_snapshot": jsonable(self._arm_state.get("motor_health", {})),
            "joint_feedback_rad": dict(self._arm_joints),
        })
        self._far_motor_baseline = summary
        self._save_motor_evidence()

    def _record_completed_motor_step(
        self, execution: dict[str, Any]
    ) -> dict[str, Any] | None:
        started = self._active_step_motor_started
        if started is None:
            return None
        samples = [
            sample for sample in self._motor_samples
            if float(sample["monotonic_sec"]) >= started
        ]
        summary = self._summarize_motor_samples(samples)
        baseline_motors = (
            self._far_motor_baseline or {}
        ).get("motors", {})
        for name, motor in summary["motors"].items():
            baseline = baseline_motors.get(name)
            motor["baseline_median_raw"] = (
                None if baseline is None else baseline.get("median_raw")
            )
            motor["peak_abs_delta_from_baseline_raw"] = (
                None if baseline is None else max(
                    abs(float(motor["minimum_raw"]) - float(baseline["median_raw"])),
                    abs(float(motor["maximum_raw"]) - float(baseline["median_raw"])),
                )
            )
        cartesian_feedback: dict[str, Any]
        try:
            _, completed_tip = self._rotation_translation(
                self._latest_transform("base_link", "arm/touch_tip")
            )
            planned = np.asarray(
                (self._approach_plan or {}).get("waypoint_base_m", []),
                dtype=np.float64,
            )
            start = np.asarray(
                (self._approach_plan or {}).get("actual_tip_base_m", []),
                dtype=np.float64,
            )
            cartesian_feedback = {
                "completed_tip_base_m": completed_tip.tolist(),
                "planned_waypoint_base_m": (
                    planned.tolist() if planned.shape == (3,) else None
                ),
                "planned_start_tip_base_m": (
                    start.tolist() if start.shape == (3,) else None
                ),
                "achieved_delta_base_m": (
                    (completed_tip - start).tolist()
                    if start.shape == (3,) else None
                ),
                "waypoint_error_base_m": (
                    (completed_tip - planned).tolist()
                    if planned.shape == (3,) else None
                ),
                "waypoint_error_norm_m": (
                    float(np.linalg.norm(completed_tip - planned))
                    if planned.shape == (3,) else None
                ),
                "source": "TF base_link <- arm/touch_tip after gateway completion",
            }
            requested_delta = (
                planned - start
                if planned.shape == (3,) and start.shape == (3,)
                else np.zeros(3, dtype=np.float64)
            )
            achieved_delta = (
                completed_tip - start
                if start.shape == (3,) else np.zeros(3, dtype=np.float64)
            )
            requested_norm_sq = float(np.dot(requested_delta, requested_delta))
            requested_norm = math.sqrt(requested_norm_sq)
            progress_ratio = (
                float(np.dot(achieved_delta, requested_delta) / requested_norm_sq)
                if requested_norm_sq > 1.0e-10 else 1.0
            )
            step_kind = str((self._approach_plan or {}).get("step_kind", ""))
            press_like = step_kind in {
                "ALIGNED_PRESS_FORWARD_WITH_FIXED_TIP_YZ",
                "ALIGN_FIXED_TIP_YZ",
                "NEAR_ALIGN_IMAGE_CENTER_YZ",
            }
            maximum_error = float(self.get_parameter(
                "approach_cartesian_press_max_error_m"
                if press_like else "approach_cartesian_coarse_max_error_m"
            ).value)
            minimum_progress = float(self.get_parameter(
                "approach_cartesian_min_progress_ratio"
            ).value)
            correctable_maximum = float(self.get_parameter(
                "approach_cartesian_correctable_max_error_m"
            ).value)
            waypoint_error = cartesian_feedback["waypoint_error_norm_m"]
            # Endpoint accuracy is the primary acceptance evidence.  In
            # particular, a short Cartesian correction can have a noisy or
            # even negative progress *ratio* because encoder/TF residuals are
            # comparable to the requested displacement.  Do not reject an
            # endpoint that is already inside the stage-specific envelope.
            accepted = bool(
                waypoint_error is not None
                and float(waypoint_error) <= maximum_error
            )
            # A bounded residual is not success, but it is useful feedback: the
            # next visual cycle can solve again from the measured tip pose.
            # For corrections shorter than 15 mm, directional ratio is not a
            # reliable hard gate.  For larger requests, genuinely wrong-way
            # motion still blocks another automatic correction.
            short_request = requested_norm < 0.015
            correction_direction_allowed = short_request or progress_ratio >= 0.0
            correctable = bool(
                not accepted
                and waypoint_error is not None
                and float(waypoint_error) <= correctable_maximum
                and correction_direction_allowed
            )
            cartesian_feedback["acceptance"] = {
                "accepted": accepted,
                "correctable": correctable,
                "step_kind": step_kind,
                "maximum_waypoint_error_m": maximum_error,
                "correctable_maximum_waypoint_error_m": correctable_maximum,
                "minimum_directional_progress_ratio": minimum_progress,
                "directional_progress_ratio": progress_ratio,
                "requested_displacement_m": requested_norm,
                "direction_gate_applied": not short_request,
                "reason": (
                    "measured endpoint passed the stage-specific Cartesian error gate"
                    if accepted else
                    f"waypoint error={float(waypoint_error) * 1000.0:.1f}mm "
                    f"(limit {maximum_error * 1000.0:.1f}mm), directional "
                    f"progress={progress_ratio:.2f} (minimum {minimum_progress:.2f}); "
                    f"requested displacement={requested_norm * 1000.0:.1f}mm; "
                    f"direction gate={'applied' if not short_request else 'diagnostic only'}; "
                    f"bounded closed-loop correction={'allowed' if correctable else 'blocked'}"
                ),
            }
        except (TypeError, ValueError, TransformException) as exc:
            cartesian_feedback = {
                "available": False,
                "reason": str(exc),
                "acceptance": {
                    "accepted": False,
                    "reason": f"Cartesian feedback unavailable: {exc}",
                },
            }
        summary.update({
            "step_index": self._automatic_approach_steps + 1,
            "step_outcome": str(execution.get("state", "UNKNOWN")),
            "source": "/arm/dashboard_state motor_health.current_raw",
            "gateway_execution": jsonable(execution),
            "planned_approach": jsonable(self._approach_plan),
            "joint_feedback_rad_at_completion": dict(self._arm_joints),
            "motor_health_at_completion": jsonable(
                self._arm_state.get("motor_health", {})
            ),
            "cartesian_feedback_at_completion": cartesian_feedback,
            "completed_at_wall_time_sec": time.time(),
        })
        self._motor_step_evidence.append(summary)
        self._active_step_motor_started = None
        self._save_motor_evidence()
        return cartesian_feedback

    def _save_motor_evidence(self) -> None:
        if self._frozen is None:
            return
        root = Path(str(self.get_parameter("artifact_root").value)).expanduser()
        directory = root / f"observation_{self._frozen['id']}"
        directory.mkdir(parents=True, exist_ok=True)
        evidence = {
            "schema_version": 1,
            "far_baseline": jsonable(self._far_motor_baseline),
            "approach_steps": jsonable(self._motor_step_evidence),
            "interpretation": (
                "Recorded evidence only; no contact threshold is claimed until real "
                "touch trials establish per-motor delta distributions."
            ),
        }
        temporary = directory / "motor_evidence.json.tmp"
        temporary.write_text(
            json.dumps(evidence, ensure_ascii=False, indent=2, allow_nan=False),
            encoding="utf-8",
        )
        temporary.replace(directory / "motor_evidence.json")

    def _pause_automatic_approach(self, state: str, reason: str) -> None:
        with self._lock:
            self._automatic_approach = False
            self._automatic_reacquire_due = None
            self._automatic_reacquire_pending = False
        self._set_state(state, reason)

    def _clear_arm_prompts(self) -> None:
        with self._lock:
            if self._arm_frozen is None:
                self._set_state("BLOCKED", "Freeze an arm-camera frame first")
                return
            self._arm_prompts.clear()
            self._arm_labels.clear()
            self._arm_mask = None
            self._arm_target = None
            self._arm_target_confirmed = False
            self._reset_automatic_approach()
            self._job_id += 1
        self._republish_overlay()
        self._set_state(
            "ARM_FRAME_READY",
            "Arm-camera prompts cleared; click the real target in the wrist image",
        )

    def _latest_transform(self, destination: str, source: str):
        try:
            return self._tf_buffer.lookup_transform(
                destination, source, Time(), timeout=Duration(seconds=0.25)
            )
        except TransformException as exc:
            raise ValueError(f"TF {destination} <- {source} unavailable: {exc}") from exc

    @staticmethod
    def _rotation_translation(transform) -> tuple[np.ndarray, np.ndarray]:
        rotation_message = transform.transform.rotation
        rotation = quaternion_matrix_xyzw(np.array([
            rotation_message.x, rotation_message.y,
            rotation_message.z, rotation_message.w,
        ]))
        translation_message = transform.transform.translation
        translation = np.array([
            translation_message.x, translation_message.y, translation_message.z,
        ], dtype=np.float64)
        return rotation, translation

    def _arm_camera_tracking_point(self, centroid_uv: np.ndarray) -> np.ndarray:
        """Lift an Arm Camera centroid to a retained base-frame tracking point.

        D435 contributes only the initial range scale.  After handoff, target
        prediction is propagated from the confirmed Arm Camera ray through the
        calibrated CameraInfo and changing wrist-camera TF; the D435 pixel
        projection is not reused.
        """
        if self._arm_frozen is None or self._target is None:
            raise ValueError("Arm Camera frame or retained metric target is unavailable")
        geometry = monocular_lateral_target(
            tuple(np.asarray(centroid_uv, dtype=np.float64)),
            np.asarray(self._arm_frozen["camera_matrix"], dtype=np.float64),
            np.asarray(self._arm_frozen["distortion"], dtype=np.float64),
            np.asarray(
                self._arm_frozen["base_from_camera_rotation"], dtype=np.float64
            ),
            np.asarray(
                self._arm_frozen["base_from_camera_translation"], dtype=np.float64
            ),
            np.asarray(self._target["point_target"], dtype=np.float64),
        )
        return np.asarray(geometry["ray_point_base_m"], dtype=np.float64)

    def _acquire_arm_target(self, automatic_reacquire: bool = False) -> None:
        if not self._active or self._target is None:
            if automatic_reacquire:
                self._pause_automatic_approach(
                    "AUTO_TARGET_LOST", "A retained D435 target is required"
                )
            else:
                self._set_state("BLOCKED", "A retained D435 target is required")
            return
        if self._state not in {
            "PRETOUCH_FAR_REACHED", "ARM_REACQUIRE_REQUIRED", "ARM_FRAME_READY",
            "ARM_TARGET_READY", "ARM_TARGET_REJECTED", "APPROACH_GATEWAY_REJECTED",
            "APPROACH_BLOCKED", "APPROACH_UNREACHABLE", "APPROACH_EXECUTION_FAILED",
            "AUTO_APPROACH_SETTLING", "AUTO_APPROACH_CARTESIAN_CORRECTING",
            "TOUCH_CHANGE_VERIFYING", "AUTO_TARGET_LOST",
            "AUTO_APPROACH_TIMEOUT", "AUTO_APPROACH_STEP_LIMIT",
        }:
            self._set_state(
                "BLOCKED", "Arm-camera acquisition is allowed only after FAR is reached"
            )
            return
        allowed, reason = self._arm_gate()
        if not allowed:
            if automatic_reacquire:
                self._pause_automatic_approach("AUTO_TARGET_LOST", reason)
            else:
                self._set_state("BLOCKED", reason)
            return
        if (
            self._last_mechanism_motion is not None
            and time.monotonic() - self._last_mechanism_motion < 0.50
        ):
            if automatic_reacquire:
                self._automatic_reacquire_due = time.monotonic() + 0.20
                self._set_state(
                    "AUTO_APPROACH_SETTLING",
                    "Arm feedback is still settling; automatic reacquisition deferred",
                )
            else:
                self._set_state("BLOCKED", "Arm is still settling; wait 0.5 s and retry")
            return
        with self._lock:
            if not self._arm_color or not self._arm_info:
                self._set_state("BLOCKED", "Arm-camera RGB or CameraInfo is missing")
                return
            color = self._arm_color[-1]
            color_stamp = stamp_seconds(color)
            info = min(self._arm_info, key=lambda item: abs(stamp_seconds(item) - color_stamp))
            info_skew = abs(stamp_seconds(info) - color_stamp)
            if info_skew > 0.25:
                self._set_state(
                    "BLOCKED", f"Arm-camera CameraInfo skew {info_skew:.3f}s is too large"
                )
                return
            rgb = self._bridge.imgmsg_to_cv2(color, desired_encoding="rgb8").copy()
            frame_id = str(color.header.frame_id)
            if not frame_id:
                self._set_state("BLOCKED", "Arm-camera image frame_id is empty")
                return
            try:
                base_from_camera = self._latest_transform("base_link", frame_id)
                rotation, translation = self._rotation_translation(base_from_camera)
                camera_matrix = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
                distortion = np.asarray(info.d, dtype=np.float64)
            except ValueError as exc:
                self._set_state("BLOCKED", str(exc))
                return
            height, width = rgb.shape[:2]
            margin = int(self.get_parameter("arm_projection_image_margin_px").value)
            projected = None
            projection_in_image = False
            # D435 projection is a one-shot cross-camera handoff prompt only.
            # Once the operator has confirmed the Arm Camera object, it must
            # not influence or clutter any subsequent tracking frame.
            if not self._arm_handoff_complete:
                try:
                    point_base = np.asarray(
                        self._target["point_target"], dtype=np.float64
                    )
                    point_camera = rotation.T @ (point_base - translation)
                    projected = project_camera_point(
                        point_camera, camera_matrix, distortion
                    )
                    projection_in_image = (
                        margin <= projected[0] < width - margin
                        and margin <= projected[1] < height - margin
                    )
                except ValueError as exc:
                    self._set_state("BLOCKED", str(exc))
                    return
            self._arm_frozen = {
                "id": str(uuid.uuid4()),
                "rgb": rgb,
                "stamp": color_stamp,
                "stamp_msg": color.header.stamp,
                "frame_id": frame_id,
                "camera_matrix": camera_matrix,
                "distortion_model": str(info.distortion_model),
                "distortion": distortion.tolist(),
                "camera_info_skew_sec": info_skew,
                "base_from_camera_rotation": rotation,
                "base_from_camera_translation": translation,
                "d435_projected_uv": (
                    None if projected is None else list(projected)
                ),
                "d435_projection_in_image": projection_in_image,
            }
            self._arm_prompts = []
            self._arm_labels = []
            self._arm_mask = None
            self._arm_target = None
            self._arm_target_confirmed = False
            self._approach_plan = None
            self._overlay_stage = "ARM"
            self._automatic_reacquire_pending = False
            self._job_id += 1
            job_id = self._job_id
            tracking_reference = self._automatic_tracking_reference
            tracking_prompt = None
            if automatic_reacquire and tracking_reference is not None:
                tracking_point = np.asarray(
                    tracking_reference.get("tracking_point_base_m", []),
                    dtype=np.float64,
                )
                if tracking_point.shape == (3,) and np.all(np.isfinite(tracking_point)):
                    try:
                        current_tracking_range = float(
                            np.linalg.norm(tracking_point - translation)
                        )
                        previous_tracking_range = float(
                            tracking_reference.get(
                                "tracking_range_m", current_tracking_range
                            )
                        )
                        if current_tracking_range > 0.02 and previous_tracking_range > 0.02:
                            self._arm_frozen["expected_mask_area_scale"] = float(
                                np.clip(
                                    (previous_tracking_range / current_tracking_range) ** 2,
                                    0.1, 10.0,
                                )
                            )
                            self._arm_frozen["tracking_range_m"] = current_tracking_range
                        predicted_camera = rotation.T @ (tracking_point - translation)
                        predicted = project_camera_point(
                            predicted_camera, camera_matrix, distortion
                        )
                        candidate = (
                            int(round(float(predicted[0]))),
                            int(round(float(predicted[1]))),
                        )
                        if 0 <= candidate[0] < width and 0 <= candidate[1] < height:
                            tracking_prompt = candidate
                    except ValueError:
                        tracking_prompt = None
                # Compatibility fallback for observations created before the
                # calibrated tracking point was introduced.
                if tracking_prompt is None:
                    previous = tracking_reference.get("centroid_uv")
                    if isinstance(previous, (list, tuple)) and len(previous) == 2:
                        candidate = (
                            int(round(float(previous[0]))),
                            int(round(float(previous[1]))),
                        )
                        if 0 <= candidate[0] < width and 0 <= candidate[1] < height:
                            tracking_prompt = candidate
            auto_prompt = bool(
                self.get_parameter("arm_projection_auto_prompt").value
            )
            if tracking_prompt is not None:
                self._arm_prompts.append(tracking_prompt)
                self._arm_labels.append(1)
                self._arm_frozen["tracking_prompt_uv"] = list(tracking_prompt)
                self._arm_frozen["tracking_prompt_source"] = (
                    "CALIBRATED_ARM_RAY_PROPAGATED_THROUGH_CURRENT_CAMERA_TF"
                    if "tracking_point_base_m" in tracking_reference
                    else "PREVIOUS_CENTROID_COMPATIBILITY_FALLBACK"
                )
                self._automatic_reacquire_pending = True
            elif projection_in_image and auto_prompt and not automatic_reacquire:
                self._arm_prompts.append(
                    (int(round(projected[0])), int(round(projected[1])))
                )
                self._arm_labels.append(1)
        self._republish_overlay()
        if automatic_reacquire and tracking_prompt is None:
            self._pause_automatic_approach(
                "AUTO_TARGET_LOST",
                "Previous arm-camera target centroid is unavailable; click the object again",
            )
            return
        if tracking_prompt is not None:
            self._set_state(
                "AUTO_ARM_SAM_RUNNING",
                f"Automatic visual reacquisition after step {self._automatic_approach_steps}; "
                "the previous confirmed arm-camera centroid is the SAM prompt",
            )
            threading.Thread(
                target=self._segment_arm_worker,
                args=(job_id, rgb, list(self._arm_prompts), list(self._arm_labels)),
                daemon=True,
                name="sam21-arm-camera-auto-worker",
            ).start()
            return
        if not projection_in_image:
            self._set_state(
                "ARM_FRAME_READY",
                "D435 projection is outside the arm-camera image; click the real target manually",
            )
            return
        if not auto_prompt:
            self._set_state(
                "ARM_FRAME_READY",
                "Arm frame acquired; orange D435 projection is reference only. "
                "Click the real target manually",
            )
            return
        self._set_state(
            "ARM_SAM_RUNNING",
            "Reusing loaded SAM 2.1 weights on the frozen arm-camera image; "
            "the projected D435 point is only an initial prompt",
        )
        threading.Thread(
            target=self._segment_arm_worker,
            args=(job_id, rgb, list(self._arm_prompts), list(self._arm_labels)),
            daemon=True,
            name="sam21-arm-camera-worker",
        ).start()

    def _add_arm_prompt(self, command: dict[str, Any]) -> None:
        if not self._active or self._arm_frozen is None:
            self._set_state("BLOCKED", "Acquire an arm-camera frame first")
            return
        try:
            u, v = int(command["u"]), int(command["v"])
            label = int(command.get("label", 1))
        except (KeyError, TypeError, ValueError):
            self._set_state("BLOCKED", "Arm-camera prompt requires integer u, v, and label")
            return
        height, width = self._arm_frozen["rgb"].shape[:2]
        if not (0 <= u < width and 0 <= v < height) or label not in {0, 1}:
            self._set_state("BLOCKED", "Arm-camera prompt is outside the image")
            return
        with self._lock:
            # A new positive click is authoritative and replaces the uncertain
            # cross-camera projected positive prompt. Keep negative prompts.
            if label == 1:
                negatives = [
                    point for point, old_label in zip(self._arm_prompts, self._arm_labels)
                    if old_label == 0
                ]
                self._arm_prompts = negatives + [(u, v)]
                self._arm_labels = [0] * len(negatives) + [1]
            else:
                if not any(item == 1 for item in self._arm_labels):
                    self._set_state("BLOCKED", "Add a positive arm target before exclusions")
                    return
                self._arm_prompts.append((u, v))
                self._arm_labels.append(0)
            self._arm_target_confirmed = False
            self._job_id += 1
            job_id = self._job_id
            rgb = self._arm_frozen["rgb"].copy()
            points = list(self._arm_prompts)
            labels = list(self._arm_labels)
        self._set_state("ARM_SAM_RUNNING", "SAM 2.1 is refining the arm-camera target")
        threading.Thread(
            target=self._segment_arm_worker,
            args=(job_id, rgb, points, labels), daemon=True,
            name="sam21-arm-camera-worker",
        ).start()

    def _segment_arm_worker(
        self, job_id: int, rgb: np.ndarray,
        points: list[tuple[int, int]], labels: list[int],
    ) -> None:
        try:
            with self._sam_lock:
                with self._lock:
                    if job_id != self._job_id or self._arm_frozen is None:
                        return
                    image_key = f"arm-{self._arm_frozen['id']}"
                mask, score, timing = self._sam.predict(
                    rgb, points, labels, image_key=image_key
                )
            pixels = np.argwhere(mask)
            minimum_pixels = int(
                self.get_parameter("arm_sam_minimum_mask_pixels").value
            )
            minimum_score = float(self.get_parameter("arm_sam_minimum_score").value)
            mask_fraction = float(len(pixels)) / float(mask.size)
            large_mask_fraction = float(
                self.get_parameter("arm_sam_large_mask_fraction").value
            )
            if mask_fraction >= large_mask_fraction:
                minimum_score = float(self.get_parameter(
                    "arm_sam_large_mask_minimum_score"
                ).value)
            if self._press_phase_active:
                minimum_score = min(
                    minimum_score,
                    float(self.get_parameter("arm_sam_press_minimum_score").value),
                )
            if len(pixels) < minimum_pixels:
                disappearance = self._appearance_change_at_previous_target(rgb)
                minimum_success_step = int(
                    self.get_parameter("touch_success_minimum_step").value
                )
                success_gate = self._touch_success_gate(None, rgb.shape[:2])
                if (
                    self._automatic_reacquire_pending
                    and self._automatic_approach_steps >= minimum_success_step
                    and bool(disappearance.get("changed"))
                    and bool(success_gate.get("eligible"))
                ):
                    disappearance["success_gate"] = success_gate
                    if not self._confirm_touch_change_observation(disappearance):
                        return
                    with self._lock:
                        if job_id != self._job_id or self._arm_frozen is None:
                            return
                        self._arm_mask = mask
                        self._arm_target = {
                            "mask_pixels": int(len(pixels)),
                            "sam_score": float(score),
                            "source": "ARM_CAMERA_RGB_TARGET_DISAPPEARANCE",
                            "touch_success_evidence": jsonable(disappearance),
                        }
                    self._republish_overlay()
                    self._complete_touch_and_return_home(disappearance)
                    return
                self._touch_success_change_streak = 0
                raise ValueError(
                    f"arm-camera SAM mask has {len(pixels)} pixels; need {minimum_pixels}"
                )
            appearance = self._appearance_change(rgb, mask)
            minimum_success_step = int(
                self.get_parameter("touch_success_minimum_step").value
            )
            centroid = np.mean(pixels[:, [1, 0]], axis=0)
            success_gate = self._touch_success_gate(centroid, rgb.shape[:2])
            if (
                self._automatic_reacquire_pending
                and self._automatic_approach_steps >= minimum_success_step
                and bool(appearance.get("changed"))
                and bool(success_gate.get("eligible"))
            ):
                appearance["success_gate"] = success_gate
                if not self._confirm_touch_change_observation(appearance):
                    return
                with self._lock:
                    if job_id != self._job_id or self._arm_frozen is None:
                        return
                    self._arm_mask = mask
                    self._arm_target = {
                        "centroid_uv": centroid.tolist(),
                        "mask_pixels": int(len(pixels)),
                        "sam_score": float(score),
                        "sam_minimum_score_applied": minimum_score,
                        "mask_fraction": mask_fraction,
                        "source": "ARM_CAMERA_RGB_LOCAL_APPEARANCE_CHANGE",
                        "touch_success_evidence": jsonable(appearance),
                    }
                self._republish_overlay()
                self._complete_touch_and_return_home(appearance)
                return
            self._touch_success_change_streak = 0
            if score < minimum_score:
                raise ValueError(
                    f"arm-camera SAM score {score:.3f} is below {minimum_score:.3f}"
                )
            height, width = mask.shape
            min_v, min_u = np.min(pixels, axis=0)
            max_v, max_u = np.max(pixels, axis=0)
            border_margin = int(min(min_u, min_v, width - 1 - max_u, height - 1 - max_v))
            with self._lock:
                if job_id != self._job_id or self._arm_frozen is None:
                    return
                self._arm_mask = mask
                self._arm_target = {
                    "centroid_uv": centroid.tolist(),
                    "mask_pixels": int(len(pixels)),
                    "sam_score": float(score),
                    "sam_minimum_score_applied": minimum_score,
                    "mask_fraction": mask_fraction,
                    "sam_timing": timing,
                    "border_margin_px": border_margin,
                    "source": "ARM_CAMERA_RGB_SAM21",
                    "metric_depth_observed": False,
                    "operator_confirmed_same_object": False,
                }
                self._arm_target_confirmed = False
                self._save_arm_artifact()
                automatic_reacquire = self._automatic_reacquire_pending
                tracking_reference = (
                    None if self._automatic_tracking_reference is None
                    else dict(self._automatic_tracking_reference)
                )
                predicted_tracking_prompt = (
                    None if self._arm_frozen is None
                    else self._arm_frozen.get("tracking_prompt_uv")
                )
                expected_mask_area_scale = (
                    1.0 if self._arm_frozen is None
                    else float(self._arm_frozen.get("expected_mask_area_scale", 1.0))
                )
                self._automatic_reacquire_pending = False
            self._republish_overlay()
            if automatic_reacquire:
                if tracking_reference is None:
                    self._pause_automatic_approach(
                        "AUTO_TARGET_LOST",
                        "Automatic target reference disappeared; click and confirm again",
                    )
                    return
                previous_area = float(tracking_reference.get("mask_pixels", 0))
                expected_area = max(1.0, previous_area * expected_mask_area_scale)
                max_shift = float(
                    self.get_parameter("automatic_arm_target_max_shift_px").value
                )
                min_ratio = float(
                    self.get_parameter("automatic_arm_target_min_area_ratio").value
                )
                max_ratio = float(
                    self.get_parameter("automatic_arm_target_max_area_ratio").value
                )
                continuity = validate_tracking_continuity(
                    (
                        predicted_tracking_prompt
                        if predicted_tracking_prompt is not None
                        else tracking_reference.get("centroid_uv", [])
                    ),
                    centroid,
                    int(round(expected_area)), int(len(pixels)),
                    max_shift, min_ratio, max_ratio,
                )
                shift = float(continuity["centroid_shift_px"])
                area_ratio = float(continuity["mask_area_ratio"])
                if not continuity["valid"]:
                    # A pushed object can disappear or leave only a small SAM
                    # fragment.  Before calling that TARGET_LOST, compare the
                    # previous target neighbourhood against the immutable
                    # operator-confirmed arm-camera colour baseline.  This is
                    # deliberately subject to the same near-target/fixed-tip
                    # geometry gate and repeated-frame confirmation as the
                    # normal Touch-success path.
                    disappearance = self._appearance_change_at_previous_target(rgb)
                    success_gate = self._touch_success_gate(None, rgb.shape[:2])
                    minimum_success_step = int(
                        self.get_parameter("touch_success_minimum_step").value
                    )
                    if (
                        self._automatic_approach_steps >= minimum_success_step
                        and bool(disappearance.get("changed"))
                        and bool(success_gate.get("eligible"))
                    ):
                        disappearance.update({
                            "success_gate": success_gate,
                            "continuity_failure": jsonable(continuity),
                            "evidence_kind": (
                                "HUMAN_CONFIRMED_REFERENCE_LOCAL_COLOR_CHANGE_"
                                "BEFORE_TARGET_LOST"
                            ),
                        })
                        if not self._confirm_touch_change_observation(disappearance):
                            return
                        with self._lock:
                            if job_id != self._job_id or self._arm_frozen is None:
                                return
                            self._arm_target["touch_success_evidence"] = jsonable(
                                disappearance
                            )
                            self._save_arm_artifact()
                        self._complete_touch_and_return_home(disappearance)
                        return
                    self._touch_success_change_streak = 0
                    self._pause_automatic_approach(
                        "AUTO_TARGET_LOST",
                        f"Arm-camera target continuity rejected: centroid shift={shift:.1f}px "
                        f"(limit {max_shift:.1f}), mask area ratio={area_ratio:.2f} "
                        f"(range {min_ratio:.2f}..{max_ratio:.2f}); click and confirm again",
                    )
                    return
                with self._lock:
                    self._arm_target_confirmed = True
                    self._arm_target["operator_confirmed_same_object"] = False
                    self._arm_target["automatic_continuity_verified"] = True
                    self._arm_target["centroid_shift_px"] = shift
                    self._arm_target["predicted_prompt_uv"] = (
                        None if predicted_tracking_prompt is None
                        else list(predicted_tracking_prompt)
                    )
                    self._arm_target["mask_area_ratio"] = area_ratio
                    self._arm_target["expected_mask_area_scale"] = (
                        expected_mask_area_scale
                    )
                    if self._arm_frozen is not None:
                        tracking_range = self._arm_frozen.get(
                            "tracking_range_m",
                            tracking_reference.get("tracking_range_m"),
                        )
                        if tracking_range is not None and math.isfinite(
                            float(tracking_range)
                        ):
                            self._arm_target["tracking_range_m"] = float(
                                tracking_range
                            )
                    if "tracking_point_base_m" in tracking_reference:
                        self._arm_target["tracking_point_base_m"] = list(
                            tracking_reference["tracking_point_base_m"]
                        )
                    self._automatic_tracking_reference = dict(self._arm_target)
                    self._save_arm_artifact()
                self._set_state(
                    "ARM_TARGET_CONFIRMED",
                    f"Automatic arm-camera continuity passed after step "
                    f"{self._automatic_approach_steps}: shift={shift:.1f}px, "
                    f"area ratio={area_ratio:.2f}; planning next guarded step",
                )
                self._plan_approach_step()
                return
            self._set_state(
                "ARM_TARGET_READY",
                f"Arm-camera SAM score={score:.3f}, mask={len(pixels)} px; "
                "confirm it is the same object or click the correct object",
            )
        except Exception as exc:
            self.get_logger().error(f"Arm-camera target segmentation rejected: {exc}")
            if self._automatic_approach or self._automatic_reacquire_pending:
                self._pause_automatic_approach(
                    "AUTO_TARGET_LOST",
                    f"Automatic arm-camera segmentation failed: {exc}; "
                    "click and confirm the object again",
                )
            else:
                self._set_state("ARM_TARGET_REJECTED", str(exc))

    def _confirm_arm_target(self) -> None:
        if self._state != "ARM_TARGET_READY" or self._arm_target is None:
            self._set_state("BLOCKED", "A valid arm-camera SAM target is required")
            return
        with self._lock:
            if self._arm_mask is None or self._arm_frozen is None:
                self._set_state("BLOCKED", "Arm-camera mask/image is unavailable")
                return
            if self._arm_appearance_reference is None:
                self._arm_appearance_reference = self._appearance_signature(
                    self._arm_frozen["rgb"], self._arm_mask
                )
                self._arm_appearance_reference_id = str(self._arm_frozen["id"])
                self._arm_appearance_reference.update({
                    "provenance": "FIRST_OPERATOR_CONFIRMED_ARM_CAMERA_REGION",
                    "arm_observation_id": self._arm_appearance_reference_id,
                    "immutable_for_touch_session": True,
                })
            self._arm_target["appearance_reference"] = jsonable(
                self._arm_appearance_reference
            )
            self._arm_target["appearance_reference_reused"] = bool(
                self._arm_appearance_reference_id != str(self._arm_frozen["id"])
            )
            tracking_point = self._arm_camera_tracking_point(
                np.asarray(self._arm_target["centroid_uv"], dtype=np.float64)
            )
            self._arm_target["tracking_point_base_m"] = tracking_point.tolist()
            self._arm_target["tracking_range_m"] = float(np.linalg.norm(
                tracking_point - np.asarray(
                    self._arm_frozen["base_from_camera_translation"],
                    dtype=np.float64,
                )
            ))
            self._arm_target_confirmed = True
            self._arm_handoff_complete = True
            self._arm_target["operator_confirmed_same_object"] = True
            self._arm_target["automatic_continuity_verified"] = False
            self._automatic_tracking_reference = dict(self._arm_target)
            self._automatic_approach = bool(
                self.get_parameter("automatic_approach_after_confirmation").value
            )
            self._automatic_approach_started = time.monotonic()
            self._automatic_approach_steps = 0
            self._automatic_reacquire_due = None
            self._automatic_last_pixel_error = None
            self._automatic_nonconverging_steps = 0
            self._touch_success_change_streak = 0
            self._approach_far_tip_x_m = None
            self._near_phase_latched = False
            self._save_arm_artifact()
        if not self._automatic_approach:
            self._set_state(
                "ARM_TARGET_CONFIRMED",
                "Arm-camera target confirmed; plan one bounded approach step",
            )
            return
        self._set_state(
            "ARM_TARGET_CONFIRMED",
            "Arm-camera target confirmed; automatic guarded stop-observe approach authorized",
        )
        self._plan_approach_step()

    @staticmethod
    def _appearance_signature(rgb: np.ndarray, mask: np.ndarray) -> dict[str, Any]:
        valid = np.asarray(mask, dtype=bool)
        if rgb.ndim != 3 or rgb.shape[2] != 3 or valid.shape != rgb.shape[:2]:
            raise ValueError("appearance image/mask shape mismatch")
        if int(np.count_nonzero(valid)) < 20:
            raise ValueError("appearance mask has fewer than 20 pixels")
        lab = cv2.cvtColor(rgb, cv2.COLOR_RGB2LAB).astype(np.float32)
        pixels = lab[valid]
        return {
            "median_lab": np.median(pixels, axis=0).tolist(),
            "sample_count": int(len(pixels)),
        }

    def _appearance_change(self, rgb: np.ndarray, mask: np.ndarray) -> dict[str, Any]:
        current = self._appearance_signature(rgb, mask)
        reference = self._arm_appearance_reference
        if reference is None:
            return {"changed": False, "reason": "NO_REFERENCE"}
        reference_lab = np.asarray(reference["median_lab"], dtype=np.float64)
        current_lab = np.asarray(current["median_lab"], dtype=np.float64)
        delta = float(np.linalg.norm(current_lab - reference_lab))
        threshold = float(self.get_parameter("touch_success_lab_delta").value)
        lab = cv2.cvtColor(rgb, cv2.COLOR_RGB2LAB).astype(np.float32)
        per_pixel_delta = np.linalg.norm(lab[np.asarray(mask, dtype=bool)] - reference_lab, axis=1)
        changed_fraction = float(np.mean(per_pixel_delta >= threshold))
        required_fraction = float(
            self.get_parameter("touch_success_changed_fraction").value
        )
        return {
            "changed": bool(delta >= threshold and changed_fraction >= required_fraction),
            "delta_lab": delta,
            "threshold_lab": threshold,
            "changed_fraction": changed_fraction,
            "required_changed_fraction": required_fraction,
            "reference": reference,
            "current": current,
        }

    def _touch_success_gate(
        self, centroid_uv: np.ndarray | None, image_shape: tuple[int, int]
    ) -> dict[str, Any]:
        remaining = (
            math.inf if self._last_executed_x_remaining_m is None
            else float(self._last_executed_x_remaining_m)
        )
        maximum_remaining = float(
            self.get_parameter("touch_success_max_x_remaining_m").value
        )
        if centroid_uv is None:
            tracking = self._automatic_tracking_reference or {}
            centroid_uv = np.asarray(tracking.get("centroid_uv", []), dtype=np.float64)
        else:
            centroid_uv = np.asarray(centroid_uv, dtype=np.float64)
        height, width = image_shape
        centre = np.asarray(
            self._fixed_arm_tip_pixel(width, height)
            if self._press_phase_active
            else ((width - 1.0) * 0.5, (height - 1.0) * 0.5),
            dtype=np.float64,
        )
        axis_error = (
            np.abs(centroid_uv - centre)
            if centroid_uv.shape == (2,) and np.all(np.isfinite(centroid_uv))
            else np.array([math.inf, math.inf])
        )
        maximum_center_error = float(
            self.get_parameter("touch_success_max_center_error_px").value
        )
        return {
            # Strong, repeated colour change remains the success evidence, but
            # it is evaluated only near the commanded target and fixed tip.
            # This prevents a sudden unrelated SAM region from turning a large
            # colour delta into a false Touch success.
            "eligible": bool(
                remaining <= maximum_remaining
                and float(np.max(axis_error)) <= maximum_center_error
            ),
            "geometry_reference_passed": bool(
                remaining <= maximum_remaining
                and float(np.max(axis_error)) <= maximum_center_error
            ),
            "x_remaining_m": remaining,
            "maximum_x_remaining_m": maximum_remaining,
            "centroid_axis_error_px": axis_error.tolist(),
            "maximum_center_error_px": maximum_center_error,
        }

    def _confirm_touch_change_observation(self, evidence: dict[str, Any]) -> bool:
        self._touch_success_change_streak += 1
        required = int(
            self.get_parameter("touch_success_required_observations").value
        )
        evidence["consecutive_change_observations"] = self._touch_success_change_streak
        evidence["required_change_observations"] = required
        if self._touch_success_change_streak >= required:
            return True
        with self._lock:
            self._automatic_reacquire_pending = False
            self._automatic_reacquire_due = (
                time.monotonic()
                + float(self.get_parameter("automatic_approach_settle_sec").value)
            )
        self._set_state(
            "TOUCH_CHANGE_VERIFYING",
            f"Large local colour change observed near the target; confirming it on "
            f"another stationary frame ({self._touch_success_change_streak}/{required})",
        )
        return False

    def _appearance_change_at_previous_target(self, rgb: np.ndarray) -> dict[str, Any]:
        """Detect disappearance/change even when SAM can no longer return a mask."""
        reference = self._arm_appearance_reference
        tracking = self._automatic_tracking_reference
        if reference is None or tracking is None:
            return {"changed": False, "reason": "NO_REFERENCE"}
        centroid = np.asarray(tracking.get("centroid_uv", []), dtype=np.float64)
        if centroid.shape != (2,) or not np.all(np.isfinite(centroid)):
            return {"changed": False, "reason": "NO_VALID_PREVIOUS_CENTROID"}
        previous_area = max(float(tracking.get("mask_pixels", 0)), 1.0)
        radius = int(np.clip(math.sqrt(previous_area / math.pi) * 0.35, 12.0, 40.0))
        height, width = rgb.shape[:2]
        yy, xx = np.ogrid[:height, :width]
        local_mask = (
            (xx - float(centroid[0])) ** 2 + (yy - float(centroid[1])) ** 2
            <= float(radius * radius)
        )
        result = self._appearance_change(rgb, local_mask)
        result.update({
            "evidence_kind": "PREVIOUS_TARGET_LOCAL_COLOR_CHANGE",
            "previous_centroid_uv": centroid.tolist(),
            "sample_radius_px": radius,
            "sam_mask_pixels": 0,
        })
        return result

    def _complete_touch_and_return_home(self, evidence: dict[str, Any]) -> None:
        self._automatic_approach = False
        self._automatic_reacquire_due = None
        self._automatic_reacquire_pending = False
        if self._arm_target is not None:
            self._arm_target["touch_success_evidence"] = jsonable(evidence)
        self._save_arm_artifact()
        request_id = f"touch-home-preview-{uuid.uuid4()}"
        self._home_preview_request_id = request_id
        self._arm_command_pub.publish(String(data=json.dumps({
            "mode": str(self._arm_state.get("mode", "REAL_DASHBOARD_TEST")),
            "request_id": request_id,
            "command": "select_preset",
            "preset_id": "home",
        }, ensure_ascii=False, allow_nan=False)))
        self._set_state(
            "TOUCH_SUCCESS_HOME_PREVIEW_REQUESTED",
            f"Touch success from arm-camera appearance change "
            f"(Lab delta={float(evidence.get('delta_lab', 0.0)):.1f}); "
            "requesting guarded home preset",
        )

    def _save_arm_artifact(self) -> None:
        if self._frozen is None or self._arm_frozen is None:
            return
        root = Path(str(self.get_parameter("artifact_root").value)).expanduser()
        directory = root / f"observation_{self._frozen['id']}"
        directory.mkdir(parents=True, exist_ok=True)
        image_name = f"arm_rgb_{self._arm_frozen['id']}.png"
        mask_name = f"arm_sam_mask_{self._arm_frozen['id']}.png"
        image_path = directory / image_name
        if not image_path.exists():
            cv2.imwrite(
                str(image_path), cv2.cvtColor(self._arm_frozen["rgb"], cv2.COLOR_RGB2BGR)
            )
        if self._arm_mask is not None:
            cv2.imwrite(str(directory / mask_name), self._arm_mask.astype(np.uint8) * 255)
        evidence = {
            "schema_version": 1,
            "arm_observation_id": self._arm_frozen["id"],
            "image_stamp": self._arm_frozen["stamp"],
            "frame_id": self._arm_frozen["frame_id"],
            "camera_info": {
                "k": self._arm_frozen["camera_matrix"].reshape(-1).tolist(),
                "distortion_model": self._arm_frozen["distortion_model"],
                "d": self._arm_frozen["distortion"],
            },
            "d435_projected_uv": self._arm_frozen["d435_projected_uv"],
            "d435_projection_in_image": self._arm_frozen["d435_projection_in_image"],
            "d435_projection_role": (
                "AUTO_PROMPT" if bool(
                    self.get_parameter("arm_projection_auto_prompt").value
                ) else "REFERENCE_ONLY_MANUAL_SELECTION_REQUIRED"
            ),
            "tf_base_link_from_arm_camera": {
                "translation_m": self._arm_frozen[
                    "base_from_camera_translation"
                ].tolist(),
                "rotation_matrix": self._arm_frozen[
                    "base_from_camera_rotation"
                ].tolist(),
                "distance_from_base_link_origin_m": float(np.linalg.norm(
                    self._arm_frozen["base_from_camera_translation"]
                )),
            },
            "prompts": [
                {"u": u, "v": v, "label": label}
                for (u, v), label in zip(self._arm_prompts, self._arm_labels)
            ],
            "target": jsonable(self._arm_target),
            "files": {
                "rgb": image_name,
                "mask": mask_name if self._arm_mask is not None else None,
            },
            "range_policy": "NO_MONOCULAR_METRIC_DEPTH",
        }
        temporary = directory / "arm_observation.json.tmp"
        temporary.write_text(
            json.dumps(evidence, ensure_ascii=False, indent=2, allow_nan=False),
            encoding="utf-8",
        )
        temporary.replace(directory / "arm_observation.json")
        per_frame = directory / f"arm_observation_{self._arm_frozen['id']}.json"
        per_frame.write_text(
            json.dumps(evidence, ensure_ascii=False, indent=2, allow_nan=False),
            encoding="utf-8",
        )

    def _transform_target(self, target: dict[str, Any]) -> dict[str, Any]:
        frozen = self._frozen
        assert frozen is not None
        source = str(frozen["frame_id"])
        snapshot = frozen["tf_snapshot"]
        destination = str(snapshot["target_frame"])
        translation = np.asarray(snapshot["translation"], dtype=np.float64)
        quaternion = np.asarray(snapshot["quaternion_xyzw"], dtype=np.float64)
        matrix = quaternion_matrix_xyzw(quaternion)
        point_source = np.asarray(target["point"], dtype=np.float64)
        normal_source = np.asarray(target["normal"], dtype=np.float64)
        point_target = matrix @ point_source + translation
        normal_target = matrix @ normal_source
        normal_target /= max(float(np.linalg.norm(normal_target)), 1.0e-12)
        return {
            "source_frame": source,
            "target_frame": destination,
            "point_target": point_target,
            "normal_target": normal_target,
            "tf_translation": translation.tolist(),
            "tf_quaternion_xyzw": quaternion.tolist(),
        }

    def _lookup_tf_snapshot(self, source: str, stamp_sec: float) -> dict[str, Any]:
        destination = str(self.get_parameter("target_frame").value)
        if not source:
            raise ValueError("frozen color image frame_id is empty")
        stamp = Time(nanoseconds=int(float(stamp_sec) * 1.0e9))
        lookup_mode = "EXACT_IMAGE_STAMP"
        try:
            transform = self._tf_buffer.lookup_transform(
                destination, source, stamp, timeout=Duration(seconds=0.25)
            )
        except TransformException as exc:
            # D435 images can lead the latest dynamic pan/tilt TF by a few
            # milliseconds. Freeze already proves the mechanism is stable, so
            # permit only a bounded latest-transform fallback and retain the
            # actual timestamp as evidence.
            try:
                transform = self._tf_buffer.lookup_transform(
                    destination, source, Time(), timeout=Duration(seconds=0.10)
                )
            except TransformException as latest_exc:
                raise ValueError(
                    f"TF {destination} <- {source} unavailable at image stamp "
                    f"and latest lookup: {latest_exc}"
                ) from latest_exc
            tf_stamp_sec = (
                float(transform.header.stamp.sec)
                + float(transform.header.stamp.nanosec) * 1.0e-9
            )
            tf_skew_sec = abs(float(stamp_sec) - tf_stamp_sec)
            maximum_tf_skew = float(
                self.get_parameter("maximum_tf_fallback_skew_sec").value
            )
            if tf_stamp_sec <= 0.0 or tf_skew_sec > maximum_tf_skew:
                raise ValueError(
                    f"TF {destination} <- {source} exact lookup failed; latest TF "
                    f"skew={tf_skew_sec:.3f}s exceeds reliable fallback limit="
                    f"{maximum_tf_skew:.3f}s: {exc}"
                ) from exc
            lookup_mode = "LATEST_STABLE_BOUNDED_FALLBACK"
            self.get_logger().warning(
                f"Using bounded latest TF for frozen D435 frame: "
                f"skew={tf_skew_sec * 1000.0:.1f}ms"
            )
        tf_stamp_sec = (
            float(transform.header.stamp.sec)
            + float(transform.header.stamp.nanosec) * 1.0e-9
        )
        tf_skew_sec = abs(float(stamp_sec) - tf_stamp_sec)
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return {
            "source_frame": source,
            "target_frame": destination,
            "lookup_stamp": stamp_sec,
            "transform_stamp": tf_stamp_sec,
            "stamp_skew_sec": tf_skew_sec,
            "lookup_mode": lookup_mode,
            "translation": [translation.x, translation.y, translation.z],
            "quaternion_xyzw": [rotation.x, rotation.y, rotation.z, rotation.w],
        }

    def _fixed_arm_tip_pixel(self, width: int, height: int) -> Tuple[float, float]:
        u_ratio = float(self.get_parameter("arm_fixed_tip_u_ratio").value)
        v_ratio = float(self.get_parameter("arm_fixed_tip_v_ratio").value)
        if width < 2 or height < 2:
            raise ValueError(f"invalid arm-camera image size {width}x{height}")
        if not (0.0 <= u_ratio <= 1.0 and 0.0 <= v_ratio <= 1.0):
            raise ValueError(
                "arm fixed-tip ratios must lie in [0,1], got "
                f"u={u_ratio:.3f}, v={v_ratio:.3f}"
            )
        return u_ratio * (width - 1), v_ratio * (height - 1)

    def _republish_overlay(self) -> None:
        with self._lock:
            if self._overlay_stage == "ARM" and self._arm_frozen is not None:
                rgb = self._arm_frozen["rgb"].copy()
                image_height, image_width = rgb.shape[:2]
                fixed_tip_uv_float = self._fixed_arm_tip_pixel(image_width, image_height)
                fixed_tip_uv = tuple(int(round(value)) for value in fixed_tip_uv_float)
                cv2.drawMarker(
                    rgb, fixed_tip_uv, (20, 240, 240),
                    markerType=cv2.MARKER_CROSS, markerSize=24, thickness=2,
                )
                cv2.putText(
                    rgb, "FIXED LOWER TIP", (max(0, fixed_tip_uv[0] - 75),
                                              max(18, fixed_tip_uv[1] - 14)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (20, 240, 240), 1,
                    cv2.LINE_AA,
                )
                if self._arm_mask is not None:
                    tint = np.zeros_like(rgb)
                    tint[..., 0] = 40
                    tint[..., 2] = 230
                    rgb[self._arm_mask] = (
                        0.55 * rgb[self._arm_mask] + 0.45 * tint[self._arm_mask]
                    ).astype(np.uint8)
                projected = self._arm_frozen.get("d435_projected_uv")
                if projected is not None:
                    projected_uv = (
                        int(round(projected[0])), int(round(projected[1]))
                    )
                    cv2.drawMarker(
                        rgb, projected_uv,
                        (255, 180, 20), markerType=cv2.MARKER_TILTED_CROSS,
                        markerSize=22, thickness=2,
                    )
                    cv2.putText(
                        rgb, "D435 HANDOFF (ONCE)",
                        (max(0, projected_uv[0] - 70), max(18, projected_uv[1] - 14)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, (255, 180, 20), 1,
                        cv2.LINE_AA,
                    )
                for (u, v), label in zip(self._arm_prompts, self._arm_labels):
                    color = (40, 220, 40) if label == 1 else (240, 40, 40)
                    cv2.drawMarker(
                        rgb, (u, v), color, markerType=cv2.MARKER_CROSS,
                        markerSize=18, thickness=2,
                    )
                if self._arm_target is not None:
                    centroid = self._arm_target.get("centroid_uv")
                    if centroid is not None:
                        centroid_uv = (
                            int(round(centroid[0])), int(round(centroid[1]))
                        )
                        cv2.circle(
                            rgb, centroid_uv,
                            7, (255, 255, 0), 2,
                        )
                        cv2.putText(
                            rgb, "SAM MASK CENTROID",
                            (max(0, centroid_uv[0] - 72), max(18, centroid_uv[1] - 14)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.40, (255, 255, 0), 1,
                            cv2.LINE_AA,
                        )
                        cv2.line(
                            rgb, fixed_tip_uv, centroid_uv,
                            (255, 255, 0), 2,
                        )
                message = self._bridge.cv2_to_imgmsg(rgb, encoding="rgb8")
                message.header.stamp = self._arm_frozen["stamp_msg"]
                message.header.frame_id = self._arm_frozen["frame_id"]
                self._overlay_pub.publish(message)
                return
            if self._frozen is None:
                return
            rgb = self._frozen["rgb"].copy()
            if self._mask is not None:
                tint = np.zeros_like(rgb)
                tint[..., 1] = 210
                rgb[self._mask] = (
                    0.55 * rgb[self._mask] + 0.45 * tint[self._mask]
                ).astype(np.uint8)
            for (u, v), label in zip(self._prompts, self._labels):
                color = (40, 220, 40) if label == 1 else (240, 40, 40)
                cv2.drawMarker(
                    rgb, (u, v), color, markerType=cv2.MARKER_CROSS,
                    markerSize=18, thickness=2,
                )
            message = self._bridge.cv2_to_imgmsg(rgb, encoding="rgb8")
            message.header.stamp = self._frozen["stamp_msg"]
            message.header.frame_id = self._frozen["frame_id"]
        self._overlay_pub.publish(message)

    def _publish_target(self) -> None:
        with self._lock:
            if self._target is None or self._frozen is None:
                return
            target = dict(self._target)
            reach_plan = None if self._reach_plan is None else dict(self._reach_plan)
            approach_plan = (
                None if self._approach_plan is None else dict(self._approach_plan)
            )
        frame = str(target["target_frame"])
        point = np.asarray(target["point_target"], dtype=float)
        normal = np.asarray(target["normal_target"], dtype=float)
        point_message = PointStamped()
        # Geometry is already transformed into base_link using the captured TF.
        # A zero visualization stamp tells RViz to use the latest map<-base_link
        # transform; the original image/TF timestamps remain in the artifact.
        point_message.header.frame_id = frame
        point_message.point.x, point_message.point.y, point_message.point.z = map(float, point)
        self._point_pub.publish(point_message)

        pose = PoseStamped()
        pose.header = point_message.header
        pose.pose.position = point_message.point
        # Orientation is deliberately identity in phase 1. The measured normal
        # is published as an arrow; tool-axis orientation requires a calibrated
        # touch_tip and a selected IK redundancy policy.
        pose.pose.orientation.w = 1.0
        self._pose_pub.publish(pose)

        markers = MarkerArray()
        sphere = Marker()
        sphere.header = point_message.header
        sphere.ns = "touch_target"
        sphere.id = 0
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position = point_message.point
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.050
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 0.1, 0.95, 0.25, 1.0
        arrow = Marker()
        arrow.header = point_message.header
        arrow.ns = "touch_target"
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.points = [point_message.point]
        endpoint = type(point_message.point)()
        endpoint.x, endpoint.y, endpoint.z = map(float, point + normal * 0.10)
        arrow.points.append(endpoint)
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.008, 0.016, 0.024
        arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 0.55, 0.05, 1.0
        label = Marker()
        label.header = point_message.header
        label.ns = "touch_target_labels"
        label.id = 10
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position = point_message.point
        label.pose.position.z += 0.065
        label.pose.orientation.w = 1.0
        label.scale.z = 0.045
        label.color.r, label.color.g, label.color.b, label.color.a = 0.1, 1.0, 0.25, 1.0
        label.text = "DEPTH TARGET"
        markers.markers = [sphere, arrow, label]
        try:
            actual_tip_tf = self._tf_buffer.lookup_transform(
                frame, "arm/touch_tip", Time(), timeout=Duration(seconds=0.05)
            )
            actual_tip = Marker()
            actual_tip.header = point_message.header
            actual_tip.ns = "touch_actual_tip"
            actual_tip.id = 20
            actual_tip.type = Marker.SPHERE
            actual_tip.action = Marker.ADD
            actual_tip.pose.position.x = actual_tip_tf.transform.translation.x
            actual_tip.pose.position.y = actual_tip_tf.transform.translation.y
            actual_tip.pose.position.z = actual_tip_tf.transform.translation.z
            actual_tip.pose.orientation.w = 1.0
            # Keep the live tip marker small enough that its centre remains
            # visually distinguishable from the finger body in close-up RViz
            # views.  The previous 25 mm sphere covered much of the jaw.
            actual_tip.scale.x = actual_tip.scale.y = actual_tip.scale.z = 0.012
            actual_tip.color.r, actual_tip.color.g = 0.85, 0.15
            actual_tip.color.b, actual_tip.color.a = 1.0, 1.0
            markers.markers.append(actual_tip)
            actual_label = Marker()
            actual_label.header = point_message.header
            actual_label.ns = "touch_target_labels"
            actual_label.id = 20
            actual_label.type = Marker.TEXT_VIEW_FACING
            actual_label.action = Marker.ADD
            actual_label.pose.position = actual_tip.pose.position
            actual_label.pose.position.z += 0.030
            actual_label.pose.orientation.w = 1.0
            actual_label.scale.z = 0.030
            actual_label.color = actual_tip.color
            actual_label.text = "ACTUAL TOUCH TIP"
            markers.markers.append(actual_label)
        except TransformException:
            pass
        # Show the arm-camera optical centre and axes from the live TF.  The
        # camera is on the rigid gripper body, so arm/base_link distance changes
        # with the arm pose and must not be documented as a static dimension.
        try:
            camera_tf = self._tf_buffer.lookup_transform(
                frame, "arm_camera_color_optical_frame", Time(),
                timeout=Duration(seconds=0.05),
            )
            arm_base_tf = self._tf_buffer.lookup_transform(
                frame, "arm/base_link", Time(), timeout=Duration(seconds=0.05)
            )
            camera_position = np.array([
                camera_tf.transform.translation.x,
                camera_tf.transform.translation.y,
                camera_tf.transform.translation.z,
            ], dtype=float)
            arm_base_position = np.array([
                arm_base_tf.transform.translation.x,
                arm_base_tf.transform.translation.y,
                arm_base_tf.transform.translation.z,
            ], dtype=float)
            camera_rotation, _ = self._rotation_translation(camera_tf)
            camera_marker = Marker()
            camera_marker.header = point_message.header
            camera_marker.ns = "arm_camera_tf"
            camera_marker.id = 40
            camera_marker.type = Marker.SPHERE
            camera_marker.action = Marker.ADD
            camera_marker.pose.position.x, camera_marker.pose.position.y, camera_marker.pose.position.z = map(float, camera_position)
            camera_marker.pose.orientation.w = 1.0
            camera_marker.scale.x = camera_marker.scale.y = camera_marker.scale.z = 0.018
            camera_marker.color.r, camera_marker.color.g = 0.95, 0.2
            camera_marker.color.b, camera_marker.color.a = 0.95, 1.0
            markers.markers.append(camera_marker)
            axis_colors = ((1.0, 0.1, 0.1), (0.1, 1.0, 0.1), (0.1, 0.45, 1.0))
            for axis_index, color in enumerate(axis_colors):
                axis = Marker()
                axis.header = point_message.header
                axis.ns = "arm_camera_tf"
                axis.id = 41 + axis_index
                axis.type = Marker.ARROW
                axis.action = Marker.ADD
                origin = Point(x=float(camera_position[0]), y=float(camera_position[1]), z=float(camera_position[2]))
                endpoint_value = camera_position + camera_rotation[:, axis_index] * 0.065
                endpoint = Point(x=float(endpoint_value[0]), y=float(endpoint_value[1]), z=float(endpoint_value[2]))
                axis.points = [origin, endpoint]
                axis.scale.x, axis.scale.y, axis.scale.z = 0.004, 0.008, 0.012
                axis.color.r, axis.color.g, axis.color.b = color
                axis.color.a = 1.0
                markers.markers.append(axis)
            camera_label = Marker()
            camera_label.header = point_message.header
            camera_label.ns = "arm_camera_tf"
            camera_label.id = 44
            camera_label.type = Marker.TEXT_VIEW_FACING
            camera_label.action = Marker.ADD
            camera_label.pose.position = camera_marker.pose.position
            camera_label.pose.position.z += 0.055
            camera_label.pose.orientation.w = 1.0
            camera_label.scale.z = 0.026
            camera_label.color = camera_marker.color
            camera_label.text = (
                "ARM CAMERA  base distance "
                f"{np.linalg.norm(camera_position - arm_base_position) * 1000.0:.1f} mm"
            )
            markers.markers.append(camera_label)
        except TransformException:
            pass
        # The calibration dimension needed for prepare-pose recording is the
        # live D435 colour optical centre relative to the SO101 base, not the
        # wrist camera relative to the SO101 base.  Render it separately so the
        # two camera relationships cannot be confused in RViz.
        try:
            d435_tf = self._tf_buffer.lookup_transform(
                frame, "camera_color_optical_frame", Time(),
                timeout=Duration(seconds=0.05),
            )
            arm_base_tf = self._tf_buffer.lookup_transform(
                frame, "arm/base_link", Time(), timeout=Duration(seconds=0.05)
            )
            d435_position = np.array([
                d435_tf.transform.translation.x,
                d435_tf.transform.translation.y,
                d435_tf.transform.translation.z,
            ], dtype=float)
            arm_base_position = np.array([
                arm_base_tf.transform.translation.x,
                arm_base_tf.transform.translation.y,
                arm_base_tf.transform.translation.z,
            ], dtype=float)
            separation = float(np.linalg.norm(arm_base_position - d435_position))
            relation = Marker()
            relation.header = point_message.header
            relation.ns = "d435_to_arm_base_tf"
            relation.id = 50
            relation.type = Marker.LINE_STRIP
            relation.action = Marker.ADD
            relation.points = [
                Point(x=float(d435_position[0]), y=float(d435_position[1]), z=float(d435_position[2])),
                Point(x=float(arm_base_position[0]), y=float(arm_base_position[1]), z=float(arm_base_position[2])),
            ]
            relation.scale.x = 0.004
            relation.color.r, relation.color.g = 1.0, 0.85
            relation.color.b, relation.color.a = 0.05, 0.9
            markers.markers.append(relation)
            relation_label = Marker()
            relation_label.header = point_message.header
            relation_label.ns = "d435_to_arm_base_tf"
            relation_label.id = 51
            relation_label.type = Marker.TEXT_VIEW_FACING
            relation_label.action = Marker.ADD
            midpoint = (d435_position + arm_base_position) * 0.5
            relation_label.pose.position.x = float(midpoint[0])
            relation_label.pose.position.y = float(midpoint[1])
            relation_label.pose.position.z = float(midpoint[2] + 0.035)
            relation_label.pose.orientation.w = 1.0
            relation_label.scale.z = 0.025
            relation_label.color = relation.color
            relation_label.text = f"D435 COLOR OPTICAL <-> ARM BASE  {separation * 1000.0:.1f} mm"
            markers.markers.append(relation_label)
        except TransformException:
            pass
        if reach_plan is not None:
            path = Marker()
            path.header = point_message.header
            path.ns = "touch_target"
            path.id = 4
            path.type = Marker.LINE_STRIP
            path.action = Marker.ADD
            path.scale.x = 0.010
            path.color.r, path.color.g, path.color.b, path.color.a = 0.9, 0.9, 0.9, 0.9
            for key in ("pre_touch_far_base_m",):
                value = reach_plan.get(key)
                if value is not None:
                    path_point = type(point_message.point)()
                    path_point.x, path_point.y, path_point.z = map(float, value)
                    path.points.append(path_point)
            path.points.append(point_message.point)
            markers.markers.append(path)
            for marker_id, key, text, rgb, scale in (
                (2, "pre_touch_far_base_m", "FAR 80 mm", (0.1, 0.45, 1.0), 0.060),
            ):
                value = reach_plan.get(key)
                if value is None:
                    continue
                marker = Marker()
                marker.header = point_message.header
                marker.ns = "touch_target"
                marker.id = marker_id
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, value)
                marker.pose.orientation.w = 1.0
                marker.scale.x = marker.scale.y = marker.scale.z = scale
                marker.color.r, marker.color.g, marker.color.b = rgb
                marker.color.a = 1.0
                markers.markers.append(marker)
                point_label = Marker()
                point_label.header = point_message.header
                point_label.ns = "touch_target_labels"
                point_label.id = marker_id + 10
                point_label.type = Marker.TEXT_VIEW_FACING
                point_label.action = Marker.ADD
                point_label.pose.position = marker.pose.position
                point_label.pose.position.z += 0.065
                point_label.pose.orientation.w = 1.0
                point_label.scale.z = 0.040
                point_label.color.r, point_label.color.g, point_label.color.b = rgb
                point_label.color.a = 1.0
                point_label.text = text
                markers.markers.append(point_label)
        if approach_plan is not None and approach_plan.get("waypoint_base_m") is not None:
            value = approach_plan["waypoint_base_m"]
            step_marker = Marker()
            step_marker.header = point_message.header
            step_marker.ns = "touch_approach"
            step_marker.id = 30
            step_marker.type = Marker.SPHERE
            step_marker.action = Marker.ADD
            step_marker.pose.position.x, step_marker.pose.position.y, step_marker.pose.position.z = map(float, value)
            step_marker.pose.orientation.w = 1.0
            step_marker.scale.x = step_marker.scale.y = step_marker.scale.z = 0.022
            step_marker.color.r, step_marker.color.g = 0.0, 0.9
            step_marker.color.b, step_marker.color.a = 1.0, 1.0
            markers.markers.append(step_marker)
            step_label = Marker()
            step_label.header = point_message.header
            step_label.ns = "touch_target_labels"
            step_label.id = 30
            step_label.type = Marker.TEXT_VIEW_FACING
            step_label.action = Marker.ADD
            step_label.pose.position = step_marker.pose.position
            step_label.pose.position.z += 0.035
            step_label.pose.orientation.w = 1.0
            step_label.scale.z = 0.025
            step_label.color = step_marker.color
            step_label.text = "NEXT VISUAL STEP"
            markers.markers.append(step_label)
        self._marker_pub.publish(markers)

    def _refresh_visualization(self) -> None:
        if self._target is not None:
            self._publish_target()

    def _save_artifact(self) -> None:
        """Persist direct sensor evidence and the latest derived result."""
        frozen = self._frozen
        if frozen is None:
            return
        root = Path(str(self.get_parameter("artifact_root").value)).expanduser()
        directory = root / f"observation_{frozen['id']}"
        directory.mkdir(parents=True, exist_ok=True)
        color_path = directory / "color_rgb.png"
        depth_raw_path = directory / "aligned_depth_raw.npy"
        depth_m_path = directory / "aligned_depth_m.npy"
        if not color_path.exists():
            cv2.imwrite(
                str(color_path), cv2.cvtColor(frozen["rgb"], cv2.COLOR_RGB2BGR)
            )
            np.save(depth_raw_path, frozen["raw_depth"])
            np.save(depth_m_path, frozen["depth_m"])
        if self._mask is not None:
            cv2.imwrite(
                str(directory / "sam_mask.png"), self._mask.astype(np.uint8) * 255
            )
        artifact = {
            "schema_version": 1,
            "observation_id": frozen["id"],
            "selection_source": "HUMAN_POINT_RVIZ",
            "image_stamp": frozen["stamp"],
            "image_frame_id": frozen["frame_id"],
            "files": {
                "color_rgb": color_path.name,
                "aligned_depth_raw": depth_raw_path.name,
                "aligned_depth_m": depth_m_path.name,
                "mask": "sam_mask.png" if self._mask is not None else None,
            },
            "camera_info": {
                "k": frozen["camera_matrix"].reshape(-1).tolist(),
                "distortion_model": frozen["distortion_model"],
                "d": frozen["distortion"],
            },
            "depth": {
                "encoding": frozen["depth_encoding"],
                "scale_m_per_unit": float(
                    self.get_parameter("depth_scale_m_per_unit").value
                ),
                "scale_source": "ROS_PARAMETER_CONFIGURED_FOR_REALSENSE_Z16",
            },
            "synchronization": {
                "depth_skew_sec": frozen["depth_skew_sec"],
                "camera_info_skew_sec": frozen["camera_info_skew_sec"],
            },
            "tf": frozen["tf_snapshot"],
            "mechanism_evidence": frozen["mechanism_evidence"],
            "prompts": [
                {"u": u, "v": v, "label": label}
                for (u, v), label in zip(self._prompts, self._labels)
            ],
            "segmentation": {
                "model": "SAM 2.1 Hiera Tiny",
                "config": self._sam.config,
                "checkpoint": str(self._sam.checkpoint),
                "checkpoint_sha256": (
                    "7402e0d864fa82708a20fbd15bc84245c2f26dff0eb43a4b5b93452deb34be69"
                ),
                "operator_confirmed": bool(frozen.get("operator_confirmed")),
                "confirmation_provenance": frozen.get("confirmation_provenance"),
            },
            "geometry": None if self._target is None else {
                key: value.tolist() if isinstance(value, np.ndarray) else value
                for key, value in self._target.items()
            },
            "execution": "PREVIEW_ONLY_NO_ARM_COMMAND",
        }
        temporary = directory / "observation.json.tmp"
        temporary.write_text(
            json.dumps(artifact, ensure_ascii=False, indent=2, allow_nan=False),
            encoding="utf-8",
        )
        temporary.replace(directory / "observation.json")

    def _deactivate(self, message: str) -> None:
        with self._lock:
            self._active = False
            self._frozen = None
            self._prompts.clear()
            self._labels.clear()
            self._mask = None
            self._target = None
            self._reach_plan = None
            self._reach_preview_request_id = None
            self._reach_execute_request_id = None
            self._clear_arm_observation()
            self._approach_execute_request_id = None
            self._reset_automatic_approach()
            self._camera_adjustment_requested_at = None
            self._job_id += 1
        with self._sam_lock:
            unloaded = self._sam.unload()
        if unloaded:
            self.get_logger().info("Released task-scoped SAM 2.1 model")
        self._set_state("INACTIVE", message)

    def _plan_pre_touch_far(self) -> None:
        if not self._active or self._frozen is None or self._target is None:
            self._set_state("BLOCKED", "A confirmed frozen 3-D target is required")
            return
        if not bool(self._frozen.get("operator_confirmed")):
            self._set_state("BLOCKED", "Confirm the 3-D target before planning Arm reach")
            return
        allowed, reason = self._arm_gate()
        if not allowed:
            self._set_state("BLOCKED", reason)
            return
        if not all(name in self._arm_joints for name in REACH_JOINT_NAMES):
            self._set_state("BLOCKED", "Complete fresh five-joint Arm feedback is required for IK")
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                "arm/base_link", "base_link", Time(), timeout=Duration(seconds=0.25)
            )
        except TransformException as exc:
            self._set_state("BLOCKED", f"TF arm/base_link <- base_link unavailable: {exc}")
            return
        rotation_message = transform.transform.rotation
        rotation = quaternion_matrix_xyzw(np.array([
            rotation_message.x, rotation_message.y, rotation_message.z, rotation_message.w,
        ]))
        translation_message = transform.transform.translation
        translation = np.array([
            translation_message.x, translation_message.y, translation_message.z,
        ])
        point_base = np.asarray(self._target["point_target"], dtype=float)
        measured_normal_base = np.asarray(self._target["normal_target"], dtype=float)
        point_arm = rotation @ point_base + translation
        # Touch tasks approach horizontally from the robot front. Do not use
        # the camera-view-dependent measured surface normal for waypoint
        # placement. This keeps y and z exactly equal to the depth target.
        approach_axis_base = np.array([1.0, 0.0, 0.0], dtype=float)
        approach_axis_arm = rotation @ approach_axis_base
        approach_axis_arm /= max(float(np.linalg.norm(approach_axis_arm)), 1.0e-12)
        far_distance = float(self.get_parameter("pre_touch_far_distance_m").value)
        near_distance = float(self.get_parameter("pre_touch_near_distance_m").value)
        far_base = point_base - approach_axis_base * far_distance
        near_base = point_base - approach_axis_base * near_distance
        far_arm = rotation @ far_base + translation
        near_arm = rotation @ near_base + translation
        current = [self._arm_joints[name] for name in REACH_JOINT_NAMES]
        effective_specs = {
            str(item.get("name")): item
            for item in self._arm_state.get("joints", [])
            if isinstance(item, dict)
        }
        if not all(name in effective_specs for name in REACH_JOINT_NAMES):
            self._set_state(
                "BLOCKED", "Arm gateway effective joint limits are unavailable"
            )
            return
        limit_margin = float(self.get_parameter("ik_joint_limit_margin_rad").value)
        lower = [float(effective_specs[name]["lower"]) + limit_margin for name in REACH_JOINT_NAMES]
        upper = [float(effective_specs[name]["upper"]) - limit_margin for name in REACH_JOINT_NAMES]
        if any(lo >= hi for lo, hi in zip(lower, upper)):
            self._set_state("BLOCKED", "Configured IK joint-limit margin leaves an invalid range")
            return
        front_half_width = float(
            self.get_parameter("prepare_front_half_width_m").value
        )
        if not math.isfinite(front_half_width) or front_half_width <= 0.0:
            self._set_state("BLOCKED", "prepare_front_half_width_m must be positive")
            return
        if far_base[1] > front_half_width:
            prepare_sector = "left"
        elif far_base[1] < -front_half_width:
            prepare_sector = "right"
        else:
            prepare_sector = "front"
        solution = solve_pre_touch(
            current, far_arm, approach_axis_arm,
            lower_limits=lower, upper_limits=upper,
            reference_sector=prepare_sector, reference_stage="far",
        )
        solution["target_point_arm_base_m"] = point_arm.tolist()
        solution["pre_touch_far_arm_base_m"] = far_arm.tolist()
        solution["pre_touch_near_arm_base_m"] = near_arm.tolist()
        solution["pre_touch_far_base_m"] = far_base.tolist()
        solution["pre_touch_near_base_m"] = near_base.tolist()
        solution["far_distance_m"] = far_distance
        solution["near_distance_m"] = near_distance
        solution["waypoint_policy"] = "BASE_FORWARD_HORIZONTAL_SAME_Y_Z"
        solution["prepare_sector_source"] = (
            "PRE_TOUCH_FAR_BASE_Y_WITH_FRONT_HALF_WIDTH"
        )
        solution["prepare_front_half_width_m"] = front_half_width
        solution["approach_axis_base"] = approach_axis_base.tolist()
        solution["measured_surface_normal_base"] = measured_normal_base.tolist()
        solution["execution_enabled"] = bool(
            self.get_parameter("allow_pre_touch_far_execution").value
        )
        gripper = float(self.get_parameter("touch_gripper_position_rad").value)
        positions = {**solution["positions"], "gripper_joint": gripper}
        if not solution["success"]:
            self._reach_plan = {**solution, "positions": positions, "candidate_id": None}
            self._publish_target()
            diagnostic = {
                "mode": str(self._arm_state.get("mode", "REAL_DASHBOARD_TEST")),
                "request_id": f"touch-diagnostic-{uuid.uuid4()}",
                "command": "diagnostic_preview",
                "positions": positions,
            }
            self._arm_command_pub.publish(
                String(data=json.dumps(diagnostic, ensure_ascii=False, allow_nan=False))
            )
            self._set_state(
                "PRETOUCH_UNREACHABLE",
                f"IK rejected: position error={solution['position_error_m']:.3f}m "
                f"(limit 0.008m); best bounded pose is shown in orange but cannot execute; "
                f"horizontal preference error={solution['axis_error_deg']:.1f}deg; "
                "REQUIRES_ARM_BASE_REPOSITION",
            )
            return
        request_id = f"touch-preview-{uuid.uuid4()}"
        self._reach_plan = {**solution, "positions": positions, "candidate_id": None}
        self._publish_target()
        self._reach_preview_request_id = request_id
        command = {
            "mode": str(self._arm_state.get("mode", "REAL_DASHBOARD_TEST")),
            "request_id": request_id,
            "command": "preview",
            "positions": positions,
        }
        self._arm_command_pub.publish(
            String(data=json.dumps(command, ensure_ascii=False, allow_nan=False))
        )
        self._set_state(
            "PRETOUCH_PREVIEW_REQUESTED",
            f"IK solution sent to Arm gateway for orange preview; "
            f"position error={solution['position_error_m'] * 1000.0:.1f}mm, "
            f"horizontal preference error={solution['axis_error_deg']:.1f}deg; "
            "no motion command sent",
        )

    def _execute_pre_touch_far(self) -> None:
        if not bool(self.get_parameter("allow_pre_touch_far_execution").value):
            self._set_state(
                "BLOCKED",
                "Real pre-touch execution is locked: the current IK has no validated "
                "whole-robot collision model",
            )
            return
        if self._state != "D435_TARGET_REACQUIRED":
            self._set_state(
                "BLOCKED",
                "FAR execution requires a stable reference pan/tilt pose and a successful "
                "D435 same-target reacquisition",
            )
            return
        if self._reach_plan is None or not self._reach_plan.get("candidate_id"):
            self._set_state("BLOCKED", "Plan and validate a current pre-touch candidate first")
            return
        request_id = f"touch-execute-{uuid.uuid4()}"
        command = {
            "mode": str(self._arm_state.get("mode", "REAL_DASHBOARD_TEST")),
            "request_id": request_id,
            "command": "execute_candidate",
            "candidate_id": self._reach_plan["candidate_id"],
            "duration_sec": 4.0,
        }
        self._arm_command_pub.publish(
            String(data=json.dumps(command, ensure_ascii=False, allow_nan=False))
        )
        self._reach_execute_request_id = request_id
        self._set_state(
            "PRETOUCH_EXECUTION_REQUESTED",
            "Guarded FAR-only request sent; this command does not start PRESS",
        )

    def _plan_approach_step(self) -> None:
        if not self._active or self._target is None:
            self._pause_automatic_approach(
                "APPROACH_BLOCKED", "A retained D435 target is required"
            )
            return
        if not self._arm_target_confirmed or self._arm_target is None or self._arm_frozen is None:
            self._pause_automatic_approach(
                "APPROACH_BLOCKED", "Confirm the target in a fresh arm-camera image first"
            )
            return
        if self._state not in {"ARM_TARGET_CONFIRMED", "APPROACH_GATEWAY_REJECTED"}:
            self._set_state("BLOCKED", "Arm-camera target is not ready for approach planning")
            return
        allowed, reason = self._arm_gate()
        if not allowed:
            self._pause_automatic_approach("APPROACH_BLOCKED", reason)
            return
        if not all(name in self._arm_joints for name in REACH_JOINT_NAMES):
            self._pause_automatic_approach(
                "APPROACH_BLOCKED", "Fresh five-joint Arm feedback is required"
            )
            return
        try:
            actual_tip_transform = self._latest_transform("base_link", "arm/touch_tip")
            _, actual_tip = self._rotation_translation(actual_tip_transform)
            height, width = self._arm_frozen["rgb"].shape[:2]
            fixed_tip_uv = np.asarray(
                self._fixed_arm_tip_pixel(width, height), dtype=np.float64
            )
            d435_anchor = np.asarray(self._target["point_target"], dtype=np.float64)
            camera_origin = np.asarray(
                self._arm_frozen["base_from_camera_translation"], dtype=np.float64
            )
            range_scale = float(np.linalg.norm(d435_anchor - camera_origin))
            mask_centroid_uv = np.asarray(
                self._arm_target["centroid_uv"], dtype=np.float64
            )
            mask_fraction = float(self._arm_target.get("mask_fraction", 0.0))
            too_close_mask_fraction = float(
                self.get_parameter("approach_too_close_mask_fraction").value
            )
            near_advance_m = float(
                self.get_parameter("approach_near_advance_distance_m").value
            )
            if self._approach_far_tip_x_m is None:
                self._approach_far_tip_x_m = float(actual_tip[0])
            advanced_from_far_m = max(
                0.0, float(actual_tip[0]) - self._approach_far_tip_x_m
            )
            too_close = bool(mask_fraction >= too_close_mask_fraction)
            if advanced_from_far_m >= near_advance_m - 1.0e-3:
                self._near_phase_latched = True
            # NEAR is a one-way phase transition for this Touch session.
            # Pure Y/Z IK can couple into measured X on the real mechanism;
            # that drift must not send the state machine back to FAR.
            near_reached = self._near_phase_latched
            image_centre_uv = np.array(
                [(width - 1.0) * 0.5, (height - 1.0) * 0.5], dtype=np.float64
            )
            # FAR centres the complete object in the image.  Once measured +X
            # travel crosses the NEAR boundary, use one unambiguous alignment
            # objective: put the fixed lower gripper tip inside the SAM mask.
            alignment_pixel_uv = fixed_tip_uv if near_reached else image_centre_uv
            tip_alignment = base_yz_pixel_alignment_delta(
                tuple(mask_centroid_uv), tuple(alignment_pixel_uv),
                np.asarray(self._arm_frozen["camera_matrix"], dtype=np.float64),
                np.asarray(self._arm_frozen["distortion"], dtype=np.float64),
                np.asarray(
                    self._arm_frozen["base_from_camera_rotation"], dtype=np.float64
                ),
                range_scale,
            )
            use_tip_alignment = bool(near_reached)
            lateral_correction = np.asarray(
                tip_alignment["delta_base_yz_m"], dtype=np.float64
            )
            # D435 X is retained only as an independent maximum-travel reference.
            # Y/Z follow the real-robot fixed-tip wrist-image mapping.
            refined_point = d435_anchor.copy()
            refined_point[1:3] = actual_tip[1:3] + lateral_correction
            maximum_correction = float(
                self.get_parameter("maximum_arm_lateral_correction_m").value
            )
            if float(np.linalg.norm(lateral_correction)) > maximum_correction:
                raise ValueError(
                    f"arm-camera visual correction {np.linalg.norm(lateral_correction):.3f}m "
                    f"exceeds {maximum_correction:.3f}m; verify the selected object"
                )
            lateral_error = refined_point[1:3] - actual_tip[1:3]
            lateral_norm = float(np.linalg.norm(lateral_error))
            pixel_error_norm = float(tip_alignment["pixel_error_norm"])
            if (
                self._automatic_approach
                and self._automatic_approach_steps > 0
                and not too_close
            ):
                minimum_progress = float(
                    self.get_parameter("approach_min_pixel_progress_px").value
                )
                if self._automatic_last_pixel_error is not None:
                    progress = self._automatic_last_pixel_error - pixel_error_norm
                    if progress < minimum_progress:
                        self._automatic_nonconverging_steps += 1
                    else:
                        self._automatic_nonconverging_steps = 0
                    limit = int(
                        self.get_parameter("approach_nonconverging_step_limit").value
                    )
                    if self._automatic_nonconverging_steps >= limit:
                        self.get_logger().warn(
                            "Arm-camera alignment made little pixel progress, but the "
                            "bounded stop-observe loop will replan from measured TF: "
                            f"previous={self._automatic_last_pixel_error:.1f}px, "
                            f"current={pixel_error_norm:.1f}px"
                        )
                        self._automatic_nonconverging_steps = 0
                self._automatic_last_pixel_error = pixel_error_norm
            elif self._automatic_approach:
                self._automatic_last_pixel_error = pixel_error_norm
            pixel_tolerance = float(
                self.get_parameter("approach_pixel_alignment_tolerance_px").value
            )
            if not math.isfinite(pixel_tolerance) or pixel_tolerance <= 0.0:
                raise ValueError(
                    "approach_pixel_alignment_tolerance_px must be positive"
                )
            pixel_aligned = pixel_error_norm <= pixel_tolerance
            press_entry_tolerance = float(
                self.get_parameter("approach_press_entry_tolerance_px").value
            )
            if not math.isfinite(press_entry_tolerance) or press_entry_tolerance <= 0.0:
                raise ValueError("approach_press_entry_tolerance_px must be positive")
            fixed_tip_in_mask = pixel_inside_mask(
                self._arm_mask, tuple(fixed_tip_uv)
            )
            press_alignment_ready = bool(
                near_reached
                and fixed_tip_in_mask
                and pixel_error_norm <= press_entry_tolerance
            )
            maximum_lateral_step = float(
                self.get_parameter("approach_max_lateral_step_m").value
            )
            if not math.isfinite(maximum_lateral_step) or maximum_lateral_step <= 0.0:
                raise ValueError("approach_max_lateral_step_m must be positive")
            if pixel_aligned:
                lateral_step = np.zeros(2, dtype=np.float64)
            elif near_reached:
                # In NEAR, apply the complete camera-derived Y/Z correction.
                # The independent maximum_arm_lateral_correction_m sanity gate
                # above still rejects non-finite or implausibly large results.
                lateral_step = lateral_error.copy()
            elif lateral_norm > 1.0e-9:
                lateral_step = lateral_error * min(
                    1.0, maximum_lateral_step / lateral_norm
                )
            else:
                lateral_step = np.zeros(2, dtype=np.float64)

            aligned_press_step = float(
                self.get_parameter("approach_aligned_press_step_distance_m").value
            )
            max_press_past_target = float(
                self.get_parameter("approach_max_press_past_target_m").value
            )
            if not math.isfinite(aligned_press_step) or aligned_press_step <= 0.0:
                raise ValueError(
                    "approach_aligned_press_step_distance_m must be positive"
                )
            if not math.isfinite(max_press_past_target) or max_press_past_target <= 0.0:
                raise ValueError("approach_max_press_past_target_m must be positive")
            remaining = float(refined_point[0] - actual_tip[0])
            if remaining <= -max_press_past_target + 1.0e-3:
                approach = {
                    "waypoint_base_m": actual_tip.copy(),
                    "remaining_before_m": remaining,
                    "remaining_after_m": remaining,
                    "complete": True,
                    "motion_axis_base": [0.0, 0.0, 0.0],
                    "step_kind": "MAX_PRESS_TRAVEL_REACHED",
                    "fixed_tip_in_mask": bool(fixed_tip_in_mask),
                    "lateral_error_before_m": lateral_error.tolist(),
                    "lateral_step_m": [0.0, 0.0],
                }
            elif too_close:
                self._press_phase_active = False
                retreat_available = max(
                    0.0, float(actual_tip[0]) - self._approach_far_tip_x_m
                )
                retreat_step = min(
                    float(self.get_parameter("approach_too_close_retreat_m").value),
                    retreat_available,
                )
                if retreat_step <= 1.0e-4:
                    raise ValueError(
                        "arm mask exceeds the 15% too-close limit at the FAR plane; "
                        "cannot retreat farther"
                    )
                waypoint_base = actual_tip.copy()
                waypoint_base[0] -= retreat_step
                self._automatic_last_pixel_error = None
                self._automatic_nonconverging_steps = 0
                approach = {
                    "waypoint_base_m": waypoint_base,
                    "remaining_before_m": remaining,
                    "remaining_after_m": remaining + retreat_step,
                    "complete": False,
                    "motion_axis_base": [-float(retreat_step), 0.0, 0.0],
                    "step_kind": "MASK_TOO_CLOSE_RETREAT_SMALL",
                    "alignment_reference": "ARM_CAMERA_IMAGE_CENTER",
                    "fixed_tip_in_mask": bool(fixed_tip_in_mask),
                    "lateral_error_before_m": lateral_error.tolist(),
                    "lateral_step_m": [0.0, 0.0],
                }
            elif not near_reached:
                # FAR-to-near stage: keep the complete mask near the wrist-image
                # centre while advancing, capped at 40 mm measured travel.
                self._press_phase_active = False
                large_lateral_limit = float(self.get_parameter(
                    "approach_large_max_lateral_step_m"
                ).value)
                if not math.isfinite(large_lateral_limit) or large_lateral_limit <= 0.0:
                    raise ValueError(
                        "approach_large_max_lateral_step_m must be positive"
                    )
                coarse_lateral_step = lateral_error * min(
                    1.0, large_lateral_limit / max(lateral_norm, 1.0e-9)
                )
                # Execute a complete FAR step.  NEAR is evaluated from measured
                # TF only after this trajectory finishes, so crossing 35 mm by
                # a few millimetres is intentional.
                forward_step = aligned_press_step
                waypoint_base = actual_tip.copy()
                waypoint_base[0] += forward_step
                waypoint_base[1:3] += coarse_lateral_step
                approach = {
                    "waypoint_base_m": waypoint_base,
                    "remaining_before_m": remaining,
                    "remaining_after_m": remaining - forward_step,
                    "complete": False,
                    "motion_axis_base": [
                        float(forward_step), float(coarse_lateral_step[0]),
                        float(coarse_lateral_step[1]),
                    ],
                    "step_kind": "COARSE_CENTER_AND_FORWARD_TO_NEAR",
                    "press_phase_latched": False,
                    "alignment_reference": "ARM_CAMERA_IMAGE_CENTER",
                    "fixed_tip_in_mask": bool(fixed_tip_in_mask),
                    "arm_mask_fraction": mask_fraction,
                    "lateral_error_before_m": lateral_error.tolist(),
                    "lateral_step_m": coarse_lateral_step.tolist(),
                }
            elif not press_alignment_ready:
                # NEAR has a single closed-loop job. Keep X fixed and repeat
                # Y/Z corrections until the fixed lower tip is aligned with and
                # contained by the SAM target. Only then permit +X PRESS.
                self._press_phase_active = False
                if float(np.linalg.norm(lateral_step)) <= 1.0e-9:
                    raise ValueError(
                        "press alignment is not satisfied but no finite Y/Z "
                        "correction is available"
                    )
                waypoint_base = actual_tip.copy()
                waypoint_base[1:3] += lateral_step
                approach = {
                    "waypoint_base_m": waypoint_base,
                    "remaining_before_m": remaining,
                    "remaining_after_m": remaining,
                    "complete": False,
                    "motion_axis_base": [
                        0.0, float(lateral_step[0]), float(lateral_step[1]),
                    ],
                    "step_kind": "ALIGN_FIXED_TIP_YZ",
                    "press_phase_latched": False,
                    "alignment_reference": "FIXED_LOWER_TIP",
                    "fixed_tip_in_mask": bool(fixed_tip_in_mask),
                    "arm_mask_fraction": mask_fraction,
                    "lateral_error_before_m": lateral_error.tolist(),
                    "lateral_step_m": lateral_step.tolist(),
                }
            else:
                # Revalidate strict alignment and mask containment before every
                # deliberate 25 mm +X press step; there is no unaligned fallback.
                self._press_phase_active = True
                forward_step = min(
                    aligned_press_step, remaining + max_press_past_target
                )
                waypoint_base = actual_tip.copy()
                waypoint_base[0] += forward_step
                approach = {
                    "waypoint_base_m": waypoint_base,
                    "remaining_before_m": remaining,
                    "remaining_after_m": remaining - forward_step,
                    "complete": False,
                    "motion_axis_base": [
                        float(forward_step), 0.0, 0.0,
                    ],
                    "step_kind": "ALIGNED_PRESS_FORWARD_WITH_FIXED_TIP_YZ",
                    "press_phase_latched": True,
                    "alignment_reference": "FIXED_LOWER_TIP",
                    "fixed_tip_in_mask": True,
                    "arm_mask_fraction": mask_fraction,
                    "lateral_error_before_m": lateral_error.tolist(),
                    "lateral_step_m": [0.0, 0.0],
                }
            if approach["complete"]:
                with self._sam_lock:
                    self._sam.unload()
                self._automatic_approach = False
                self._automatic_reacquire_due = None
                self._approach_plan = {
                    **approach, **tip_alignment,
                    "fixed_tip_uv": fixed_tip_uv.tolist(),
                    "refined_point_base_m": refined_point.tolist(),
                    "lateral_correction_m": lateral_correction.tolist(),
                    "execution_enabled": False,
                }
                self._set_state(
                    "APPROACH_MAX_TRAVEL_REACHED",
                    f"No appearance-change success was detected before the "
                    f"maximum guarded press travel "
                    f"({-approach['remaining_before_m'] * 1000.0:.1f}mm past the D435 "
                    "target plane); stopped for inspection and released SAM",
                )
                return
            arm_from_base = self._latest_transform("arm/base_link", "base_link")
            arm_rotation, arm_translation = self._rotation_translation(arm_from_base)
            waypoint_base = np.asarray(approach["waypoint_base_m"], dtype=np.float64)
            waypoint_arm = arm_rotation @ waypoint_base + arm_translation
            approach_axis_arm = arm_rotation @ np.array([1.0, 0.0, 0.0])
            approach_axis_arm /= max(float(np.linalg.norm(approach_axis_arm)), 1.0e-12)
            current = [self._arm_joints[name] for name in REACH_JOINT_NAMES]
            effective_specs = {
                str(item.get("name")): item
                for item in self._arm_state.get("joints", [])
                if isinstance(item, dict)
            }
            if not all(name in effective_specs for name in REACH_JOINT_NAMES):
                raise ValueError("Arm gateway effective joint limits are unavailable")
            limit_margin = float(self.get_parameter("ik_joint_limit_margin_rad").value)
            lower = [
                float(effective_specs[name]["lower"]) + limit_margin
                for name in REACH_JOINT_NAMES
            ]
            upper = [
                float(effective_specs[name]["upper"]) - limit_margin
                for name in REACH_JOINT_NAMES
            ]
            if any(lo >= hi for lo, hi in zip(lower, upper)):
                raise ValueError("configured IK joint-limit margin leaves an invalid range")
            if approach["step_kind"] == "ALIGN_FIXED_TIP_YZ":
                solution = solve_near_tip_alignment(
                    current, waypoint_arm,
                    lower_limits=lower, upper_limits=upper,
                )
            elif approach["step_kind"] == "ALIGNED_PRESS_FORWARD_WITH_FIXED_TIP_YZ":
                solution = solve_near_forward_press(
                    current, waypoint_arm,
                    lower_limits=lower, upper_limits=upper,
                )
            else:
                solution = solve_pre_touch(
                    current, waypoint_arm, approach_axis_arm,
                    lower_limits=lower, upper_limits=upper,
                    locked_reference_id=(
                        str(self._reach_plan["prepare_reference_id"])
                        if self._reach_plan is not None else None
                    ),
                )
        except (KeyError, TypeError, ValueError, TransformException) as exc:
            if self._automatic_approach:
                self._pause_automatic_approach(
                    "APPROACH_BLOCKED",
                    f"Automatic approach paused: {exc}",
                )
            else:
                self._set_state("APPROACH_BLOCKED", str(exc))
            return
        gripper = float(self.get_parameter("touch_gripper_position_rad").value)
        positions = {**solution["positions"], "gripper_joint": gripper}
        predicted_tip_arm = np.asarray(solution["tip_position"], dtype=np.float64)
        predicted_tip_base = arm_rotation.T @ (predicted_tip_arm - arm_translation)
        approach_ik_limit = float(self.get_parameter("approach_max_ik_error_m").value)
        safe_approach_solution = bool(
            solution["success"]
            and float(solution["position_error_m"]) <= approach_ik_limit
        )
        if approach["step_kind"] in {
            "ALIGNED_PRESS_FORWARD_WITH_FIXED_TIP_YZ",
            "COARSE_CENTER_AND_FORWARD_TO_NEAR",
        }:
            forward_limit = refined_point[0] + max_press_past_target
            safe_approach_solution = bool(
                safe_approach_solution
                and predicted_tip_base[0] > actual_tip[0] + 0.0005
                and predicted_tip_base[0] <= forward_limit + approach_ik_limit
            )
        elif approach["step_kind"] in {
            "ALIGN_FIXED_TIP_YZ", "NEAR_ALIGN_IMAGE_CENTER_YZ",
        }:
            safe_approach_solution = bool(
                safe_approach_solution
                and abs(predicted_tip_base[0] - actual_tip[0])
                <= approach_ik_limit
            )
        elif approach["step_kind"] == "MASK_TOO_CLOSE_RETREAT_SMALL":
            safe_approach_solution = bool(
                safe_approach_solution
                and predicted_tip_base[0] < actual_tip[0] - 0.0005
            )
        self._approach_plan = {
            **approach,
            **tip_alignment,
            "fixed_tip_uv": fixed_tip_uv.tolist(),
            "refined_point_base_m": refined_point.tolist(),
            "lateral_correction_m": lateral_correction.tolist(),
            "alignment_reference": (
                "FIXED_LOWER_TIP" if use_tip_alignment else "IMAGE_CENTER"
            ),
            "alignment_error_px": float(tip_alignment["pixel_error_norm"]),
            "fixed_tip_error_px": (
                float(tip_alignment["pixel_error_norm"])
                if use_tip_alignment else None
            ),
            "arm_mask_fraction": mask_fraction,
            "too_close_mask_fraction_threshold": too_close_mask_fraction,
            "too_close": too_close,
            "advanced_from_far_m": advanced_from_far_m,
            "near_advance_distance_m": near_advance_m,
            "near_reached": near_reached,
            "near_phase_latched": self._near_phase_latched,
            "waypoint_arm_base_m": waypoint_arm.tolist(),
            "actual_tip_base_m": actual_tip.tolist(),
            "predicted_tip_base_m": predicted_tip_base.tolist(),
            "ik": solution,
            "positions": positions,
            "candidate_id": None,
            "execution_enabled": bool(
                self.get_parameter("allow_approach_execution").value
            ),
            "policy": "40MM_NEAR_CENTER_THEN_TIP_ALIGN;15PCT_SMALL_RETREAT",
        }
        self._publish_target()
        if not safe_approach_solution:
            diagnostic_id = f"touch-approach-diagnostic-{uuid.uuid4()}"
            self._arm_command_pub.publish(String(data=json.dumps({
                "mode": str(self._arm_state.get("mode", "REAL_DASHBOARD_TEST")),
                "request_id": diagnostic_id,
                "command": "diagnostic_preview",
                "positions": positions,
            }, ensure_ascii=False, allow_nan=False)))
            reason = (
                f"Step IK/error/direction gate rejected "
                f"({solution['position_error_m'] * 1000.0:.1f}mm, limit "
                f"{approach_ik_limit * 1000.0:.1f}mm); kind="
                f"{approach['step_kind']}, X actual/planned/predicted="
                f"{actual_tip[0]:.4f}/{float(np.asarray(approach['waypoint_base_m'])[0]):.4f}/"
                f"{predicted_tip_base[0]:.4f}m; "
                "bounded orange diagnostic only",
            )
            reason = "".join(reason)
            if self._automatic_approach:
                self._pause_automatic_approach(
                    "APPROACH_UNREACHABLE", f"Automatic approach paused: {reason}"
                )
            else:
                self._set_state("APPROACH_UNREACHABLE", reason)
            return
        request_id = f"touch-approach-preview-{uuid.uuid4()}"
        self._approach_preview_request_id = request_id
        self._arm_command_pub.publish(String(data=json.dumps({
            "mode": str(self._arm_state.get("mode", "REAL_DASHBOARD_TEST")),
            "request_id": request_id,
            "command": "preview",
            "positions": positions,
        }, ensure_ascii=False, allow_nan=False)))
        self._set_state(
            "APPROACH_PREVIEW_REQUESTED",
            f"Visual step preview requested: remaining "
            f"{approach['remaining_before_m'] * 1000.0:.1f} → "
            f"{approach['remaining_after_m'] * 1000.0:.1f}mm; "
            f"kind={approach['step_kind']}; D435→arm-camera Y/Z correction="
            f"({lateral_correction[0] * 1000.0:.1f}, "
            f"{lateral_correction[1] * 1000.0:.1f})mm; "
            f"image error={pixel_error_norm:.1f}px; "
            f"arm mask={mask_fraction * 100.0:.1f}% / "
            f"too-close={too_close_mask_fraction * 100.0:.1f}%; "
            f"advanced from FAR={advanced_from_far_m * 1000.0:.1f}/"
            f"{near_advance_m * 1000.0:.1f}mm",
        )

    def _execute_approach_step(self, automatic: bool = False) -> None:
        if not bool(self.get_parameter("allow_approach_execution").value):
            self._set_state("BLOCKED", "Real approach-step execution is configuration-locked")
            return
        if (
            self._state != "APPROACH_STEP_PLANNED"
            or self._approach_plan is None
            or not self._approach_plan.get("candidate_id")
        ):
            self._set_state("BLOCKED", "Plan and review a current approach step first")
            return
        request_id = f"touch-approach-execute-{uuid.uuid4()}"
        self._arm_command_pub.publish(String(data=json.dumps({
            "mode": str(self._arm_state.get("mode", "REAL_DASHBOARD_TEST")),
            "request_id": request_id,
            "command": "execute_candidate",
            "candidate_id": self._approach_plan["candidate_id"],
            "duration_sec": 1.5,
        }, ensure_ascii=False, allow_nan=False)))
        self._approach_execute_request_id = request_id
        self._active_step_motor_started = time.monotonic()
        self._set_state(
            "APPROACH_EXECUTION_REQUESTED",
            ("Automatic" if automatic else "Operator-requested")
            + " guarded approach step sent through the Arm gateway; "
            "the target/contact step remains locked",
        )

    def _automatic_approach_tick(self) -> None:
        with self._lock:
            if not self._automatic_approach:
                return
            started = self._automatic_approach_started
            steps = self._automatic_approach_steps
            due = self._automatic_reacquire_due
            state = self._state
        maximum_steps = int(
            self.get_parameter("automatic_approach_max_steps").value
        )
        timeout_sec = float(
            self.get_parameter("automatic_approach_timeout_sec").value
        )
        if started is None or time.monotonic() - started > timeout_sec:
            self._pause_automatic_approach(
                "AUTO_APPROACH_TIMEOUT",
                f"Automatic approach exceeded {timeout_sec:.1f}s; no further motion sent",
            )
            return
        if steps >= maximum_steps:
            self._pause_automatic_approach(
                "AUTO_APPROACH_STEP_LIMIT",
                f"Automatic approach reached the {maximum_steps}-step limit; "
                "no further motion sent",
            )
            return
        if (
            state in {
                "AUTO_APPROACH_SETTLING",
                "AUTO_APPROACH_CARTESIAN_CORRECTING",
                "TOUCH_CHANGE_VERIFYING",
            }
            and due is not None
            and time.monotonic() >= due
        ):
            with self._lock:
                # Claim this due event before starting asynchronous SAM work.
                self._automatic_reacquire_due = None
            self._acquire_arm_target(automatic_reacquire=True)

    def _far_arm_handoff_tick(self) -> None:
        """Automatically freeze and segment the first wrist-camera frame after FAR."""
        with self._lock:
            due = self._far_arm_acquire_due
            state = self._state
        if due is None or state != "PRETOUCH_FAR_REACHED" or time.monotonic() < due:
            return
        if (
            self._last_mechanism_motion is not None
            and time.monotonic() - self._last_mechanism_motion < 0.50
        ):
            with self._lock:
                self._far_arm_acquire_due = time.monotonic() + 0.20
            return
        with self._lock:
            self._far_arm_acquire_due = None
            self._capture_far_motor_baseline()
        self._acquire_arm_target(automatic_reacquire=False)

    def _publish_one_shot_camera_adjustment(self) -> tuple[float, float]:
        pan = float(self.get_parameter("touch_camera_pan_deg").value)
        tilt = float(self.get_parameter("touch_camera_tilt_deg").value)
        self._pan_target_pub.publish(Float32(data=pan))
        self._tilt_target_pub.publish(Float32(data=tilt))
        self._camera_adjustment_requested_at = time.monotonic()
        self.get_logger().info(
            f"Touch camera one-shot adjustment requested: pan={pan:.2f}, tilt={tilt:.2f} deg"
        )
        return pan, tilt

    def _start_prepare_camera_adjustment(self) -> None:
        if self._reach_plan is None:
            self._set_state("BLOCKED", "Validated FAR plan disappeared before camera adjustment")
            return
        pan = float(self._reach_plan["prepare_camera_pan_deg"])
        tilt = float(self._reach_plan["prepare_camera_tilt_deg"])
        now = time.monotonic()
        self._prepare_camera_goal = {
            "pan_deg": pan, "tilt_deg": tilt, "requested_at": now,
            "reference_id": str(self._reach_plan["prepare_reference_id"]),
        }
        self._pan_target_pub.publish(Float32(data=pan))
        self._tilt_target_pub.publish(Float32(data=tilt))
        self._camera_adjustment_requested_at = now
        self._set_state(
            "PREPARE_CAMERA_ADJUSTING",
            f"Arm preview accepted; moving D435 to measured reference "
            f"{self._prepare_camera_goal['reference_id']} pan={pan:.2f}, tilt={tilt:.2f} deg; "
            "no Arm motion command sent",
        )

    def _prepare_camera_tick(self) -> None:
        goal = self._prepare_camera_goal
        if not self._active or goal is None or self._state != "PREPARE_CAMERA_ADJUSTING":
            return
        ready, _reason = self._camera_feedback_ready()
        if not ready:
            return
        # The prepare table records observed feedback poses.  The OpenRB target
        # and feedback coordinates have a real, repeatable offset, so equality
        # between the requested number and feedback is not an arrival proof.
        # Once fresh feedback is stationary, freeze a new D435 frame and let
        # target re-detection/3-D shift validation decide whether it is usable.
        pan_error = abs(float(self._pan_raw) - float(goal["pan_deg"]))
        tilt_error = abs(float(self._tilt_raw) - float(goal["tilt_deg"]))
        if self._reach_plan is not None:
            self._reach_plan["prepare_camera_observation"] = {
                "reference_pan_feedback_deg": float(goal["pan_deg"]),
                "reference_tilt_feedback_deg": float(goal["tilt_deg"]),
                "settled_pan_feedback_deg": float(self._pan_raw),
                "settled_tilt_feedback_deg": float(self._tilt_raw),
                "pan_residual_deg": pan_error,
                "tilt_residual_deg": tilt_error,
                "gate": "FRESH_STATIONARY_FEEDBACK_THEN_D435_TARGET_REDETECTION",
            }
        self.get_logger().info(
            "D435 prepare motion settled; reference residual is advisory: "
            f"pan={pan_error:.2f}deg, tilt={tilt_error:.2f}deg"
        )
        self._prepare_camera_goal = None
        self._start_d435_target_reacquisition()

    def _start_d435_target_reacquisition(self) -> None:
        if self._target is None or self._reach_plan is None:
            self._set_state("D435_TARGET_REACQUIRE_REQUIRED", "Original target or FAR plan is missing")
            return
        old_target = dict(self._target)
        old_plan = dict(self._reach_plan)
        # Make freeze failure unambiguous; otherwise an older frozen frame
        # could remain non-null and be mistaken for the post-adjustment image.
        self._frozen = None
        self._freeze()
        if self._frozen is None:
            self._set_state("D435_TARGET_REACQUIRE_REQUIRED", "Could not freeze a fresh D435 frame")
            return
        self._reach_plan = old_plan
        self._frozen["operator_confirmed"] = True
        self._frozen["confirmation_provenance"] = (
            "INHERITED_FROM_OPERATOR_CONFIRMED_TARGET_FOR_AUTOMATIC_CONTINUITY_CHECK"
        )
        try:
            snapshot = self._frozen["tf_snapshot"]
            rotation = quaternion_matrix_xyzw(np.asarray(snapshot["quaternion_xyzw"], dtype=float))
            translation = np.asarray(snapshot["translation"], dtype=float)
            point_base = np.asarray(old_target["point_target"], dtype=float)
            point_camera = rotation.T @ (point_base - translation)
            u, v = project_camera_point(
                point_camera, self._frozen["camera_matrix"],
                np.asarray(self._frozen["distortion"], dtype=float),
            )
            height, width = self._frozen["rgb"].shape[:2]
            if not (0 <= u < width and 0 <= v < height):
                raise ValueError(f"projected target ({u:.1f}, {v:.1f}) is outside the D435 image")
        except (KeyError, TypeError, ValueError) as exc:
            self._d435_reacquire_reference = None
            self._set_state(
                "D435_TARGET_REACQUIRE_REQUIRED",
                f"Automatic D435 target projection failed: {exc}; click the same object again",
            )
            return
        self._d435_reacquire_reference = {
            "target": old_target, "plan": old_plan, "projected_uv": [u, v],
        }
        self._set_state(
            "D435_TARGET_REACQUIRING",
            f"Reference camera pose reached; SAM is verifying the retained target near "
            f"({u:.0f}, {v:.0f})",
        )
        self._add_prompt({"u": int(round(u)), "v": int(round(v)), "label": 1})

    def _camera_feedback_ready(self) -> tuple[bool, str]:
        now = time.monotonic()
        if (
            self._pan_raw is None
            or self._tilt_raw is None
            or self._pan_received is None
            or self._tilt_received is None
            or now - self._pan_received > 1.0
            or now - self._tilt_received > 1.0
        ):
            return False, "Pan/tilt feedback is missing or stale; camera pose cannot be recorded"
        if (
            self._camera_adjustment_requested_at is not None
            and (
                self._pan_received < self._camera_adjustment_requested_at
                or self._tilt_received < self._camera_adjustment_requested_at
            )
        ):
            return False, "Waiting for pan/tilt feedback after the one-shot adjustment request"
        if (
            self._camera_adjustment_requested_at is not None
            and now - self._camera_adjustment_requested_at
            < float(self.get_parameter("camera_motion_settle_sec").value)
        ):
            return False, "One-shot pan/tilt adjustment was just requested; wait before freezing"
        if (
            self._last_camera_motion is not None
            and now - self._last_camera_motion
            < float(self.get_parameter("camera_motion_settle_sec").value)
        ):
            return False, "Pan/tilt is still moving; wait before freezing"
        return True, "Fresh, stable pan/tilt feedback is available"

    def _set_state(self, state: str, message: str) -> None:
        with self._lock:
            self._state = state
            self._message = message
        self.get_logger().info(f"{state}: {message}")
        self._publish_state()

    def _publish_state(self) -> None:
        with self._lock:
            available, sam_reason = self._sam.availability()
            arm_allowed, arm_reason = self._arm_gate()
            frozen = self._frozen
            target = self._target
            payload = {
                "schema_version": 1,
                "active": self._active,
                "state": self._state,
                "message": self._message,
                "base_lock": self._active,
                "motion_execution": "GUARDED_FAR_AND_AUTO_STOP_OBSERVE_APPROACH_ONLY",
                "arm_gate": {"allowed": arm_allowed, "reason": arm_reason},
                "robot_mode": self._robot_mode,
                "camera_pose": {
                    "requested_pan_deg": float(
                        self.get_parameter("touch_camera_pan_deg").value
                    ),
                    "requested_tilt_deg": float(
                        self.get_parameter("touch_camera_tilt_deg").value
                    ),
                    "feedback_pan_deg": self._pan_raw,
                    "feedback_tilt_deg": self._tilt_raw,
                    "ready": self._camera_feedback_ready()[0],
                    "prepare_reference_goal": self._prepare_camera_goal,
                    "policy": (
                        "ONE_SHOT_ENTRY_THEN_STABLE_FEEDBACK_AND_D435_REACQUISITION_GATE"
                    ),
                },
                "sam": {
                    "implementation": "SAM 2.1 Hiera Tiny",
                    "device": "cpu",
                    "cpu_threads": self._sam.cpu_threads,
                    "available": available,
                    "loaded": self._sam.predictor is not None,
                    "reason": sam_reason,
                    "checkpoint": str(self._sam.checkpoint),
                    "config": self._sam.config,
                },
                "observation": None if frozen is None else {
                    "id": frozen["id"],
                    "stamp": frozen["stamp"],
                    "frame_id": frozen["frame_id"],
                    "width": int(frozen["rgb"].shape[1]),
                    "height": int(frozen["rgb"].shape[0]),
                    "depth_scale_m_per_unit": float(
                        self.get_parameter("depth_scale_m_per_unit").value
                    ),
                    "depth_skew_sec": frozen["depth_skew_sec"],
                    "camera_info_skew_sec": frozen["camera_info_skew_sec"],
                    "prompt_count": len(self._prompts),
                    "artifact_directory": str(
                        Path(str(self.get_parameter("artifact_root").value)).expanduser()
                        / f"observation_{frozen['id']}"
                    ),
                },
                "target": None if target is None else {
                    key: value.tolist() if isinstance(value, np.ndarray) else value
                    for key, value in target.items()
                },
                "reach_plan": self._reach_plan,
                "arm_camera_handoff": {
                    "stage": self._overlay_stage,
                    "d435_handoff_complete": self._arm_handoff_complete,
                    "frame_id": (
                        None if self._arm_frozen is None
                        else self._arm_frozen.get("frame_id")
                    ),
                    "d435_projected_uv": (
                        None if self._arm_frozen is None
                        else self._arm_frozen.get("d435_projected_uv")
                    ),
                    "target": jsonable(self._arm_target),
                    "operator_confirmed_same_object": self._arm_target_confirmed,
                    "distance_policy": "NO_MONOCULAR_METRIC_DEPTH",
                },
                "approach_plan": jsonable(self._approach_plan),
                "automatic_approach": {
                    "enabled": self._automatic_approach,
                    "steps_completed": self._automatic_approach_steps,
                    "maximum_steps": int(self.get_parameter(
                        "automatic_approach_max_steps"
                    ).value),
                    "too_close_mask_fraction": float(self.get_parameter(
                        "approach_too_close_mask_fraction"
                    ).value),
                    "too_close_retreat_m": float(self.get_parameter(
                        "approach_too_close_retreat_m"
                    ).value),
                    "near_advance_distance_m": float(self.get_parameter(
                        "approach_near_advance_distance_m"
                    ).value),
                    "reacquire_due": self._automatic_reacquire_due,
                    "policy": "ONE_HUMAN_CONFIRMATION_THEN_BOUNDED_STOP_OBSERVE_LOOP",
                },
                "contact_evidence": {
                    "state": "RECORDED_NOT_YET_CLASSIFIED",
                    "geometric": None,
                    "motion_resistance": None,
                    "current_delta": {
                        "far_baseline": jsonable(self._far_motor_baseline),
                        "approach_steps": jsonable(self._motor_step_evidence),
                        "classification": "NO_VALIDATED_CONTACT_THRESHOLD",
                    },
                    "visual_change": None,
                },
                "required_before_real_touch": [
                    "calibrated arm/touch_tip transform",
                    "SO101 MoveIt configuration matching current URDF and limits",
                    "gateway-mediated trajectory/servo path",
                    "measured contact current/force and retreat thresholds",
                    "T1/T2/T3 staged acceptance tests",
                ],
            }
        self._lock_pub.publish(Bool(data=self._active))
        self._state_pub.publish(String(data=json.dumps(payload, ensure_ascii=False, allow_nan=False)))

    def _maintenance(self) -> None:
        idle = float(self.get_parameter("sam_idle_unload_sec").value)
        # Once Touch Mode has loaded SAM, retain the weights across the D435
        # and arm-camera stages. Exit/cancel and APPROACH_COMPLETE release it.
        with self._sam_lock:
            unloaded = False if self._active else self._sam.unload_if_idle(idle)
        if unloaded:
            self.get_logger().info(f"Unloaded idle SAM 2.1 model after {idle:.0f}s")
            self._publish_state()
        if self._target is not None:
            self._publish_target()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TouchObservationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._lock_pub.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
