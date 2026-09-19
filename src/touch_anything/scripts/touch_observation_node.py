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
import traceback
from typing import Any
import uuid

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped, PoseStamped, TransformStamped, Twist
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
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
    tip_transform,
)
from approach_geometry import (
    base_yz_pixel_alignment_delta,
    compensated_cartesian_delta,
    diagnostic_visualization_should_defer,
    empirical_image_center_alignment_delta,
    monocular_lateral_target,
    mild_single_motor_hold_entry_overrun,
    next_approach_waypoint,
    pixel_inside_mask,
    phase_sam_minimum_score,
    phase_tracking_area_ratio_bounds,
    project_camera_point,
    receding_joint_action_horizon,
    rolling_approach_phase,
    rolling_forward_target_x,
    rolling_tracker_freshness_gate,
    rolling_z_motion_compensation,
    should_retreat_for_too_close,
    validate_tracking_continuity,
)
from target_tracker import ContinuousTargetTracker, TrackerConfig


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


def post_motion_tf_rejection_reason(
    transform_stamp_sec: float,
    now_ros_sec: float,
    minimum_completion_stamp: float | None,
    maximum_age_sec: float,
) -> str | None:
    """Explain why a TF sample is not valid post-motion evidence."""
    if transform_stamp_sec <= 0.0:
        return "latest Arm TF has no dynamic timestamp"
    if (
        minimum_completion_stamp is not None
        and transform_stamp_sec < minimum_completion_stamp
    ):
        return (
            "latest Arm TF predates the completed motion: "
            f"stamp={transform_stamp_sec:.6f}s, required>="
            f"{minimum_completion_stamp:.6f}s"
        )
    age_sec = now_ros_sec - transform_stamp_sec
    if age_sec > maximum_age_sec:
        return (
            f"latest Arm TF is still pre-motion/stale: age={age_sec:.3f}s "
            f"(limit {maximum_age_sec:.3f}s)"
        )
    if age_sec < -0.5:
        return (
            f"latest Arm TF is {abs(age_sec):.3f}s in the future; "
            "ROS clocks are not synchronized"
        )
    return None


def motion_tracking_evidence(
    successful_frame_stamps: list[float],
    dispatch_stamp_sec: float,
    completion_stamp_sec: float,
    minimum_frames: int,
    maximum_gap_sec: float,
) -> dict[str, Any]:
    """Prove that successful tracker updates covered one arm-motion window.

    Merely seeing a healthy tracker before and after a trajectory is not
    evidence that tracking ran *during* motion.  Retain the successful image
    stamps and require both a minimum sample count and bounded gaps, including
    the two motion-window boundaries.
    """
    start = float(dispatch_stamp_sec)
    end = float(completion_stamp_sec)
    required = max(1, int(minimum_frames))
    gap_limit = float(maximum_gap_sec)
    stamps = sorted({
        float(stamp) for stamp in successful_frame_stamps
        if math.isfinite(float(stamp)) and start < float(stamp) <= end
    })
    boundary_stamps = [start, *stamps, end]
    gaps = [
        boundary_stamps[index + 1] - boundary_stamps[index]
        for index in range(len(boundary_stamps) - 1)
    ]
    maximum_observed_gap = max(gaps, default=math.inf)
    valid_window = bool(
        math.isfinite(start)
        and math.isfinite(end)
        and end > start
        and math.isfinite(gap_limit)
        and gap_limit > 0.0
    )
    continuous = bool(
        valid_window
        and len(stamps) >= required
        and maximum_observed_gap <= gap_limit
    )
    return {
        "continuous": continuous,
        "dispatch_stamp_sec": start,
        "completion_stamp_sec": end,
        "motion_window_sec": end - start,
        "successful_frame_count": len(stamps),
        "minimum_required_frames": required,
        "first_successful_frame_stamp_sec": stamps[0] if stamps else None,
        "last_successful_frame_stamp_sec": stamps[-1] if stamps else None,
        "maximum_gap_sec": maximum_observed_gap,
        "maximum_allowed_gap_sec": gap_limit,
        "reason": (
            "successful tracker frames continuously covered the arm motion"
            if continuous else
            f"tracker coverage failed: frames={len(stamps)}/{required}, "
            f"maximum gap={maximum_observed_gap:.3f}s/{gap_limit:.3f}s"
        ),
    }


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


def arm_frame_geometry_evidence(frame: dict[str, Any]) -> dict[str, Any]:
    """Serialize Arm-camera geometry without requiring it on tracker frames.

    KLT/ORB frames deliberately contain RGB and a stamp only. SAM semantic
    recovery may save one of those frames before operator confirmation; the
    absent CameraInfo/TF is evidence state, not target loss.
    """
    required = (
        "camera_matrix",
        "distortion_model",
        "distortion",
        "base_from_camera_translation",
        "base_from_camera_rotation",
    )
    ready = bool(frame.get("geometry_ready")) and all(
        key in frame for key in required
    )
    if not ready:
        return {
            "geometry_ready": False,
            "camera_info": None,
            "tf_base_link_from_arm_camera": None,
        }
    translation = np.asarray(
        frame["base_from_camera_translation"], dtype=np.float64
    )
    return {
        "geometry_ready": True,
        "camera_info": {
            "k": np.asarray(frame["camera_matrix"], dtype=np.float64)
            .reshape(-1).tolist(),
            "distortion_model": frame["distortion_model"],
            "d": jsonable(frame["distortion"]),
        },
        "tf_base_link_from_arm_camera": {
            "translation_m": translation.tolist(),
            "rotation_matrix": np.asarray(
                frame["base_from_camera_rotation"], dtype=np.float64
            ).tolist(),
            "distance_from_base_link_origin_m": float(np.linalg.norm(translation)),
        },
    }


def approach_motion_request_in_flight(
    preview_request_id: str | None, execute_request_id: str | None
) -> bool:
    """True while a visual step is owned by the Arm gateway."""
    return preview_request_id is not None or execute_request_id is not None


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
        self.declare_parameter("arm_camera_max_receive_age_sec", 1.0)
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
        self.declare_parameter("approach_press_entry_tolerance_px", 35.0)
        self.declare_parameter("approach_tip_align_elbow_mild_excess_ticks", 5)
        self.declare_parameter("approach_too_close_mask_fraction", 0.15)
        self.declare_parameter("approach_too_close_retreat_m", 0.005)
        self.declare_parameter("approach_near_advance_distance_m", 0.035)
        self.declare_parameter("approach_max_press_past_target_m", 0.100)
        self.declare_parameter("approach_alignment_tolerance_m", 0.003)
        self.declare_parameter("approach_max_lateral_step_m", 0.005)
        self.declare_parameter("approach_max_ik_error_m", 0.003)
        # PRESS is explicitly authorized by the operator after inspecting the
        # wrist-camera image.  It therefore uses a wider, stage-specific IK
        # residual envelope than autonomous FAR/NEAR alignment.  Joint limits,
        # finite solver output, +X direction and the total travel bound remain
        # mandatory.
        self.declare_parameter("approach_press_max_ik_error_m", 0.020)
        self.declare_parameter("approach_press_recovery_step_m", 0.025)
        self.declare_parameter("touch_gripper_position_rad", 1.549)
        self.declare_parameter("approach_near_z_first_tolerance_px", 15.0)
        self.declare_parameter("approach_near_max_rotation_error_deg", 5.0)
        self.declare_parameter("automatic_approach_after_confirmation", True)
        self.declare_parameter("automatic_approach_max_steps", 24)
        self.declare_parameter("automatic_approach_timeout_sec", 180.0)
        self.declare_parameter("automatic_approach_settle_sec", 0.75)
        # The outer visual loop runs at 10 Hz.  Do not always idle for the old
        # fixed settle delay: observe the real touch-tip TF and reacquire as
        # soon as a short window is stable.  A bounded maximum wait prevents a
        # permanently noisy TF source from hanging the Touch state machine.
        self.declare_parameter("automatic_approach_min_settle_sec", 0.20)
        self.declare_parameter("automatic_approach_stability_window_sec", 0.30)
        self.declare_parameter("automatic_approach_stability_span_m", 0.0025)
        self.declare_parameter("automatic_approach_stability_max_wait_sec", 1.20)
        self.declare_parameter("automatic_approach_stability_hard_span_m", 0.008)
        # During the supervised Touch demo, a finite but noisy TF span must not
        # permanently stop the closed loop.  After the bounded wait, reacquire
        # from the latest measured pose and keep trying until the operator
        # stops the workflow.  A missing/non-finite TF remains a hard failure.
        self.declare_parameter("automatic_approach_continue_when_unstable", True)
        self.declare_parameter("automatic_arm_target_max_shift_px", 90.0)
        self.declare_parameter("automatic_arm_target_max_area_ratio", 3.0)
        self.declare_parameter("automatic_arm_target_near_max_area_ratio", 12.0)
        self.declare_parameter("automatic_arm_target_far_min_area_ratio", 0.20)
        self.declare_parameter("automatic_arm_target_near_min_area_ratio", 0.05)
        self.declare_parameter("approach_cartesian_coarse_max_error_m", 0.040)
        self.declare_parameter("approach_cartesian_press_max_error_m", 0.020)
        self.declare_parameter("approach_cartesian_align_max_error_m", 0.008)
        self.declare_parameter("approach_cartesian_min_progress_ratio", 0.30)
        self.declare_parameter("approach_cartesian_correctable_max_error_m", 0.060)
        self.declare_parameter("approach_cartesian_max_correction_retries", 3)
        # The guarded trajectory server owns the servo bus while executing, so
        # its periodic /joint_states publication can be delayed until the
        # action callback releases that bus.  Wait for a post-motion Arm TF
        # sample before judging the Cartesian endpoint; an old, internally
        # self-consistent TF is not evidence of the pose just reached.
        self.declare_parameter("approach_cartesian_feedback_max_age_sec", 0.75)
        self.declare_parameter("approach_cartesian_feedback_wait_sec", 3.0)
        # A terminal gateway state and its JointState/TF may arrive in either
        # order. Require a sample from the completion window, then wait for the
        # next sample instead of accepting a fresh-looking pre-motion pose.
        self.declare_parameter("approach_cartesian_completion_tf_slack_sec", 0.25)
        self.declare_parameter("approach_cartesian_correction_step_max_m", 0.030)
        self.declare_parameter("approach_cartesian_correction_min_improvement_m", 0.003)
        self.declare_parameter("approach_cartesian_correction_min_improvement_ratio", 0.20)
        self.declare_parameter("approach_near_lateral_step_max_m", 0.030)
        self.declare_parameter("approach_near_feedforward_x_hold_m", 0.004)
        self.declare_parameter("approach_near_feedforward_y_gain", 1.10)
        self.declare_parameter("approach_near_feedforward_z_gain", 1.27)
        self.declare_parameter("approach_near_feedforward_z_up_bias_m", 0.019)
        self.declare_parameter("approach_near_feedforward_command_max_m", 0.050)
        # Latest real Z alignment evidence gives approximately
        # achieved_Z = 0.785 * commanded_Z - 14.5 mm.  Use its bounded inverse
        # in both the initial visual step and same-waypoint correction.
        self.declare_parameter("approach_near_z_first_feedforward_z_gain", 1.27)
        self.declare_parameter("approach_near_z_first_feedforward_z_bias_m", 0.019)
        self.declare_parameter("approach_near_z_first_command_max_m", 0.050)
        self.declare_parameter("arm_sam_minimum_mask_pixels", 40)
        self.declare_parameter("arm_sam_minimum_score", 0.40)
        self.declare_parameter("arm_sam_far_tracking_minimum_score", 0.20)
        self.declare_parameter("arm_sam_near_tracking_minimum_score", 0.10)
        self.declare_parameter("arm_sam_press_minimum_score", 0.05)
        self.declare_parameter("approach_far_feedforward_z_up_bias_m", 0.022)
        self.declare_parameter("touch_success_lab_delta", 60.0)
        self.declare_parameter("touch_success_changed_fraction", 0.65)
        self.declare_parameter("touch_success_max_x_remaining_m", 0.010)
        self.declare_parameter("touch_success_max_center_error_px", 15.0)
        self.declare_parameter("touch_success_required_observations", 2)
        self.declare_parameter("touch_success_minimum_step", 1)
        # SAM establishes semantic identity only at the wrist-camera handoff
        # and after a genuine geometric tracker loss.  Between those events a
        # lightweight KLT/RANSAC tracker runs independently from the 10 Hz
        # motion state machine; ORB/RANSAC is its first recovery tier.
        self.declare_parameter("continuous_tracker_enabled", True)
        self.declare_parameter("continuous_tracker_period_sec", 0.04)
        self.declare_parameter("approach_tracking_minimum_motion_frames", 3)
        self.declare_parameter("approach_tracking_maximum_motion_gap_sec", 0.50)
        # During each automatic approach segment the gateway accepts a bounded
        # 10 Hz joint target stream.  The low-level server executes it at
        # 20 Hz and enters HOLD if this producer misses its watchdog.
        self.declare_parameter("rolling_visual_servo_enabled", True)
        self.declare_parameter("rolling_visual_servo_duration_sec", 2.5)
        self.declare_parameter("rolling_visual_servo_continuous_until_press", True)
        self.declare_parameter("rolling_visual_servo_session_timeout_sec", 15.0)
        self.declare_parameter("rolling_visual_servo_forward_horizon_m", 0.006)
        self.declare_parameter("rolling_visual_servo_alignment_frames", 3)
        self.declare_parameter("rolling_visual_servo_max_yz_step_m", 0.003)
        self.declare_parameter("rolling_visual_servo_empirical_m_per_px", 0.00020)
        self.declare_parameter("rolling_visual_servo_z_residual_gain", 0.80)
        self.declare_parameter("rolling_visual_servo_far_z_per_x_gain", 0.88)
        self.declare_parameter("rolling_visual_servo_z_compensation_max_m", 0.025)
        self.declare_parameter("rolling_visual_servo_z_slowdown_start_m", 0.006)
        self.declare_parameter("rolling_visual_servo_z_slowdown_full_m", 0.020)
        self.declare_parameter("rolling_visual_servo_max_z_drop_m", 0.012)
        self.declare_parameter("rolling_visual_servo_max_tracker_age_sec", 0.20)
        self.declare_parameter(
            "rolling_visual_servo_max_tracker_inflight_sec", 0.15
        )
        self.declare_parameter("rolling_visual_servo_min_updates", 3)
        self.declare_parameter("rolling_visual_servo_horizon_steps", 10)
        self.declare_parameter(
            "rolling_visual_servo_horizon_step_period_sec", 0.05
        )
        self.declare_parameter(
            "rolling_visual_servo_execution_lead_sec", 0.08
        )
        self.declare_parameter(
            "rolling_visual_servo_max_prediction_lead_sec", 0.25
        )
        self.declare_parameter(
            "rolling_visual_servo_max_prediction_velocity_rad_s", 1.0
        )
        self.declare_parameter("tracker_max_features", 160)
        self.declare_parameter("tracker_minimum_tracks", 6)
        self.declare_parameter("tracker_minimum_inlier_ratio", 0.50)
        self.declare_parameter("tracker_minimum_coverage_ratio", 0.10)
        self.declare_parameter("tracker_lk_fb_error_px", 1.5)
        self.declare_parameter("tracker_orb_minimum_matches", 10)
        self.declare_parameter("tracker_rolling_keyframe_interval_frames", 15)
        self.declare_parameter("tracker_semantic_recovery_cooldown_sec", 1.0)
        # Press evidence is computed inside the continuously tracked region
        # relative to the first human-confirmed wrist-camera keyframe.
        self.declare_parameter("tracker_touch_lab_delta", 45.0)
        self.declare_parameter("tracker_touch_changed_fraction", 0.50)
        self.declare_parameter("tracker_touch_histogram_distance", 0.35)
        self.declare_parameter("tracker_touch_required_frames", 3)
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
        # Wrist tracking must always consume the newest camera sample.  Keeping
        # a queue here allowed an overloaded single-threaded executor to work
        # through old images after an Arm trajectory, eventually making the
        # image timestamp older than the TF buffer.
        self._arm_color: deque[Image] = deque(maxlen=1)
        # CameraInfo is calibration metadata, not a dynamic observation.  Keep
        # only the latest validated message and reuse it across RGB frames.
        self._arm_info: deque[CameraInfo] = deque(maxlen=1)
        self._arm_color_received: float | None = None
        self._arm_info_received: float | None = None
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
        self._rolling_servo_session_id: str | None = None
        self._rolling_servo_sequence = 0
        self._rolling_servo_deadline: float | None = None
        self._rolling_servo_last_tracker_stamp: float | None = None
        self._rolling_servo_updates_sent = 0
        self._rolling_servo_horizon_plan_count = 0
        self._rolling_servo_update_sent_this_tick = False
        self._rolling_servo_stop_reason: str | None = None
        self._rolling_servo_phase = "INACTIVE"
        self._rolling_servo_horizon_plan_count = 0
        self._rolling_servo_alignment_streak = 0
        self._rolling_servo_start_tip_base_m: np.ndarray | None = None
        self._rolling_servo_last_waypoint_base_m: np.ndarray | None = None
        self._rolling_servo_tracker_gate: dict[str, Any] | None = None
        self._pending_touch_success_evidence: dict[str, Any] | None = None
        self._approach_feedback_wait_request_id: str | None = None
        self._approach_feedback_wait_started: float | None = None
        self._approach_completion_observed_ros_sec: float | None = None
        self._approach_feedback_diagnostics: dict[str, Any] = {}
        self._automatic_approach = False
        self._automatic_approach_started: float | None = None
        self._automatic_approach_steps = 0
        self._automatic_reacquire_due: float | None = None
        self._automatic_settle_started: float | None = None
        self._automatic_settle_deadline: float | None = None
        self._automatic_settle_samples: deque[tuple[float, np.ndarray]] = deque(
            maxlen=30
        )
        self._automatic_tracking_reference: dict[str, Any] | None = None
        self._automatic_reacquire_pending = False
        self._automatic_last_pixel_error: float | None = None
        self._automatic_nonconverging_steps = 0
        self._last_executed_x_remaining_m: float | None = None
        self._cartesian_correction_retries = 0
        self._cartesian_correction_previous_error_m: float | None = None
        self._press_phase_active = False
        self._press_authorized = False
        self._press_motion_attempted = False
        self._press_recovery_pending = False
        self._approach_far_tip_x_m: float | None = None
        self._near_phase_latched = False
        self._near_tool_rotation_arm: np.ndarray | None = None
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
        self._target_tracker = ContinuousTargetTracker(TrackerConfig(
            max_features=int(self.get_parameter("tracker_max_features").value),
            lk_fb_error_px=float(self.get_parameter("tracker_lk_fb_error_px").value),
            minimum_tracks=int(self.get_parameter("tracker_minimum_tracks").value),
            minimum_inlier_ratio=float(self.get_parameter(
                "tracker_minimum_inlier_ratio"
            ).value),
            minimum_coverage_ratio=float(self.get_parameter(
                "tracker_minimum_coverage_ratio"
            ).value),
            orb_minimum_matches=int(self.get_parameter(
                "tracker_orb_minimum_matches"
            ).value),
            rolling_keyframe_interval_frames=int(self.get_parameter(
                "tracker_rolling_keyframe_interval_frames"
            ).value),
        ))
        # Serialize tracker mutation independently from the node-state lock.
        # OpenCV may release the GIL and a live frame can take longer under
        # load; holding _lock here used to starve Arm state and command
        # callbacks. The generation makes any in-flight result disposable
        # after a reset/reinitialization.
        self._tracker_operation_lock = threading.Lock()
        self._tracker_generation = 0
        self._tracker_last_stamp: float | None = None
        self._tracker_last_result: dict[str, Any] | None = None
        self._tracker_last_good_monotonic: float | None = None
        self._tracker_last_good_input_age_sec = 0.0
        self._tracker_inflight_started_monotonic: float | None = None
        self._tracker_inflight_image_received_monotonic: float | None = None
        self._tracker_inflight_stamp: float | None = None
        self._tracker_inflight_generation: int | None = None
        self._tracker_last_recovery_attempt: float | None = None
        self._tracker_semantic_recovery = False
        self._tracker_resume_after_confirmation = False
        self._tracker_identity_reference: dict[str, Any] | None = None
        self._tracker_appearance_streak = 0
        self._tracker_overlay_last_published = 0.0
        self._tracker_runtime_error_count = 0
        self._tracker_last_runtime_error: str | None = None
        self._tracker_discarded_generation_count = 0
        self._tracker_good_frame_stamps: deque[float] = deque(maxlen=600)
        # Overlay is diagnostic latest-value output. It must never backpressure
        # tracking or safety/control callbacks when RViz or rosbag is slow.
        self._overlay_dirty = True
        self._overlay_refresh_requested_count = 0
        self._overlay_coalesced_update_count = 0
        self._overlay_published_count = 0
        self._overlay_last_publish_duration_ms: float | None = None
        self._overlay_last_error: str | None = None
        self._overlay_deferred_for_motion_count = 0
        self._target_visualization_published_count = 0
        self._target_visualization_deferred_for_motion_count = 0
        self._target_visualization_last_publish_duration_ms: float | None = None
        # Per-tick debug telemetry is queued and published from its own
        # callback group.  A slow rosbag/RViz subscriber must never block the
        # tracker or 10 Hz control callback that the telemetry is measuring.
        self._tracker_debug_queue: deque[str] = deque(maxlen=1200)
        self._control_debug_queue: deque[str] = deque(maxlen=1200)
        self._tracker_debug_dropped = 0
        self._control_debug_dropped = 0
        self._control_tick_sequence = 0
        self._state_publish_deferred_count = 0
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
        self._arm_joint_velocities: dict[str, float] = {}
        # Coherent raw follower snapshot used for direct calibrated FK.  This
        # is deliberately separate from /tf: a delayed robot_state_publisher
        # must not make a freshly completed trajectory look unobservable.
        self._arm_joint_snapshot: dict[str, float] = {}
        self._arm_joint_snapshot_received: float | None = None
        self._arm_joint_snapshot_stamp_sec: float | None = None
        self._arm_mount_transform: TransformStamped | None = None
        self._arm_mount_received: float | None = None
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

        # High-bandwidth camera callbacks must not share the state machine's
        # default mutually-exclusive callback group.  With D435 RGB/depth,
        # CameraInfo and the wrist camera all active, that group can remain
        # continuously runnable and starve the 10 Hz approach timer after an
        # Arm trajectory.  The sensor callbacks only replace bounded latest-
        # message buffers under ``self._lock``, so they are safe in a
        # reentrant group.  Keep the OpenCV tracker in its own serial group:
        # it may run concurrently with sensor reception, but never with a
        # second tracker invocation.
        self._sensor_callback_group = ReentrantCallbackGroup()
        self._tracker_callback_group = MutuallyExclusiveCallbackGroup()
        self._arm_state_callback_group = MutuallyExclusiveCallbackGroup()
        self._control_callback_group = MutuallyExclusiveCallbackGroup()
        self._visualization_callback_group = MutuallyExclusiveCallbackGroup()
        self._telemetry_callback_group = ReentrantCallbackGroup()

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        # Dynamic TF is latest-state telemetry here, not a history stream.
        # Keep only the newest sample so image traffic cannot leave seconds of
        # old Arm transforms queued in this process. /tf_static remains
        # reliable/transient-local so fixed geometry is never lost.
        dynamic_tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        static_tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            # Several robot_state_publisher instances contribute independent
            # fixed trees.  Depth 1 can discard the bridge required to join
            # the mobile base and Arm trees for a late-joining listener.
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._tf_listener = TransformListener(
            self._tf_buffer, self, qos=dynamic_tf_qos, static_qos=static_tf_qos
        )
        self.create_subscription(
            Image, str(self.get_parameter("color_topic").value), self._on_color,
            qos_profile_sensor_data,
            callback_group=self._sensor_callback_group,
        )
        self.create_subscription(
            Image, str(self.get_parameter("depth_topic").value), self._on_depth,
            qos_profile_sensor_data,
            callback_group=self._sensor_callback_group,
        )
        self.create_subscription(
            CameraInfo, str(self.get_parameter("camera_info_topic").value), self._on_info,
            qos_profile_sensor_data,
            callback_group=self._sensor_callback_group,
        )
        arm_sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Image, str(self.get_parameter("arm_color_topic").value), self._on_arm_color,
            arm_sensor_qos,
            callback_group=self._sensor_callback_group,
        )
        self.create_subscription(
            CameraInfo, str(self.get_parameter("arm_camera_info_topic").value),
            self._on_arm_info, arm_sensor_qos,
            callback_group=self._sensor_callback_group,
        )
        latest_reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            String, "/touch_anything/command", self._on_command, 20,
            callback_group=self._control_callback_group,
        )
        self.create_subscription(
            String, "/arm/dashboard_state", self._on_arm_state,
            latest_reliable_qos, callback_group=self._arm_state_callback_group,
        )
        self.create_subscription(
            String, "/robot_mode", self._on_robot_mode, latest_reliable_qos,
            callback_group=self._telemetry_callback_group,
        )
        self.create_subscription(
            Twist, "/cmd_vel", self._on_cmd_vel, latest_reliable_qos,
            callback_group=self._telemetry_callback_group,
        )
        self.create_subscription(
            Float32, "/chokudomotor/angle",
            lambda msg: self._on_axis("pan", float(msg.data)), latest_reliable_qos,
            callback_group=self._telemetry_callback_group,
        )
        self.create_subscription(
            Float32, "/cameraswingmotor/angle",
            lambda msg: self._on_axis("tilt", float(msg.data)), latest_reliable_qos,
            callback_group=self._telemetry_callback_group,
        )
        joint_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, joint_qos,
            callback_group=self._telemetry_callback_group,
        )
        mount_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            TransformStamped, "/arm/mount_transform", self._on_arm_mount_transform,
            mount_qos, callback_group=self._telemetry_callback_group,
        )
        self.create_subscription(
            Float32, "/lift/position", self._on_lift_position,
            latest_reliable_qos, callback_group=self._telemetry_callback_group,
        )
        self.create_subscription(
            String, "/lift/state", self._on_lift_state, latest_reliable_qos,
            callback_group=self._telemetry_callback_group,
        )

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
        # Target markers are periodic, diagnostic latest-value data. Reliable
        # transient-local publication blocked the Python executor for 4.5--6 s
        # in the 2026-09-19 real bag while rosbag/RViz were attached, starving
        # the independent tracker timer even though camera frames continued.
        visualization_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._point_pub = self.create_publisher(
            PointStamped, "/touch_anything/target_point", visualization_qos
        )
        self._pose_pub = self.create_publisher(
            PoseStamped, "/touch_anything/target_pose", visualization_qos
        )
        self._marker_pub = self.create_publisher(
            MarkerArray, "/touch_anything/markers", visualization_qos
        )
        debug_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._tracker_debug_pub = self.create_publisher(
            String, "/touch_anything/tracker_debug", debug_qos
        )
        self._control_debug_pub = self.create_publisher(
            String, "/touch_anything/control_debug", debug_qos
        )
        self.create_timer(
            0.5, self._publish_state, callback_group=self._control_callback_group
        )
        # Keep the arm-camera TF marker useful while the operator moves or
        # previews the arm.  This is visualization-only and publishes no
        # mechanism command.
        self.create_timer(
            0.5, self._refresh_visualization,
            callback_group=self._visualization_callback_group,
        )
        self.create_timer(
            0.10, self._overlay_publish_tick,
            callback_group=self._visualization_callback_group,
        )
        self.create_timer(
            0.1, self._instrumented_automatic_approach_tick,
            callback_group=self._control_callback_group,
        )
        self.create_timer(
            0.1, self._far_arm_handoff_tick,
            callback_group=self._control_callback_group,
        )
        self.create_timer(
            0.1, self._prepare_camera_tick,
            callback_group=self._control_callback_group,
        )
        self.create_timer(
            max(0.02, float(self.get_parameter(
                "continuous_tracker_period_sec"
            ).value)),
            self._continuous_tracker_tick,
            callback_group=self._tracker_callback_group,
        )
        self.create_timer(
            0.02, self._debug_publish_tick,
            callback_group=self._telemetry_callback_group,
        )
        self.create_timer(
            5.0, self._maintenance,
            callback_group=self._control_callback_group,
        )
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
            self._arm_color_received = time.monotonic()

    def _on_arm_info(self, message: CameraInfo) -> None:
        with self._lock:
            self._arm_info.append(message)
            self._arm_info_received = time.monotonic()

    def _on_arm_mount_transform(self, message: TransformStamped) -> None:
        if (
            message.header.frame_id != "base_link"
            or message.child_frame_id != "arm/base_link"
        ):
            self.get_logger().warning(
                "Ignoring unexpected /arm/mount_transform frames: "
                f"{message.header.frame_id} -> {message.child_frame_id}"
            )
            return
        self._arm_mount_transform = message
        self._arm_mount_received = time.monotonic()

    def _arm_image_snapshot(self, color: Image) -> dict[str, Any]:
        """Copy one wrist RGB frame without consulting CameraInfo or TF.

        KLT/ORB propagation is image-space only.  Keeping this path independent
        from calibration and robot geometry prevents a transient TF gap from
        starving visual identity tracking.
        """
        rgb = self._bridge.imgmsg_to_cv2(color, desired_encoding="rgb8").copy()
        frame_id = str(color.header.frame_id)
        if not frame_id:
            raise ValueError("Arm-camera image frame_id is empty")
        return {
            "id": str(uuid.uuid4()),
            "rgb": rgb,
            "stamp": stamp_seconds(color),
            "stamp_msg": color.header.stamp,
            "frame_id": frame_id,
            "d435_projected_uv": None,
            "d435_projection_in_image": False,
            "tracking_frame": True,
            "geometry_ready": False,
        }

    def _arm_frame_with_geometry(self, frame: dict[str, Any]) -> dict[str, Any]:
        """Attach cached intrinsics and the latest robot TF for motion planning."""
        with self._lock:
            info = self._arm_info[-1] if self._arm_info else None
        if info is None:
            raise ValueError("Arm-camera CameraInfo is unavailable")
        camera_matrix = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
        if (
            not np.all(np.isfinite(camera_matrix))
            or float(camera_matrix[0, 0]) <= 0.0
            or float(camera_matrix[1, 1]) <= 0.0
        ):
            raise ValueError("Arm-camera cached CameraInfo intrinsics are invalid")
        color_stamp = float(frame["stamp"])
        info_skew = abs(stamp_seconds(info) - color_stamp)
        base_from_camera = self._latest_transform(
            "base_link", str(frame["frame_id"])
        )
        rotation, translation = self._rotation_translation(base_from_camera)
        return {
            **frame,
            "camera_matrix": camera_matrix,
            "distortion_model": str(info.distortion_model),
            "distortion": np.asarray(info.d, dtype=np.float64).tolist(),
            # Retain the skew as diagnostic evidence. CameraInfo describes a
            # fixed calibration and therefore is not rejected merely because
            # its ROS stamp differs from a later image stamp.
            "camera_info_skew_sec": info_skew,
            "base_from_camera_rotation": rotation,
            "base_from_camera_translation": translation,
            "geometry_ready": True,
        }

    def _arm_frame_snapshot(self, color: Image) -> dict[str, Any]:
        """Build a full wrist-camera frame for one-shot SAM/geometry work."""
        return self._arm_frame_with_geometry(self._arm_image_snapshot(color))

    def _refresh_arm_tracking_geometry(self) -> None:
        """Refresh planning geometry without disturbing image-space tracking."""
        with self._lock:
            if self._arm_frozen is None or self._arm_target is None:
                raise ValueError("Arm-camera tracking image/target is unavailable")
            frame = dict(self._arm_frozen)
            centroid = np.asarray(
                self._arm_target.get("centroid_uv", []), dtype=np.float64
            )
        if centroid.shape != (2,) or not np.all(np.isfinite(centroid)):
            raise ValueError("Arm-camera tracked centroid is invalid")
        frame = self._arm_frame_with_geometry(frame)
        with self._lock:
            # The single-threaded executor normally makes this exact. Keep the
            # identity check so a future executor change cannot attach geometry
            # to a different RGB frame.
            if (
                self._arm_frozen is None
                or float(self._arm_frozen["stamp"]) != float(frame["stamp"])
            ):
                raise ValueError("Arm-camera frame changed while geometry was sampled")
            self._arm_frozen = frame
            tracking_point = self._arm_camera_tracking_point(centroid)
            self._arm_target["tracking_point_base_m"] = tracking_point.tolist()
            self._arm_target["tracking_range_m"] = float(np.linalg.norm(
                tracking_point - np.asarray(
                    frame["base_from_camera_translation"], dtype=np.float64
                )
            ))

    def _queue_debug_payload(self, kind: str, payload: dict[str, Any]) -> None:
        """Queue small JSON telemetry without publishing in a critical callback."""
        queue = (
            self._tracker_debug_queue
            if kind == "tracker" else self._control_debug_queue
        )
        if len(queue) == queue.maxlen:
            if kind == "tracker":
                self._tracker_debug_dropped += 1
            else:
                self._control_debug_dropped += 1
        payload = {
            "schema_version": 1,
            "kind": kind.upper() + "_TICK",
            "queued_ros_sec": float(self.get_clock().now().nanoseconds) * 1.0e-9,
            "queue_dropped_total": (
                self._tracker_debug_dropped
                if kind == "tracker" else self._control_debug_dropped
            ),
            **payload,
        }
        queue.append(json.dumps(
            jsonable(payload), ensure_ascii=False, allow_nan=False,
            separators=(",", ":"),
        ))

    def _debug_publish_tick(self) -> None:
        """Drain debug queues outside tracker/control callback groups."""
        for queue, publisher in (
            (self._tracker_debug_queue, self._tracker_debug_pub),
            (self._control_debug_queue, self._control_debug_pub),
        ):
            for _ in range(min(8, len(queue))):
                publisher.publish(String(data=queue.popleft()))

    def _queue_tracker_tick(
        self,
        *,
        image_stamp: float | None,
        result: dict[str, Any],
        started_monotonic: float,
        image_received_monotonic: float | None,
        frame_claimed_monotonic: float | None,
        generation: int | None,
        event: str = "RESULT",
    ) -> None:
        completed = time.monotonic()
        with self._lock:
            execution = self._arm_state.get("execution", {})
            context = {
                "touch_active": self._active,
                "touch_state": self._state,
                "automatic_approach": self._automatic_approach,
                "approach_request_id": self._approach_execute_request_id,
                "gateway_execution_state": execution.get("state"),
                "gateway_request_id": execution.get("last_request_id"),
            }
        self._queue_debug_payload("tracker", {
            "event": event,
            "image_stamp": image_stamp,
            "generation": generation,
            "callback_started_monotonic_sec": started_monotonic,
            "callback_completed_monotonic_sec": completed,
            "callback_duration_ms": (completed - started_monotonic) * 1000.0,
            "frame_claimed_monotonic_sec": frame_claimed_monotonic,
            "initial_lock_wait_ms": (
                None if frame_claimed_monotonic is None
                else max(
                    0.0, frame_claimed_monotonic - started_monotonic
                ) * 1000.0
            ),
            "input_receive_age_ms_at_start": (
                None if image_received_monotonic is None
                else max(
                    0.0,
                    (frame_claimed_monotonic or started_monotonic)
                    - image_received_monotonic,
                ) * 1000.0
            ),
            "input_receive_age_ms_at_claim": (
                None if image_received_monotonic is None
                else max(
                    0.0,
                    (frame_claimed_monotonic or started_monotonic)
                    - image_received_monotonic,
                ) * 1000.0
            ),
            "result": result,
            "context": context,
        })

    def _continuous_tracker_tick(self) -> None:
        """Propagate the confirmed mask at camera rate, without running SAM."""
        tick_started = time.monotonic()
        claimed_stamp: float | None = None
        image_received_monotonic: float | None = None
        frame_claimed_monotonic: float | None = None
        tracker_generation: int | None = None
        if not bool(self.get_parameter("continuous_tracker_enabled").value):
            return
        with self._lock:
            if (
                not self._active
                or not self._arm_handoff_complete
                or not self._target_tracker.active
                or self._tracker_semantic_recovery
                or not self._arm_color
            ):
                return
            color = self._arm_color[-1]
            stamp = stamp_seconds(color)
            if self._tracker_last_stamp is not None and stamp <= self._tracker_last_stamp:
                return
            # Claim the image before conversion so another timer cannot process
            # the same frame while OpenCV releases the GIL.
            self._tracker_last_stamp = stamp
            claimed_stamp = stamp
            image_received_monotonic = self._arm_color_received
            tracker = self._target_tracker
            tracker_generation = self._tracker_generation
            frame_claimed_monotonic = time.monotonic()
            self._tracker_inflight_started_monotonic = frame_claimed_monotonic
            self._tracker_inflight_image_received_monotonic = (
                image_received_monotonic
            )
            self._tracker_inflight_stamp = claimed_stamp
            self._tracker_inflight_generation = tracker_generation
        try:
            # Tracking consumes RGB only. CameraInfo and TF are intentionally
            # deferred until the motion state machine requests another plan.
            frame = self._arm_image_snapshot(color)
            height, width = frame["rgb"].shape[:2]
            fixed_tip = self._fixed_arm_tip_pixel(width, height)
            with self._tracker_operation_lock:
                result = tracker.update(
                    frame["rgb"], excluded_tip_uv=fixed_tip
                )
                mask = (
                    None if tracker.current_mask is None
                    else tracker.current_mask.copy()
                )
            with self._lock:
                if (
                    tracker_generation != self._tracker_generation
                    or tracker is not self._target_tracker
                    or not self._active
                    or not self._arm_handoff_complete
                ):
                    # A cancel, reacquisition or new confirmation superseded
                    # this frame while OpenCV was running.
                    self._tracker_discarded_generation_count += 1
                    self._queue_tracker_tick(
                        image_stamp=claimed_stamp,
                        result=result,
                        started_monotonic=tick_started,
                        image_received_monotonic=image_received_monotonic,
                        frame_claimed_monotonic=frame_claimed_monotonic,
                        generation=tracker_generation,
                        event="DISCARDED_GENERATION",
                    )
                    return
                self._tracker_last_result = dict(result)
            self._queue_tracker_tick(
                image_stamp=claimed_stamp,
                result=result,
                started_monotonic=tick_started,
                image_received_monotonic=image_received_monotonic,
                frame_claimed_monotonic=frame_claimed_monotonic,
                generation=tracker_generation,
            )
            if result["status"] == "LOST":
                self._request_semantic_tracker_recovery(result, frame)
                return

            if mask is None:
                self._request_semantic_tracker_recovery(
                    {**result, "reason": "tracker returned no propagated mask"}, frame
                )
                return
            centroid = np.asarray(result["centroid_uv"], dtype=np.float64)
            with self._lock:
                previous = {} if self._arm_target is None else dict(self._arm_target)
                previous.pop("tracking_point_base_m", None)
                previous.pop("tracking_range_m", None)
                self._arm_frozen = frame
                self._arm_mask = mask.copy()
                self._arm_target = {
                    **previous,
                    "centroid_uv": centroid.tolist(),
                    "mask_pixels": int(result["mask_pixels"]),
                    "mask_fraction": float(result["mask_fraction"]),
                    "source": str(result["method"]),
                    "tracker": jsonable(result),
                    "operator_confirmed_same_object": True,
                    "automatic_continuity_verified": True,
                    "metric_depth_observed": False,
                }
                self._arm_target_confirmed = True
                self._automatic_tracking_reference = dict(self._arm_target)
                completed_now = time.monotonic()
                self._tracker_last_good_monotonic = completed_now
                self._tracker_last_good_input_age_sec = max(
                    0.0, completed_now - image_received_monotonic
                )
                self._tracker_good_frame_stamps.append(float(frame["stamp"]))
                should_check_touch = bool(
                    self._press_phase_active
                    and self._press_authorized
                    and self._press_motion_attempted
                )
            now = time.monotonic()
            if now - self._tracker_overlay_last_published >= 0.10:
                self._tracker_overlay_last_published = now
                self._request_overlay_refresh()
            if should_check_touch:
                evidence = self._tracked_region_appearance_change(frame["rgb"], mask)
                gate = self._touch_success_gate(centroid, frame["rgb"].shape[:2])
                # Once the operator explicitly authorized PRESS and a PRESS
                # trajectory was attempted, repeated change in the tracked
                # button region is primary evidence.  Metric/pixel geometry is
                # retained for diagnosis but cannot veto that evidence.
                gate["eligible"] = True
                gate["policy"] = (
                    "OPERATOR_AUTHORIZED_PRESS_PLUS_TRACKED_REGION_CHANGE;"
                    "GEOMETRY_DIAGNOSTIC_ONLY"
                )
                evidence["success_gate"] = gate
                if bool(evidence.get("changed")) and bool(gate.get("eligible")):
                    self._tracker_appearance_streak += 1
                else:
                    self._tracker_appearance_streak = 0
                required = int(self.get_parameter(
                    "tracker_touch_required_frames"
                ).value)
                evidence["consecutive_change_frames"] = self._tracker_appearance_streak
                evidence["required_change_frames"] = required
                if self._tracker_appearance_streak >= required:
                    self._complete_touch_and_return_home(evidence)
        except Exception as exc:
            # Never allow an individual cv_bridge/OpenCV/tracker frame to tear
            # down the ROS process. A later newest-frame callback may recover;
            # the motion state machine independently requires a fresh result.
            details = f"{type(exc).__name__}: {exc}"
            self._tracker_runtime_error_count += 1
            self._tracker_last_runtime_error = details
            self._queue_tracker_tick(
                image_stamp=claimed_stamp,
                result={
                    "status": "RUNTIME_ERROR",
                    "method": "NONE",
                    "reason": details,
                    "tracked_points": 0,
                    "inlier_ratio": 0.0,
                },
                started_monotonic=tick_started,
                image_received_monotonic=image_received_monotonic,
                frame_claimed_monotonic=frame_claimed_monotonic,
                generation=tracker_generation,
                event="EXCEPTION",
            )
            self.get_logger().error(
                "Continuous target tracker skipped frame without stopping the node: "
                + details + "\n" + traceback.format_exc()
            )
        finally:
            with self._lock:
                if (
                    self._tracker_inflight_stamp == claimed_stamp
                    and self._tracker_inflight_generation == tracker_generation
                ):
                    self._tracker_inflight_started_monotonic = None
                    self._tracker_inflight_image_received_monotonic = None
                    self._tracker_inflight_stamp = None
                    self._tracker_inflight_generation = None

    def _request_semantic_tracker_recovery(
        self, result: dict[str, Any], frame: dict[str, Any]
    ) -> None:
        """Run SAM recovery automatically, but never accept it automatically."""
        now = time.monotonic()
        with self._lock:
            if self._rolling_servo_session_id is not None:
                self._stop_rolling_visual_servo(
                    "KLT/ORB target loss during active motion"
                )
                self._automatic_approach = False
                self._automatic_reacquire_due = None
                self._set_state(
                    "AUTO_TARGET_LOST_HOLDING",
                    "Target tracker was lost during rolling motion; STOP sent and "
                    "the Arm server must acknowledge guarded HOLD before recovery",
                )
                return
            # Semantic recovery sets _automatic_approach False. Never allow a
            # transient tracker loss to do that while the gateway is validating
            # or executing an already planned step, otherwise an accepted
            # preview remains parked forever in PLANNED.
            if approach_motion_request_in_flight(
                self._approach_preview_request_id,
                self._approach_execute_request_id,
            ):
                return
            cooldown = float(self.get_parameter(
                "tracker_semantic_recovery_cooldown_sec"
            ).value)
            if (
                self._tracker_last_recovery_attempt is not None
                and now - self._tracker_last_recovery_attempt < cooldown
            ):
                return
            reference = self._automatic_tracking_reference or {}
            centroid = np.asarray(reference.get("centroid_uv", []), dtype=np.float64)
            if centroid.shape != (2,) or not np.all(np.isfinite(centroid)):
                self._pause_automatic_approach(
                    "AUTO_TARGET_LOST",
                    "KLT and ORB lost the target and no valid recovery prompt remains",
                )
                return
            height, width = frame["rgb"].shape[:2]
            prompt = (
                int(np.clip(round(float(centroid[0])), 0, width - 1)),
                int(np.clip(round(float(centroid[1])), 0, height - 1)),
            )
            self._tracker_last_recovery_attempt = now
            self._tracker_semantic_recovery = True
            self._tracker_resume_after_confirmation = bool(self._automatic_approach)
            self._automatic_approach = False
            self._automatic_reacquire_due = None
            self._arm_frozen = frame
            self._arm_prompts = [prompt]
            self._arm_labels = [1]
            self._arm_mask = None
            self._arm_target = None
            self._arm_target_confirmed = False
            self._approach_plan = None
            self._automatic_reacquire_pending = False
            self._job_id += 1
            job_id = self._job_id
        self._set_state(
            "ARM_SEMANTIC_RECOVERY_RUNNING",
            "KLT and ORB tracking failed; SAM semantic recovery is running. "
            "The recovered region will require operator confirmation: "
            + str(result.get("reason", "unknown tracker failure")),
        )
        threading.Thread(
            target=self._segment_arm_worker,
            args=(job_id, frame["rgb"].copy(), [prompt], [1]),
            daemon=True,
            name="sam21-arm-camera-semantic-recovery-worker",
        ).start()

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
                and (
                    execution.get("last_request_id") == self._approach_execute_request_id
                    or execution.get("owner_request_id")
                    == self._approach_execute_request_id
                    or (
                        self._rolling_servo_session_id is not None
                        and execution.get("visual_servo_session_id")
                        == self._rolling_servo_session_id
                    )
                )
            ):
                execution_state = str(execution.get("state", "UNKNOWN"))
                if (
                    execution_state == "EXECUTING"
                    and self._approach_plan is not None
                    and "motion_start_tip_base_m" not in self._approach_plan
                ):
                    try:
                        motion_start_tf = self._latest_transform(
                            "base_link", "arm/touch_tip"
                        )
                        _, motion_start_tip = self._rotation_translation(
                            motion_start_tf
                        )
                        self._approach_plan[
                            "motion_start_tip_base_m"
                        ] = motion_start_tip.tolist()
                        self._approach_plan["motion_start_tf_stamp_sec"] = (
                            float(motion_start_tf.header.stamp.sec)
                            + float(motion_start_tf.header.stamp.nanosec) * 1.0e-9
                        )
                        self._approach_plan[
                            "motion_start_monotonic_sec"
                        ] = now_monotonic
                        pre_execute = np.asarray(
                            self._approach_plan.get("pre_execute_tip_base_m", []),
                            dtype=np.float64,
                        )
                        if pre_execute.shape == (3,):
                            self._approach_plan[
                                "startup_transient_delta_base_m"
                            ] = (motion_start_tip - pre_execute).tolist()
                    except (TransformException, ValueError):
                        pass
                completion_transform = None
                completion_feedback_error = None
                rolling_stop_reason = self._rolling_servo_stop_reason
                continuous_terminal = bool(
                    execution_state == "SUCCEEDED"
                    and rolling_stop_reason is not None
                    and rolling_stop_reason != "segment duration completed"
                    and self.get_parameter(
                        "rolling_visual_servo_continuous_until_press"
                    ).value
                )
                if execution_state in {"SUCCEEDED", "FAILED", "CANCELLED", "BLOCKED"}:
                    rolling_updates = self._rolling_servo_updates_sent
                    self._rolling_servo_session_id = None
                    self._rolling_servo_deadline = None
                    self._capture_motion_tracking_evidence(execution)
                    if not continuous_terminal:
                        (
                            completion_transform,
                            completion_feedback_error,
                            feedback_waiting,
                        ) = self._completion_transform_after_motion(execution)
                        if feedback_waiting:
                            self._set_state(
                                "APPROACH_CARTESIAN_FEEDBACK_WAITING",
                                "Arm trajectory ended; waiting for a fresh post-motion "
                                f"arm/touch_tip TF before endpoint validation: "
                                f"{completion_feedback_error}",
                            )
                            return
                if execution_state == "SUCCEEDED":
                    self._approach_execute_request_id = None
                    minimum_updates = int(self.get_parameter(
                        "rolling_visual_servo_min_updates"
                    ).value)
                    if (
                        self._approach_plan is not None
                        and self._approach_plan.get("control_policy")
                        == "ROLLING_VISUAL_SERVO_10HZ_TRACKER_20HZ_ACTUATOR"
                        and rolling_updates < minimum_updates
                    ):
                        self._pause_automatic_approach(
                            "APPROACH_ROLLING_CONTROL_NOT_PROVEN",
                            f"visual-servo stopped after only {rolling_updates} updates; "
                            f"at least {minimum_updates} are required",
                        )
                        return
                    if rolling_stop_reason == "PRESS_CONFIRMATION_READY":
                        self._automatic_approach = False
                        self._rolling_servo_phase = "WAITING_PRESS_CONFIRMATION"
                        self._set_state(
                            "PRESS_CONFIRMATION_REQUIRED",
                            "Continuous FAR→NEAR tracking reached stable fixed-tip "
                            "alignment; Arm is HOLDING. Inspect the live image and "
                            "explicitly confirm PRESS",
                        )
                        return
                    if rolling_stop_reason == "TOUCH_SUCCESS_CONFIRMED":
                        evidence = self._pending_touch_success_evidence
                        if evidence is None:
                            self._pause_automatic_approach(
                                "TOUCH_SUCCESS_EVIDENCE_MISSING",
                                "STOP/HOLD succeeded but pending appearance evidence is missing",
                            )
                            return
                        self._rolling_servo_phase = "TOUCH_COMPLETE"
                        self._complete_touch_and_return_home(evidence)
                        return
                    if rolling_stop_reason == "MAX_PRESS_TRAVEL_REACHED":
                        self._automatic_approach = False
                        self._rolling_servo_phase = "MAX_PRESS_TRAVEL_REACHED"
                        self._set_state(
                            "APPROACH_MAX_TRAVEL_REACHED",
                            "Continuous PRESS reached its configured maximum travel "
                            "without appearance-change success; Arm is HOLDING",
                        )
                        return
                    if rolling_stop_reason == "CONTINUOUS_APPROACH_TIMEOUT":
                        self._pause_automatic_approach(
                            "AUTO_APPROACH_TIMEOUT",
                            "Continuous visual-servo session reached its safety timeout; "
                            "Arm is HOLDING",
                        )
                        return
                    if continuous_terminal:
                        self._pause_automatic_approach(
                            "APPROACH_VISUAL_SERVO_STOPPED",
                            "Continuous visual-servo stopped safely and the Arm is "
                            f"HOLDING: {rolling_stop_reason}",
                        )
                        return
                    cartesian_feedback = self._record_completed_motor_step(
                        execution,
                        completion_transform=completion_transform,
                        completion_feedback_error=completion_feedback_error,
                    )
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
                        if str((self._approach_plan or {}).get("step_kind", "")) == (
                            "ALIGNED_PRESS_FORWARD_WITH_FIXED_TIP_YZ"
                        ):
                            self._schedule_press_recovery(reason)
                            return
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
                        current_error = float(
                            acceptance.get("waypoint_error_m", math.inf)
                        )
                        previous_error = self._cartesian_correction_previous_error_m
                        if previous_error is not None:
                            minimum_improvement = max(
                                float(self.get_parameter(
                                    "approach_cartesian_correction_min_improvement_m"
                                ).value),
                                previous_error * float(self.get_parameter(
                                    "approach_cartesian_correction_min_improvement_ratio"
                                ).value),
                            )
                            if previous_error - current_error < minimum_improvement:
                                # Repeating the same joint-space correction is
                                # counterproductive once its measured Cartesian
                                # residual grows. Return to the outer visual
                                # loop from the real endpoint instead of
                                # freezing Touch at TIP ALIGN.
                                if self._arm_target is not None:
                                    self._automatic_tracking_reference = dict(
                                        self._arm_target
                                    )
                                self._cartesian_correction_retries = 0
                                self._cartesian_correction_previous_error_m = None
                                self._automatic_approach_steps += 1
                                self._clear_arm_observation(preserve_tracker=True)
                                self._start_automatic_settle_observation()
                                self._set_state(
                                    "AUTO_APPROACH_SETTLING",
                                    "Same-waypoint correction stopped converging "
                                    f"({previous_error * 1000.0:.1f}→"
                                    f"{current_error * 1000.0:.1f}mm); "
                                    "re-observing from the measured endpoint",
                                )
                                return
                        self._cartesian_correction_previous_error_m = current_error
                        self._cartesian_correction_retries += 1
                        if self._automatic_approach:
                            self._automatic_approach_steps += 1
                        if self._request_cartesian_waypoint_correction(
                            cartesian_feedback, reason
                        ):
                            return
                        self._pause_automatic_approach(
                            "APPROACH_CARTESIAN_NOT_REACHED",
                            "Measured endpoint was correctable, but a bounded "
                            "same-waypoint correction could not be generated",
                        )
                        return
                    else:
                        self._cartesian_correction_retries = 0
                        self._cartesian_correction_previous_error_m = None
                    tracking_evidence = (
                        {} if self._approach_plan is None else
                        self._approach_plan.get("motion_tracking_evidence", {})
                    )
                    if not bool(tracking_evidence.get("continuous", False)):
                        self._pause_automatic_approach(
                            "APPROACH_TRACKING_NOT_CONTINUOUS",
                            "Arm endpoint was accepted, but tracker coverage during "
                            "motion was not proven; no next trajectory will be sent: "
                            + str(tracking_evidence.get(
                                "reason", "missing motion tracking evidence"
                            )),
                        )
                        return
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
                    self._clear_arm_observation(preserve_tracker=True)
                    if self._automatic_approach:
                        self._start_automatic_settle_observation()
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
                    cartesian_feedback = self._record_completed_motor_step(
                        execution,
                        completion_transform=completion_transform,
                        completion_feedback_error=completion_feedback_error,
                    )
                    acceptance = (
                        {} if cartesian_feedback is None
                        else cartesian_feedback.get("acceptance", {})
                    )
                    step_kind = str((self._approach_plan or {}).get("step_kind", ""))
                    if step_kind == "ALIGNED_PRESS_FORWARD_WITH_FIXED_TIP_YZ":
                        self._schedule_press_recovery(reason)
                        return
                    if step_kind == "PRESS_RECOVERY_RETREAT_TO_NEAR":
                        self._pause_automatic_approach(
                            "PRESS_RECOVERY_FAILED",
                            "PRESS recovery could not return to NEAR: " + reason,
                        )
                        return
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
                    mild_elbow = mild_single_motor_hold_entry_overrun(
                        reason,
                        motor="elbow_flex",
                        maximum_excess_ticks=int(self.get_parameter(
                            "approach_tip_align_elbow_mild_excess_ticks"
                        ).value),
                    )
                    if (
                        self._automatic_approach
                        and step_kind == "ALIGN_FIXED_TIP_YZ"
                        and bool(mild_elbow.get("eligible"))
                        and cartesian_feedback is not None
                        and np.asarray(cartesian_feedback.get(
                            "completed_tip_base_m", []
                        )).shape == (3,)
                    ):
                        # The motion happened and only elbow_flex missed HOLD
                        # by a few ticks.  Do not chase the old command or abort:
                        # the next visual iteration must start from measured TF.
                        completed_tip = np.asarray(
                            cartesian_feedback["completed_tip_base_m"],
                            dtype=np.float64,
                        )
                        refined = np.asarray(
                            (self._approach_plan or {}).get(
                                "refined_point_base_m", []
                            ), dtype=np.float64,
                        )
                        if refined.shape == (3,):
                            self._last_executed_x_remaining_m = float(
                                refined[0] - completed_tip[0]
                            )
                        if self._arm_target is not None:
                            self._automatic_tracking_reference = dict(
                                self._arm_target
                            )
                        self._cartesian_correction_retries = 0
                        self._cartesian_correction_previous_error_m = None
                        self._automatic_approach_steps += 1
                        self._clear_arm_observation(preserve_tracker=True)
                        self._start_automatic_settle_observation()
                        self._set_state(
                            "AUTO_APPROACH_SETTLING",
                            "TIP ALIGN elbow_flex HOLD-entry miss was mild "
                            f"({mild_elbow['excess_ticks']} ticks over); "
                            "re-observing and continuing from actual touch-tip TF",
                        )
                        return
                    if self._automatic_approach and hold_entry_only and measured_usable:
                        current_error = float(
                            acceptance.get("waypoint_error_m", math.inf)
                        )
                        previous_error = self._cartesian_correction_previous_error_m
                        minimum_improvement = (
                            0.0 if previous_error is None else max(
                                float(self.get_parameter(
                                    "approach_cartesian_correction_min_improvement_m"
                                ).value),
                                previous_error * float(self.get_parameter(
                                    "approach_cartesian_correction_min_improvement_ratio"
                                ).value),
                            )
                        )
                        maximum_retries = int(self.get_parameter(
                            "approach_cartesian_max_correction_retries"
                        ).value)
                        converging = bool(
                            previous_error is None
                            or previous_error - current_error >= minimum_improvement
                        )
                        if (
                            converging
                            and self._cartesian_correction_retries < maximum_retries
                        ):
                            self._cartesian_correction_previous_error_m = current_error
                            self._cartesian_correction_retries += 1
                            self._automatic_approach_steps += 1
                            if self._request_cartesian_waypoint_correction(
                                cartesian_feedback, reason
                            ):
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
        observed_velocities = {
            str(name): float(value)
            for name, value in zip(message.name, message.velocity)
            if (
                str(name).startswith(
                    ("shoulder_", "elbow_", "wrist_", "gripper_")
                )
                and math.isfinite(float(value))
            )
        }
        common = set(observed).intersection(self._arm_joints)
        if (
            common
            and max(abs(observed[name] - self._arm_joints[name]) for name in common) > 0.003
        ):
            self._last_mechanism_motion = time.monotonic()
        # Merge partial messages for UI/gating, but only timestamp a direct-FK
        # snapshot when all five kinematic joints belong to this one message.
        self._arm_joints.update(observed)
        self._arm_joint_velocities.update(observed_velocities)
        if all(name in observed for name in REACH_JOINT_NAMES):
            self._arm_joint_snapshot = {
                name: observed[name] for name in REACH_JOINT_NAMES
            }
            self._arm_joint_snapshot_received = time.monotonic()
            stamp = stamp_seconds(message)
            self._arm_joint_snapshot_stamp_sec = stamp if stamp > 0.0 else None

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
                self._rolling_servo_session_id = None
                self._rolling_servo_sequence = 0
                self._rolling_servo_deadline = None
                self._rolling_servo_updates_sent = 0
                self._rolling_servo_stop_reason = None
                self._rolling_servo_phase = "INACTIVE"
                self._rolling_servo_alignment_streak = 0
                self._pending_touch_success_evidence = None
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
            self._request_overlay_refresh()
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
        elif name == "confirm_press":
            self._confirm_press()
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
        self._request_overlay_refresh()
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
            self._request_overlay_refresh()
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

    def _clear_arm_observation(self, preserve_tracker: bool = False) -> None:
        with self._lock:
            if not preserve_tracker:
                self._arm_frozen = None
                self._arm_prompts.clear()
                self._arm_labels.clear()
                self._arm_mask = None
                self._arm_target = None
                self._arm_target_confirmed = False
                with self._tracker_operation_lock:
                    self._target_tracker.reset()
                self._tracker_generation += 1
                self._tracker_last_stamp = None
                self._tracker_last_result = None
                self._tracker_last_good_monotonic = None
                self._tracker_last_good_input_age_sec = 0.0
                self._tracker_inflight_started_monotonic = None
                self._tracker_inflight_image_received_monotonic = None
                self._tracker_inflight_stamp = None
                self._tracker_inflight_generation = None
                self._tracker_good_frame_stamps.clear()
                self._tracker_semantic_recovery = False
                self._tracker_resume_after_confirmation = False
            self._approach_plan = None
            self._approach_preview_request_id = None
            self._overlay_stage = "ARM"

    def _reset_automatic_approach(self) -> None:
        self._automatic_approach = False
        self._automatic_approach_started = None
        self._automatic_approach_steps = 0
        self._automatic_reacquire_due = None
        self._automatic_settle_started = None
        self._automatic_settle_deadline = None
        self._automatic_settle_samples.clear()
        self._automatic_tracking_reference = None
        self._automatic_reacquire_pending = False
        self._automatic_last_pixel_error = None
        self._automatic_nonconverging_steps = 0
        self._last_executed_x_remaining_m = None
        self._cartesian_correction_retries = 0
        self._cartesian_correction_previous_error_m = None
        self._approach_feedback_wait_request_id = None
        self._approach_feedback_wait_started = None
        self._approach_completion_observed_ros_sec = None
        self._approach_feedback_diagnostics = {}
        self._press_phase_active = False
        self._press_authorized = False
        self._press_motion_attempted = False
        self._press_recovery_pending = False
        self._approach_far_tip_x_m = None
        self._near_phase_latched = False
        self._near_tool_rotation_arm = None
        self._far_arm_acquire_due = None
        self._arm_appearance_reference = None
        self._arm_appearance_reference_id = None
        self._touch_success_change_streak = 0
        with self._tracker_operation_lock:
            self._target_tracker.reset()
        self._tracker_generation += 1
        self._tracker_last_stamp = None
        self._tracker_last_result = None
        self._tracker_last_good_monotonic = None
        self._tracker_last_good_input_age_sec = 0.0
        self._tracker_inflight_started_monotonic = None
        self._tracker_inflight_image_received_monotonic = None
        self._tracker_inflight_stamp = None
        self._tracker_inflight_generation = None
        self._tracker_good_frame_stamps.clear()
        self._tracker_last_recovery_attempt = None
        self._tracker_semantic_recovery = False
        self._tracker_resume_after_confirmation = False
        self._tracker_identity_reference = None
        self._tracker_appearance_streak = 0
        self._rolling_servo_phase = "INACTIVE"
        self._rolling_servo_alignment_streak = 0
        self._rolling_servo_start_tip_base_m = None
        self._rolling_servo_last_waypoint_base_m = None
        self._rolling_servo_tracker_gate = None
        self._pending_touch_success_evidence = None
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
        self,
        execution: dict[str, Any],
        *,
        completion_transform: Any | None = None,
        completion_feedback_error: str | None = None,
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
            if completion_transform is None:
                if completion_feedback_error:
                    raise ValueError(completion_feedback_error)
                completion_transform = self._latest_transform(
                    "base_link", "arm/touch_tip"
                )
            _, completed_tip = self._rotation_translation(
                completion_transform
            )
            commanded = np.asarray(
                (self._approach_plan or {}).get("waypoint_base_m", []),
                dtype=np.float64,
            )
            planned = np.asarray(
                (self._approach_plan or {}).get(
                    "cartesian_goal_waypoint_base_m", commanded
                ),
                dtype=np.float64,
            )
            start = np.asarray(
                (self._approach_plan or {}).get("actual_tip_base_m", []),
                dtype=np.float64,
            )
            cartesian_feedback = {
                "completed_tip_base_m": completed_tip.tolist(),
                "pre_execute_tip_base_m": jsonable(
                    (self._approach_plan or {}).get("pre_execute_tip_base_m")
                ),
                "motion_start_tip_base_m": jsonable(
                    (self._approach_plan or {}).get("motion_start_tip_base_m")
                ),
                "startup_transient_delta_base_m": jsonable(
                    (self._approach_plan or {}).get(
                        "startup_transient_delta_base_m"
                    )
                ),
                "planned_waypoint_base_m": (
                    planned.tolist() if planned.shape == (3,) else None
                ),
                "commanded_waypoint_base_m": (
                    commanded.tolist() if commanded.shape == (3,) else None
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
                "source": getattr(
                    self, "_latest_tip_feedback_source", "DYNAMIC_TF_FALLBACK"
                ),
            }
            requested_delta = (
                commanded - start
                if commanded.shape == (3,) and start.shape == (3,)
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
                "NEAR_ALIGN_IMAGE_CENTER_YZ",
            }
            if step_kind == "ALIGN_FIXED_TIP_YZ":
                maximum_error_parameter = "approach_cartesian_align_max_error_m"
            elif press_like:
                maximum_error_parameter = "approach_cartesian_press_max_error_m"
            else:
                maximum_error_parameter = "approach_cartesian_coarse_max_error_m"
            maximum_error = float(
                self.get_parameter(maximum_error_parameter).value
            )
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
            # The first measured endpoint is allowed to enter the bounded
            # correction loop even after wrong-way motion.  Whether another
            # command is safe is decided from measured residual improvement;
            # a second non-improving result is stopped before another retry.
            correction_direction_allowed = bool(
                step_kind == "ALIGN_FIXED_TIP_YZ"
                or short_request
                or progress_ratio >= 0.0
            )
            correctable = bool(
                not accepted
                and waypoint_error is not None
                and float(waypoint_error) <= correctable_maximum
                and correction_direction_allowed
            )
            cartesian_feedback["acceptance"] = {
                "accepted": accepted,
                "correctable": correctable,
                "waypoint_error_m": waypoint_error,
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
            "post_motion_tf_diagnostics": jsonable(
                self._approach_feedback_diagnostics
            ),
            "completed_at_wall_time_sec": time.time(),
        })
        self._motor_step_evidence.append(summary)
        self._active_step_motor_started = None
        self._save_motor_evidence()
        return cartesian_feedback

    def _capture_motion_tracking_evidence(
        self, execution: dict[str, Any]
    ) -> dict[str, Any]:
        """Freeze tracker coverage for the just-completed arm trajectory."""
        plan = self._approach_plan
        if plan is None:
            return {
                "continuous": False,
                "reason": "approach plan disappeared before tracker audit",
            }
        existing = plan.get("motion_tracking_evidence")
        if isinstance(existing, dict):
            return existing
        try:
            dispatch_stamp = float(plan.get("execute_dispatch_ros_sec"))
        except (TypeError, ValueError):
            dispatch_stamp = math.nan
        try:
            gateway_completion_stamp = float(execution.get("event_ros_sec"))
        except (TypeError, ValueError):
            gateway_completion_stamp = math.nan
        gateway_stamp_valid = bool(
            math.isfinite(gateway_completion_stamp)
            and gateway_completion_stamp > 0.0
        )
        completion_stamp = gateway_completion_stamp
        if not gateway_stamp_valid:
            completion_stamp = float(self.get_clock().now().nanoseconds) * 1.0e-9
        evidence = motion_tracking_evidence(
            list(self._tracker_good_frame_stamps),
            dispatch_stamp,
            completion_stamp,
            int(self.get_parameter(
                "approach_tracking_minimum_motion_frames"
            ).value),
            float(self.get_parameter(
                "approach_tracking_maximum_motion_gap_sec"
            ).value),
        )
        evidence["completion_stamp_source"] = (
            "GATEWAY_EVENT_ROS_TIME"
            if gateway_stamp_valid
            else "LOCAL_RECEIVE_ROS_TIME_FALLBACK"
        )
        plan["motion_tracking_evidence"] = evidence
        return evidence

    def _completion_transform_after_motion(
        self, execution: dict[str, Any]
    ) -> tuple[Any | None, str | None, bool]:
        """Return only fresh post-motion TF, or defer endpoint validation.

        During a real trajectory the servo-bus lock can temporarily delay the
        JointState -> robot_state_publisher chain.  tf2 Time(0) may then either
        return the pre-motion Arm pose or fail because the moving mount and Arm
        subtrees no longer have an overlapping cache interval.  Neither case
        is valid completion evidence.
        """
        request_id = str(execution.get("last_request_id", ""))
        now_monotonic = time.monotonic()
        try:
            gateway_event_ros_sec = float(execution.get("event_ros_sec"))
        except (TypeError, ValueError):
            gateway_event_ros_sec = math.nan
        if self._approach_feedback_wait_request_id != request_id:
            self._approach_feedback_wait_request_id = request_id
            self._approach_feedback_wait_started = now_monotonic
            local_observed_ros_sec = (
                float(self.get_clock().now().nanoseconds) * 1.0e-9
            )
            self._approach_completion_observed_ros_sec = (
                gateway_event_ros_sec
                if math.isfinite(gateway_event_ros_sec)
                and gateway_event_ros_sec > 0.0
                else local_observed_ros_sec
            )
            self._approach_feedback_diagnostics = {}

        maximum_age = float(
            self.get_parameter("approach_cartesian_feedback_max_age_sec").value
        )
        wait_limit = float(
            self.get_parameter("approach_cartesian_feedback_wait_sec").value
        )
        completion_slack = float(self.get_parameter(
            "approach_cartesian_completion_tf_slack_sec"
        ).value)
        reason: str | None = None
        transform = None
        try:
            transform = self._latest_arm_tip_transform()
            stamp = transform.header.stamp
            transform_stamp_sec = (
                float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
            )
            now_ros_sec = float(self.get_clock().now().nanoseconds) * 1.0e-9
            age_sec = now_ros_sec - transform_stamp_sec
            completion_observed = self._approach_completion_observed_ros_sec
            minimum_completion_stamp = (
                None if completion_observed is None else
                completion_observed - completion_slack
            )
            motion_start_stamp = float(
                (self._approach_plan or {}).get("motion_start_tf_stamp_sec", 0.0)
                or 0.0
            )
            if motion_start_stamp > 0.0:
                minimum_completion_stamp = max(
                    motion_start_stamp,
                    minimum_completion_stamp or motion_start_stamp,
                )
            self._approach_feedback_diagnostics = {
                "request_id": request_id,
                "query_ros_sec": now_ros_sec,
                "terminal_state_first_observed_ros_sec": completion_observed,
                "terminal_stamp_source": (
                    "GATEWAY_EVENT_ROS_TIME"
                    if math.isfinite(gateway_event_ros_sec)
                    and gateway_event_ros_sec > 0.0
                    else "LOCAL_RECEIVE_ROS_TIME_FALLBACK"
                ),
                "terminal_state_local_receive_ros_sec": now_ros_sec,
                "terminal_state_transport_delay_sec": (
                    None if completion_observed is None else
                    max(0.0, now_ros_sec - completion_observed)
                ),
                "motion_start_tf_stamp_sec": (
                    motion_start_stamp if motion_start_stamp > 0.0 else None
                ),
                "minimum_accepted_tf_stamp_sec": minimum_completion_stamp,
                "latest_tf_stamp_sec": transform_stamp_sec,
                "latest_tf_age_sec": age_sec,
                "maximum_tf_age_sec": maximum_age,
                "completion_tf_slack_sec": completion_slack,
                "feedback_source": getattr(
                    self, "_latest_tip_feedback_source", "UNKNOWN"
                ),
            }
            reason = post_motion_tf_rejection_reason(
                transform_stamp_sec,
                now_ros_sec,
                minimum_completion_stamp,
                maximum_age,
            )
            if reason is not None:
                transform = None
        except (TypeError, ValueError, TransformException) as exc:
            reason = str(exc)
            self._approach_feedback_diagnostics = {
                "request_id": request_id,
                "query_ros_sec": (
                    float(self.get_clock().now().nanoseconds) * 1.0e-9
                ),
                "terminal_state_first_observed_ros_sec": (
                    self._approach_completion_observed_ros_sec
                ),
                "lookup_error": reason,
            }

        if transform is not None:
            self._approach_feedback_wait_request_id = None
            self._approach_feedback_wait_started = None
            self._approach_completion_observed_ros_sec = None
            return transform, None, False

        started = self._approach_feedback_wait_started
        elapsed = 0.0 if started is None else now_monotonic - started
        if elapsed < wait_limit:
            return None, reason or "fresh Arm TF is not available yet", True

        self._approach_feedback_wait_request_id = None
        self._approach_feedback_wait_started = None
        self._approach_completion_observed_ros_sec = None
        return (
            None,
            f"fresh post-motion Cartesian feedback did not arrive within "
            f"{wait_limit:.1f}s: {reason or 'unknown TF error'}",
            False,
        )

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
            self._automatic_settle_started = None
            self._automatic_settle_deadline = None
            self._automatic_settle_samples.clear()
            self._automatic_reacquire_pending = False
        self._set_state(state, reason)

    def _schedule_press_recovery(self, reason: str) -> None:
        """Leave PRESS and return through the camera-preserving NEAR loop.

        A failed operator-authorized PRESS must not reuse the generic
        same-waypoint correction path: that path keeps chasing the failed
        forward goal and can accumulate a different wrist-camera attitude.
        Revoke the one-shot authorization, retain the NEAR latch, retreat in
        bounded steps to the NEAR X plane, then reacquire and run the normal
        fixed-tip Y/Z alignment again.
        """
        self._press_authorized = False
        self._press_phase_active = False
        self._press_recovery_pending = True
        self._near_phase_latched = True
        self._cartesian_correction_retries = 0
        self._cartesian_correction_previous_error_m = None
        self._approach_feedback_wait_request_id = None
        self._approach_feedback_wait_started = None
        if self._arm_target is not None:
            self._automatic_tracking_reference = dict(self._arm_target)
        self._automatic_approach = True
        self._automatic_approach_started = time.monotonic()
        self._clear_arm_observation(preserve_tracker=True)
        self._start_automatic_settle_observation()
        self._set_state(
            "PRESS_RECOVERY_TO_NEAR",
            "PRESS endpoint was not accepted; authorization revoked. "
            "Returning to the retained NEAR plane before camera-preserving "
            f"tip alignment: {reason}",
        )

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
        self._request_overlay_refresh()
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

    def _arm_base_transform(self, destination: str, source: str):
        """Return the latest physical Arm mount without requiring a full TF tree."""
        direct = self._arm_mount_transform
        received = self._arm_mount_received
        if (
            direct is None
            or received is None
            or time.monotonic() - received > 1.0
        ):
            return self._latest_transform(destination, source)
        valid_pair = {destination, source} == {"base_link", "arm/base_link"}
        if not valid_pair:
            raise ValueError(
                f"unsupported Arm mount transform {destination} <- {source}"
            )
        rotation, translation = self._rotation_translation(direct)
        if destination == "arm/base_link":
            rotation = rotation.T
            translation = -(rotation @ translation)
        quaternion = self._quaternion_xyzw_from_matrix(rotation)
        result = TransformStamped()
        result.header.stamp = direct.header.stamp
        result.header.frame_id = destination
        result.child_frame_id = source
        result.transform.translation.x = float(translation[0])
        result.transform.translation.y = float(translation[1])
        result.transform.translation.z = float(translation[2])
        result.transform.rotation.x = float(quaternion[0])
        result.transform.rotation.y = float(quaternion[1])
        result.transform.rotation.z = float(quaternion[2])
        result.transform.rotation.w = float(quaternion[3])
        return result

    @staticmethod
    def _quaternion_xyzw_from_matrix(rotation: np.ndarray) -> np.ndarray:
        """Convert one proper 3x3 rotation to a normalized xyzw quaternion."""
        matrix = np.asarray(rotation, dtype=np.float64)
        trace = float(np.trace(matrix))
        if trace > 0.0:
            scale = math.sqrt(trace + 1.0) * 2.0
            quaternion = np.array([
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
                0.25 * scale,
            ])
        else:
            index = int(np.argmax(np.diag(matrix)))
            j, k = (index + 1) % 3, (index + 2) % 3
            scale = math.sqrt(max(
                1.0 + matrix[index, index] - matrix[j, j] - matrix[k, k],
                1.0e-12,
            )) * 2.0
            quaternion = np.zeros(4, dtype=np.float64)
            quaternion[index] = 0.25 * scale
            quaternion[j] = (matrix[j, index] + matrix[index, j]) / scale
            quaternion[k] = (matrix[k, index] + matrix[index, k]) / scale
            quaternion[3] = (matrix[k, j] - matrix[j, k]) / scale
        norm = float(np.linalg.norm(quaternion))
        if norm <= 1.0e-12 or not np.all(np.isfinite(quaternion)):
            raise ValueError("calibrated FK produced an invalid rotation")
        return quaternion / norm

    def _latest_arm_tip_transform(self):
        """Prefer fresh raw joint feedback plus calibrated FK over dynamic TF."""
        received = self._arm_joint_snapshot_received
        maximum_age = float(
            self.get_parameter("approach_cartesian_feedback_max_age_sec").value
        )
        if (
            received is not None
            and time.monotonic() - received <= maximum_age
            and all(name in self._arm_joint_snapshot for name in REACH_JOINT_NAMES)
        ):
            arm_base = self._arm_base_transform("base_link", "arm/base_link")
            base_rotation, base_translation = self._rotation_translation(arm_base)
            raw_positions = [
                self._arm_joint_snapshot[name] for name in REACH_JOINT_NAMES
            ]
            arm_tip = tip_transform(raw_positions)
            tip_rotation = base_rotation @ arm_tip[:3, :3]
            tip_translation = base_translation + base_rotation @ arm_tip[:3, 3]
            quaternion = self._quaternion_xyzw_from_matrix(tip_rotation)
            result = TransformStamped()
            stamp_sec = self._arm_joint_snapshot_stamp_sec
            if stamp_sec is None:
                stamp_sec = float(self.get_clock().now().nanoseconds) * 1.0e-9
            whole_seconds = math.floor(stamp_sec)
            result.header.stamp.sec = int(whole_seconds)
            result.header.stamp.nanosec = int(
                round((stamp_sec - whole_seconds) * 1.0e9)
            )
            if result.header.stamp.nanosec >= 1_000_000_000:
                result.header.stamp.sec += 1
                result.header.stamp.nanosec -= 1_000_000_000
            result.header.frame_id = "base_link"
            result.child_frame_id = "arm/touch_tip"
            result.transform.translation.x = float(tip_translation[0])
            result.transform.translation.y = float(tip_translation[1])
            result.transform.translation.z = float(tip_translation[2])
            result.transform.rotation.x = float(quaternion[0])
            result.transform.rotation.y = float(quaternion[1])
            result.transform.rotation.z = float(quaternion[2])
            result.transform.rotation.w = float(quaternion[3])
            self._latest_tip_feedback_source = (
                "DIRECT_JOINT_STATE_CALIBRATED_FK_PLUS_ARM_MOUNT_TF"
            )
            return result
        self._latest_tip_feedback_source = "DYNAMIC_TF_FALLBACK"
        return self._latest_transform("base_link", "arm/touch_tip")

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
            now = time.monotonic()
            maximum_age = float(self.get_parameter(
                "arm_camera_max_receive_age_sec"
            ).value)
            color_age = (
                math.inf if self._arm_color_received is None
                else now - self._arm_color_received
            )
            if (
                not self._arm_color
                or not self._arm_info
                or color_age > maximum_age
            ):
                info_age = (
                    math.inf if self._arm_info_received is None
                    else now - self._arm_info_received
                )
                reason = (
                    "Arm-camera RGB is missing/stale or CameraInfo is unavailable: "
                    f"RGB age={color_age:.2f}s, CameraInfo age={info_age:.2f}s, "
                    f"limit={maximum_age:.2f}s"
                )
                if automatic_reacquire:
                    self._automatic_reacquire_due = now + 0.5
                    self._set_state(
                        "AUTO_APPROACH_SETTLING",
                        reason + "; waiting for a fresh frame without motion",
                    )
                else:
                    self._set_state("BLOCKED", reason)
                return
            color = self._arm_color[-1]
        try:
            # CameraInfo is static calibration metadata. Its message stamp is
            # not required to match every RGB frame; only the RGB must be
            # fresh. Geometry uses the newest common TF rather than an image
            # stamp that may fall outside the TF buffer after motion.
            arm_frame = self._arm_frame_snapshot(color)
        except (ValueError, TransformException) as exc:
            if automatic_reacquire:
                with self._lock:
                    self._automatic_reacquire_due = time.monotonic() + 0.10
                self._set_state(
                    "AUTO_APPROACH_GEOMETRY_WAIT",
                    f"Arm-camera geometry is temporarily unavailable: {exc}; "
                    "waiting without motion",
                )
            else:
                self._set_state("BLOCKED", str(exc))
            return
        with self._lock:
            rgb = arm_frame["rgb"]
            color_stamp = float(arm_frame["stamp"])
            frame_id = str(arm_frame["frame_id"])
            camera_matrix = np.asarray(arm_frame["camera_matrix"], dtype=np.float64)
            distortion = np.asarray(arm_frame["distortion"], dtype=np.float64)
            rotation = np.asarray(
                arm_frame["base_from_camera_rotation"], dtype=np.float64
            )
            translation = np.asarray(
                arm_frame["base_from_camera_translation"], dtype=np.float64
            )
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
                **arm_frame,
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
        self._request_overlay_refresh()
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
            mask_fraction = float(len(pixels)) / float(mask.size)
            # The old score relaxation depended on a >=25% mask.  Current
            # Touch views rarely reach that occupancy and a correct partially
            # occluded target was consequently reported as lost.  Select the
            # score gate from the retained approach phase instead.
            minimum_score = phase_sam_minimum_score(
                operator_handoff_complete=bool(self._arm_handoff_complete),
                near_phase_latched=bool(self._near_phase_latched),
                press_phase_active=bool(self._press_phase_active),
                initial_score=float(self.get_parameter("arm_sam_minimum_score").value),
                far_tracking_score=float(self.get_parameter(
                    "arm_sam_far_tracking_minimum_score"
                ).value),
                near_tracking_score=float(self.get_parameter(
                    "arm_sam_near_tracking_minimum_score"
                ).value),
                press_score=float(self.get_parameter(
                    "arm_sam_press_minimum_score"
                ).value),
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
                    self._request_overlay_refresh()
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
                self._request_overlay_refresh()
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
            self._request_overlay_refresh()
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
                min_ratio, max_ratio = phase_tracking_area_ratio_bounds(
                    near_phase_latched=bool(self._near_phase_latched),
                    far_minimum_ratio=float(self.get_parameter(
                        "automatic_arm_target_far_min_area_ratio"
                    ).value),
                    near_minimum_ratio=float(self.get_parameter(
                        "automatic_arm_target_near_min_area_ratio"
                    ).value),
                    maximum_ratio=float(self.get_parameter(
                        "automatic_arm_target_max_area_ratio"
                    ).value),
                    near_maximum_ratio=float(self.get_parameter(
                        "automatic_arm_target_near_max_area_ratio"
                    ).value),
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
                if self._near_phase_latched:
                    # The fixed jaw increasingly occludes the object in NEAR.
                    # Mask shrinkage is diagnostic, not loss evidence. Keep
                    # centroid continuity and excessive growth as identity
                    # checks; a large growth can indicate a target switch.
                    continuity["valid"] = bool(
                        math.isfinite(shift)
                        and shift <= max_shift
                        and math.isfinite(area_ratio)
                        and area_ratio <= max_ratio
                    )
                    continuity["area_shrink_blocking"] = False
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
            recovery = bool(self._tracker_semantic_recovery)
            self._set_state(
                "ARM_TARGET_READY",
                (
                    "Automatic SAM semantic recovery produced a candidate: "
                    if recovery else "Arm-camera SAM candidate: "
                )
                + f"score={score:.3f}, mask={len(pixels)} px; operator must "
                "confirm the same elevator button or click to correct it",
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
        try:
            # SAM recovery and KLT propagation are deliberately image-only.
            # Attach current intrinsics/TF once, immediately before metric
            # planning is authorized.
            self._refresh_arm_tracking_geometry()
        except (ValueError, TransformException) as exc:
            self._set_state(
                "ARM_TARGET_READY",
                f"Target identity is retained, but current Arm-camera geometry "
                f"is unavailable: {exc}; confirm again after TF recovers",
            )
            return
        with self._lock:
            if self._arm_mask is None or self._arm_frozen is None:
                self._set_state("BLOCKED", "Arm-camera mask/image is unavailable")
                return
            semantic_recovery = bool(self._tracker_semantic_recovery)
            resume_after_confirmation = bool(
                self._tracker_resume_after_confirmation
            )
            height, width = self._arm_frozen["rgb"].shape[:2]
            try:
                with self._tracker_operation_lock:
                    tracker_result = self._target_tracker.initialize(
                        self._arm_frozen["rgb"], self._arm_mask,
                        excluded_tip_uv=self._fixed_arm_tip_pixel(width, height),
                        preserve_identity_reference=semantic_recovery,
                    )
            except ValueError as exc:
                self._set_state(
                    "ARM_TARGET_REJECTED",
                    "Confirmed SAM region cannot start geometric tracking: " + str(exc),
                )
                return
            self._tracker_generation += 1
            self._tracker_last_result = tracker_result
            self._tracker_last_stamp = float(self._arm_frozen["stamp"])
            self._tracker_last_good_monotonic = time.monotonic()
            self._tracker_last_good_input_age_sec = 0.0
            self._tracker_good_frame_stamps.clear()
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
            self._arm_target["tracker_initialization"] = jsonable(tracker_result)
            # _refresh_arm_tracking_geometry() already recorded the metric ray
            # point and range on this exact frozen frame.
            self._arm_target_confirmed = True
            self._arm_handoff_complete = True
            self._arm_target["operator_confirmed_same_object"] = True
            self._arm_target["automatic_continuity_verified"] = False
            self._automatic_tracking_reference = dict(self._arm_target)
            self._tracker_semantic_recovery = False
            self._tracker_resume_after_confirmation = False
            self._automatic_approach = bool(
                resume_after_confirmation
                or self.get_parameter("automatic_approach_after_confirmation").value
            )
            self._automatic_approach_started = time.monotonic()
            self._automatic_approach_steps = 0
            self._automatic_reacquire_due = None
            self._automatic_last_pixel_error = None
            self._automatic_nonconverging_steps = 0
            self._touch_success_change_streak = 0
            # Do not redefine FAR after a manual re-confirmation.  FAR is a
            # physical X reference captured once for this Touch session.  The
            # previous reset made a NEAR pose become the new FAR pose, erased
            # >=35 mm of measured progress, and falsely reported that a
            # too-close target had no retreat room.
            self._save_arm_artifact()
        if not self._automatic_approach:
            self._set_state(
                "ARM_TARGET_CONFIRMED",
                "Arm-camera target confirmed; plan one bounded approach step",
            )
            return
        self._set_state(
            "ARM_TARGET_CONFIRMED",
            (
                "Recovered elevator-button identity confirmed; continuous tracking "
                "and the interrupted approach are resumed"
                if semantic_recovery else
                "Arm-camera target confirmed; KLT tracking is active and the "
                "guarded approach is authorized"
            ),
        )
        self._plan_approach_step()

    def _confirm_press(self) -> None:
        """Authorize deliberate +X PRESS after inspecting aligned NEAR view."""
        if (
            self._state != "PRESS_CONFIRMATION_REQUIRED"
            or self._approach_plan is None
            or not bool(self._approach_plan.get("press_alignment_ready"))
        ):
            self._set_state(
                "BLOCKED",
                "PRESS confirmation requires a current aligned NEAR observation",
            )
            return
        self._press_authorized = True
        self._automatic_approach = True
        self._automatic_approach_started = time.monotonic()
        self._set_state(
            "PRESS_CONFIRMED",
            "Operator confirmed PRESS; planning guarded +X motion",
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
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        histogram = cv2.calcHist(
            [hsv], [0, 1], valid.astype(np.uint8), [16, 8], [0, 180, 0, 256]
        )
        cv2.normalize(histogram, histogram, alpha=1.0, norm_type=cv2.NORM_L1)
        return {
            "median_lab": np.median(pixels, axis=0).tolist(),
            "sample_count": int(len(pixels)),
            "hsv_histogram_16x8": histogram.reshape(-1).tolist(),
        }

    def _tracked_region_appearance_change(
        self, rgb: np.ndarray, mask: np.ndarray
    ) -> dict[str, Any]:
        """Compare the propagated target region to the immutable human keyframe."""
        current = self._appearance_signature(rgb, mask)
        reference = self._arm_appearance_reference
        if reference is None:
            return {"changed": False, "reason": "NO_HUMAN_CONFIRMED_REFERENCE"}
        reference_lab = np.asarray(reference["median_lab"], dtype=np.float64)
        current_lab = np.asarray(current["median_lab"], dtype=np.float64)
        median_delta = float(np.linalg.norm(current_lab - reference_lab))
        lab_threshold = float(self.get_parameter("tracker_touch_lab_delta").value)
        lab = cv2.cvtColor(rgb, cv2.COLOR_RGB2LAB).astype(np.float32)
        changed_fraction = float(np.mean(
            np.linalg.norm(
                lab[np.asarray(mask, dtype=bool)] - reference_lab, axis=1
            ) >= lab_threshold
        ))
        fraction_threshold = float(self.get_parameter(
            "tracker_touch_changed_fraction"
        ).value)
        reference_hist = np.asarray(
            reference.get("hsv_histogram_16x8", []), dtype=np.float32
        )
        current_hist = np.asarray(
            current.get("hsv_histogram_16x8", []), dtype=np.float32
        )
        histogram_distance = (
            0.0
            if reference_hist.shape != current_hist.shape or reference_hist.size == 0
            else float(cv2.compareHist(
                reference_hist.reshape(-1, 1), current_hist.reshape(-1, 1),
                cv2.HISTCMP_BHATTACHARYYA,
            ))
        )
        histogram_threshold = float(self.get_parameter(
            "tracker_touch_histogram_distance"
        ).value)
        return {
            "changed": bool(
                changed_fraction >= fraction_threshold
                and (
                    median_delta >= lab_threshold
                    or histogram_distance >= histogram_threshold
                )
            ),
            "evidence_kind": "TRACKED_REGION_APPEARANCE_CHANGE",
            "identity_source": "FIRST_HUMAN_CONFIRMED_ARM_CAMERA_REGION",
            "median_delta_lab": median_delta,
            "lab_delta_threshold": lab_threshold,
            "changed_fraction": changed_fraction,
            "changed_fraction_threshold": fraction_threshold,
            "hsv_histogram_bhattacharyya": histogram_distance,
            "histogram_distance_threshold": histogram_threshold,
            "reference": reference,
            "current": current,
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
            self._start_automatic_settle_observation()
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
        if self._rolling_servo_session_id is not None:
            if self._pending_touch_success_evidence is not None:
                return
            self._pending_touch_success_evidence = jsonable(evidence)
            self._stop_rolling_visual_servo("TOUCH_SUCCESS_CONFIRMED")
            self._set_state(
                "TOUCH_SUCCESS_STOPPING",
                "Tracked appearance change confirmed; waiting for visual-servo "
                "STOP/HOLD acknowledgement before requesting HOME",
            )
            return
        self._automatic_approach = False
        self._automatic_reacquire_due = None
        self._automatic_reacquire_pending = False
        self._press_phase_active = False
        self._press_authorized = False
        self._pending_touch_success_evidence = None
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
            f"(Lab delta={float(evidence.get('median_delta_lab', evidence.get('delta_lab', 0.0))):.1f}); "
            "requesting guarded home preset",
        )

    def _save_arm_artifact(self) -> None:
        if self._frozen is None or self._arm_frozen is None:
            return
        geometry = arm_frame_geometry_evidence(self._arm_frozen)
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
            "geometry_ready": geometry["geometry_ready"],
            "camera_info": geometry["camera_info"],
            "d435_projected_uv": self._arm_frozen.get("d435_projected_uv"),
            "d435_projection_in_image": self._arm_frozen.get(
                "d435_projection_in_image", False
            ),
            "d435_projection_role": (
                "AUTO_PROMPT" if bool(
                    self.get_parameter("arm_projection_auto_prompt").value
                ) else "REFERENCE_ONLY_MANUAL_SELECTION_REQUIRED"
            ),
            "tf_base_link_from_arm_camera": geometry[
                "tf_base_link_from_arm_camera"
            ],
            "prompts": [
                {"u": u, "v": v, "label": label}
                for (u, v), label in zip(self._arm_prompts, self._arm_labels)
            ],
            "target": jsonable(self._arm_target),
            "continuous_tracker": {
                "last_result": jsonable(self._tracker_last_result),
                "semantic_recovery_active": self._tracker_semantic_recovery,
                "identity_reference_observation_id": (
                    self._arm_appearance_reference_id
                ),
                "appearance_change_streak": self._tracker_appearance_streak,
            },
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

    def _republish_overlay(self) -> bool:
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
                tracker = self._tracker_last_result
                if isinstance(tracker, dict):
                    current_points = tracker.get("feature_points_uv", [])
                    previous_points = tracker.get("previous_feature_points_uv", [])
                    if (
                        isinstance(current_points, list)
                        and isinstance(previous_points, list)
                    ):
                        for previous, current in zip(
                            previous_points[:80], current_points[:80]
                        ):
                            if len(previous) != 2 or len(current) != 2:
                                continue
                            p0 = tuple(int(round(value)) for value in previous)
                            p1 = tuple(int(round(value)) for value in current)
                            cv2.line(rgb, p0, p1, (40, 210, 255), 1, cv2.LINE_AA)
                    if isinstance(current_points, list):
                        for point in current_points[:80]:
                            if not isinstance(point, list) or len(point) != 2:
                                continue
                            uv = tuple(int(round(value)) for value in point)
                            cv2.circle(rgb, uv, 2, (40, 255, 80), -1, cv2.LINE_AA)
                            cv2.circle(rgb, uv, 4, (10, 80, 10), 1, cv2.LINE_AA)
                    tracker_text = (
                        f"TRACKER {tracker.get('status', '--')} / "
                        f"{tracker.get('method', '--')}  "
                        f"valid={int(tracker.get('tracked_points', 0))}  "
                        f"inlier={float(tracker.get('inlier_ratio', 0.0)):.2f}  "
                        f"{float(tracker.get('processing_ms', 0.0)):.1f} ms"
                    )
                    cv2.rectangle(
                        rgb, (5, 5),
                        (min(image_width - 5, 625), 31),
                        (15, 15, 15), thickness=-1,
                    )
                    cv2.putText(
                        rgb, tracker_text, (12, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.46, (80, 255, 120), 1,
                        cv2.LINE_AA,
                    )
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
            elif self._frozen is None:
                return False
            else:
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
        # DDS publication of a 640x480 RGB image can block under rosbag/RViz
        # backpressure. Never hold the shared state lock while publishing it.
        self._overlay_pub.publish(message)
        return True

    def _request_overlay_refresh(self) -> None:
        with self._lock:
            self._overlay_refresh_requested_count += 1
            if self._overlay_dirty:
                self._overlay_coalesced_update_count += 1
            self._overlay_dirty = True

    def _overlay_publish_tick(self) -> None:
        with self._lock:
            if not self._overlay_dirty:
                return
            if self._automatic_approach or any((
                self._reach_execute_request_id is not None,
                self._approach_execute_request_id is not None,
                self._home_execute_request_id is not None,
            )):
                # A 640x480 raw-image DDS publish blocked the Python process
                # for 4.071 s and 4.111 s in the 2026-09-18 regression bag.
                # Keep only the newest dirty snapshot while any arm action is
                # active or while the automatic stop-observe loop is chaining
                # steps. Tracking, joint feedback and safety state must never
                # wait behind diagnostic UI transport; the latest overlay is
                # emitted once automatic motion pauses or completes.
                self._overlay_deferred_for_motion_count += 1
                return
            # Claim the current snapshot. Updates arriving during a slow DDS
            # publish set this flag again and are rendered on the next tick;
            # old overlays never form an unbounded queue.
            self._overlay_dirty = False
        started = time.perf_counter()
        published = False
        error = None
        try:
            published = self._republish_overlay()
        except Exception as exc:
            # Visualization is diagnostic-only. Preserve the failure in state,
            # but never route an RViz/DDS problem into the motion error path.
            error = f"{type(exc).__name__}: {exc}"
            self.get_logger().warning("Touch overlay publish failed: " + error)
        finally:
            duration_ms = (time.perf_counter() - started) * 1000.0
            with self._lock:
                self._overlay_last_publish_duration_ms = duration_ms
                self._overlay_last_error = error
                if published:
                    self._overlay_published_count += 1

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
        except (ValueError, TransformException):
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
        except (TransformException, ValueError):
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
        except (TransformException, ValueError):
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
        with self._lock:
            has_target = self._target is not None
            motion_active = diagnostic_visualization_should_defer(
                automatic_approach=bool(self._automatic_approach),
                rolling_session_active=(
                    self._rolling_servo_session_id is not None
                ),
                execute_request_active=any((
                    self._reach_execute_request_id is not None,
                    self._approach_execute_request_id is not None,
                    self._home_execute_request_id is not None,
                )),
            )
            if has_target and motion_active:
                self._target_visualization_deferred_for_motion_count += 1
                return
        if not has_target:
            return
        started = time.perf_counter()
        self._publish_target()
        duration_ms = (time.perf_counter() - started) * 1000.0
        with self._lock:
            self._target_visualization_published_count += 1
            self._target_visualization_last_publish_duration_ms = duration_ms

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
        if self._rolling_servo_session_id is not None:
            self._stop_rolling_visual_servo("Touch Mode exit requested")
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
            self._rolling_servo_session_id = None
            self._rolling_servo_deadline = None
            self._rolling_servo_phase = "INACTIVE"
            self._pending_touch_success_evidence = None
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
            transform = self._arm_base_transform("arm/base_link", "base_link")
        except (TransformException, ValueError) as exc:
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
        if self._state not in {
            "ARM_TARGET_CONFIRMED",
            "APPROACH_GATEWAY_REJECTED",
            "PRESS_CONFIRMED",
        }:
            self._set_state("BLOCKED", "Arm-camera target is not ready for approach planning")
            return
        allowed, reason = self._arm_gate()
        if not allowed:
            if (
                self._automatic_approach
                and reason == "Arm Dashboard state is missing or stale"
            ):
                with self._lock:
                    self._automatic_reacquire_due = time.monotonic() + 0.10
                self._set_state(
                    "AUTO_APPROACH_ARM_STATE_WAIT",
                    reason + "; waiting for a fresh latest-value update without motion",
                )
            else:
                self._pause_automatic_approach("APPROACH_BLOCKED", reason)
            return
        if not all(name in self._arm_joints for name in REACH_JOINT_NAMES):
            self._pause_automatic_approach(
                "APPROACH_BLOCKED", "Fresh five-joint Arm feedback is required"
            )
            return
        try:
            actual_tip_transform = self._latest_arm_tip_transform()
            _, actual_tip = self._rotation_translation(actual_tip_transform)
            height, width = self._arm_frozen["rgb"].shape[:2]
            fixed_tip_uv = np.asarray(
                self._fixed_arm_tip_pixel(width, height), dtype=np.float64
            )
            d435_anchor = np.asarray(self._target["point_target"], dtype=np.float64)
            camera_origin = np.asarray(
                self._arm_frozen["base_from_camera_translation"], dtype=np.float64
            )
            # Prefer the target range projected along the current arm-camera
            # ray.  Euclidean distance to the uncorrected D435 anchor is an
            # overestimate whenever its lateral TF error is non-zero and made
            # the old NEAR correction overshoot.
            tracking_range = self._arm_target.get("tracking_range_m")
            range_scale = (
                float(tracking_range)
                if tracking_range is not None
                and math.isfinite(float(tracking_range))
                and float(tracking_range) > 0.02
                else float(np.linalg.norm(d435_anchor - camera_origin))
            )
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
            entering_near = bool(
                not self._near_phase_latched
                and advanced_from_far_m >= near_advance_m - 1.0e-3
            )
            if entering_near:
                self._near_phase_latched = True
                # Latch the calibrated FAR/NEAR-entry attitude once.  Every
                # subsequent image correction refers to this same attitude,
                # rather than accepting cumulative camera swivel as the new
                # reference after each stop-observe cycle.
                self._near_tool_rotation_arm = tip_transform(
                    [self._arm_joints[name] for name in REACH_JOINT_NAMES]
                )[:3, :3].copy()
            # NEAR is a one-way phase transition for this Touch session.
            # Pure Y/Z IK can couple into measured X on the real mechanism;
            # that drift must not send the state machine back to FAR.
            near_reached = self._near_phase_latched
            image_centre_uv = np.array(
                [(width - 1.0) * 0.5, (height - 1.0) * 0.5], dtype=np.float64
            )
            # FAR centres the complete object in the image.  Once measured +X
            # travel crosses the NEAR boundary, use one unambiguous alignment
            # objective: bring the fixed lower gripper tip close to the SAM
            # centroid. Mask containment is diagnostic only because jaw
            # occlusion can fragment the near-field mask.
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
            alignment_subphase = "FAR_IMAGE_CENTER"
            if near_reached:
                pixel_error_uv = np.asarray(
                    tip_alignment["pixel_error_uv"], dtype=np.float64
                )
                jacobian = np.asarray(
                    tip_alignment["image_jacobian_px_per_m"], dtype=np.float64
                )
                z_first_tolerance = float(self.get_parameter(
                    "approach_near_z_first_tolerance_px"
                ).value)
                if not math.isfinite(z_first_tolerance) or z_first_tolerance <= 0.0:
                    raise ValueError("approach_near_z_first_tolerance_px must be positive")
                if abs(float(pixel_error_uv[0])) > z_first_tolerance:
                    # First remove horizontal image error while deliberately
                    # leaving vertical error for the next stopped observation.
                    # With this camera geometry that is predominantly base Z;
                    # solving through the measured Jacobian retains coupling.
                    desired_motion = np.array([-pixel_error_uv[0], 0.0])
                    lateral_correction = np.linalg.lstsq(
                        jacobian, desired_motion, rcond=None
                    )[0]
                    alignment_subphase = "Z_FIRST_TARGET_ABOVE_TIP"
                else:
                    # Once target and tip share an image column, correct only
                    # the vertical image error (predominantly base Y).
                    desired_motion = np.array([0.0, -pixel_error_uv[1]])
                    lateral_correction = np.linalg.lstsq(
                        jacobian, desired_motion, rcond=None
                    )[0]
                    alignment_subphase = "Y_SECOND_TARGET_TO_TIP"
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
                # The pinhole Jacobian is still only a local approximation.
                # Damping prevents one mismodelled frame from commanding the
                # full correction, while repeated stop-observe cycles converge.
                near_step_limit = float(self.get_parameter(
                    "approach_near_lateral_step_max_m"
                ).value)
                if not math.isfinite(near_step_limit) or near_step_limit <= 0.0:
                    raise ValueError("approach_near_lateral_step_max_m must be positive")
                damped = lateral_error * 0.50
                damped_norm = float(np.linalg.norm(damped))
                lateral_step = damped * min(
                    1.0, near_step_limit / max(damped_norm, 1.0e-9)
                )
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
            if self._press_recovery_pending:
                near_x = self._approach_far_tip_x_m + near_advance_m
                recovery_remaining = max(0.0, float(actual_tip[0]) - near_x)
                if recovery_remaining <= 0.002:
                    # The real tip is back at NEAR. Resume the ordinary visual
                    # loop; the next branch will preserve camera attitude and
                    # align fixed tip to the mask before asking for PRESS again.
                    self._press_recovery_pending = False
                    self._automatic_last_pixel_error = None
                    self._automatic_nonconverging_steps = 0
                else:
                    recovery_step_limit = float(self.get_parameter(
                        "approach_press_recovery_step_m"
                    ).value)
                    if not math.isfinite(recovery_step_limit) or recovery_step_limit <= 0.0:
                        raise ValueError("approach_press_recovery_step_m must be positive")
                    recovery_step = min(recovery_step_limit, recovery_remaining)
                    waypoint_base = actual_tip.copy()
                    waypoint_base[0] -= recovery_step
                    approach = {
                        "waypoint_base_m": waypoint_base,
                        "remaining_before_m": remaining,
                        "remaining_after_m": remaining + recovery_step,
                        "complete": False,
                        "motion_axis_base": [-float(recovery_step), 0.0, 0.0],
                        "step_kind": "PRESS_RECOVERY_RETREAT_TO_NEAR",
                        "press_phase_latched": False,
                        "alignment_reference": "PRESERVE_CURRENT_CAMERA_ROTATION",
                        "fixed_tip_in_mask": bool(fixed_tip_in_mask),
                        "arm_mask_fraction": mask_fraction,
                        "lateral_error_before_m": lateral_error.tolist(),
                        "lateral_step_m": [0.0, 0.0],
                    }
            if self._press_recovery_pending:
                # ``approach`` was constructed above for a bounded retreat.
                pass
            elif remaining <= -max_press_past_target + 1.0e-3:
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
            elif should_retreat_for_too_close(
                too_close=too_close,
                press_alignment_ready=press_alignment_ready,
            ):
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
                    "alignment_reference": (
                        "FIXED_LOWER_TIP" if near_reached else "ARM_CAMERA_IMAGE_CENTER"
                    ),
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
                # Y/Z corrections until the fixed lower tip is sufficiently
                # close to the SAM centroid.  Do not require the tip pixel to
                # be inside the binary mask: near-field jaw occlusion can
                # shrink or fragment that mask even when alignment is valid.
                # Only then permit operator-confirmed +X PRESS.
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
                    "alignment_subphase": alignment_subphase,
                    "fixed_tip_in_mask": bool(fixed_tip_in_mask),
                    "arm_mask_fraction": mask_fraction,
                    "lateral_error_before_m": lateral_error.tolist(),
                    "lateral_step_m": lateral_step.tolist(),
                }
            else:
                # Revalidate centroid alignment before every deliberate 25 mm
                # +X press step; binary-mask containment is diagnostic only.
                if not self._press_authorized:
                    self._press_phase_active = False
                    # Operator review time must not consume the automatic-loop
                    # timeout, and no background tick may send a motion.
                    self._automatic_approach = False
                    self._automatic_reacquire_due = None
                    self._approach_plan = {
                        **tip_alignment,
                        "step_kind": "PRESS_CONFIRMATION_REQUIRED",
                        "press_alignment_ready": True,
                        "fixed_tip_uv": fixed_tip_uv.tolist(),
                        "fixed_tip_in_mask": bool(fixed_tip_in_mask),
                        "arm_mask_fraction": mask_fraction,
                        "too_close_mask_fraction_threshold": too_close_mask_fraction,
                        "too_close": too_close,
                        "advanced_from_far_m": advanced_from_far_m,
                        "near_advance_distance_m": near_advance_m,
                        "near_reached": True,
                        "near_phase_latched": True,
                        "alignment_error_px": pixel_error_norm,
                        "execution_enabled": False,
                    }
                    self._publish_target()
                    self._set_state(
                        "PRESS_CONFIRMATION_REQUIRED",
                        "NEAR alignment is ready; inspect the Arm Camera image "
                        "and explicitly confirm before PRESS",
                    )
                    return
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
                    "fixed_tip_in_mask": bool(fixed_tip_in_mask),
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
            if approach["step_kind"] == "ALIGN_FIXED_TIP_YZ":
                # Preserve the visual waypoint as the acceptance target while
                # biasing the command against the repeatable local real-arm
                # under-travel measured in Touch test evidence.  The outer
                # vision loop must not advance until measured TF reaches the
                # unmodified visual goal.
                visual_waypoint = np.asarray(
                    approach["waypoint_base_m"], dtype=np.float64
                )
                desired_delta = visual_waypoint - actual_tip
                z_first = bool(
                    approach.get("alignment_subphase")
                    == "Z_FIRST_TARGET_ABOVE_TIP"
                )
                z_gain = float(self.get_parameter(
                    "approach_near_z_first_feedforward_z_gain"
                    if z_first else "approach_near_feedforward_z_gain"
                ).value)
                z_bias = float(self.get_parameter(
                    "approach_near_z_first_feedforward_z_bias_m"
                    if z_first else "approach_near_feedforward_z_up_bias_m"
                ).value)
                command_max = float(self.get_parameter(
                    "approach_near_z_first_command_max_m"
                    if z_first else "approach_near_feedforward_command_max_m"
                ).value)
                compensation = compensated_cartesian_delta(
                    desired_delta,
                    x_hold_bias_m=float(self.get_parameter(
                        "approach_near_feedforward_x_hold_m"
                    ).value),
                    y_gain=float(self.get_parameter(
                        "approach_near_feedforward_y_gain"
                    ).value),
                    z_gain=z_gain,
                    z_up_bias_m=z_bias,
                    maximum_norm_m=command_max,
                )
                commanded_delta = np.asarray(
                    compensation["command_delta_base_m"], dtype=np.float64
                )
                approach["cartesian_goal_waypoint_base_m"] = visual_waypoint.tolist()
                approach["waypoint_base_m"] = (
                    actual_tip + commanded_delta
                ).tolist()
                approach["motion_axis_base"] = commanded_delta.tolist()
                approach["cartesian_feedforward"] = {
                    **compensation,
                    "model": "LOCAL_TOUCH_NEAR_V1",
                    "alignment_subphase": approach.get("alignment_subphase"),
                    "evidence": (
                        "2026-09-06 latest Z_FIRST inverse regression: "
                        "achieved_Z ~= 0.372 * commanded_Z - 0.00924"
                        if z_first else
                        "Y_SECOND uses no fixed Z bias after image-column alignment"
                    ),
                }
            elif approach["step_kind"] == "COARSE_CENTER_AND_FORWARD_TO_NEAR":
                # The latest real evidence shows good X/Y tracking but a
                # repeatable 21--25 mm downward Z endpoint error in all three
                # FAR steps.  Keep the visual waypoint as the acceptance goal
                # and bias only the commanded Z upward; the next stopped TF
                # observation still decides whether the goal was reached.
                visual_waypoint = np.asarray(
                    approach["waypoint_base_m"], dtype=np.float64
                )
                z_bias = float(self.get_parameter(
                    "approach_far_feedforward_z_up_bias_m"
                ).value)
                if not math.isfinite(z_bias) or z_bias < 0.0:
                    raise ValueError(
                        "approach_far_feedforward_z_up_bias_m must be finite and non-negative"
                    )
                commanded_waypoint = visual_waypoint.copy()
                commanded_waypoint[2] += z_bias
                approach["cartesian_goal_waypoint_base_m"] = visual_waypoint.tolist()
                approach["waypoint_base_m"] = commanded_waypoint.tolist()
                approach["motion_axis_base"] = (
                    commanded_waypoint - actual_tip
                ).tolist()
                approach["cartesian_feedforward"] = {
                    "model": "LOCAL_TOUCH_FAR_Z_V1",
                    "desired_delta_base_m": (
                        visual_waypoint - actual_tip
                    ).tolist(),
                    "command_delta_base_m": (
                        commanded_waypoint - actual_tip
                    ).tolist(),
                    "z_up_bias_m": z_bias,
                    "evidence": "2026-09-06 latest three FAR endpoints",
                }

            arm_from_base = self._arm_base_transform("arm/base_link", "base_link")
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
                    target_rotation=self._near_tool_rotation_arm,
                )
            elif approach["step_kind"] in {
                "ALIGNED_PRESS_FORWARD_WITH_FIXED_TIP_YZ",
                "PRESS_RECOVERY_RETREAT_TO_NEAR",
            }:
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
        operator_authorized_press = bool(
            approach["step_kind"] == "ALIGNED_PRESS_FORWARD_WITH_FIXED_TIP_YZ"
            and self._press_authorized
        )
        approach_ik_limit = float(self.get_parameter(
            "approach_press_max_ik_error_m"
            if operator_authorized_press else "approach_max_ik_error_m"
        ).value)
        safe_approach_solution = bool(
            (
                int(solution.get("solver_status", 0)) > 0
                if operator_authorized_press else solution["success"]
            )
            and float(solution["position_error_m"]) <= approach_ik_limit
        )
        if approach["step_kind"] == "ALIGN_FIXED_TIP_YZ":
            # A Y/Z visual correction is not allowed to succeed by swivelling
            # the eye-in-hand camera onto the target/tip line of sight.
            safe_approach_solution = bool(
                safe_approach_solution
                and float(solution.get("rotation_error_deg", math.inf))
                <= float(self.get_parameter(
                    "approach_near_max_rotation_error_deg"
                ).value)
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
            x_hold_allowance = (
                abs(float(self.get_parameter(
                    "approach_near_feedforward_x_hold_m"
                ).value))
                if approach["step_kind"] == "ALIGN_FIXED_TIP_YZ" else 0.0
            )
            safe_approach_solution = bool(
                safe_approach_solution
                and abs(predicted_tip_base[0] - actual_tip[0])
                <= approach_ik_limit + x_hold_allowance
            )
        elif approach["step_kind"] == "MASK_TOO_CLOSE_RETREAT_SMALL":
            safe_approach_solution = bool(
                safe_approach_solution
                and predicted_tip_base[0] < actual_tip[0] - 0.0005
            )
        elif approach["step_kind"] == "PRESS_RECOVERY_RETREAT_TO_NEAR":
            safe_approach_solution = bool(
                safe_approach_solution
                and predicted_tip_base[0] < actual_tip[0] - 0.0005
                and predicted_tip_base[0]
                >= self._approach_far_tip_x_m
                + float(self.get_parameter("approach_near_advance_distance_m").value)
                - approach_ik_limit
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
            "press_entry_tolerance_px": press_entry_tolerance,
            "press_alignment_ready": press_alignment_ready,
            "fixed_tip_in_mask": bool(fixed_tip_in_mask),
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
            f"PRESS ready={press_alignment_ready} "
            f"(centroid limit={press_entry_tolerance:.1f}px; "
            f"tip-in-mask={fixed_tip_in_mask} diagnostic only); "
            f"arm mask={mask_fraction * 100.0:.1f}% / "
            f"too-close={too_close_mask_fraction * 100.0:.1f}%; "
            f"advanced from FAR={advanced_from_far_m * 1000.0:.1f}/"
            f"{near_advance_m * 1000.0:.1f}mm",
        )

    def _build_rolling_action_horizon(
        self,
        target_positions: dict[str, float],
        *,
        tracker_completed_age_sec: float,
    ) -> dict[str, Any]:
        """Build a latency-compensated 10-step replaceable action tail."""
        specs = {
            str(item.get("name")): item
            for item in self._arm_state.get("joints", [])
            if isinstance(item, dict)
        }
        names = list(target_positions)
        if not names or not all(name in specs for name in names):
            raise ValueError("action-horizon joint limits are unavailable")
        bounds = {
            name: (float(specs[name]["lower"]), float(specs[name]["upper"]))
            for name in names
        }
        current: dict[str, float] = {}
        velocities: dict[str, float] = {}
        feedback_clamped: dict[str, dict[str, float]] = {}
        for name in names:
            if name not in self._arm_joints:
                raise ValueError(f"action-horizon feedback is missing for {name}")
            lower, upper = bounds[name]
            measured = float(self._arm_joints[name])
            bounded = float(np.clip(measured, lower, upper))
            current[name] = bounded
            if bounded != measured:
                feedback_clamped[name] = {
                    "measured": measured,
                    "bounded": bounded,
                }
            velocities[name] = float(self._arm_joint_velocities.get(name, 0.0))
        completed_age = max(0.0, float(tracker_completed_age_sec))
        observation_age = min(
            2.0,
            completed_age + max(0.0, self._tracker_last_good_input_age_sec),
        )
        maximum_lead = float(self.get_parameter(
            "rolling_visual_servo_max_prediction_lead_sec"
        ).value)
        prediction_lead = min(
            maximum_lead,
            observation_age + float(self.get_parameter(
                "rolling_visual_servo_execution_lead_sec"
            ).value),
        )
        plan = receding_joint_action_horizon(
            current_positions=current,
            target_positions={
                name: float(target_positions[name]) for name in names
            },
            current_velocities=velocities,
            joint_bounds=bounds,
            steps=int(self.get_parameter(
                "rolling_visual_servo_horizon_steps"
            ).value),
            step_period_sec=float(self.get_parameter(
                "rolling_visual_servo_horizon_step_period_sec"
            ).value),
            prediction_lead_sec=prediction_lead,
            maximum_prediction_velocity_rad_s=float(self.get_parameter(
                "rolling_visual_servo_max_prediction_velocity_rad_s"
            ).value),
        )
        self._rolling_servo_horizon_plan_count += 1
        return {
            "action_horizon": plan["positions"],
            "horizon_step_period_sec": plan["step_period_sec"],
            "horizon_plan_id": f"touch-horizon-{uuid.uuid4()}",
            "observation_age_sec": observation_age,
            "prediction_lead_sec": prediction_lead,
            "diagnostics": {
                **plan,
                "feedback_clamped": feedback_clamped,
                "plan_count": self._rolling_servo_horizon_plan_count,
            },
        }

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
        if self._approach_plan.get("step_kind") == (
            "ALIGNED_PRESS_FORWARD_WITH_FIXED_TIP_YZ"
        ):
            self._press_motion_attempted = True
        duration_sec = 2.5 if self._approach_plan.get("cartesian_correction") else 1.5
        # Capture the real endpoint immediately before dispatch.  The first
        # EXECUTING feedback below is captured separately, exposing sag or a
        # kick that begins only when the next trajectory starts.
        try:
            _, pre_execute_tip = self._rotation_translation(
                self._latest_arm_tip_transform()
            )
            self._approach_plan["pre_execute_tip_base_m"] = pre_execute_tip.tolist()
            self._approach_plan["pre_execute_monotonic_sec"] = time.monotonic()
        except (TransformException, ValueError):
            self._approach_plan["pre_execute_tip_base_m"] = None
        dispatch_ros_sec = float(self.get_clock().now().nanoseconds) * 1.0e-9
        self._approach_plan["execute_dispatch_ros_sec"] = dispatch_ros_sec
        self._approach_plan["tracker_stamp_at_dispatch_sec"] = (
            None if self._tracker_last_stamp is None
            else float(self._tracker_last_stamp)
        )
        # Set the in-flight identity before publishing.  A fast Dashboard
        # response or the 10 Hz visualization timer must not observe a command
        # whose ownership has not yet been recorded locally.
        self._approach_execute_request_id = request_id
        rolling = bool(
            automatic
            and self.get_parameter("rolling_visual_servo_enabled").value
        )
        if rolling:
            session_id = f"touch-visual-servo-{uuid.uuid4()}"
            self._rolling_servo_session_id = session_id
            self._rolling_servo_sequence = 0
            self._rolling_servo_updates_sent = 0
            self._rolling_servo_horizon_plan_count = 0
            self._rolling_servo_last_tracker_stamp = self._tracker_last_stamp
            self._rolling_servo_stop_reason = None
            self._rolling_servo_tracker_gate = None
            self._rolling_servo_alignment_streak = 0
            self._rolling_servo_phase = "STARTING"
            start_tip = np.asarray(
                self._approach_plan.get("pre_execute_tip_base_m", []),
                dtype=np.float64,
            )
            self._rolling_servo_start_tip_base_m = (
                start_tip.copy() if start_tip.shape == (3,) else None
            )
            initial_waypoint = np.asarray(
                self._approach_plan.get("waypoint_base_m", []), dtype=np.float64
            )
            self._rolling_servo_last_waypoint_base_m = (
                initial_waypoint.copy()
                if initial_waypoint.shape == (3,) else None
            )
            duration_sec = float(
                self.get_parameter(
                    "rolling_visual_servo_session_timeout_sec"
                    if self.get_parameter(
                        "rolling_visual_servo_continuous_until_press"
                    ).value
                    else "rolling_visual_servo_duration_sec"
                ).value
            )
            self._rolling_servo_deadline = time.monotonic() + duration_sec
            horizon_now = time.monotonic()
            tracker_completed_age = (
                0.0
                if self._tracker_last_good_monotonic is None
                else max(
                    0.0, horizon_now - self._tracker_last_good_monotonic
                )
            )
            try:
                action_plan = self._build_rolling_action_horizon(
                    self._approach_plan["positions"],
                    tracker_completed_age_sec=tracker_completed_age,
                )
            except (KeyError, TypeError, ValueError) as exc:
                self._rolling_servo_session_id = None
                self._rolling_servo_deadline = None
                self._approach_execute_request_id = None
                self._set_state(
                    "BLOCKED",
                    f"Cannot build initial rolling action horizon: {exc}",
                )
                return
            command = {
                "mode": str(self._arm_state.get("mode", "REAL_DASHBOARD_TEST")),
                "request_id": request_id,
                "command": "visual_servo_start",
                "candidate_id": self._approach_plan["candidate_id"],
                "session_id": session_id,
                "positions": self._approach_plan["positions"],
                "tracker_stamp": self._tracker_last_stamp,
                "action_horizon": action_plan["action_horizon"],
                "horizon_step_period_sec": action_plan[
                    "horizon_step_period_sec"
                ],
                "horizon_plan_id": action_plan["horizon_plan_id"],
                "observation_age_sec": action_plan["observation_age_sec"],
                "prediction_lead_sec": action_plan["prediction_lead_sec"],
            }
            self._approach_plan["control_policy"] = (
                "DELAY_AWARE_RECEDING_10_STEP_ACTION_HORIZON"
            )
            self._approach_plan["visual_servo_session_id"] = session_id
            self._approach_plan["initial_action_horizon"] = action_plan[
                "diagnostics"
            ]
        else:
            command = {
                "mode": str(self._arm_state.get("mode", "REAL_DASHBOARD_TEST")),
                "request_id": request_id,
                "command": "execute_candidate",
                "candidate_id": self._approach_plan["candidate_id"],
                "duration_sec": duration_sec,
            }
        self._arm_command_pub.publish(String(data=json.dumps(
            command, ensure_ascii=False, allow_nan=False
        )))
        self._approach_feedback_wait_request_id = None
        self._approach_feedback_wait_started = None
        self._active_step_motor_started = time.monotonic()
        self._set_state(
            "APPROACH_EXECUTION_REQUESTED",
            ("Automatic" if automatic else "Operator-requested")
            + (
                " rolling visual-servo session sent through the Arm gateway; "
                "fresh tracked keyframes will update the active target at 10 Hz"
                if rolling else
                " guarded approach step sent through the Arm gateway; "
                "the target/contact step remains locked"
            ),
        )

    def _start_automatic_settle_observation(self) -> None:
        """Start a 10 Hz measured-TF stability gate before camera reacquisition."""
        now = time.monotonic()
        minimum = max(
            0.0,
            float(self.get_parameter("automatic_approach_min_settle_sec").value),
        )
        maximum = max(
            minimum,
            float(self.get_parameter(
                "automatic_approach_stability_max_wait_sec"
            ).value),
        )
        self._automatic_settle_started = now
        self._automatic_settle_deadline = now + maximum
        self._automatic_settle_samples.clear()
        self._automatic_reacquire_due = now + minimum

    def _automatic_settle_status(self, now: float) -> dict[str, Any]:
        """Sample the real tip and classify the recent translation stability."""
        window = max(
            0.1,
            float(self.get_parameter(
                "automatic_approach_stability_window_sec"
            ).value),
        )
        try:
            _, tip = self._rotation_translation(
                self._latest_arm_tip_transform()
            )
            self._automatic_settle_samples.append((now, tip.copy()))
        except (TransformException, ValueError):
            pass
        while (
            self._automatic_settle_samples
            and now - self._automatic_settle_samples[0][0] > window * 1.5
        ):
            self._automatic_settle_samples.popleft()
        recent = [
            (stamp, point)
            for stamp, point in self._automatic_settle_samples
            if now - stamp <= window
        ]
        if len(recent) < 3:
            return {"stable": False, "span_m": math.inf, "samples": len(recent)}
        coverage = recent[-1][0] - recent[0][0]
        points = np.asarray([point for _, point in recent], dtype=np.float64)
        axis_span = np.max(points, axis=0) - np.min(points, axis=0)
        span = float(np.linalg.norm(axis_span))
        limit = float(self.get_parameter(
            "automatic_approach_stability_span_m"
        ).value)
        return {
            "stable": bool(coverage >= window * 0.75 and span <= limit),
            "span_m": span,
            "axis_span_m": axis_span.tolist(),
            "coverage_sec": coverage,
            "samples": len(recent),
        }

    def _request_cartesian_waypoint_correction(
        self, cartesian_feedback: dict[str, Any], failure_reason: str
    ) -> bool:
        """Preview a bounded correction to the *same* Cartesian waypoint.

        No image is reacquired here.  A visual observation defines one absolute
        waypoint; joint execution must first make the measured touch-tip TF
        converge to that waypoint.  Only an accepted endpoint may advance the
        outer visual loop.
        """
        if self._approach_plan is None:
            return False
        try:
            completed = np.asarray(
                cartesian_feedback.get("completed_tip_base_m", []), dtype=np.float64
            )
            goal = np.asarray(
                self._approach_plan.get(
                    "cartesian_goal_waypoint_base_m",
                    self._approach_plan.get("waypoint_base_m", []),
                ),
                dtype=np.float64,
            )
            if completed.shape != (3,) or goal.shape != (3,):
                return False
            residual = goal - completed
            residual_norm = float(np.linalg.norm(residual))
            step_max = float(self.get_parameter(
                "approach_cartesian_correction_step_max_m"
            ).value)
            if not math.isfinite(step_max) or step_max <= 0.0:
                return False
            correction = residual * min(1.0, step_max / max(residual_norm, 1.0e-9))
            kind = str(self._approach_plan.get("step_kind", ""))
            feedforward: dict[str, Any] | None = None
            command_delta = correction
            if kind == "ALIGN_FIXED_TIP_YZ":
                z_first = bool(
                    self._approach_plan.get("alignment_subphase")
                    == "Z_FIRST_TARGET_ABOVE_TIP"
                )
                feedforward = compensated_cartesian_delta(
                    correction,
                    x_hold_bias_m=float(self.get_parameter(
                        "approach_near_feedforward_x_hold_m"
                    ).value),
                    y_gain=float(self.get_parameter(
                        "approach_near_feedforward_y_gain"
                    ).value),
                    z_gain=float(self.get_parameter(
                        "approach_near_z_first_feedforward_z_gain"
                        if z_first else "approach_near_feedforward_z_gain"
                    ).value),
                    z_up_bias_m=float(self.get_parameter(
                        "approach_near_z_first_feedforward_z_bias_m"
                        if z_first else "approach_near_feedforward_z_up_bias_m"
                    ).value),
                    maximum_norm_m=float(self.get_parameter(
                        "approach_near_z_first_command_max_m"
                        if z_first else "approach_near_feedforward_command_max_m"
                    ).value),
                )
                command_delta = np.asarray(
                    feedforward["command_delta_base_m"], dtype=np.float64
                )
            correction_waypoint = completed + command_delta

            arm_from_base = self._arm_base_transform("arm/base_link", "base_link")
            arm_rotation, arm_translation = self._rotation_translation(arm_from_base)
            waypoint_arm = arm_rotation @ correction_waypoint + arm_translation
            current = [self._arm_joints[name] for name in REACH_JOINT_NAMES]
            specs = {
                str(item.get("name")): item
                for item in self._arm_state.get("joints", [])
                if isinstance(item, dict)
            }
            if not all(name in specs for name in REACH_JOINT_NAMES):
                return False
            margin = float(self.get_parameter("ik_joint_limit_margin_rad").value)
            lower = [float(specs[name]["lower"]) + margin for name in REACH_JOINT_NAMES]
            upper = [float(specs[name]["upper"]) - margin for name in REACH_JOINT_NAMES]
            if kind == "ALIGN_FIXED_TIP_YZ":
                solution = solve_near_tip_alignment(
                    current, waypoint_arm, lower, upper,
                    target_rotation=self._near_tool_rotation_arm,
                )
            elif kind == "ALIGNED_PRESS_FORWARD_WITH_FIXED_TIP_YZ":
                solution = solve_near_forward_press(current, waypoint_arm, lower, upper)
            else:
                axis_arm = arm_rotation @ np.array([1.0, 0.0, 0.0])
                axis_arm /= max(float(np.linalg.norm(axis_arm)), 1.0e-12)
                solution = solve_pre_touch(
                    current, waypoint_arm, axis_arm, lower, upper,
                    locked_reference_id=(
                        str(self._reach_plan["prepare_reference_id"])
                        if self._reach_plan is not None else None
                    ),
                )
            ik_limit = float(self.get_parameter("approach_max_ik_error_m").value)
            if not solution.get("success") or float(solution["position_error_m"]) > ik_limit:
                return False
            if (
                kind == "ALIGN_FIXED_TIP_YZ"
                and float(solution.get("rotation_error_deg", math.inf))
                > float(self.get_parameter(
                    "approach_near_max_rotation_error_deg"
                ).value)
            ):
                return False
            positions = {
                **solution["positions"],
                "gripper_joint": float(self.get_parameter(
                    "touch_gripper_position_rad"
                ).value),
            }
            previous_plan = dict(self._approach_plan)
            self._approach_plan = {
                **previous_plan,
                "waypoint_base_m": correction_waypoint.tolist(),
                "cartesian_goal_waypoint_base_m": goal.tolist(),
                "actual_tip_base_m": completed.tolist(),
                "waypoint_arm_base_m": waypoint_arm.tolist(),
                "motion_axis_base": command_delta.tolist(),
                "ik": solution,
                "positions": positions,
                "candidate_id": None,
                "cartesian_correction": True,
                "cartesian_correction_retry": self._cartesian_correction_retries,
                "cartesian_residual_before_m": residual.tolist(),
                "cartesian_residual_before_norm_m": residual_norm,
                "cartesian_correction_step_m": correction.tolist(),
                "cartesian_correction_command_m": command_delta.tolist(),
                "cartesian_feedforward": (
                    None if feedforward is None else {
                        **feedforward,
                        "model": "LOCAL_TOUCH_NEAR_V1",
                        "evidence": "2026-09-06 motor_evidence regression",
                    }
                ),
                "cartesian_original_failure": failure_reason,
            }
            request_id = f"touch-cartesian-correction-preview-{uuid.uuid4()}"
            self._approach_preview_request_id = request_id
            self._arm_command_pub.publish(String(data=json.dumps({
                "mode": str(self._arm_state.get("mode", "REAL_DASHBOARD_TEST")),
                "request_id": request_id,
                "command": "preview",
                "positions": positions,
            }, ensure_ascii=False, allow_nan=False)))
            self._set_state(
                "APPROACH_PREVIEW_REQUESTED",
                "Same-waypoint Cartesian correction preview requested: "
                f"residual={residual_norm * 1000.0:.1f}mm, "
                f"bounded residual={np.linalg.norm(correction) * 1000.0:.1f}mm, "
                f"compensated command={np.linalg.norm(command_delta) * 1000.0:.1f}mm, "
                f"retry={self._cartesian_correction_retries}; no new image used",
            )
            return True
        except (KeyError, TypeError, ValueError, TransformException):
            return False

    def _stop_rolling_visual_servo(self, reason: str) -> None:
        session_id = self._rolling_servo_session_id
        if session_id is None:
            return
        if self._rolling_servo_deadline is None and self._rolling_servo_stop_reason:
            return
        self._rolling_servo_sequence += 1
        self._rolling_servo_stop_reason = reason
        self._arm_command_pub.publish(String(data=json.dumps({
            "mode": str(self._arm_state.get("mode", "REAL_DASHBOARD_TEST")),
            "request_id": f"touch-visual-servo-stop-{uuid.uuid4()}",
            "command": "visual_servo_stop",
            "session_id": session_id,
            "sequence": self._rolling_servo_sequence,
            "tracker_stamp": self._tracker_last_stamp,
            "reason": reason,
        }, ensure_ascii=False, allow_nan=False)))
        # Keep the session identity until the gateway reports terminal HOLD.
        # This prevents another approach command from racing the STOP/ACK.
        self._rolling_servo_deadline = None

    def _rolling_visual_servo_tick(self, now: float) -> bool:
        """Update the executing waypoint from the newest tracked keyframe.

        Returns true whenever a rolling session owns this control tick.  All
        failure paths issue STOP; the low-level 350 ms watchdog is the second,
        independent protection if this callback stops running altogether.
        """
        session_id = self._rolling_servo_session_id
        if session_id is None:
            return False
        if self._rolling_servo_deadline is None:
            return True
        if now >= self._rolling_servo_deadline:
            self._stop_rolling_visual_servo(
                "CONTINUOUS_APPROACH_TIMEOUT"
                if self.get_parameter(
                    "rolling_visual_servo_continuous_until_press"
                ).value
                else "segment duration completed"
            )
            return True
        tracker = self._tracker_last_result or {}
        freshness_now = time.monotonic()
        tracker_age = (
            math.inf
            if self._tracker_last_good_monotonic is None
            else max(0.0, freshness_now - self._tracker_last_good_monotonic)
        )
        maximum_age = float(
            self.get_parameter("rolling_visual_servo_max_tracker_age_sec").value
        )
        with self._lock:
            inflight_started = self._tracker_inflight_started_monotonic
            inflight_received = self._tracker_inflight_image_received_monotonic
            inflight_stamp = self._tracker_inflight_stamp
        inflight_processing_age = (
            None if inflight_started is None
            else max(0.0, freshness_now - inflight_started)
        )
        inflight_input_age = (
            None if inflight_received is None
            else max(0.0, freshness_now - inflight_received)
        )
        tracker_gate = rolling_tracker_freshness_gate(
            status=str(tracker.get("status", "UNKNOWN")),
            completed_age_sec=tracker_age,
            maximum_completed_age_sec=maximum_age,
            inflight_processing_age_sec=inflight_processing_age,
            inflight_input_age_sec=inflight_input_age,
            maximum_inflight_processing_sec=float(self.get_parameter(
                "rolling_visual_servo_max_tracker_inflight_sec"
            ).value),
        )
        tracker_gate["inflight_image_stamp"] = inflight_stamp
        self._rolling_servo_tracker_gate = tracker_gate
        if tracker_gate["decision"] == "WAIT_INFLIGHT":
            # Do not refresh the actuator watchdog with a target derived from
            # the stale completed frame.  The in-flight callback gets one
            # bounded grace interval; its completion resumes updates, while a
            # processing stall still reaches STOP and the independent 350 ms
            # low-level watchdog remains the final HOLD protection.
            return True
        if tracker_gate["decision"] == "STOP":
            self._stop_rolling_visual_servo(
                f"tracking unavailable during motion: status={tracker.get('status')}, "
                f"completed_age={tracker_age:.3f}s, "
                f"inflight_age={inflight_processing_age}, "
                f"inflight_input_age={inflight_input_age}"
            )
            return True
        try:
            if self._approach_plan is None or self._arm_frozen is None or self._arm_target is None:
                raise ValueError("rolling approach geometry is unavailable")
            self._refresh_arm_tracking_geometry()
            _, actual_tip = self._rotation_translation(
                self._latest_arm_tip_transform()
            )
            if self._rolling_servo_start_tip_base_m is None:
                self._rolling_servo_start_tip_base_m = actual_tip.copy()
            height, width = self._arm_frozen["rgb"].shape[:2]
            step_kind = str(self._approach_plan.get("step_kind", ""))
            solver_kind = step_kind
            continuous = bool(self.get_parameter(
                "rolling_visual_servo_continuous_until_press"
            ).value)
            if self._approach_far_tip_x_m is None:
                self._approach_far_tip_x_m = float(actual_tip[0])
            near_distance = float(
                self.get_parameter("approach_near_advance_distance_m").value
            )
            advanced_from_far = max(
                0.0, float(actual_tip[0]) - float(self._approach_far_tip_x_m)
            )
            if continuous:
                phase = rolling_approach_phase(
                    advanced_from_far_m=advanced_from_far,
                    near_distance_m=near_distance,
                    press_authorized=bool(self._press_authorized),
                )
            else:
                phase = step_kind
            if phase == "PRESS":
                solver_kind = "ALIGNED_PRESS_FORWARD_WITH_FIXED_TIP_YZ"
                self._press_phase_active = True
                self._press_motion_attempted = True
            elif phase == "NEAR_ALIGN":
                solver_kind = "ALIGN_FIXED_TIP_YZ"
                self._near_phase_latched = True
                if self._near_tool_rotation_arm is None:
                    self._near_tool_rotation_arm = tip_transform(
                        [self._arm_joints[name] for name in REACH_JOINT_NAMES]
                    )[:3, :3].copy()
            self._rolling_servo_phase = phase
            z_drop_from_start = float(
                self._rolling_servo_start_tip_base_m[2] - actual_tip[2]
            )
            maximum_z_drop = float(self.get_parameter(
                "rolling_visual_servo_max_z_drop_m"
            ).value)
            if z_drop_from_start > maximum_z_drop:
                self._stop_rolling_visual_servo(
                    "CARTESIAN_Z_DROP_LIMIT: "
                    f"phase={phase} drop={z_drop_from_start:.4f}m "
                    f"limit={maximum_z_drop:.4f}m"
                )
                return True
            alignment_uv = (
                np.asarray(self._fixed_arm_tip_pixel(width, height), dtype=np.float64)
                if (
                    phase in {"NEAR_ALIGN", "PRESS"}
                    or (
                        not continuous
                        and step_kind != "COARSE_CENTER_AND_FORWARD_TO_NEAR"
                    )
                )
                else np.asarray([(width - 1.0) * 0.5, (height - 1.0) * 0.5])
            )
            centroid = np.asarray(
                self._arm_target.get("centroid_uv", []), dtype=np.float64
            )
            tracking_range = float(self._arm_target.get("tracking_range_m", math.nan))
            if centroid.shape != (2,) or not np.all(np.isfinite(centroid)):
                raise ValueError("tracked centroid is invalid")
            if continuous:
                # The 2026-09-18 real bag disproved the rendered-TF Jacobian
                # sign for this eye-in-hand pose: while the target moved
                # 125 px to image-right, the Jacobian requested lower base Z
                # and helped drive the real tip 44.8 mm downward.  Use the
                # previously measured real-robot pixel signs for active
                # motion.  Retain the model result only as a diagnostic so a
                # future camera calibration can be compared without silently
                # controlling the arm.
                alignment = empirical_image_center_alignment_delta(
                    tuple(centroid), tuple(alignment_uv),
                    float(self.get_parameter(
                        "rolling_visual_servo_empirical_m_per_px"
                    ).value),
                )
                alignment["mapping"] = "REAL_ROBOT_EMPIRICAL_YZ_V1"
                try:
                    if not math.isfinite(tracking_range) or tracking_range <= 0.02:
                        raise ValueError("tracked range is invalid")
                    jacobian_alignment = base_yz_pixel_alignment_delta(
                        tuple(centroid), tuple(alignment_uv),
                        np.asarray(
                            self._arm_frozen["camera_matrix"], dtype=np.float64
                        ),
                        np.asarray(
                            self._arm_frozen["distortion"], dtype=np.float64
                        ),
                        np.asarray(
                            self._arm_frozen["base_from_camera_rotation"],
                            dtype=np.float64,
                        ),
                        tracking_range,
                    )
                    alignment.update({
                        "jacobian_diagnostic_delta_base_yz_m": np.asarray(
                            jacobian_alignment["delta_base_yz_m"],
                            dtype=np.float64,
                        ).tolist(),
                        "jacobian_diagnostic_singular_values": np.asarray(
                            jacobian_alignment["jacobian_singular_values"],
                            dtype=np.float64,
                        ).tolist(),
                    })
                except (KeyError, TypeError, ValueError) as exc:
                    # This is intentionally diagnostic-only.  A stale range or
                    # singular rendered-TF Jacobian must not stop the measured
                    # real-robot mapping that owns the active control output.
                    alignment["jacobian_diagnostic_error"] = str(exc)
            else:
                if not math.isfinite(tracking_range) or tracking_range <= 0.02:
                    raise ValueError("tracked range is invalid")
                alignment = base_yz_pixel_alignment_delta(
                    tuple(centroid), tuple(alignment_uv),
                    np.asarray(
                        self._arm_frozen["camera_matrix"], dtype=np.float64
                    ),
                    np.asarray(
                        self._arm_frozen["distortion"], dtype=np.float64
                    ),
                    np.asarray(
                        self._arm_frozen["base_from_camera_rotation"],
                        dtype=np.float64,
                    ),
                    tracking_range,
                )
            mask_fraction = float(self._arm_target.get("mask_fraction", 0.0))
            too_close_limit = float(self.get_parameter(
                "approach_too_close_mask_fraction"
            ).value)
            if (
                phase == "FAR_TRACK_AND_ADVANCE"
                and mask_fraction >= too_close_limit
            ):
                self._stop_rolling_visual_servo(
                    "MASK_TOO_CLOSE_DURING_CONTINUOUS_FAR"
                )
                return True
            delta_yz = np.asarray(alignment["delta_base_yz_m"], dtype=np.float64)
            maximum_step = float(
                self.get_parameter("rolling_visual_servo_max_yz_step_m").value
            )
            delta_norm = float(np.linalg.norm(delta_yz))
            delta_yz *= min(1.0, maximum_step / max(delta_norm, 1.0e-9))
            waypoint_base = np.asarray(
                self._approach_plan["waypoint_base_m"], dtype=np.float64
            ).copy()
            previous_waypoint = (
                None
                if self._rolling_servo_last_waypoint_base_m is None
                else self._rolling_servo_last_waypoint_base_m.copy()
            )
            previous_target_error = (
                np.zeros(3, dtype=np.float64)
                if previous_waypoint is None
                else previous_waypoint - actual_tip
            )
            z_tracking_error = float(previous_target_error[2])
            z_compensation: dict[str, Any] | None = None
            if continuous:
                horizon = float(self.get_parameter(
                    "rolling_visual_servo_forward_horizon_m"
                ).value)
                if not math.isfinite(horizon) or horizon <= 0.0:
                    raise ValueError("rolling forward horizon must be positive")
                z_compensation = rolling_z_motion_compensation(
                    visual_delta_z_m=float(delta_yz[1]),
                    previous_target_error_z_m=z_tracking_error,
                    nominal_x_horizon_m=horizon,
                    phase=phase,
                    residual_gain=float(self.get_parameter(
                        "rolling_visual_servo_z_residual_gain"
                    ).value),
                    far_z_per_x_gain=float(self.get_parameter(
                        "rolling_visual_servo_far_z_per_x_gain"
                    ).value),
                    maximum_compensation_m=float(self.get_parameter(
                        "rolling_visual_servo_z_compensation_max_m"
                    ).value),
                    slowdown_start_error_m=float(self.get_parameter(
                        "rolling_visual_servo_z_slowdown_start_m"
                    ).value),
                    slowdown_full_error_m=float(self.get_parameter(
                        "rolling_visual_servo_z_slowdown_full_m"
                    ).value),
                )
                effective_horizon = float(
                    z_compensation["scaled_x_horizon_m"]
                )
                if phase == "FAR_TRACK_AND_ADVANCE":
                    waypoint_base[0], _ = rolling_forward_target_x(
                        phase=phase,
                        actual_x_m=float(actual_tip[0]),
                        far_x_m=float(self._approach_far_tip_x_m),
                        near_distance_m=near_distance,
                        horizon_m=effective_horizon,
                    )
                    self._rolling_servo_alignment_streak = 0
                elif phase == "NEAR_ALIGN":
                    waypoint_base[0] = float(actual_tip[0])
                    tolerance = float(self.get_parameter(
                        "approach_press_entry_tolerance_px"
                    ).value)
                    aligned = float(alignment["pixel_error_norm"]) <= tolerance
                    self._rolling_servo_alignment_streak = (
                        self._rolling_servo_alignment_streak + 1 if aligned else 0
                    )
                    required = int(self.get_parameter(
                        "rolling_visual_servo_alignment_frames"
                    ).value)
                    if self._rolling_servo_alignment_streak >= required:
                        self._approach_plan.update({
                            **alignment,
                            "step_kind": "PRESS_CONFIRMATION_REQUIRED",
                            "press_alignment_ready": True,
                            "execution_enabled": False,
                            "advanced_from_far_m": advanced_from_far,
                        })
                        self._stop_rolling_visual_servo(
                            "PRESS_CONFIRMATION_READY"
                        )
                        return True
                elif phase == "PRESS":
                    refined = np.asarray(
                        self._approach_plan.get("refined_point_base_m", []),
                        dtype=np.float64,
                    )
                    if refined.shape != (3,):
                        raise ValueError("PRESS target plane is unavailable")
                    waypoint_base[0], press_complete = rolling_forward_target_x(
                        phase=phase,
                        actual_x_m=float(actual_tip[0]),
                        far_x_m=float(self._approach_far_tip_x_m),
                        near_distance_m=near_distance,
                        horizon_m=effective_horizon,
                        target_plane_x_m=float(refined[0]),
                        maximum_press_past_target_m=float(self.get_parameter(
                            "approach_max_press_past_target_m"
                        ).value),
                    )
                    if press_complete:
                        self._stop_rolling_visual_servo(
                            "MAX_PRESS_TRAVEL_REACHED"
                        )
                        return True
            # Y/Z are always closed around the newest measured tip, so the
            # correction is bounded per tick and never accumulated blindly.
            waypoint_base[1] = actual_tip[1] + delta_yz[0]
            waypoint_base[2] = actual_tip[2] + (
                float(z_compensation["command_delta_z_m"])
                if z_compensation is not None else float(delta_yz[1])
            )
            arm_from_base = self._arm_base_transform("arm/base_link", "base_link")
            arm_rotation, arm_translation = self._rotation_translation(arm_from_base)
            waypoint_arm = arm_rotation @ waypoint_base + arm_translation
            current = [self._arm_joints[name] for name in REACH_JOINT_NAMES]
            specs = {
                str(item.get("name")): item
                for item in self._arm_state.get("joints", [])
                if isinstance(item, dict)
            }
            if not all(name in specs for name in REACH_JOINT_NAMES):
                raise ValueError("effective joint limits are unavailable")
            margin = float(self.get_parameter("ik_joint_limit_margin_rad").value)
            lower = [float(specs[name]["lower"]) + margin for name in REACH_JOINT_NAMES]
            upper = [float(specs[name]["upper"]) - margin for name in REACH_JOINT_NAMES]
            if solver_kind == "ALIGN_FIXED_TIP_YZ":
                solution = solve_near_tip_alignment(
                    current, waypoint_arm, lower, upper,
                    target_rotation=self._near_tool_rotation_arm,
                )
            elif solver_kind in {
                "ALIGNED_PRESS_FORWARD_WITH_FIXED_TIP_YZ",
                "PRESS_RECOVERY_RETREAT_TO_NEAR",
            }:
                solution = solve_near_forward_press(current, waypoint_arm, lower, upper)
            else:
                axis_arm = arm_rotation @ np.array([1.0, 0.0, 0.0])
                axis_arm /= max(float(np.linalg.norm(axis_arm)), 1.0e-12)
                solution = solve_pre_touch(
                    current, waypoint_arm, axis_arm, lower, upper,
                    locked_reference_id=(
                        str(self._reach_plan["prepare_reference_id"])
                        if self._reach_plan is not None else None
                    ),
                )
            error_limit = float(self.get_parameter("approach_max_ik_error_m").value)
            if not bool(solution.get("success")) or float(
                solution.get("position_error_m", math.inf)
            ) > error_limit:
                raise ValueError(
                    f"rolling IK rejected: error={solution.get('position_error_m')}"
                )
            positions = {
                **solution["positions"],
                "gripper_joint": float(
                    self.get_parameter("touch_gripper_position_rad").value
                ),
            }
            action_plan = self._build_rolling_action_horizon(
                positions,
                tracker_completed_age_sec=tracker_age,
            )
            self._rolling_servo_sequence += 1
            self._arm_command_pub.publish(String(data=json.dumps({
                "mode": str(self._arm_state.get("mode", "REAL_DASHBOARD_TEST")),
                "request_id": f"touch-visual-servo-update-{uuid.uuid4()}",
                "command": "visual_servo_update",
                "session_id": session_id,
                "sequence": self._rolling_servo_sequence,
                "positions": positions,
                "tracker_stamp": self._tracker_last_stamp,
                "action_horizon": action_plan["action_horizon"],
                "horizon_step_period_sec": action_plan[
                    "horizon_step_period_sec"
                ],
                "horizon_plan_id": action_plan["horizon_plan_id"],
                "observation_age_sec": action_plan["observation_age_sec"],
                "prediction_lead_sec": action_plan["prediction_lead_sec"],
            }, ensure_ascii=False, allow_nan=False)))
            self._rolling_servo_last_waypoint_base_m = waypoint_base.copy()
            self._rolling_servo_updates_sent += 1
            self._rolling_servo_update_sent_this_tick = True
            if self._approach_plan is not None:
                self._approach_plan["rolling_visual_servo"] = {
                    "session_id": session_id,
                    "phase": phase,
                    "sequence": self._rolling_servo_sequence,
                    "updates_sent": self._rolling_servo_updates_sent,
                    "tracker_stamp": self._tracker_last_stamp,
                    "tracker_age_sec": tracker_age,
                    "new_tracker_frame": (
                        self._tracker_last_stamp != self._rolling_servo_last_tracker_stamp
                    ),
                    "centroid_uv": centroid.tolist(),
                    "pixel_error_uv": alignment["pixel_error_uv"],
                    "alignment_mapping": alignment.get(
                        "mapping", "DYNAMIC_TF_IMAGE_JACOBIAN"
                    ),
                    "unbounded_delta_base_yz_m": np.asarray(
                        alignment["delta_base_yz_m"], dtype=np.float64
                    ).tolist(),
                    "bounded_delta_base_yz_m": delta_yz.tolist(),
                    "jacobian_diagnostic_delta_base_yz_m": alignment.get(
                        "jacobian_diagnostic_delta_base_yz_m"
                    ),
                    "jacobian_diagnostic_error": alignment.get(
                        "jacobian_diagnostic_error"
                    ),
                    "actual_tip_base_m": actual_tip.tolist(),
                    "previous_waypoint_base_m": (
                        None if previous_waypoint is None
                        else previous_waypoint.tolist()
                    ),
                    "previous_target_error_base_m": previous_target_error.tolist(),
                    "z_tracking_error_m": z_tracking_error,
                    "z_drop_from_session_start_m": z_drop_from_start,
                    "z_compensation": z_compensation,
                    "advanced_from_far_m": advanced_from_far,
                    "waypoint_base_m": waypoint_base.tolist(),
                    "new_target_error_base_m": (
                        waypoint_base - actual_tip
                    ).tolist(),
                    "positions": positions,
                    "action_horizon": action_plan["diagnostics"],
                }
            self._rolling_servo_last_tracker_stamp = self._tracker_last_stamp
        except (KeyError, TypeError, ValueError, TransformException) as exc:
            self._stop_rolling_visual_servo(f"rolling correction failed: {exc}")
        return True

    def _instrumented_automatic_approach_tick(self) -> None:
        """Measure every active 10 Hz control tick without changing its decisions."""
        started = time.monotonic()
        self._rolling_servo_update_sent_this_tick = False
        with self._lock:
            initial_lock_acquired = time.monotonic()
            active_before = self._active
            automatic_before = self._automatic_approach
            state_before = self._state
            request_before = self._approach_execute_request_id
        error = None
        try:
            self._automatic_approach_tick()
        except Exception as exc:
            error = f"{type(exc).__name__}: {exc}"
            raise
        finally:
            completed = time.monotonic()
            with self._lock:
                active_after = self._active
                automatic_after = self._automatic_approach
                state_after = self._state
                request_after = self._approach_execute_request_id
                tracker_result = (
                    None if self._tracker_last_result is None
                    else dict(self._tracker_last_result)
                )
                tracker_stamp = self._tracker_last_stamp
                tracker_age_ms = (
                    None if self._tracker_last_good_monotonic is None
                    else max(
                        0.0, completed - self._tracker_last_good_monotonic
                    ) * 1000.0
                )
                execution = dict(self._arm_state.get("execution", {}))
                rolling_control = (
                    None
                    if self._approach_plan is None
                    else self._approach_plan.get("rolling_visual_servo")
                )
                rolling_tracker_gate = (
                    None if self._rolling_servo_tracker_gate is None
                    else dict(self._rolling_servo_tracker_gate)
                )
                self._control_tick_sequence += 1
                sequence = self._control_tick_sequence
            # Outside Touch mode this timer is intentionally silent.  During
            # Touch it records even idle/wait cycles so a missing tick is
            # distinguishable from a deliberate no-command decision.
            if active_before or active_after:
                self._queue_debug_payload("control", {
                    "sequence": sequence,
                    "callback_started_monotonic_sec": started,
                    "callback_completed_monotonic_sec": completed,
                    "callback_duration_ms": (completed - started) * 1000.0,
                    "initial_lock_wait_ms": (
                        initial_lock_acquired - started
                    ) * 1000.0,
                    "touch_state_before": state_before,
                    "touch_state_after": state_after,
                    "automatic_approach_before": automatic_before,
                    "automatic_approach_after": automatic_after,
                    "approach_request_id_before": request_before,
                    "approach_request_id_after": request_after,
                    "guarded_step_dispatched": bool(
                        request_after is not None and request_after != request_before
                    ),
                    "tracker_input_stamp": tracker_stamp,
                    "tracker_age_ms": tracker_age_ms,
                    "tracker_result": tracker_result,
                    "gateway_execution": execution,
                    "exception": error,
                    "rolling_target_control_implemented": bool(
                        self.get_parameter("rolling_visual_servo_enabled").value
                    ),
                    "active_goal_target_updated_this_tick": (
                        self._rolling_servo_update_sent_this_tick
                    ),
                    "rolling_visual_servo_session_id": self._rolling_servo_session_id,
                    "rolling_visual_servo_phase": self._rolling_servo_phase,
                    "rolling_visual_servo_sequence": self._rolling_servo_sequence,
                    "rolling_visual_servo_updates_sent": self._rolling_servo_updates_sent,
                    "rolling_action_horizon_plan_count": (
                        self._rolling_servo_horizon_plan_count
                    ),
                    "rolling_visual_servo_stop_reason": self._rolling_servo_stop_reason,
                    "rolling_tracker_gate": rolling_tracker_gate,
                    "rolling_control": rolling_control,
                    "control_policy": (
                        "DELAY_AWARE_RECEDING_10_STEP_ACTION_HORIZON"
                        if self.get_parameter("rolling_visual_servo_enabled").value
                        else "STOP_OBSERVE_REPLAN_GUARDED_STEPS"
                    ),
                })

    def _automatic_approach_tick(self) -> None:
        now = time.monotonic()
        with self._lock:
            if not self._automatic_approach:
                return
            started = self._automatic_approach_started
            steps = self._automatic_approach_steps
            due = self._automatic_reacquire_due
            state = self._state
        if self._rolling_visual_servo_tick(now):
            return
        maximum_steps = int(
            self.get_parameter("automatic_approach_max_steps").value
        )
        timeout_sec = float(
            self.get_parameter("automatic_approach_timeout_sec").value
        )
        if started is None or now - started > timeout_sec:
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
                "AUTO_APPROACH_GEOMETRY_WAIT",
                "AUTO_APPROACH_ARM_STATE_WAIT",
            }
            and due is not None
            and now >= due
        ):
            with self._lock:
                settle_started = self._automatic_settle_started
                settle_deadline = self._automatic_settle_deadline
                stability = (
                    None
                    if settle_started is None
                    else self._automatic_settle_status(now)
                )
                if stability is not None and not bool(stability["stable"]):
                    if settle_deadline is None or now < settle_deadline:
                        self._automatic_reacquire_due = now + 0.10
                        return
                    span = float(stability["span_m"])
                    hard_span = float(self.get_parameter(
                        "automatic_approach_stability_hard_span_m"
                    ).value)
                    continue_when_unstable = bool(self.get_parameter(
                        "automatic_approach_continue_when_unstable"
                    ).value)
                    if not math.isfinite(span) and continue_when_unstable:
                        # No trustworthy measured pose exists yet. Keep the
                        # same automatic step alive and sample again; never
                        # invent a pose or send blind motion, and never turn a
                        # transient TF gap into ARM_NOT_STABLE.
                        self._automatic_reacquire_due = now + 0.10
                        return
                    if not math.isfinite(span):
                        self._automatic_reacquire_due = None
                        self._automatic_settle_started = None
                        self._automatic_settle_deadline = None
                        self._automatic_settle_samples.clear()
                        failure = (
                            "Touch-tip stability cannot be measured because "
                            "no finite TF span is available"
                        )
                    elif span > hard_span and not continue_when_unstable:
                        self._automatic_reacquire_due = None
                        self._automatic_settle_started = None
                        self._automatic_settle_deadline = None
                        self._automatic_settle_samples.clear()
                        failure = (
                            "Touch-tip did not become stable before visual "
                            f"reacquisition: span={span * 1000.0:.1f}mm, "
                            f"hard limit={hard_span * 1000.0:.1f}mm; "
                            "no further motion sent"
                        )
                    else:
                        if span > hard_span:
                            self.get_logger().warning(
                                "Touch-tip remains unstable after the bounded "
                                f"wait: span={span * 1000.0:.1f}mm exceeds "
                                f"{hard_span * 1000.0:.1f}mm; supervised "
                                "try-anyway mode is active, so reacquiring from "
                                "the latest measured pose"
                            )
                        else:
                            # Bounded residual motion is preferable to another
                            # unconditional long idle.  The next plan still
                            # starts from freshly measured TF.
                            self.get_logger().warning(
                                "Touch-tip settle reached maximum wait with "
                                f"{span * 1000.0:.1f}mm span; reacquiring from reality"
                            )
                        failure = None
                else:
                    failure = None
                # Claim this due event before starting asynchronous SAM work.
                self._automatic_reacquire_due = None
                self._automatic_settle_started = None
                self._automatic_settle_deadline = None
                self._automatic_settle_samples.clear()
            if failure is not None:
                self._pause_automatic_approach("ARM_NOT_STABLE", failure)
                return
            with self._lock:
                tracker_result = (
                    None if self._tracker_last_result is None
                    else dict(self._tracker_last_result)
                )
                tracker_age = (
                    math.inf if self._tracker_last_good_monotonic is None
                    else now - self._tracker_last_good_monotonic
                )
                recovery_active = self._tracker_semantic_recovery
                tracker_ready = bool(
                    tracker_result is not None
                    and tracker_result.get("status") in {
                        "INITIALIZED", "TRACKING", "RECOVERED"
                    }
                    and tracker_age <= float(self.get_parameter(
                        "arm_camera_max_receive_age_sec"
                    ).value)
                    and self._arm_target is not None
                    and self._arm_mask is not None
                    and self._arm_frozen is not None
                )
                if not tracker_ready:
                    if not recovery_active:
                        # The camera-rate tracker owns reacquisition.  Wait for
                        # its next KLT/ORB frame (or its automatic SAM recovery)
                        # instead of launching SAM after every motor waypoint.
                        self._automatic_reacquire_due = now + 0.10
                    return
                self._arm_target_confirmed = True
                self._automatic_tracking_reference = dict(self._arm_target)
            try:
                # Camera-rate KLT/ORB is intentionally image-only. Sample the
                # newest common TF only at this planning boundary. A transient
                # TF gap delays the same step and emits no motion command.
                self._refresh_arm_tracking_geometry()
            except (ValueError, TransformException) as exc:
                with self._lock:
                    self._automatic_reacquire_due = now + 0.10
                self._set_state(
                    "AUTO_APPROACH_GEOMETRY_WAIT",
                    f"Continuous image tracking is healthy; waiting for current "
                    f"Arm-camera TF before planning: {exc}",
                )
                return
            with self._lock:
                self._automatic_tracking_reference = dict(self._arm_target)
            self._set_state(
                "ARM_TARGET_CONFIRMED",
                f"Continuous {tracker_result['method']} observation is fresh "
                f"({tracker_age * 1000.0:.0f}ms); planning from measured tip TF",
            )
            self._plan_approach_step()

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
        # Several gateway state-machine branches call _set_state() while their
        # outer _on_arm_state() RLock is still held.  Publishing the large,
        # reliable/transient state message from there can retain the shared
        # lock under DDS/rosbag backpressure and starve both tracker and
        # control callbacks for seconds.  Defer those publications to the
        # existing 2 Hz state timer; calls made without an outer lock still
        # publish immediately.
        lock_is_owned = getattr(self._lock, "_is_owned", lambda: False)
        if bool(lock_is_owned()):
            self._state_publish_deferred_count += 1
            return
        self._publish_state()

    def _publish_state(self) -> None:
        with self._lock:
            available, sam_reason = self._sam.availability()
            arm_allowed, arm_reason = self._arm_gate()
            arm_state_age_sec = (
                None if self._arm_state_received is None
                else max(0.0, time.monotonic() - self._arm_state_received)
            )
            frozen = self._frozen
            target = self._target
            payload = {
                "schema_version": 1,
                "active": self._active,
                "state": self._state,
                "message": self._message,
                "base_lock": self._active,
                "motion_execution": (
                    "ROLLING_VISUAL_SERVO;TRACKER_10HZ_TARGET_UPDATES;"
                    "ACTUATOR_20HZ_VELOCITY_LIMITED;WATCHDOG_HOLD_350MS"
                ),
                "rolling_visual_servo": {
                    "implemented": True,
                    "enabled": bool(self.get_parameter(
                        "rolling_visual_servo_enabled"
                    ).value),
                    "session_id": self._rolling_servo_session_id,
                    "phase": self._rolling_servo_phase,
                    "sequence": self._rolling_servo_sequence,
                    "updates_sent": self._rolling_servo_updates_sent,
                    "stop_reason": self._rolling_servo_stop_reason,
                    "update_rate_hz": 10.0,
                    "continuous_until_press": bool(self.get_parameter(
                        "rolling_visual_servo_continuous_until_press"
                    ).value),
                    "tracker_gate": jsonable(self._rolling_servo_tracker_gate),
                    "gateway": self._arm_state.get("visual_servo", {}),
                },
                "arm_gate": {
                    "allowed": arm_allowed,
                    "reason": arm_reason,
                    "state_receive_age_sec": arm_state_age_sec,
                },
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
                "target_tracker": {
                    "enabled": bool(self.get_parameter(
                        "continuous_tracker_enabled"
                    ).value),
                    "active": self._target_tracker.active,
                    "semantic_recovery_active": self._tracker_semantic_recovery,
                    "recovery_requires_human_confirmation": True,
                    "last_result": jsonable(self._tracker_last_result),
                    "runtime_error_count": self._tracker_runtime_error_count,
                    "last_runtime_error": self._tracker_last_runtime_error,
                    "generation": self._tracker_generation,
                    "last_input_stamp": self._tracker_last_stamp,
                    "discarded_generation_count": (
                        self._tracker_discarded_generation_count
                    ),
                    "geometry_ready": bool(
                        self._arm_frozen is not None
                        and self._arm_frozen.get("geometry_ready", False)
                    ),
                    "input_policy": (
                        "LATEST_RGB_DEPTH_1_FOR_KLT_ORB;CACHED_CAMERA_INFO_AND_"
                        "LATEST_COMMON_TF_ONLY_AT_PLANNING_BOUNDARY"
                    ),
                    "last_good_age_sec": (
                        None if self._tracker_last_good_monotonic is None
                        else max(
                            0.0,
                            time.monotonic() - self._tracker_last_good_monotonic,
                        )
                    ),
                    "identity_reference": (
                        "FIRST_HUMAN_CONFIRMED_ARM_CAMERA_KEYFRAME"
                        if self._arm_appearance_reference_id is not None else None
                    ),
                    "flow": (
                        "SAM_HUMAN_INIT -> PYRAMID_KLT_RANSAC -> "
                        "ORB_RANSAC_RECOVERY -> SAM_RECOVERY_HUMAN_CONFIRM"
                    ),
                },
                "visualization": {
                    "policy": (
                        "LATEST_VALUE_COALESCED_DIAGNOSTIC_ONLY;DEFERRED_DURING_"
                        "AUTOMATIC_OR_ACTIVE_ARM_MOTION"
                    ),
                    "overlay_dirty": self._overlay_dirty,
                    "refresh_requested_count": (
                        self._overlay_refresh_requested_count
                    ),
                    "coalesced_update_count": self._overlay_coalesced_update_count,
                    "published_count": self._overlay_published_count,
                    "last_publish_duration_ms": (
                        self._overlay_last_publish_duration_ms
                    ),
                    "last_error": self._overlay_last_error,
                    "deferred_for_motion_count": (
                        self._overlay_deferred_for_motion_count
                    ),
                    "target_marker_qos": "BEST_EFFORT_VOLATILE_DEPTH_1",
                    "target_marker_published_count": (
                        self._target_visualization_published_count
                    ),
                    "target_marker_deferred_for_motion_count": (
                        self._target_visualization_deferred_for_motion_count
                    ),
                    "target_marker_last_publish_duration_ms": (
                        self._target_visualization_last_publish_duration_ms
                    ),
                },
                "debug_telemetry": {
                    "tracker_topic": "/touch_anything/tracker_debug",
                    "control_topic": "/touch_anything/control_debug",
                    "publish_policy": (
                        "QUEUED_IN_CRITICAL_CALLBACK;BEST_EFFORT_DRAINED_IN_"
                        "INDEPENDENT_TELEMETRY_CALLBACK_GROUP"
                    ),
                    "tracker_queue_depth": len(self._tracker_debug_queue),
                    "control_queue_depth": len(self._control_debug_queue),
                    "tracker_dropped_total": self._tracker_debug_dropped,
                    "control_dropped_total": self._control_debug_dropped,
                    "state_publish_deferred_count": (
                        self._state_publish_deferred_count
                    ),
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
                    "policy": (
                        "ONE_HUMAN_CONFIRMATION_THEN_TRACK_DURING_EACH_GUARDED_"
                        "TRAJECTORY;STOP_OBSERVE_REPLAN_BETWEEN_STEPS"
                    ),
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
                    "visual_change": {
                        "policy": (
                            "REPEATED_CHANGE_INSIDE_CONTINUOUSLY_TRACKED_REGION_"
                            "VERSUS_FIRST_HUMAN_CONFIRMED_KEYFRAME"
                        ),
                        "consecutive_frames": self._tracker_appearance_streak,
                        "required_frames": int(self.get_parameter(
                            "tracker_touch_required_frames"
                        ).value),
                    },
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
        self._refresh_visualization()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TouchObservationNode()
    # Six independently schedulable lanes are required here: serialized Touch
    # state/control, latency-critical Arm state, latest-frame sensor/telemetry
    # ingestion, serial OpenCV tracking, best-effort visualization, and TF
    # callbacks. Shared Touch state remains protected by the node lock and
    # motion planning stays in the serialized control callback group.
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        # rclpy.spin() propagates exceptions from any timer/subscription and
        # used to terminate the complete Touch node. Keep diagnostics/UI alive,
        # stop automatic motion, and continue servicing later callbacks.
        while rclpy.ok():
            try:
                executor.spin_once()
            except Exception as exc:
                details = f"{type(exc).__name__}: {exc}"
                with node._lock:
                    node._automatic_approach = False
                    node._automatic_reacquire_due = None
                    node._state = "TOUCH_RUNTIME_ERROR"
                    node._message = (
                        "Unexpected callback error; automatic motion stopped, "
                        "Touch node remains alive: " + details
                    )
                node.get_logger().error(
                    "Unexpected Touch callback error; keeping node alive and "
                    "stopping automatic motion: " + details + "\n"
                    + traceback.format_exc()
                )
                try:
                    node._publish_state()
                except Exception as publish_exc:
                    node.get_logger().error(
                        "Failed to publish runtime-error state: "
                        f"{type(publish_exc).__name__}: {publish_exc}"
                    )
    except KeyboardInterrupt:
        pass
    finally:
        # SIGINT/SIGTERM may already have invalidated the rclpy context before
        # this cleanup block runs.  Releasing the advisory base lock is best
        # effort during shutdown; never turn a normal exit into a traceback.
        if rclpy.ok():
            try:
                node._lock_pub.publish(Bool(data=False))
            except Exception as exc:
                node.get_logger().warning(
                    "Could not publish Touch base-lock release during shutdown: "
                    f"{type(exc).__name__}: {exc}"
                )
        executor.remove_node(node)
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
