"""Audited monocular geometry for the guarded Touch Anything approach stage."""

from __future__ import annotations

import ast
import cv2
import numpy as np


def diagnostic_visualization_should_defer(
    *, automatic_approach: bool, rolling_session_active: bool,
    execute_request_active: bool,
) -> bool:
    """Keep diagnostic DDS publication outside the motion latency budget."""
    return bool(
        automatic_approach
        or rolling_session_active
        or execute_request_active
    )


def rolling_tracker_freshness_gate(
    *, status: str, completed_age_sec: float,
    maximum_completed_age_sec: float,
    inflight_processing_age_sec: float | None,
    inflight_input_age_sec: float | None,
    maximum_inflight_processing_sec: float,
) -> dict:
    """Classify tracker freshness without racing an in-flight frame.

    A rolling controller may consume only completed tracker output.  When that
    output has just crossed its age limit, however, a newly received frame can
    already be processing in the independent tracker callback.  In that case
    the safe action is to send no new target and wait for the bounded in-flight
    callback, rather than STOP based on the preceding completion timestamp.
    """
    healthy_status = status in {"INITIALIZED", "TRACKING", "RECOVERED"}
    limits = np.asarray(
        [maximum_completed_age_sec, maximum_inflight_processing_sec],
        dtype=np.float64,
    )
    if not np.all(np.isfinite(limits)) or np.any(limits <= 0.0):
        raise ValueError("tracker freshness limits must be finite and positive")
    if not np.isfinite(completed_age_sec) and completed_age_sec != float("inf"):
        raise ValueError("completed tracker age must be finite or +inf")
    if completed_age_sec < 0.0:
        raise ValueError("completed tracker age must be non-negative")
    if healthy_status and completed_age_sec <= maximum_completed_age_sec:
        decision = "FRESH"
        reason = "latest completed tracker result is inside its age limit"
    else:
        inflight_values_valid = bool(
            inflight_processing_age_sec is not None
            and inflight_input_age_sec is not None
            and np.isfinite(inflight_processing_age_sec)
            and np.isfinite(inflight_input_age_sec)
            and inflight_processing_age_sec >= 0.0
            and inflight_input_age_sec >= 0.0
        )
        inflight_fresh = bool(
            healthy_status
            and inflight_values_valid
            and float(inflight_processing_age_sec)
            <= maximum_inflight_processing_sec
            and float(inflight_input_age_sec) <= maximum_completed_age_sec
        )
        if inflight_fresh:
            decision = "WAIT_INFLIGHT"
            reason = (
                "completed result is stale, but a fresh camera frame is "
                "inside the bounded tracker processing deadline"
            )
        else:
            decision = "STOP"
            reason = (
                "tracker status is unhealthy or neither a completed nor "
                "bounded in-flight result is fresh"
            )
    return {
        "decision": decision,
        "reason": reason,
        "status": status,
        "completed_age_sec": float(completed_age_sec),
        "maximum_completed_age_sec": float(maximum_completed_age_sec),
        "inflight_processing_age_sec": (
            None if inflight_processing_age_sec is None
            else float(inflight_processing_age_sec)
        ),
        "inflight_input_age_sec": (
            None if inflight_input_age_sec is None
            else float(inflight_input_age_sec)
        ),
        "maximum_inflight_processing_sec": float(
            maximum_inflight_processing_sec
        ),
    }


def receding_joint_action_horizon(
    *, current_positions: dict[str, float],
    target_positions: dict[str, float],
    current_velocities: dict[str, float],
    joint_bounds: dict[str, tuple[float, float]],
    steps: int, step_period_sec: float,
    prediction_lead_sec: float,
    maximum_prediction_velocity_rad_s: float,
) -> dict:
    """Build a smooth, replaceable joint-action chunk from predicted state.

    The predicted start advances measured joint feedback by the bounded
    perception/planning/transport lead.  Ten short targets are normally sent;
    a newer observation replaces the unconsumed tail at the actuator server.
    """
    names = list(target_positions)
    if (
        not names
        or set(current_positions) != set(names)
        or set(current_velocities) != set(names)
        or set(joint_bounds) != set(names)
    ):
        raise ValueError("horizon inputs must contain identical non-empty joints")
    if not isinstance(steps, int) or not 2 <= steps <= 20:
        raise ValueError("action horizon steps must be within [2, 20]")
    scalars = np.asarray([
        step_period_sec, prediction_lead_sec,
        maximum_prediction_velocity_rad_s,
    ], dtype=np.float64)
    if (
        not np.all(np.isfinite(scalars))
        or step_period_sec <= 0.0
        or prediction_lead_sec < 0.0
        or maximum_prediction_velocity_rad_s <= 0.0
    ):
        raise ValueError("action horizon timing/velocity inputs are invalid")
    predicted: dict[str, float] = {}
    clipped_velocities: dict[str, float] = {}
    for name in names:
        lower, upper = (float(value) for value in joint_bounds[name])
        current = float(current_positions[name])
        target = float(target_positions[name])
        velocity = float(current_velocities[name])
        if not np.all(np.isfinite([lower, upper, current, target, velocity])):
            raise ValueError(f"non-finite action horizon input for {name}")
        if lower >= upper or not lower <= current <= upper or not lower <= target <= upper:
            raise ValueError(f"action horizon joint {name} is outside its bounds")
        velocity = float(np.clip(
            velocity,
            -maximum_prediction_velocity_rad_s,
            maximum_prediction_velocity_rad_s,
        ))
        clipped_velocities[name] = velocity
        predicted[name] = float(np.clip(
            current + velocity * prediction_lead_sec,
            lower,
            upper,
        ))
    horizon: list[dict[str, float]] = []
    for index in range(steps):
        linear = float(index + 1) / float(steps)
        blend = linear * linear * (3.0 - 2.0 * linear)
        horizon.append({
            name: float(
                predicted[name]
                + (float(target_positions[name]) - predicted[name]) * blend
            )
            for name in names
        })
    horizon[-1] = {name: float(target_positions[name]) for name in names}
    return {
        "positions": horizon,
        "steps": steps,
        "step_period_sec": float(step_period_sec),
        "duration_sec": float(steps * step_period_sec),
        "prediction_lead_sec": float(prediction_lead_sec),
        "current_positions": {
            name: float(current_positions[name]) for name in names
        },
        "clipped_velocities": clipped_velocities,
        "predicted_start_positions": predicted,
        "terminal_positions": {
            name: float(target_positions[name]) for name in names
        },
    }


def rolling_approach_phase(
    *, advanced_from_far_m: float, near_distance_m: float,
    press_authorized: bool,
) -> str:
    """Select the continuous visual-servo phase from measured progress."""
    values = np.asarray([advanced_from_far_m, near_distance_m], dtype=np.float64)
    if not np.all(np.isfinite(values)) or advanced_from_far_m < 0.0:
        raise ValueError("rolling approach progress must be finite and non-negative")
    if near_distance_m <= 0.0:
        raise ValueError("near distance must be positive")
    if press_authorized:
        return "PRESS"
    if advanced_from_far_m >= near_distance_m - 1.0e-3:
        return "NEAR_ALIGN"
    return "FAR_TRACK_AND_ADVANCE"


def rolling_forward_target_x(
    *, phase: str, actual_x_m: float, far_x_m: float,
    near_distance_m: float, horizon_m: float,
    target_plane_x_m: float | None = None,
    maximum_press_past_target_m: float | None = None,
) -> tuple[float, bool]:
    """Return the next measured-frame X target and whether travel is complete."""
    values = np.asarray(
        [actual_x_m, far_x_m, near_distance_m, horizon_m], dtype=np.float64
    )
    if not np.all(np.isfinite(values)) or near_distance_m <= 0.0 or horizon_m < 0.0:
        raise ValueError("rolling X inputs must be finite with non-negative horizon")
    if phase == "FAR_TRACK_AND_ADVANCE":
        near_x = far_x_m + near_distance_m
        return float(min(actual_x_m + horizon_m, near_x)), False
    if phase == "NEAR_ALIGN":
        return float(actual_x_m), False
    if phase == "PRESS":
        if target_plane_x_m is None or maximum_press_past_target_m is None:
            raise ValueError("PRESS requires target plane and maximum overtravel")
        maximum_x = float(target_plane_x_m) + float(maximum_press_past_target_m)
        if not np.isfinite(maximum_x) or maximum_press_past_target_m <= 0.0:
            raise ValueError("PRESS maximum travel is invalid")
        complete = actual_x_m >= maximum_x - 1.0e-3
        return float(min(actual_x_m + horizon_m, maximum_x)), bool(complete)
    raise ValueError(f"unknown rolling approach phase: {phase}")


def rolling_z_motion_compensation(
    *, visual_delta_z_m: float, previous_target_error_z_m: float,
    nominal_x_horizon_m: float, phase: str,
    residual_gain: float, far_z_per_x_gain: float,
    maximum_compensation_m: float,
    slowdown_start_error_m: float, slowdown_full_error_m: float,
) -> dict:
    """Carry unachieved Z motion forward and slow X while Z lags.

    ``previous_target_error_z_m`` is the preceding Cartesian target minus the
    newest measured tip.  Unlike a fixed bias, carrying this residual cannot
    silently disappear when the next image update is generated from the
    measured pose.  X-to-Z feedforward is active only while X advances.
    """
    values = np.asarray([
        visual_delta_z_m, previous_target_error_z_m, nominal_x_horizon_m,
        residual_gain, far_z_per_x_gain, maximum_compensation_m,
        slowdown_start_error_m, slowdown_full_error_m,
    ], dtype=np.float64)
    if not np.all(np.isfinite(values)):
        raise ValueError("rolling Z compensation inputs must be finite")
    if (
        nominal_x_horizon_m <= 0.0
        or residual_gain < 0.0
        or far_z_per_x_gain < 0.0
        or maximum_compensation_m <= 0.0
        or slowdown_start_error_m < 0.0
        or slowdown_full_error_m <= slowdown_start_error_m
    ):
        raise ValueError("rolling Z compensation limits are invalid")
    error = abs(previous_target_error_z_m)
    horizon_scale = float(np.clip(
        (slowdown_full_error_m - error)
        / (slowdown_full_error_m - slowdown_start_error_m),
        0.0, 1.0,
    ))
    advancing = phase in {"FAR_TRACK_AND_ADVANCE", "PRESS"}
    scaled_horizon = nominal_x_horizon_m * horizon_scale if advancing else 0.0
    model_feedforward = (
        far_z_per_x_gain * scaled_horizon if advancing else 0.0
    )
    residual_carry = residual_gain * previous_target_error_z_m
    compensation = float(np.clip(
        residual_carry + model_feedforward,
        -maximum_compensation_m,
        maximum_compensation_m,
    ))
    return {
        "visual_delta_z_m": float(visual_delta_z_m),
        "previous_target_error_z_m": float(previous_target_error_z_m),
        "residual_carry_z_m": float(residual_carry),
        "model_feedforward_z_m": float(model_feedforward),
        "compensation_z_m": compensation,
        "command_delta_z_m": float(visual_delta_z_m + compensation),
        "x_horizon_scale": horizon_scale,
        "scaled_x_horizon_m": float(scaled_horizon),
    }


def mild_single_motor_hold_entry_overrun(
    message: str, *, motor: str, maximum_excess_ticks: int
) -> dict:
    """Recognize a small, isolated post-motion HOLD-entry miss.

    The Arm gateway embeds the violating-motor dictionary in its diagnostic
    message.  Only an isolated requested motor is eligible; malformed text,
    multiple violations, or a larger miss remain hard failures.
    """
    marker = "Final error exceeds per-motor HOLD-entry tolerance:"
    suffix = "; all errors="
    if marker not in message:
        return {"eligible": False, "reason": "NOT_HOLD_ENTRY_ERROR"}
    try:
        encoded = message.split(marker, 1)[1].split(suffix, 1)[0].strip()
        violations = ast.literal_eval(encoded)
    except (ValueError, SyntaxError):
        return {"eligible": False, "reason": "UNPARSEABLE_VIOLATIONS"}
    if not isinstance(violations, dict) or set(violations) != {motor}:
        return {"eligible": False, "reason": "NOT_ISOLATED_REQUESTED_MOTOR"}
    entry = violations.get(motor)
    if not isinstance(entry, dict):
        return {"eligible": False, "reason": "INVALID_MOTOR_ENTRY"}
    try:
        error = abs(int(entry["error"]))
        tolerance = abs(int(entry["tolerance"]))
    except (KeyError, TypeError, ValueError):
        return {"eligible": False, "reason": "INVALID_ERROR_OR_TOLERANCE"}
    excess = error - tolerance
    return {
        "eligible": bool(0 < excess <= int(maximum_excess_ticks)),
        "motor": motor,
        "error_ticks": error,
        "tolerance_ticks": tolerance,
        "excess_ticks": excess,
        "maximum_excess_ticks": int(maximum_excess_ticks),
    }


def phase_sam_minimum_score(
    *,
    operator_handoff_complete: bool,
    near_phase_latched: bool,
    press_phase_active: bool,
    initial_score: float,
    far_tracking_score: float,
    near_tracking_score: float,
    press_score: float,
) -> float:
    """Select a SAM score gate from approach phase, not apparent mask size.

    Mask occupancy is view-dependent and is now only a too-close hint.  Once
    the operator has confirmed the Arm Camera object, subsequent stopped-frame
    tracking uses a lower phase-specific score while continuity and appearance
    checks continue to guard object identity.
    """
    scores = np.asarray(
        [initial_score, far_tracking_score, near_tracking_score, press_score],
        dtype=np.float64,
    )
    if not np.all(np.isfinite(scores)) or np.any(scores < 0.0) or np.any(scores > 1.0):
        raise ValueError("SAM score thresholds must be finite values in [0,1]")
    if not operator_handoff_complete:
        return float(initial_score)
    if press_phase_active:
        return float(press_score)
    if near_phase_latched:
        return float(near_tracking_score)
    return float(far_tracking_score)


def phase_tracking_area_ratio_bounds(
    *,
    near_phase_latched: bool,
    far_minimum_ratio: float,
    near_minimum_ratio: float,
    maximum_ratio: float,
    near_maximum_ratio: float | None = None,
) -> tuple[float, float]:
    """Return phase-aware mask continuity bounds for the wrist camera.

    In NEAR, the fixed jaw occludes an increasing portion of the object and
    SAM may return a stable visible fragment.  A small centroid shift remains
    useful evidence, while the old global 0.35 area-ratio floor is not.
    """
    values = np.asarray(
        [
            far_minimum_ratio,
            near_minimum_ratio,
            maximum_ratio,
            maximum_ratio if near_maximum_ratio is None else near_maximum_ratio,
        ], dtype=np.float64
    )
    if not np.all(np.isfinite(values)):
        raise ValueError("tracking area-ratio bounds must be finite")
    minimum = near_minimum_ratio if near_phase_latched else far_minimum_ratio
    maximum = (
        maximum_ratio
        if not near_phase_latched or near_maximum_ratio is None
        else near_maximum_ratio
    )
    if minimum <= 0.0 or maximum < minimum:
        raise ValueError("tracking area-ratio bounds are invalid")
    return float(minimum), float(maximum)


def should_retreat_for_too_close(
    *, too_close: bool, press_alignment_ready: bool
) -> bool:
    """Retreat only when proximity and missing press alignment coexist."""
    return bool(too_close and not press_alignment_ready)


def compensated_cartesian_delta(
    desired_delta_base_m: np.ndarray,
    *,
    x_hold_bias_m: float = 0.0,
    y_gain: float = 1.0,
    z_gain: float = 1.0,
    z_up_bias_m: float = 0.0,
    maximum_norm_m: float = 0.040,
) -> dict:
    """Return a bounded empirical command for a measured Cartesian residual.

    ``desired_delta_base_m`` remains the evidence/acceptance target.  The
    returned command only supplies feed-forward for the repeatable real-arm
    under-travel observed in the local Touch pose.  A positive Z holding bias
    is applied even when desired Z is zero because the measured error is a
    gravity-direction offset, not merely a scale error.
    """
    desired = np.asarray(desired_delta_base_m, dtype=np.float64).reshape(3)
    values = np.array(
        [*desired, x_hold_bias_m, y_gain, z_gain, z_up_bias_m, maximum_norm_m],
        dtype=np.float64,
    )
    if not np.all(np.isfinite(values)):
        raise ValueError("Cartesian compensation inputs must be finite")
    if y_gain <= 0.0 or z_gain <= 0.0 or maximum_norm_m <= 0.0:
        raise ValueError("compensation gains and maximum norm must be positive")
    command = np.array(
        [
            desired[0] + x_hold_bias_m,
            desired[1] * y_gain,
            desired[2] * z_gain + z_up_bias_m,
        ],
        dtype=np.float64,
    )
    unbounded = command.copy()
    norm = float(np.linalg.norm(command))
    if norm > maximum_norm_m:
        command *= maximum_norm_m / norm
    return {
        "desired_delta_base_m": desired,
        "command_delta_base_m": command,
        "unbounded_command_delta_base_m": unbounded,
        "bounded": bool(norm > maximum_norm_m),
        "maximum_norm_m": float(maximum_norm_m),
    }


def adaptive_close_mask_fraction(
    target_width_m: float,
    target_height_m: float,
    reference_side_m: float = 0.038,
    reference_fraction: float = 0.25,
    minimum_fraction: float = 0.06,
    maximum_fraction: float = 0.35,
) -> float:
    """Scale the wrist-camera close threshold from D435 target dimensions."""
    values = np.asarray(
        [target_width_m, target_height_m, reference_side_m,
         reference_fraction, minimum_fraction, maximum_fraction], dtype=float,
    )
    if not np.all(np.isfinite(values)):
        raise ValueError("adaptive mask-fraction inputs must be finite")
    if target_width_m <= 0.0 or target_height_m <= 0.0 or reference_side_m <= 0.0:
        raise ValueError("target and reference dimensions must be positive")
    if not 0.0 < minimum_fraction <= maximum_fraction < 1.0:
        raise ValueError("mask-fraction bounds must satisfy 0 < min <= max < 1")
    if not 0.0 < reference_fraction < 1.0:
        raise ValueError("reference mask fraction must lie in (0,1)")
    fraction = reference_fraction * (
        target_width_m * target_height_m / (reference_side_m ** 2)
    )
    return float(np.clip(fraction, minimum_fraction, maximum_fraction))


def project_camera_point(
    point_camera_m: np.ndarray,
    camera_matrix: np.ndarray,
    distortion: np.ndarray | None = None,
) -> tuple[float, float]:
    """Project a metric camera-frame point into the calibrated raw RGB image."""
    point = np.asarray(point_camera_m, dtype=np.float64).reshape(3)
    if not np.all(np.isfinite(point)) or point[2] <= 0.0:
        raise ValueError("target is not in front of the arm camera")
    coefficients = (
        np.asarray(distortion, dtype=np.float64).reshape(-1)
        if distortion is not None else np.zeros(0, dtype=np.float64)
    )
    pixels, _ = cv2.projectPoints(
        point.reshape(1, 1, 3), np.zeros(3), np.zeros(3),
        np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3), coefficients,
    )
    u, v = pixels.reshape(2)
    if not np.isfinite(u) or not np.isfinite(v):
        raise ValueError("arm-camera projection is non-finite")
    return float(u), float(v)


def monocular_lateral_target(
    pixel_uv: tuple[float, float],
    camera_matrix: np.ndarray,
    distortion: np.ndarray | None,
    base_from_camera_rotation: np.ndarray,
    base_from_camera_translation: np.ndarray,
    d435_anchor_base_m: np.ndarray,
) -> dict:
    """Use a calibrated RGB ray to refine target Y/Z without claiming depth.

    The D435 point supplies only the range scale and fixed X approach plane.
    The arm-camera observation supplies the ray direction.  This deliberately
    does not report monocular metric depth as observed evidence.
    """
    k = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
    coefficients = (
        np.asarray(distortion, dtype=np.float64).reshape(-1)
        if distortion is not None else np.zeros(0, dtype=np.float64)
    )
    pixel = np.asarray(pixel_uv, dtype=np.float64).reshape(1, 1, 2)
    ray_xy = cv2.undistortPoints(pixel, k, coefficients).reshape(2)
    ray_camera = np.array([ray_xy[0], ray_xy[1], 1.0], dtype=np.float64)
    ray_camera /= np.linalg.norm(ray_camera)
    rotation = np.asarray(base_from_camera_rotation, dtype=np.float64).reshape(3, 3)
    camera_origin = np.asarray(base_from_camera_translation, dtype=np.float64).reshape(3)
    anchor = np.asarray(d435_anchor_base_m, dtype=np.float64).reshape(3)
    ray_base = rotation @ ray_camera
    ray_base /= np.linalg.norm(ray_base)
    range_along_ray = float(np.dot(anchor - camera_origin, ray_base))
    if not np.isfinite(range_along_ray) or range_along_ray <= 0.02:
        raise ValueError("D435 anchor does not provide a valid positive arm-camera ray range")
    ray_point = camera_origin + range_along_ray * ray_base
    refined = np.array([anchor[0], ray_point[1], ray_point[2]], dtype=np.float64)
    return {
        "point_base_m": refined,
        "ray_point_base_m": ray_point,
        "ray_base": ray_base,
        "range_scale_source": "D435_ANCHOR_PROJECTED_ON_ARM_CAMERA_RAY",
        "range_along_ray_m": range_along_ray,
        "anchor_ray_residual_m": float(np.linalg.norm(anchor - ray_point)),
        "monocular_depth_observed": False,
    }


def visual_tip_alignment_delta(
    target_pixel_uv: tuple[float, float],
    fixed_tip_pixel_uv: tuple[float, float],
    camera_matrix: np.ndarray,
    distortion: np.ndarray | None,
    base_from_camera_rotation: np.ndarray,
    range_scale_m: float,
) -> dict:
    """Convert target-to-fixed-tip image error into a base-frame Y/Z correction.

    The wrist camera and the lower jaw tip are rigidly related, so the lower
    tip has a repeatable image coordinate.  Both image rays are evaluated at
    the same approximate range; only the resulting base Y/Z delta is used.
    This is image-based alignment and does not claim monocular depth.
    """
    if not np.isfinite(range_scale_m) or range_scale_m <= 0.02:
        raise ValueError("a positive range scale is required for visual tip alignment")
    k = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
    coefficients = (
        np.asarray(distortion, dtype=np.float64).reshape(-1)
        if distortion is not None else np.zeros(0, dtype=np.float64)
    )
    pixels = np.asarray(
        [target_pixel_uv, fixed_tip_pixel_uv], dtype=np.float64
    ).reshape(2, 1, 2)
    normalized = cv2.undistortPoints(pixels, k, coefficients).reshape(2, 2)
    rays = np.column_stack((normalized, np.ones(2, dtype=np.float64)))
    rays /= np.linalg.norm(rays, axis=1, keepdims=True)
    camera_delta = float(range_scale_m) * (rays[0] - rays[1])
    rotation = np.asarray(base_from_camera_rotation, dtype=np.float64).reshape(3, 3)
    base_delta = rotation @ camera_delta
    pixel_error = np.asarray(target_pixel_uv, dtype=float) - np.asarray(
        fixed_tip_pixel_uv, dtype=float
    )
    return {
        "delta_base_m": base_delta,
        "pixel_error_uv": pixel_error,
        "pixel_error_norm": float(np.linalg.norm(pixel_error)),
        "range_scale_m": float(range_scale_m),
        "range_source": "D435_ANCHOR_RANGE_SCALE_ONLY",
        "depth_observed_by_arm_camera": False,
    }


def base_yz_pixel_alignment_delta(
    target_pixel_uv: tuple[float, float],
    fixed_tip_pixel_uv: tuple[float, float],
    camera_matrix: np.ndarray,
    distortion: np.ndarray | None,
    base_from_camera_rotation: np.ndarray,
    range_scale_m: float,
    jacobian_probe_m: float = 0.001,
) -> dict:
    """Solve the local image Jacobian for a base-frame Y/Z camera motion.

    The wrist camera is rigidly attached to the tip, so a commanded tip Y/Z
    translation produces the same camera translation. Unlike projecting a 3-D
    ray delta and then discarding base X, this directly retains the coupling
    between both image axes and the two allowed base-frame motion axes.
    """
    if not np.isfinite(range_scale_m) or range_scale_m <= 0.02:
        raise ValueError("a positive range scale is required for visual tip alignment")
    if not np.isfinite(jacobian_probe_m) or jacobian_probe_m <= 0.0:
        raise ValueError("jacobian probe distance must be positive")
    k = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
    coefficients = (
        np.asarray(distortion, dtype=np.float64).reshape(-1)
        if distortion is not None else np.zeros(0, dtype=np.float64)
    )
    target_uv = np.asarray(target_pixel_uv, dtype=np.float64).reshape(2)
    fixed_uv = np.asarray(fixed_tip_pixel_uv, dtype=np.float64).reshape(2)
    normalized = cv2.undistortPoints(
        target_uv.reshape(1, 1, 2), k, coefficients
    ).reshape(2)
    ray_camera = np.array([normalized[0], normalized[1], 1.0], dtype=np.float64)
    ray_camera /= np.linalg.norm(ray_camera)
    point_camera = float(range_scale_m) * ray_camera
    rotation = np.asarray(base_from_camera_rotation, dtype=np.float64).reshape(3, 3)
    camera_from_base_rotation = rotation.T
    jacobian = np.zeros((2, 2), dtype=np.float64)
    for column, base_axis in enumerate((1, 2)):
        camera_motion_base = np.zeros(3, dtype=np.float64)
        camera_motion_base[base_axis] = float(jacobian_probe_m)
        moved_point_camera = (
            point_camera - camera_from_base_rotation @ camera_motion_base
        )
        moved_uv = np.asarray(
            project_camera_point(moved_point_camera, k, coefficients),
            dtype=np.float64,
        )
        jacobian[:, column] = (moved_uv - target_uv) / float(jacobian_probe_m)
    if not np.all(np.isfinite(jacobian)) or np.linalg.matrix_rank(jacobian) < 2:
        raise ValueError("base Y/Z image Jacobian is singular at the current camera pose")
    desired_pixel_motion = fixed_uv - target_uv
    delta_yz, _, _, singular_values = np.linalg.lstsq(
        jacobian, desired_pixel_motion, rcond=None
    )
    if not np.all(np.isfinite(delta_yz)):
        raise ValueError("base Y/Z image-Jacobian solution is non-finite")
    return {
        "delta_base_yz_m": delta_yz,
        "pixel_error_uv": target_uv - fixed_uv,
        "pixel_error_norm": float(np.linalg.norm(target_uv - fixed_uv)),
        "image_jacobian_px_per_m": jacobian,
        "jacobian_singular_values": singular_values,
        "range_scale_m": float(range_scale_m),
        "range_source": "D435_ANCHOR_RANGE_SCALE_ONLY",
        "depth_observed_by_arm_camera": False,
    }


def empirical_image_center_alignment_delta(
    target_pixel_uv: tuple[float, float],
    image_center_uv: tuple[float, float],
    metres_per_pixel: float,
) -> dict:
    """Map wrist-camera target offset to the measured robot-base Y/Z motion.

    This mapping is deliberately empirical rather than inferred from the
    rendered camera TF.  Real-robot tests established the following signs:

    * target below centre  -> move the arm tip robot-right (-base Y)
    * target above centre  -> move the arm tip robot-left  (+base Y)
    * target left of centre -> move the arm tip down        (-base Z)
    * target right of centre -> move the arm tip up         (+base Z)

    The caller independently caps the resulting metric step.
    """
    target = np.asarray(target_pixel_uv, dtype=np.float64).reshape(2)
    centre = np.asarray(image_center_uv, dtype=np.float64).reshape(2)
    if not np.all(np.isfinite(target)) or not np.all(np.isfinite(centre)):
        raise ValueError("target and image-centre pixels must be finite")
    if not np.isfinite(metres_per_pixel) or metres_per_pixel <= 0.0:
        raise ValueError("metres_per_pixel must be positive")
    error_uv = target - centre
    delta_yz = np.array(
        [-error_uv[1] * metres_per_pixel, error_uv[0] * metres_per_pixel],
        dtype=np.float64,
    )
    return {
        "delta_base_yz_m": delta_yz,
        "pixel_error_uv": error_uv,
        "pixel_error_norm": float(np.linalg.norm(error_uv)),
        "image_center_uv": centre,
        "metres_per_pixel": float(metres_per_pixel),
        "mapping": "IMAGE_DOWN_TO_BASE_NEG_Y_IMAGE_RIGHT_TO_BASE_POS_Z",
    }


def pixel_inside_mask(mask: np.ndarray, pixel_uv: tuple[float, float]) -> bool:
    """Return whether the rounded image pixel is part of a 2-D SAM mask."""
    candidate = np.asarray(mask)
    if candidate.ndim != 2:
        raise ValueError("mask must be a 2-D array")
    pixel = np.asarray(pixel_uv, dtype=np.float64).reshape(2)
    if not np.all(np.isfinite(pixel)):
        return False
    u, v = (int(round(pixel[0])), int(round(pixel[1])))
    height, width = candidate.shape
    return bool(0 <= u < width and 0 <= v < height and candidate[v, u])


def bounded_retreat_step(
    current_x_m: float, far_x_m: float, maximum_retreat_m: float,
    boundary_tolerance_m: float = 0.0005,
) -> float:
    """Return a non-positive retreat that never crosses the FAR X plane."""
    values = np.asarray(
        [current_x_m, far_x_m, maximum_retreat_m, boundary_tolerance_m],
        dtype=np.float64,
    )
    if not np.all(np.isfinite(values)):
        raise ValueError("retreat inputs must be finite")
    if maximum_retreat_m <= 0.0 or boundary_tolerance_m < 0.0:
        raise ValueError("retreat distance must be positive and tolerance non-negative")
    if current_x_m <= far_x_m + boundary_tolerance_m:
        return 0.0
    return float(max(far_x_m, current_x_m - maximum_retreat_m) - current_x_m)


def next_approach_waypoint(
    actual_tip_base_m: np.ndarray,
    refined_target_base_m: np.ndarray,
    step_distance_m: float,
    stop_distance_m: float,
) -> dict:
    """Advance only in base +X while adopting visual Y/Z corrections."""
    actual = np.asarray(actual_tip_base_m, dtype=np.float64).reshape(3)
    target = np.asarray(refined_target_base_m, dtype=np.float64).reshape(3)
    if step_distance_m <= 0.0 or stop_distance_m <= 0.0:
        raise ValueError("approach step and stop distances must be positive")
    remaining = float(target[0] - actual[0])
    complete = remaining <= stop_distance_m + 1.0e-3
    next_remaining = max(stop_distance_m, remaining - step_distance_m)
    waypoint = np.array(
        [target[0] - next_remaining, target[1], target[2]], dtype=np.float64
    )
    return {
        "waypoint_base_m": waypoint,
        "remaining_before_m": remaining,
        "remaining_after_m": next_remaining,
        "complete": complete,
        "motion_axis_base": [1.0, 0.0, 0.0],
    }


def validate_tracking_continuity(
    previous_centroid_uv, current_centroid_uv,
    previous_area_px: int, current_area_px: int,
    maximum_shift_px: float, minimum_area_ratio: float, maximum_area_ratio: float,
) -> dict:
    """Validate conservative same-object evidence between stopped frames."""
    previous = np.asarray(previous_centroid_uv, dtype=float)
    current = np.asarray(current_centroid_uv, dtype=float)
    if previous.shape != (2,) or current.shape != (2,):
        raise ValueError("previous and current centroids must each contain u,v")
    if previous_area_px <= 0 or current_area_px <= 0:
        raise ValueError("mask areas must be positive")
    shift = float(np.linalg.norm(current - previous))
    area_ratio = float(current_area_px) / float(previous_area_px)
    valid = bool(
        shift <= maximum_shift_px
        and minimum_area_ratio <= area_ratio <= maximum_area_ratio
    )
    return {"valid": valid, "centroid_shift_px": shift, "mask_area_ratio": area_ratio}
