"""Audited monocular geometry for the guarded Touch Anything approach stage."""

from __future__ import annotations

import cv2
import numpy as np


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
