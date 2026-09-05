#!/usr/bin/env python3
"""Small audited SO101 FK/position-and-axis IK used for preview only.

This is not a collision planner.  It reproduces the deployed visual URDF chain
and deliberately reports that limitation to the caller.
"""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np
from scipy.optimize import least_squares


JOINT_NAMES = (
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_flex_joint",
    "wrist_flex_joint", "wrist_roll_joint",
)
LOWER = np.array([-1.91986, -1.74533, -1.69, -1.6, -2.3], dtype=float)
UPPER = np.array([1.91986, 1.74533, 1.54, 1.6, 2.3], dtype=float)
REFERENCE = np.array([0.03217642458100545, 1.74533, -0.295563703024748,
                      -0.0373075230972284, -1.2867643467643466], dtype=float)
# 2026-09-05 constrained joint/eye-in-hand solve: 47 stable ChArUco
# observations across four independently fixed board poses.  Raw corners were
# re-solved using the physically measured 33.8 mm square / 16.5 mm marker.
# Shoulder-pan zero and wrist-roll zero remain fixed gauge freedoms; they
# cannot be separated from unknown board pose / eye rotation by this dataset.
SCALE = np.array([1.0642740565970414, 1.0225823378138021,
                  1.0156842687470935, 1.083955420493,
                  1.3663034512798975], dtype=float)
VISUAL_OFFSET = np.array([0.0, 0.09842437585106671,
                          -0.052102814132610975,
                          0.023105072091396325,
                          -0.3361107058855241], dtype=float)

# Operator-measured, comfortable Touch Prepare poses from the 2026-09-04
# screenshots in todo-my/arm-baseline/skills-arm/ref-images.  These are seeds,
# never direct commands: every result still passes bounded IK and the Arm
# gateway Preview/Validate gate.  Lateral captures intentionally serve both
# FAR and NEAR because the operator recorded one comfortable pose for both.
PREPARE_REFERENCE_SEEDS = (
    ("front_low_far", (-0.104, 1.636, -0.288, -0.807, -0.015), 270.97, 45.18),
    ("front_low_near", (-0.056, 1.578, 0.375, -1.548, -0.014), 270.97, 32.78),
    ("front_mid_far", (-0.101, 0.997, 0.301, -1.159, 0.070), 271.00, 32.80),
    ("front_mid_near", (-0.092, -0.553, 1.393, -0.719, 0.070), 270.70, 12.39),
    ("front_high_far", (0.032, 0.246, -1.024, 0.443, 0.016), 266.75, 11.25),
    ("front_high_near", (-0.049, -0.534, -0.828, 1.210, 0.017), 267.01, 2.90),
    ("right_low", (0.673, 1.745, -0.722, -0.813, 0.016), 225.62, 52.12),
    ("right_mid", (0.756, 0.634, 0.529, -0.965, 0.014), 203.20, 37.35),
    ("right_high", (0.833, -0.338, -0.251, 0.440, 0.016), 183.60, 26.28),
    ("left_low", (-0.772, 1.686, -0.299, -1.088, 0.014), 310.25, 44.56),
    ("left_mid", (-0.733, 0.150, 0.713, -0.579, 0.015), 292.76, 16.52),
    ("left_high", (-0.813, 0.314, -0.931, 0.450, 0.015), 306.91, 11.43),
)

# xyz/rpy copied from the deployed so_arm101_macro.xacro joint chain.
CHAIN = (
    ((0.0388353, -8.97657e-09, 0.0624), (math.pi, 0.0, -math.pi)),
    ((-0.0303992, -0.0182778, -0.0542), (-math.pi / 2, -math.pi / 2, 0.0)),
    ((-0.11257, -0.028, 0.0), (0.0, 0.0, math.pi / 2)),
    ((-0.1349, 0.0052, 0.0), (0.0, 0.0, -1.57)),
    ((0.0, -0.0611, 0.0181), (-1.57, 3.14, 0.0)),
)
TIP_FIXED = ((-0.0079, -0.000218121, -0.0981274), (0.0, math.pi, 0.0))
# Operator-confirmed contact origin, expressed in gripper_frame_link.
# The operator physically identified gripper_frame_link's origin as the desired
# contact point.  Keep no additional inferred STL offset here.
TOUCH_TIP_OFFSET = (0.0, 0.0, 0.0)
TOUCH_WRIST_ROLL_TARGET_RAD = 0.0
TOUCH_WRIST_ROLL_TOLERANCE_RAD = 0.03


def _rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _transform(xyz: Iterable[float], rpy: Iterable[float]) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = _rpy(*rpy)
    result[:3, 3] = np.asarray(tuple(xyz), dtype=float)
    return result


def _rz(angle: float) -> np.ndarray:
    result = np.eye(4)
    c, s = math.cos(angle), math.sin(angle)
    result[:3, :3] = ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))
    return result


def visual_positions(command_positions: Iterable[float]) -> np.ndarray:
    return (REFERENCE + SCALE *
            (np.asarray(tuple(command_positions), dtype=float) - REFERENCE) +
            VISUAL_OFFSET)


def tip_transform(command_positions: Iterable[float]) -> np.ndarray:
    values = visual_positions(command_positions)
    result = np.eye(4)
    for (xyz, rpy), angle in zip(CHAIN, values):
        result = result @ _transform(xyz, rpy) @ _rz(float(angle))
    return result @ _transform(*TIP_FIXED) @ _transform(TOUCH_TIP_OFFSET, (0.0, 0.0, 0.0))


def solve_near_tip_alignment(
    current_positions: Iterable[float], target_position: Iterable[float],
    lower_limits: Iterable[float] | None = None,
    upper_limits: Iterable[float] | None = None,
) -> dict:
    """Solve a local NEAR correction with only shoulder pan/lift and elbow.

    Wrist flex and wrist roll are held at their measured values.  This avoids
    the full pose/axis optimisation used for FAR and PRESS from turning a small
    image-space Y/Z correction into a large wrist motion or X retreat.
    """
    current = np.asarray(tuple(current_positions), dtype=float)
    target = np.asarray(tuple(target_position), dtype=float)
    lower = LOWER if lower_limits is None else np.asarray(tuple(lower_limits), dtype=float)
    upper = UPPER if upper_limits is None else np.asarray(tuple(upper_limits), dtype=float)
    if current.shape != (5,) or target.shape != (3,):
        raise ValueError("five current joints and one 3-D target are required")
    if lower.shape != (5,) or upper.shape != (5,) or np.any(lower >= upper):
        raise ValueError("five valid lower/upper joint limits are required")

    fixed_wrist = current[3:5].copy()

    def compose(active: np.ndarray) -> np.ndarray:
        return np.concatenate((active, fixed_wrist))

    def residual(active: np.ndarray) -> np.ndarray:
        q = compose(active)
        position_error = tip_transform(q)[:3, 3] - target
        # Position is primary. A light local-motion cost removes redundant
        # solutions without introducing a prepare-pose or tool-axis objective.
        return np.concatenate((position_error * 300.0,
                               (active - current[:3]) * 0.015))

    result = least_squares(
        residual, np.clip(current[:3], lower[:3], upper[:3]),
        bounds=(lower[:3], upper[:3]), max_nfev=200,
    )
    positions = compose(result.x)
    pose = tip_transform(positions)
    position_error = float(np.linalg.norm(pose[:3, 3] - target))
    return {
        "success": bool(result.success and position_error <= 0.008),
        "positions": {
            name: float(value) for name, value in zip(JOINT_NAMES, positions)
        },
        "tip_position": pose[:3, 3].tolist(),
        "tool_axis": pose[:3, 2].tolist(),
        "position_error_m": position_error,
        "axis_error_deg": 0.0,
        "orientation_constrained": False,
        "solver_status": int(result.status),
        "solver_message": str(result.message),
        "collision_checked": False,
        "tip_frame": "arm/touch_tip (operator-confirmed gripper_frame_link origin)",
        "prepare_strategy": "NEAR_LOCAL_SHOULDER_PAN_LIFT_ELBOW_ONLY",
        "locked_joints": ["wrist_flex_joint", "wrist_roll_joint"],
    }


def solve_near_forward_press(
    current_positions: Iterable[float], target_position: Iterable[float],
    lower_limits: Iterable[float] | None = None,
    upper_limits: Iterable[float] | None = None,
) -> dict:
    """Solve a short NEAR +X press with a constrained planar joint set.

    Shoulder pan and wrist roll stay at their measured values. Shoulder lift
    and elbow flex provide the main reach motion, while wrist flex is allowed
    to compensate the tool pitch. Position remains the primary objective;
    preserving the current tool axis is a soft secondary objective.
    """
    current = np.asarray(tuple(current_positions), dtype=float)
    target = np.asarray(tuple(target_position), dtype=float)
    lower = LOWER if lower_limits is None else np.asarray(tuple(lower_limits), dtype=float)
    upper = UPPER if upper_limits is None else np.asarray(tuple(upper_limits), dtype=float)
    if current.shape != (5,) or target.shape != (3,):
        raise ValueError("five current joints and one 3-D target are required")
    if lower.shape != (5,) or upper.shape != (5,) or np.any(lower >= upper):
        raise ValueError("five valid lower/upper joint limits are required")

    fixed_pan = float(current[0])
    fixed_roll = float(current[4])
    current_active = current[[1, 2, 3]].copy()
    current_axis = tip_transform(current)[:3, 2].copy()

    def compose(active: np.ndarray) -> np.ndarray:
        return np.array(
            [fixed_pan, active[0], active[1], active[2], fixed_roll],
            dtype=float,
        )

    def residual(active: np.ndarray) -> np.ndarray:
        q = compose(active)
        pose = tip_transform(q)
        position_error = pose[:3, 3] - target
        axis_error = np.cross(pose[:3, 2], current_axis)
        return np.concatenate((
            position_error * 400.0,
            axis_error * 0.8,
            (active - current_active) * 0.010,
        ))

    active_lower = lower[[1, 2, 3]]
    active_upper = upper[[1, 2, 3]]
    result = least_squares(
        residual,
        np.clip(current_active, active_lower, active_upper),
        bounds=(active_lower, active_upper),
        max_nfev=250,
    )
    positions = compose(result.x)
    pose = tip_transform(positions)
    position_error = float(np.linalg.norm(pose[:3, 3] - target))
    dot = float(np.clip(np.dot(pose[:3, 2], current_axis), -1.0, 1.0))
    axis_error_deg = math.degrees(math.acos(dot))
    return {
        "success": bool(result.success and position_error <= 0.008),
        "positions": {
            name: float(value) for name, value in zip(JOINT_NAMES, positions)
        },
        "tip_position": pose[:3, 3].tolist(),
        "tool_axis": pose[:3, 2].tolist(),
        "position_error_m": position_error,
        "axis_error_deg": axis_error_deg,
        "orientation_constrained": False,
        "orientation_status": "SOFT_PRESERVE_CURRENT_TOOL_AXIS",
        "solver_status": int(result.status),
        "solver_message": str(result.message),
        "collision_checked": False,
        "tip_frame": "arm/touch_tip (operator-confirmed gripper_frame_link origin)",
        "prepare_strategy": "NEAR_FORWARD_LIFT_ELBOW_WRIST_FLEX",
        "primary_joints": ["shoulder_lift_joint", "elbow_flex_joint"],
        "compensation_joint": "wrist_flex_joint",
        "locked_joints": ["shoulder_pan_joint", "wrist_roll_joint"],
    }


def solve_pre_touch(
    current_positions: Iterable[float], target_position: Iterable[float],
    outward_normal: Iterable[float],
    lower_limits: Iterable[float] | None = None,
    upper_limits: Iterable[float] | None = None,
    reference_sector: str | None = None,
    reference_stage: str | None = None,
    locked_reference_id: str | None = None,
) -> dict:
    current = np.asarray(tuple(current_positions), dtype=float)
    target = np.asarray(tuple(target_position), dtype=float)
    normal = np.asarray(tuple(outward_normal), dtype=float)
    normal /= max(float(np.linalg.norm(normal)), 1.0e-12)
    lower = LOWER if lower_limits is None else np.asarray(tuple(lower_limits), dtype=float)
    upper = UPPER if upper_limits is None else np.asarray(tuple(upper_limits), dtype=float)
    if lower.shape != (5,) or upper.shape != (5,) or np.any(lower >= upper):
        raise ValueError("five valid lower/upper joint limits are required")
    # Touch does not need tool roll. Keeping wrist roll near neutral leaves the
    # side-mounted camera in a repeatable orientation and makes the rigid
    # lower-tip image coordinate usable for visual servoing.
    lower = lower.copy()
    upper = upper.copy()
    lower[4] = max(lower[4], TOUCH_WRIST_ROLL_TARGET_RAD - TOUCH_WRIST_ROLL_TOLERANCE_RAD)
    upper[4] = min(upper[4], TOUCH_WRIST_ROLL_TARGET_RAD + TOUCH_WRIST_ROLL_TOLERANCE_RAD)
    if lower[4] >= upper[4]:
        raise ValueError("effective limits do not include the neutral Touch wrist-roll pose")
    def residual(q: np.ndarray, prepare_reference: np.ndarray) -> np.ndarray:
        pose = tip_transform(q)
        position_error = pose[:3, 3] - target
        # touch_tip +Z follows the jaw's forward direction. Five arm joints are
        # sufficient for 3-D position plus a 2-DOF tool-axis direction; roll
        # about that direction is deliberately unconstrained.
        axis_error = np.cross(pose[:3, 2], normal)
        return np.concatenate((position_error * 200.0, axis_error * 1.5,
                               (q - current) * 0.010,
                               (q - prepare_reference) * 0.004,
                               # Prefer the recorded wrist-flex view for the
                               # selected height/sector so the side camera
                               # remains approximately level and forward.
                               [(q[3] - prepare_reference[3]) * 0.08]))

    # A measured prepare reference must always be selected because its paired
    # D435 pan/tilt pose is part of the post-preview observation gate.  The
    # current pose remains in the residual as a small continuity cost, but is
    # not an unnamed seed without camera evidence.
    eligible_entries = list(PREPARE_REFERENCE_SEEDS)
    if locked_reference_id is not None:
        eligible_entries = [
            item for item in eligible_entries if item[0] == locked_reference_id
        ]
    else:
        if reference_sector is not None:
            sector = reference_sector.strip().lower()
            if sector not in {"front", "left", "right"}:
                raise ValueError(f"invalid prepare reference sector: {reference_sector}")
            eligible_entries = [
                item for item in eligible_entries if item[0].startswith(f"{sector}_")
            ]
        if reference_stage is not None:
            stage = reference_stage.strip().lower()
            if stage not in {"far", "near"}:
                raise ValueError(f"invalid prepare reference stage: {reference_stage}")
            # Lateral references were explicitly captured for BOTH stages.
            eligible_entries = [
                item for item in eligible_entries
                if item[0].startswith(("left_", "right_")) or item[0].endswith(f"_{stage}")
            ]
        # Height is selected from measured geometry, not from optimizer cost.
        # This prevents a FRONT target from borrowing a LEFT/RIGHT camera pose.
        if eligible_entries:
            nearest = min(
                eligible_entries,
                key=lambda item: float(np.linalg.norm(
                    tip_transform(item[1])[:3, 3] - target
                )),
            )
            eligible_entries = [nearest]
    if not eligible_entries:
        raise ValueError(
            "no operator prepare reference matches the requested sector/stage"
        )

    named_seeds = [
        (name, np.clip(np.asarray(values, dtype=float), lower, upper),
         np.clip(np.asarray(values, dtype=float), lower, upper))
        for name, values, _pan, _tilt in eligible_entries
    ]
    candidates = [
        (name, reference, least_squares(
            lambda q, ref=reference: residual(q, ref), seed,
            bounds=(lower, upper), max_nfev=300,
        ))
        for name, seed, reference in named_seeds
    ]
    selected_name, selected_reference, result = min(
        candidates,
        key=lambda item: float(np.linalg.norm(residual(item[2].x, item[1]))),
    )
    selected_entry = next(item for item in PREPARE_REFERENCE_SEEDS if item[0] == selected_name)
    pose = tip_transform(result.x)
    position_error = float(np.linalg.norm(pose[:3, 3] - target))
    dot = float(np.clip(np.dot(pose[:3, 2], normal), -1.0, 1.0))
    axis_error_deg = math.degrees(math.acos(dot))
    success = bool(result.success and position_error <= 0.008)
    return {
        "success": success,
        "positions": {name: float(value) for name, value in zip(JOINT_NAMES, result.x)},
        "tip_position": pose[:3, 3].tolist(),
        "tool_axis": pose[:3, 2].tolist(),
        "position_error_m": position_error,
        "axis_error_deg": axis_error_deg,
        "orientation_constrained": False,
        "orientation_status": "SOFT_HORIZONTAL_FORWARD_PREFERENCE",
        "wrist_roll_target_rad": TOUCH_WRIST_ROLL_TARGET_RAD,
        "wrist_roll_error_rad": float(result.x[4] - TOUCH_WRIST_ROLL_TARGET_RAD),
        "wrist_flex_policy": "PREFER_SELECTED_OPERATOR_REFERENCE_CAMERA_VIEW",
        "joint_limits_source": (
            "ARM_GATEWAY_EFFECTIVE_LIMITS" if lower_limits is not None
            else "URDF_NOMINAL_LIMITS"
        ),
        "solver_status": int(result.status),
        "solver_message": str(result.message),
        "collision_checked": False,
        "tip_frame": "arm/touch_tip (operator-confirmed gripper_frame_link origin)",
        "prepare_reference_id": selected_name,
        "prepare_camera_pan_deg": float(selected_entry[2]),
        "prepare_camera_tilt_deg": float(selected_entry[3]),
        "prepare_reference_distance_rad": float(np.linalg.norm(result.x - selected_reference)),
        "prepare_reference_count": len(eligible_entries),
        "prepare_reference_sector": reference_sector,
        "prepare_reference_stage": reference_stage,
        "prepare_reference_locked": locked_reference_id is not None,
        "prepare_strategy": "GEOMETRY_CLASSIFIED_OPERATOR_REFERENCE_BOUNDED_IK",
        "prepare_disposition": (
            "COMFORTABLE" if success else "REQUIRES_ARM_BASE_REPOSITION"
        ),
    }
