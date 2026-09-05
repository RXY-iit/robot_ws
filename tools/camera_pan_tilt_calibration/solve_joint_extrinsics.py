#!/usr/bin/env python3
"""Jointly solve D435 Pan/Tilt and SO101 wrist-camera extrinsics.

Raw ChArUco observations constrain the relative pose of the two cameras.  The
absolute Z gauge is supplied only by direct physical ground-height evidence;
unobservable base X/Y/yaw are held at their existing installation anchors.
The public SO-frame camera transform is reported for comparison only and never
enters the optimizer or its residuals.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

import audit_revised_tf as capture


BASE_LINK_GROUND_Z = 0.1125
ARM_MOUNT = np.array([0.32084, 0.01262, 1.005 - BASE_LINK_GROUND_Z, 0.0, 0.0, -0.01532])
PAN_Z = 0.735 - BASE_LINK_GROUND_Z
TILT_Z = 0.770 - BASE_LINK_GROUND_Z
PAN_TO_TILT_X = 0.025
HOLDER_TO_D435_OPTICAL = capture.HOLDER_TO_D435_OPTICAL

# The 2026-09-04 Touch Prepare measurements are direct Z-height differences.
TOUCH_HEIGHT_REFERENCES = capture.TOUCH_HEIGHT_REFERENCES


def tf(xyz=(0.0, 0.0, 0.0), rotation=None) -> np.ndarray:
    result = np.eye(4)
    result[:3, 3] = xyz
    if rotation is not None:
        result[:3, :3] = rotation.as_matrix()
    return result


def axis_motion(point: np.ndarray, axis: np.ndarray, angle: float) -> np.ndarray:
    rotation = Rotation.from_rotvec(axis * angle).as_matrix()
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = point - rotation @ point
    return result


def unit(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm < 1e-9:
        return np.array([0.0, 0.0, 1.0])
    return vector / norm


def axis_frame(point: np.ndarray, z_axis: np.ndarray) -> np.ndarray:
    """Choose a deterministic frame whose local +Z is the physical axis."""
    z_axis = unit(z_axis)
    reference = np.array([1.0, 0.0, 0.0])
    if abs(float(np.dot(reference, z_axis))) > 0.9:
        reference = np.array([0.0, 1.0, 0.0])
    x_axis = unit(reference - z_axis * float(np.dot(reference, z_axis)))
    y_axis = np.cross(z_axis, x_axis)
    result = np.eye(4)
    result[:3, :3] = np.column_stack((x_axis, y_axis, z_axis))
    result[:3, 3] = point
    return result


def axes(params: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    pan = unit(params[2:5])
    raw_tilt = params[5:8]
    tilt = unit(raw_tilt - pan * float(np.dot(raw_tilt, pan)))
    return pan, tilt


def unpack(params: np.ndarray):
    pan_axis, tilt_axis = axes(params)
    pan_point = np.array([params[0], params[1], PAN_Z])
    tilt_point = np.array([params[0] + PAN_TO_TILT_X, params[1], TILT_Z])
    holder_reference = tf(params[8:11], Rotation.from_rotvec(params[11:14]))
    arm_camera = tf(params[14:17], Rotation.from_rotvec(params[17:20]))
    return pan_point, pan_axis, tilt_point, tilt_axis, holder_reference, arm_camera


def d435_holder(params: np.ndarray, pan_raw: float, tilt_raw: float) -> np.ndarray:
    pan_point, pan_axis, tilt_point, tilt_axis, holder_reference, _ = unpack(params)
    pan_angle = capture.calibrated_angle(
        pan_raw, capture.PAN_REFERENCE, capture.PAN_LINEAR, capture.PAN_QUADRATIC
    )
    tilt_angle = capture.calibrated_angle(
        tilt_raw, capture.TILT_REFERENCE, capture.TILT_LINEAR, capture.TILT_QUADRATIC
    )
    return (
        axis_motion(pan_point, pan_axis, pan_angle)
        @ axis_motion(tilt_point, tilt_axis, tilt_angle)
        @ holder_reference
    )


def arm_base_to_gripper(joints: list[float]) -> np.ndarray:
    visual = (
        capture.ARM_REFERENCE
        + capture.ARM_SCALE * (np.asarray(joints[:5]) - capture.ARM_REFERENCE)
        + capture.ARM_VISUAL_OFFSET
    )
    result = np.eye(4)
    for (xyz, rpy), angle in zip(capture.ARM_CHAIN, visual):
        result = (
            result
            @ capture.tf(xyz, rpy)
            @ capture.rz(float(angle))
        )
    return result


def sample_error(params: np.ndarray, sample: dict) -> tuple[np.ndarray, np.ndarray]:
    *_, arm_camera = unpack(params)
    holder = d435_holder(params, sample["pan"], sample["tilt"])
    d435 = holder @ capture.tf(
        HOLDER_TO_D435_OPTICAL[:3], HOLDER_TO_D435_OPTICAL[3:]
    )
    predicted = (
        np.linalg.inv(d435)
        @ capture.tf(ARM_MOUNT[:3], ARM_MOUNT[3:])
        @ arm_base_to_gripper(sample["joints"])
        @ arm_camera
    )
    delta = np.linalg.inv(sample["observed_d435_from_arm"]) @ predicted
    return delta[:3, 3], Rotation.from_matrix(delta[:3, :3]).as_rotvec()


def physical_errors(params: np.ndarray) -> list[float]:
    pan_point, pan_axis, tilt_point, tilt_axis, holder_ref, _ = unpack(params)
    errors: list[float] = []
    # Axis origins/heights and perpendicularity are exact by parameterization.
    errors.append(float(np.dot(pan_axis, tilt_axis)) * 100.0)

    # Measured values were absolute X/Z separation, with approximate Y alignment.
    for tilt_raw, expected_abs_xz in (
        (101.9, np.array([0.010, 0.023])),
        (57.0, np.array([0.023, 0.013])),
    ):
        holder = d435_holder(params, 267.0, tilt_raw)
        pan_angle = capture.calibrated_angle(
            267.0, capture.PAN_REFERENCE, capture.PAN_LINEAR, capture.PAN_QUADRATIC
        )
        tilt_point_moved = axis_motion(pan_point, pan_axis, pan_angle) @ np.r_[tilt_point, 1.0]
        delta = holder[:3, 3] - tilt_point_moved[:3]
        errors.extend(((np.abs(delta[[0, 2]]) - expected_abs_xz) / 0.004).tolist())
        errors.append(float(delta[1]) / 0.008)

    # Direct absolute ground-height observations; allow 5 mm measurement noise.
    for pan_raw, tilt_raw, arm_minus_holder_z in TOUCH_HEIGHT_REFERENCES:
        holder = d435_holder(params, pan_raw, tilt_raw)
        expected_holder_ground_z = 1.005 - arm_minus_holder_z
        errors.append(
            (float(holder[2, 3]) + BASE_LINK_GROUND_Z - expected_holder_ground_z) / 0.005
        )
    return errors


def residual(params: np.ndarray, samples: list[dict], include_physical=True) -> np.ndarray:
    values: list[float] = []
    for sample in samples:
        translation, rotation = sample_error(params, sample)
        values.extend((translation / 0.012).tolist())
        values.extend((rotation / math.radians(2.0)).tolist())
    if include_physical:
        values.extend(physical_errors(params))
    return np.asarray(values)


def metrics(params: np.ndarray, samples: list[dict]) -> dict:
    translations = []
    rotations = []
    for sample in samples:
        translation, rotation = sample_error(params, sample)
        translations.append(float(np.linalg.norm(translation)))
        rotations.append(math.degrees(float(np.linalg.norm(rotation))))
    return {
        "count": len(samples),
        "translation_rms_m": float(np.sqrt(np.mean(np.square(translations)))),
        "translation_median_m": float(np.median(translations)),
        "translation_max_m": float(np.max(translations)),
        "rotation_rms_deg": float(np.sqrt(np.mean(np.square(rotations)))),
        "rotation_median_deg": float(np.median(rotations)),
        "rotation_max_deg": float(np.max(rotations)),
    }


def initial_parameters() -> np.ndarray:
    # Previous measured solution is only an optimizer seed, not a residual/prior.
    old_pan_origin = np.array([-0.10311226, -0.01622603, 0.0484708])
    old_pan_r = Rotation.from_euler(
        "xyz", [-0.046699431335, 0.577007773547, -0.025488422845]
    ).as_matrix()
    pan_axis = old_pan_r[:, 2]
    # Move the axis point along the same line to the measured Z plane.
    point = old_pan_origin + pan_axis * ((PAN_Z - old_pan_origin[2]) / pan_axis[2])
    old_tilt_r = old_pan_r @ Rotation.from_euler(
        "xyz", [-1.570796334434, -0.578051063699, -0.048776641774]
    ).as_matrix()
    tilt_axis = old_tilt_r[:, 2]

    # Reference holder pose from the last real-tested D435 model.
    old_holder = (
        capture.tf(old_pan_origin, [-0.046699431335, 0.577007773547, -0.025488422845])
        @ capture.tf(
            [-0.006783516432, 0.013179735657, 0.730546419919],
            [-1.570796334434, -0.578051063699, -0.048776641774],
        )
        @ capture.tf(
            [0.030726909918, 0.027085166017, -0.018056763343],
            [1.592333052027, -0.074371123979, 0.733543316960],
        )
    )
    # Bring only absolute Z near the direct measured range for initialization.
    old_holder[2, 3] = 1.005 - 0.232 - BASE_LINK_GROUND_Z
    arm_camera = capture.tf(
        [0.02980, 0.04821, -0.04671], [2.54576, 0.04301, -0.50527]
    )
    return np.r_[
        point[:2], pan_axis, tilt_axis,
        old_holder[:3, 3], Rotation.from_matrix(old_holder[:3, :3]).as_rotvec(),
        arm_camera[:3, 3], Rotation.from_matrix(arm_camera[:3, :3]).as_rotvec(),
    ]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-root", type=Path, default=Path(__file__).parent / "output")
    parser.add_argument(
        "--arm-intrinsics", type=Path,
        default=Path(__file__).parents[2] / "src/robot_bringup/config/arm_camera_calibrated.yaml",
    )
    parser.add_argument(
        "--report", type=Path,
        default=Path(__file__).parents[2] / "todo-my/arm-baseline/tf-updata/JOINT-EXTRINSIC-SOLVE-20260904.json",
    )
    args = parser.parse_args()
    loaded = yaml.safe_load(args.arm_intrinsics.read_text())
    camera = loaded.get("camera", loaded)
    intrinsics = {
        "k": camera["camera_matrix"]["data"],
        "d": camera["distortion_coefficients"]["data"],
    }
    samples = capture.load_samples(args.output_root, intrinsics)
    train = [sample for index, sample in enumerate(samples) if index % 5 != 0]
    validation = [sample for index, sample in enumerate(samples) if index % 5 == 0]

    solved = least_squares(
        residual, initial_parameters(), args=(train, True),
        loss="soft_l1", f_scale=1.0, max_nfev=3000, verbose=1,
    )
    pan_point, pan_axis, tilt_point, tilt_axis, holder, arm_camera = unpack(solved.x)
    pan_frame = axis_frame(pan_point, pan_axis)
    tilt_frame = axis_frame(tilt_point, tilt_axis)
    pan_to_tilt = np.linalg.inv(pan_frame) @ tilt_frame
    tilt_to_holder = np.linalg.inv(tilt_frame) @ holder
    report = {
        "schema_version": 1,
        "deployment_status": "CANDIDATE_NOT_DEPLOYED",
        "optimizer_success": bool(solved.success),
        "optimizer_message": str(solved.message),
        "sample_count": len(samples),
        "split": "deterministic every fifth sample held out",
        "train_metrics": metrics(solved.x, train),
        "validation_metrics": metrics(solved.x, validation),
        "all_metrics": metrics(solved.x, samples),
        "hard_facts": {
            "base_link_height_above_ground_m": BASE_LINK_GROUND_Z,
            "arm_base_height_above_ground_m": 1.005,
            "arm_mount_base_link_xyz_rpy": ARM_MOUNT.tolist(),
            "pan_axis_height_above_ground_m": 0.735,
            "tilt_axis_height_above_ground_m": 0.770,
            "pan_tilt_motion_does_not_move_arm": True,
        },
        "observability": {
            "absolute_base_xy_yaw": "FIXED_TO_EXISTING_INSTALLATION_ANCHOR",
            "absolute_board_pose": "NOT_MEASURED",
            "relative_dual_camera_extrinsic": "OBSERVABLE_FROM_SHARED_BOARD",
        },
        "solution": {
            "pan_axis_point_base_m": pan_point.tolist(),
            "pan_axis_unit_base": pan_axis.tolist(),
            "tilt_axis_point_reference_base_m": tilt_point.tolist(),
            "tilt_axis_unit_reference_base": tilt_axis.tolist(),
            "holder_reference_xyz_m": holder[:3, 3].tolist(),
            "holder_reference_rpy_rad": Rotation.from_matrix(holder[:3, :3]).as_euler("xyz").tolist(),
            "gripper_to_arm_camera_optical_xyz_m": arm_camera[:3, 3].tolist(),
            "gripper_to_arm_camera_optical_rpy_rad": Rotation.from_matrix(arm_camera[:3, :3]).as_euler("xyz").tolist(),
            "axis_dot_product": float(np.dot(pan_axis, tilt_axis)),
        },
        "urdf_solution": {
            "camera_pan_joint_xyz_m": pan_frame[:3, 3].tolist(),
            "camera_pan_joint_rpy_rad": Rotation.from_matrix(pan_frame[:3, :3]).as_euler("xyz").tolist(),
            "camera_tilt_joint_xyz_m": pan_to_tilt[:3, 3].tolist(),
            "camera_tilt_joint_rpy_rad": Rotation.from_matrix(pan_to_tilt[:3, :3]).as_euler("xyz").tolist(),
            "realsense_holder_joint_xyz_m": tilt_to_holder[:3, 3].tolist(),
            "realsense_holder_joint_rpy_rad": Rotation.from_matrix(tilt_to_holder[:3, :3]).as_euler("xyz").tolist(),
            "arm_camera_body_joint_xyz_m": arm_camera[:3, 3].tolist(),
            "arm_camera_body_joint_rpy_rad": Rotation.from_matrix(arm_camera[:3, :3]).as_euler("xyz").tolist(),
        },
        "physical_constraint_normalized_residuals": physical_errors(solved.x),
        "public_model_usage": "COMPARISON_ONLY_NOT_IN_OPTIMIZER",
        "source_sessions": list(capture.SESSIONS),
        "optimizer_parameters": solved.x.tolist(),
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
