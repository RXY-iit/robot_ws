#!/usr/bin/env python3
"""Audit/re-fit the camera mechanism against captured dual-camera ChArUco data.

The revised model fixes facts established by physical measurement:
* SO101 base Z is 1.005 m at the physical Lift lower limit;
* Pan and Tilt move only the D435, never the Lift/SO101;
* Pan/Tilt axis-origin heights are 0.735/0.770 m;
* the public so-frame wrist optical transform is a fixed prior, not a fit value.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


SESSIONS = (
    "session_20260903_010858_115137",
    "session_20260903_014710_255810",
    "session_20260903_020201_959088",
    "session_20260903_023525_587586",
)
BASE_LINK_GROUND_Z = 0.1125
ARM_MOUNT = np.array([
    0.32084, 0.01262, 1.00500 - BASE_LINK_GROUND_Z, 0.0, 0.0, -0.01532,
])
PUBLIC_GRIPPER_TO_ARM_CAMERA = np.array([
    # Public translation plus explicit SAPIEN-camera -> ROS REP-103 optical
    # axis conversion.  The source frame is +X forward, -Y right, +Z up;
    # ROS optical is +Z forward, +X right, +Y down.
    0.0025, 0.0675, -0.0062, 2.70710133, 0.0, -0.000000673205,
])
# Exact fixed D435 macro transform from its bottom-screw frame to the color
# optical frame.  This must not be absorbed into the external holder fit.
HOLDER_TO_D435_OPTICAL = np.array([
    0.0106, 0.0325, 0.0125, -math.pi / 2.0, 0.0, -math.pi / 2.0,
])
PAN_REFERENCE = 267.1
TILT_REFERENCE = 101.8
PAN_LINEAR = 0.0170137853
PAN_QUADRATIC = -0.00000617291927
TILT_LINEAR = 0.0160077655
TILT_QUADRATIC = -0.00000578161438
PAN_TO_TILT_BASE_DELTA = np.array([0.025, 0.0, 0.035])
TOUCH_HEIGHT_REFERENCES = (
    # pan_deg, tilt_deg, measured arm/base_link minus bottom-screw ground Z
    (270.97, 32.78, 0.235),
    (270.97, 45.18, 0.242),
    (270.70, 12.39, 0.226),
    (271.00, 32.80, 0.236),
    (267.01, 2.90, 0.223),
    (266.75, 11.25, 0.225),
)

ARM_REFERENCE = np.array([
    0.03217642458100545, 1.74533, -0.295563703024748,
    -0.0373075230972284, -1.2867643467643466,
])
ARM_SCALE = np.array([1.07092812, 0.98232268, 0.97438663, 1.05793015, 1.37093532])
ARM_VISUAL_OFFSET = np.array([0.0, 0.0, 0.0, 0.0, -0.33725009])
ARM_CHAIN = (
    ((0.0388353, -8.97657e-09, 0.0624), (math.pi, 0.0, -math.pi)),
    ((-0.0303992, -0.0182778, -0.0542), (-math.pi / 2, -math.pi / 2, 0.0)),
    ((-0.11257, -0.028, 0.0), (0.0, 0.0, math.pi / 2)),
    ((-0.1349, 0.0052, 0.0), (0.0, 0.0, -1.57)),
    ((0.0, -0.0611, 0.0181), (-1.57, 3.14, 0.0)),
)


def tf(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    result[:3, 3] = xyz
    return result


def rz(angle: float) -> np.ndarray:
    return tf(rpy=(0.0, 0.0, angle))


def calibrated_angle(raw: float, reference: float, linear: float, quadratic: float) -> float:
    delta = raw - reference
    return linear * delta + quadratic * delta * delta


def arm_base_to_camera(joints: list[float], image_axis_correction: np.ndarray) -> np.ndarray:
    visual = ARM_REFERENCE + ARM_SCALE * (np.asarray(joints[:5]) - ARM_REFERENCE) + ARM_VISUAL_OFFSET
    result = np.eye(4)
    for (xyz, rpy), angle in zip(ARM_CHAIN, visual):
        result = result @ tf(xyz, rpy) @ rz(float(angle))
    public_camera = tf(
        PUBLIC_GRIPPER_TO_ARM_CAMERA[:3], PUBLIC_GRIPPER_TO_ARM_CAMERA[3:]
    )
    # The public model fixes the physical camera origin.  A rotation-only
    # correction reconciles that model's optical convention with the current
    # UVC driver's image axes without moving the published camera centre.
    public_camera[:3, :3] = (
        public_camera[:3, :3]
        @ Rotation.from_rotvec(image_axis_correction).as_matrix()
    )
    return result @ public_camera


def pnp_pose(corners: dict, camera_info: dict, override_intrinsics: dict | None = None) -> np.ndarray:
    entries = corners["corners"]
    obj = np.asarray([entry["board_xyz_m"] for entry in entries], dtype=np.float64)
    image = np.asarray([entry["pixel_uv"] for entry in entries], dtype=np.float64)
    source = override_intrinsics or camera_info
    k = np.asarray(source["k"], dtype=np.float64).reshape(3, 3)
    d = np.asarray(source.get("d", []), dtype=np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, image, k, d, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise RuntimeError("solvePnP failed")
    result = np.eye(4)
    result[:3, :3] = cv2.Rodrigues(rvec)[0]
    result[:3, 3] = tvec[:, 0]
    return result


def load_samples(root: Path, arm_intrinsics: dict) -> list[dict]:
    result = []
    for session_name in SESSIONS:
        session = root / session_name
        for directory in sorted(session.glob("sample_*")):
            metadata_file = directory / "metadata.json"
            if not metadata_file.exists():
                continue
            metadata = json.loads(metadata_file.read_text())
            state = metadata.get("arm_joint_state_at_capture") or {}
            names = state.get("name") or []
            positions = state.get("position_rad") or []
            by_name = dict(zip(names, positions))
            required = [
                "shoulder_pan_joint", "shoulder_lift_joint", "elbow_flex_joint",
                "wrist_flex_joint", "wrist_roll_joint",
            ]
            if not all(name in by_name for name in required):
                continue
            try:
                d435_board = pnp_pose(
                    json.loads((directory / "charuco.json").read_text()),
                    metadata["camera_info"]["color"],
                )
                arm_board = pnp_pose(
                    json.loads((directory / "arm_charuco.json").read_text()),
                    metadata["camera_info"]["arm_color"], arm_intrinsics,
                )
            except (KeyError, RuntimeError, cv2.error):
                continue
            result.append({
                "id": f"{session_name}/{directory.name}",
                "pan": float(metadata["feedback"]["pan"]["value_raw"]),
                "tilt": float(metadata["feedback"]["tilt"]["value_raw"]),
                "joints": [float(by_name[name]) for name in required],
                "observed_d435_from_arm": d435_board @ np.linalg.inv(arm_board),
            })
    return result


def mechanism(params: np.ndarray, pan_raw: float, tilt_raw: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    pan_origin = np.array([params[0], params[1], 0.735 - BASE_LINK_GROUND_Z])
    pan_rotation = Rotation.from_rotvec(params[2:5]).as_matrix()
    tilt_relative_rotation = Rotation.from_rotvec(params[5:8]).as_matrix()
    holder = tf(params[8:11], params[11:14])
    pan_angle = calibrated_angle(pan_raw, PAN_REFERENCE, PAN_LINEAR, PAN_QUADRATIC)
    tilt_angle = calibrated_angle(tilt_raw, TILT_REFERENCE, TILT_LINEAR, TILT_QUADRATIC)
    pan_at_angle = tf(pan_origin, Rotation.from_matrix(pan_rotation).as_euler("xyz")) @ rz(pan_angle)
    pan_to_tilt = pan_rotation.T @ PAN_TO_TILT_BASE_DELTA
    tilt_at_angle = pan_at_angle @ tf(
        pan_to_tilt, Rotation.from_matrix(tilt_relative_rotation).as_euler("xyz")
    ) @ rz(tilt_angle)
    return pan_at_angle, tilt_at_angle, tilt_at_angle @ holder


def residual(params: np.ndarray, samples: list[dict]) -> np.ndarray:
    errors = []
    arm_mount = tf(ARM_MOUNT[:3], ARM_MOUNT[3:])
    for sample in samples:
        _pan, _tilt, holder = mechanism(params, sample["pan"], sample["tilt"])
        d435 = holder @ tf(HOLDER_TO_D435_OPTICAL[:3], HOLDER_TO_D435_OPTICAL[3:])
        arm_camera = arm_mount @ arm_base_to_camera(sample["joints"], params[14:17])
        predicted = np.linalg.inv(d435) @ arm_camera
        delta = np.linalg.inv(sample["observed_d435_from_arm"]) @ predicted
        errors.extend(delta[:3, 3] * 50.0)
        errors.extend(Rotation.from_matrix(delta[:3, :3]).as_rotvec() * 2.0)
    # Physical axes are perpendicular.  Their measured origin separation is
    # already exact by construction; only the axis dot product needs a gate.
    pan_axis = Rotation.from_rotvec(params[2:5]).as_matrix()[:, 2]
    tilt_axis = (
        Rotation.from_rotvec(params[2:5]).as_matrix()
        @ Rotation.from_rotvec(params[5:8]).as_matrix()
    )[:, 2]
    errors.append(float(np.dot(pan_axis, tilt_axis)) * 20.0)
    # Direct measurements gave absolute X/Z separations, not signed vectors.
    for tilt_raw, target in (
        (101.9, np.array([0.010, 0.023])),
        (57.0, np.array([0.023, 0.013])),
    ):
        _pan, tilt, holder = mechanism(params, 267.0, tilt_raw)
        delta = holder[:3, 3] - tilt[:3, 3]
        errors.extend((np.abs(delta[[0, 2]]) - target) * 200.0)
        errors.append(delta[1] * 100.0)
    # Touch Prepare measurements directly constrain absolute holder height.
    # Both stated values are above ground; convert the predicted base_link Z
    # back to ground before comparing them.
    for pan_raw, tilt_raw, measured_delta_z in TOUCH_HEIGHT_REFERENCES:
        _pan, _tilt, holder = mechanism(params, pan_raw, tilt_raw)
        expected_ground_z = 1.005 - measured_delta_z
        predicted_ground_z = holder[2, 3] + BASE_LINK_GROUND_Z
        errors.append((predicted_ground_z - expected_ground_z) * 300.0)
    return np.asarray(errors)


def summarize(params: np.ndarray, samples: list[dict]) -> dict:
    arm_mount = tf(ARM_MOUNT[:3], ARM_MOUNT[3:])
    translation = []
    rotation = []
    for sample in samples:
        _pan, _tilt, holder = mechanism(params, sample["pan"], sample["tilt"])
        d435 = holder @ tf(HOLDER_TO_D435_OPTICAL[:3], HOLDER_TO_D435_OPTICAL[3:])
        predicted = np.linalg.inv(d435) @ (
            arm_mount @ arm_base_to_camera(sample["joints"], params[14:17])
        )
        delta = np.linalg.inv(sample["observed_d435_from_arm"]) @ predicted
        translation.append(float(np.linalg.norm(delta[:3, 3])))
        rotation.append(float(np.linalg.norm(Rotation.from_matrix(delta[:3, :3]).as_rotvec())))
    pan, tilt, holder = mechanism(params, PAN_REFERENCE, TILT_REFERENCE)
    pan_axis = pan[:3, :3][:, 2]
    tilt_axis = tilt[:3, :3][:, 2]
    return {
        "sample_count": len(samples),
        "translation_rms_m": float(np.sqrt(np.mean(np.square(translation)))),
        "translation_median_m": float(np.median(translation)),
        "translation_max_m": float(np.max(translation)),
        "rotation_rms_deg": math.degrees(float(np.sqrt(np.mean(np.square(rotation))))),
        "rotation_median_deg": math.degrees(float(np.median(rotation))),
        "rotation_max_deg": math.degrees(float(np.max(rotation))),
        "arm_mount_at_lift_lower": ARM_MOUNT.tolist(),
        "base_link_height_above_ground_m": BASE_LINK_GROUND_Z,
        "arm_base_height_above_ground_m": 1.005,
        "public_gripper_to_arm_camera_optical_xyz_rpy": PUBLIC_GRIPPER_TO_ARM_CAMERA.tolist(),
        "uvc_image_axis_correction_rotvec": params[14:17].tolist(),
        "uvc_image_axis_correction_deg": math.degrees(float(np.linalg.norm(params[14:17]))),
        "d435_bottom_screw_to_color_optical_xyz_rpy": HOLDER_TO_D435_OPTICAL.tolist(),
        "pan_origin_xyz_m": pan[:3, 3].tolist(),
        "pan_axis_unit": pan_axis.tolist(),
        "tilt_origin_at_reference_xyz_m": tilt[:3, 3].tolist(),
        "tilt_axis_at_reference_unit": tilt_axis.tolist(),
        "axis_dot_product": float(np.dot(pan_axis, tilt_axis)),
        "holder_from_tilt_xyz_m": params[8:11].tolist(),
        "holder_from_tilt_rpy_rad": params[11:14].tolist(),
        "holder_at_reference_xyz_m": holder[:3, 3].tolist(),
        "optimizer_parameters": params.tolist(),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-root", type=Path, default=Path(__file__).parent / "output")
    parser.add_argument(
        "--arm-intrinsics", type=Path,
        default=Path(__file__).parents[2] / "src/robot_bringup/config/arm_camera_calibrated.yaml",
    )
    parser.add_argument("--report", type=Path, default=Path("/tmp/revised_tf_audit.json"))
    args = parser.parse_args()
    loaded = yaml.safe_load(args.arm_intrinsics.read_text())
    camera = loaded.get("camera", loaded)
    arm_intrinsics = {
        "k": camera["camera_matrix"]["data"],
        "d": camera["distortion_coefficients"]["data"],
    }
    samples = load_samples(args.output_root, arm_intrinsics)
    if not samples:
        raise SystemExit("No valid dual-camera samples")
    # Initial axes follow the previous fitted directions, but all axis points
    # and the wrist-camera prior use the revised physical model.
    # Deterministic seed from the previous physically constrained result.
    initial = np.array([
        0.263022643723, -0.005840533771,
        -0.161261593283, 0.528889458473, -0.302624144749,
        -1.584178220242, 0.153230125481, 0.621354970558,
        0.010867378612, -0.033386726487, -0.000309485129,
        1.537099461227, -0.026728694167, -0.308047366471,
        -0.164520087664, -0.083524257192, 0.106431350875,
    ])
    solved = least_squares(
        residual, initial, args=(samples,), loss="soft_l1", f_scale=0.5,
        max_nfev=1500, verbose=1,
    )
    report = summarize(solved.x, samples)
    report.update({
        "deployment_status": "OFFLINE_CANDIDATE_REQUIRES_REAL_VALIDATION",
        "absolute_base_extrinsic_observable": False,
        "absolute_base_extrinsic_limitation": (
            "Captured board poses were not independently measured in base_link; "
            "the dataset constrains relative dual-camera geometry but cannot by "
            "itself certify an absolute base_link extrinsic."
        ),
        "optimizer_success": bool(solved.success),
        "optimizer_status": int(solved.status),
        "optimizer_message": str(solved.message),
        "mechanism_fit_mode": "REFIT_WITH_ABSOLUTE_GROUND_HEIGHT_CONSTRAINTS",
        "source_sessions": list(SESSIONS),
        "wrist_camera_model_source": {
            "repository": "https://github.com/livekit-examples/so-frame",
            "commit": "657fb4a0bf65bac8f7b336deed2631a4932311c2",
            "urdf": "simulation/urdf/so101_on_frame.urdf",
        },
    })
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
