from __future__ import annotations

import os
import time
from pathlib import Path

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

from std_msgs.msg import Float32

from robot_skills import result_codes as codes
from robot_skills.executors.base import make_result, now
from robot_skills.schemas import SkillContext, SkillResult


class CameraSkills:
    def __init__(self, node):
        self.node = node
        self.pan_pub = node.create_publisher(Float32, "/chokudomotor/target_angle", 10)
        self.tilt_pub = node.create_publisher(Float32, "/cameraswingmotor/target_angle", 10)
        self._pan_feedback: float | None = None
        self._tilt_feedback: float | None = None
        self._pan_feedback_time = 0.0
        self._tilt_feedback_time = 0.0
        self._real_pan_neutral_deg = float(
            node.declare_parameter("real_camera_pan_neutral_deg", 267.0).value
        )
        self._real_tilt_neutral_deg = float(
            node.declare_parameter("real_camera_tilt_neutral_deg", 102.0).value
        )
        self._logical_pan_neutral_deg = float(
            node.declare_parameter("logical_camera_pan_neutral_deg", 0.0).value
        )
        self._logical_tilt_neutral_deg = float(
            node.declare_parameter("logical_camera_tilt_neutral_deg", 67.0).value
        )
        # Bounds include the measured right/up (213 deg) and left/up (334 deg)
        # poses, with only a small margin beyond the verified real positions.
        self._real_pan_min_deg = float(node.declare_parameter("real_camera_pan_min_deg", 210.0).value)
        self._real_pan_max_deg = float(node.declare_parameter("real_camera_pan_max_deg", 335.0).value)
        self._real_tilt_min_deg = float(node.declare_parameter("real_camera_tilt_min_deg", 10.0).value)
        self._real_tilt_max_deg = float(node.declare_parameter("real_camera_tilt_max_deg", 120.0).value)
        self._real_pose_config = self._load_real_pose_config()
        self.node.create_subscription(Float32, "/chokudomotor/angle", self._pan_feedback_cb, 10)
        self.node.create_subscription(Float32, "/cameraswingmotor/angle", self._tilt_feedback_cb, 10)

    def _pan_feedback_cb(self, msg: Float32):
        self._pan_feedback = float(msg.data)
        self._pan_feedback_time = time.monotonic()

    def _tilt_feedback_cb(self, msg: Float32):
        self._tilt_feedback = float(msg.data)
        self._tilt_feedback_time = time.monotonic()

    def set_camera_pose(
        self,
        context: SkillContext,
        pan_deg: float,
        tilt_deg: float,
        wait_timeout_sec: float = 1.0,
        tolerance_deg: float = 2.0,
        scan_band: str | None = None,
        require_feedback: bool = False,
    ) -> SkillResult:
        started = now()
        if not (-180.0 <= pan_deg <= 180.0 and -90.0 <= tilt_deg <= 132.0):
            return make_result(
                "set_camera_pose",
                False,
                codes.FAILED_SCHEMA,
                "Camera target angle is out of configured bounds.",
                started,
                {"pan_deg": pan_deg, "tilt_deg": tilt_deg},
            )
        if context.mode == "dry_run":
            return make_result(
                "set_camera_pose",
                True,
                codes.DRY_RUN_OK,
                "Dry-run camera pose accepted.",
                started,
                {"pan_deg": pan_deg, "tilt_deg": tilt_deg},
            )
        hardware_pan_deg, hardware_tilt_deg, pose_source = self._hardware_targets_for_pose(
            context.mode, pan_deg, tilt_deg, scan_band
        )
        if context.mode == "real":
            # A navigation-home command is safe in AUTO only before base motion;
            # the navigation plan gates it with await_robot_stationary. All other
            # camera movement continues to require MANUAL mode.
            navigation_home = scan_band == "return_home"
            if context.robot_mode != "MANUAL" and not (
                navigation_home and context.robot_mode == "AUTO"
            ):
                return make_result(
                    "set_camera_pose",
                    False,
                    codes.NEEDS_HUMAN,
                    "Real camera motion requires MANUAL mode. Press B and retry.",
                    started,
                    {"robot_mode": context.robot_mode},
                )
            if self.pan_pub.get_subscription_count() < 1 or self.tilt_pub.get_subscription_count() < 1:
                return make_result(
                    "set_camera_pose",
                    False,
                    codes.FAILED_PRECHECK,
                    "Real camera motor driver is not subscribed to the target topics.",
                    started,
                )
            feedback_age = max(
                time.monotonic() - self._pan_feedback_time if self._pan_feedback_time else float("inf"),
                time.monotonic() - self._tilt_feedback_time if self._tilt_feedback_time else float("inf"),
            )
            if feedback_age > 1.0:
                return make_result(
                    "set_camera_pose",
                    False,
                    codes.FAILED_PRECHECK,
                    "Real camera feedback is missing or stale.",
                    started,
                    {"feedback_age_sec": feedback_age},
                )
            if not (
                self._real_pan_min_deg <= hardware_pan_deg <= self._real_pan_max_deg
                and self._real_tilt_min_deg <= hardware_tilt_deg <= self._real_tilt_max_deg
            ):
                return make_result(
                    "set_camera_pose",
                    False,
                    codes.FAILED_SCHEMA,
                    "Converted real camera target is outside the hardware safety bounds.",
                    started,
                    {
                        "logical_pan_deg": pan_deg,
                        "logical_tilt_deg": tilt_deg,
                        "hardware_pan_deg": hardware_pan_deg,
                        "hardware_tilt_deg": hardware_tilt_deg,
                        "scan_band": scan_band,
                        "pose_source": pose_source,
                    },
                )
        command_time = time.monotonic()
        self.pan_pub.publish(Float32(data=float(hardware_pan_deg)))
        self.tilt_pub.publish(Float32(data=float(hardware_tilt_deg)))
        deadline = time.monotonic() + max(0.0, wait_timeout_sec)
        while time.monotonic() < deadline:
            if self._feedback_at_target(hardware_pan_deg, hardware_tilt_deg, tolerance_deg):
                completed_time = time.monotonic()
                return make_result(
                    "set_camera_pose",
                    True,
                    codes.OK,
                    "Camera pose reached target feedback window.",
                    started,
                    {
                        "pan_deg": pan_deg,
                        "tilt_deg": tilt_deg,
                        "hardware_pan_deg": hardware_pan_deg,
                        "hardware_tilt_deg": hardware_tilt_deg,
                        "scan_band": scan_band,
                        "pose_source": pose_source,
                        "pan_feedback": self._pan_feedback,
                        "tilt_feedback": self._tilt_feedback,
                        "waited_for_feedback": True,
                        "command_time_monotonic": command_time,
                        "completed_time_monotonic": completed_time,
                    },
                )
            time.sleep(0.02)
        completed_time = time.monotonic()
        return make_result(
            "set_camera_pose",
            not require_feedback,
            codes.FAILED_TIMEOUT if require_feedback else codes.COMMAND_ACCEPTED,
            (
                "Camera pose did not reach the required feedback window before timeout."
                if require_feedback
                else "Camera pose command published, but target feedback was not confirmed before timeout."
            ),
            started,
            {
                "pan_deg": pan_deg,
                "tilt_deg": tilt_deg,
                "hardware_pan_deg": hardware_pan_deg,
                "hardware_tilt_deg": hardware_tilt_deg,
                "scan_band": scan_band,
                "pose_source": pose_source,
                "pan_feedback": self._pan_feedback,
                "tilt_feedback": self._tilt_feedback,
                "waited_for_feedback": True,
                "feedback_timeout_sec": wait_timeout_sec,
                "command_time_monotonic": command_time,
                "completed_time_monotonic": completed_time,
            },
        )

    def _hardware_targets(self, mode: str, pan_deg: float, tilt_deg: float) -> tuple[float, float]:
        if mode != "real":
            return float(pan_deg), float(tilt_deg)
        return (
            self._real_pan_neutral_deg + (float(pan_deg) - self._logical_pan_neutral_deg),
            self._real_tilt_neutral_deg + (float(tilt_deg) - self._logical_tilt_neutral_deg),
        )

    def _hardware_targets_for_pose(
        self,
        mode: str,
        pan_deg: float,
        tilt_deg: float,
        scan_band: str | None,
    ) -> tuple[float, float, str]:
        if mode == "real" and scan_band:
            calibration = self._real_pose_config.get("calibration") or {}
            pose = (self._real_pose_config.get("poses") or {}).get(str(scan_band)) or {}
            if calibration.get("use_calibrated_poses") is True:
                hardware_pan = pose.get("hardware_pan_deg")
                hardware_tilt = pose.get("hardware_tilt_deg")
                if hardware_pan is not None and hardware_tilt is not None:
                    return float(hardware_pan), float(hardware_tilt), "real_camera_poses"
        hardware_pan, hardware_tilt = self._hardware_targets(mode, pan_deg, tilt_deg)
        return hardware_pan, hardware_tilt, "neutral_offset_fallback" if mode == "real" else "logical"

    @staticmethod
    def _load_real_pose_config() -> dict:
        override = os.environ.get("ROBOT_REAL_CAMERA_POSES", "").strip()
        candidates: list[Path] = []
        if override:
            candidates.append(Path(override).expanduser())
        try:
            candidates.append(Path(get_package_share_directory("robot_skills")) / "config" / "real_camera_poses.yaml")
        except PackageNotFoundError:
            pass
        candidates.append(Path(__file__).resolve().parents[2] / "config" / "real_camera_poses.yaml")
        for path in candidates:
            if path.is_file():
                try:
                    return yaml.safe_load(path.read_text(encoding="utf-8")) or {}
                except Exception:
                    return {}
        return {}

    def _feedback_at_target(self, pan_deg: float, tilt_deg: float, tolerance_deg: float) -> bool:
        if self._pan_feedback is None or self._tilt_feedback is None:
            return False
        return (
            abs(self._pan_feedback - float(pan_deg)) <= tolerance_deg
            and abs(self._tilt_feedback - float(tilt_deg)) <= tolerance_deg
        )
