from __future__ import annotations

import time

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
        self.node.create_subscription(Float32, "/chokudomotor/angle", self._pan_feedback_cb, 10)
        self.node.create_subscription(Float32, "/cameraswingmotor/angle", self._tilt_feedback_cb, 10)

    def _pan_feedback_cb(self, msg: Float32):
        self._pan_feedback = float(msg.data)

    def _tilt_feedback_cb(self, msg: Float32):
        self._tilt_feedback = float(msg.data)

    def set_camera_pose(
        self,
        context: SkillContext,
        pan_deg: float,
        tilt_deg: float,
        wait_timeout_sec: float = 1.0,
        tolerance_deg: float = 2.0,
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
        command_time = time.monotonic()
        self.pan_pub.publish(Float32(data=float(pan_deg)))
        self.tilt_pub.publish(Float32(data=float(tilt_deg)))
        deadline = time.monotonic() + max(0.0, wait_timeout_sec)
        while time.monotonic() < deadline:
            if self._feedback_at_target(pan_deg, tilt_deg, tolerance_deg):
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
            True,
            codes.COMMAND_ACCEPTED,
            "Camera pose command published, but target feedback was not confirmed before timeout.",
            started,
            {
                "pan_deg": pan_deg,
                "tilt_deg": tilt_deg,
                "pan_feedback": self._pan_feedback,
                "tilt_feedback": self._tilt_feedback,
                "waited_for_feedback": True,
                "feedback_timeout_sec": wait_timeout_sec,
                "command_time_monotonic": command_time,
                "completed_time_monotonic": completed_time,
            },
        )

    def _feedback_at_target(self, pan_deg: float, tilt_deg: float, tolerance_deg: float) -> bool:
        if self._pan_feedback is None or self._tilt_feedback is None:
            return False
        return (
            abs(self._pan_feedback - float(pan_deg)) <= tolerance_deg
            and abs(self._tilt_feedback - float(tilt_deg)) <= tolerance_deg
        )
