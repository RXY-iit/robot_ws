"""Validate a bounded odometry-propagated localization seed, without ROS effects."""

import math


def propagate_base_pose(anchor, odom_anchor, odom_now):
    mx, my, myaw = anchor
    ox, oy, oyaw = odom_anchor
    nx, ny, nyaw = odom_now
    angle = myaw - oyaw
    return (mx + math.cos(angle) * (nx-ox) - math.sin(angle) * (ny-oy),
            my + math.sin(angle) * (nx-ox) + math.cos(angle) * (ny-oy),
            math.atan2(math.sin(myaw+nyaw-oyaw), math.cos(myaw+nyaw-oyaw)))


def trusted_seed(metrics: dict, config: dict, report_age: float) -> tuple[float, float, float]:
    def bounded_age(key: str, limit: float, source_time: bool = False) -> None:
        age = metrics.get(key)
        extra_age = 0 if source_time else report_age
        if not isinstance(age, (int, float)) or not math.isfinite(age) or age < 0 or age + extra_age > limit:
            raise ValueError(f"{key} missing or stale; manual initial pose required")

    if not math.isfinite(report_age) or report_age < 0 or report_age > float(config.get("max_report_age_sec", 4)):
        raise ValueError("Localization health report is stale")
    bounded_age("last_trusted_pose_age_sec", float(config.get("max_anchor_age_sec", 120)))
    bounded_age("wheel_odom_age_sec", float(config.get("max_odom_age_sec", 3)))
    if metrics.get("trusted_pose_has_odom_anchor") is not True:
        raise ValueError("No wheel-odometry anchor for the trusted pose")
    if config.get("require_timestamped_base", False):
        if metrics.get("trusted_pose_frame") != "base_footprint":
            raise ValueError("Trusted seed must use a timestamped base pose, not a LiDAR pose")
        stamps = [metrics.get(k) for k in ("trusted_pose_stamp_sec", "wheel_odom_stamp_sec")]
        if not all(isinstance(v, (int, float)) and math.isfinite(v) and v > 0 for v in stamps) or stamps[1] < stamps[0]:
            raise ValueError("Invalid or reset odometry source timeline")
        # The backend rechecks the current TF stamp before publishing. Never add
        # wall-clock report age to an age measured in simulation seconds.
        bounded_age("wheel_odom_source_age_sec", float(config.get("max_odom_age_sec", 3)), source_time=True)
    pose = metrics.get("odom_propagated_trusted_pose")
    if not isinstance(pose, (list, tuple)) or len(pose) != 3 or not all(
        isinstance(v, (float, int)) and math.isfinite(v) for v in pose
    ):
        raise ValueError("No finite trusted pose; manual initial pose required")
    distance = metrics.get("trusted_pose_delta_m")
    yaw = metrics.get("trusted_pose_delta_yaw_rad")
    for value, limit in [(distance, float(config.get("max_odom_delta_m", 1))),
                         (yaw, float(config.get("max_odom_delta_yaw_rad", 0.5)))]:
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0 or value > limit:
            raise ValueError("Odometry displacement exceeds trusted-seed limits; use manual initial pose")
    return tuple(float(v) for v in pose)
