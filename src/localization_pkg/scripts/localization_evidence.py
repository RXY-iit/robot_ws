"""Time and geometry checks shared by localization and offline regression tests."""

import math

import numpy as np


def hint_is_current(scan_stamp, hint_stamp, hint_received, now_wall, max_skew=0.5, max_wall_age=2.0):
    values = (scan_stamp, hint_stamp, hint_received, now_wall)
    if not all(math.isfinite(v) for v in values) or min(scan_stamp, hint_stamp) <= 0:
        return False
    return abs(scan_stamp - hint_stamp) <= max_skew and 0 <= now_wall - hint_received <= max_wall_age


def matching_seed(map_odom, odom_lidar, fast_current=None, fast_reference=None, matched_reference=None):
    if all(v is not None for v in (fast_current, fast_reference, matched_reference)):
        return matched_reference @ np.linalg.inv(fast_reference) @ fast_current
    return map_odom @ odom_lidar


def registration_valid(rmse, fitness, correspondences, transform, min_fitness=0.25, min_correspondences=30):
    return bool(math.isfinite(rmse) and math.isfinite(fitness) and
                np.isfinite(transform).all() and fitness >= min_fitness and
                correspondences >= min_correspondences)


def hint_agrees_with_odometry(hint, wheel_seed, max_translation=0.75, max_angle=0.8):
    """Reject a divergent optional hint, never restrict a human initial pose."""
    if not np.isfinite(hint).all() or not np.isfinite(wheel_seed).all():
        return False
    distance = np.linalg.norm(hint[:3, 3] - wheel_seed[:3, 3])
    rotation = wheel_seed[:3, :3].T @ hint[:3, :3]
    angle = math.acos(float(np.clip((np.trace(rotation) - 1) / 2, -1, 1)))
    return bool(distance <= max_translation and angle <= max_angle)
