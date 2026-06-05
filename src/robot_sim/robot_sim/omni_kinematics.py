import math
from typing import List, Tuple

import numpy as np


WHEEL_COUNT = 3
WHEEL_RADIUS_M = 0.1125

# Same values as omni_base_driver/include/omni_base_driver/picking_robot_matrix.hpp.
WHEEL_DIST = [0.32330, 0.32330, 0.38500]
WHEEL_PHI = [-0.883978, 0.883978, math.pi]

R_X = [WHEEL_DIST[i] * math.cos(WHEEL_PHI[i]) for i in range(WHEEL_COUNT)]
R_Y = [WHEEL_DIST[i] * math.sin(WHEEL_PHI[i]) for i in range(WHEEL_COUNT)]

R = np.array([
    [1.0, 0.0, -R_Y[0]],
    [0.0, 1.0, R_X[0]],
    [1.0, 0.0, -R_Y[1]],
    [0.0, 1.0, R_X[1]],
    [1.0, 0.0, -R_Y[2]],
    [0.0, 1.0, R_X[2]],
])


def cmd_vel_to_wheels(vx: float, vy: float, vth: float) -> Tuple[List[float], List[float]]:
    """Convert robot cmd_vel to wheel linear velocities and steer angles.

    This mirrors PickingRobotMatrix::calcWheelVelAng() so the sim controller
    follows the same steering flip logic as the real robot.
    """
    v = np.array([[vx], [vy], [vth]])
    v_wheel = R @ v

    drive_vel = []
    steer_phi = []
    for i in range(WHEEL_COUNT):
        vx_i = float(v_wheel[i * 2, 0])
        vy_i = float(v_wheel[i * 2 + 1, 0])
        speed = math.sqrt(vx_i * vx_i + vy_i * vy_i)
        angle = math.atan2(vy_i, vx_i)

        if angle < -0.71 * math.pi:
            angle += math.pi
            speed *= -1.0
        elif angle > 0.71 * math.pi:
            angle -= math.pi
            speed *= -1.0

        drive_vel.append(speed)
        steer_phi.append(angle)

    return drive_vel, steer_phi


def linear_to_wheel_rad_s(linear_m_s: float, wheel_radius_m: float = WHEEL_RADIUS_M) -> float:
    return linear_m_s / wheel_radius_m
