"""Apply vision-derived joint scale corrections to the Arm visualization only.

The guarded control server continues to use its safety calibration unchanged.
This node affects only the robot_state_publisher input, so TF geometry can be
improved without changing commanded positions, limits, or protection logic.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


REFERENCE = {
    "shoulder_pan_joint": 0.03217642458100545,
    "shoulder_lift_joint": 1.74533,
    "elbow_flex_joint": -0.295563703024748,
    "wrist_flex_joint": -0.0373075230972284,
    "wrist_roll_joint": -1.2867643467643466,
}

SCALE = {
    "shoulder_pan_joint": 1.0642740565970414,
    "shoulder_lift_joint": 1.0225823378138021,
    "elbow_flex_joint": 1.0156842687470935,
    "wrist_flex_joint": 1.083955420493,
    "wrist_roll_joint": 1.3663034512798975,
}

# 2026-09-04 physical zero check: real wrist roll is visually neutral at
# -0.031 rad, while the previous RViz model was neutral at -0.277 rad.
# Apply the corresponding visual-only offset; hardware commands and safety
# limits remain in the original measured coordinate system.
VISUAL_OFFSET = {
    # Constrained 47-sample / four-fixed-board ChArUco solve.  These offsets
    # affect visualization/FK only; motor commands and safety calibration stay
    # in their verified raw coordinate system.
    "shoulder_lift_joint": 0.09842437585106671,
    "elbow_flex_joint": -0.052102814132610975,
    "wrist_flex_joint": 0.023105072091396325,
    "wrist_roll_joint": -0.3361107058855241,
}


def corrected_position(name: str, value: float) -> float:
    if name not in SCALE:
        return value
    return (REFERENCE[name] + SCALE[name] * (value - REFERENCE[name]) +
            VISUAL_OFFSET.get(name, 0.0))


class ArmVisualJointStateCalibrator(Node):
    def __init__(self) -> None:
        super().__init__("arm_visual_joint_state_calibrator")
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(
            JointState, "/arm/visual_joint_states", output_qos
        )
        self.create_subscription(
            JointState, "/joint_states", self._on_state, input_qos
        )
        self.get_logger().info(
            "Vision-derived Arm joint scale correction enabled for visualization TF only"
        )

    def _on_state(self, source: JointState) -> None:
        if not any(name in SCALE for name in source.name):
            return
        if len(source.name) != len(source.position):
            self.get_logger().warn("Ignoring malformed JointState name/position lengths")
            return
        values = [corrected_position(name, value) for name, value in zip(source.name, source.position)]
        if not all(math.isfinite(value) for value in values):
            self.get_logger().warn("Ignoring non-finite Arm JointState")
            return
        target = JointState()
        target.header = source.header
        target.name = list(source.name)
        target.position = values
        target.velocity = list(source.velocity)
        target.effort = list(source.effort)
        self._publisher.publish(target)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmVisualJointStateCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
