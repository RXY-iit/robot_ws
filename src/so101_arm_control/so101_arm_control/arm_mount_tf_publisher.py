"""Publish the base_link to SO101 transforms from the lift estimate.

Both the cyan actual model and orange Dashboard-preview model must share the
same moving mount.  The lift currently has no encoder feedback, so this node
maps the software position estimate onto the operator-measured physical mount
range and deliberately clamps it to that range.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster


def clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def map_lift_to_mount_z(
    position_mm: float,
    position_min_mm: float,
    position_max_mm: float,
    mount_z_min_m: float,
    mount_z_max_m: float,
) -> float:
    """Linearly map a lift estimate into the measured SO101 base height."""
    if position_max_mm <= position_min_mm:
        raise ValueError("position_max_mm must be greater than position_min_mm")
    bounded = clamp(position_mm, position_min_mm, position_max_mm)
    ratio = (bounded - position_min_mm) / (position_max_mm - position_min_mm)
    return mount_z_min_m + ratio * (mount_z_max_m - mount_z_min_m)


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> tuple[float, ...]:
    """Return x, y, z, w for fixed-axis roll/pitch/yaw."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class ArmMountTfPublisher(Node):
    def __init__(self) -> None:
        super().__init__("so101_arm_mount_tf_publisher")
        self.parent_frame = str(
            self.declare_parameter("parent_frame", "base_link").value
        )
        self.child_frames = list(
            self.declare_parameter(
                "child_frames", ["arm/world", "arm_preview/world"]
            ).value
        )
        self.x_m = float(self.declare_parameter("x_m", 0.32084).value)
        self.y_m = float(self.declare_parameter("y_m", 0.01262).value)
        # Direct ground-height measurement on 2026-09-04.  This independent
        # Lift/Arm branch must never inherit D435 Pan/Tilt motion.
        # base_link is 0.1125 m above the floor, so the published coordinate
        # is 1.005 - 0.1125 = 0.8925 m.
        self.mount_z_min_m = float(self.declare_parameter("mount_z_min_m", 0.8925).value)
        self.mount_z_max_m = float(self.declare_parameter("mount_z_max_m", 1.1225).value)
        self.position_min_mm = float(self.declare_parameter("position_min_mm", 0.0).value)
        self.position_max_mm = float(self.declare_parameter("position_max_mm", 200.0).value)
        self.position_mm = float(self.declare_parameter("initial_position_mm", 0.0).value)
        self.roll = float(self.declare_parameter("roll", 0.0).value)
        # The arm was physically remounted upright on 2026-09-02: its base
        # plate is parallel to the ground.  XYZ/yaw remain survey/calibration
        # estimates and are deliberately independent parameters.
        self.pitch = float(self.declare_parameter("pitch", 0.0).value)
        self.yaw = float(self.declare_parameter("yaw", -0.01532).value)
        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 20.0).value)

        # Fail at startup instead of publishing a plausible-looking bad TF.
        map_lift_to_mount_z(
            self.position_mm,
            self.position_min_mm,
            self.position_max_mm,
            self.mount_z_min_m,
            self.mount_z_max_m,
        )
        if not self.child_frames:
            raise ValueError("child_frames must not be empty")
        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")

        self._broadcaster = TransformBroadcaster(self)
        self.create_subscription(Float32, "/lift/position", self._on_lift_position, 10)
        self.create_timer(1.0 / self.publish_rate_hz, self._publish)
        self.get_logger().info(
            "SO101 moving mount ready: lift %.1f..%.1f mm -> Z %.4f..%.4f m; "
            "initial %.1f mm is an operator-confirmed software estimate"
            % (
                self.position_min_mm,
                self.position_max_mm,
                self.mount_z_min_m,
                self.mount_z_max_m,
                self.position_mm,
            )
        )

    def _on_lift_position(self, message: Float32) -> None:
        if math.isfinite(message.data):
            self.position_mm = float(message.data)
        else:
            self.get_logger().warn("Ignoring non-finite /lift/position sample")

    def _publish(self) -> None:
        z_m = map_lift_to_mount_z(
            self.position_mm,
            self.position_min_mm,
            self.position_max_mm,
            self.mount_z_min_m,
            self.mount_z_max_m,
        )
        qx, qy, qz, qw = quaternion_from_euler(self.roll, self.pitch, self.yaw)
        stamp = self.get_clock().now().to_msg()
        transforms = []
        for child_frame in self.child_frames:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.parent_frame
            transform.child_frame_id = str(child_frame)
            transform.transform.translation.x = self.x_m
            transform.transform.translation.y = self.y_m
            transform.transform.translation.z = z_m
            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            transforms.append(transform)
        self._broadcaster.sendTransform(transforms)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmMountTfPublisher()
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
