#!/usr/bin/env python3
"""Visualize GLIM/PLY anchor poses A, B, and C in RViz.

A: first mapping pose stored in a GLIM dump trajectory file.
B: visual overlay target pose from saved calibration or RViz /initialpose.
C: current GLIM pose topic, usually /glim_ros/pose_corrected.

This tool publishes visualization_msgs/MarkerArray only. It does not modify
TF, GLIM, Nav2, or /l402_glim_points.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Optional, Union

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


def yaw_from_quaternion(z: float, w: float, x: float = 0.0, y: float = 0.0) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def read_first_traj_pose(map_dump: Path, traj_file: str) -> tuple[float, float, float]:
    path = map_dump / traj_file
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                raise ValueError(f"Expected at least 8 fields in {path}: {line}")
            x = float(parts[1])
            y = float(parts[2])
            qx = float(parts[4])
            qy = float(parts[5])
            qz = float(parts[6])
            qw = float(parts[7])
            return x, y, yaw_from_quaternion(qz, qw, qx, qy)
    raise ValueError(f"No trajectory pose found in {path}")


def read_calib_pose(path: Path) -> Optional[tuple[float, float, float]]:
    if not path.exists():
        return None

    values: dict[str, Union[float, str]] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, raw_value = line.split(":", 1)
        try:
            values[key.strip()] = float(raw_value.strip())
        except ValueError:
            values[key.strip()] = raw_value.strip()

    required = ("target_x", "target_y", "target_yaw")
    if not all(isinstance(values.get(key), float) for key in required):
        return None
    return (
        float(values["target_x"]),
        float(values["target_y"]),
        float(values["target_yaw"]),
    )


class GlimAnchorPoseVisualizer(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("glim_anchor_pose_visualizer")
        self.args = args
        self.pose_a = read_first_traj_pose(Path(args.map_dump), args.traj_file)
        self.pose_b = read_calib_pose(Path(args.calib)) if args.calib else None
        self.pose_c: Optional[tuple[float, float, float]] = None

        self.pub = self.create_publisher(MarkerArray, args.marker_topic, 1)
        self.create_subscription(PoseStamped, args.glim_pose_topic, self.on_glim_pose, 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            args.initialpose_topic,
            self.on_initialpose,
            10,
        )
        self.create_timer(args.period, self.publish_markers)

        ax, ay, ayaw = self.pose_a
        self.get_logger().info(
            f"A from {args.map_dump}/{args.traj_file}: "
            f"x={ax:.6f}, y={ay:.6f}, yaw={ayaw:.6f} rad ({math.degrees(ayaw):.3f} deg)"
        )
        if self.pose_b is None:
            self.get_logger().info(
                "B is not available yet. Click RViz 2D Pose Estimate or provide --calib."
            )
        else:
            bx, by, byaw = self.pose_b
            self.get_logger().info(
                f"B from calib: x={bx:.6f}, y={by:.6f}, "
                f"yaw={byaw:.6f} rad ({math.degrees(byaw):.3f} deg)"
            )

    def on_glim_pose(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        self.pose_c = (
            msg.pose.position.x,
            msg.pose.position.y,
            yaw_from_quaternion(q.z, q.w, q.x, q.y),
        )

    def on_initialpose(self, msg: PoseWithCovarianceStamped) -> None:
        q = msg.pose.pose.orientation
        self.pose_b = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw_from_quaternion(q.z, q.w, q.x, q.y),
        )
        bx, by, byaw = self.pose_b
        self.get_logger().info(
            f"B updated from /initialpose: x={bx:.6f}, y={by:.6f}, "
            f"yaw={byaw:.6f} rad ({math.degrees(byaw):.3f} deg)"
        )

    def publish_markers(self) -> None:
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        entries = [
            ("A dump first pose", self.pose_a, (1.0, 0.1, 0.1, 1.0), 0),
            ("B visual target", self.pose_b, (0.1, 1.0, 0.1, 1.0), 10),
            ("C current GLIM pose", self.pose_c, (0.1, 0.4, 1.0, 1.0), 20),
        ]
        for label, pose, color, marker_id in entries:
            if pose is None:
                continue
            markers.markers.append(self.make_arrow(marker_id, stamp, label, pose, color))
            markers.markers.append(self.make_text(marker_id + 1, stamp, label, pose, color))

        # Keep old markers from hanging around when B/C are unavailable.
        for marker_id in (0, 1, 10, 11, 20, 21):
            if not any(m.id == marker_id for m in markers.markers):
                marker = Marker()
                marker.header.frame_id = self.args.frame_id
                marker.header.stamp = stamp
                marker.ns = self.args.namespace
                marker.id = marker_id
                marker.action = Marker.DELETE
                markers.markers.append(marker)

        self.pub.publish(markers)

    def make_arrow(
        self,
        marker_id: int,
        stamp,
        label: str,
        pose: tuple[float, float, float],
        color: tuple[float, float, float, float],
    ) -> Marker:
        x, y, yaw = pose
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        marker = Marker()
        marker.header.frame_id = self.args.frame_id
        marker.header.stamp = stamp
        marker.ns = self.args.namespace
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = self.args.z
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        marker.scale.x = self.args.arrow_length
        marker.scale.y = self.args.arrow_width
        marker.scale.z = self.args.arrow_width
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def make_text(
        self,
        marker_id: int,
        stamp,
        label: str,
        pose: tuple[float, float, float],
        color: tuple[float, float, float, float],
    ) -> Marker:
        x, y, yaw = pose
        marker = Marker()
        marker.header.frame_id = self.args.frame_id
        marker.header.stamp = stamp
        marker.ns = self.args.namespace
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = self.args.z + self.args.text_z_offset
        marker.scale.z = self.args.text_size
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.text = (
            f"{label}\n"
            f"x={x:.3f} y={y:.3f}\n"
            f"yaw={math.degrees(yaw):.1f} deg"
        )
        return marker


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-dump", required=True, help="GLIM map dump directory")
    parser.add_argument("--traj-file", default="traj_lidar.txt")
    parser.add_argument("--calib", default=None, help="Saved visual calibration YAML-like file")
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--marker-topic", default="/glim_anchor_poses")
    parser.add_argument("--namespace", default="glim_anchor_poses")
    parser.add_argument("--glim-pose-topic", default="/glim_ros/pose_corrected")
    parser.add_argument("--initialpose-topic", default="/initialpose")
    parser.add_argument("--period", type=float, default=0.5)
    parser.add_argument("--z", type=float, default=0.15)
    parser.add_argument("--arrow-length", type=float, default=0.7)
    parser.add_argument("--arrow-width", type=float, default=0.08)
    parser.add_argument("--text-size", type=float, default=0.22)
    parser.add_argument("--text-z-offset", type=float, default=0.35)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = GlimAnchorPoseVisualizer(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
