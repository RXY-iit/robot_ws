#!/usr/bin/env python3
"""Publish a PLY point cloud and align it for RViz visualization.

This is a visualization helper. It does not change GLIM localization or TF.
It transforms the published PLY points so an anchor pose in the PLY map moves
to either:
  * the pose clicked with RViz "2D Pose Estimate" (/initialpose), or
  * a TF pose, usually map -> odom for GLIM visual overlay alignment.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Optional, Union

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.parameter import Parameter
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

from ply_to_nav2_map import read_ply_vertices


def yaw_from_quaternion(z: float, w: float, x: float = 0.0, y: float = 0.0) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class InteractivePlyInitialposePublisher(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("interactive_ply_initialpose_publisher")
        self.args = args
        self.original_points = read_ply_vertices(Path(args.input)).astype(np.float64)

        if args.z_min is not None or args.z_max is not None:
            z_min = -np.inf if args.z_min is None else args.z_min
            z_max = np.inf if args.z_max is None else args.z_max
            self.original_points = self.original_points[
                (self.original_points[:, 2] >= z_min)
                & (self.original_points[:, 2] <= z_max)
            ]

        self.current_points = self.original_points.copy()
        self.transform_applied = False
        self.auto_tf_applied = False
        self.last_calib: Optional[dict[str, Union[float, str]]] = None
        self.last_auto_target: Optional[tuple[float, float, float]] = None
        self.start_time = self.get_clock().now()

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(PointCloud2, args.topic, qos)
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            args.initialpose_topic,
            self.on_initialpose,
            10,
        )
        self.timer = self.create_timer(args.period, self.publish_cloud)
        self.tf_buffer: Optional[Buffer] = None
        self.tf_listener: Optional[TransformListener] = None
        self.tf_timer = None
        if args.auto_from_tf:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.tf_timer = self.create_timer(args.tf_check_period, self.try_auto_align_from_tf)

        self.msg = self.make_cloud(self.current_points)
        if args.load_calib:
            self.load_calib(Path(args.load_calib))
        self.publish_cloud()
        self.get_logger().info(
            f"Publishing {len(self.current_points)} PLY points on {args.topic} "
            f"in frame {args.frame_id}"
        )
        self.get_logger().info(
            "Click RViz '2D Pose Estimate' to move the PLY anchor pose "
            f"({args.anchor_x:.3f}, {args.anchor_y:.3f}, {args.anchor_yaw:.3f} rad) "
            f"to the clicked pose from {args.initialpose_topic}"
        )
        if args.auto_from_tf:
            self.get_logger().info(
                "Auto visual alignment enabled: after "
                f"{args.auto_delay:.1f}s, move the PLY anchor pose to TF "
                f"{args.tf_parent_frame} -> {args.tf_child_frame}"
            )

    def make_cloud(self, points: np.ndarray) -> PointCloud2:
        header = Header()
        header.frame_id = self.args.frame_id
        header.stamp = self.get_clock().now().to_msg()
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        return point_cloud2.create_cloud(header, fields, points.astype(np.float32))

    def publish_cloud(self) -> None:
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)

    def on_initialpose(self, msg: PoseWithCovarianceStamped) -> None:
        if self.args.once and self.transform_applied:
            self.get_logger().info("Ignoring /initialpose because --once already applied")
            return

        pose = msg.pose.pose
        target_x = pose.position.x
        target_y = pose.position.y
        target_yaw = yaw_from_quaternion(
            pose.orientation.z,
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
        )

        self.apply_alignment(target_x, target_y, target_yaw, source="initialpose")

    def try_auto_align_from_tf(self) -> None:
        if self.args.auto_once and self.auto_tf_applied:
            return
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        if elapsed < self.args.auto_delay:
            return
        if self.tf_buffer is None:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.args.tf_parent_frame,
                self.args.tf_child_frame,
                Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"Waiting for TF {self.args.tf_parent_frame} -> "
                f"{self.args.tf_child_frame}: {exc}"
            )
            return

        t = transform.transform.translation
        q = transform.transform.rotation
        target_yaw = yaw_from_quaternion(q.z, q.w, q.x, q.y)

        if self.last_auto_target is not None:
            last_x, last_y, last_yaw = self.last_auto_target
            dist = math.hypot(t.x - last_x, t.y - last_y)
            yaw_diff = abs(normalize_angle(target_yaw - last_yaw))
            if dist < self.args.auto_pos_epsilon and yaw_diff < self.args.auto_yaw_epsilon:
                return

        self.apply_alignment(t.x, t.y, target_yaw, source="tf")
        self.last_auto_target = (t.x, t.y, target_yaw)
        self.auto_tf_applied = True
        if self.args.auto_once and self.tf_timer is not None:
            self.tf_timer.cancel()

    def apply_alignment(
        self,
        target_x: float,
        target_y: float,
        target_yaw: float,
        source: str,
        save: bool = True,
    ) -> None:
        anchor_x = self.args.anchor_x
        anchor_y = self.args.anchor_y
        anchor_yaw = self.args.anchor_yaw
        delta_yaw = normalize_angle(target_yaw - anchor_yaw)

        c = math.cos(delta_yaw)
        s = math.sin(delta_yaw)
        shifted = self.original_points.copy()
        local_x = shifted[:, 0] - anchor_x
        local_y = shifted[:, 1] - anchor_y
        shifted[:, 0] = c * local_x - s * local_y + target_x
        shifted[:, 1] = s * local_x + c * local_y + target_y

        self.current_points = shifted
        self.msg = self.make_cloud(self.current_points)
        self.transform_applied = True
        self.last_calib = {
            "source": source,
            "anchor_x": anchor_x,
            "anchor_y": anchor_y,
            "anchor_yaw": anchor_yaw,
            "target_x": target_x,
            "target_y": target_y,
            "target_yaw": target_yaw,
            "delta_x": target_x - anchor_x,
            "delta_y": target_y - anchor_y,
            "delta_yaw": delta_yaw,
        }
        self.publish_cloud()

        self.get_logger().info(
            f"Applied PLY visual alignment from {source}: "
            f"target=({target_x:.3f}, {target_y:.3f}, {target_yaw:.3f} rad), "
            f"delta_yaw={delta_yaw:.3f} rad"
        )
        if save and self.args.save_calib:
            self.save_calib(Path(self.args.save_calib))

    def save_calib(self, path: Path) -> None:
        if self.last_calib is None:
            return
        path.parent.mkdir(parents=True, exist_ok=True)
        lines = [
            "# Calibration generated by interactive_ply_initialpose_publisher.py",
            f"input: {self.args.input}",
            f"topic: {self.args.topic}",
            f"frame_id: {self.args.frame_id}",
        ]
        for key, value in self.last_calib.items():
            if isinstance(value, str):
                lines.append(f"{key}: {value}")
            else:
                lines.append(f"{key}: {value:.9f}")
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        self.get_logger().info(f"Saved calibration to {path}")

    def load_calib(self, path: Path) -> None:
        if not path.exists():
            self.get_logger().info(f"Calibration file not found, starting unaligned: {path}")
            return

        values: dict[str, Union[float, str]] = {}
        for line in path.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if not line or line.startswith("#") or ":" not in line:
                continue
            key, raw_value = line.split(":", 1)
            key = key.strip()
            raw_value = raw_value.strip()
            try:
                values[key] = float(raw_value)
            except ValueError:
                values[key] = raw_value

        required = ("target_x", "target_y", "target_yaw")
        if not all(key in values and isinstance(values[key], float) for key in required):
            self.get_logger().warn(f"Calibration file is missing target pose fields: {path}")
            return

        self.apply_alignment(
            float(values["target_x"]),
            float(values["target_y"]),
            float(values["target_yaw"]),
            source="loaded_calib",
            save=False,
        )
        self.get_logger().info(f"Loaded calibration from {path}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", help="Input binary little-endian PLY file")
    parser.add_argument("--topic", default="/l402_glim_points")
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--initialpose-topic", default="/initialpose")
    parser.add_argument(
        "--use-sim-time",
        action="store_true",
        help="Use /clock time. Enable this for rosbag replay with --clock.",
    )
    parser.add_argument("--period", type=float, default=1.0)
    parser.add_argument("--z-min", type=float, default=None)
    parser.add_argument("--z-max", type=float, default=None)
    parser.add_argument("--anchor-x", type=float, default=0.0)
    parser.add_argument("--anchor-y", type=float, default=0.0)
    parser.add_argument(
        "--anchor-yaw",
        type=float,
        default=0.0,
        help="Anchor yaw in radians. This is overridden by --anchor-yaw-deg.",
    )
    parser.add_argument("--anchor-yaw-deg", type=float, default=None)
    parser.add_argument("--once", action="store_true", help="Apply only the first /initialpose")
    parser.add_argument("--load-calib", default=None, help="Load a previously saved visual calibration")
    parser.add_argument("--save-calib", default=None, help="Write applied transform as a YAML-like file")
    parser.add_argument(
        "--auto-from-tf",
        action="store_true",
        help="Use TF to visually align the PLY anchor pose automatically.",
    )
    parser.add_argument(
        "--tf-parent-frame",
        default=None,
        help="Parent frame for auto TF alignment. Defaults to --frame-id.",
    )
    parser.add_argument(
        "--tf-child-frame",
        default="odom",
        help="Child frame for auto TF alignment.",
    )
    parser.add_argument(
        "--auto-delay",
        type=float,
        default=8.0,
        help="Seconds to wait before the first TF auto-alignment attempt.",
    )
    parser.add_argument(
        "--tf-check-period",
        type=float,
        default=1.0,
        help="Seconds between TF lookup attempts.",
    )
    parser.add_argument(
        "--auto-repeat",
        action="store_true",
        help="Keep reapplying TF alignment. Default is one automatic alignment only.",
    )
    parser.add_argument(
        "--auto-pos-epsilon",
        type=float,
        default=0.01,
        help="Minimum TF translation change in meters before auto-repeat republishes.",
    )
    parser.add_argument(
        "--auto-yaw-epsilon",
        type=float,
        default=math.radians(0.5),
        help="Minimum TF yaw change in radians before auto-repeat republishes.",
    )
    parser.add_argument(
        "--auto-yaw-epsilon-deg",
        type=float,
        default=None,
        help="Minimum TF yaw change in degrees. Overrides --auto-yaw-epsilon.",
    )
    args = parser.parse_args()
    if args.anchor_yaw_deg is not None:
        args.anchor_yaw = math.radians(args.anchor_yaw_deg)
    if args.auto_yaw_epsilon_deg is not None:
        args.auto_yaw_epsilon = math.radians(args.auto_yaw_epsilon_deg)
    if args.tf_parent_frame is None:
        args.tf_parent_frame = args.frame_id
    args.auto_once = not args.auto_repeat
    return args


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = InteractivePlyInitialposePublisher(args)
    if args.use_sim_time:
        node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
