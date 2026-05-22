#!/usr/bin/python3
"""Estimate an initial map->odom transform from a saved PLY map and live LiDAR.

The tool projects both clouds to XY, searches yaw + translation, and prints a
map->odom transform that can be used as an initial alignment hint.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener

from ply_to_nav2_map import read_ply_vertices


def read_xyz_points(msg: PointCloud2) -> np.ndarray:
    points = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    if isinstance(points, np.ndarray) and points.dtype.names:
        return np.column_stack(
            [
                points["x"].astype(np.float32),
                points["y"].astype(np.float32),
                points["z"].astype(np.float32),
            ]
        )
    return np.asarray(list(points), dtype=np.float32).reshape(-1, 3)


def quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float32,
    )


def apply_transform(points: np.ndarray, transform) -> np.ndarray:
    q = transform.rotation
    t = transform.translation
    rot = quat_to_rot(q.x, q.y, q.z, q.w)
    out = points @ rot.T
    out[:, 0] += t.x
    out[:, 1] += t.y
    out[:, 2] += t.z
    return out.astype(np.float32)


def yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def filter_points(points: np.ndarray, args: argparse.Namespace) -> np.ndarray:
    if points.size == 0:
        return points
    r = np.linalg.norm(points[:, :2], axis=1)
    mask = (
        (points[:, 2] >= args.z_min)
        & (points[:, 2] <= args.z_max)
        & (r >= args.range_min)
        & (r <= args.range_max)
    )
    return points[mask]


def downsample_xy(points: np.ndarray, max_points: int) -> np.ndarray:
    if len(points) <= max_points:
        return points
    stride = int(math.ceil(len(points) / max_points))
    return points[::stride][:max_points]


def rasterize_xy(points_xy: np.ndarray, resolution: float, padding: float) -> tuple[np.ndarray, np.ndarray]:
    min_xy = points_xy.min(axis=0) - padding
    max_xy = points_xy.max(axis=0) + padding
    shape_xy = np.maximum(1, np.ceil((max_xy - min_xy) / resolution).astype(np.int64) + 1)
    grid = np.zeros((int(shape_xy[1]), int(shape_xy[0])), dtype=np.float32)
    ij = np.floor((points_xy - min_xy) / resolution).astype(np.int64)
    ok = (
        (ij[:, 0] >= 0)
        & (ij[:, 0] < shape_xy[0])
        & (ij[:, 1] >= 0)
        & (ij[:, 1] < shape_xy[1])
    )
    grid[ij[ok, 1], ij[ok, 0]] = 1.0
    return grid, min_xy.astype(np.float32)


def best_grid_offset(map_grid: np.ndarray, scan_grid: np.ndarray) -> tuple[float, int, int]:
    pad_shape = (
        map_grid.shape[0] + scan_grid.shape[0] - 1,
        map_grid.shape[1] + scan_grid.shape[1] - 1,
    )
    corr = np.fft.irfft2(
        np.fft.rfft2(map_grid, pad_shape) * np.conj(np.fft.rfft2(scan_grid, pad_shape)),
        pad_shape,
    )
    iy, ix = np.unravel_index(int(np.argmax(corr)), corr.shape)
    if iy >= map_grid.shape[0]:
        iy -= pad_shape[0]
    if ix >= map_grid.shape[1]:
        ix -= pad_shape[1]
    return float(corr.max()), int(ix), int(iy)


def estimate_alignment(
    map_points: np.ndarray,
    scan_points: np.ndarray,
    args: argparse.Namespace,
) -> tuple[float, float, float, float]:
    map_xy = downsample_xy(map_points[:, :2], args.max_map_points)
    scan_xy = downsample_xy(scan_points[:, :2], args.max_scan_points)
    if len(map_xy) < 20 or len(scan_xy) < 20:
        raise RuntimeError("Not enough points after filtering for alignment")

    map_grid, map_origin = rasterize_xy(map_xy, args.resolution, args.padding)

    best: tuple[float, float, float, float] | None = None
    yaw_values = np.deg2rad(
        np.arange(args.yaw_min_deg, args.yaw_max_deg + 0.5 * args.yaw_step_deg, args.yaw_step_deg)
    )
    for yaw in yaw_values:
        c = math.cos(float(yaw))
        s = math.sin(float(yaw))
        rot = np.array([[c, -s], [s, c]], dtype=np.float32)
        rotated = scan_xy @ rot.T
        scan_grid, scan_origin = rasterize_xy(rotated, args.resolution, args.padding)
        score, off_x, off_y = best_grid_offset(map_grid, scan_grid)
        t_xy = map_origin + np.array([off_x, off_y], dtype=np.float32) * args.resolution - scan_origin
        normalized = score / max(1.0, float(np.count_nonzero(scan_grid)))
        candidate = (normalized, float(yaw), float(t_xy[0]), float(t_xy[1]))
        if best is None or candidate[0] > best[0]:
            best = candidate

    assert best is not None
    return best


class AutoInitialAlign(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("auto_initial_align")
        self.args = args
        self.map_points = filter_points(read_ply_vertices(Path(args.map)), args)
        self.scan_chunks: list[np.ndarray] = []
        self.warned_tf = False
        self.done = False
        self.received_clouds = 0
        self.empty_clouds = 0
        self.filtered_empty_clouds = 0
        self.tf_failed_clouds = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(PointCloud2, args.topic, self.cloud_callback, qos)
        self.create_timer(args.duration, self.finish)
        self.get_logger().info(
            f"Loaded {len(self.map_points)} map points from {args.map}; "
            f"collecting {args.duration:.1f}s from {args.topic}"
        )

    def cloud_callback(self, msg: PointCloud2) -> None:
        if self.done:
            return
        self.received_clouds += 1
        points = read_xyz_points(msg)
        if points.size == 0:
            self.empty_clouds += 1
            return
        points = filter_points(points, self.args)
        if points.size == 0:
            self.filtered_empty_clouds += 1
            return

        if self.args.target_frame:
            transformed = self.transform_points(msg, points)
            if transformed is None:
                self.tf_failed_clouds += 1
                return
            points = transformed

        self.scan_chunks.append(points)
        if len(self.scan_chunks) % 10 == 0:
            count = sum(len(chunk) for chunk in self.scan_chunks)
            self.get_logger().info(f"Collected {count} filtered scan points")

    def transform_points(self, msg: PointCloud2, points: np.ndarray) -> np.ndarray | None:
        if msg.header.frame_id == self.args.target_frame:
            return points
        try:
            transform = self.tf_buffer.lookup_transform(
                self.args.target_frame,
                msg.header.frame_id,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.args.target_frame,
                    msg.header.frame_id,
                    Time(),
                    timeout=Duration(seconds=0.05),
                )
            except TransformException as exc:
                if not self.warned_tf:
                    self.warned_tf = True
                    self.get_logger().warning(
                        f"Cannot transform {msg.header.frame_id} -> {self.args.target_frame}: {exc}"
                    )
                return None
        return apply_transform(points, transform.transform)

    def finish(self) -> None:
        if self.done:
            return
        self.done = True
        if not self.scan_chunks:
            self.get_logger().error(
                "No scan points collected; cannot estimate initial alignment. "
                f"received_clouds={self.received_clouds}, empty_clouds={self.empty_clouds}, "
                f"filtered_empty_clouds={self.filtered_empty_clouds}, "
                f"tf_failed_clouds={self.tf_failed_clouds}. "
                "Check /livox/lidar hz and the TF chain to --target-frame."
            )
            return

        scan_points = np.vstack(self.scan_chunks)
        scan_points = filter_points(scan_points, self.args)
        try:
            score, yaw, tx, ty = estimate_alignment(self.map_points, scan_points, self.args)
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            return

        self.report(score, yaw, tx, ty)
        if self.args.publish_tf:
            self.publish_static_tf(yaw, tx, ty)
        if self.args.publish_initialpose:
            self.publish_initialpose(yaw, tx, ty)

        if self.args.keep_alive and (self.args.publish_tf or self.args.publish_initialpose):
            self.get_logger().info("Keeping node alive so the published transform remains available")
            return
        return

    def report(self, score: float, yaw: float, tx: float, ty: float) -> None:
        self.get_logger().info("Estimated initial map->odom alignment")
        print()
        print("map->odom:")
        print(f"  x: {tx:.6f}")
        print(f"  y: {ty:.6f}")
        print("  z: 0.000000")
        print(f"  yaw_rad: {yaw:.6f}")
        print(f"  yaw_deg: {math.degrees(yaw):.3f}")
        print(f"  overlap_score: {score:.3f}")
        print()
        print("Check-only TF command:")
        print(
            "ros2 run tf2_ros static_transform_publisher "
            f"{tx:.6f} {ty:.6f} 0 {yaw:.6f} 0 0 map odom"
        )
        print()
        print("注意: 如果 GLIM 已经在 publish map->odom，不要同时运行这个 static TF，")
        print("否则 TF 会重复。先用它验证初始对齐，再决定如何注入到 localization。")

    def publish_static_tf(self, yaw: float, tx: float, ty: float) -> None:
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.map_frame
        msg.child_frame_id = self.args.odom_frame
        msg.transform.translation.x = tx
        msg.transform.translation.y = ty
        msg.transform.translation.z = 0.0
        qx, qy, qz, qw = yaw_to_quat(yaw)
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw
        self.static_tf.sendTransform(msg)
        self.get_logger().info(f"Published static TF {self.args.map_frame}->{self.args.odom_frame}")

    def publish_initialpose(self, yaw: float, tx: float, ty: float) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.args.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = tx
        msg.pose.pose.position.y = ty
        qx, qy, qz, qw = yaw_to_quat(yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = math.radians(15.0) ** 2
        self.initialpose_pub.publish(msg)
        self.get_logger().info("Published /initialpose for AMCL-style localization")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", required=True, help="Saved map PLY in the map frame")
    parser.add_argument("--topic", default="/livox/lidar", help="Live PointCloud2 topic")
    parser.add_argument("--target-frame", default="odom", help="Frame to accumulate live scans in")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--duration", type=float, default=5.0, help="Seconds of scans to collect")
    parser.add_argument("--resolution", type=float, default=0.20, help="2D search grid resolution")
    parser.add_argument("--padding", type=float, default=1.0)
    parser.add_argument("--z-min", type=float, default=-0.5)
    parser.add_argument("--z-max", type=float, default=1.5)
    parser.add_argument("--range-min", type=float, default=0.3)
    parser.add_argument("--range-max", type=float, default=25.0)
    parser.add_argument("--yaw-min-deg", type=float, default=-180.0)
    parser.add_argument("--yaw-max-deg", type=float, default=180.0)
    parser.add_argument("--yaw-step-deg", type=float, default=5.0)
    parser.add_argument("--max-map-points", type=int, default=20000)
    parser.add_argument("--max-scan-points", type=int, default=20000)
    parser.add_argument("--publish-tf", action="store_true", help="Publish estimated map->odom as static TF")
    parser.add_argument("--publish-initialpose", action="store_true", help="Publish /initialpose")
    parser.add_argument("--keep-alive", action="store_true", help="Keep node alive after publishing")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = AutoInitialAlign(args)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            keep_published_output = args.keep_alive and (args.publish_tf or args.publish_initialpose)
            if node.done and not keep_published_output:
                break
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
