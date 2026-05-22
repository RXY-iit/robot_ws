#!/usr/bin/env python3
"""Accumulate a PointCloud2 topic and save a simple Nav2 occupancy map."""

from __future__ import annotations

import argparse
import math
import signal
import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


class PointCloudToMap(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("pointcloud_to_nav2_map")
        self.args = args
        self.points_xy: list[np.ndarray] = []
        self.count_messages = 0
        self.count_points = 0
        self.frame_id = ""
        self.warned_tf = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(PointCloud2, args.topic, self.cloud_callback, qos)

        if args.duration > 0:
            self.create_timer(args.duration, self.finish)

        self.get_logger().info(
            f"Listening to {args.topic}; z filter [{args.z_min}, {args.z_max}] m; "
            f"resolution {args.resolution} m/cell"
        )

    def cloud_callback(self, msg: PointCloud2) -> None:
        pts = read_xyz_points(msg)
        if pts.size == 0:
            return

        if self.count_messages == 0:
            self.get_logger().info(f"First cloud frame_id={msg.header.frame_id}")

        pts = filter_range(pts, self.args.range_min, self.args.range_max)
        if pts.size == 0:
            return

        if self.args.target_frame:
            pts = self.transform_points_to_target(msg, pts)
            if pts is None or pts.size == 0:
                return

        z = pts[:, 2]
        mask = (z >= self.args.z_min) & (z <= self.args.z_max)
        pts = pts[mask]
        if pts.size == 0:
            return

        self.points_xy.append(pts[:, :2].copy())
        self.count_messages += 1
        self.count_points += len(pts)
        self.frame_id = msg.header.frame_id

        if self.count_messages % 20 == 0:
            self.get_logger().info(
                f"Accumulated {self.count_points} filtered points from "
                f"{self.count_messages} clouds; frame_id={self.frame_id}"
            )

    def transform_points_to_target(self, msg: PointCloud2, pts: np.ndarray) -> np.ndarray | None:
        source_frame = msg.header.frame_id
        if source_frame == self.args.target_frame:
            return pts

        try:
            transform = self.tf_buffer.lookup_transform(
                self.args.target_frame,
                source_frame,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            if not self.warned_tf:
                self.warned_tf = True
                self.get_logger().warning(
                    f"Cannot transform {source_frame} -> {self.args.target_frame}: {exc}"
                )
            return None

        return apply_transform(pts, transform.transform)

    def finish(self) -> None:
        save_map(self.args, self.points_xy, self.frame_id, self.get_logger())
        rclpy.shutdown()


def inflate_obstacles(occupied: np.ndarray, radius_cells: int) -> np.ndarray:
    if radius_cells <= 0:
        return occupied

    inflated = occupied.copy()
    ys, xs = np.nonzero(occupied)
    offsets = [
        (dy, dx)
        for dy in range(-radius_cells, radius_cells + 1)
        for dx in range(-radius_cells, radius_cells + 1)
        if dx * dx + dy * dy <= radius_cells * radius_cells
    ]
    height, width = occupied.shape
    for dy, dx in offsets:
        ny = ys + dy
        nx = xs + dx
        ok = (0 <= nx) & (nx < width) & (0 <= ny) & (ny < height)
        inflated[ny[ok], nx[ok]] = True
    return inflated


def read_xyz_points(msg: PointCloud2) -> np.ndarray:
    points = point_cloud2.read_points(
        msg,
        field_names=("x", "y", "z"),
        skip_nans=True,
    )

    if isinstance(points, np.ndarray) and points.dtype.names:
        return np.column_stack(
            [
                points["x"].astype(np.float32),
                points["y"].astype(np.float32),
                points["z"].astype(np.float32),
            ]
        )

    return np.asarray(list(points), dtype=np.float32).reshape(-1, 3)


def filter_range(points: np.ndarray, range_min: float, range_max: float) -> np.ndarray:
    xy_range = np.linalg.norm(points[:, :2], axis=1)
    return points[(xy_range >= range_min) & (xy_range <= range_max)]


def apply_transform(points: np.ndarray, transform) -> np.ndarray:
    q = transform.rotation
    t = transform.translation
    rot = quat_to_rot(q.x, q.y, q.z, q.w)
    out = points @ rot.T
    out[:, 0] += t.x
    out[:, 1] += t.y
    out[:, 2] += t.z
    return out.astype(np.float32)


def quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float32,
    )


def save_map(
    args: argparse.Namespace,
    chunks: list[np.ndarray],
    frame_id: str,
    logger,
) -> None:
    if not chunks:
        logger.error("No points received; map was not written.")
        return

    points = np.vstack(chunks)
    min_x = float(np.min(points[:, 0]) - args.padding)
    max_x = float(np.max(points[:, 0]) + args.padding)
    min_y = float(np.min(points[:, 1]) - args.padding)
    max_y = float(np.max(points[:, 1]) + args.padding)

    width = max(1, int(math.ceil((max_x - min_x) / args.resolution)))
    height = max(1, int(math.ceil((max_y - min_y) / args.resolution)))

    ix = np.floor((points[:, 0] - min_x) / args.resolution).astype(np.int64)
    iy = np.floor((points[:, 1] - min_y) / args.resolution).astype(np.int64)
    ok = (0 <= ix) & (ix < width) & (0 <= iy) & (iy < height)

    hits = np.zeros((height, width), dtype=np.uint16)
    np.add.at(hits, (iy[ok], ix[ok]), 1)
    occupied = hits >= args.min_hits
    occupied = inflate_obstacles(occupied, int(round(args.inflate_radius / args.resolution)))

    # Nav2 map_server convention with negate=0: 0=occupied, 254=free.
    # This first-pass map marks all non-obstacle cells free.
    image = np.full((height, width), 254, dtype=np.uint8)
    image[occupied] = 0
    image = np.flipud(image)

    pgm_path = Path(args.output).with_suffix(".pgm")
    yaml_path = Path(args.output).with_suffix(".yaml")
    pgm_path.parent.mkdir(parents=True, exist_ok=True)

    with pgm_path.open("wb") as f:
        f.write(f"P5\n# generated from {args.topic}, frame_id={frame_id}\n{width} {height}\n255\n".encode())
        f.write(image.tobytes())

    yaml_text = (
        f"image: {pgm_path.name}\n"
        f"mode: trinary\n"
        f"resolution: {args.resolution:.6f}\n"
        f"origin: [{min_x:.6f}, {min_y:.6f}, 0.0]\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.25\n"
    )
    yaml_path.write_text(yaml_text)

    logger.info(f"Wrote {pgm_path}")
    logger.info(f"Wrote {yaml_path}")
    logger.info(
        f"Map size: {width} x {height}; origin=({min_x:.3f}, {min_y:.3f}); "
        f"occupied cells={int(np.count_nonzero(occupied))}"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/glim_ros/aligned_points")
    parser.add_argument("--target-frame", default="", help="Optional fixed frame to transform clouds into before projection")
    parser.add_argument("--output", default="maps/l402_2d_map")
    parser.add_argument("--duration", type=float, default=60.0, help="Seconds to accumulate; 0 means until Ctrl+C")
    parser.add_argument("--resolution", type=float, default=0.05, help="Meters per pixel")
    parser.add_argument("--z-min", type=float, default=-0.3, help="Minimum z kept as obstacle")
    parser.add_argument("--z-max", type=float, default=1.5, help="Maximum z kept as obstacle")
    parser.add_argument("--range-min", type=float, default=0.0, help="Minimum XY range kept before projection")
    parser.add_argument("--range-max", type=float, default=float("inf"), help="Maximum XY range kept before projection")
    parser.add_argument("--padding", type=float, default=1.0, help="Free-space padding around point bounds")
    parser.add_argument("--min-hits", type=int, default=2, help="Minimum points in a cell to mark occupied")
    parser.add_argument("--inflate-radius", type=float, default=0.12, help="Obstacle inflation baked into the map")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = PointCloudToMap(args)

    def handle_signal(signum, frame) -> None:
        del signum, frame
        node.finish()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.finish()


if __name__ == "__main__":
    main()
