#!/usr/bin/env python3

import os
import datetime
import csv
from typing import List, Optional, Tuple
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import Float32, Float32MultiArray
from aruco_interfaces.msg import ArucoMarkers

try:
    from pynput import keyboard
except ImportError:
    raise ImportError("pynput not found. Install with: pip install pynput")


class ButtonCSVLogger(Node):
    """Log the latest angles/pose to CSV when 'k' key is pressed."""

    def __init__(self) -> None:
        super().__init__('button_snapshot_logger')

        # File setup
        dir_path = os.path.expanduser('~/pickup_ws/angle_arucopose_csv/')
        os.makedirs(dir_path, exist_ok=True)
        ts = datetime.datetime.now().strftime('%m%d_%H%M%S')
        fname = f'aruco_motor_button_log_{ts}.csv'
        self.filepath = os.path.join(dir_path, fname)
        self._init_csv()

        # Latest data caches
        self.latest_angles: Optional[List[float]] = None
        self.latest_chokudo: Optional[float] = None
        self.latest_markers: List[Tuple[int, ArucoMarkers]] = []
        self.data_lock = Lock()

        # Subscriptions
        self.create_subscription(Float32MultiArray, '/motor_current_angles', self._angle_cb, 10)
        self.create_subscription(Float32, '/chokudomotor/angle', self._chokudo_cb, 10)
        self.create_subscription(ArucoMarkers, '/aruco/markers', self._markers_cb, 10)

        # Start keyboard listener
        self.listener = keyboard.Listener(on_press=self._on_key_press)
        self.listener.start()
        self.get_logger().info("Keyboard listener started. Press 'z' to log.")

    def _init_csv(self) -> None:
        header = ['timestamp'] + [f'motor{i + 1}' for i in range(10)] + [
            'marker_id', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'
        ]
        with open(self.filepath, mode='w', newline='') as f:
            csv.writer(f).writerow(header)

    def _angle_cb(self, msg: Float32MultiArray) -> None:
        if len(msg.data) == 9:
            self.latest_angles = list(msg.data)
        else:
            self.get_logger().warn('Invalid motor angle length; expect 9 values.')

    def _chokudo_cb(self, msg: Float32) -> None:
        self.latest_chokudo = float(msg.data)

    def _markers_cb(self, msg: ArucoMarkers) -> None:
        # Keep only the latest message; filter to marker IDs 0,1,2 for consistency
        with self.data_lock:
            filtered = []
            for mid, pose in zip(msg.marker_ids, msg.poses):
                if mid in (0, 1, 2):
                    filtered.append((mid, pose))
            self.latest_markers = filtered

    def _on_key_press(self, key) -> None:
        """Called when a key is pressed."""
        try:
            # Check if 'z' key was pressed
            if key == keyboard.KeyCode(char='z'):
                self._log_snapshot()
        except AttributeError:
            # Special keys don't have a char attribute
            pass

    def _log_snapshot(self) -> None:
        """Log current state to CSV."""
        with self.data_lock:
            if self.latest_angles is None or self.latest_chokudo is None:
                self.get_logger().warn('Button pressed but angles not ready; skipping log.')
                return

            if not self.latest_markers:
                self.get_logger().warn('Button pressed but no markers; skipping log.')
                return

            timestamp = self.get_clock().now().to_msg()
            unix_time = Time.from_msg(timestamp).nanoseconds * 1e-9

            for marker_id, pose in self.latest_markers:
                row = [unix_time] + self.latest_angles + [self.latest_chokudo] + [
                    marker_id,
                    pose.position.x, pose.position.y, pose.position.z,
                    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
                ]
                with open(self.filepath, mode='a', newline='') as f:
                    csv.writer(f).writerow(row)

            self.get_logger().info(f'Logged {len(self.latest_markers)} marker(s) to {self.filepath}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ButtonCSVLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
