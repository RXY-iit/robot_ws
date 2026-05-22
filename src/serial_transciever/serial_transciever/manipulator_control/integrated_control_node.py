#!/usr/bin/env python3

import os
import datetime
import csv
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32, Float32MultiArray, Int8MultiArray
from aruco_interfaces.msg import ArucoMarkers
import threading
import sys
import re
from typing import List, Optional, Tuple
from threading import Lock
from pathlib import Path

try:
    from pynput import keyboard
except ImportError:
    raise ImportError("pynput not found. Install with: pip install pynput")


def _find_headers_dir() -> Path:
    current = Path(__file__).resolve()
    while True:
        headers_dir = current / 'src' / 'hose_control' / 'include' / 'hose_control'
        if headers_dir.exists():
            return headers_dir
        if current.parent == current:
            break
        current = current.parent
    cwd_candidate = Path.cwd() / 'src' / 'hose_control' / 'include' / 'hose_control'
    if cwd_candidate.exists():
        return cwd_candidate
    return Path('/home')


def _strip_cpp_comments(text: str) -> str:
    # remove /* */ comments
    text = re.sub(r'/\*.*?\*/', '', text, flags=re.S)
    # remove //... comments
    text = re.sub(r'//.*$', '', text, flags=re.M)
    return text


def _parse_flat_vector(file_path: Path, var_name: str):
    text = file_path.read_text(encoding='utf-8')
    text = _strip_cpp_comments(text)
    m = re.search(rf'{re.escape(var_name)}\s*=\s*\{{(.*?)\}};', text, re.S)
    if not m:
        return []
    content = m.group(1)
    nums = re.findall(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', content)
    return [float(n) for n in nums]


def _parse_single_float(file_path: Path, var_name: str):
    text = file_path.read_text(encoding='utf-8')
    text = _strip_cpp_comments(text)
    m = re.search(rf'{re.escape(var_name)}\s*=\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*;', text)
    if not m:
        return None
    try:
        return float(m.group(1))
    except Exception:
        return None


def _parse_matrix(file_path: Path, var_name: str):
    text = file_path.read_text(encoding='utf-8')
    text = _strip_cpp_comments(text)
    m = re.search(rf'{re.escape(var_name)}\s*=\s*\{{(.*?)\}};', text, re.S)
    if not m:
        return []
    block = m.group(1)
    rows = re.findall(r'\{([^}]*)\}', block)
    result = []
    for r in rows:
        nums = re.findall(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', r)
        if nums:
            result.append([float(n) for n in nums])
    return result


class IntegratedControlNode(Node):
    """統合コントロール: button_snapshot_logger + motor_manual_chokudo + thread_slack_or_taut"""

    def __init__(self) -> None:
        super().__init__('integrated_control_node')

        # ========== Snapshot Logger の初期化 ==========
        # CSVは初回スナップショット時に作成（遅延生成）
        self.csv_dir = os.path.expanduser('~/pickup_ws/angle_arucopose_csv/')
        self.filepath: Optional[str] = None

        # Latest data caches
        self.latest_angles: Optional[List[float]] = None
        self.latest_chokudo: Optional[float] = None
        self.latest_cameraswing: Optional[float] = None
        self.latest_markers: List[Tuple[int, ArucoMarkers]] = []
        self.data_lock = Lock()

        # ========== Thread Sensor Mode ==========
        # 現在のモード（t=通常, s=逆, None=無効）
        self.sensor_mode = None

        # ========== Header-based Presets ==========
        self.key_to_angles = {}
        self._stop_motor10 = None
        self._load_presets_from_headers()

        # ========== Subscriptions ==========
        self.create_subscription(Float32MultiArray, '/motor_current_angles', self._angle_cb, 10)
        self.create_subscription(Float32, '/chokudomotor/angle', self._chokudo_cb, 10)
        self.create_subscription(Float32, '/cameraswingmotor/angle', self._camera_swing_cb, 10)
        self.create_subscription(ArucoMarkers, '/aruco/markers', self._markers_cb, 10)
        self.create_subscription(Int8MultiArray, '/switch', self._switch_cb, 10)

        # ========== Publishers ==========
        self.motor_pub = self.create_publisher(Float32MultiArray, '/motor_angles', 10)
        self.chokudo_pub = self.create_publisher(Float32, '/chokudomotor/target_angle', 10)
        self.cameraswing_pub = self.create_publisher(Float32, '/cameraswingmotor/target_angle', 10)

        # Start keyboard listener
        self.listener = keyboard.Listener(on_press=self._on_key_press)
        self.listener.start()

        # キーボード入力スレッド
        thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        thread.start()

        self.get_logger().info("Integrated Control Node started.")
        self._print_help()

    def _load_presets_from_headers(self) -> None:
        """Load preset angles from C++ header files."""
        headers_dir = _find_headers_dir()
        self.get_logger().info(f'Looking for headers in: {headers_dir}')

        try:
            init_path = headers_dir / 'motor_initial_position.hpp'
            pickup_path = headers_dir / 'motor_pickup_position.hpp'
            narrow_path = headers_dir / 'narrow_space_controll_position.hpp'

            if init_path.exists():
                stop_angles = _parse_flat_vector(init_path, 'stop_angles_')
                if stop_angles:
                    self.key_to_angles['a'] = [float(x) for x in stop_angles]
                motor10 = _parse_single_float(init_path, 'stop_motor10_angle_')
                if motor10 is not None:
                    self._stop_motor10 = float(motor10)

            if pickup_path.exists():
                pickup_seq = _parse_matrix(pickup_path, 'pickup_sequence')
                pickup_keys = ['b', 'c', 'h', 'f', 'p']
                for k, seq in zip(pickup_keys, pickup_seq):
                    if seq:
                        self.key_to_angles[k] = [float(x) for x in seq]

            if narrow_path.exists():
                narrow_seq = _parse_matrix(narrow_path, 'narrow_sequence')
                if len(narrow_seq) >= 1:
                    self.key_to_angles['n'] = [float(x) for x in narrow_seq[0]]
                if len(narrow_seq) >= 2:
                    self.key_to_angles['m'] = [float(x) for x in narrow_seq[1]]
                if len(narrow_seq) >= 3:
                    self.key_to_angles['l'] = [float(x) for x in narrow_seq[2]]    

            if self.key_to_angles:
                keys_list = ', '.join(sorted(self.key_to_angles.keys()))
                self.get_logger().info(f'Loaded header presets for keys: [{keys_list}]')
            else:
                self.get_logger().warn('No header values found — using fallback hard-coded presets')

        except Exception as e:
            self.get_logger().error(f'Error parsing header files: {e}')

    def _init_csv(self) -> None:
        """CSV初期化"""
        header = ['timestamp'] + [f'motor{i + 1}' for i in range(9)] + ['chokudo'] + [
            'marker_id', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'
        ]
        with open(self.filepath, mode='w', newline='') as f:
            csv.writer(f).writerow(header)

    # ========== Callbacks ==========
    def _angle_cb(self, msg: Float32MultiArray) -> None:
        if len(msg.data) == 9:
            self.latest_angles = list(msg.data)
        else:
            self.get_logger().warn('Invalid motor angle length; expect 9 values.')

    def _chokudo_cb(self, msg: Float32) -> None:
        self.latest_chokudo = float(msg.data)

    def _camera_swing_cb(self, msg: Float32) -> None:
        self.latest_cameraswing = float(msg.data)

    def _markers_cb(self, msg: ArucoMarkers) -> None:
        with self.data_lock:
            filtered = []
            for mid, pose in zip(msg.marker_ids, msg.poses):
                if mid in (0, 1, 2):
                    filtered.append((mid, pose))
            self.latest_markers = filtered

    def _switch_cb(self, msg: Int8MultiArray) -> None:
        """センサー値による制御"""
        if self.sensor_mode is None:
            # モードが設定されていない場合はスキップ
            return

        if len(msg.data) != 9:
            self.get_logger().warn(f'Invalid /switch length: {len(msg.data)}')
            return

        new_angles = self.latest_angles.copy() if self.latest_angles else [0.0] * 9
        updated = False

        if self.sensor_mode == 't':
            for idx, val in enumerate(msg.data):
                if val == 0:
                    new_angles[idx] += 3.0
                    updated = True
                    self.get_logger().info(f'[Mode T] Switch[{idx+1}] = 0 → +3°')
        elif self.sensor_mode == 's':
            for idx, val in enumerate(msg.data):
                if val == 1:
                    new_angles[idx] -= 3.0
                    updated = True
                    self.get_logger().info(f'[Mode S] Switch[{idx+1}] = 1 → -3°')

        if updated:
            msg_pub = Float32MultiArray()
            msg_pub.data = new_angles
            self.motor_pub.publish(msg_pub)
            formatted = ', '.join(f'{a:.2f}' for a in new_angles)
            self.get_logger().info(f'Published new angles: [{formatted}]')

    # ========== Keyboard Handlers ==========
    def _on_key_press(self, key) -> None:
        """Called when a key is pressed (from pynput)."""
        try:
            # Check if 'z' key was pressed (snapshot)
            if key == keyboard.KeyCode(char='z'):
                self._log_snapshot()
        except AttributeError:
            # Special keys don't have a char attribute
            pass

    def keyboard_listener(self) -> None:
        """キーボード入力リスナー"""
        while True:
            line = sys.stdin.readline().strip()
            line_upper = line.upper()

            # ========== Header-based Presets (優先) ==========
            line_lower = line.lower()
            if line_lower in self.key_to_angles:
                full = self.key_to_angles[line_lower]
                # Publish to /motor_angles: first 9 elements
                motor_msg = Float32MultiArray()
                motor_msg.data = [float(x) for x in full[:9]]
                self.motor_pub.publish(motor_msg)
                formatted = ', '.join(f'{a:.2f}' for a in motor_msg.data)
                self.get_logger().info(f'Published preset "{line_lower}": [{formatted}]')
                # Publish chokudo if available
                if len(full) >= 10:
                    chokudo_val = float(full[9])
                elif self._stop_motor10 is not None:
                    chokudo_val = self._stop_motor10
                else:
                    chokudo_val = None
                if chokudo_val is not None:
                    chokudo_msg = Float32()
                    chokudo_msg.data = chokudo_val
                    self.chokudo_pub.publish(chokudo_msg)
                    self.get_logger().info(f'Published chokudo: {chokudo_val:.2f}')
                continue

            # ========== Mode Switch (t/s) ==========
            if line_upper == "T":
                if self.sensor_mode == 't':
                    self.sensor_mode = None
                    self.get_logger().info("Disabled Mode T")
                else:
                    self.sensor_mode = 't'
                    self.get_logger().info("Switched to Mode T: (switch==0 → +3°)")
                continue
            elif line_upper == "S":
                if self.sensor_mode == 's':
                    self.sensor_mode = None
                    self.get_logger().info("Disabled Mode S")
                else:
                    self.sensor_mode = 's'
                    self.get_logger().info("Switched to Mode S: (switch==1 → -3°)")
                continue

            # ========== Predefined Poses (fallback if not in header) ==========
            if line_upper == "A" and 'a' not in self.key_to_angles:
                target_angles = [280.37, 272.99, 232.03, 169.01, 56.60, 68.29, 318.43, 331.00, 106.08]
                msg = Float32MultiArray()
                msg.data = target_angles
                self.motor_pub.publish(msg)
                self.get_logger().info('Sent command: Set all motors to predefined pose "A" (fallback).')
                continue

            if line_upper == "K" and 'k' not in self.key_to_angles:
                target_angles = [280.28, 274.22, 232.73, 168.13, 61.96, 72.95, 415.72, 585.26, 445.34]
                msg = Float32MultiArray()
                msg.data = target_angles
                self.motor_pub.publish(msg)
                self.get_logger().info('Sent command: Set all motors to predefined pose "K" (fallback).')
                continue

            # ========== Chokudo Motor (直動機構) ==========
            if line_upper == "CA+":
                self._publish_chokudo_offset(+360.0)
            elif line_upper == "CA-":
                self._publish_chokudo_offset(-360.0)
            elif line_upper == "CA":
                self._publish_chokudo_offset(+4000.0)
            elif line_upper == "CB":
                self._publish_chokudo_offset(-4000.0)
            elif line_upper == "CAA":
                msg = Float32()
                msg.data = 168.0
                self.chokudo_pub.publish(msg)
                self.get_logger().info(f'Published to /chokudomotor/target_angle: 168.0 deg (absolute)')
            elif line_upper == "CBB":
                msg = Float32()
                msg.data = -14000.0
                self.chokudo_pub.publish(msg)
                self.get_logger().info(f'Published to /chokudomotor/target_angle: -14000.0 deg (absolute)')
            # ========== Camera Swing Motor ==========
            elif line_upper == "D+":
                self._publish_cameraswing_offset(-10.0)
            elif line_upper == "D++":
                self._publish_cameraswing_offset(-20.0)
            elif line_upper == "D+++":
                self._publish_cameraswing_offset(-30.0)
            elif line_upper == "D++++":
                self._publish_cameraswing_offset(-40.0)
            elif line_upper == "D-":
                self._publish_cameraswing_offset(+10.0)
            elif line_upper == "D--":
                self._publish_cameraswing_offset(+20.0)
            elif line_upper == "D---":
                self._publish_cameraswing_offset(+30.0)
            elif line_upper == "D----":
                self._publish_cameraswing_offset(+40.0)
            # ========== Motor Offset (A/B + number) ==========
            elif len(line_upper) >= 2 and line_upper[0] in ['A', 'B'] and line_upper[1:].isdigit():
                motor_index = int(line_upper[1:]) - 1
                if 0 <= motor_index < 9:
                    offset = +90.0 if line_upper[0] == 'A' else -90.0
                    self._send_offset_to_motor(motor_index, offset)
                else:
                    self.get_logger().warn(f'Motor index out of range: {motor_index + 1}')
            # ========== Absolute Motor Position (number + value) ==========
            else:
                abs_match = re.match(r'^(\d+)\s+([+-]?\d+(?:\.\d*)?)$', line)
                if abs_match:
                    motor_index = int(abs_match.group(1)) - 1
                    target_deg = float(abs_match.group(2))
                    if 0 <= motor_index < 9:
                        self._send_absolute_to_motor(motor_index, target_deg)
                    else:
                        self.get_logger().warn(f'Motor index out of range: {motor_index + 1}')
                elif line.lower() not in ('', 'z'):  # 'z'/'Z' is snapshot via pynput
                    self.get_logger().warn(f'Invalid input format: {line}')

    def _log_snapshot(self) -> None:
        """Log current state to CSV when 'z' is pressed."""
        with self.data_lock:
            if self.latest_angles is None or self.latest_chokudo is None:
                self.get_logger().warn('Snapshot pressed but angles not ready; skipping log.')
                return

            if not self.latest_markers:
                self.get_logger().warn('Snapshot pressed but no markers; skipping log.')
                return

            # 初回呼び出し時にCSVファイルを生成
            if self.filepath is None:
                os.makedirs(self.csv_dir, exist_ok=True)
                ts = datetime.datetime.now().strftime('%m%d_%H%M%S')
                fname = f'aruco_motor_integrated_log_{ts}.csv'
                self.filepath = os.path.join(self.csv_dir, fname)
                self._init_csv()
                self.get_logger().info(f'Created CSV: {self.filepath}')

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

    def _send_offset_to_motor(self, motor_index: int, offset_deg: float) -> None:
        """Send offset command to a single motor."""
        new_angles = self.latest_angles.copy() if self.latest_angles else [0.0] * 9
        new_angles[motor_index] = (new_angles[motor_index] + offset_deg)
        msg = Float32MultiArray()
        msg.data = new_angles
        self.motor_pub.publish(msg)
        self.get_logger().info(
            f'Sent command: motor {motor_index + 1} → {new_angles[motor_index]:.2f} deg')

    def _send_absolute_to_motor(self, motor_index: int, target_deg: float) -> None:
        """Send absolute position command to a single motor."""
        new_angles = self.latest_angles.copy() if self.latest_angles else [0.0] * 9
        new_angles[motor_index] = target_deg
        msg = Float32MultiArray()
        msg.data = new_angles
        self.motor_pub.publish(msg)
        self.get_logger().info(
            f'Sent command: motor {motor_index + 1} → {target_deg:.2f} deg (absolute)')

    def _publish_chokudo_offset(self, offset_deg: float) -> None:
        """Publish offset command to chokudo motor."""
        target = (self.latest_chokudo + offset_deg) if self.latest_chokudo else offset_deg
        msg = Float32()
        msg.data = target
        self.chokudo_pub.publish(msg)
        self.get_logger().info(f'Published to /chokudomotor/target_angle: {target:.2f} deg')

    def _publish_cameraswing_offset(self, offset_deg: float) -> None:
        """Publish offset command to camera swing motor."""
        target = (self.latest_cameraswing + offset_deg) if self.latest_cameraswing else offset_deg
        msg = Float32()
        msg.data = target
        self.cameraswing_pub.publish(msg)
        self.get_logger().info(f'Published to /cameraswingmotor/target_angle: {target:.2f} deg')

    def _print_help(self) -> None:
        """Print help message."""
        header_keys = ', '.join(sorted(self.key_to_angles.keys())) if self.key_to_angles else 'none'
        help_text = f"""
========== Integrated Control Node Help ==========
[Header Presets (優先)]
  {header_keys}     : Presets loaded from C++ headers

[Mode Switch]
  T           : Sensor Mode T (switch==0 → +3°)
  S           : Sensor Mode S (switch==1 → -3°)

[Predefined Poses (fallback)]
  A           : Set all motors to pose A (if not in header)
  K           : Set all motors to pose K (if not in header)

[Motor Control (Relative)]
  A1-A9       : +90° for motor 1-9
  B1-B9       : -90° for motor 1-9

[Motor Control (Absolute)]
  1 180       : Set motor 1 to 180°
  2 -45.5     : Set motor 2 to -45.5°

[Chokudo Motor (直動機構)]
  CA+  / CA-  : ±360°
  CA   / CB   : ±4000°
  CAA  / CBB  : ±14000°

[Camera Swing Motor]
  D+   / D-   : ±10°
  D++  / D--  : ±20°
  D+++ / D--- : ±30°
  D++++ / D---: ±40°

[Snapshot (from pynput)]
  Z (from keyboard) : Log current angles/pose to CSV

================================================
"""
        self.get_logger().info(help_text)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IntegratedControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
