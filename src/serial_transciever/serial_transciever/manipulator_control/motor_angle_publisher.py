import re
import os
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8MultiArray
import threading
import sys
import time


def _find_headers_dir() -> Path:
    # Try to locate the headers directory relative to this file by walking upwards
    current = Path(__file__).resolve()
    while True:
        candidate = current
        headers_dir = candidate / 'src' / 'hose_control' / 'include' / 'hose_control'
        if headers_dir.exists():
            return headers_dir
        if current.parent == current:
            break
        current = current.parent
    # fallback to workspace root or current working dir
    cwd_candidate = Path.cwd() / 'src' / 'hose_control' / 'include' / 'hose_control'
    if cwd_candidate.exists():
        return cwd_candidate
    return Path('/home')  # fallback non-existent path


def _strip_cpp_comments(text: str) -> str:
    # remove /* */ comments
    text = re.sub(r'/\*.*?\*/', '', text, flags=re.S)
    # remove //... comments
    text = re.sub(r'//.*$', '', text, flags=re.M)
    return text


def _parse_flat_vector(file_path: Path, var_name: str):
    text = file_path.read_text(encoding='utf-8')
    # strip C/C++ comments so commented-out initializers are ignored
    text = _strip_cpp_comments(text)
    # match var_name = { ... };
    m = re.search(rf'{re.escape(var_name)}\s*=\s*\{{(.*?)\}};', text, re.S)
    if not m:
        return []
    content = m.group(1)
    nums = re.findall(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', content)
    return [float(n) for n in nums]


def _parse_matrix(file_path: Path, var_name: str):
    text = file_path.read_text(encoding='utf-8')
    # strip C/C++ comments so commented-out rows are ignored
    text = _strip_cpp_comments(text)
    m = re.search(rf'{re.escape(var_name)}\s*=\s*\{{(.*?)\}};', text, re.S)
    if not m:
        return []
    block = m.group(1)
    # find inner rows { ... }
    rows = re.findall(r'\{([^}]*)\}', block)
    result = []
    for r in rows:
        nums = re.findall(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', r)
        if nums:
            result.append([float(n) for n in nums])
    return result


class MotorAnglePublisher(Node):
    def __init__(self):
        super().__init__('motor_angle_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/motor_angles', 10)
        # subscribe to current motor angles so we can apply +/- adjustments
        self.current_angles = [0.0] * 9
        self._angles_lock = threading.Lock()
        # store latest /switch message
        self.last_switch = None
        self._switch_lock = threading.Lock()
        try:
            self.angle_sub = self.create_subscription(
                Float32MultiArray,
                '/motor_current_angles',
                self._current_angle_callback,
                10
            )
            self.switch_sub = self.create_subscription(
                Int8MultiArray,
                '/switch',
                self._switch_callback,
                10
            )
        except Exception:
            # if rclpy isn't fully initialized or topic not present, ignore subscription errors
            pass

        # mode: 'a' => apply +3 where switch==0; 'b' => apply -3 where switch==1
        self.mode = 'a'

        headers_dir = _find_headers_dir()
        self.get_logger().info(f'Looking for headers in: {headers_dir}')

        # default empty maps
        self.key_to_angles = {}

        try:
            init_path = headers_dir / 'motor_initial_position.hpp'
            pickup_path = headers_dir / 'motor_pickup_position.hpp'
            narrow_path = headers_dir / 'narrow_space_controll_position.hpp'

            if init_path.exists():
                stop_angles = _parse_flat_vector(init_path, 'stop_angles_')
                if stop_angles:
                    # ensure floats and take first 9 if header has extra
                    self.key_to_angles['a'] = [float(x) for x in stop_angles[:9]]

            if pickup_path.exists():
                pickup_seq = _parse_matrix(pickup_path, 'pickup_sequence')
                # map keys in the order the header lists them (b, c, h, f, p)
                pickup_keys = ['b', 'c', 'h', 'f', 'p']
                for k, seq in zip(pickup_keys, pickup_seq):
                    if seq:
                        self.key_to_angles[k] = [float(x) for x in seq[:9]]

            if narrow_path.exists():
                narrow_seq = _parse_matrix(narrow_path, 'narrow_sequence')
                # map 'n' and 'm' to first two narrow entries if present
                if len(narrow_seq) >= 1:
                    self.key_to_angles['n'] = [float(x) for x in narrow_seq[0][:9]]
                if len(narrow_seq) >= 2:
                    self.key_to_angles['m'] = [float(x) for x in narrow_seq[1][:9]]

        except Exception as e:
            self.get_logger().error(f'Error parsing header files: {e}')

        if not self.key_to_angles:
            # fallback to original hardcoded mapping if parsing failed
            self.get_logger().warn('No header values found — falling back to built-in presets')
            self.key_to_angles = {
                "a": [257, 265, 202, 77, 20, 22, 146, 103, 36],
                "p": [334, 749, 1142, 335, 678, 1167, 668, 327, 201],
                "b": [168, 237, 178, 10, 16, -85, 42, 471, 527],
                "h": [168, 366, 852, 1, 185, 1062, 659, 846, 207],
                "c": [168, 237, 178, 10, 16, -85, 668, 471, 527],
                "e": [168, 237, 852, 1, 57, 1062, 659, 846, 207],
                "f": [342, 743, 1012, 327, 224, 1213, 659, 846, 207],
                "g": [342, 1051, 1017, 328, 389, 1062, 662, 849, 207]
            }

        keys_list = ', '.join(sorted(self.key_to_angles.keys()))
        # also support s (plus) and t (minus) for small adjustments using /switch topic
        self.get_logger().info(f'Press one of these keys + Enter to publish preset angles: [{keys_list}], s=+3 per /switch, t=-3 per /switch')

        # start a background rclpy.spin so subscriptions are serviced while input loop runs
        # automation thread handles s/t loops; initialized as None
        self._auto_thread = None
        self._auto_stop = None

        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

        # run blocking input loop
        self.run()

    def run(self):
        try:
            while rclpy.ok():
                # use stdin.readline() to behave well with background spin thread
                key = sys.stdin.readline().strip().lower()
                if not key:
                    continue
                if key in self.key_to_angles:
                    # if auto mode is running, stop it before sending a preset
                    if self._auto_thread and self._auto_thread.is_alive():
                        self.get_logger().info('Stopping auto mode to execute preset command')
                        self.stop_auto()
                    msg = Float32MultiArray()
                    msg.data = [float(x) for x in self.key_to_angles[key]]
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Published angles for key "{key}": {msg.data}')
                elif key == 's':
                    # start auto-tighten mode: for indices where /switch == 0, rotate until they become 1
                    self.start_auto('s')
                elif key == 't':
                    # start auto-loosen mode: for indices where /switch == 1, rotate until they become 0
                    self.start_auto('t')
                else:
                    self.get_logger().warn(f'Invalid key "{key}". Available: {list(self.key_to_angles.keys())}')
        except KeyboardInterrupt:
            self.get_logger().info('Publisher stopped by user.')

    def _spin(self):
        try:
            rclpy.spin(self)
        except Exception:
            # spin may exit if rclpy.shutdown is called; ignore
            pass

    def start_auto(self, mode: str):
        # mode: 's' -> tighten until switch==1 (apply +3 when switch==0)
        # mode: 't' -> loosen until switch==0 (apply -3 when switch==1)
        if self._auto_thread and self._auto_thread.is_alive():
            self.get_logger().warn('Auto mode already running; press nothing or wait for completion')
            return
        self._auto_stop = threading.Event()
        self._auto_thread = threading.Thread(target=self._auto_worker, args=(mode,), daemon=True)
        self._auto_thread.start()

    def stop_auto(self, wait: float = 1.0):
        # Signal auto thread to stop and optionally wait for join
        if self._auto_thread and self._auto_thread.is_alive():
            self.get_logger().info('Signalling auto thread to stop...')
            if self._auto_stop:
                self._auto_stop.set()
            # join with timeout
            self._auto_thread.join(timeout=wait)
            if self._auto_thread.is_alive():
                self.get_logger().warn('Auto thread did not exit within timeout')
            else:
                self.get_logger().info('Auto thread stopped')

    def _auto_worker(self, mode: str):
        target = 1 if mode == 's' else 0
        step = 3.0 if mode == 's' else -3.0
        self.get_logger().info(f'Starting auto mode {mode} (step {step}, target switch value {target})')
        try:
            while not (self._auto_stop and self._auto_stop.is_set()):
                with self._switch_lock, self._angles_lock:
                    if self.last_switch is None or len(self.current_angles) == 0:
                        # wait for data
                        pass
                    else:
                        L = min(len(self.last_switch), len(self.current_angles))
                        indices = [i for i in range(L) if int(self.last_switch[i]) != target]
                        if not indices:
                            self.get_logger().info(f'Auto mode {mode} complete (all switches == {target})')
                            break
                        # prepare new angles by applying step to indices
                        new_angles = list(self.current_angles)
                        for i in indices:
                            new_angles[i] = new_angles[i] + step
                        msg = Float32MultiArray()
                        msg.data = new_angles
                        self.publisher_.publish(msg)
                        self.get_logger().debug(f'Auto mode {mode} published for indices {indices}')
                time.sleep(0.2)
        finally:
            # cleanup
            self._auto_stop = None
            self._auto_thread = None
            self.get_logger().info(f'Auto mode {mode} exited')

    def _current_angle_callback(self, msg: Float32MultiArray):
        if not hasattr(msg, 'data'):
            return
        with self._angles_lock:
            try:
                if len(msg.data) == 9:
                    self.current_angles = list(msg.data)
                else:
                    # accept other lengths but store
                    self.current_angles = list(msg.data)
            except Exception:
                pass

    def _switch_callback(self, msg: Int8MultiArray):
        # store latest switch message as list of ints (do not apply changes automatically)
        if not hasattr(msg, 'data'):
            return
        with self._switch_lock:
            try:
                self.last_switch = [int(x) for x in msg.data]
                self.get_logger().debug(f'Received /switch: {self.last_switch}')
            except Exception:
                self.last_switch = None


def main(args=None):
    rclpy.init(args=args)
    node = MotorAnglePublisher()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
