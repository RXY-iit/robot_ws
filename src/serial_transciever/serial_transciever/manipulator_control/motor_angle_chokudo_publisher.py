import re
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32


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


class MotorAngleChokudoPublisher(Node):
    """Publishes presets to both /motor_angles (first 9) and /chokudo_angles (full vector).

    This node parses the same headers as the other publisher nodes and exposes
    the same key mappings. It keeps full-length vectors (including a possible
    10th value) and republishes appropriately.
    """

    def __init__(self):
        super().__init__('motor_angle_chokudo_publisher')
        self.motor_pub = self.create_publisher(Float32MultiArray, '/motor_angles', 10)
        self.chokudo_pub = self.create_publisher(Float32, '/chokudomotor/target_angle', 10)

        headers_dir = _find_headers_dir()
        self.get_logger().info(f'Looking for headers in: {headers_dir}')

        self.key_to_angles = {}
        try:
            init_path = headers_dir / 'motor_initial_position.hpp'
            pickup_path = headers_dir / 'motor_pickup_position.hpp'
            narrow_path = headers_dir / 'narrow_space_controll_position.hpp'

            if init_path.exists():
                stop_angles = _parse_flat_vector(init_path, 'stop_angles_')
                if stop_angles:
                    self.key_to_angles['a'] = [float(x) for x in stop_angles]
                # also try to read separate motor10 angle if present
                motor10 = _parse_single_float(init_path, 'stop_motor10_angle_')
                if motor10 is not None:
                    # store as separate fallback for keys that lack 10th element
                    self._stop_motor10 = float(motor10)
                else:
                    self._stop_motor10 = None

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

        except Exception as e:
            self.get_logger().error(f'Error parsing header files: {e}')

        if not self.key_to_angles:
            self.get_logger().warn('No header values found — falling back to built-in presets')
            # fallback with possible 9-element values
            self.key_to_angles = {
                'a': [257, 265, 202, 77, 20, 22, 146, 103, 36],
                'p': [334, 749, 1142, 335, 678, 1167, 668, 327, 201],
                'b': [168, 237, 178, 10, 16, -85, 42, 471, 527],
                'h': [168, 366, 852, 1, 185, 1062, 659, 846, 207],
                'c': [168, 237, 178, 10, 16, -85, 668, 471, 527],
                'f': [342, 743, 1012, 327, 224, 1213, 659, 846, 207],
            }

        keys_list = ', '.join(sorted(self.key_to_angles.keys()))
        self.get_logger().info(f'Press one of these keys + Enter to publish preset angles: [{keys_list}]')

        self.run()

    def run(self):
        try:
            while rclpy.ok():
                key = input('Enter key: ').strip().lower()
                if not key:
                    continue
                if key in self.key_to_angles:
                    full = self.key_to_angles[key]
                    # Publish to /motor_angles: first 9 elements (if available)
                    motor_msg = Float32MultiArray()
                    motor_msg.data = [float(x) for x in full[:9]]
                    self.motor_pub.publish(motor_msg)
                    # Publish chokudo: if 10th element present use it, else fallback to _stop_motor10 if available
                    if len(full) >= 10:
                        chokudo_val = float(full[9])
                    else:
                        chokudo_val = getattr(self, '_stop_motor10', None)
                    if chokudo_val is not None:
                        chokudo_msg = Float32()
                        chokudo_msg.data = chokudo_val
                        self.chokudo_pub.publish(chokudo_msg)
                        self.get_logger().info(f'Published to /motor_angles and /chokudomotor/target_angle for key "{key}" (chokudo={chokudo_val})')
                    else:
                        self.get_logger().info(f'Published to /motor_angles for key "{key}" (no chokudo value available)')
                else:
                    self.get_logger().warn(f'Invalid key "{key}". Available: {list(self.key_to_angles.keys())}')
        except KeyboardInterrupt:
            self.get_logger().info('Publisher stopped by user.')


def main(args=None):
    rclpy.init(args=args)
    node = MotorAngleChokudoPublisher()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
