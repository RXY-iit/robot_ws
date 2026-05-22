import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
import threading
import time
import sys
import re
import itertools

class OffsetCommandNode(Node):
    def __init__(self):
        super().__init__('angle_sequence_controller_node')

        # --- 現在値の保持 ---
        self.current_angles = [0.0] * 9
        self.current_chokudo = 0.0
        self.current_cameraswing = 0.0

        # --- シーケンス制御用の変数 ---
        self.is_sequence_running = False
        self.sequence_thread = None
        self.target_angles_sequence = [0.0] * 9
        self.move_complete_event = threading.Event()
        self.ANGLE_TOLERANCE = 20.0  # 動作完了とみなす許容誤差 (度)

        # --- Subscribers ---
        self.create_subscription(Float32MultiArray, '/motor_current_angles', self.current_angle_callback, 10)
        self.create_subscription(Float32, '/chokudomotor/angle', self.chokudo_callback, 10)
        self.create_subscription(Float32, '/cameraswingmotor/angle', self.camera_swing_callback, 10)

        # --- Publishers ---
        self.publisher = self.create_publisher(Float32MultiArray, '/motor_angles', 10)
        self.chokudo_pub = self.create_publisher(Float32, '/chokudomotor/target_angle', 10)
        self.cameraswing_pub = self.create_publisher(Float32, '/cameraswingmotor/target_angle', 10)

        # --- キーボード入力スレッドを開始 ---
        thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        thread.start()
        self.get_logger().info(
            '\n--- Robot Controller Ready ---\n'
            '1. Set base position with: GOTO <ang1> <ang2> ... <ang9>\n'
            '2. Run sequence with: RUN_SEQUENCE <off1> <off2> ...\n'
            '   (Example: RUN_SEQUENCE 0 90 180 270)\n'
            '3. Stop sequence with: STOP\n'
            '--------------------------------'
        )

    def current_angle_callback(self, msg):
        if len(msg.data) == 9:
            self.current_angles = list(msg.data)
            if self.is_sequence_running and self.check_move_completion():
                self.move_complete_event.set()
        else:
            self.get_logger().warn(f'Invalid current_angles length: {len(msg.data)}')

    def chokudo_callback(self, msg): self.current_chokudo = msg.data
    def camera_swing_callback(self, msg): self.current_cameraswing = msg.data

    def check_move_completion(self):
        for i in range(9):
            if abs(self.current_angles[i] - self.target_angles_sequence[i]) > self.ANGLE_TOLERANCE:
                return False
        return True

    def keyboard_listener(self):
        while True:
            line = sys.stdin.readline().strip().upper()
            parts = line.split()
            if not parts: continue
            command = parts[0]

            if self.is_sequence_running and command not in ["STOP"]:
                self.get_logger().warn('Sequence is running. Type "STOP" to interrupt.')
                continue

            # === コマンド処理 ===
            if command == "GOTO":
                self.handle_goto(parts)
            elif command == "RUN_SEQUENCE":
                self.handle_run_sequence(parts)
            elif command == "STOP":
                self.handle_stop()
            else: # 既存の手動制御コマンド
                self.handle_manual_commands(line)

    def handle_goto(self, parts):
        if len(parts) != 10:
            self.get_logger().warn('Usage: GOTO <ang1> <ang2> ... <ang9>')
            return
        try:
            target_angles = [float(p) for p in parts[1:]]
            self.get_logger().info(f"Moving to base position: {target_angles}")
            msg = Float32MultiArray(data=target_angles)
            self.publisher.publish(msg)
        except ValueError:
            self.get_logger().warn('Invalid angle format. Please provide 9 numbers.')

    def handle_run_sequence(self, parts):
        if len(parts) < 2:
            self.get_logger().warn('Usage: RUN_SEQUENCE <offset1> <offset2> ...')
            return
        try:
            offsets = [float(p) for p in parts[1:]]
            if not self.is_sequence_running:
                self.is_sequence_running = True
                self.get_logger().info(f"Starting sequence with offsets: {offsets}")
                self.sequence_thread = threading.Thread(
                    target=self.run_combination_sequence, args=(offsets,), daemon=True)
                self.sequence_thread.start()
            else:
                self.get_logger().info('Sequence is already running.')
        except ValueError:
            self.get_logger().warn('Invalid offset format. Please provide numbers.')

    def handle_stop(self):
        if self.is_sequence_running:
            self.get_logger().info('Stopping sequence...')
            self.is_sequence_running = False
            self.move_complete_event.set()
            if self.sequence_thread:
                self.sequence_thread.join()
            self.get_logger().info('Sequence stopped.')
        else:
            self.get_logger().info('Sequence is not running.')

    def run_combination_sequence(self, offsets):
        base_angles = self.current_angles.copy()
        self.get_logger().info(f"Sequence starting from base angles: {[f'{a:.1f}' for a in base_angles]}")

        # 各モーターの目標角度リストを作成
        # 例: motor1_targets = [base1+off1, base1+off2, ...]
        motor_target_lists = [[base_angle + offset for offset in offsets] for base_angle in base_angles]

        # 全組み合わせを生成
        all_combinations = list(itertools.product(*motor_target_lists))
        total_combinations = len(all_combinations)
        self.get_logger().info(f'Total combinations to run: {total_combinations}')

        for i, combination in enumerate(all_combinations):
            if not self.is_sequence_running:
                self.get_logger().info('Sequence was interrupted.')
                break

            self.target_angles_sequence = list(combination)
            msg = Float32MultiArray(data=self.target_angles_sequence)
            self.publisher.publish(msg)

            formatted_targets = ', '.join(f'{v:.1f}' for v in self.target_angles_sequence)
            self.get_logger().info(f'[{i + 1}/{total_combinations}] Sending: [{formatted_targets}]')

            self.move_complete_event.clear()
            completed = self.move_complete_event.wait(timeout=15.0) # タイムアウトを少し長めに設定

            if not completed and self.is_sequence_running:
                self.get_logger().warn('Move did not complete within timeout. Proceeding anyway.')

        if self.is_sequence_running:
            self.get_logger().info('Sequence finished.')
            self.is_sequence_running = False

    def handle_manual_commands(self, line):
        # 既存の手動制御（A1, B1, CA+ など）のロジックをここに記述
        # この部分は元のコードから変更ありません
        if len(line) >= 2 and line[0] in ['A', 'B'] and line[1:].isdigit():
            motor_index = int(line[1:]) - 1
            if 0 <= motor_index < 9:
                offset = +90.0 if line[0] == 'A' else -90.0
                new_angles = self.current_angles.copy()
                new_angles[motor_index] += offset
                self.publisher.publish(Float32MultiArray(data=new_angles))
                self.get_logger().info(f'Sent command: motor {motor_index + 1} -> {new_angles[motor_index]:.2f} deg')
            else: self.get_logger().warn(f'Motor index out of range: {motor_index + 1}')
        elif line == "CA+": self.chokudo_pub.publish(Float32(data=self.current_chokudo + 360.0))
        # ... 他の手動コマンドも同様に記述 ...
        else:
            self.get_logger().warn(f'Unknown command: {line}')

def main(args=None):
    rclpy.init(args=args)
    node = OffsetCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()