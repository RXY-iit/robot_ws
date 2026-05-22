import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Int8MultiArray
import threading
import time


class AllmotorManualSwitchNode(Node):
    def __init__(self):
        super().__init__('allmotor_manual_switch_node')

        self.current_angles = [0.0] * 9
        self.current_chokudo = 0.0  # chokudo motor の現在角度

        # モータ配列の現在値
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/motor_current_angles',
            self.current_angle_callback,
            10
        )

        # chokudo motor の現在角度
        self.chokudo_sub = self.create_subscription(
            Float32,
            '/chokudomotor/angle',
            self.chokudo_callback,
            10
        )

        # cameraswing motor の現在角度
        self.cameraswing_sub = self.create_subscription(
            Float32,
            '/cameraswingmotor/angle',
            self.camera_swing_callback,
            10
        )
        # --- 追加: スイッチ状態保持 ---
        self.prev_switch_state = [0] * 9
        # --- 追加: キーボードによる手動指令を記録するタイムスタンプ（秒） ---
        self.manual_command_time = [0.0] * 9
        # 手動指令があったとみなしてたるみ除去から除外する時間（秒）
        self.manual_exclude_timeout = 2.0

        # --- 追加: /switch 購読 ---
        self.switch_sub = self.create_subscription(
            Int8MultiArray,
            '/switch',
            self.switch_callback,
            10
        )

        # 指令用 publishers
        self.publisher = self.create_publisher(Float32MultiArray, '/motor_angles', 10)
        self.chokudo_pub = self.create_publisher(Float32, '/chokudomotor/target_angle', 10)
        self.cameraswing_pub = self.create_publisher(Float32, '/cameraswingmotor/target_angle', 10)

        # キーボード入力スレッド
        thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        thread.start()

    def current_angle_callback(self, msg):
        if len(msg.data) == 9:
            self.current_angles = list(msg.data)
            formatted = ', '.join(f'{v:.2f}' for v in self.current_angles)
            self.get_logger().info(f'Current angles updated: [{formatted}]')
        else:
            self.get_logger().warn(f'Invalid current_angles length: {len(msg.data)}')
    def chokudo_callback(self, msg):
        self.current_chokudo = msg.data
        self.get_logger().info(f'Chokudo angle updated: {self.current_chokudo:.2f} deg')

    def camera_swing_callback(self, msg):
        self.current_cameraswing = msg.data
        self.get_logger().info(f'Cameraswing angle updated: {self.current_cameraswing:.2f} deg')

    def keyboard_listener(self):
        import sys, re          # ← 先頭で re を追加
        while True:
            line = sys.stdin.readline().strip().upper()
            #直動機構モーター
            if line == "CA+": # 上がる
                self.publish_chokudo_offset(+360.0)
            elif line == "CA-": # 下がる
                self.publish_chokudo_offset(-360.0)
            elif line == "CA": #上がる
                self.publish_chokudo_offset(+4000.0)
            elif line == "CB": # 下がる
                self.publish_chokudo_offset(-4000.0)
            elif line == "CAA": #上がる
                self.publish_chokudo_offset(+14000.0)
            elif line == "CBB": # 下がる
                self.publish_chokudo_offset(-14000.0)
            #カメラスイングモーター
            elif line == "D+": #下げる
                self.publish_cameraswing_offset(+70.0)
            elif line == "D-": #上げる
                self.publish_cameraswing_offset(-70.0)
                                    
            elif len(line) >= 2 and line[0] in ['A', 'B'] and line[1:].isdigit():
                motor_index = int(line[1:]) - 1
                if 0 <= motor_index < 9:
                    offset = +90.0 if line[0] == 'A' else -90.0
                    self.send_offset_to_motor(motor_index, offset)
                else:
                    self.get_logger().warn(f'Motor index out of range: {motor_index + 1}')
            else:
                self.get_logger().warn(f'Invalid input format: {line}')
            # 例: 3 180   → motor-3 を 180deg に設定
            abs_match = re.match(r'^(\d+)\s+([+-]?\d+(?:\.\d*)?)$', line)
            if abs_match:
                motor_index = int(abs_match.group(1)) - 1
                target_deg  = float(abs_match.group(2))
                if 0 <= motor_index < 9:
                    self.send_absolute_to_motor(motor_index, target_deg)
                else:
                    self.get_logger().warn(f'Motor index out of range: {motor_index + 1}')
                continue   # 次のループへ            
            

    def send_offset_to_motor(self, motor_index, offset_deg):
        new_angles = self.current_angles.copy()
        new_angles[motor_index] = (new_angles[motor_index] + offset_deg)
        # キーボードからの指令としてタイムスタンプを記録
        try:
            self.manual_command_time[motor_index] = time.time()
        except Exception:
            pass
        # internal state にも指令値を反映しておく（後続の publish が古い値で上書きしないように）
        try:
            self.current_angles[motor_index] = new_angles[motor_index]
        except Exception:
            pass
        # 追加ログ: publish 直前の配列とタイムスタンプ確認
        try:
            self.get_logger().info(f'Publishing (keyboard offset) data={[f"{v:.2f}" for v in new_angles]} manual_time={self.manual_command_time[motor_index]:.3f}')
        except Exception:
            pass
        msg = Float32MultiArray()
        msg.data = new_angles
        self.publisher.publish(msg)
        self.get_logger().info(
            f'Sent command: motor {motor_index + 1} → {new_angles[motor_index]:.2f} deg')

    def send_absolute_to_motor(self, motor_index, target_deg):
        new_angles = self.current_angles.copy()
        new_angles[motor_index] = target_deg
        # キーボードからの指令としてタイムスタンプを記録
        try:
            self.manual_command_time[motor_index] = time.time()
        except Exception:
            pass
        # internal state にも指令値を反映しておく（後続の publish が古い値で上書きしないように）
        try:
            self.current_angles[motor_index] = new_angles[motor_index]
        except Exception:
            pass
        # 追加ログ: publish 直前の配列とタイムスタンプ確認
        try:
            self.get_logger().info(f'Publishing (keyboard absolute) data={[f"{v:.2f}" for v in new_angles]} manual_time={self.manual_command_time[motor_index]:.3f}')
        except Exception:
            pass
        msg = Float32MultiArray()
        msg.data = new_angles
        self.publisher.publish(msg)
        self.get_logger().info(
            f'Sent command: motor {motor_index + 1} → {target_deg:.2f} deg (absolute)')

    def publish_chokudo_offset(self, offset_deg):
        target = self.current_chokudo + offset_deg
        msg = Float32()
        msg.data = target
        self.chokudo_pub.publish(msg)
        self.get_logger().info(f'Published to /chokudomotor/angle: {target:.2f} deg')
    
    def publish_cameraswing_offset(self, offset_deg):
        target = self.current_cameraswing + offset_deg
        msg = Float32()
        msg.data = target
        self.cameraswing_pub.publish(msg)
        self.get_logger().info(f'Published to /cameraswingmotor/angle: {target:.2f} deg')
    # ---------- 新規コールバック ----------
    def switch_callback(self, msg: Int8MultiArray):
        if len(msg.data) != 9:
            self.get_logger().warn(f'Invalid /switch length: {len(msg.data)}')
            return
        # いまの角度を一度コピー
        new_angles = self.current_angles.copy()
        updated = False
        now = time.time()

        for idx, val in enumerate(msg.data):
            if val == 0:
                # 直近でキーボードから指令が来ているモータはたるみ除去から除外
                if now - self.manual_command_time[idx] <= self.manual_exclude_timeout:
                    self.get_logger().info(f'Switch[{idx+1}] = 0 but skipped (recent manual command)')
                    continue
                new_angles[idx] += 3.0     # +3° 加算
                updated = True
                self.get_logger().info(f'Switch[{idx+1}] = 0 → motor +3 deg')
        # 追加ログ: switch で publish する前に送信データと各モータの manual_command_time を出す
        if updated:
            try:
                times_str = [f'{t:.3f}' for t in self.manual_command_time]
                self.get_logger().info(f'Publishing (switch) data={[f"{v:.2f}" for v in new_angles]} manual_times={times_str}')
            except Exception:
                pass
        # ひとつでも更新があれば publish
        if updated:
            msg_pub = Float32MultiArray()
            msg_pub.data = new_angles
            self.publisher.publish(msg_pub)

def main(args=None):
    rclpy.init(args=args)
    node = AllmotorManualSwitchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
