import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
import threading
import sys, re

class OffsetCommandNode(Node):
    def __init__(self):
        super().__init__('angle_serial_manual_node')

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
        while True:
            line = sys.stdin.readline().strip().upper()

            if line == "A":
                # 'a' が入力されたら、すべてのモーターを指定の角度に設定
                target_angles = [280.37, 272.99, 232.03, 169.01, 56.60, 68.29, 318.43, 331.00, 106.08]
                msg = Float32MultiArray()
                msg.data = target_angles
                self.publisher.publish(msg)
                self.get_logger().info(f'Sent command: Set all motors to predefined pose "A".')
                continue # 他の条件と一致しないように次のループへ

            if line == "K":
                # 'k' が入力されたら、すべてのモーターを指定の角度に設定
                target_angles = [280.28, 274.22, 232.73, 168.13, 61.96, 72.95, 415.72, 585.26, 445.34]
                msg = Float32MultiArray()
                msg.data = target_angles
                self.publisher.publish(msg)
                self.get_logger().info('Sent command: Set all motors to predefined pose "K".')
                continue

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
            elif line == "D+": #上げる
                self.publish_cameraswing_offset(-10.0)
            elif line == "D++": #上げる
                self.publish_cameraswing_offset(-20.0)
            elif line == "D+++": #上げる
                self.publish_cameraswing_offset(-30.0)
            elif line == "D++++": #上げる
                self.publish_cameraswing_offset(-40.0)
            elif line == "D-": #下げる
                self.publish_cameraswing_offset(+10.0)
            elif line == "D--": #下げる
                self.publish_cameraswing_offset(+20.0)
            elif line == "D---": #下げる
                self.publish_cameraswing_offset(+30.0)
            elif line == "D----": #下げる
                self.publish_cameraswing_offset(+40.0)
                                    
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
                if motor_index == 9:
                    self.publish_chokudo_offset(target_deg)
                continue   # 次のループへ


    def send_offset_to_motor(self, motor_index, offset_deg):
        new_angles = self.current_angles.copy()
        new_angles[motor_index] = (new_angles[motor_index] + offset_deg)
        msg = Float32MultiArray()
        msg.data = new_angles
        self.publisher.publish(msg)
        self.get_logger().info(
            f'Sent command: motor {motor_index + 1} → {new_angles[motor_index]:.2f} deg')

    def send_absolute_to_motor(self, motor_index, target_deg):
        new_angles = self.current_angles.copy()
        new_angles[motor_index] = target_deg
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
