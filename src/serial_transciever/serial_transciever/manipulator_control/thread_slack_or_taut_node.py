import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8MultiArray
import threading
import sys

class SensorMotorKeyboardSwitchNode(Node):
    def __init__(self):
        super().__init__('sensor_motor_keyboard_switch_node')

        # 現在のモータ角度
        self.current_angles = [0.0] * 9

        # 現在のモード（t=通常, s=逆）
        self.mode = 't'

        # --- 購読 ---
        self.angle_sub = self.create_subscription(
            Float32MultiArray,
            '/motor_current_angles',
            self.current_angle_callback,
            10
        )

        self.switch_sub = self.create_subscription(
            Int8MultiArray,
            '/switch',
            self.switch_callback,
            10
        )

        # --- パブリッシャー ---
        self.motor_pub = self.create_publisher(Float32MultiArray, '/motor_angles', 10)

        # --- キーボード入力スレッド ---
        thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        thread.start()

        self.get_logger().info("Node started. Press 't' for +3° mode (switch=0), 's' for -3° mode (switch=1).")

    # 現在角度更新
    def current_angle_callback(self, msg: Float32MultiArray):
        if len(msg.data) == 9:
            self.current_angles = list(msg.data)
        else:
            self.get_logger().warn(f'Invalid /motor_current_angles length: {len(msg.data)}')

    # センサー値による制御
    def switch_callback(self, msg: Int8MultiArray):
        if len(msg.data) != 9:
            self.get_logger().warn(f'Invalid /switch length: {len(msg.data)}')
            return

        new_angles = self.current_angles.copy()
        updated = False

        if self.mode == 't':
            for idx, val in enumerate(msg.data):
                if val == 0:
                    new_angles[idx] += 3.0
                    updated = True
                    self.get_logger().info(f'[Mode T] Switch[{idx+1}] = 0 → +3°')
        elif self.mode == 's':
            for idx, val in enumerate(msg.data):
                if val == 1:
                    new_angles[idx] -= 3.0
                    updated = True
                    self.get_logger().info(f'[Mode B] Switch[{idx+1}] = 1 → -3°')

        if updated:
            msg_pub = Float32MultiArray()
            msg_pub.data = new_angles
            self.motor_pub.publish(msg_pub)
            formatted = ', '.join(f'{a:.2f}' for a in new_angles)
            self.get_logger().info(f'Published new angles: [{formatted}]')

    # キーボードでモードを切り替え
    def keyboard_listener(self):
        while True:
            key = sys.stdin.readline().strip().lower()
            if key == 't':
                self.mode = 't'
                self.get_logger().info("Switched to Mode T: (switch==0 → +3°)")
            elif key == 's':
                self.mode = 's'
                self.get_logger().info("Switched to Mode S: (switch==1 → -3°)")
            else:
                self.get_logger().warn(f'Unknown key: {key}')

def main(args=None):
    rclpy.init(args=args)
    node = SensorMotorKeyboardSwitchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
