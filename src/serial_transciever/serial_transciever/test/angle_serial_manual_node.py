import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import threading

class OffsetCommandNode(Node):
    def __init__(self):
        super().__init__('angle_serial_manual_node')

        self.current_angles = [0.0] * 9

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/motor_current_angles',
            self.current_angle_callback,
            10
        )

        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/motor_angles',
            10
        )

        # 別スレッドでキーボード入力待機
        thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        thread.start()

    def current_angle_callback(self, msg):
        if len(msg.data) == 9:
            self.current_angles = list(msg.data)
            self.get_logger().info(f'Current angles updated: {self.current_angles}')
        else:
            self.get_logger().warn(f'Invalid current_angles length: {len(msg.data)}')

    def keyboard_listener(self):
        import sys
        while True:
            line = sys.stdin.readline().strip().upper()
            if len(line) >= 2 and line[0] in ['A', 'B'] and line[1:].isdigit():
                motor_index = int(line[1:]) - 1
                if 0 <= motor_index < 9:
                    offset = +90.0 if line[0] == 'A' else -90.0
                    self.send_offset_to_motor(motor_index, offset)
                else:
                    self.get_logger().warn(f'Motor index out of range: {motor_index + 1}')
            else:
                self.get_logger().warn(f'Invalid input format: {line}')

    def send_offset_to_motor(self, motor_index, offset_deg):
        new_angles = self.current_angles.copy()
        new_angles[motor_index] = (new_angles[motor_index] + offset_deg)
        msg = Float32MultiArray()
        msg.data = new_angles
        self.publisher.publish(msg)
        self.get_logger().info(
            f'Sent command: motor {motor_index + 1} → {new_angles[motor_index]:.2f} deg')

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
