import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SerialControlNode(Node):
    def __init__(self):
        super().__init__('serial_node')

        # --- シリアルポート初期化 ---
        try:
            self.ser = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
            self.get_logger().info("Serial port opened")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            exit(1)

        # --- サブスクライブ開始 ---
        self.subscription = self.create_subscription(
            String,
            '/control_command',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        command = msg.data.strip()
        self.get_logger().info(f"Received command: '{command}'")

        # --- OpenRBに送信 ---
        try:
            self.ser.write((command + '\n').encode())
            self.get_logger().info(f"Sent to serial: {command}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def destroy_node(self):
        self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
