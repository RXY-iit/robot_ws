import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy

class JoyOffsetCommandNode(Node):
    def __init__(self):
        super().__init__('joy_angle_command_node')

        self.current_angles = [0.0] * 9
        self.motor_index = 0  # 現在選択中のモーター番号
        self.prev_buttons = []
        self.prev_axes = []

        self.subscription_joy = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.subscription_angles = self.create_subscription(
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

    def current_angle_callback(self, msg):
        if len(msg.data) == 9:
            self.current_angles = list(msg.data)

    def joy_callback(self, msg: Joy):
        if not self.prev_buttons:
            self.prev_buttons = [0] * len(msg.buttons)
        if not self.prev_axes:
            self.prev_axes = [0.0] * len(msg.axes)

        # 十字キー上下（axes[7]）でモーター選択を変更（-1か+1）
        dpad_updown = int(msg.axes[7])
        if dpad_updown != int(self.prev_axes[7]):
            if dpad_updown == 1:  # 上
                self.motor_index = max(0, self.motor_index - 1)
                self.get_logger().info(f"Motor selected: {self.motor_index + 1}")
            elif dpad_updown == -1:  # 下
                self.motor_index = min(8, self.motor_index + 1)
                self.get_logger().info(f"Motor selected: {self.motor_index + 1}")

        # ボタン押下を検出（立ち上がり判定）
        for i, (prev, curr) in enumerate(zip(self.prev_buttons, msg.buttons)):
            if curr == 1 and prev == 0:
                if i == 0:  # Aボタン
                    self.send_offset(+90.0)
                elif i == 1:  # Bボタン
                    self.send_offset(-90.0)

        self.prev_buttons = msg.buttons
        self.prev_axes = msg.axes

    def send_offset(self, offset_deg):
        new_angles = self.current_angles.copy()
        new_angles[self.motor_index] += offset_deg
        msg = Float32MultiArray()
        msg.data = new_angles
        self.publisher.publish(msg)
        self.get_logger().info(
            f"Motor {self.motor_index + 1} → {new_angles[self.motor_index]:.2f} deg")

def main(args=None):
    rclpy.init(args=args)
    node = JoyOffsetCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
