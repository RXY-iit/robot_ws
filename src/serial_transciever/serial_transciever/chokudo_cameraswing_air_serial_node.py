import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class DualMotorSerialNode(Node):
    def __init__(self):
        super().__init__('chokudo_cameraswing_air_serial_node')

        # Publisher（常時配信）
        self.angle1_pub = self.create_publisher(Float32, '/chokudomotor/angle', 10)
        self.angle2_pub = self.create_publisher(Float32, '/cameraswingmotor/angle', 10)
        self.pressure_pub = self.create_publisher(Float32, '/sensor/pressure', 10)

        # 指令値保持
        self.angle1_cmd = 160
        self.angle2_cmd = 67
        self.prev_angle1_cmd = None
        self.prev_angle2_cmd = None

        # ログ出力の制御パラメータ
        p_thresh = self.declare_parameter('angle_change_threshold', 0.1)
        p_flag = self.declare_parameter('log_only_on_change', True)
        self.angle_change_threshold = p_thresh.value
        self.log_only_on_change = p_flag.value

        # 最後にログ出力した角度の保持
        self.last_logged_angles = None

        # 個別にサブスクライブ
        self.sub1 = self.create_subscription(
            Float32,
            '/chokudomotor/target_angle',
            self.chokudo_callback,
            10
        )
        self.sub2 = self.create_subscription(
            Float32,
            '/cameraswingmotor/target_angle',
            self.swing_callback,
            10
        )

        # Serial接続
        try:
            self.ser = serial.Serial(
                '/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_183098125055344E312E3120FF092507-if00',
                115200,
                timeout=1
            )
            self.get_logger().info('Serial connection established.')
        except serial.SerialException:
            self.get_logger().error('Failed to open serial port.')
            self.ser = None

        # 常時read（10Hz）
        self.timer = self.create_timer(0.1, self.read_motor_data)

    def chokudo_callback(self, msg):
        self.angle1_cmd = msg.data
        self.send_angles()

    def swing_callback(self, msg):
        self.angle2_cmd = msg.data
        self.send_angles()

    # def send_angles(self):
    #     if self.ser and self.ser.is_open:
    #         data_str = f"{self.angle1_cmd:.2f},{self.angle2_cmd:.2f}\n"
    #         self.ser.write(data_str.encode('utf-8'))
    #         self.get_logger().info(f'Sent: {data_str.strip()}')
    #     else:
    #         self.get_logger().warn('Serial port not open.')
    def send_angles(self):
        if self.ser and self.ser.is_open:
            # 同じ値を連続送信しないように判定
            if (self.prev_angle1_cmd == self.angle1_cmd and
                self.prev_angle2_cmd == self.angle2_cmd):
                return  # 指令が変わっていなければ送らない

            # 送信処理
            data_str = f"{self.angle1_cmd:.2f},{self.angle2_cmd:.2f}\n"
            self.ser.write(data_str.encode('utf-8'))
            self.get_logger().info(f'Sent: {data_str.strip()}')

            # 更新
            self.prev_angle1_cmd = self.angle1_cmd
            self.prev_angle2_cmd = self.angle2_cmd
        else:
            self.get_logger().warn('Serial port not open.')

    def read_motor_data(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b'read\n')
                line = self.ser.readline().decode('utf-8').strip()

                if line:
                    parts = line.split(',')
                    if len(parts) == 3:
                        angle1 = float(parts[0])
                        angle2 = float(parts[1])
                        pressure = float(parts[2])

                        msg1 = Float32(); msg1.data = angle1
                        msg2 = Float32(); msg2.data = angle2
                        msg3 = Float32(); msg3.data = pressure
                        self.angle1_pub.publish(msg1)
                        self.angle2_pub.publish(msg2)
                        self.pressure_pub.publish(msg3)
                        # ログは初回または閾値以上変化したときのみ出力
                        try:
                            if self.last_logged_angles is None:
                                self.get_logger().info(
                                    f"Recv -> angle1: {angle1:.2f}, angle2: {angle2:.2f}, pressure: {pressure:.2f}"
                                )
                                self.last_logged_angles = [angle1, angle2]
                            else:
                                diffs = [abs(angle1 - self.last_logged_angles[0]), abs(angle2 - self.last_logged_angles[1])]
                                if (not self.log_only_on_change) or any(d > self.angle_change_threshold for d in diffs):
                                    self.get_logger().info(
                                        f"Recv -> angle1: {angle1:.2f}, angle2: {angle2:.2f}, pressure: {pressure:.2f}"
                                    )
                                    self.last_logged_angles = [angle1, angle2]
                        except Exception:
                            # 比較に失敗したら安全側でログ出力して記録を更新
                            self.get_logger().info(
                                f"Recv -> angle1: {angle1:.2f}, angle2: {angle2:.2f}, pressure: {pressure:.2f}"
                            )
                            self.last_logged_angles = [angle1, angle2]
                    else:
                        self.get_logger().warn(f'Invalid response: {line}')
            except Exception as e:
                self.get_logger().error(f'Serial read error: {e}')
        else:
            self.get_logger().warn('Serial port not open.')

def main(args=None):
    rclpy.init(args=args)
    node = DualMotorSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down')
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

