# vacuum_flag_to_relay.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

class FlagRelayBridge(Node):
    def __init__(self):
        super().__init__('flag_relay_bridge')
        self.sub = self.create_subscription(Bool, '/vacuum_flag', self.cb, 10)
        self.pub = self.create_publisher(String, '/relay_switch', 10)
        self.prev_state = None  #連続同値 publish を抑制

    def cb(self, msg: Bool):
        if msg.data != self.prev_state:
            cmd = 'ON' if msg.data else 'OFF'
            self.pub.publish(String(data=cmd))
            self.get_logger().info(f'Relay {cmd} published')
            self.prev_state = msg.data

def main():
    rclpy.init()
    rclpy.spin(FlagRelayBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
