# relay_controller.py
import serial, rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RelayController(Node):
    def __init__(self):
        super().__init__('relay_controller')
        port = '/dev/serial/by-id/usb-Microchip_Technology_Inc._USB-RELAY1_X-RL2-if00'
        self.ser = serial.Serial(port, 9600, timeout=1)
        self.sub = self.create_subscription(String, '/relay_switch', self.cb, 10)
        self.cmd = {'ON': b'A1B1', 'OFF': b'A0B0'}

    def cb(self, msg: String):
        key = msg.data.upper()
        if key in self.cmd:
            self.ser.write(self.cmd[key])
            self.get_logger().info(f'Relay {key} sent')
        else:
            self.get_logger().warn('Unknown command (use ON/OFF)')

def main():
    rclpy.init()
    rclpy.spin(RelayController())
    rclpy.shutdown()
