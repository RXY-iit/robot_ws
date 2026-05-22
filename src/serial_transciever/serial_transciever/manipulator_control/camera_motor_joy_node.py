import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32


class CameraMotorJoyNode(Node):
    def __init__(self):
        super().__init__('camera_motor_joy_node')

        self.enable_button = int(self.declare_parameter('enable_button', 4).value)
        self.pan_axis = int(self.declare_parameter('pan_axis', 6).value)
        self.tilt_axis = int(self.declare_parameter('tilt_axis', 7).value)
        self.pan_sign = float(self.declare_parameter('pan_sign', 1.0).value)
        self.tilt_sign = float(self.declare_parameter('tilt_sign', -1.0).value)
        self.pan_step_deg = float(self.declare_parameter('pan_step_deg', 2.5).value)
        self.tilt_step_deg = float(self.declare_parameter('tilt_step_deg', 5.0).value)
        self.deadzone = float(self.declare_parameter('deadzone', 0.2).value)
        self.command_period_sec = float(self.declare_parameter('command_period_sec', 0.03).value)
        self.initial_pan_angle = float(self.declare_parameter('initial_pan_angle', 275.0).value)
        self.initial_tilt_angle = float(self.declare_parameter('initial_tilt_angle', 67.0).value)
        self.initial_command_delay_sec = float(
            self.declare_parameter('initial_command_delay_sec', 1.0).value
        )

        self.current_pan = None
        self.current_tilt = None
        self.target_pan = self.initial_pan_angle if math.isfinite(self.initial_pan_angle) else None
        self.target_tilt = self.initial_tilt_angle if math.isfinite(self.initial_tilt_angle) else None
        self.last_joy = None
        self.initial_sent = False

        self.pan_pub = self.create_publisher(Float32, '/chokudomotor/target_angle', 10)
        self.tilt_pub = self.create_publisher(Float32, '/cameraswingmotor/target_angle', 10)
        self.create_subscription(Float32, '/chokudomotor/angle', self._pan_angle_cb, 10)
        self.create_subscription(Float32, '/cameraswingmotor/angle', self._tilt_angle_cb, 10)
        self.create_subscription(Joy, '/joy', self._joy_cb, 10)

        self.create_timer(self.command_period_sec, self._command_timer_cb)
        self.create_timer(self.initial_command_delay_sec, self._initial_timer_cb)

        self.get_logger().info(
            'Camera motor Joy control ready: hold button '
            f'{self.enable_button}, pan axis {self.pan_axis}, tilt axis {self.tilt_axis}'
        )

    def _pan_angle_cb(self, msg):
        self.current_pan = float(msg.data)

    def _tilt_angle_cb(self, msg):
        self.current_tilt = float(msg.data)

    def _joy_cb(self, msg):
        self.last_joy = msg

    def _initial_timer_cb(self):
        if self.initial_sent:
            return

        sent_any = False
        if math.isfinite(self.initial_pan_angle):
            self.target_pan = self.initial_pan_angle
            self._publish_pan(self.target_pan, 'initial')
            sent_any = True
        if math.isfinite(self.initial_tilt_angle):
            self.target_tilt = self.initial_tilt_angle
            self._publish_tilt(self.target_tilt, 'initial')
            sent_any = True

        self.initial_sent = True
        if sent_any:
            self.get_logger().info('Published camera motor initial target angle(s).')

    def _command_timer_cb(self):
        if self.last_joy is None:
            return
        if not self._button_pressed(self.last_joy, self.enable_button):
            return

        pan_input = self._axis_value(self.last_joy, self.pan_axis)
        tilt_input = self._axis_value(self.last_joy, self.tilt_axis)

        if abs(pan_input) >= self.deadzone:
            if self.target_pan is None:
                self.target_pan = self.current_pan if self.current_pan is not None else 0.0
            self.target_pan += pan_input * self.pan_sign * self.pan_step_deg
            self._publish_pan(self.target_pan, 'joy')

        if abs(tilt_input) >= self.deadzone:
            if self.target_tilt is None:
                self.target_tilt = self.current_tilt if self.current_tilt is not None else 0.0
            self.target_tilt += tilt_input * self.tilt_sign * self.tilt_step_deg
            self._publish_tilt(self.target_tilt, 'joy')

    @staticmethod
    def _axis_value(msg, axis_index):
        if axis_index < 0 or axis_index >= len(msg.axes):
            return 0.0
        return float(msg.axes[axis_index])

    @staticmethod
    def _button_pressed(msg, button_index):
        return 0 <= button_index < len(msg.buttons) and msg.buttons[button_index] == 1

    def _publish_pan(self, target, source):
        self.pan_pub.publish(Float32(data=float(target)))
        self.get_logger().info(f'{source}: pan target -> {target:.2f} deg')

    def _publish_tilt(self, target, source):
        self.tilt_pub.publish(Float32(data=float(target)))
        self.get_logger().info(f'{source}: tilt target -> {target:.2f} deg')


def main(args=None):
    rclpy.init(args=args)
    node = CameraMotorJoyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
