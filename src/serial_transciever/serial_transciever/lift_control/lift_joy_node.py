import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String


class LiftJoyNode(Node):
    """Maps Joy-Con buttons to lift commands.

    Default mapping (verify with: ros2 topic echo /joy):
      enable_button (4 = L1) + up_button   (3 = Y) → UP
      enable_button (4 = L1) + down_button (0 = A) → DOWN
      any other combination                         → STOP

    Pressing both up and down simultaneously is treated as STOP.
    Commands are only published when the state changes to reduce topic noise.

    Parameters
    ----------
    enable_button : int   Dead-man switch button index (default 4 = L1).
    up_button     : int   Button index for UP   (default 3 = Y).
    down_button   : int   Button index for DOWN (default 0 = A).
    """

    def __init__(self):
        super().__init__('lift_joy_node')

        self.enable_button = int(self.declare_parameter('enable_button', 4).value)
        self.up_button = int(self.declare_parameter('up_button', 3).value)
        self.down_button = int(self.declare_parameter('down_button', 0).value)

        self.cmd_pub = self.create_publisher(String, '/lift/command', 10)
        self.create_subscription(Joy, '/joy', self._joy_cb, 10)

        self._last_cmd: str | None = None

        self.get_logger().info(
            f'Lift joy node ready  '
            f'enable={self.enable_button}  '
            f'UP(Y)={self.up_button}  '
            f'DOWN(A)={self.down_button}'
        )
        self.get_logger().info(
            'Hold L1+Y to move UP, L1+A to move DOWN, release to STOP'
        )

    def _joy_cb(self, msg: Joy):
        enable = self._btn(msg, self.enable_button)
        up = self._btn(msg, self.up_button)
        down = self._btn(msg, self.down_button)

        if enable and up and not down:
            cmd = 'UP'
        elif enable and down and not up:
            cmd = 'DOWN'
        else:
            cmd = 'STOP'

        # Publish only on state change to keep topic traffic minimal
        if cmd != self._last_cmd:
            self.cmd_pub.publish(String(data=cmd))
            self._last_cmd = cmd
            self.get_logger().info(f'[lift_joy] → {cmd}')

    @staticmethod
    def _btn(msg: Joy, idx: int) -> bool:
        return 0 <= idx < len(msg.buttons) and msg.buttons[idx] == 1


def main(args=None):
    rclpy.init(args=args)
    node = LiftJoyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
