import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String


def lift_command_from_axes(
    axes, up_axis: int = 2, down_axis: int = 5, active_below: float = -0.5
) -> str:
    """Return a mutually exclusive hold-to-run command from LT/RT axes."""
    up = 0 <= up_axis < len(axes) and float(axes[up_axis]) < active_below
    down = 0 <= down_axis < len(axes) and float(axes[down_axis]) < active_below
    if up and not down:
        return 'UP'
    if down and not up:
        return 'DOWN'
    return 'STOP'


class LiftJoyNode(Node):
    """Maps the Joy controller triggers to lift commands.

    Default mapping (verify with: ros2 topic echo /joy):
      LT axis (2) pressed below -0.5 → UP
      RT axis (5) pressed below -0.5 → DOWN
      neither or both pressed        → STOP

    Pressing both up and down simultaneously is treated as STOP.
    Each trigger is itself a hold-to-run control; L1 is not required. Commands
    are only published when the state changes to reduce topic noise. A Joy
    receive watchdog sends STOP if the selected Joy stream disappears while
    the lift is moving.

    Parameters
    ----------
    up_axis             : int    LT axis index (default 2).
    down_axis           : int    RT axis index (default 5).
    trigger_active_below: float  Trigger is pressed below this value (-0.5).
    joy_timeout_sec     : float  STOP after this long without /joy (0.30 s).
    """

    def __init__(self):
        super().__init__('lift_joy_node')

        self.up_axis = int(self.declare_parameter('up_axis', 2).value)
        self.down_axis = int(self.declare_parameter('down_axis', 5).value)
        self.trigger_active_below = float(
            self.declare_parameter('trigger_active_below', -0.5).value
        )
        self.joy_timeout_sec = float(
            self.declare_parameter('joy_timeout_sec', 0.30).value
        )

        self.cmd_pub = self.create_publisher(String, '/lift/command', 10)
        self.create_subscription(Joy, '/joy', self._joy_cb, 10)

        self._last_cmd: str | None = None
        self._last_joy_time: float | None = None
        self.create_timer(0.05, self._watchdog_cb)

        self.get_logger().info(
            f'Lift joy node ready  '
            f'UP(LT axis)={self.up_axis}  '
            f'DOWN(RT axis)={self.down_axis}  '
            f'pressed<{self.trigger_active_below:.2f}  '
            f'Joy timeout={self.joy_timeout_sec:.2f}s'
        )
        self.get_logger().info('Hold LT to move UP, RT to move DOWN; release to STOP')

    def _joy_cb(self, msg: Joy):
        self._last_joy_time = time.monotonic()
        cmd = lift_command_from_axes(
            msg.axes,
            self.up_axis,
            self.down_axis,
            self.trigger_active_below,
        )
        self._publish_if_changed(cmd)

    def _watchdog_cb(self):
        if self._last_cmd not in ('UP', 'DOWN') or self._last_joy_time is None:
            return
        if time.monotonic() - self._last_joy_time > self.joy_timeout_sec:
            self.get_logger().warn('[lift_joy] Joy input timed out; sending STOP')
            self._publish_if_changed('STOP')

    def _publish_if_changed(self, cmd: str):
        # Publish only on state change to keep topic traffic minimal.
        if cmd != self._last_cmd:
            self.cmd_pub.publish(String(data=cmd))
            self._last_cmd = cmd
            self.get_logger().info(f'[lift_joy] → {cmd}')

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
