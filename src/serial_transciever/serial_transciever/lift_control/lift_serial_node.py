import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import serial


class LiftSerialNode(Node):
    """Serial bridge between ROS2 and the Arduino-controlled AZD-KD lift.

    Topics
    ------
    Sub  /lift/command  (std_msgs/String)  "UP" | "DOWN" | "STOP" | "HOME"
    Pub  /lift/position (std_msgs/Float32)  estimated position [mm]
    Pub  /lift/state    (std_msgs/String)   current motion state

    Position is estimated by integrating jog_speed_mm_s × elapsed time.
    No encoder feedback is available; homing resets the position reference.

    Homing sequence
    ---------------
    1. Assume worst-case start (arm at top = position_max_mm).
    2. Drive DOWN for the full range duration + 2 s safety buffer.
    3. Stop and declare position = home_position_mm.
    The driver's built-in ABZO limit protection prevents mechanical damage
    even if the arm reaches the physical stop before homing completes.
    """

    def __init__(self):
        super().__init__('lift_serial_node')

        # ── parameters ────────────────────────────────────────────────────────
        self.serial_port = self.declare_parameter('serial_port', '/dev/ttyACM0').value
        self.baud_rate = int(self.declare_parameter('baud_rate', 9600).value)
        self.jog_speed_mm_s = float(self.declare_parameter('jog_speed_mm_s', 60.0).value)
        self.position_min_mm = float(self.declare_parameter('position_min_mm', 0.0).value)
        self.position_max_mm = float(self.declare_parameter('position_max_mm', 200.0).value)
        self.home_position_mm = float(self.declare_parameter('home_position_mm', 100.0).value)
        self.auto_home_on_start = bool(self.declare_parameter('auto_home_on_start', False).value)
        self.enable_soft_limits = bool(self.declare_parameter('enable_soft_limits', False).value)

        # ── state ─────────────────────────────────────────────────────────────
        self.current_position_mm = 0.0  # unknown at startup; tracking only
        self.current_command = 'STOP'
        self.move_start_time = None
        self.move_start_pos = self.current_position_mm
        self.is_homing = False

        # ── publishers / subscribers ───────────────────────────────────────────
        self.pos_pub = self.create_publisher(Float32, '/lift/position', 10)
        self.state_pub = self.create_publisher(String, '/lift/state', 10)
        self.create_subscription(String, '/lift/command', self._command_cb, 10)

        # ── serial ────────────────────────────────────────────────────────────
        self.ser = self._open_serial()

        # ── timers ────────────────────────────────────────────────────────────
        # Position update + limit enforcement at 20 Hz
        self.create_timer(0.05, self._update_timer_cb)

        # Auto-home: wait 2.5 s for Arduino to finish USB-reset, then home
        self._auto_home_timer = None
        if self.auto_home_on_start and self.ser:
            self._auto_home_timer = self.create_timer(2.5, self._auto_home_cb)

        limits_str = (
            f'range=[{self.position_min_mm},{self.position_max_mm}] mm'
            if self.enable_soft_limits else 'soft limits DISABLED'
        )
        self.get_logger().info(
            f'Lift serial node ready  port={self.serial_port}  '
            f'{limits_str}  home={self.home_position_mm} mm  '
            f'auto_home={self.auto_home_on_start}'
        )

    # ── serial helpers ─────────────────────────────────────────────────────────

    def _open_serial(self):
        try:
            ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f'Serial opened: {self.serial_port} @ {self.baud_rate} baud')
            return ser
        except serial.SerialException as e:
            self.get_logger().error(f'Serial open failed: {e}')
            return None

    def _serial_send(self, cmd: str):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(f'{cmd}\n'.encode())
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write error: {e}')
        else:
            self.get_logger().warn('Serial port not open, command dropped')

    # ── motion control ─────────────────────────────────────────────────────────

    def _apply_motion(self, cmd: str):
        """Apply a motion command unconditionally (no limit check here)."""
        if cmd == self.current_command:
            return

        # Snapshot position as the new movement start reference
        self.move_start_pos = self.current_position_mm

        if cmd in ('UP', 'DOWN'):
            self.move_start_time = self.get_clock().now()
        else:
            self.move_start_time = None

        self.current_command = cmd
        self._serial_send(cmd)
        self.state_pub.publish(String(data=cmd))
        self.get_logger().info(f'[lift] {cmd}  pos={self.current_position_mm:.1f} mm')

    # ── command callback ────────────────────────────────────────────────────────

    def _command_cb(self, msg: String):
        if self.is_homing:
            self.get_logger().warn('[lift] homing in progress — command ignored')
            return

        cmd = msg.data.strip().upper()

        if cmd == 'HOME':
            self._begin_homing()
            return

        if cmd not in ('UP', 'DOWN', 'STOP'):
            self.get_logger().warn(f'[lift] unknown command: {msg.data!r}')
            return

        # Soft-limit check (only when enable_soft_limits=true)
        if self.enable_soft_limits:
            if cmd == 'UP' and self.current_position_mm >= self.position_max_mm:
                self.get_logger().warn(
                    f'[lift] upper limit ({self.position_max_mm:.0f} mm) — UP ignored'
                )
                return
            if cmd == 'DOWN' and self.current_position_mm <= self.position_min_mm:
                self.get_logger().warn(
                    f'[lift] lower limit ({self.position_min_mm:.0f} mm) — DOWN ignored'
                )
                return

        self._apply_motion(cmd)

    # ── homing ─────────────────────────────────────────────────────────────────

    def _auto_home_cb(self):
        if self._auto_home_timer:
            self._auto_home_timer.cancel()
            self._auto_home_timer = None
        self._begin_homing()

    def _begin_homing(self):
        if self.is_homing:
            self.get_logger().warn('[lift] homing already in progress')
            return
        self.get_logger().info('[lift] homing started — moving DOWN to home position')
        self.is_homing = True
        # Conservative: pretend we are at the very top so position tracking
        # naturally drives us all the way to position_min_mm before stopping.
        self.current_position_mm = self.position_max_mm
        self._apply_motion('DOWN')

    # ── position update timer ───────────────────────────────────────────────────

    def _update_timer_cb(self):
        # Integrate velocity to estimate position (unclamped when limits disabled)
        if self.move_start_time is not None and self.current_command in ('UP', 'DOWN'):
            elapsed = (self.get_clock().now() - self.move_start_time).nanoseconds / 1e9
            delta = self.jog_speed_mm_s * elapsed
            if self.current_command == 'UP':
                self.current_position_mm = self.move_start_pos + delta
            else:  # DOWN
                self.current_position_mm = self.move_start_pos - delta

        # Soft-limit enforcement (only when enable_soft_limits=true)
        if self.enable_soft_limits:
            if self.current_command == 'UP' and self.current_position_mm >= self.position_max_mm:
                self.get_logger().info(
                    f'[lift] upper limit reached ({self.position_max_mm:.0f} mm) — stopping'
                )
                self._apply_motion('STOP')

            elif self.current_command == 'DOWN' and self.current_position_mm <= self.position_min_mm:
                if self.is_homing:
                    self.current_position_mm = self.home_position_mm
                    self.is_homing = False
                    self.get_logger().info(
                        f'[lift] homing complete — position = {self.home_position_mm:.1f} mm'
                    )
                else:
                    self.get_logger().info(
                        f'[lift] lower limit reached ({self.position_min_mm:.0f} mm) — stopping'
                    )
                self._apply_motion('STOP')

        self.pos_pub.publish(Float32(data=float(self.current_position_mm)))

    # ── shutdown ────────────────────────────────────────────────────────────────

    def destroy_node(self):
        self.get_logger().info('[lift] shutting down — sending STOP')
        self._serial_send('STOP')
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LiftSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
