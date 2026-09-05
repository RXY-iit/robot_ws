import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import serial
import json
from pathlib import Path
import time
from datetime import datetime, timezone


DEFAULT_LIFT_ARDUINO_PORT = (
    '/dev/serial/by-id/'
    'usb-Arduino__www.arduino.cc__0043_03536383236351E062C1-if00'
)
LIFT_PROTOCOL = 'LIFT_CONTROL_V2'


def usb_identity_for_port(port: str) -> dict:
    """Return the Linux USB identity behind a serial device path."""
    result = {
        'configured_path': port,
        'resolved_path': None,
        'vendor_id': None,
        'product_id': None,
        'manufacturer': None,
        'product': None,
        'serial': None,
    }
    try:
        resolved = Path(port).resolve(strict=True)
    except (FileNotFoundError, OSError):
        return result
    result['resolved_path'] = str(resolved)
    device = Path('/sys/class/tty') / resolved.name / 'device'
    try:
        cursor = device.resolve(strict=True)
    except (FileNotFoundError, OSError):
        return result
    for parent in (cursor, *cursor.parents):
        vendor_file = parent / 'idVendor'
        product_file = parent / 'idProduct'
        if not vendor_file.exists() or not product_file.exists():
            continue
        for key, filename in (
            ('vendor_id', 'idVendor'),
            ('product_id', 'idProduct'),
            ('manufacturer', 'manufacturer'),
            ('product', 'product'),
            ('serial', 'serial'),
        ):
            try:
                result[key] = (parent / filename).read_text().strip()
            except (FileNotFoundError, OSError):
                pass
        break
    return result


def usb_identity_matches(identity: dict, vendor_id: str, product_id: str) -> bool:
    """Return true only when both expected USB IDs match."""
    return (
        str(identity.get('vendor_id') or '').lower() == vendor_id.lower()
        and str(identity.get('product_id') or '').lower() == product_id.lower()
    )


def parse_lift_reply(line: str) -> tuple[str, str]:
    """Parse a firmware reply into (kind, value)."""
    cleaned = line.strip()
    if cleaned.startswith('ACK '):
        return 'ACK', cleaned[4:].strip().upper()
    if cleaned.startswith('READY '):
        return 'READY', cleaned[6:].strip()
    if cleaned.startswith('IDENTITY '):
        return 'IDENTITY', cleaned[9:].strip()
    if cleaned.startswith('ERR '):
        return 'ERROR', cleaned[4:].strip()
    return 'UNKNOWN', cleaned


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
        self.serial_port = self.declare_parameter(
            'serial_port', DEFAULT_LIFT_ARDUINO_PORT
        ).value
        self.baud_rate = int(self.declare_parameter('baud_rate', 9600).value)
        self.expected_usb_vendor_id = str(
            self.declare_parameter('expected_usb_vendor_id', '2341').value
        )
        self.expected_usb_product_id = str(
            self.declare_parameter('expected_usb_product_id', '0043').value
        )
        self.require_usb_identity = bool(
            self.declare_parameter('require_usb_identity', True).value
        )
        self.ack_timeout_sec = float(
            self.declare_parameter('ack_timeout_sec', 1.0).value
        )
        self.jog_speed_mm_s = float(self.declare_parameter('jog_speed_mm_s', 60.0).value)
        self.position_min_mm = float(self.declare_parameter('position_min_mm', 0.0).value)
        self.position_max_mm = float(self.declare_parameter('position_max_mm', 200.0).value)
        self.home_position_mm = float(self.declare_parameter('home_position_mm', 100.0).value)
        self.initial_position_mm = float(
            self.declare_parameter('initial_position_mm', self.position_min_mm).value
        )
        self.initial_position_reference = str(
            self.declare_parameter(
                'initial_position_reference', 'UNREFERENCED_SOFTWARE_ESTIMATE'
            ).value
        )
        self.auto_home_on_start = bool(self.declare_parameter('auto_home_on_start', False).value)
        self.enable_soft_limits = bool(self.declare_parameter('enable_soft_limits', False).value)

        # ── state ─────────────────────────────────────────────────────────────
        # The position is only a software integration estimate. Do not clamp it
        # when manual soft limits are disabled: LT/RT remain supervised,
        # unrestricted hold-to-run controls until real ABZO/limit feedback is
        # integrated. Keep the optional bounds for the future instrumented mode.
        if self.enable_soft_limits:
            self.current_position_mm = min(
                max(self.initial_position_mm, self.position_min_mm), self.position_max_mm
            )
        else:
            self.current_position_mm = self.initial_position_mm
        self.current_command = 'STOP'
        self.move_start_time = None
        self.move_start_pos = self.current_position_mm
        self.is_homing = False
        self.serial_write_count = 0
        self.serial_write_error_count = 0
        self.last_serial_error = None
        self.position_reference = self.initial_position_reference
        self.usb_identity = usb_identity_for_port(self.serial_port)
        self.usb_identity_verified = False
        self.protocol_ready = False
        self.protocol_identity = None
        self.ack_supported = False
        self.ack_count = 0
        self.ack_timeout_count = 0
        self.consecutive_ack_timeouts = 0
        self.last_ack_timeout_monotonic = None
        self.last_ack_command = None
        self.last_ack_monotonic = None
        self.last_protocol_error = None
        self.pending_acks = []
        self._rx_buffer = bytearray()
        self._identify_due = None

        # ── publishers / subscribers ───────────────────────────────────────────
        self.pos_pub = self.create_publisher(Float32, '/lift/position', 10)
        self.state_pub = self.create_publisher(String, '/lift/state', 10)
        self.health_pub = self.create_publisher(String, '/lift/health_summary', 10)
        self.create_subscription(String, '/lift/command', self._command_cb, 10)

        # ── serial ────────────────────────────────────────────────────────────
        self.ser = self._open_serial()

        # ── timers ────────────────────────────────────────────────────────────
        # Position update + limit enforcement at 20 Hz
        self.create_timer(0.05, self._update_timer_cb)
        self.create_timer(0.05, self._serial_poll_cb)
        self.create_timer(1.0, self._connection_monitor_cb)
        self.create_timer(1.0, self._publish_health_summary)

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
            f'auto_home={self.auto_home_on_start}  '
            f'initial={self.current_position_mm:.1f} mm (software estimate)'
        )

    # ── serial helpers ─────────────────────────────────────────────────────────

    def _open_serial(self):
        self.usb_identity = usb_identity_for_port(self.serial_port)
        self.usb_identity_verified = usb_identity_matches(
            self.usb_identity,
            self.expected_usb_vendor_id,
            self.expected_usb_product_id,
        )
        if self.require_usb_identity and not self.usb_identity_verified:
            actual = (
                f"{self.usb_identity.get('vendor_id') or 'missing'}:"
                f"{self.usb_identity.get('product_id') or 'missing'}"
            )
            expected = f'{self.expected_usb_vendor_id}:{self.expected_usb_product_id}'
            self.serial_write_error_count += 1
            self.last_serial_error = (
                f'Lift Arduino USB identity mismatch/missing: expected {expected}, '
                f'observed {actual} at {self.serial_port}'
            )
            self.get_logger().error(self.last_serial_error)
            return None
        try:
            ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0)
            self.get_logger().info(f'Serial opened: {self.serial_port} @ {self.baud_rate} baud')
            self.last_serial_error = None
            self.protocol_ready = False
            self.protocol_identity = None
            self.ack_supported = False
            self.pending_acks.clear()
            self._rx_buffer.clear()
            # Opening an UNO resets it. Wait for its bootloader before IDENTIFY.
            self._identify_due = time.monotonic() + 2.5
            return ser
        except serial.SerialException as e:
            self.serial_write_error_count += 1
            self.last_serial_error = f'Serial open failed: {e}'
            self.get_logger().error(f'Serial open failed: {e}')
            return None

    def _serial_send(self, cmd: str):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(f'{cmd}\n'.encode())
                self.ser.flush()
                self.serial_write_count += 1
                self.pending_acks.append((cmd, time.monotonic()))
            except (OSError, serial.SerialException) as e:
                self.serial_write_error_count += 1
                self.last_serial_error = str(e)
                self.get_logger().error(f'Serial write error: {e}')
        else:
            self.serial_write_error_count += 1
            self.last_serial_error = 'Serial port is not open'
            self.get_logger().warn('Serial port not open, command dropped')

    def _serial_poll_cb(self):
        if not self.ser or not self.ser.is_open:
            return
        try:
            waiting = self.ser.in_waiting
            if waiting:
                self._rx_buffer.extend(self.ser.read(waiting))
            while b'\n' in self._rx_buffer:
                raw, _, remainder = self._rx_buffer.partition(b'\n')
                self._rx_buffer = bytearray(remainder)
                kind, value = parse_lift_reply(raw.decode(errors='replace'))
                if kind == 'READY':
                    self.protocol_ready = value == LIFT_PROTOCOL
                    self.protocol_identity = value
                    self.ack_supported = value == LIFT_PROTOCOL
                elif kind == 'IDENTITY':
                    self.protocol_identity = value
                    self.protocol_ready = value == LIFT_PROTOCOL
                    self.ack_supported = value == LIFT_PROTOCOL
                    self._remove_pending_ack('IDENTIFY')
                elif kind == 'ACK':
                    self.ack_supported = True
                    self.ack_count += 1
                    self.consecutive_ack_timeouts = 0
                    self.last_protocol_error = None
                    self.last_ack_command = value
                    self.last_ack_monotonic = time.monotonic()
                    self._remove_pending_ack(value)
                elif kind == 'ERROR':
                    self.last_protocol_error = value
                    self.get_logger().error(f'Lift Arduino protocol error: {value}')
        except (OSError, serial.SerialException) as exc:
            self.last_serial_error = f'Serial read failed: {exc}'
            self.serial_write_error_count += 1
            self.get_logger().error(self.last_serial_error)
            self._close_serial()

        now = time.monotonic()
        if self._identify_due is not None and now >= self._identify_due:
            self._identify_due = None
            self._serial_send('IDENTIFY')
        kept = []
        for command, sent_at in self.pending_acks:
            if now - sent_at > self.ack_timeout_sec:
                self.ack_timeout_count += 1
                self.consecutive_ack_timeouts += 1
                self.last_ack_timeout_monotonic = now
                self.last_protocol_error = f'No ACK for {command}'
                self.get_logger().warn(self.last_protocol_error)
            else:
                kept.append((command, sent_at))
        self.pending_acks = kept

    def _remove_pending_ack(self, command: str):
        for index, pending in enumerate(self.pending_acks):
            if pending[0] == command:
                del self.pending_acks[index]
                return

    def _close_serial(self):
        if self.ser:
            try:
                if self.ser.is_open:
                    self.ser.close()
            except (OSError, serial.SerialException):
                pass
        self.ser = None

    def _connection_monitor_cb(self):
        identity = usb_identity_for_port(self.serial_port)
        identity_ok = usb_identity_matches(
            identity, self.expected_usb_vendor_id, self.expected_usb_product_id
        )
        self.usb_identity = identity
        self.usb_identity_verified = identity_ok
        if self.ser and self.ser.is_open and self.require_usb_identity and not identity_ok:
            self.last_serial_error = 'Configured Lift Arduino device disappeared or changed identity'
            self.get_logger().error(self.last_serial_error)
            self._close_serial()
        if not self.ser:
            self.ser = self._open_serial()

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

        if cmd in ('SET_MIN', 'SET_MAX'):
            if self.current_command != 'STOP':
                self.get_logger().warn(f'[lift] {cmd} ignored while motion is commanded')
                return
            if cmd == 'SET_MIN':
                self.current_position_mm = self.position_min_mm
                self.position_reference = 'OPERATOR_CONFIRMED_LOWER_LIMIT'
            else:
                self.current_position_mm = self.position_max_mm
                self.position_reference = 'OPERATOR_CONFIRMED_UPPER_LIMIT'
            self.move_start_pos = self.current_position_mm
            self.pos_pub.publish(Float32(data=float(self.current_position_mm)))
            self.state_pub.publish(String(data=cmd))
            self.get_logger().info(
                f'[lift] position reference set by operator: '
                f'{self.current_position_mm:.1f} mm ({cmd})'
            )
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
                    self.position_reference = 'TIMED_HOME_ESTIMATE'
                    self.get_logger().info(
                        f'[lift] homing complete — position = {self.home_position_mm:.1f} mm'
                    )
                else:
                    self.get_logger().info(
                        f'[lift] lower limit reached ({self.position_min_mm:.0f} mm) — stopping'
                    )
                self._apply_motion('STOP')

        self.pos_pub.publish(Float32(data=float(self.current_position_mm)))

    def _publish_health_summary(self):
        serial_open = bool(self.ser and self.ser.is_open)
        now = time.monotonic()
        value = {
            'schema_version': 1,
            'generated_at': datetime.now(timezone.utc).isoformat(),
            'source': 'lift_serial_node runtime state; position is not encoder feedback',
            'publish_rate_hz': 1.0,
            'serial': {
                'port': self.serial_port,
                'baud_rate': self.baud_rate,
                'open': serial_open,
                'write_count': self.serial_write_count,
                'write_error_count': self.serial_write_error_count,
                'last_error': self.last_serial_error,
                'usb_identity': self.usb_identity,
                'usb_identity_verified': self.usb_identity_verified,
                'expected_usb_vendor_id': self.expected_usb_vendor_id,
                'expected_usb_product_id': self.expected_usb_product_id,
            },
            'protocol': {
                'expected': LIFT_PROTOCOL,
                'ready': self.protocol_ready,
                'identity': self.protocol_identity,
                'ack_supported': self.ack_supported,
                'ack_count': self.ack_count,
                'ack_timeout_count': self.ack_timeout_count,
                'consecutive_ack_timeouts': self.consecutive_ack_timeouts,
                'last_ack_timeout_age_sec': (
                    now - self.last_ack_timeout_monotonic
                    if self.last_ack_timeout_monotonic is not None else None
                ),
                'pending_ack_count': len(self.pending_acks),
                'last_ack_command': self.last_ack_command,
                'last_ack_age_sec': (
                    now - self.last_ack_monotonic
                    if self.last_ack_monotonic is not None else None
                ),
                'last_error': self.last_protocol_error,
            },
            'motion': {
                'command': self.current_command,
                'is_homing': self.is_homing,
                'estimated_position_mm': float(self.current_position_mm),
                'position_reference': self.position_reference,
                'position_evidence': 'SOFTWARE_TIME_INTEGRATION',
            },
            'configuration': {
                'jog_speed_mm_s': self.jog_speed_mm_s,
                'position_min_mm': self.position_min_mm,
                'position_max_mm': self.position_max_mm,
                'home_position_mm': self.home_position_mm,
                'initial_position_mm': self.initial_position_mm,
                'initial_position_reference': self.initial_position_reference,
                'auto_home_on_start': self.auto_home_on_start,
                'soft_limits_enabled': self.enable_soft_limits,
            },
            'hardware_feedback': {
                'abzo_position_available': False,
                'upper_limit_available': False,
                'lower_limit_available': False,
                'home_complete_available': False,
                'alarm_ready_available': False,
            },
        }
        self.health_pub.publish(String(data=json.dumps(value, ensure_ascii=False)))

    # ── shutdown ────────────────────────────────────────────────────────────────

    def destroy_node(self):
        self.get_logger().info('[lift] shutting down — sending STOP')
        self._serial_send('STOP')
        self._close_serial()
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
