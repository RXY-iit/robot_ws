import threading
import unittest
from types import SimpleNamespace

from so101_arm_control.read_only_probe import MOTOR_NAMES
from so101_arm_control.real_trajectory_server import (
    COMMAND_CONTROL_MODE,
    RealTrajectoryServer,
    temperature_sample_update,
)


class _Logger:
    def warning(self, _message):
        pass

    def error(self, _message):
        pass


class _FakeBus:
    def __init__(self):
        self.is_connected = True
        self.active_motors = set()
        self.positions = {name: 1000 + index for index, name in enumerate(MOTOR_NAMES)}
        self.goals = dict(self.positions)
        self.currents = {name: 0 for name in MOTOR_NAMES}
        self.temperatures = {name: 35 for name in MOTOR_NAMES}
        self.direct_temperatures = None
        self.torque = {name: 0 for name in MOTOR_NAMES}

    def sync_read(self, data_name, **_kwargs):
        return dict(
            {
                "Present_Position": self.positions,
                "Goal_Position": self.goals,
                "Present_Current": self.currents,
                "Present_Temperature": self.temperatures,
                "Torque_Enable": self.torque,
            }[data_name]
        )

    def sync_write(self, data_name, values, **_kwargs):
        if data_name != "Goal_Position" or set(values) != set(MOTOR_NAMES):
            raise AssertionError("unexpected fake-bus write")
        self.goals = dict(values)

    def read(self, data_name, motor_name, **_kwargs):
        if data_name != "Present_Temperature":
            raise AssertionError("unexpected fake-bus read")
        source = self.direct_temperatures or self.temperatures
        return source[motor_name]

    def enable_torque(self, motor_name, **_kwargs):
        self.torque[motor_name] = 1

    def disable_torque(self, motor_name, **_kwargs):
        self.torque[motor_name] = 0


def _server(bus=None):
    server = object.__new__(RealTrajectoryServer)
    server._state_lock = threading.Lock()
    server._bus_lock = threading.Lock()
    server._bus = bus or _FakeBus()
    server._active = False
    server._active_motors = set()
    server._holding = False
    server._hold_state = "RELEASED"
    server._hold_reason = "test"
    server._hold_targets_raw = {}
    server._hold_started_at = None
    server._hold_current_counts = {name: 0 for name in MOTOR_NAMES}
    server._hold_position_counts = {name: 0 for name in MOTOR_NAMES}
    server._control_mode = COMMAND_CONTROL_MODE
    server._positions = [0.0] * len(MOTOR_NAMES)
    server._desired = list(server._positions)
    server._max_temperature = 55
    server._temperature_consecutive_samples = 5
    server._temperature_confirmation_reads = 3
    server._temperature_confirmation_min_hot_reads = 2
    server._temperature_rejected_spike_count = 0
    server._last_temperature_rejection = None
    server._hard_current = 300
    server._teleop_entry_leader_norm = {}
    server._teleop_entry_follower_raw = {}
    server._teleop_last_target_raw = {}
    server._teleop_last_command_at = None
    server._teleop_clamped = {}
    server.get_logger = lambda: _Logger()
    return server


class CodeHoldInvariantTest(unittest.TestCase):
    def test_temperature_requires_consecutive_samples(self):
        count = 0
        for _ in range(4):
            ready, count = temperature_sample_update(
                temperature_c=60,
                max_temperature_c=55,
                consecutive_count=count,
                samples_required=5,
            )
            self.assertFalse(ready)
        ready, count = temperature_sample_update(
            temperature_c=60,
            max_temperature_c=55,
            consecutive_count=count,
            samples_required=5,
        )
        self.assertTrue(ready)
        ready, count = temperature_sample_update(
            temperature_c=35,
            max_temperature_c=55,
            consecutive_count=count,
            samples_required=5,
        )
        self.assertFalse(ready)
        self.assertEqual(count, 0)

    def test_code_mode_engages_and_manual_release_is_the_only_normal_off_path(self):
        server = _server()

        self.assertTrue(
            server._engage_code_hold(
                "code2real selected; maintaining current six-joint pose"
            )
        )
        self.assertTrue(server._holding)
        self.assertEqual(server._hold_state, "HOLDING")
        self.assertEqual(set(server._active_motors), set(MOTOR_NAMES))
        self.assertTrue(all(value == 1 for value in server._bus.torque.values()))
        self.assertEqual(server._hold_targets_raw, server._bus.positions)

        server._bus.positions["wrist_roll"] += 20
        held = server._continue_hold_at_current_pose("cancelled; hold current pose")
        self.assertEqual(held["wrist_roll"], 1024)
        self.assertTrue(all(value == 1 for value in server._bus.torque.values()))

        self.assertEqual(
            server._release_hold("operator requested", manual=True), []
        )
        self.assertFalse(server._holding)
        self.assertEqual(server._hold_state, "MANUAL_RELEASED")
        self.assertTrue(all(value == 0 for value in server._bus.torque.values()))

    def test_hot_motor_blocks_code_hold_and_latches_fault(self):
        bus = _FakeBus()
        bus.temperatures["elbow_flex"] = 56
        server = _server(bus)

        self.assertFalse(server._engage_code_hold("test"))
        self.assertEqual(server._hold_state, "FAULT")
        self.assertFalse(server._holding)
        self.assertTrue(all(value == 0 for value in bus.torque.values()))

    def test_unconfirmed_sync_temperature_spike_does_not_block_code_hold(self):
        bus = _FakeBus()
        bus.temperatures["elbow_flex"] = 80
        bus.direct_temperatures = dict(bus.temperatures)
        bus.direct_temperatures["elbow_flex"] = 35
        server = _server(bus)

        self.assertTrue(server._engage_code_hold("test"))
        self.assertEqual(server._hold_state, "HOLDING")
        self.assertEqual(server._temperature_rejected_spike_count, 1)
        self.assertEqual(
            server._last_temperature_rejection["confirmation_reads_c"],
            [35, 35, 35],
        )

    def test_mode_sync_does_not_override_manual_release_or_clear_fault(self):
        server = _server()
        server._hold_state = "MANUAL_RELEASED"
        server._on_control_mode(SimpleNamespace(data=COMMAND_CONTROL_MODE))
        self.assertEqual(server._hold_state, "MANUAL_RELEASED")
        self.assertTrue(all(value == 0 for value in server._bus.torque.values()))

        server._on_control_mode(SimpleNamespace(data="real2sim"))
        self.assertEqual(server._hold_state, "RELEASED")
        server._on_control_mode(SimpleNamespace(data=COMMAND_CONTROL_MODE))
        self.assertEqual(server._hold_state, "HOLDING")
        self.assertTrue(all(value == 1 for value in server._bus.torque.values()))

        server._release_hold("safety trip", fault=True)
        server._release_hold("operator requested", manual=True)
        self.assertEqual(server._hold_state, "FAULT")
        server._on_control_mode(SimpleNamespace(data="real2sim"))
        self.assertEqual(server._hold_state, "FAULT")


if __name__ == "__main__":
    unittest.main()
