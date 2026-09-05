"""Read-only AZD-KD ABZO and limit/status monitor for the lift."""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Int32, String

from .azd_modbus import (
    AzdReadOnlyModbusClient,
    ModbusError,
    bits_set,
    registers_to_i32,
    registers_to_u32,
)


class LiftAbzoMonitorNode(Node):
    """Poll AZD-KD monitor registers without writing to the driver."""

    def __init__(self) -> None:
        super().__init__("lift_abzo_monitor_node")
        self.port = str(self.declare_parameter("serial_port", "").value)
        self.slave_id = int(self.declare_parameter("slave_id", 1).value)
        self.baud_rate = int(self.declare_parameter("baud_rate", 115200).value)
        self.parity = str(self.declare_parameter("parity", "E").value).upper()
        self.stop_bits = int(self.declare_parameter("stop_bits", 1).value)
        self.poll_rate_hz = float(self.declare_parameter("poll_rate_hz", 5.0).value)
        self.timeout_sec = float(self.declare_parameter("timeout_sec", 0.15).value)
        self.mm_per_step = float(self.declare_parameter("position_mm_per_step", 0.0).value)
        self.position_offset_mm = float(
            self.declare_parameter("position_offset_mm", 0.0).value
        )
        self.physical_limit_inputs_configured = bool(
            self.declare_parameter("physical_limit_inputs_configured", False).value
        )
        self.software_limits_configured = bool(
            self.declare_parameter("software_limits_configured", False).value
        )

        self.raw_position_pub = self.create_publisher(
            Int32, "/lift/abzo_position_raw", 10
        )
        self.state_pub = self.create_publisher(String, "/lift/abzo_state", 10)

        self.client = None
        self.state: dict = {
            "schema_version": 1,
            "source": "AZD-KD Modbus RTU function 03 (read-only)",
            "connected": False,
            "serial_port": self.port,
            "slave_id": self.slave_id,
            "baud_rate": self.baud_rate,
            "poll_rate_hz": self.poll_rate_hz,
            "position": {},
            "limits": {},
            "driver": {},
            "communication": {
                "successful_queries": 0,
                "failed_queries": 0,
                "consecutive_failures": 0,
                "last_error": None,
            },
        }
        self._diagnostic_index = 0
        self._open_client()
        period = 1.0 / max(0.2, min(self.poll_rate_hz, 10.0))
        self.create_timer(period, self._poll)
        self.get_logger().info(
            "AZD-KD read-only monitor configured: "
            f"port={self.port or '<unset>'} id={self.slave_id} "
            f"serial={self.baud_rate},8{self.parity}{self.stop_bits} "
            f"rate={1.0 / period:.1f} Hz"
        )

    def _open_client(self) -> None:
        if not self.port:
            self.state["communication"]["last_error"] = (
                "serial_port is unset; provide the USB-RS485 /dev/serial/by-id path"
            )
            return
        if not Path(self.port).exists():
            self.state["communication"]["last_error"] = (
                f"serial device does not exist: {self.port}"
            )
            return
        try:
            self.client = AzdReadOnlyModbusClient(
                port=self.port,
                slave_id=self.slave_id,
                baud_rate=self.baud_rate,
                parity=self.parity,
                stop_bits=self.stop_bits,
                timeout_sec=self.timeout_sec,
            )
        except (OSError, ValueError, serial.SerialException) as exc:
            self.state["communication"]["last_error"] = str(exc)
            self.get_logger().error(f"Cannot open AZD-KD RS-485 port: {exc}")

    def _mark_success(self) -> None:
        communication = self.state["communication"]
        communication["successful_queries"] += 1
        communication["consecutive_failures"] = 0
        communication["last_error"] = None
        communication["last_success_at"] = datetime.now(timezone.utc).isoformat()
        self.state["connected"] = True

    def _mark_failure(self, exc: Exception) -> None:
        communication = self.state["communication"]
        communication["failed_queries"] += 1
        communication["consecutive_failures"] += 1
        communication["last_error"] = str(exc)
        if communication["consecutive_failures"] >= 3:
            self.state["connected"] = False
        if communication["consecutive_failures"] in {1, 3, 10}:
            self.get_logger().warn(f"AZD-KD read failed: {exc}")

    def _poll(self) -> None:
        if self.client is None or not self.client.is_open:
            self._publish_state()
            return
        try:
            # 00C6h..00CDh: command position through feedback position.
            registers = self.client.read_holding_registers(0x00C6, 8)
            command_position = registers_to_i32(registers[0], registers[1])
            feedback_position = registers_to_i32(registers[6], registers[7])
            position_mm = None
            if self.mm_per_step != 0.0:
                position_mm = feedback_position * self.mm_per_step + self.position_offset_mm
            self.state["position"] = {
                "feedback_steps": feedback_position,
                "command_steps": command_position,
                "position_mm": position_mm,
                "mm_per_step": self.mm_per_step if self.mm_per_step != 0.0 else None,
                "offset_mm": self.position_offset_mm if self.mm_per_step != 0.0 else None,
                "evidence": (
                    "ABZO_MODBUS_CALIBRATED"
                    if position_mm is not None
                    else "ABZO_MODBUS_RAW_UNCALIBRATED"
                ),
            }
            self.raw_position_pub.publish(Int32(data=feedback_position))
            self._mark_success()
            self._poll_one_diagnostic_group()
        except (ModbusError, OSError, serial.SerialException, ValueError) as exc:
            self._mark_failure(exc)
        self._publish_state()

    def _poll_one_diagnostic_group(self) -> None:
        readers = (
            self._read_alarm,
            self._read_direct_io,
            self._read_temperature,
            self._read_position_counters,
            self._read_power_and_switches,
            self._read_internal_io,
        )
        reader = readers[self._diagnostic_index % len(readers)]
        self._diagnostic_index += 1
        try:
            reader()
            self._mark_success()
        except (ModbusError, OSError, serial.SerialException, ValueError) as exc:
            self._mark_failure(exc)

    def _read_alarm(self) -> None:
        registers = self.client.read_holding_registers(0x0080, 2)
        self.state["driver"]["present_alarm_code"] = registers_to_u32(*registers)

    def _read_direct_io(self) -> None:
        registers = self.client.read_holding_registers(0x00D4, 2)
        self.state["driver"]["direct_io_raw"] = registers_to_u32(*registers)

    def _read_temperature(self) -> None:
        # 00F6h/7h present information; 00F8h/9h and 00FAh/Bh temperatures.
        registers = self.client.read_holding_registers(0x00F6, 6)
        self.state["driver"].update(
            {
                "present_information": registers_to_u32(registers[0], registers[1]),
                "driver_temperature_c": registers_to_i32(registers[2], registers[3]) / 10.0,
                "motor_temperature_c": registers_to_i32(registers[4], registers[5]) / 10.0,
            }
        )

    def _read_position_counters(self) -> None:
        registers = self.client.read_holding_registers(0x0120, 4)
        self.state["position"].update(
            {
                "feedback_counter_steps": registers_to_i32(registers[0], registers[1]),
                "command_counter_steps": registers_to_i32(registers[2], registers[3]),
            }
        )

    def _read_power_and_switches(self) -> None:
        registers = self.client.read_holding_registers(0x0146, 12)
        self.state["driver"].update(
            {
                "inverter_voltage_v": registers_to_u32(registers[0], registers[1]) / 10.0,
                "power_supply_voltage_v": registers_to_u32(registers[2], registers[3]) / 10.0,
                "sw1_raw": registers_to_u32(registers[4], registers[5]),
                "id_switch_raw": registers_to_u32(registers[6], registers[7]),
                "baud_switch_raw": registers_to_u32(registers[8], registers[9]),
                "rs485_reception_count": registers_to_u32(registers[10], registers[11]),
            }
        )

    def _read_internal_io(self) -> None:
        registers = self.client.read_holding_registers(0x0170, 10)
        input_status_1 = registers[0]
        output_status_1 = registers[8]
        output_status_2 = registers[9]
        physical = bits_set(
            input_status_1,
            (("fw_ls", 12), ("rv_ls", 13), ("homes", 14), ("slit", 15)),
        )
        software = bits_set(
            output_status_1,
            (
                ("home_end", 0),
                ("absolute_position_enabled", 1),
                ("fw_sls", 9),
                ("rv_sls", 10),
            ),
        )
        runtime = bits_set(
            output_status_2,
            (
                ("alarm_a", 1),
                ("alarm_b", 2),
                ("system_ready", 3),
                ("ready", 4),
                ("pulse_ready", 5),
                ("moving", 6),
                ("information", 7),
            ),
        )
        self.state["limits"] = {
            "physical_inputs_configured": self.physical_limit_inputs_configured,
            "physical": physical,
            "physical_evidence": (
                "AZD_INTERNAL_IO_CONFIRMED_CONFIGURED"
                if self.physical_limit_inputs_configured
                else "AZD_INTERNAL_IO_ASSIGNMENT_NOT_CONFIRMED"
            ),
            "software_limits_configured": self.software_limits_configured,
            "software": software,
            "software_evidence": (
                "AZD_SOFTWARE_LIMITS_CONFIRMED_CONFIGURED"
                if self.software_limits_configured
                else "AZD_SOFTWARE_LIMIT_CONFIGURATION_NOT_CONFIRMED"
            ),
        }
        self.state["driver"]["runtime_io"] = runtime

    def _publish_state(self) -> None:
        self.state["generated_at"] = datetime.now(timezone.utc).isoformat()
        self.state_pub.publish(String(data=json.dumps(self.state, ensure_ascii=False)))

    def destroy_node(self) -> None:
        if self.client is not None:
            self.client.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LiftAbzoMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
