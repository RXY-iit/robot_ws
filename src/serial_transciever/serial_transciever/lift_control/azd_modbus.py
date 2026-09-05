"""Small read-only Modbus RTU client for the Oriental Motor AZD-KD.

Only function 03 (Read Holding Registers) is implemented intentionally.  The
class exposes no register-write operation, so a monitoring bug cannot issue a
motion command or alter a driver parameter.
"""

from __future__ import annotations

import struct
import time
from typing import Iterable

import serial


class ModbusError(RuntimeError):
    """Raised when an RTU response is missing, malformed, or reports an error."""


def crc16_modbus(data: bytes) -> int:
    """Return the Modbus CRC-16 value for *data*."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def append_crc(payload: bytes) -> bytes:
    """Append Modbus's little-endian CRC to a payload."""
    return payload + struct.pack("<H", crc16_modbus(payload))


def registers_to_i32(upper: int, lower: int) -> int:
    """Combine the AZ manual's upper/lower 16-bit words as signed int32."""
    unsigned = ((upper & 0xFFFF) << 16) | (lower & 0xFFFF)
    return unsigned - 0x100000000 if unsigned & 0x80000000 else unsigned


def registers_to_u32(upper: int, lower: int) -> int:
    """Combine the AZ manual's upper/lower 16-bit words as unsigned int32."""
    return ((upper & 0xFFFF) << 16) | (lower & 0xFFFF)


class AzdReadOnlyModbusClient:
    """Read-only, two-wire Modbus RTU client for one AZD-KD driver."""

    def __init__(
        self,
        port: str,
        slave_id: int = 1,
        baud_rate: int = 115200,
        parity: str = "E",
        stop_bits: int = 1,
        timeout_sec: float = 0.15,
        inter_request_delay_sec: float = 0.005,
    ) -> None:
        if not 1 <= slave_id <= 31:
            raise ValueError("AZD-KD Modbus slave_id must be in [1, 31]")
        if parity not in {"N", "E", "O"}:
            raise ValueError("parity must be N, E, or O")
        if stop_bits not in {1, 2}:
            raise ValueError("stop_bits must be 1 or 2")
        self.slave_id = slave_id
        self.inter_request_delay_sec = max(0.0, inter_request_delay_sec)
        self.last_bus_activity = 0.0
        self.serial = serial.Serial(
            port=port,
            baudrate=baud_rate,
            bytesize=serial.EIGHTBITS,
            parity=parity,
            stopbits=stop_bits,
            timeout=timeout_sec,
            write_timeout=timeout_sec,
        )

    @property
    def is_open(self) -> bool:
        return bool(self.serial.is_open)

    def close(self) -> None:
        self.serial.close()

    def read_holding_registers(self, address: int, count: int) -> list[int]:
        """Read holding registers with function 03.

        This is the only operation supported by this client.
        """
        if not 0 <= address <= 0xFFFF:
            raise ValueError("register address is outside uint16")
        if not 1 <= count <= 125:
            raise ValueError("register count must be in [1, 125]")

        payload = struct.pack(">BBHH", self.slave_id, 0x03, address, count)
        request = append_crc(payload)
        expected_length = 5 + count * 2
        remaining_silence = self.inter_request_delay_sec - (
            time.monotonic() - self.last_bus_activity
        )
        if remaining_silence > 0.0:
            time.sleep(remaining_silence)
        self.serial.reset_input_buffer()
        written = self.serial.write(request)
        if written != len(request):
            raise ModbusError(f"short request write: {written}/{len(request)} bytes")
        self.serial.flush()
        response = self.serial.read(expected_length)
        self.last_bus_activity = time.monotonic()
        if len(response) < 5:
            raise ModbusError(f"short/no response: {len(response)} byte(s)")
        if response[0] != self.slave_id:
            raise ModbusError(
                f"unexpected slave id {response[0]}, expected {self.slave_id}"
            )
        if response[1] == 0x83:
            raise ModbusError(f"driver exception response: code 0x{response[2]:02X}")
        if response[1] != 0x03:
            raise ModbusError(f"unexpected function code 0x{response[1]:02X}")
        byte_count = response[2]
        if byte_count != count * 2:
            raise ModbusError(
                f"unexpected data length {byte_count}, expected {count * 2}"
            )
        total_length = byte_count + 5
        if len(response) != total_length:
            raise ModbusError(
                f"short response: {len(response)}/{total_length} bytes"
            )
        received_crc = struct.unpack("<H", response[-2:])[0]
        expected_crc = crc16_modbus(response[:-2])
        if received_crc != expected_crc:
            raise ModbusError(
                f"CRC mismatch: received 0x{received_crc:04X}, "
                f"expected 0x{expected_crc:04X}"
            )
        data = response[3:-2]
        return list(struct.unpack(f">{count}H", data))


def bits_set(value: int, names: Iterable[tuple[str, int]]) -> dict[str, bool]:
    """Decode named bits from one register."""
    return {name: bool(value & (1 << bit)) for name, bit in names}
