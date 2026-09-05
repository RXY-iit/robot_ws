from serial_transciever.lift_control.azd_modbus import (
    AzdReadOnlyModbusClient,
    ModbusError,
    append_crc,
    bits_set,
    crc16_modbus,
    registers_to_i32,
    registers_to_u32,
)


class FakeSerial:
    def __init__(self, response):
        self.response = response
        self.request = None

    def reset_input_buffer(self):
        pass

    def write(self, request):
        self.request = request
        return len(request)

    def flush(self):
        pass

    def read(self, _length):
        return self.response


def test_crc_matches_standard_modbus_read_example():
    # Canonical request: slave 1, function 03, address 0, quantity 10.
    payload = bytes.fromhex("01 03 00 00 00 0A")
    assert crc16_modbus(payload) == 0xCDC5
    assert append_crc(payload) == bytes.fromhex("01 03 00 00 00 0A C5 CD")


def test_az_upper_lower_word_conversion():
    assert registers_to_i32(0x0000, 0x1234) == 0x1234
    assert registers_to_i32(0xFFFF, 0xFFFE) == -2
    assert registers_to_u32(0xABCD, 0x1234) == 0xABCD1234


def test_limit_bit_decode_uses_manual_bit_numbers():
    raw = (1 << 12) | (1 << 14)
    decoded = bits_set(raw, (("fw_ls", 12), ("rv_ls", 13), ("homes", 14)))
    assert decoded == {"fw_ls": True, "rv_ls": False, "homes": True}


def test_read_holding_registers_validates_and_decodes_response():
    response = append_crc(bytes.fromhex("01 03 04 00 00 12 34"))
    client = object.__new__(AzdReadOnlyModbusClient)
    client.slave_id = 1
    client.inter_request_delay_sec = 0.0
    client.last_bus_activity = 0.0
    client.serial = FakeSerial(response)

    assert client.read_holding_registers(0x00CC, 2) == [0, 0x1234]
    assert client.serial.request == append_crc(bytes.fromhex("01 03 00 CC 00 02"))


def test_read_holding_registers_rejects_bad_crc():
    response = append_crc(bytes.fromhex("01 03 04 00 00 12 34"))[:-1] + b"\x00"
    client = object.__new__(AzdReadOnlyModbusClient)
    client.slave_id = 1
    client.inter_request_delay_sec = 0.0
    client.last_bus_activity = 0.0
    client.serial = FakeSerial(response)

    try:
        client.read_holding_registers(0x00CC, 2)
    except ModbusError as exc:
        assert "CRC mismatch" in str(exc)
    else:
        raise AssertionError("bad CRC was accepted")
