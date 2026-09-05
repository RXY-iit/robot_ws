from serial_transciever.lift_control.lift_serial_node import (
    parse_lift_reply,
    usb_identity_matches,
)


def test_usb_identity_requires_both_verified_ids():
    identity = {'vendor_id': '2341', 'product_id': '0043'}
    assert usb_identity_matches(identity, '2341', '0043')
    assert not usb_identity_matches(identity, '1a86', '0043')
    assert not usb_identity_matches(identity, '2341', '55d3')
    assert not usb_identity_matches({}, '2341', '0043')


def test_lift_v2_replies_are_parsed():
    assert parse_lift_reply('READY LIFT_CONTROL_V2\r\n') == (
        'READY', 'LIFT_CONTROL_V2'
    )
    assert parse_lift_reply('IDENTITY LIFT_CONTROL_V2\n') == (
        'IDENTITY', 'LIFT_CONTROL_V2'
    )
    assert parse_lift_reply('ACK up\n') == ('ACK', 'UP')
    assert parse_lift_reply('ERR UNKNOWN_COMMAND X\n') == (
        'ERROR', 'UNKNOWN_COMMAND X'
    )
