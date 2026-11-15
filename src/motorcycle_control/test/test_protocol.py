import math
import pytest
from motorcycle_control.protocol import Parser, command, configure, decode_status, speed_value


@pytest.mark.parametrize('speed,hexdata', [(5, '013200'), (20, '01c800'),
                                          (30, '012c01'), (40, '019001'),
                                          (50, '01f401'), (70, '01bc02')])
def test_commands(speed, hexdata):
    assert command(1, speed_value(speed, 70)).hex() == 'aac30002' + hexdata + '55'


def test_configuration():
    packet = configure(500000)
    assert len(packet) == 20
    assert packet[:5] == bytes.fromhex('aa55120301')
    assert packet[13:15] == b'\x00\x01'
    assert packet[-1] == sum(packet[2:19]) & 255


def test_fragmented_status_and_error():
    parser = Parser()
    packet = bytes.fromhex('aac8010245011f017c042a0055')
    for byte in packet[:-1]:
        assert parser.feed(bytes([byte])) == []
    frames = parser.feed(packet[-1:] + bytes.fromhex('aac30102ff010055'))
    assert len(frames) == 2
    assert frames[0][0] == 0x201
    assert decode_status(frames[0][1]) == dict(speed_kmh=32.5, wheel_rpm=287,
        motor_rpm=1148, throttle_percent=42, reserved=0)
    assert decode_status(frames[1][1]) == {'error': 1}


def test_noise_resync_and_embedded_markers():
    parser = Parser()
    packet = bytes.fromhex('aac80102aa0055000000000055')
    assert parser.feed(b'noise\xaa\xcf' + packet) == [(0x201, packet[4:-1])]
    # Extended and remote packets must not be treated as standard status replies.
    assert parser.feed(bytes.fromhex('aae301020000ff010055aad8010255')) == []


@pytest.mark.parametrize('value', [-1, 71, math.nan, math.inf])
def test_invalid_targets(value):
    with pytest.raises(ValueError):
        speed_value(value, 70)


@pytest.mark.parametrize('data', [b'', b'\xff', b'\x00\x01\x00', b'\x00' * 6 + b'\x65\x00'])
def test_invalid_status(data):
    with pytest.raises(ValueError):
        decode_status(data)
