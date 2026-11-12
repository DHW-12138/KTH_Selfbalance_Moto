import math
import struct
from unittest.mock import patch

import pytest
from iim42652_gyro.driver import IIM42652, decode_gyro


class Bus:
    def __init__(self):
        self.regs = {0x75: 0x6F, 0x2D: 0x18, 0x4C: 0x30}
        self.transfers = []
        self.ready = True

    def xfer2(self, data):
        self.transfers.append(data)
        reg = data[0] & 0x7F
        if data[0] & 0x80:
            if reg == 0x25:
                assert data == [0xA5, 0, 0, 0, 0, 0, 0]
                return [0] + list(struct.pack('>hhh', 131, -131, 0))
            if reg == 0x2D:
                return [0, 0x18 if self.ready else 0]
            return [0, self.regs.get(reg, 0)]
        assert len(data) == 2
        self.regs[reg] = data[1]
        return [0, 0]


def test_signed_units_and_invalid():
    values = decode_gyro(struct.pack('>hhh', 131, -131, 32767), 131.0)
    assert values[:2] == pytest.approx([math.pi / 180, -math.pi / 180])
    assert values[2] == pytest.approx(32767 / 131 * math.pi / 180)
    assert decode_gyro(struct.pack('>hhh', 0, -32768, 0), 131) is None


@patch('iim42652_gyro.driver.time.sleep')
def test_init_poll_and_stop(sleep):
    bus = Bus()
    sensor = IIM42652(bus)
    assert bus.regs[0x4F] == 0x68  # +/-250 dps, 100 Hz
    assert bus.regs[0x4C] == 0x33  # big endian, I2C disabled
    assert bus.regs[0x4E] == 0x0C
    assert sensor.sample() == pytest.approx([math.pi / 180, -math.pi / 180, 0])
    bus.ready = False
    assert sensor.sample() is None
    sensor.stop()
    assert bus.regs[0x4E] == 0
    assert any(call.args[0] >= 0.05 for call in sleep.call_args_list)


@patch('iim42652_gyro.driver.time.sleep')
def test_identity_errors(sleep):
    for identity in (0, 0xFF, 0x47):
        bus = Bus()
        bus.regs[0x75] = identity
        with pytest.raises(RuntimeError, match='WHO_AM_I'):
            IIM42652(bus)


@patch('iim42652_gyro.driver.time.sleep')
def test_reset_check_and_config_failure(sleep):
    bus = Bus()
    bus.ready = False
    with pytest.raises(RuntimeError, match='reset'):
        IIM42652(bus)
    bus = Bus()
    original = bus.xfer2
    bus.xfer2 = lambda data: [0, 0] if data == [0xCF, 0] else original(data)
    with pytest.raises(RuntimeError, match='readback'):
        IIM42652(bus)


def test_parameter_validation():
    with pytest.raises(ValueError):
        IIM42652(Bus(), full_scale=300.0)
    with pytest.raises(ValueError):
        IIM42652(Bus(), odr=123.0)
