"""Bank-0, 16-bit UI-register driver; no ROS dependency."""
import math
import struct
import time

# DS-000440 revision 1.1, Table 1 and user bank 0 register map.
SCALES = {2000.0: (0, 16.4), 1000.0: (1, 32.8), 500.0: (2, 65.5),
          250.0: (3, 131.0), 125.0: (4, 262.0), 62.5: (5, 524.3),
          31.25: (6, 1048.6), 15.625: (7, 2097.2)}
ODRS = {12.5: 11, 25.0: 10, 50.0: 9, 100.0: 8, 200.0: 7, 500.0: 15}


def decode_gyro(data, lsb_per_dps):
    raw = struct.unpack('>hhh', bytes(data))
    # INTF_CONFIG0 bit 7 is zero: -32768 represents an invalid sample.
    if -32768 in raw:
        return None
    return tuple(v / lsb_per_dps * math.pi / 180.0 for v in raw)


class IIM42652:
    def __init__(self, bus, full_scale=250.0, odr=100.0):
        if full_scale not in SCALES or odr not in ODRS:
            raise ValueError('Unsupported full_scale_dps or odr_hz')
        self.bus = bus
        self.started = None
        time.sleep(0.01)
        self.write(0x76, 0)
        identity = self.read(0x75)
        if identity != 0x6F:
            raise RuntimeError(f'WHO_AM_I=0x{identity:02X}, expected 0x6F; check SPI wiring/jumpers')
        self.lsb_per_dps = SCALES[full_scale][1]
        try:
            # A previous process may have left the gyro running.
            time.sleep(0.05)
            self.write(0x11, 0x01)
            time.sleep(0.01)  # Soft reset requires >=1 ms, no accesses meanwhile.
            self.write(0x76, 0x00)
            if self.read(0x75) != 0x6F or not self.read(0x2D) & 0x10:
                raise RuntimeError('WHO_AM_I/reset completion check failed')
            self.write(0x4C, (self.read(0x4C) & ~0x83) | 0x10 | 0x03)
            config = (SCALES[full_scale][0] << 5) | ODRS[odr]
            self.write(0x4F, config)
            if self.read(0x4F) != config or self.read(0x4C) & 0x93 != 0x13:
                raise RuntimeError('Configuration readback failed')
            self.write(0x4E, 0x0C)  # Gyro LN, accel off; temperature default on.
            self.started = time.monotonic()
            time.sleep(0.05)  # No writes for >=200 us; gyro stays on >=45 ms.
            if self.read(0x4E) != 0x0C:
                raise RuntimeError('PWR_MGMT0 readback failed')
            self.read(0x2D)  # Clear pre-startup status.
        except Exception:
            self.stop()
            raise

    def read(self, register):
        return self.read_block(register, 1)[0]

    def write(self, register, value):
        self.bus.xfer2([register & 0x7F, value])

    def read_block(self, register, count):
        # xfer2 holds CS low across command and all data bytes; no dummy byte.
        return self.bus.xfer2([register | 0x80] + [0] * count)[1:]

    def sample(self):
        if not self.read(0x2D) & 0x08:
            return None
        # One contiguous transaction, high byte first, signed two's complement.
        return decode_gyro(self.read_block(0x25, 6),
                           self.lsb_per_dps)

    def stop(self):
        if self.started is not None:
            time.sleep(max(0.0, 0.045 - (time.monotonic() - self.started)))
            try:
                self.write(0x4E, 0)
            finally:
                self.started = None
