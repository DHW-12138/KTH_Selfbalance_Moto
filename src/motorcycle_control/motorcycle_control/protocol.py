"""Waveshare USB-CAN-A variable packets and motorcycle application protocol."""
import math
import struct

RATES = {1000000: 1, 800000: 2, 500000: 3, 400000: 4, 250000: 5,
         200000: 6, 125000: 7, 100000: 8, 50000: 9, 20000: 10, 10000: 11, 5000: 12}


def configure(bitrate):
    # Variable protocol, standard frames, accept all, normal mode, no auto retry.
    packet = bytearray([0xAA, 0x55, 0x12, RATES[bitrate], 1] + [0] * 8
                       + [0, 1] + [0] * 4)
    return bytes(packet + bytes([sum(packet[2:]) & 255]))


def command(code, value=0):
    payload = struct.pack('<BH', code, value)
    return b'\xaa\xc3\x00\x02' + payload + b'\x55'


def speed_value(speed, maximum):
    if not math.isfinite(speed) or not 0 <= speed <= maximum:
        raise ValueError(f'target_speed_kmh must be finite and between 0 and {maximum}')
    return int(math.floor(speed * 10 + 0.5))


class Parser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while self.buffer:
            if self.buffer[0] != 0xAA:
                del self.buffer[0]
                continue
            if len(self.buffer) < 2:
                break
            kind = self.buffer[1]
            dlc = kind & 15
            if kind & 0xC0 != 0xC0 or dlc > 8:
                del self.buffer[0]
                continue
            id_size = 4 if kind & 0x20 else 2
            size = 3 + id_size + (0 if kind & 0x10 else dlc)
            if len(self.buffer) < size:
                break
            if self.buffer[size - 1] != 0x55:
                del self.buffer[0]
                continue
            identifier = int.from_bytes(self.buffer[2:2 + id_size], 'little')
            if not kind & 0x30 and identifier <= 0x7FF:
                frames.append((identifier, bytes(self.buffer[4:size - 1])))
            del self.buffer[:size]
        return frames


def decode_status(data):
    if len(data) == 3 and data[0] == 0xFF:
        return {'error': int.from_bytes(data[1:], 'little')}
    if len(data) != 8:
        raise ValueError('status DLC must be 8; error DLC must be 3 with prefix FF')
    speed, wheel, motor, throttle, reserved = struct.unpack('<HHHBB', data)
    if throttle > 100:
        raise ValueError('invalid throttle percentage')
    return dict(speed_kmh=speed / 10, wheel_rpm=wheel, motor_rpm=motor,
                throttle_percent=throttle, reserved=reserved)
