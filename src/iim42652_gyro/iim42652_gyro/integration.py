"""Relative body-rate integration. Quaternion order is x, y, z, w."""
import math


class OrientationIntegrator:
    def __init__(self, max_dt=0.1):
        if not math.isfinite(max_dt) or max_dt <= 0:
            raise ValueError('integration_max_dt must be positive and finite')
        self.max_dt = max_dt
        self.reset()

    def reset(self):
        self.q = (0.0, 0.0, 0.0, 1.0)
        self.previous = None

    def update(self, velocity, now):
        """Return (quaternion, gap_skipped); preserve attitude across long gaps."""
        previous = self.previous
        self.previous = (now, tuple(velocity))
        if previous is None:
            return self.q, False
        dt = now - previous[0]
        if dt <= 0 or dt > self.max_dt:
            return self.q, True
        # Trapezoidal rate estimate followed by a rotation-vector exponential.
        rate = tuple((a + b) / 2 for a, b in zip(previous[1], velocity))
        speed = math.sqrt(sum(v * v for v in rate))
        half_angle = speed * dt / 2
        scale = math.sin(half_angle) / speed if speed > 1e-12 else dt / 2
        bx, by, bz = (v * scale for v in rate)
        bw = math.cos(half_angle)
        ax, ay, az, aw = self.q
        # q_new = q_old * delta_q: rates are expressed in the sensor/body frame.
        q = (aw*bx + ax*bw + ay*bz - az*by,
             aw*by - ax*bz + ay*bw + az*bx,
             aw*bz + ax*by - ay*bx + az*bw,
             aw*bw - ax*bx - ay*by - az*bz)
        norm = math.sqrt(sum(v*v for v in q))
        self.q = tuple(v / norm for v in q)
        return self.q, False


def quaternion_to_rpy(quaternion):
    """ZYX Euler representation (roll, pitch, yaw), in radians; not unwrapped."""
    norm = math.sqrt(sum(v * v for v in quaternion))
    if not math.isfinite(norm) or norm <= 0:
        raise ValueError('Quaternion must be finite and nonzero')
    x, y, z, w = (v / norm for v in quaternion)
    roll = math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y))
    sin_pitch = max(-1.0, min(1.0, 2 * (w*y - z*x)))
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
    return roll, pitch, yaw
