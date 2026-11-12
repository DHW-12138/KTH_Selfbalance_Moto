"""Standalone SPI calibration. Stop the ROS IMU node before running."""
import argparse
import json
import math
from pathlib import Path
import statistics
import time

from .driver import IIM42652
from .integration import OrientationIntegrator, quaternion_to_rpy


def summarize(samples, duration, odr, max_std, max_peak):
    if len(samples) < 2:
        raise ValueError('Too few valid samples')
    columns = list(zip(*(v for _, v in samples)))
    bias = [statistics.mean(c) for c in columns]
    std = [statistics.stdev(c) for c in columns]
    peak = max(abs(v) for c in columns for v in c)
    split = len(samples) // 2
    half_difference = [abs(statistics.mean(c[:split]) - statistics.mean(c[split:]))
                       for c in columns]
    gaps = [b[0] - a[0] for a, b in zip(samples, samples[1:])]
    reasons = []
    if len(samples) < duration * odr * 0.7:
        reasons.append('Valid sample count below 70% of nominal ODR')
    if max(gaps) > 0.2:
        reasons.append('Sample gap exceeded 0.2 seconds')
    if max(std) > max_std:
        reasons.append('Noise/movement exceeded max-std')
    if peak > max_peak:
        reasons.append('Measured rotation exceeded max-peak')
    if max(half_difference) > max_std:
        reasons.append('First/second half means differ: warming or movement suspected')
    return dict(samples=len(samples), observed_hz=len(samples)/duration,
                bias_rad_s=bias, std_rad_s=std,
                variance_rad2_s2=[v*v for v in std], max_sample_gap_s=max(gaps),
                half_mean_difference_rad_s=half_difference, peak_rad_s=peak,
                accepted=not reasons, rejection_reasons=reasons)


def capture(sensor, duration, label):
    samples = []
    start = time.monotonic()
    next_progress = start + 10
    print(f'{label}: starting {duration:g} seconds; keep IMU completely still', flush=True)
    while time.monotonic() - start < duration:
        velocity = sensor.sample()
        now = time.monotonic()
        if velocity is not None:
            samples.append((now, velocity))
        if now >= next_progress:
            print(f'  {now-start:.0f}s, {len(samples)} valid samples', flush=True)
            next_progress += 10
        time.sleep(0.001)
    return samples


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=60)
    parser.add_argument('--verify-duration', type=float, default=60)
    parser.add_argument('--warmup', type=float, default=60)
    parser.add_argument('--bus', type=int, default=0)
    parser.add_argument('--device', type=int, default=0)
    parser.add_argument('--output', default='imu_calibration')
    parser.add_argument('--max-std', type=float, default=0.01, help='rad/s stationarity heuristic')
    parser.add_argument('--max-peak', type=float, default=0.1, help='rad/s stationarity heuristic')
    args = parser.parse_args()
    for name in ('duration', 'verify_duration', 'max_std', 'max_peak'):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            parser.error(f'{name} must be finite and positive')
    if not math.isfinite(args.warmup) or args.warmup < 0:
        parser.error('warmup must be finite and nonnegative')
    if args.duration > 600 or args.verify_duration > 600:
        parser.error('Capture duration must be <=600 seconds')
    prefix = Path(args.output).expanduser().resolve()
    report_path = Path(str(prefix) + '.json')
    config_path = Path(str(prefix) + '.yaml')
    if report_path.exists() or config_path.exists():
        parser.error('Output exists; choose another --output prefix to preserve previous measurements')
    prefix.parent.mkdir(parents=True, exist_ok=True)
    from spidev import SpiDev
    bus = SpiDev()
    sensor = None
    try:
        bus.open(args.bus, args.device)
        bus.mode = 0
        bus.max_speed_hz = 100000
        bus.bits_per_word = 8
        sensor = IIM42652(bus, full_scale=250.0, odr=100.0)
        print('WHO_AM_I OK; gyro +/-250 dps, 100 Hz. Stop if any other process uses SPI.', flush=True)
        print(f'Warmup {args.warmup:g}s; keep the sensor still throughout.', flush=True)
        # Read during warmup to keep status/data current; no results retained.
        until = time.monotonic() + args.warmup
        while time.monotonic() < until:
            sensor.sample()
            time.sleep(0.005)
        samples = capture(sensor, args.duration, 'Calibration')
        calibration = summarize(samples, args.duration, 100, args.max_std, args.max_peak)
        report = dict(calibration=calibration, spi_bus=args.bus, spi_device=args.device,
                      full_scale_dps=250.0, odr_hz=100.0,
                      note='Gyro-only stationarity checks cannot detect constant slow rotation.')
        if not calibration['accepted']:
            report_path.write_text(json.dumps(report, indent=2) + '\n')
            raise RuntimeError(f'Calibration rejected: {calibration["rejection_reasons"]}; report: {report_path}')
        bias = calibration['bias_rad_s']
        print(f'Bias rad/s: {bias}', flush=True)
        verification = capture(sensor, args.verify_duration, 'Independent drift verification')
        quality = summarize(verification, args.verify_duration, 100, args.max_std, args.max_peak)
        integrator = OrientationIntegrator(max_dt=0.1)
        skipped = 0
        for stamp, velocity in verification:
            _, gap = integrator.update(tuple(v-b for v, b in zip(velocity, bias)), stamp)
            skipped += int(gap)
        residual = [v-b for v, b in zip(quality['bias_rad_s'], bias)]
        report['verification'] = dict(quality=quality, residual_bias_rad_s=residual,
            relative_rpy_deg=[math.degrees(v) for v in quaternion_to_rpy(integrator.q)],
            skipped_intervals=skipped, duration_s=args.verify_duration)
        report_path.write_text(json.dumps(report, indent=2) + '\n')
        if not quality['accepted'] or skipped:
            raise RuntimeError(f'Verification quality failed; no YAML written. Inspect {report_path}')
        # JSON is valid YAML; retain full hardware configuration in generated params.
        params = {'iim42652_gyro': {'ros__parameters': {
            'spi_bus': args.bus, 'spi_device': args.device, 'spi_speed_hz': 100000,
            'frame_id': 'imu_link', 'full_scale_dps': 250.0, 'odr_hz': 100.0,
            'gyro_bias_rad_s': bias,
            'angular_velocity_variance': max(calibration['variance_rad2_s2']),
            'integration_max_dt': 0.1, 'integration_reference_frame': 'imu_initial'}}}
        config_path.write_text(json.dumps(params, indent=2) + '\n')
        print('Verification RPY drift (degrees):', report['verification']['relative_rpy_deg'])
        print('Verification residual bias rad/s:', residual)
        print(f'Report: {report_path}\nROS params: {config_path}', flush=True)
        print('No hardware-independent drift accuracy threshold is assumed; inspect the report before control use.')
    finally:
        try:
            if sensor is not None:
                sensor.stop()
        finally:
            bus.close()


if __name__ == '__main__':
    main()
