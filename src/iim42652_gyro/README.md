# IIM-42652 SPI Gyroscope for Raspberry Pi 5 and ROS 2 Jazzy

Python ROS 2 package for the MikroElektronika **6DOF IMU 17 Click**, running on Raspberry Pi 5 with Ubuntu 24.04 and ROS 2 Jazzy.

The node reads three-axis angular velocity over SPI, subtracts configurable gyroscope bias, and publishes angular velocity, integrated relative orientation, and roll/pitch/yaw angles. Defaults are **±250 °/s**, **100 Hz sensor ODR**, and **100 kHz SPI**.

The accelerometer is currently disabled. Orientation is estimated by gyroscope integration only; no accelerometer or magnetometer fusion is implemented. All angles are relative to the initial or reset attitude and will drift over time.

## 1. Wiring

Disconnect power before wiring. Keep all four COMM SEL jumpers, JP1–JP4, in the **SPI** position. The manufacturer identifies this as the factory-default left position; confirm against the board's SPI/I2C labels.

Use the Raspberry Pi **physical header pin numbers**, not BCM GPIO numbers:

| Raspberry Pi 5 physical pin | Click board pin |
|---|---|
| 1: 3.3 V | 3V3 / 3.3V |
| 6: GND | GND |
| 19: GPIO10 / MOSI | SDI / MOSI |
| 21: GPIO9 / MISO | SDO / MISO |
| 23: GPIO11 / SCLK | SCK |
| 24: GPIO8 / CE0 | CS |

Leave SDA, SCL, INT, and NC pins disconnected. Supply **3.3 V**, not 5 V. Use short jumper wires. Disconnect any other host, such as a Teensy, from the IMU before connecting the Raspberry Pi.

SPI uses the CE0 chip-select signal, not an I2C device address. Do not use `i2cdetect` to test this connection.

References: [Click board specifications and pinout](https://www.mikroe.com/6dof-imu-17-click), [Raspberry Pi GPIO documentation](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html).

## 2. Enable SPI and install dependencies

This guide assumes ROS 2 Jazzy is already installed. Use the system Python environment so ROS packages remain accessible.

In `/boot/firmware/config.txt`, ensure the following appears under `[all]`:

```ini
dtparam=spi=on
```

Save and reboot, then run:

```bash
sudo apt update
sudo apt install python3-spidev python3-colcon-common-extensions python3-pytest \
  ros-jazzy-sensor-msgs ros-jazzy-geometry-msgs ros-jazzy-std-srvs \
  ros-jazzy-launch-ros ros-jazzy-ament-index-python
ls -l /dev/spidev*
```

The expected device is `/dev/spidev0.0`. If it is absent, check the boot/device-tree configuration before changing the node's bus parameters.

### Device permissions

Check the group owning `/dev/spidev0.0`. On the tested Raspberry Pi it was `dialout`:

```bash
sudo usermod -aG dialout "$USER"
```

Log out and back in, or run `newgrp dialout` in the terminal used to launch the node. If the device belongs to `spi` instead, add the user to that group.

If dedicated permissions are needed, create a rule for this device:

```bash
sudo groupadd -f spi
sudo usermod -aG spi "$USER"
echo 'SUBSYSTEM=="spidev", KERNEL=="spidev0.0", GROUP="spi", MODE="0660"' | sudo tee /etc/udev/rules.d/60-iim42652-spi.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=spidev
```

Log out and back in after changing group membership. Run the ROS node as the regular user.

### Check sensor identity

Stop the ROS IMU node and any other programs accessing this SPI device before running this test:

```bash
python3 - <<'PYCODE'
import spidev
import time

spi = spidev.SpiDev()
try:
    spi.open(0, 0)
    spi.max_speed_hz = 100000
    spi.mode = 0
    spi.bits_per_word = 8
    time.sleep(0.05)
    spi.xfer2([0x76, 0x00])  # Select bank 0.
    identity = spi.xfer2([0x75 | 0x80, 0x00])[1]
    print(f'WHO_AM_I = 0x{identity:02X} (expected 0x6F)')
finally:
    spi.close()
PYCODE
```

An identity of `0x00` or `0xFF` means no valid identity was read. Check power, chip select, MISO, wiring, and jumper continuity. The presence of a device file alone does not establish sensor communication.

## 3. Build and run

Copy the workspace to `~/ros2_ws` on the Raspberry Pi, preserving this layout:

```text
ros2_ws/
└── src/
    └── iim42652_gyro/
        ├── package.xml
        ├── setup.py
        ├── config/
        ├── launch/
        ├── iim42652_gyro/
        └── test/
```

Build and launch:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select iim42652_gyro
source install/setup.bash
ros2 launch iim42652_gyro imu.launch.py
```

Alternatively, run the executable directly:

```bash
ros2 run iim42652_gyro imu_node --ros-args \
  -p spi_bus:=0 -p spi_device:=0 -p spi_speed_hz:=100000 \
  -p full_scale_dps:=250.0 -p odr_hz:=100.0
```

To use a custom parameter file:

```bash
ros2 launch iim42652_gyro imu.launch.py params_file:=/absolute/path/to/imu.yaml
```

Parameters are read at startup. Restart the node after changing them. If editing the package's `config/imu.yaml`, rebuild before launching to ensure the installed configuration is updated.

## 4. Topics and reset service

All publishers use Sensor Data QoS: **best effort, volatile**. Subscribers must use compatible QoS. Messages from the same valid sample share the same timestamp.

| Topic | Message type | Contents | Units / frame |
|---|---|---|---|
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | Bias-corrected angular velocity | rad/s, `imu_link` by default |
| `/imu/angular_velocity` | `geometry_msgs/msg/Vector3Stamped` | `vector.x/y/z = wx/wy/wz` | rad/s, sensor/body frame |
| `/imu/orientation_integrated` | `geometry_msgs/msg/QuaternionStamped` | Relative orientation, quaternion x/y/z/w | Relative to `imu_initial` by default |
| `/imu/angles` | `geometry_msgs/msg/Vector3Stamped` | `vector.x/y/z = roll/pitch/yaw` | rad, ZYX Euler representation relative to the initial attitude |

In `/imu/data_raw`, `orientation_covariance[0]` and `linear_acceleration_covariance[0]` are `-1`, indicating unavailable fields. Its orientation placeholder must not be used as an attitude estimate. Angular velocity is identical to that published on `/imu/angular_velocity`.

Open another terminal on the Raspberry Pi:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic list -t
ros2 topic info /imu/data_raw --verbose
ros2 topic echo /imu/data_raw --qos-reliability best_effort --once
ros2 topic echo /imu/angles --qos-reliability best_effort --once
ros2 topic echo /imu/angular_velocity --qos-reliability best_effort --once
ros2 topic hz /imu/data_raw
```

Omit `--once` to stream messages; press Ctrl+C to stop viewing.

Reset the integrated attitude:

```bash
ros2 service call /imu/reset_integrated_orientation std_srvs/srv/Trigger '{}'
```

Resetting defines a new reference attitude at the next valid sample. It resets both quaternion and Euler-angle estimates, but does not calibrate or modify gyroscope bias. Restarting the node also resets the attitude.

## 5. Parameters and coordinate conventions

| Parameter | Default | Meaning |
|---|---|---|
| `spi_bus` | `0` | SPI bus number |
| `spi_device` | `0` | Chip select; 0 selects CE0 |
| `spi_speed_hz` | `100000` | SPI clock; accepted range 1–1,000,000 Hz |
| `frame_id` | `imu_link` | Sensor/body frame used for angular velocity |
| `full_scale_dps` | `250.0` | Gyroscope full-scale magnitude in °/s |
| `odr_hz` | `100.0` | Sensor output data rate |
| `gyro_bias_rad_s` | `[0.0, 0.0, 0.0]` | Bias subtracted from each sensor-frame axis, in rad/s |
| `angular_velocity_variance` | `0.0` | Shared diagonal variance, in (rad/s)²; zero means unknown |
| `integration_max_dt` | `0.1` | Maximum accepted integration interval, seconds |
| `integration_reference_frame` | `imu_initial` | Reference frame for integrated orientation and angles |

The driver preserves the chip's XYZ axes and right-hand rotation convention. Check the datasheet axis diagram against the physical mounting. Changing `frame_id` does not rotate measurements. The node does not publish TF; define the mounting transform to `base_link` separately.

The quaternion represents the current IMU frame's orientation relative to the initial reference frame. `QuaternionStamped` has no child-frame field; the oriented body is the sensor frame named by `frame_id`.

Euler angles use the convention `R = Rz(yaw) Ry(pitch) Rx(roll)`. Roll and yaw lie in `[-pi, pi]`; pitch lies in `[-pi/2, pi/2]`. Roll/yaw can jump at the wrap boundary, and Euler angles are singular near pitch ±90°. Controllers must handle wrapped angle errors. Body angular velocities wx/wy/wz are generally **not** the derivatives of roll/pitch/yaw.

The RPY values carried in `Vector3Stamped` are a documented tuple, not an ordinary geometric direction vector. Do not transform these angles using a TF vector rotation. For general 3D attitude control, use quaternion attitude errors.

## 6. SPI protocol, registers, and scaling

The driver uses SPI mode 0, eight-bit words, MSB first, and active-low chip select. The chip defaults to four-wire SPI after reset. Read commands use `register | 0x80`; writes use `register & 0x7F`. A single `xfer2` keeps CS low across the command and data. No additional dummy byte is inserted: reading three axes takes seven transferred bytes, including the command.

If an earlier I2C program disabled SPI, power-cycle the sensor before switching to this SPI driver.

Register settings follow [TDK DS-000440 Rev 1.1](https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000440-iim-42652-typ-v1.1.pdf), Table 1 and the interface/register descriptions:

| Bank 0 register | Driver behavior |
|---|---|
| `REG_BANK_SEL`, `0x76` | Select bank 0 |
| `WHO_AM_I`, `0x75` | Require `0x6F` |
| `DEVICE_CONFIG`, `0x11` | Write `0x01` for soft reset; wait 10 ms, exceeding the 1 ms minimum |
| `INT_STATUS`, `0x2D` | Check bit 4 for reset completion and bit 3 for data ready; read clears status |
| `INTF_CONFIG0`, `0x4C` | Preserve other bits; bit 7=0, bit 4=1, bits 1:0=11: invalid-sample marker, big-endian data, I2C disabled |
| `GYRO_CONFIG0`, `0x4F` | Default `0x68`: FS_SEL=3, ODR=8 |
| `PWR_MGMT0`, `0x4E` | `0x0C`: gyroscope low-noise mode, accelerometer off; wait 50 ms |
| `GYRO_DATA_X1` through `Z0`, `0x25`–`0x2A` | Read six contiguous bytes as three big-endian signed 16-bit integers |

After enabling the gyroscope, no register writes are issued for at least 200 µs, and the gyroscope remains enabled for at least 45 ms. The driver waits 50 ms and also enforces the minimum on-time when stopping it. Filter settings retain their reset defaults. Initialization includes configuration readback checks.

| Full scale (±°/s) | Sensitivity (LSB/(°/s)) |
|---|---|
| 2000 | 16.4 |
| 1000 | 32.8 |
| 500 | 65.5 |
| 250 | 131 |
| 125 | 262 |
| 62.5 | 524.3 |
| 31.25 | 1048.6 |
| 15.625 | 2097.2 |

Conversion is `rad/s = raw / sensitivity × pi / 180`, followed by subtraction of the configured bias. A sample containing the invalid marker `-32768` on any axis is discarded.

Supported ODRs are 12.5, 25, 50, 100, 200, and 500 Hz, with register encodings 0xB, 0xA, 0x9, 0x8, 0x7, and 0xF respectively.

For message semantics, see the [ROS Imu message definition](https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/Imu.msg).

## 7. Integration and timing limitations

The node averages adjacent bias-corrected angular velocity samples, converts the resulting rotation vector to an incremental quaternion, right-multiplies the current quaternion, and normalizes the result. Euler angles are computed from this quaternion, rather than by treating independent axis integrals as Euler angles.

Integration uses monotonic host time for `dt`; message timestamps use the ROS clock after reading the sensor. These are not hardware sampling timestamps. Intervals longer than `integration_max_dt` are skipped while the existing attitude is retained. A warning is emitted subject to log throttling. Missing motion cannot be reconstructed; reset the attitude after a significant interruption.

The polling timer runs at twice the sensor ODR and publishes only when DATA_RDY is set and the sample is valid. Actual publication frequency may be below the nominal sensor ODR because of Linux scheduling and SPI overhead. FIFO buffering is not used, so retention of every sensor sample is not guaranteed.

On SPI read errors, stale values are not republished and errors are logged at most once every five seconds. If the sensor loses power, restart the node to reinitialize it.

Gyroscope integration has no gravity or magnetic reference. Bias and temperature changes accumulate into angle drift. The initial attitude is defined as zero; it is not necessarily level, north-aligned, or aligned with a world frame.

For a rough functional check, static angular velocity should be close to zero after calibration. Positive right-hand rotation should produce positive angular velocity on the corresponding axis. A rate of 90 °/s is approximately 1.571 rad/s. Known-angle rotation tests can reveal major scale/sign errors; precise accuracy validation requires a suitable reference instrument or rate table. Avoid exceeding the configured full scale.

## 8. Bias calibration and independent drift verification

**Stop the ROS IMU node and all other programs accessing the SPI device first.** The calibration tool accesses the device directly; it does not coordinate access with other processes.

Secure the IMU and keep it completely still throughout the procedure. Defaults are 60 seconds of warmup, 60 seconds of calibration, and 60 seconds of independent verification, approximately three minutes total.

After building and sourcing the workspace:

```bash
ros2 run iim42652_gyro calibrate_gyro \
  --output "$HOME/imu_calibration_$(date +%Y%m%d_%H%M%S)"
```

Alternatively, run directly from the source directory without building:

```bash
cd ~/ros2_ws/src/iim42652_gyro
python3 -m iim42652_gyro.calibrate \
  --output "$HOME/imu_calibration_$(date +%Y%m%d_%H%M%S)"
```

See all options with:

```bash
python3 -m iim42652_gyro.calibrate --help
```

The tool initializes the sensor at ±250 °/s and 100 Hz and measures angular velocity before subtracting any previously configured bias. This avoids subtracting bias twice during recalibration.

It produces:

- A `.json` report containing sample count, observed frequency, per-axis mean, sample standard deviation, variance, maximum sample gap, first/second-half mean differences, independent verification residual bias, and relative RPY drift in degrees.
- A `.yaml` ROS parameter file containing the measured bias, selected SPI bus/device, and matching sensor configuration. The file uses JSON syntax, which is valid YAML.

The node supports one shared diagonal angular velocity variance. The generated configuration uses the largest of the three measured axis variances. This is measurement variance, not the variance of the estimated mean.

Heuristic checks reject obvious motion, sample counts below 70% of nominal, gaps above 0.2 seconds, and substantial changes in mean. Skipped integration intervals during verification also prevent parameter-file generation. These checks cannot detect sustained slow constant rotation: the operator must ensure the sensor is stationary. Passing the checks does not certify a particular control-system accuracy.

Existing output files are not overwritten. The tool does not automatically replace the active node configuration. Review the report, then launch with the actual generated path:

```bash
ros2 launch iim42652_gyro imu.launch.py \
  params_file:=/home/hanswang/imu_calibration_YYYYMMDD_HHMMSS.yaml
```

Continue specifying this parameter file on subsequent launches; otherwise the default bias remains zero. Restarting the node resets integrated angles. Reassess calibration after changes in mounting, temperature, sensor range, ODR, or filter configuration.

## 9. Tests and validation status

Run the automated tests:

```bash
cd ~/ros2_ws/src/iim42652_gyro
python3 -m pytest -q test
```

The current suite contains **15 tests**, covering SPI transactions, identity/configuration failures, signed decoding and units, invalid samples, quaternion integration and composition, gaps and reset, normalization, Euler conversion, and calibration statistics/rejection checks. Python syntax checks also passed in the development environment.

The operator has reported successful Raspberry Pi SPI identity reads (`0x6F`), ROS topic publication, and a completed calibration/verification run. The development environment itself does not have an attached IMU and has not independently reproduced these hardware results. Precise dynamic accuracy, long-term drift, and final vehicle installation calibration remain to be verified.
