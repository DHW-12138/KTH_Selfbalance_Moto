import math
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped
from std_srvs.srv import Trigger
from spidev import SpiDev

from .driver import IIM42652
from .integration import OrientationIntegrator, quaternion_to_rpy


class GyroNode(Node):
    def __init__(self):
        super().__init__('iim42652_gyro')
        self.bus = None
        self.sensor = None
        try:
            defaults = {'spi_bus': 0, 'spi_device': 0, 'spi_speed_hz': 100000,
                        'frame_id': 'imu_link',
                        'full_scale_dps': 250.0, 'odr_hz': 100.0,
                        'gyro_bias_rad_s': [0.0, 0.0, 0.0],
                        'angular_velocity_variance': 0.0,
                        'integration_max_dt': 0.1,
                        'integration_reference_frame': 'imu_initial'}
            for name, value in defaults.items():
                self.declare_parameter(name, value)
            get = lambda name: self.get_parameter(name).value
            self.frame = get('frame_id')
            self.integrator = OrientationIntegrator(get('integration_max_dt'))
            self.reference_frame = get('integration_reference_frame')
            if not self.reference_frame:
                raise ValueError('integration_reference_frame must not be empty')
            self.bias = list(get('gyro_bias_rad_s'))
            variance = float(get('angular_velocity_variance'))
            if len(self.bias) != 3 or not all(math.isfinite(v) for v in self.bias):
                raise ValueError('gyro_bias_rad_s must contain three finite values')
            if not self.frame or not math.isfinite(variance) or variance < 0:
                raise ValueError('Invalid frame_id or angular_velocity_variance')
            self.covariance = [variance if i in (0, 4, 8) else 0.0 for i in range(9)]
            speed = get('spi_speed_hz')
            if not 1 <= speed <= 1000000:
                raise ValueError('spi_speed_hz must be 1..1000000 for this driver')
            self.bus = SpiDev()
            self.bus.open(get('spi_bus'), get('spi_device'))
            self.bus.max_speed_hz = speed
            self.bus.mode = 0
            self.bus.bits_per_word = 8
            self.bus.lsbfirst = False
            self.bus.cshigh = False
            self.bus.no_cs = False
            self.sensor = IIM42652(self.bus, get('full_scale_dps'), get('odr_hz'))
            self.publisher = self.create_publisher(Imu, '/imu/data_raw', qos_profile_sensor_data)
            self.orientation_publisher = self.create_publisher(
                QuaternionStamped, '/imu/orientation_integrated', qos_profile_sensor_data)
            self.angles_publisher = self.create_publisher(
                Vector3Stamped, '/imu/angles', qos_profile_sensor_data)
            self.velocity_publisher = self.create_publisher(
                Vector3Stamped, '/imu/angular_velocity', qos_profile_sensor_data)
            self.reset_service = self.create_service(
                Trigger, '/imu/reset_integrated_orientation', self.reset_orientation)
            self.last_sample = time.monotonic()
            self.last_warning = 0.0
            # Poll faster than ODR to reduce scheduling-phase losses.
            self.timer = self.create_timer(1.0 / (2.0 * get('odr_hz')), self.poll)
            self.get_logger().info(f'IIM-42652 on SPI {get("spi_bus")}.{get("spi_device")}; publishing /imu/data_raw')
        except Exception:
            self.close()
            self.destroy_node()
            raise

    def poll(self):
        try:
            velocity = self.sensor.sample()
            now = time.monotonic()
            if velocity is None:
                if now - self.last_sample > 2.0 and now - self.last_warning > 5.0:
                    self.get_logger().warning('No valid gyro sample for >2 s; check sensor/power')
                    self.last_warning = now
                return
            self.last_sample = now
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame
            msg.orientation.w = 1.0
            msg.orientation_covariance[0] = -1.0
            msg.linear_acceleration_covariance[0] = -1.0
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (
                v - b for v, b in zip(velocity, self.bias))
            msg.angular_velocity_covariance = self.covariance
            self.publisher.publish(msg)
            velocity_corrected = (msg.angular_velocity.x, msg.angular_velocity.y,
                                  msg.angular_velocity.z)
            quaternion, skipped = self.integrator.update(velocity_corrected, now)
            if skipped and now - self.last_warning > 5.0:
                self.get_logger().warning('Integration skipped a time gap; relative attitude is incomplete')
                self.last_warning = now
            orientation = QuaternionStamped()
            orientation.header.stamp = msg.header.stamp
            orientation.header.frame_id = self.reference_frame
            (orientation.quaternion.x, orientation.quaternion.y,
             orientation.quaternion.z, orientation.quaternion.w) = quaternion
            self.orientation_publisher.publish(orientation)
            angles = Vector3Stamped()
            angles.header.stamp = msg.header.stamp
            angles.header.frame_id = self.reference_frame
            angles.vector.x, angles.vector.y, angles.vector.z = quaternion_to_rpy(quaternion)
            self.angles_publisher.publish(angles)
            angular_velocity = Vector3Stamped()
            angular_velocity.header = msg.header
            angular_velocity.vector.x = msg.angular_velocity.x
            angular_velocity.vector.y = msg.angular_velocity.y
            angular_velocity.vector.z = msg.angular_velocity.z
            self.velocity_publisher.publish(angular_velocity)
        except OSError as error:
            now = time.monotonic()
            if now - self.last_warning > 5.0:
                self.get_logger().error(f'SPI read failed: {error}; retrying')
                self.last_warning = now

    def reset_orientation(self, request, response):
        self.integrator.reset()
        response.success = True
        response.message = 'Relative orientation reset; next sample defines the reference attitude'
        return response

    def close(self):
        try:
            if self.sensor:
                self.sensor.stop()
        except OSError as error:
            self.get_logger().warning(f'Could not stop gyro: {error}')
        finally:
            if self.bus:
                self.bus.close()
                self.bus = None


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GyroNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
