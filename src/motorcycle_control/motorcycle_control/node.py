"""SSH-operated ROS 2 speed supervisor. All callbacks use one executor thread."""
import json
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String
from std_srvs.srv import Trigger
import serial

from .protocol import Parser, RATES, command, configure, decode_status, speed_value


class Master(Node):
    def __init__(self):
        super().__init__('motorcycle_master')
        defaults = dict(port='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
                        serial_baud=2000000, can_bitrate=500000,
                        max_speed_kmh=70.0, target_speed_kmh=0.0,
                        feedback_timeout_s=1.0)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        get = lambda key: self.get_parameter(key).value
        self.maximum = get('max_speed_kmh')
        self.timeout = get('feedback_timeout_s')
        if not 0 < self.maximum <= 6553.5 or not 0.2 <= self.timeout <= 60:
            raise ValueError('invalid maximum speed or feedback timeout')
        if get('can_bitrate') not in RATES:
            raise ValueError('unsupported CAN bitrate')
        self.target = speed_value(get('target_speed_kmh'), self.maximum)
        self.active = False
        self.last_status = None
        self.fault = False
        self.parser = Parser()
        self.bus = serial.Serial(get('port'), get('serial_baud'), timeout=0,
                                 write_timeout=0.1, exclusive=True)
        try:
            self.write(configure(get('can_bitrate')))
            time.sleep(0.1)
            self.write(command(0))
        except Exception:
            self.bus.close()
            raise
        self.publisher = self.create_publisher(String, '~/status', 10)
        self.create_service(Trigger, '~/start', self.start)
        self.create_service(Trigger, '~/stop', self.stop)
        self.add_on_set_parameters_callback(self.parameters)
        # Wall-clock timers: /clock pauses must not pause communication supervision.
        from rclpy.clock import Clock, ClockType
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.01, self.receive, clock=self.wall_clock)
        self.create_timer(0.1, self.transmit, clock=self.wall_clock)
        self.create_timer(0.2, self.poll, clock=self.wall_clock)
        self.get_logger().info('Ready in STOP state; waiting for Teensy status. CAN settings sent, not verified.')

    def write(self, packet):
        if self.bus.write(packet) != len(packet):
            raise OSError('short serial write')

    def parameters(self, params):
        try:
            for p in params:
                if p.name != 'target_speed_kmh':
                    raise ValueError('only target_speed_kmh may change at runtime')
                speed_value(p.value, self.maximum)
            for p in params:
                self.target = speed_value(p.value, self.maximum)
            return SetParametersResult(successful=True)
        except (ValueError, TypeError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))

    def healthy(self):
        return (not self.fault and self.last_status is not None
                and time.monotonic() - self.last_status < self.timeout)

    def start(self, request, response):
        if not self.healthy() or self.target == 0:
            response.success = False
            response.message = 'Need fresh valid status, no controller fault, and a positive target'
            return response
        try:
            self.write(command(1, self.target))
            self.active = True
            response.success = True
            response.message = f'SET_SPEED sent: {self.target / 10} km/h; execution not acknowledged'
        except (OSError, serial.SerialException) as exc:
            self.active = False
            response.success = False
            response.message = str(exc)
        return response

    def stop(self, request, response):
        self.active = False
        try:
            self.write(command(0))
            response.success = True
            response.message = 'STOP sent; throttle zero/coast requested, execution not acknowledged'
        except (OSError, serial.SerialException) as exc:
            response.success = False
            response.message = str(exc)
        return response

    def fail(self, reason):
        self.active = False
        self.fault = True
        try:
            self.write(command(0))
        except (OSError, serial.SerialException):
            pass
        self.get_logger().error(reason)

    def receive(self):
        try:
            frames = self.parser.feed(self.bus.read(4096))
            for identifier, data in frames:
                if identifier != 0x201:
                    continue
                try:
                    status = decode_status(data)
                except ValueError as exc:
                    self.fail(str(exc))
                    continue
                if 'error' in status:
                    self.fail(f'Teensy error: {status["error"]}')
                else:
                    self.last_status = time.monotonic()
                    self.fault = False
                status['active'] = self.active
                status['target_speed_kmh'] = self.target / 10
                self.publisher.publish(String(data=json.dumps(status)))
        except (OSError, serial.SerialException) as exc:
            self.fail(f'Serial receive failed: {exc}')

    def transmit(self):
        if self.active and not self.healthy():
            self.fail('Feedback timeout: stopped; explicit restart required')
        if self.target == 0:
            self.active = False
        try:
            self.write(command(1, self.target) if self.active else command(0))
        except (OSError, serial.SerialException) as exc:
            self.fail(f'Serial transmit failed: {exc}')

    def poll(self):
        try:
            self.write(command(2))
        except (OSError, serial.SerialException) as exc:
            self.fail(f'Status request failed: {exc}')

    def close(self):
        self.active = False
        try:
            self.write(command(0))
            time.sleep(0.02)
        except (OSError, serial.SerialException):
            pass
        finally:
            self.bus.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Master()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
