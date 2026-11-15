"""Exercise actual supervisor callbacks without a local ROS installation."""
import ast
from pathlib import Path
from types import SimpleNamespace
import time

from motorcycle_control.protocol import command, speed_value


def master():
    tree = ast.parse((Path(__file__).parents[1] / 'motorcycle_control/node.py').read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    namespace = dict(Node=object, time=time, command=command, speed_value=speed_value,
                     serial=SimpleNamespace(SerialException=OSError))
    exec(compile(ast.Module(body=[cls], type_ignores=[]), 'node.py', 'exec'), namespace)
    node = namespace['Master'].__new__(namespace['Master'])
    node.active = False
    node.fault = False
    node.last_status = time.monotonic()
    node.timeout = 1.0
    node.target = 50
    node.maximum = 70
    node.sent = []
    node.write = node.sent.append
    node.get_logger = lambda: SimpleNamespace(error=lambda message: None)
    return node


def test_start_requires_fresh_feedback():
    node = master()
    node.last_status = None
    result = node.start(None, SimpleNamespace())
    assert not result.success and not node.active and not node.sent
    node.last_status = time.monotonic()
    assert node.start(None, SimpleNamespace()).success
    assert node.active and node.sent == [command(1, 50)]


def test_stop_cancels_periodic_speed():
    node = master()
    node.active = True
    assert node.stop(None, SimpleNamespace()).success
    node.transmit()
    assert not node.active and node.sent == [command(0), command(0)]


def test_timeout_does_not_auto_restart():
    node = master()
    node.active = True
    node.last_status -= 2
    node.transmit()
    assert not node.active and all(p == command(0) for p in node.sent)
    node.last_status = time.monotonic()
    node.fault = False
    node.transmit()
    assert not node.active and node.sent[-1] == command(0)


def test_write_failure_does_not_leave_active():
    node = master()
    def broken(packet):
        raise OSError('disconnected')
    node.write = broken
    assert not node.start(None, SimpleNamespace()).success
    assert not node.active
    node.active = True
    assert not node.stop(None, SimpleNamespace()).success
    assert not node.active


def test_zero_target_requires_new_start():
    node = master()
    node.active = True
    node.target = 0
    node.transmit()
    node.target = 50
    node.transmit()
    assert not node.active and node.sent == [command(0), command(0)]
