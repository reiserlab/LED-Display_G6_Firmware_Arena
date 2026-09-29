import time

import pytest
import serial

from arena_port import AmbiguousArenaPort, describe_usb_ports, find_arena_port


def pytest_addoption(parser):
    # Resolved lazily in the fixture: an eager default here would run port detection
    # while pytest registers options, before an explicit --port could rescue an
    # ambiguous bench (issue #61).
    parser.addoption("--port", default=None, help="Teensy USB-CDC device node (auto-detected when omitted)")


@pytest.fixture(scope="session")
def ser(pytestconfig):
    try:
        port = pytestconfig.getoption("--port") or find_arena_port()
    except AmbiguousArenaPort as e:
        pytest.fail(str(e))
    if not port:
        pytest.fail(
            "No G6 arena USB-CDC port found; pass --port <device>. "
            f"USB serial ports seen: {describe_usb_ports()}"
        )
    s = serial.Serial(port, baudrate=115200, timeout=0.1)
    time.sleep(0.3)
    s.reset_input_buffer()
    yield s
    s.close()
