import glob
import pytest
import serial
import time


def _find_teensy_port():
    matches = glob.glob("/dev/serial/by-id/usb-Reiser_Lab_G6_Arena_*-if00")
    return matches[0] if matches else None


def pytest_addoption(parser):
    parser.addoption("--port", default=_find_teensy_port(), help="Teensy USB-CDC device node")


@pytest.fixture(scope="session")
def ser(pytestconfig):
    port = pytestconfig.getoption("--port")
    s = serial.Serial(port, baudrate=115200, timeout=0.1)
    time.sleep(0.3)
    s.reset_input_buffer()
    yield s
    s.close()
