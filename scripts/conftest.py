import time

import pytest
import serial

from arena_port import find_arena_port


def pytest_addoption(parser):
    parser.addoption(
        "--port", default=find_arena_port(), help="Teensy USB-CDC device node"
    )


@pytest.fixture(scope="session")
def ser(pytestconfig):
    port = pytestconfig.getoption("--port")
    s = serial.Serial(port, baudrate=115200, timeout=0.1)
    time.sleep(0.3)
    s.reset_input_buffer()
    yield s
    s.close()
