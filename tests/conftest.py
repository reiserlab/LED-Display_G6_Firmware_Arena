"""Pytest configuration for the Arena-Firmware HIL test suite.

CLI options
-----------
--transport  serial | tcp   (default: serial)
--port       USB-CDC device node (auto-discovered if omitted)
--ip         controller IP address (required for --transport=tcp)

Markers
-------
serial_only  tests that only make sense over USB-CDC
tcp_only     tests that only make sense over TCP

The session-scoped `transport` fixture opens/closes the selected backend.
"""

import glob

import pytest

from .transport import SerialTransport, TcpTransport


def _find_teensy() -> str | None:
    matches = glob.glob("/dev/serial/by-id/usb-Reiser_Lab_G6_Arena_*-if00")
    return matches[0] if matches else None


def pytest_addoption(parser):
    parser.addoption(
        "--transport", default="serial", choices=["serial", "tcp"],
        help="Transport backend to use (default: serial)",
    )
    parser.addoption(
        "--port", default=None,
        help="Teensy USB-CDC device node (auto-detected if omitted)",
    )
    parser.addoption(
        "--ip", default=None,
        help="Controller IP address (required for --transport=tcp)",
    )
    parser.addoption(
        "--pat", default=None,
        help="Path to a .pat file for the T5 SD playback test in test_lab79_sd.py",
    )


def pytest_configure(config):
    config.addinivalue_line("markers", "serial_only: skip when TCP transport is selected")
    config.addinivalue_line("markers", "tcp_only: skip when serial transport is selected")


def pytest_collection_modifyitems(config, items):
    tr = config.getoption("--transport")
    for item in items:
        if "serial_only" in item.keywords and tr != "serial":
            item.add_marker(pytest.mark.skip(reason="--transport=serial not selected"))
        if "tcp_only" in item.keywords and tr != "tcp":
            item.add_marker(pytest.mark.skip(reason="--transport=tcp not selected"))


@pytest.fixture(scope="session")
def transport(pytestconfig):
    tr = pytestconfig.getoption("--transport")
    if tr == "serial":
        port = pytestconfig.getoption("--port") or _find_teensy()
        if not port:
            pytest.fail("No Teensy USB-CDC port found; pass --port /dev/...")
        t = SerialTransport(port)
    else:
        ip = pytestconfig.getoption("--ip")
        if not ip:
            pytest.fail("TCP transport requires --ip <controller-address>")
        t = TcpTransport(ip)
    t.open()
    yield t
    t.close()


@pytest.fixture(scope="session")
def pat_data(pytestconfig):
    """Raw bytes from the --pat file, or None if --pat was not supplied."""
    path = pytestconfig.getoption("--pat")
    if path is None:
        return None
    with open(path, "rb") as f:
        return f.read()
