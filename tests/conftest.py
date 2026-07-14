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
        "--transport",
        default="serial",
        choices=["serial", "tcp"],
        help="Transport backend to use (default: serial)",
    )
    parser.addoption(
        "--port",
        default=None,
        help="Teensy USB-CDC device node (auto-detected if omitted)",
    )
    parser.addoption(
        "--ip",
        default=None,
        help="Controller IP address (required for --transport=tcp)",
    )
    parser.addoption(
        "--pat",
        default=None,
        help="Path to a .pat file for the T5 SD playback test in test_lab79_sd.py",
    )
    parser.addoption(
        "--visual",
        action="store_true",
        default=False,
        help="Run opt-in guided visual tests (need a human observing the arena; "
             "pair with -s so prompts/output are shown).",
    )
    parser.addoption(
        "--ad3",
        action="store_true",
        default=False,
        help="Run opt-in AD3-instrumented tests (need a Digilent Analog "
             "Discovery 3 wired per the test's docstring; run under the "
             "debugad3 pixi environment, e.g. `pixi run -e debugad3 "
             "test-serial-ad3`).",
    )


def pytest_configure(config):
    config.addinivalue_line(
        "markers", "serial_only: skip when TCP transport is selected"
    )
    config.addinivalue_line(
        "markers", "tcp_only: skip when serial transport is selected"
    )
    config.addinivalue_line(
        "markers", "visual: opt-in guided visual test (needs a human + --visual)"
    )
    config.addinivalue_line(
        "markers", "ad3: opt-in AD3-instrumented test (needs an AD3 + --ad3)"
    )


def pytest_collection_modifyitems(config, items):
    tr = config.getoption("--transport")
    for item in items:
        if "serial_only" in item.keywords and tr != "serial":
            item.add_marker(pytest.mark.skip(reason="--transport=serial not selected"))
        if "tcp_only" in item.keywords and tr != "tcp":
            item.add_marker(pytest.mark.skip(reason="--transport=tcp not selected"))
        if "visual" in item.keywords and not config.getoption("--visual"):
            item.add_marker(pytest.mark.skip(
                reason="guided visual test — pass --visual (with -s) to run"))
        if "ad3" in item.keywords and not config.getoption("--ad3"):
            item.add_marker(pytest.mark.skip(
                reason="AD3-instrumented test — pass --ad3 to run"))


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


@pytest.fixture(scope="session")
def pat(transport, pat_data):
    """Upload --pat to the SD card and return its 1-based pattern index.

    Skips the test if --pat was not supplied.  Uploads once per session and
    leaves the file on the SD card (named 'conftest.pat').
    """
    if pat_data is None:
        pytest.skip("--pat not provided; pass a .pat file path to run this test")
    st, _, _, _ = transport.upload_file(0, pat_data, timeout=120.0)
    assert st == 0, "upload of --pat file failed"
    st2, _, payload, _ = transport.rename_file(0, "conftest.pat")
    assert st2 == 0, "rename of --pat file failed"
    import struct
    idx = struct.unpack_from("<H", bytes(payload[:2]))[0]
    return idx
