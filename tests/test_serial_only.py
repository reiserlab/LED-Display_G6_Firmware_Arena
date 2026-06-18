"""USB-CDC-specific tests.

Skipped automatically when --transport=tcp is selected.
"""

import pytest

from .commands import ALL_OFF_CMD, SET_DIAG_OUTPUT_CMD

pytestmark = pytest.mark.serial_only


def test_settle_not_confused(transport):
    """First command after CDC open responds correctly (settle done in fixture)."""
    st, echo, _, _ = transport.command(ALL_OFF_CMD)
    assert st == 0
    assert echo == ALL_OFF_CMD


def test_diag_lines_emitted_when_enabled(transport):
    transport.command(SET_DIAG_OUTPUT_CMD, bytes([0x01]))
    _, _, _, diag = transport.command(ALL_OFF_CMD)
    assert any("[cmd]" in d for d in diag), f"no [cmd] diag line in: {diag!r}"
    transport.command(SET_DIAG_OUTPUT_CMD, bytes([0x00]))  # restore


def test_diag_off_no_lines(transport):
    transport.command(SET_DIAG_OUTPUT_CMD, bytes([0x00]))
    _, _, _, diag = transport.command(ALL_OFF_CMD)
    assert not diag, f"unexpected diag lines: {diag!r}"
