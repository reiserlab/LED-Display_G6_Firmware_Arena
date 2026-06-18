"""Test SET_DIAG_OUTPUT_CMD (0xC3) — g_dbg_on flag behavior.

Sends commands to the running firmware over USB-CDC and checks responses.
Requires firmware built with -DDEBUG_SERIAL (teensy41-printf env).

Usage:
    pixi run test-diag [-- --port /dev/ttyACM0]
"""

import time

ALL_OFF_CMD         = 0x00
SET_DIAG_OUTPUT_CMD = 0xC3
DIAG_SENTINEL       = 0xFF


# ── Serial framing ─────────────────────────────────────────────────────────────

def _send(ser, *payload):
    ser.write(bytes(payload))
    ser.flush()


def _read_frame(ser, timeout=1.0):
    """Read one response frame, collecting any diagnostic lines encountered.

    Wire layout:
      Diagnostic line:  0xFF <ascii text> \\n
      Response frame:   [length] [status] [echo_cmd] [payload...]

    Diagnostic lines come BEFORE the response (DBG_PRINTF fires inside
    handleBinaryCommand, sendResponse fires at the end of the case branch).

    Returns (status, echo, payload_str, diag_lines).
    Raises RuntimeError on timeout.
    """
    deadline = time.monotonic() + timeout
    buf = bytearray()
    diag_lines = []

    while time.monotonic() < deadline:
        chunk = ser.read(max(1, ser.in_waiting))
        if chunk:
            buf.extend(chunk)

        i = 0
        while i < len(buf):
            if buf[i] == DIAG_SENTINEL:
                nl = buf.find(b"\n", i)
                if nl == -1:
                    break                        # incomplete diag line — wait
                line = buf[i + 1 : nl].decode("ascii", errors="replace").strip()
                diag_lines.append(line)
                i = nl + 1
            else:
                length = buf[i]
                if i + 1 + length > len(buf):
                    break                        # frame not complete yet
                frame   = buf[i + 1 : i + 1 + length]
                status  = frame[0] if len(frame) > 0 else -1
                echo    = frame[1] if len(frame) > 1 else -1
                payload = frame[2:].decode("ascii", errors="replace") if len(frame) > 2 else ""
                return status, echo, payload, diag_lines

        buf = buf[i:]

    raise RuntimeError(f"response timeout (diag so far: {diag_lines!r})")


def _set_diag(ser, on_byte=None):
    if on_byte is None:
        _send(ser, 0x01, SET_DIAG_OUTPUT_CMD)
    else:
        _send(ser, 0x02, SET_DIAG_OUTPUT_CMD, on_byte)
    return _read_frame(ser)


# ── Tests ──────────────────────────────────────────────────────────────────────

def test_arg_zero_turns_off(ser):
    st, echo, pay, _ = _set_diag(ser, 0x00)
    assert st == 0
    assert echo == SET_DIAG_OUTPUT_CMD
    assert pay == "diag off"


def test_arg_one_turns_on(ser):
    _set_diag(ser, 0x00)
    st, echo, pay, _ = _set_diag(ser, 0x01)
    assert st == 0
    assert echo == SET_DIAG_OUTPUT_CMD
    assert pay == "diag on"


def test_nonzero_arg_turns_on(ser):
    _set_diag(ser, 0x00)
    st, echo, pay, _ = _set_diag(ser, 0xFF)
    assert st == 0
    assert pay == "diag on"


def test_missing_arg_defaults_on(ser):
    _set_diag(ser, 0x00)
    st, echo, pay, _ = _set_diag(ser, None)
    assert st == 0
    assert pay == "diag on"


def test_diag_lines_emitted_when_on(ser):
    _set_diag(ser, 0x01)
    _send(ser, 0x01, ALL_OFF_CMD)
    _, _, _, diag = _read_frame(ser)
    assert any("[cmd]" in d for d in diag), f"no [cmd] line in {diag!r}"


def test_no_diag_lines_when_off(ser):
    _set_diag(ser, 0x00)
    _send(ser, 0x01, ALL_OFF_CMD)
    _, _, _, diag = _read_frame(ser)
    assert not diag, f"unexpected lines: {diag!r}"


def test_all_off_preserves_diag_state(ser):
    _set_diag(ser, 0x00)
    _send(ser, 0x01, ALL_OFF_CMD)
    _read_frame(ser)
    _send(ser, 0x01, ALL_OFF_CMD)
    _, _, _, diag = _read_frame(ser)
    assert not diag, f"diag leaked on: {diag!r}"
