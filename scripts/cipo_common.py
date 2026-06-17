"""Shared helpers for the USB-CDC CIPO bench-test scripts.

Used by all_on_serial.py and multi_port_capture.py. Both drive the G6-ArenaSlim
controller over the Teensy USB-CDC link using G4 binary framing and parse the
`[spi] CIPO` diagnostic rows the firmware prints back out the same pipe.

pyserial ships with the platformio pixi env, so these run via `pixi run python
scripts/<tool>.py ...`.
"""

import datetime as _dt
import os
import re
import sys
import time

try:
    import serial  # pyserial, bundled with the platformio pixi env
except ImportError:
    sys.exit("pyserial not found -- run via `pixi run python ...`")

# Frame command bytes (G4 binary framing: [length, cmd, params...]).
ALL_OFF_CMD = 0x00
ALL_ON_CMD = 0xFF
# Aliases kept for the multi-port caller's shorter names.
ALL_OFF = ALL_OFF_CMD
ALL_ON = ALL_ON_CMD

# DEBUG_SERIAL diagnostics toggle (SET_DIAG_OUTPUT, 0xC3). The firmware flag
# persists across USB reconnects, so a prior web-serial session may have muted
# it -- capture tools re-enable it explicitly. Each diagnostic line the firmware
# emits is prefixed with DIAG_SENTINEL (0xFF); harmless here, since CIPO_RE
# searches within the line and still matches past a leading byte.
SET_DIAG_OUTPUT_CMD = 0xC3
DIAG_SENTINEL = 0xFF

# A CIPO dump row, e.g.:  [spi] CIPO set4  cs=9  B0=?1 30 62  B1=00 00 00
CIPO_RE = re.compile(
    r"\[spi\]\s*CIPO\s*set(\d+)\s+cs=(\S+)\s+"
    r"B0=([0-9A-Fa-f?]{2})\s+([0-9A-Fa-f?]{2})\s+([0-9A-Fa-f?]{2})\s+"
    r"B1=([0-9A-Fa-f?]{2})\s+([0-9A-Fa-f?]{2})\s+([0-9A-Fa-f?]{2})"
)


def classify(b0, b1, b2):
    """Classify a 3-byte CIPO triple (strings, may contain '?')."""
    trip = f"{b0} {b1} {b2}".lower()
    if trip == "00 00 00":
        return "BLOCKED"
    if trip == "81 00 00":
        return "SENTINEL"
    # Panel echo: header low-nibble = 1 (??1) and cmd byte = 0x30 (GRAY_16).
    if b0[-1] == "1" and b1.lower() == "30":
        return "ECHO"
    return "OTHER"


def safe_text(data):
    r"""Render raw CDC bytes as ASCII text for the log.

    Printable bytes and \t \r \n are kept literal — so the diagnostic rows
    stay readable and greppable, and line structure is preserved. Every other
    byte (binary framing / command-echo) is shown as \xNN instead of writing a
    raw byte that turns the log into mojibake. Stateless and per-byte, so it is
    safe to call on each streamed chunk independently.
    """
    out = []
    for b in data:
        if b in (0x09, 0x0A, 0x0D) or 0x20 <= b < 0x7F:
            out.append(chr(b))
        else:
            out.append(f"\\x{b:02x}")
    return "".join(out)


def enable_diagnostics(ser):
    """Re-enable DEBUG_SERIAL diagnostics on the controller (idempotent).

    Sends SET_DIAG_OUTPUT with on=1 so a prior web-serial mute can't leave the
    CIPO stream silent. No-op on a non-DEBUG firmware build (just acked).
    """
    ser.write(bytes([0x02, SET_DIAG_OUTPUT_CMD, 0x01]))
    ser.flush()


def open_cdc(port, baud=115200, timeout=0.2, settle=0.3):
    """Open a Teensy USB-CDC port, let it settle, and flush stale input.

    The CDC link ignores the baud rate and opening does not reset the board.
    """
    ser = serial.Serial(port, baudrate=baud, timeout=timeout)
    time.sleep(settle)
    ser.reset_input_buffer()
    return ser


def timestamped_log(logdir, *parts):
    """Return logdir/<YYYYmmdd_HHMMSS>_<parts...>.log, creating logdir."""
    os.makedirs(logdir, exist_ok=True)
    ts = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    name = "_".join((ts, *parts)) + ".log"
    return os.path.join(logdir, name)
