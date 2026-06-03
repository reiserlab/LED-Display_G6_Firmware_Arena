"""Serial twin of all_on.py for CIPO bench testing.

Instead of TCP (port 62222) this drives the controller over the Teensy USB-CDC
link, which the firmware accepts via SerialManager using the SAME G4 binary
framing: [length, cmd, params...]. Because DEBUG_SERIAL routes the `[spi] CIPO`
diagnostic prints out the same bidirectional CDC pipe, this one process both
sends "all on" and captures the CIPO stream -- no separate `pio device monitor`
(which would fight for the exclusive port).

    pixi run python scripts/all_on_serial.py --port /dev/cu.usbmodem121699401 \
        --label A --duration 50

Reads the captured stream, tees it to log/<ts>_<label>.log, and prints a
classification of every `[spi] CIPO` row at the end:
    ?1 30 ??  -> panel echo (confirmation flowing)            [ECHO]
    81 00 00  -> empty-buffer sentinel (alive, slot empty)    [SENTINEL]
    00 00 00  -> line held low at the Teensy (blocked)        [BLOCKED]
"""

import argparse
import datetime as _dt
import os
import re
import sys
import time

try:
    import serial  # pyserial, bundled with the platformio pixi env
except ImportError:
    sys.exit("pyserial not found -- run via `pixi run python ...`")

ALL_OFF_CMD = 0x00
ALL_ON_CMD = 0xFF

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
    if b0[-1] in "1" and b1.lower() == "30":
        return "ECHO"
    return "OTHER"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="Teensy USB-CDC device node")
    ap.add_argument("--label", default="run", help="A/B/C label for the log name")
    ap.add_argument("--duration", type=float, default=50.0, help="ON seconds")
    ap.add_argument("--logdir", default="log")
    args = ap.parse_args()

    os.makedirs(args.logdir, exist_ok=True)
    ts = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    logpath = os.path.join(args.logdir, f"{ts}_{args.label}.log")

    # Teensy CDC ignores the baud rate; opening does not reset the board.
    ser = serial.Serial(args.port, baudrate=115200, timeout=0.2)
    time.sleep(0.3)
    ser.reset_input_buffer()

    rows = []                       # (set, cs, b0,b1,b2 / B1..., class0, class1)
    buf = bytearray()
    deadline = time.time() + args.duration

    print(f"[{args.label}] all_on -> 0x{ALL_ON_CMD:02X} on {args.port}; "
          f"capturing {args.duration:.0f}s -> {logpath}")
    ser.write(bytes([0x01, ALL_ON_CMD]))
    ser.flush()

    with open(logpath, "wb") as raw:
        while time.time() < deadline:
            chunk = ser.read(4096)
            if not chunk:
                continue
            raw.write(chunk)
            raw.flush()
            buf += chunk
            # Decode complete text lines; binary cmd-echo bytes are tolerated
            # (they just produce the odd undecodable line, which we skip).
            while b"\n" in buf:
                line, _, buf = buf.partition(b"\n")
                text = line.decode("ascii", "replace").rstrip("\r")
                m = CIPO_RE.search(text)
                if m:
                    setn, cs = m.group(1), m.group(2)
                    b0 = (m.group(3), m.group(4), m.group(5))
                    b1 = (m.group(6), m.group(7), m.group(8))
                    rows.append((setn, cs, b0, classify(*b0), b1, classify(*b1)))

    ser.write(bytes([0x01, ALL_OFF_CMD]))
    ser.flush()
    time.sleep(0.2)
    ser.close()

    # ---- summary ----
    print(f"\n[{args.label}] captured {len(rows)} CIPO rows. "
          f"Non-zero / interesting rows:")
    tally = {}
    nonzero = []
    for setn, cs, b0, c0, b1, c1 in rows:
        for slot, (bb, cc) in (("B0", (b0, c0)), ("B1", (b1, c1))):
            tally[cc] = tally.get(cc, 0) + 1
            if cc not in ("BLOCKED",):
                nonzero.append(
                    f"  set{setn:<2} cs={cs:<3} {slot}={bb[0]} {bb[1]} {bb[2]}  [{cc}]"
                )
    # de-dup consecutive identical lines for readability
    seen = None
    for ln in nonzero:
        if ln != seen:
            print(ln)
            seen = ln
    print(f"\n[{args.label}] tally: " +
          "  ".join(f"{k}={v}" for k, v in sorted(tally.items())))
    verdict = ("ECHO present -> confirmation flowing"
               if tally.get("ECHO") else
               "NO echo -> CIPO blocked/empty")
    print(f"[{args.label}] verdict: {verdict}")
    print(f"[{args.label}] raw log: {logpath}")


if __name__ == "__main__":
    main()
