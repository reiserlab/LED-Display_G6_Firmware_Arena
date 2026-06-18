"""Serial twin of all_on.py for CIPO bench testing.

Instead of TCP (port 62222) this drives the controller over the Teensy USB-CDC
link, which the firmware accepts via SerialManager using the SAME G4 binary
framing: [length, cmd, params...]. Because DEBUG_SERIAL routes the `[spi] CIPO`
diagnostic prints out the same bidirectional CDC pipe, this one process both
sends "all on" and captures the CIPO stream -- no separate `pio device monitor`
(which would fight for the exclusive port).

Requires a diagnostic arena build: the `[spi] CIPO` rows are gated on
-DDEBUG_SERIAL (env teensy41-printf; flash via `pixi run deploy-printf`). The
default `pixi run deploy` (env teensy41) compiles them out and nothing matches.

    pixi run python scripts/all_on_serial.py --port /dev/cu.usbmodem121699401 \
        --label A --duration 50

Reads the captured stream and logs it to log/<ts>_<label>.log as text: ASCII
diagnostics stay readable and any binary framing / command-echo bytes are
rendered as \\xNN hex escapes (not saved raw). Then prints a classification of
every `[spi] CIPO` row at the end:
    ?1 30 ??  -> panel echo (confirmation flowing)            [ECHO]
    81 00 00  -> empty-buffer sentinel (alive, slot empty)    [SENTINEL]
    00 00 00  -> line held low at the Teensy (blocked)        [BLOCKED]
"""

import argparse
import time

from cipo_common import (
    ALL_OFF_CMD,
    ALL_ON_CMD,
    CIPO_RE,
    classify,
    enable_diagnostics,
    open_cdc,
    safe_text,
    timestamped_log,
)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="Teensy USB-CDC device node")
    ap.add_argument("--label", default="run", help="A/B/C label for the log name")
    ap.add_argument("--duration", type=float, default=5.0, help="ON seconds")
    ap.add_argument("--logdir", default="log")
    args = ap.parse_args()

    logpath = timestamped_log(args.logdir, args.label)
    ser = open_cdc(args.port)
    enable_diagnostics(ser)  # undo any prior web-serial mute so CIPO rows flow

    rows = []  # (set, cs, b0,b1,b2 / B1..., class0, class1)
    buf = bytearray()
    deadline = time.time() + args.duration

    print(
        f"[{args.label}] all_on -> 0x{ALL_ON_CMD:02X} on {args.port}; "
        f"capturing {args.duration:.0f}s -> {logpath}"
    )
    ser.write(bytes([0x01, ALL_ON_CMD]))
    ser.flush()

    with open(logpath, "w", encoding="ascii") as raw:
        while time.time() < deadline:
            chunk = ser.read(4096)
            if not chunk:
                continue
            raw.write(safe_text(chunk))
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
    print(
        f"\n[{args.label}] captured {len(rows)} CIPO rows. Non-zero / interesting rows:"
    )
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
    print(
        f"\n[{args.label}] tally: "
        + "  ".join(f"{k}={v}" for k, v in sorted(tally.items()))
    )
    verdict = (
        "ECHO present -> confirmation flowing"
        if tally.get("ECHO")
        else "NO echo -> CIPO blocked/empty"
    )
    print(f"[{args.label}] verdict: {verdict}")
    print(f"[{args.label}] log: {logpath}")


if __name__ == "__main__":
    main()
