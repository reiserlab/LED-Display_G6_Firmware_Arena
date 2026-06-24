#!/usr/bin/env python3
"""LAB-42 host driver — make the arena emit the V2 display-from-PSRAM command.

Pairs with the LAB-41 panel firmware: the panel holds a 100-frame animation in
its own PSRAM (loaded locally at boot), and this script tells the arena to drive
it via the V2 panel-protocol command (header 0x02) — so the wire carries a tiny
frame-index command, not 201-byte pixel frames.

The arena master speaks the same binary framing as the rest of the host tooling:
send  [length, cmd, params...]   (length counts everything after itself)
recv  [length, status, echo_cmd, ...payload]

Examples:
    # Play the whole demo: indices 0..99 at 30 fps (default).
    ./psram_demo_drive.py

    # Show a single frame and hold it.
    ./psram_demo_drive.py --index 42

    # Custom auto-advance window.
    ./psram_demo_drive.py --play 40 40 20      # vertical-bar sweep at 20 fps

    # Stop / pin the port.
    ./psram_demo_drive.py --off
    ./psram_demo_drive.py --port /dev/cu.usbmodemXXXX --play 0 100 30

Requires the arena flashed with the LAB-42 build (branch
mreiser/lab-42-arena-v2-display-from-psram) and wired to a panel running a
production LAB-41 build (pico_v0NN, NOT the *_psramtest console build).
"""

import argparse
import sys
import time

# Host -> arena opcodes (src/commands.h).
ALL_OFF_CMD             = 0x00
SET_REFRESH_RATE_CMD    = 0x16
GET_CONTROLLER_INFO_CMD = 0x67
DISPLAY_PSRAM_INDEX_CMD = 0x71   # payload: u16 LE index
PSRAM_PLAY_CMD          = 0x72   # payload: start(2) count(2) fps(2), all u16 LE

MASTER_VID = 0x16C0   # Teensy


def u16le(n):
    return [n & 0xFF, (n >> 8) & 0xFF]


class Master:
    """Arena master CDC: binary framing [length, cmd, params...]."""

    def __init__(self, port):
        import serial
        self.s = serial.Serial(port, 115200, timeout=2.0)
        time.sleep(0.3)
        self.s.reset_input_buffer()
        self.port = port

    def _read_exact(self, n):
        buf = b""
        while len(buf) < n:
            chunk = self.s.read(n - len(buf))
            if not chunk:
                break
            buf += chunk
        return buf

    def command(self, frame):
        self.s.write(bytes(frame))
        self.s.flush()
        hdr = self._read_exact(1)
        if not hdr:
            return None
        body = self._read_exact(hdr[0])
        if len(body) < 2:
            return None
        return body[0], body[1], body[2:]   # status, echo_cmd, payload

    def display_index(self, index):
        # [len=3, cmd, idx_lo, idx_hi]
        return self.command([0x03, DISPLAY_PSRAM_INDEX_CMD, *u16le(index)])

    def play(self, start, count, fps):
        # [len=7, cmd, start(2), count(2), fps(2)]
        return self.command([0x07, PSRAM_PLAY_CMD,
                             *u16le(start), *u16le(count), *u16le(fps)])

    def controller_info(self):
        return self.command([0x01, GET_CONTROLLER_INFO_CMD])

    def all_off(self):
        return self.command([0x01, ALL_OFF_CMD])

    def close(self):
        self.s.close()


def autodetect(vid):
    from serial.tools import list_ports
    return [p.device for p in list_ports.comports() if p.vid == vid]


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="arena master CDC (default: autodetect VID 0x16C0)")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--index", type=int, help="show a single PSRAM frame index and hold")
    g.add_argument("--play", nargs=3, type=int, metavar=("START", "COUNT", "FPS"),
                   help="auto-advance [START, START+COUNT) at FPS")
    g.add_argument("--off", action="store_true", help="ALL_OFF and exit")
    ap.add_argument("--info", action="store_true",
                    help="print get-controller-info (expect capability bit1 = v2_local_storage)")
    args = ap.parse_args()

    port = args.port
    if not port:
        found = autodetect(MASTER_VID)
        if len(found) != 1:
            print(f"arena autodetect found {found}; pass --port", file=sys.stderr)
            sys.exit(1)
        port = found[0]

    m = Master(port)
    print(f"# arena : {port}")
    try:
        if args.info:
            r = m.controller_info()
            if r and len(r[2]) >= 2:
                print(f"controller-info: version={r[2][0]} capability=0x{r[2][1]:02X}"
                      f"  (v2_local_storage={'yes' if r[2][1] & 0x02 else 'no'})")
            else:
                print(f"controller-info: {r}")
            return

        if args.off:
            print("ALL_OFF:", m.all_off())
            return

        if args.index is not None:
            print(f"DISPLAY_PSRAM index={args.index}:", m.display_index(args.index))
            return

        start, count, fps = args.play if args.play else (0, 100, 30)
        print(f"PSRAM_PLAY start={start} count={count} fps={fps}:",
              m.play(start, count, fps))
        print("# arena is now auto-advancing the PSRAM animation; "
              "run with --off to stop.")
    finally:
        m.close()


if __name__ == "__main__":
    main()
