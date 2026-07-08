"""Drive SD-backed pattern playback on the G6-ArenaSlim controller.

Exercises the v1 display modes that load `.pat` files from the controller's
SD card, using the G4-compatible binary protocol (port 62222):

  * trial-params (0x08) — select mode + pattern + timing
  * set-frame-position (0x70) — jump to a frame (Mode 3)

Standalone counterpart to all_on.py — no arena_interface package required.

Examples:
    # Mode 2 (open loop): play pattern 1 at 30 fps from frame 0
    python play_pattern.py --mode 2 --pattern 1 --rate 30

    # Mode 3 (show frame): select pattern 2, then show frame 10
    python play_pattern.py --mode 3 --pattern 2 --frame 10

    # Mode 4 (closed loop): pattern 1, gain -20 (= -2.0 fps/V)
    python play_pattern.py --mode 4 --pattern 1 --gain -20
"""

import argparse
import socket
import sys

PORT = 62222
TRIAL_PARAMS_CMD = 0x08
SET_FRAME_POSITION_CMD = 0x70


def _printable(b):
    return "".join(chr(c) if 0x20 <= c < 0x7F else "." for c in b)


def _u16le(n):
    return [n & 0xFF, (n >> 8) & 0xFF]


def send(sock, label, payload):
    sock.sendall(bytes(payload))
    sock.settimeout(1.0)
    try:
        resp = sock.recv(256)
    except socket.timeout:
        resp = b""
    status = resp[1] if len(resp) >= 2 else None
    print(
        f"{label:>16} -> {bytes(payload).hex(' ')}\n"
        f"{'':>16}    <- {resp.hex(' ') or '<empty>'}  "
        f"status={status}  ascii={_printable(resp[3:])!r}"
    )
    return resp


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--ip", default="10.103.40.36", help="controller IP")
    parser.add_argument("--mode", type=int, choices=(2, 3, 4), default=2,
                        help="display mode: 2=open loop, 3=show frame, 4=closed loop")
    parser.add_argument("--pattern", type=int, default=1,
                        help="1-based pattern ID (Nth .pat in /patterns)")
    parser.add_argument("--rate", type=int, default=0,
                        help="frame-advance rate in Hz (Mode 2)")
    parser.add_argument("--gain", type=int, default=0,
                        help="signed velocity gain, 10x fps/V (Mode 4)")
    parser.add_argument("--init", type=int, default=0,
                        help="initial frame index (0-based)")
    parser.add_argument("--frame", type=int, default=None,
                        help="after trial-params, send set-frame-position to this index")
    args = parser.parse_args()

    if not -32768 <= args.gain <= 32767:
        parser.error("--gain must fit in a signed int16 (-32768..32767)")

    # trial-params: [len=0x0c, 0x08, mode, pat(LE16), rate(LE16), init(LE16),
    # gain(LE16), duration(LE16)]. duration is in 10 ms ticks; 0 = no
    # controller-run auto-stop (GH #4 canonical re-layout).
    params = (
        [args.mode & 0xFF]
        + _u16le(args.pattern)
        + _u16le(args.rate)
        + _u16le(args.init)
        + _u16le(args.gain)
        + [0, 0]
    )
    trial = [0x0C, TRIAL_PARAMS_CMD] + params

    try:
        sock = socket.create_connection((args.ip, PORT), timeout=2.0)
    except OSError as e:
        print(f"connect to {args.ip}:{PORT} failed: {e}", file=sys.stderr)
        sys.exit(1)

    with sock:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        resp = send(sock, "trial-params", trial)
        if len(resp) >= 2 and resp[1] != 0:
            print("trial-params rejected — check SD card / pattern ID "
                  "(controller shows a CE glyph on error).", file=sys.stderr)

        if args.frame is not None:
            # set-frame-position: [len=3, 0x70, lo, hi]
            sfp = [0x03, SET_FRAME_POSITION_CMD] + _u16le(args.frame)
            send(sock, "set-frame-pos", sfp)


if __name__ == "__main__":
    main()
