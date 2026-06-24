"""Query the G6-ArenaSlim controller's identity / capabilities.

Sends GET_CONTROLLER_INFO (0xC2) over the G4-compatible binary protocol
(port 62222) and decodes the {version, capability_bitmap} reply. Standalone
counterpart to all_on.py — no arena_interface package required.

    python controller_info.py [--ip 10.103.40.36]
"""

import argparse
import socket
import sys

PORT = 62222
GET_CONTROLLER_INFO_CMD = 0xC2

# Capability bitmap bits (g6_03-controller.md § 5).
CAPABILITY_BITS = [
    (0, "g6_mode"),
    (1, "v2_local_storage"),
    (2, "mode_1_tsi"),
    (3, "v3_triggered"),
    (4, "v3_gated"),
]


def _printable(b):
    return "".join(chr(c) if 0x20 <= c < 0x7F else "." for c in b)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", default="10.103.40.36", help="controller IP")
    args = parser.parse_args()

    try:
        sock = socket.create_connection((args.ip, PORT), timeout=2.0)
    except OSError as e:
        print(f"connect to {args.ip}:{PORT} failed: {e}", file=sys.stderr)
        sys.exit(1)

    with sock:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        # G4 binary framing: [length, cmd]; length excludes itself.
        sock.sendall(bytes([0x01, GET_CONTROLLER_INFO_CMD]))
        sock.settimeout(1.0)
        try:
            resp = sock.recv(256)
        except socket.timeout:
            resp = b""

    print(f"sent: 01 {GET_CONTROLLER_INFO_CMD:02X}")
    print(f"recv ({len(resp)} bytes): {resp.hex(' ') or '<empty>'}")
    print(f"ascii: {_printable(resp)!r}")

    # Response framing: [length, status, echo_cmd, version, capability].
    if len(resp) < 5:
        print("response too short — expected [len, status, echo, version, cap]",
              file=sys.stderr)
        sys.exit(2)

    status, echo_cmd, version, capability = resp[1], resp[2], resp[3], resp[4]
    if status != 0 or echo_cmd != GET_CONTROLLER_INFO_CMD:
        print(f"unexpected reply: status={status} echo=0x{echo_cmd:02X}",
              file=sys.stderr)
        sys.exit(2)

    enabled = [name for bit, name in CAPABILITY_BITS if capability & (1 << bit)]
    print(f"\ncontroller version : {version}")
    print(f"capability bitmap  : 0x{capability:02X}")
    print(f"capabilities       : {', '.join(enabled) or 'none'}")


if __name__ == "__main__":
    main()
