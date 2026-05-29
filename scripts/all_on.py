"""Light up every LED on the G6 arena for a few seconds, then go dark.

Standalone test script for the G6-ArenaSlim controller — uses only the
G4-compatible binary wire protocol (port 62222), so no arena_interface
package is needed. Run from any Python 3 environment with network access
to the arena.

    python all_on.py
"""

import socket
import time

IP_ADDRESS = "10.103.40.36"
PORT = 62222

ALL_OFF_CMD = 0x00
ALL_ON_CMD = 0xFF
ON_DURATION_S = 5.0


def _printable(b):
    return "".join(chr(c) if 0x20 <= c < 0x7F else "." for c in b)


def send_binary(sock, cmd_byte, label):
    # G4 binary framing: [length, cmd, params...]; length excludes itself.
    sock.sendall(bytes([0x01, cmd_byte]))
    # Read whatever the controller sends back so we can see the parsed-cmd echo.
    sock.settimeout(0.5)
    try:
        resp = sock.recv(256)
    except socket.timeout:
        resp = b""
    print(
        f"{label:>8} -> 0x{cmd_byte:02X}  resp={resp.hex(' ') or '<empty>'}  "
        f"ascii={_printable(resp)!r}"
    )


def main():
    with socket.create_connection((IP_ADDRESS, PORT), timeout=2.0) as sock:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        t0 = time.time()
        send_binary(sock, ALL_ON_CMD, "all_on")
        all_on_call_s = time.time() - t0
        print(f"all_on duration: {all_on_call_s:.6f} s")

        time.sleep(ON_DURATION_S)

        send_binary(sock, ALL_OFF_CMD, "all_off")


if __name__ == "__main__":
    main()
