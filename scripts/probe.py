"""Liveness probe for the G6-ArenaSlim controller.

Sends GET_ETHERNET_IP_ADDRESS (0xC1) over the G4-compatible binary protocol
and prints the full response. If this works, TCP framing, binary-command
parsing, and the response path are all healthy — so any subsequent failure
to light LEDs is downstream of `CommandProcessor::handleBinaryCommand()`.

    python probe.py
"""

import socket
import sys

IP_ADDRESS = "10.103.40.36"
PORT = 62222
GET_IP_CMD = 0xC1


def _printable(b):
    return "".join(chr(c) if 0x20 <= c < 0x7F else "." for c in b)


def main():
    try:
        sock = socket.create_connection((IP_ADDRESS, PORT), timeout=2.0)
    except OSError as e:
        print(f"connect to {IP_ADDRESS}:{PORT} failed: {e}", file=sys.stderr)
        sys.exit(1)

    with sock:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.sendall(bytes([0x01, GET_IP_CMD]))
        sock.settimeout(1.0)
        try:
            resp = sock.recv(256)
        except socket.timeout:
            resp = b""

    print(f"sent: 01 {GET_IP_CMD:02X}")
    print(f"recv ({len(resp)} bytes): {resp.hex(' ') or '<empty>'}")
    print(f"ascii: {_printable(resp)!r}")

    if not resp:
        print("EMPTY RESPONSE — controller likely never reached "
              "NetworkManager::sendResponse(). Suspect:", file=sys.stderr)
        print("  - firmware not running (no Morse 'OK' at boot?)", file=sys.stderr)
        print("  - Ethernet.begin() hung (DHCP timeout?)", file=sys.stderr)
        print("  - wrong IP or no link", file=sys.stderr)
        sys.exit(2)


if __name__ == "__main__":
    main()
