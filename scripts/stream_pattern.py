"""Stream a .pat file to the G6 arena over the wire protocol and display it.

Parses a v2 G6PT pattern file (18-byte header + per-frame "FR"-prefixed panel
blocks — see docs/development/g6_04-pattern-file-format.md) and pushes each
frame straight to the controller via STREAM_FRAME (0x32); no SD upload
needed. Panel blocks are forwarded byte-for-byte from the file (they're
already fully formatted: header/parity + cmd + pixel data + duty_cycle), so
the arena firmware just relays them to SPI unmodified.

Standalone script — only needs the G4-compatible binary wire protocol; no
arena_interface package required. Talks over either transport the firmware
accepts, same framing on both:
  * TCP, port 62222 (default) — see all_on.py / play_pattern.py
  * Teensy USB-CDC serial (--port) — see all_on_serial.py / cipo_common.py

Examples:
    # Loop the default pattern forever at 20 fps until Ctrl-C, over TCP
    python stream_pattern.py

    # Same, over serial
    python stream_pattern.py --port /dev/cu.usbmodem121699401

    # Play a specific pattern twice at 10 fps against a given controller IP
    python stream_pattern.py my_pattern.pat --ip 10.0.0.5 --fps 10 --loops 2
"""

import argparse
import socket
import struct
import sys
import time
from pathlib import Path

try:
    import serial  # pyserial, bundled with the platformio pixi env
except ImportError:
    serial = None

TCP_PORT = 62222
IP_ADDRESS = "10.103.40.36"
SERIAL_BAUD = 115200  # CDC link ignores the actual rate; opening does not reset the board
DIAG_SENTINEL = 0xFF  # DEBUG_SERIAL diagnostic-line marker (SerialManager.h)

DEFAULT_PATTERN = (
    Path(__file__).resolve().parent.parent
    / "debug" / "G6_4x10_4x10_grating_rotation_20px_50pct.pat"
)

STREAM_FRAME_CMD = 0x32
STOP_DISPLAY_CMD = 0x30
ALL_OFF_CMD = 0x00
SET_PANEL_DISPLAY_MODE_CMD = 0x1B
MODE_PERSIST = 1

HEADER_BYTE_COUNT = 18
FORMAT_VERSION = 2           # high nibble of header byte 4 (src/constants.h pattern_format_version)
FRAME_PREFIX_BYTE_COUNT = 4  # "FR" + uint16 LE frame index
FRAME_CRC_BYTE_COUNT = 2     # CRC-16/CCITT trailer — on-disk only, not sent on the wire


def crc8_autosar(data):
    """CRC-8/AUTOSAR: poly 0x2F, init 0xFF, refin=false, refout=false, xorout 0xFF.
    Mirrors G6::crc8_autosar() in src/Crc.h -- used for the pattern-file header."""
    crc = 0xFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x2F) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc ^ 0xFF


def crc16_ccitt_false(data):
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, refin=false, refout=false,
    xorout 0x0000. Mirrors G6::crc16_ccitt_false() in src/Crc.h -- used for the
    per-frame trailer."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class PatternFile:
    """Parses a v2 G6PT pattern file (docs/development/g6_04-pattern-file-format.md).

    STREAM_FRAME bypasses the firmware's own SD-read validation (SdManager::
    validateHeaderBytes / readFrame), which checks the header CRC-8 and each
    frame's CRC-16 before ever displaying it -- so this constructor repeats
    those same checks host-side. Without them a corrupted/truncated file that
    still happens to match the expected size would stream straight to SPI as
    garbage, which looks just like a wiring/hardware fault on the arena.
    """

    def __init__(self, path):
        data = Path(path).read_bytes()
        if data[:4] != b"G6PT":
            raise ValueError(f"{path}: not a G6PT pattern (bad magic {data[:4]!r})")
        version = (data[4] >> 4) & 0x0F
        if version != FORMAT_VERSION:
            raise ValueError(
                f"{path}: header format version {version} != expected {FORMAT_VERSION}"
            )
        header_crc = data[HEADER_BYTE_COUNT - 1]
        computed_crc = crc8_autosar(data[: HEADER_BYTE_COUNT - 1])
        if computed_crc != header_crc:
            raise ValueError(
                f"{path}: header CRC-8 mismatch (got 0x{header_crc:02X}, "
                f"computed 0x{computed_crc:02X}) -- header is corrupted"
            )
        self.frame_count = struct.unpack_from("<H", data, 6)[0]
        self.row_count = data[8]
        self.col_count = data[9]
        self.gs_val = data[10]
        self.num_panels = self.row_count * self.col_count
        try:
            self.block_byte_count = {1: 53, 2: 203}[self.gs_val]
        except KeyError:
            raise ValueError(
                f"{path}: unsupported gs_val {self.gs_val} (expected 1=GS2 or 2=GS16)"
            )
        self.frame_byte_count = (
            FRAME_PREFIX_BYTE_COUNT
            + self.num_panels * self.block_byte_count
            + FRAME_CRC_BYTE_COUNT
        )

        expected = HEADER_BYTE_COUNT + self.frame_count * self.frame_byte_count
        if len(data) != expected:
            raise ValueError(
                f"{path}: size {len(data)} != expected {expected} "
                f"(frame_count={self.frame_count}, {self.row_count}x{self.col_count}, "
                f"gs_val={self.gs_val})"
            )
        self._data = data
        self._body_byte_count = (
            FRAME_PREFIX_BYTE_COUNT + self.num_panels * self.block_byte_count
        )
        self._validate_frame_crcs(path)

    def _validate_frame_crcs(self, path):
        """Check every frame's 'FR' magic and CRC-16/CCITT-FALSE trailer up
        front (once, at load time) rather than per frame_payload() call, since
        frame_payload() is called repeatedly while a pattern loops."""
        for i in range(self.frame_count):
            start = HEADER_BYTE_COUNT + i * self.frame_byte_count
            end = start + self._body_byte_count
            body = self._data[start:end]
            if body[0:2] != b"FR":
                raise ValueError(f"{path}: frame {i}: bad frame magic {body[0:2]!r}")
            want = struct.unpack_from("<H", self._data, end)[0]
            got = crc16_ccitt_false(body)
            if got != want:
                raise ValueError(
                    f"{path}: frame {i}: CRC-16 mismatch (got 0x{got:04X}, "
                    f"expected 0x{want:04X}) -- frame data is corrupted"
                )

    def frame_payload(self, i):
        """One frame's on-wire STREAM_FRAME payload: the 4-byte 'FR'+index
        prefix followed by all panel blocks, verbatim from the file (the
        trailing CRC-16 is an on-disk integrity check, not part of the wire
        message, so it's excluded here)."""
        start = HEADER_BYTE_COUNT + i * self.frame_byte_count
        return self._data[start : start + self._body_byte_count]


# ---------------------------------------------------------------------------
# Transport: TCP or Teensy USB-CDC serial, same G4 binary framing on both --
# command [length, cmd, params...], response [length, status, echo, ...].
# STREAM_FRAME (0x32) is the one opcode-first exception, not length-prefixed.
# ---------------------------------------------------------------------------
class Transport:
    def send(self, data):
        raise NotImplementedError

    def _read_chunk(self):
        raise NotImplementedError

    def close(self):
        pass

    def recv_status(self, timeout=2.0):
        """Accumulate bytes until a full [length, status, echo, ...] response
        frame is present, skipping any leading DEBUG_SERIAL diagnostic lines
        (each prefixed with DIAG_SENTINEL and newline-terminated). Returns the
        status byte, or None if nothing complete arrived in time."""
        deadline = time.monotonic() + timeout
        buf = bytearray()
        while time.monotonic() < deadline:
            chunk = self._read_chunk()
            if chunk:
                buf.extend(chunk)
            i = 0
            while i < len(buf):
                if buf[i] == DIAG_SENTINEL:
                    nl = buf.find(b"\n", i)
                    if nl == -1:
                        break  # incomplete diag line -- wait for more
                    i = nl + 1
                    continue
                length = buf[i]
                if i + 1 + length > len(buf):
                    break  # incomplete frame -- wait for more
                frame = buf[i + 1:i + 1 + length]
                return frame[0] if frame else None
            buf = buf[i:]
        return None


class TcpTransport(Transport):
    def __init__(self, ip, port=TCP_PORT):
        self._sock = socket.create_connection((ip, port), timeout=2.0)
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def send(self, data):
        self._sock.sendall(data)

    def _read_chunk(self):
        self._sock.settimeout(0.2)
        try:
            return self._sock.recv(4096)
        except socket.timeout:
            return b""

    def close(self):
        self._sock.close()


class SerialTransport(Transport):
    """Teensy USB-CDC link. Mirrors cipo_common.open_cdc()."""

    def __init__(self, port, baud=SERIAL_BAUD, settle=0.3):
        if serial is None:
            sys.exit("pyserial not found -- run via `pixi run python ...`")
        self._ser = serial.Serial(port, baudrate=baud, timeout=0.2)
        time.sleep(settle)
        self._ser.reset_input_buffer()

    def send(self, data):
        self._ser.write(data)
        self._ser.flush()

    def _read_chunk(self):
        return self._ser.read(max(1, self._ser.in_waiting))

    def close(self):
        self._ser.close()


def send_command(transport, label, body, timeout=1.0):
    transport.send(bytes(body))
    status = transport.recv_status(timeout)
    print(f"{label:>16} -> status={status}")
    return status


def send_frame(transport, payload, timeout=2.0):
    """STREAM_FRAME (0x32) wire form: opcode-first, not length-prefixed like
    standard commands — [0x32, len_lo, len_hi, prefix, panel_blocks...]."""
    n = len(payload)
    transport.send(bytes([STREAM_FRAME_CMD, n & 0xFF, (n >> 8) & 0xFF]) + payload)
    return transport.recv_status(timeout)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("pattern", nargs="?", default=str(DEFAULT_PATTERN),
                        help="path to a .pat (G6PT v2) file")
    parser.add_argument("--ip", default=IP_ADDRESS,
                        help="controller IP (TCP transport, default unless --port given)")
    parser.add_argument("--port", default=None,
                        help="Teensy USB-CDC device node -- use serial transport instead of TCP")
    parser.add_argument("--fps", type=float, default=20.0,
                        help="frame-advance rate in Hz (0 = as fast as the arena acks)")
    parser.add_argument("--loops", type=int, default=0,
                        help="times to play the pattern through (0 = forever, until Ctrl-C)")
    args = parser.parse_args()

    pat = PatternFile(args.pattern)
    print(
        f"{args.pattern}: {pat.frame_count} frames, {pat.row_count}x{pat.col_count} panels, "
        f"gs_val={pat.gs_val} ({'GS2' if pat.gs_val == 1 else 'GS16'}), "
        f"{pat.frame_byte_count} B/frame on disk"
    )

    if args.port:
        transport = SerialTransport(args.port)
        print(f"connected via serial {args.port}")
    else:
        try:
            transport = TcpTransport(args.ip)
        except OSError as e:
            print(f"connect to {args.ip}:{TCP_PORT} failed: {e}", file=sys.stderr)
            sys.exit(1)
        print(f"connected via TCP {args.ip}:{TCP_PORT}")

    period = 1.0 / args.fps if args.fps > 0 else 0.0

    try:
        # Persist mode: without this the controller rewrites every streamed
        # panel block's cmd byte to Oneshot (the default), which only shows
        # each frame briefly instead of holding it for continuous playback.
        send_command(transport, "panel-mode", [0x02, SET_PANEL_DISPLAY_MODE_CMD, MODE_PERSIST])

        loop_count = 0
        try:
            while args.loops == 0 or loop_count < args.loops:
                for i in range(pat.frame_count):
                    t0 = time.monotonic()
                    status = send_frame(transport, pat.frame_payload(i))
                    if status != 0:
                        print(f"frame {i}: stream-frame status={status}", file=sys.stderr)
                    if period:
                        dt = period - (time.monotonic() - t0)
                        if dt > 0:
                            time.sleep(dt)
                loop_count += 1
                print(f"played loop {loop_count}" + ("" if args.loops else " (Ctrl-C to stop)"))
        except KeyboardInterrupt:
            print("\ninterrupted")
        finally:
            send_command(transport, "stop-display", [0x01, STOP_DISPLAY_CMD])
            send_command(transport, "all-off", [0x01, ALL_OFF_CMD])
    finally:
        transport.close()


if __name__ == "__main__":
    main()
