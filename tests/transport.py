"""Binary framing and transport backends for the Arena-Firmware HIL test suite.

Wire format:
  Command frame:   [length, cmd, params...] — length excludes itself
  Response frame:  [length, status, echo_cmd, payload...]  — status==0 is success
  Diagnostic line: 0xFF <ascii text> \\n   — emitted before response when diag is on

Bulk and opcode-first commands (not length-prefixed):
  SET_PATTERN_FILE (0x85):     [0x85, idx_lo, idx_hi, size_b0..b7, data...]
  SET_PATTERN_FILENAME (0x83): [0x83, idx_lo, idx_hi, name_len, chars...]

GET_PATTERN_FILE (0x84) response:
  Framed header [len=10, 0, 0x84, size_b0..b7] followed immediately by raw file bytes.

Ported from scripts/test_diag_flag.py (_read_frame) and scripts/cipo_common.py (open_cdc).
"""

from abc import ABC, abstractmethod
import socket
import struct
import time

try:
    import serial
except ImportError:
    serial = None

DIAG_SENTINEL = 0xFF
TCP_PORT = 62222

# Opcodes used by transport helper methods.
_CMD_SET_PATTERN_FILE     = 0x85
_CMD_SET_PATTERN_FILENAME = 0x83
_CMD_GET_PATTERN_FILE     = 0x84
_CMD_GET_SD_ARCHIVE       = 0x8A


def build_frame(cmd: int, params: bytes = b"") -> bytes:
    body = bytes([cmd]) + params
    return bytes([len(body)]) + body


def _parse_response_ex(buf: bytes) -> tuple:
    """Parse a raw buffer into (status, echo, payload_bytes, diag_lines, bytes_consumed).

    bytes_consumed is the number of bytes in buf belonging to the response frame
    (including any leading diagnostic lines). Bytes beyond that offset are not
    part of the frame — e.g. raw file data that follows a GET_PATTERN_FILE header.

    Raises ValueError if buf does not yet contain a complete response frame.
    """
    buf = bytes(buf)
    diag_lines = []
    i = 0
    while i < len(buf):
        if buf[i] == DIAG_SENTINEL:
            nl = buf.find(b"\n", i)
            if nl == -1:
                raise ValueError("incomplete diag line")
            diag_lines.append(buf[i + 1:nl].decode("ascii", errors="replace").strip())
            i = nl + 1
        else:
            length = buf[i]
            if i + 1 + length > len(buf):
                raise ValueError("incomplete frame")
            frame   = buf[i + 1:i + 1 + length]
            status  = frame[0] if len(frame) > 0 else -1
            echo    = frame[1] if len(frame) > 1 else -1
            payload = frame[2:] if len(frame) > 2 else b""
            return status, echo, payload, diag_lines, i + 1 + length
    raise ValueError("empty buffer")


def parse_response(buf: bytes) -> tuple:
    """Parse a raw buffer into (status, echo, payload_bytes, diag_lines).

    Strips leading 0xFF diagnostic lines. Raises ValueError if buf does not
    yet contain a complete response frame (used by SerialTransport to probe
    for completeness while accumulating bytes).
    """
    st, echo, payload, diag_lines, _ = _parse_response_ex(bytes(buf))
    return st, echo, payload, diag_lines


class Transport(ABC):
    @abstractmethod
    def open(self): ...

    @abstractmethod
    def close(self): ...

    @abstractmethod
    def _send(self, data: bytes): ...

    @abstractmethod
    def _recv_raw(self, timeout: float) -> bytes: ...

    @abstractmethod
    def _recv_more(self, n: int, timeout: float) -> bytes:
        """Receive exactly n additional raw bytes (used after a framed header)."""
        ...

    def command(self, cmd: int, params: bytes = b"", timeout: float = 1.0) -> tuple:
        """Send a command frame and return (status, echo, payload_bytes, diag_lines)."""
        self._send(build_frame(cmd, params))
        raw = self._recv_raw(timeout)
        return parse_response(raw)

    def upload_file(self, idx: int, data: bytes, timeout: float = 30.0) -> tuple:
        """Upload file data via SET_PATTERN_FILE_CMD (0x85).

        idx=0  → writes to /patterns/pattern.temp (staging slot for new files).
        idx>0  → overwrites the existing 1-based pattern in place.

        Wire: opcode-first, no length prefix:
          [0x85, idx_lo, idx_hi, size_b0..b7, file_data...]   (11-byte header + data)
        Response: standard ack frame.
        """
        header = bytes([_CMD_SET_PATTERN_FILE]) + struct.pack("<HQ", idx, len(data))
        self._send(header + data)
        raw = self._recv_raw(timeout)
        return parse_response(raw)

    def rename_file(self, idx: int, new_name: str, timeout: float = 5.0) -> tuple:
        """Rename a pattern via SET_PATTERN_FILENAME_CMD (0x83).

        idx=0  → promotes pattern.temp to a permanent pattern named new_name.
        idx>0  → renames the existing 1-based pattern.

        Wire: [0x83, idx_lo, idx_hi, name_len, chars...]  (opcode-first)
        Response: (status=0, echo=0x83, payload=new_idx uint16 LE).
        """
        name_bytes = new_name.encode("ascii")
        frame = (bytes([_CMD_SET_PATTERN_FILENAME])
                 + struct.pack("<H", idx)
                 + bytes([len(name_bytes)])
                 + name_bytes)
        self._send(frame)
        raw = self._recv_raw(timeout)
        return parse_response(raw)

    def download_file(self, idx: int, timeout: float = 30.0) -> tuple:
        """Download a pattern file via GET_PATTERN_FILE_CMD (0x84).

        Wire (send): standard frame [0x03, 0x84, idx_lo, idx_hi]
        Wire (recv): framed header  [len=10, 0, 0x84, size_b0..b7]
                     then raw file bytes (size = uint64 LE from header payload).

        Returns (status, echo, file_bytes, diag_lines).
        On error (status != 0), file_bytes is empty.
        """
        self._send(build_frame(_CMD_GET_PATTERN_FILE, struct.pack("<H", idx)))
        raw = self._recv_raw(timeout)
        status, echo, payload, diag_lines, consumed = _parse_response_ex(raw)
        if status != 0:
            return status, echo, b"", diag_lines
        if len(payload) < 8:
            raise RuntimeError("GET_PATTERN_FILE: short size payload in header frame")
        file_size = struct.unpack("<Q", payload[:8])[0]
        leftover = raw[consumed:]
        if len(leftover) >= file_size:
            return status, echo, bytes(leftover[:file_size]), diag_lines
        more = self._recv_more(file_size - len(leftover), timeout)
        return status, echo, bytes(leftover) + more, diag_lines

    def get_sd_archive(self, timeout: float = 60.0) -> tuple:
        """Fetch the SD archive via GET_SD_ARCHIVE_CMD (0x8A).

        Same bulk-response shape as download_file/GET_PATTERN_FILE: framed
        header [len=10, status, 0x8A, size_b0..b7] then raw ZIP bytes on
        success. On error (e.g. CE_DISPLAY_ACTIVE, or "Archive already in
        progress") the handler returns a plain ack frame with no bulk data
        following, so draining is skipped. Always drain on success, even if
        the caller only cares about `status`: leftover ZIP bytes left in the
        stream would otherwise desync the next command's response framing.

        Returns (status, echo, archive_bytes, diag_lines).
        """
        self._send(build_frame(_CMD_GET_SD_ARCHIVE))
        raw = self._recv_raw(timeout)
        status, echo, payload, diag_lines, consumed = _parse_response_ex(raw)
        if status != 0:
            return status, echo, b"", diag_lines
        if len(payload) < 8:
            raise RuntimeError("GET_SD_ARCHIVE: short size payload in header frame")
        archive_size = struct.unpack("<Q", payload[:8])[0]
        leftover = raw[consumed:]
        if len(leftover) >= archive_size:
            return status, echo, bytes(leftover[:archive_size]), diag_lines
        more = self._recv_more(archive_size - len(leftover), timeout)
        return status, echo, bytes(leftover) + more, diag_lines

    def begin_get_sd_archive(self, timeout: float = 5.0) -> tuple:
        """Send GET_SD_ARCHIVE (0x8A) and return (status, echo, archive_size,
        leftover_bytes, diag_lines) after parsing just the framed header,
        WITHOUT draining the raw ZIP body.

        Mirrors begin_download() above: used by tests that need to control
        the pace/timing of draining themselves, e.g. to simulate a host that
        stops reading entirely (a hard stall) partway through the archive.
        """
        self._send(build_frame(_CMD_GET_SD_ARCHIVE))
        raw = self._recv_raw(timeout)
        status, echo, payload, diag_lines, consumed = _parse_response_ex(raw)
        if status != 0:
            return status, echo, 0, b"", diag_lines
        if len(payload) < 8:
            raise RuntimeError("GET_SD_ARCHIVE: short size payload in header frame")
        archive_size = struct.unpack("<Q", payload[:8])[0]
        return status, echo, archive_size, raw[consumed:], diag_lines


class SerialTransport(Transport):
    """USB-CDC backend. Mirrors open_cdc() from scripts/cipo_common.py."""

    def __init__(self, port: str, baud: int = 115200, settle: float = 0.3):
        self.port = port
        self.baud = baud
        self.settle = settle
        self._ser = None

    def open(self):
        self._ser = serial.Serial(self.port, baudrate=self.baud, timeout=0.2)
        time.sleep(self.settle)
        self._ser.reset_input_buffer()

    def close(self):
        if self._ser:
            # Drain bytes the Teensy may still be sending (e.g. end of a file
            # download). Reading them allows the USB TX buffer to drain so
            # sendRaw() on the firmware side can exit its spin-wait before the
            # port closes — otherwise the Teensy locks up for up to 5 seconds
            # and the USB CDC device becomes unreachable.
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                waiting = self._ser.in_waiting
                if waiting:
                    self._ser.read(waiting)
                else:
                    time.sleep(0.05)
                    if not self._ser.in_waiting:
                        break
            self._ser.close()
            self._ser = None

    def _send(self, data: bytes):
        self._ser.write(data)
        self._ser.flush()

    def _recv_raw(self, timeout: float = 1.0) -> bytes:
        """Accumulate bytes until parse_response succeeds or deadline passes.

        Mirrors the _read_frame accumulation loop in scripts/test_diag_flag.py.
        Raises RuntimeError on timeout.
        """
        deadline = time.monotonic() + timeout
        buf = bytearray()
        while time.monotonic() < deadline:
            chunk = self._ser.read(max(1, self._ser.in_waiting))
            if chunk:
                buf.extend(chunk)
            try:
                parse_response(buf)
                return bytes(buf)
            except ValueError:
                pass
        raise RuntimeError(f"serial response timeout ({timeout}s)")

    def _recv_more(self, n: int, timeout: float) -> bytes:
        """Receive n additional raw bytes (e.g. file payload after header frame)."""
        deadline = time.monotonic() + timeout
        buf = bytearray()
        while len(buf) < n and time.monotonic() < deadline:
            chunk = self._ser.read(max(1, self._ser.in_waiting))
            if chunk:
                buf.extend(chunk)
        if len(buf) < n:
            raise RuntimeError(
                f"serial recv_more timeout: got {len(buf)}/{n} bytes"
            )
        return bytes(buf[:n])

    def begin_download(self, idx: int, timeout: float = 2.0) -> tuple:
        """Send GET_PATTERN_FILE (0x84) and return (status, echo, file_size,
        leftover_bytes, diag_lines) after parsing just the framed header,
        WITHOUT draining the raw body.

        Used by download_file_throttled() below, and directly by tests that
        need to control the pace/timing of draining themselves — e.g. to
        simulate a host that stops reading entirely (a hard stall) rather
        than just a slow one.
        """
        self._send(build_frame(_CMD_GET_PATTERN_FILE, struct.pack("<H", idx)))
        raw = self._recv_raw(timeout)
        status, echo, payload, diag_lines, consumed = _parse_response_ex(raw)
        if status != 0:
            return status, echo, 0, b"", diag_lines
        if len(payload) < 8:
            raise RuntimeError("GET_PATTERN_FILE: short size payload in header frame")
        file_size = struct.unpack("<Q", payload[:8])[0]
        return status, echo, file_size, raw[consumed:], diag_lines

    def download_file_throttled(self, idx: int, bytes_per_sec: float,
                                idle_timeout: float = 20.0,
                                overall_timeout: float = 600.0) -> tuple:
        """Download via GET_PATTERN_FILE (0x84), pacing host reads to ~bytes_per_sec
        on AVERAGE via a token bucket with a CAPPED burst allowance, so the total
        transfer takes roughly file_size/bytes_per_sec — not just "eventually
        completes" — while never leaving the OS read buffer undrained for more
        than one poll interval (~20 ms), well under the controller's stall bound
        (fix 1, ~1.5 s).

        Three earlier versions of this helper got the pacing wrong, all
        confirmed empirically via GET_CONTROLLER_INFO's debug fields (see
        debug/issue-16-bulk-download-notes.md investigation):
          v1: read a fixed chunk once per 200 ms window, then went fully idle for
              the rest of that window (up to ~180 ms of true silence, repeated)
              — bursty enough to look like a stall.
          v2: a token bucket with an UNCAPPED debt — the running total could get
              ahead of the target trajectory (e.g. from whatever the controller
              pushed into the ~4 KB kernel buffer before the first read), and
              then reads were withheld entirely until wall-clock time paid the
              debt down, which took over 1.5 s and starved the controller for
              real, not just on average.
          v3: dropped pacing on the READ side entirely (always drained all of
              `in_waiting`) and tried to pace by varying the sleep between
              checks — but with nothing capping how much a single read could
              grab, an initial burst let the whole file drain in a few checks,
              finishing in ~10 s instead of the intended ~80 s: not a stall, but
              not a paced transfer either.
        v4 fixed the pacing itself (token bucket capped at half a second's
        worth: no debt, no unbounded burst credit), but still used a FIXED
        overall deadline from the start — exactly the wall-clock-not-idle bug
        fix 3 exists to fix, just relocated to the test harness: a transfer
        that was still progressing, only slower than bytes_per_sec would
        ideally allow (real SD/USB throughput varies under sustained test-
        session load — see debug/issue-16-bulk-download-notes.md), got killed
        by an arbitrary total-time cutoff even though the controller's own
        debug fields confirmed it was never actually stalled.
        `idle_timeout` mirrors the controller's own idle-based deadline: it
        resets on every chunk actually read, so only a genuine stall (no
        forward progress at all) trips it. `overall_timeout` is a generous
        absolute backstop against a truly-hung transfer, not the primary bound.
        """
        status, echo, file_size, leftover, diag_lines = self.begin_download(idx, 2.0)
        if status != 0:
            return status, echo, b"", diag_lines
        data = bytearray(leftover)
        start = time.monotonic()
        overall_deadline = start + overall_timeout
        idle_deadline = start + idle_timeout
        poll_interval = 0.02
        max_tokens = max(1.0, bytes_per_sec * 0.5)
        tokens = 0.0
        last = start
        while len(data) < file_size:
            now = time.monotonic()
            if now > overall_deadline:
                raise RuntimeError(
                    f"throttled download exceeded overall cap ({overall_timeout}s): "
                    f"got {len(data)}/{file_size} bytes"
                )
            if now > idle_deadline:
                raise RuntimeError(
                    f"throttled download idle for {idle_timeout}s: "
                    f"got {len(data)}/{file_size} bytes"
                )
            tokens = min(max_tokens, tokens + bytes_per_sec * (now - last))
            last = now
            if tokens >= 1.0:
                avail = self._ser.in_waiting
                if avail:
                    want = min(avail, int(tokens), file_size - len(data))
                    if want > 0:
                        chunk = self._ser.read(want)
                        data.extend(chunk)
                        tokens -= len(chunk)
                        idle_deadline = time.monotonic() + idle_timeout
            time.sleep(poll_interval)
        return status, echo, bytes(data[:file_size]), diag_lines

class TcpTransport(Transport):
    """TCP backend. Mirrors the socket pattern in scripts/all_on.py and controller_info.py."""

    def __init__(self, ip: str, port: int = TCP_PORT):
        self.ip = ip
        self.port = port
        self._sock = None

    def open(self):
        self._sock = socket.create_connection((self.ip, self.port), timeout=2.0)
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def close(self):
        if self._sock:
            self._sock.close()
            self._sock = None

    def _send(self, data: bytes):
        self._sock.sendall(data)

    def _recv_raw(self, timeout: float = 1.0) -> bytes:
        self._sock.settimeout(timeout)
        return self._sock.recv(4096)

    def _recv_more(self, n: int, timeout: float) -> bytes:
        """Receive n additional raw bytes by looping recv until n bytes accumulated."""
        deadline = time.monotonic() + timeout
        buf = bytearray()
        self._sock.settimeout(1.0)
        while len(buf) < n and time.monotonic() < deadline:
            try:
                chunk = self._sock.recv(min(4096, n - len(buf)))
                if chunk:
                    buf.extend(chunk)
            except socket.timeout:
                pass
        if len(buf) < n:
            raise RuntimeError(
                f"TCP recv_more timeout: got {len(buf)}/{n} bytes"
            )
        return bytes(buf[:n])
