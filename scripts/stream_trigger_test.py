#!/usr/bin/env python3
"""Phase A — arena-level external-trigger test via the V1 *streaming* path.

Puts every panel on the arena into **Triggered** (cmd 0x32) or **Gated** (cmd 0x33)
mode by streaming a host-built Gray_16 frame over USB-serial with `STREAM_FRAME`
(0x32). The arena copies the frame verbatim and clocks it to all panels, so the
per-panel *command byte* — and therefore the display mode — is chosen entirely
here on the host. **No arena firmware change is needed for this path.**

Pair it with the AD3 driving the arena's external EINT input (jumper = direct-to-
EINT) — see ad3_trigger.py.

Modes
-----
  gated      stream a 0x33 frame; the arena's STREAMING_FRAME retransmit keeps it
             fresh. EINT HIGH -> pattern visible, EINT LOW -> dark. (Hold AD3 W1
             HIGH for a steady image; toggle it to strobe.)
  triggered  stream a 0x32 frame ONCE, then send STOP_DISPLAY (0x30) so the arena
             stops retransmitting — otherwise each ~300 Hz retransmit would reset
             the panel's 20-edge row counter mid-frame. Each EINT rising edge
             fires one row; 20 edges = one frame, then dark. Re-run (or --repeat)
             to re-arm. Drive EINT continuously at ~1-8 kHz for a bright frame.
  off        STOP_DISPLAY + ALL_OFF.

Examples
--------
    ./stream_trigger_test.py gated      --pattern all_on --duty 255
    ./stream_trigger_test.py triggered  --pattern all_on --duty 80
    ./stream_trigger_test.py triggered  --pattern border --duty 80 --repeat 50 --interval 0.5
    ./stream_trigger_test.py off
"""

import argparse
import sys
import time

# --- arena host-command opcodes (src/commands.h) ---------------------------
STREAM_FRAME_CMD = 0x32   # stream form: [0x32, len_lo, len_hi, <frame bytes>]
STOP_DISPLAY_CMD = 0x30   # short form:  [0x01, 0x30]  -> enterAllOff (halts retransmit)
ALL_OFF_CMD      = 0x00
GET_CONTROLLER_INFO_CMD = 0x67

# --- arena frame geometry (src/constants.h, src/G6PanelProtocol.h) ---------
PANEL_COUNT_PER_FRAME      = 20    # 2 rows x 10 cols
STREAM_FRAME_PREFIX_BYTES  = 4     # 'F','R', idx_lo, idx_hi (skipped by transferFrame)
HEADER_SIZE                = 2     # version byte + cmd byte (no trailing CRC; parity in header)
PANEL_SIZE                 = 20    # 20x20 pixels
GS16_PAYLOAD_BYTES         = PANEL_SIZE * PANEL_SIZE // 2 + 1   # 200 nibble bytes + 1 duty = 201
GS16_BLOCK_BYTES           = HEADER_SIZE + GS16_PAYLOAD_BYTES   # 203
GS16_FRAME_BYTES           = STREAM_FRAME_PREFIX_BYTES + PANEL_COUNT_PER_FRAME * GS16_BLOCK_BYTES  # 4064

# --- panel display-command bytes, V1 Gray_16 (panel/src/protocol.h) --------
# NB: same numeric value as STREAM_FRAME_CMD but a *different* layer (this byte
# rides inside each per-panel block, not the arena command header).
PANEL_PROTOCOL_V1            = 0x01
PANEL_CMD_GS16_TRIGGERED     = 0x32
PANEL_CMD_GS16_GATED         = 0x33

MASTER_VID = 0x16C0   # Teensy USB CDC


# ---------------------------------------------------------------------------
# Wire helpers (mirror panel/src/message.cpp; see panel/tools/test_psram_demo.py)
# ---------------------------------------------------------------------------
def _popcount(x):
    return bin(x & 0xFF).count("1")


def _parity_bit(cmd, payload):
    """Even parity over version[0..6] + cmd + payload (Message::calculate_parity_bit)."""
    ones = _popcount(PANEL_PROTOCOL_V1 & 0x7F) + _popcount(cmd)
    for b in payload:
        ones += _popcount(b)
    return ones & 1


def pack_gray_16(grid, duty):
    """20x20 grid of 0..15 -> 200 nibble bytes (2 px/byte, even px = high nibble) + duty."""
    out = bytearray(GS16_PAYLOAD_BYTES)
    pixel = 0
    for i in range(PANEL_SIZE):
        for j in range(PANEL_SIZE):
            v = grid[i][j] & 0x0F
            byte = pixel // 2
            if pixel % 2 == 0:
                out[byte] = (out[byte] & 0x0F) | (v << 4)
            else:
                out[byte] = (out[byte] & 0xF0) | v
            pixel += 1
    out[-1] = duty & 0xFF
    return bytes(out)


def build_block(cmd_id, payload):
    """One per-panel V1 block: [header(version|parity), cmd, payload...]."""
    header = PANEL_PROTOCOL_V1 | (_parity_bit(cmd_id, payload) << 7)
    return bytes([header, cmd_id]) + payload


def build_stream_command(cmd_id, grid, duty):
    """Full STREAM_FRAME command: [0x32, len_lo, len_hi] + prefix + 20 identical blocks."""
    payload = pack_gray_16(grid, duty)
    block = build_block(cmd_id, payload)
    assert len(block) == GS16_BLOCK_BYTES, (len(block), GS16_BLOCK_BYTES)
    prefix = bytes([ord('F'), ord('R'), 0, 0])
    frame = prefix + block * PANEL_COUNT_PER_FRAME
    assert len(frame) == GS16_FRAME_BYTES, (len(frame), GS16_FRAME_BYTES)
    n = len(frame)
    return bytes([STREAM_FRAME_CMD, n & 0xFF, (n >> 8) & 0xFF]) + frame


# ---------------------------------------------------------------------------
# Test patterns (20x20, values 0..15)
# ---------------------------------------------------------------------------
def pattern(name, level=15):
    g = [[0] * PANEL_SIZE for _ in range(PANEL_SIZE)]
    if name == "all_on":
        for i in range(PANEL_SIZE):
            for j in range(PANEL_SIZE):
                g[i][j] = level
    elif name == "border":
        for i in range(PANEL_SIZE):
            for j in range(PANEL_SIZE):
                if i in (0, PANEL_SIZE - 1) or j in (0, PANEL_SIZE - 1):
                    g[i][j] = level
    elif name == "checker":
        for i in range(PANEL_SIZE):
            for j in range(PANEL_SIZE):
                g[i][j] = level if ((i // 2 + j // 2) & 1) else 0
    elif name == "cross":
        mid = PANEL_SIZE // 2
        for k in range(PANEL_SIZE):
            g[mid][k] = level
            g[k][mid] = level
    else:
        raise SystemExit("unknown pattern %r" % name)
    return g


# ---------------------------------------------------------------------------
# Arena CDC master (binary framing; mirrors psram_demo_drive.py Master)
# ---------------------------------------------------------------------------
class Master:
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

    def stream(self, cmd_id, grid, duty):
        return self.command(build_stream_command(cmd_id, grid, duty))

    def stop_display(self):
        return self.command([0x01, STOP_DISPLAY_CMD])

    def all_off(self):
        return self.command([0x01, ALL_OFF_CMD])

    def info(self):
        return self.command([0x01, GET_CONTROLLER_INFO_CMD])


def find_port():
    """Auto-detect the arena Teensy CDC by USB VID."""
    try:
        from serial.tools import list_ports
    except ImportError:
        return None
    for p in list_ports.comports():
        if (p.vid == MASTER_VID) or ("usbmodem" in (p.device or "")):
            return p.device
    return None


def _show(label, resp):
    if resp is None:
        print(f"  {label}: <no response>")
    else:
        status, echo, payload = resp
        print(f"  {label}: status={status} echo=0x{echo:02X} "
              f"payload={payload.hex(' ') if payload else ''}")


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=("gated", "triggered", "off", "info"))
    ap.add_argument("--port", default=None, help="arena CDC port (auto-detect if omitted)")
    ap.add_argument("--pattern", default="all_on",
                    choices=("all_on", "border", "checker", "cross"))
    ap.add_argument("--level", type=int, default=15, help="Gray_16 pixel level 0..15")
    ap.add_argument("--duty", type=int, default=255, help="duty_cycle 0..255")
    ap.add_argument("--repeat", type=int, default=1,
                    help="triggered: re-arm this many times (re-stream + STOP each cycle)")
    ap.add_argument("--interval", type=float, default=0.5,
                    help="triggered: seconds between re-arms")
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        print("No arena CDC port found; pass --port /dev/cu.usbmodemXXXX", file=sys.stderr)
        sys.exit(1)
    print(f"arena port: {port}")
    m = Master(port)

    if args.mode == "info":
        _show("controller_info", m.info())
        return

    if args.mode == "off":
        _show("stop_display", m.stop_display())
        _show("all_off", m.all_off())
        return

    grid = pattern(args.pattern, args.level)

    if args.mode == "gated":
        # Continuous retransmit is fine for Gated; EINT level gates the output.
        _show(f"stream GATED (0x33) pattern={args.pattern} duty={args.duty}",
              m.stream(PANEL_CMD_GS16_GATED, grid, args.duty))
        print("  -> hold AD3 W1 HIGH for a steady image; LOW = dark; toggle = strobe.")
        print("  -> run `stream_trigger_test.py off` to clear.")
        return

    # triggered: single-shot per arm. Stream 0x32 once, then STOP_DISPLAY so the
    # arena stops retransmitting (a retransmit would reset the row counter).
    for k in range(args.repeat):
        r = m.stream(PANEL_CMD_GS16_TRIGGERED, grid, args.duty)
        m.stop_display()
        if k == 0 or args.repeat <= 5:
            _show(f"arm TRIGGERED (0x32) #{k+1} pattern={args.pattern} duty={args.duty}", r)
        if k + 1 < args.repeat:
            time.sleep(args.interval)
    print(f"  -> armed {args.repeat}x. Drive EINT at ~1-8 kHz for a bright frame; "
          f"20 edges = 1 frame then dark.")


if __name__ == "__main__":
    main()
