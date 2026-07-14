"""Tests for the V1 STREAM_FRAME path (triggered / gated display modes).

Validates that the arena firmware acknowledges STREAM_FRAME (0x32) commands
with status=0 for both Gated (0x33) and Triggered (0x32) per-panel modes.

Background
----------
Puts every panel into Triggered (0x32) or Gated (0x33) mode by streaming a
host-built Gray_16 frame with STREAM_FRAME (0x32). The arena copies the frame
verbatim and clocks it to all panels, so the per-panel command byte — and
therefore the display mode — is chosen entirely here on the host.

Pair with an AD3 driving the arena's external EINT input (jumper = direct-to-
EINT) to observe triggered/gated behavior on the physical display.

  gated     Stream a 0x33 frame; arena retransmits it. EINT HIGH -> visible,
            EINT LOW -> dark.
  triggered Stream a 0x32 frame once, then STOP_DISPLAY. Each EINT rising edge
            fires one row; 20 edges = one frame, then dark. Re-arm to replay.
"""

from .commands import ALL_OFF_CMD, STOP_DISPLAY_CMD, STREAM_FRAME_CMD
from .transport import parse_response

# --- arena frame geometry (src/constants.h, src/G6PanelProtocol.h) ---------
PANEL_COUNT_PER_FRAME     = 20    # 2 rows x 10 cols
STREAM_FRAME_PREFIX_BYTES = 4     # 'F','R', idx_lo, idx_hi (skipped by transferFrame)
HEADER_SIZE               = 2     # version byte + cmd byte
PANEL_SIZE                = 20    # 20x20 pixels
GS16_PAYLOAD_BYTES        = PANEL_SIZE * PANEL_SIZE // 2 + 1   # 200 nibble bytes + 1 duty = 201
GS16_BLOCK_BYTES          = HEADER_SIZE + GS16_PAYLOAD_BYTES   # 203
GS16_FRAME_BYTES          = STREAM_FRAME_PREFIX_BYTES + PANEL_COUNT_PER_FRAME * GS16_BLOCK_BYTES  # 4064

# --- panel display-command bytes, V1 Gray_16 (panel/src/protocol.h) --------
# NB: same numeric value as STREAM_FRAME_CMD but a *different* layer — this byte
# rides inside each per-panel block, not the arena command header.
PANEL_PROTOCOL_V1        = 0x01
PANEL_CMD_GS16_ONESHOT   = 0x30
PANEL_CMD_GS16_TRIGGERED = 0x32
PANEL_CMD_GS16_GATED     = 0x33


# ---------------------------------------------------------------------------
# Wire helpers (mirror panel/src/message.cpp)
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
    """Full opcode-first STREAM_FRAME wire command: [0x32, len_lo, len_hi, prefix, 20 blocks].

    Not length-prefixed like standard commands — mirrors SET_PATTERN_FILE (0x85).
    """
    payload = pack_gray_16(grid, duty)
    block = build_block(cmd_id, payload)
    assert len(block) == GS16_BLOCK_BYTES, (len(block), GS16_BLOCK_BYTES)
    prefix = bytes([ord('F'), ord('R'), 0, 0])
    frame = prefix + block * PANEL_COUNT_PER_FRAME
    assert len(frame) == GS16_FRAME_BYTES, (len(frame), GS16_FRAME_BYTES)
    n = len(frame)
    return bytes([STREAM_FRAME_CMD, n & 0xFF, (n >> 8) & 0xFF]) + frame


def make_grid(name, level=15):
    """Return a 20x20 list-of-lists for a named test pattern."""
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
        raise ValueError(f"unknown pattern {name!r}")
    return g


def _stream(transport, cmd_id, grid, duty):
    """Send a STREAM_FRAME command and return the parsed ack tuple."""
    transport._send(build_stream_command(cmd_id, grid, duty))
    raw = transport._recv_raw(timeout=4.0)   # longer: 4064-byte frame + SPI fan-out to 20 panels
    return parse_response(raw)


# ---------------------------------------------------------------------------
_ALL_ON_GRID = make_grid("all_on")


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def test_stream_gated_acks(transport):
    """STREAM_FRAME with per-panel GATED (0x33) cmd returns status=0."""
    st, echo, _, _ = _stream(transport, PANEL_CMD_GS16_GATED, _ALL_ON_GRID, 255)
    transport.command(STOP_DISPLAY_CMD)   # stop continuous retransmit; leave arena idle
    transport.command(ALL_OFF_CMD)
    assert st == 0
    assert echo == STREAM_FRAME_CMD


def test_stream_triggered_acks(transport):
    """STREAM_FRAME with per-panel TRIGGERED (0x32) cmd returns status=0."""
    st, echo, _, _ = _stream(transport, PANEL_CMD_GS16_TRIGGERED, _ALL_ON_GRID, 80)
    transport.command(STOP_DISPLAY_CMD)   # required: stop retransmit so EINT row counter isn't reset mid-frame
    assert st == 0
    assert echo == STREAM_FRAME_CMD
