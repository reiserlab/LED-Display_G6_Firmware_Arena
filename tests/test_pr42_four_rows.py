"""Guided-visual per-row addressability check for the 4x10 arena topology
(PR #42, src/ArenaConfig.h): rows 2 and 3 are newly wired to real per-row CS
lines that were previously tied HIGH as MISO OE-decode inputs (see the
removed cs_decode_tie_high_pins table). The panel_sets[] CS/panel-index
arithmetic was verified by code review (every CS pin is distinct, panel_b0/
panel_b1 form a 0..39 bijection matching panel_index = row*10 + col), but
that only proves the table is internally consistent -- it cannot prove the
table matches how the physical board is actually wired.

maDisplayTools/configs/arena_hardware/arena_10-10_v1p1r7.yaml (the hardware
profile this PR's ArenaConfig.h comment cites) is explicitly flagged
"untested / unvalidated draft ... confirm against actual arena PCB net-trace
before relying on this for bring-up." This test is that confirmation: it
lights exactly one physical row at a time (all 10 columns) and asks a human
watching the arena to confirm which row lit -- catching a swapped/aliased
row-CS pin that no protocol-level check can see.

Run: pixi run test-serial-visual -- -s   (pair with -s so prompts are shown)
"""

import sys
import time

import pytest

from .commands import ALL_OFF_CMD, SET_PANEL_DISPLAY_MODE_CMD, STOP_DISPLAY_CMD, STREAM_FRAME_CMD
from .test_stream_trigger import PANEL_CMD_GS16_ONESHOT, build_block, make_grid, pack_gray_16
from .transport import parse_response

MODE_PERSIST = 1

ROW_COUNT = 4
COL_COUNT = 10
PANEL_COUNT_PER_FRAME = ROW_COUNT * COL_COUNT  # 40; src/constants.h

_ON_BLOCK = build_block(PANEL_CMD_GS16_ONESHOT, pack_gray_16(make_grid("all_on", level=15), 255))
_OFF_BLOCK = build_block(PANEL_CMD_GS16_ONESHOT, pack_gray_16(make_grid("all_on", level=0), 0))
_FRAME_PREFIX = bytes([ord("F"), ord("R"), 0, 0])


def _row_frame(target_row):
    """Build a full 40-panel STREAM_FRAME with only `target_row` lit (all 10
    columns), every other row dark. panel_index = row * COL_COUNT + col
    (src/ArenaConfig.h). target_row=None -> every panel dark."""
    blocks = bytearray()
    for panel_index in range(PANEL_COUNT_PER_FRAME):
        row = panel_index // COL_COUNT
        blocks += _ON_BLOCK if row == target_row else _OFF_BLOCK
    frame = _FRAME_PREFIX + bytes(blocks)
    n = len(frame)
    return bytes([STREAM_FRAME_CMD, n & 0xFF, (n >> 8) & 0xFF]) + frame


def _set_panel_display_mode(transport, mode):
    st, _, payload, _ = transport.command(SET_PANEL_DISPLAY_MODE_CMD, bytes([mode]))
    assert st == 0, f"SET_PANEL_DISPLAY_MODE({mode}) failed: status={st}"
    assert payload and payload[0] == mode


def _stream_row(transport, target_row):
    """Set persist mode and stream a single-row frame. Deliberately does NOT
    call STOP_DISPLAY_CMD -- see test_pr15_stuck_row_timeout.py's
    _stream_all_on() docstring for why (STOP_DISPLAY_CMD blanks frame_buf_
    immediately). The arena keeps re-delivering this frame at its refresh
    rate until the next _stream_row()/_restore call."""
    _set_panel_display_mode(transport, MODE_PERSIST)
    transport._send(_row_frame(target_row))
    raw = transport._recv_raw(timeout=4.0)
    st, echo, _, _ = parse_response(raw)
    assert st == 0, f"STREAM_FRAME row={target_row} failed: status={st}"
    assert echo == STREAM_FRAME_CMD


def _pause(message: str, settle_s: float = 5.0):
    print("\n" + message)
    if sys.stdin is not None and sys.stdin.isatty():
        input("    >>> press Enter when you've confirmed… ")
    else:
        print(f"    (no interactive TTY — pausing {settle_s:.0f}s; pass -s for prompts)")
        time.sleep(settle_s)


@pytest.fixture(autouse=True)
def _restore(transport):
    yield
    transport.command(STOP_DISPLAY_CMD)
    transport.command(ALL_OFF_CMD)
    transport.command(SET_PANEL_DISPLAY_MODE_CMD, bytes([MODE_PERSIST]))


@pytest.mark.visual
@pytest.mark.parametrize("row", range(ROW_COUNT))
def test_visual_row_lights_up_alone(transport, row):
    """Stream a frame with only `row` lit (all 10 columns) and ask a human
    to confirm exactly that physical row is bright and no other row shows any
    light -- rows 2 and 3 are the newly-wired ones (PR #42); rows 0 and 1 are
    included as a same-run baseline (if those broke too, it's not just the
    new wiring)."""
    _stream_row(transport, row)
    _pause(
        f"Streaming row index {row} of {ROW_COUNT} (0-based) -- all 10 columns "
        f"of that row lit, all other rows dark.\n"
        f"        CONFIRM: exactly one physical row is lit across all 10 "
        f"columns, and note which physical row (top/bottom position) it is.\n"
        f"        BUG signature to watch for: the WRONG physical row lights "
        f"(a swapped CS pin), MULTIPLE rows light (a CS pin driving two "
        f"rows), or NO row lights / a row is dim/partial (a dead or "
        f"floating CS pin -- rows 2/3 depend on the MISO OE-decode pins "
        f"upgraded from tie-high to real per-row CS, see ArenaConfig.h)."
    )


@pytest.mark.visual
def test_visual_all_rows_off_between_frames(transport):
    """Sanity bookend: after the last row test, an explicit all-dark frame
    should leave the whole 4x10 grid dark -- confirms _restore's teardown
    path (STOP_DISPLAY_CMD) actually blanks all 4 rows, not just the 2 that
    existed pre-PR#42."""
    _stream_row(transport, None)
    _pause(
        "Streaming an all-dark 4x10 frame.\n"
        "        CONFIRM: the entire arena (all 4 rows x 10 columns) is dark."
    )
