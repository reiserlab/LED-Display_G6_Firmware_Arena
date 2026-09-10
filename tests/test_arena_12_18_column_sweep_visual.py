"""Guided-visual LED-column sweep for the 12-column arena_12-18 topology
(ARENA_HW_12_18 build, src/hw/ArenaConfig_12_18.h): lights exactly one
physical pixel column at a time, all 4 rows tall, and sweeps it from the
first pixel column of panel column P1 to the last pixel column of panel
column P12 -- 12 panel columns x 20 pixels/panel = 240 columns total.

This exercises every panel column's CS/COPI/CIPO wiring and every pixel
column's bit position in one continuous run: a stuck, swapped, or
out-of-order panel column shows up as the sweep skipping, freezing, or
jumping to the wrong physical position instead of moving smoothly left to
right across the whole 12-column width. It complements
test_pr42_four_rows.py (which confirms per-ROW CS wiring on the 4x10 board)
by confirming per-COLUMN wiring and pixel bit-order on the wider 4x12 board.

Requires the arena flashed with the teensy41-12-18[-performance] build (see
platformio.ini) -- panel_count_per_frame there is 48 (4 rows x 12 cols), not
40. Run against a 10-10-flashed arena and the very first STREAM_FRAME will
fail its status==0 assertion ("Bad stream-frame size"), which is itself a
useful signal that the wrong firmware variant is on the board.

Run: pixi run test-serial-visual -- -s   (pair with -s so prompts are shown)
"""

import sys
import time

import pytest

from .commands import ALL_OFF_CMD, SET_PANEL_DISPLAY_MODE_CMD, STOP_DISPLAY_CMD, STREAM_FRAME_CMD
from .test_stream_trigger import PANEL_CMD_GS16_ONESHOT, PANEL_SIZE, build_block, pack_gray_16
from .transport import parse_response

MODE_PERSIST = 1

ROW_COUNT = 4
COL_COUNT = 12
PANEL_COUNT_PER_FRAME = ROW_COUNT * COL_COUNT  # 48; src/constants.h (ARENA_HW_12_18)
TOTAL_LED_COLUMNS = COL_COUNT * PANEL_SIZE     # 240 pixel columns across the full width

# Seconds to hold each pixel column lit before advancing -- slow enough to
# see, fast enough that a full sweep doesn't drag (240 steps * 0.1s = 24s).
STEP_DELAY_S = 0.1


def _column_grid(local_col, level=15):
    """20x20 grid (row-major, src/G6PanelProtocol.h pixel order) with only
    pixel column `local_col` lit across all 20 rows; local_col=None -> dark."""
    g = [[0] * PANEL_SIZE for _ in range(PANEL_SIZE)]
    if local_col is not None:
        for row in range(PANEL_SIZE):
            g[row][local_col] = level
    return g


# One ON block per local pixel column (reused across all 12 panel columns --
# only WHICH panel gets it changes, not the block bytes) plus a single dark
# block for every panel not in the currently-lit column.
_ON_BLOCKS = [
    build_block(PANEL_CMD_GS16_ONESHOT, pack_gray_16(_column_grid(c), 255))
    for c in range(PANEL_SIZE)
]
_OFF_BLOCK = build_block(PANEL_CMD_GS16_ONESHOT, pack_gray_16(_column_grid(None), 0))
_FRAME_PREFIX = bytes([ord("F"), ord("R"), 0, 0])


def _led_column_frame(target_panel_col, target_local_col):
    """Build a full 48-panel STREAM_FRAME with only pixel column
    `target_local_col` of panel column `target_panel_col` lit, in all 4 rows
    -- i.e. one continuous vertical LED column at that position across the
    whole arena height. panel_index = row * COL_COUNT + col (src/ArenaConfig.h).
    target_panel_col=None -> every panel dark."""
    on_block = _ON_BLOCKS[target_local_col]
    blocks = bytearray()
    for panel_index in range(PANEL_COUNT_PER_FRAME):
        col = panel_index % COL_COUNT
        blocks += on_block if (target_panel_col is not None and col == target_panel_col) else _OFF_BLOCK
    frame = _FRAME_PREFIX + bytes(blocks)
    n = len(frame)
    return bytes([STREAM_FRAME_CMD, n & 0xFF, (n >> 8) & 0xFF]) + frame


def _set_panel_display_mode(transport, mode):
    st, _, payload, _ = transport.command(SET_PANEL_DISPLAY_MODE_CMD, bytes([mode]))
    assert st == 0, f"SET_PANEL_DISPLAY_MODE({mode}) failed: status={st}"
    assert payload and payload[0] == mode


def _stream_led_column(transport, panel_col, local_col):
    """Stream a single-LED-column frame. Deliberately does NOT call
    STOP_DISPLAY_CMD between steps (see test_pr15_stuck_row_timeout.py) --
    the arena keeps re-delivering each frame at its refresh rate until the
    next call, which is what makes the sweep look continuous."""
    transport._send(_led_column_frame(panel_col, local_col))
    raw = transport._recv_raw(timeout=4.0)
    st, echo, _, _ = parse_response(raw)
    assert st == 0, (
        f"STREAM_FRAME panel_col={panel_col} local_col={local_col} failed: "
        f"status={st} (mismatched ARENA_HW_* build? expected 48-panel frame)"
    )
    assert echo == STREAM_FRAME_CMD


def _pause(message: str, settle_s: float = 5.0):
    print("\n" + message)
    if sys.stdin is not None and sys.stdin.isatty():
        input("    >>> press Enter when you've confirmed... ")
    else:
        print(f"    (no interactive TTY -- pausing {settle_s:.0f}s; pass -s for prompts)")
        time.sleep(settle_s)


@pytest.fixture(autouse=True)
def _restore(transport):
    yield
    transport.command(STOP_DISPLAY_CMD)
    transport.command(ALL_OFF_CMD)
    transport.command(SET_PANEL_DISPLAY_MODE_CMD, bytes([MODE_PERSIST]))


@pytest.mark.visual
def test_visual_led_column_sweep(transport):
    """Sweep a single lit LED column across the whole arena width, from the
    first pixel column of panel P1 to the last pixel column of panel P12,
    then ask a human watching the arena to confirm the sweep moved smoothly
    and continuously in one direction with no skips, freezes, jumps, or
    extra lit columns."""
    _set_panel_display_mode(transport, MODE_PERSIST)
    print(
        f"\nSweeping {TOTAL_LED_COLUMNS} LED columns across {COL_COUNT} panel "
        f"columns (P1..P{COL_COUNT}), {STEP_DELAY_S:.2f}s per column..."
    )
    for global_col in range(TOTAL_LED_COLUMNS):
        panel_col, local_col = divmod(global_col, PANEL_SIZE)
        _stream_led_column(transport, panel_col, local_col)
        time.sleep(STEP_DELAY_S)
    _pause(
        f"Swept all {TOTAL_LED_COLUMNS} LED columns left to right across "
        f"panel columns P1..P{COL_COUNT} (all 4 rows tall at each step).\n"
        f"        CONFIRM: exactly one LED column was lit at any given "
        f"moment, and it moved smoothly and continuously from the first "
        f"column of P1 to the last column of P{COL_COUNT} with no skips, "
        f"freezes, backward jumps, or extra lit columns.\n"
        f"        BUG signature to watch for: the sweep freezes or skips at "
        f"a panel-column boundary (wrong/missing CS pin for that column), "
        f"jumps to an unexpected physical position (a panel-column wired "
        f"out of order), or more than one column is lit at once (a CS pin "
        f"shared between two columns)."
    )
