"""Guided VISUAL confirmation for panel display mode (0x1B/0x1C) — GitHub #6.

This is the wire-/behaviour-level half of the #6 acceptance that the protocol
tests in ``test_gh6_panel_display_mode.py`` can't cover without a logic analyser.
A human watches the arena while the test drives each mode across transmit paths.

What's distinguishable by eye:
  * persist  (1) — Mode-2 playback animates smoothly (baseline / default).
  * triggered(2) — that same playback FREEZES: panels render only on external
                   trigger edges, so with no trigger source the arena stops
                   updating. Switching back to persist resumes it. This is the
                   clean confirmation that the triggered opcode is emitted and
                   honoured — no trigger hardware required.
  * gated    (3) — LEDs follow the gate line (HIGH = visible, LOW = dark).
                   Needs the panel gate line driven to see the effect.
  * oneshot  (0) — NOT visually distinguishable from persist on a continuously
                   refreshed display (the 0x10 vs 0x11 difference is wire-only).
                   Confirm that one with a logic analyser / CIPO capture.

Opt-in: pass --visual (skipped otherwise) and -s so the prompts show. Needs a
multi-frame pattern via --pat and the arena powered with panels visible.

Run:
    pixi run test-serial -- \\
        --pat ../webDisplayTools/patterns/g6_2x10/patterns/002_grating_sq.pat \\
        --visual -s
"""

import sys
import time
import struct

import pytest

from .commands import (
    ALL_OFF_CMD,
    ALL_ON_CMD,
    TRIAL_PARAMS_CMD,
    SET_PANEL_DISPLAY_MODE_CMD,
    GET_PANEL_DISPLAY_MODE_CMD,
)

MODE = {"oneshot": 0, "persist": 1, "triggered": 2, "gated": 3}


def _pause(message: str, settle_s: float = 5.0):
    """Show a step instruction and wait. Uses Enter when interactive (-s on a
    TTY), otherwise sleeps so the run never blocks under pytest capture."""
    print("\n" + message)
    if sys.stdin is not None and sys.stdin.isatty():
        input("    >>> press Enter when you've confirmed… ")
    else:
        print(f"    (no interactive TTY — pausing {settle_s:.0f}s; pass -s for prompts)")
        time.sleep(settle_s)


def _set_mode(transport, name: str):
    mode = MODE[name]
    st, _, payload, _ = transport.command(SET_PANEL_DISPLAY_MODE_CMD, bytes([mode]))
    assert st == 0, f"SET_PANEL_DISPLAY_MODE {name}({mode}) failed: status={st}"
    assert payload and payload[0] == mode, f"SET echo: expected {mode}, got {payload}"
    print(f"  → panel display mode set to {name} ({mode})")


def _get_mode(transport) -> int:
    st, _, payload, _ = transport.command(GET_PANEL_DISPLAY_MODE_CMD)
    assert st == 0, f"GET_PANEL_DISPLAY_MODE failed: status={st}"
    return payload[0]


def _start_mode2(transport, pat_id: int, fps: int, init_pos: int = 0):
    """trial_params: Mode 2 (open-loop auto-advance) at fps, signed int16 rate."""
    params = (
        bytes([2])
        + struct.pack("<H", pat_id)   # pattern_id
        + struct.pack("<h", fps)      # frame_rate (int16; >0 forward)
        + struct.pack("<b", 0)        # gain
        + struct.pack("<H", init_pos) # init_pos
        + b"\x00\x00\x00"             # reserved
    )
    st, _, _, _ = transport.command(TRIAL_PARAMS_CMD, params)
    assert st == 0, f"trial_params (mode 2) failed: status={st}"


@pytest.fixture(autouse=True)
def _restore(transport):
    """Leave the arena in a safe, default state after the walkthrough."""
    yield
    transport.command(SET_PANEL_DISPLAY_MODE_CMD, bytes([MODE["persist"]]))
    transport.command(ALL_OFF_CMD)


@pytest.mark.visual
def test_visual_panel_display_modes(transport, pat):
    """Operator-guided pass over persist / triggered / gated across paths."""
    print("\n" + "=" * 70)
    print("GH#6 guided visual check — watch the arena panels for each step.")
    print("=" * 70)

    # ── Mode-2 playback: persist → triggered → persist ────────────────────────
    transport.command(ALL_OFF_CMD)
    _set_mode(transport, "persist")
    _start_mode2(transport, pat, fps=4)
    _pause("STEP 1  PERSIST + Mode-2 playback:\n"
           "        The pattern should SCROLL/ANIMATE smoothly (~4 fps).")

    _set_mode(transport, "triggered")
    _pause("STEP 2  → TRIGGERED (no trigger source connected):\n"
           "        The animation should FREEZE (or go dark) — panels are waiting\n"
           "        for external trigger edges that never arrive.")

    _set_mode(transport, "persist")
    _pause("STEP 3  → PERSIST again:\n"
           "        The animation should RESUME scrolling.\n"
           "        (Steps 1–3 confirm the triggered opcode on the Mode-2 path.)")

    # ── Gated: needs the gate line driven ─────────────────────────────────────
    _set_mode(transport, "gated")
    _pause("STEP 4  → GATED:\n"
           "        LEDs follow the panel GATE line: HIGH = visible, LOW = dark.\n"
           "        Drive/toggle the gate line and confirm the arena blinks with it.\n"
           "        (If the gate line is not driven, behaviour depends on its idle\n"
           "        level — note what you observe.)")

    # ── ALL-ON path ───────────────────────────────────────────────────────────
    _set_mode(transport, "persist")
    transport.command(ALL_ON_CMD)
    _pause("STEP 5  ALL-ON + PERSIST:\n"
           "        The whole arena should be lit steadily.")

    _set_mode(transport, "triggered")
    _pause("STEP 6  ALL-ON → TRIGGERED:\n"
           "        No further refresh without trigger edges; the display holds the\n"
           "        last latched frame. Drive the trigger line to see it re-render.")

    # ── Automatable teeth: the command layer still round-trips correctly ──────
    assert _get_mode(transport) == MODE["triggered"], "mode not sticky after ALL-ON"
    print("\n  get-panel-display-mode read back 'triggered' (2) — sticky OK.")
    print("\nVisual walkthrough complete.")
