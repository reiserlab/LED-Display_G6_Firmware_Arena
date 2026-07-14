"""Arena-level demonstration of Panel-Firmware/next-steps-pr-15.md finding #1:
a stuck-lit row after a two-PIO completion-poll timeout in Oneshot/Triggered
display mode.

Root cause (display_scan_twopio.cpp): on a per-row completion-poll timeout,
twopio_prime() only loads the all-off word into the PIO's Y/OSR register — it
never executes the `out pins, 20` that would actually drive the row GPIOs
off. Persistent/Gated self-correct on the next row-burst; Oneshot and
Triggered do not, so the affected row can stay physically lit indefinitely.

Prerequisite — flash ONE panel with the debug-timeout build before running
anything in this file:

    (Panel-Firmware) pixi run platformio run -d panel -e pico_v031_twopiotimeouttest -t upload

That build shrinks TWOPIO_ROW_TIMEOUT_US to 5 us (a real full-duty burst
takes ~45-50 us), so wait_burst_done() times out on (almost) every row,
forcing the broken self-heal path to run continuously. Drive that panel with
an all-on frame through THIS test file and row 0 will demonstrably fail to
go dark (see the IMPORTANT note below for why it's always row 0, not some
other row). On production firmware the timeout essentially never fires, so
these tests are only meaningful against the debug-flashed panel.

Two ways to observe the stuck row:

  visual  Guided — a human watches the debug-flashed panel. Opt-in via
          --visual (pair with -s):
              pixi run test-serial-visual -- -s
              pixi run test-tcp-visual -- --ip 10.0.0.x -s

  ad3     Automated — an AD3 probes the row GPIO directly. Opt-in via --ad3,
          run under the debugad3 pixi environment (adds dwfpy):
              pixi run -e debugad3 test-serial-ad3
              pixi run -e debugad3 test-tcp-ad3
          Wiring (see ad3_row_probe.py's docstring for detail):
              AD3 DIO0 -> debug panel's GP20 test point (row 0, bottom row).
              AD3 DIO1 -> arena external EINT input (~330 ohm series R), for
                          the Triggered test only.
              AD3 GND  -> panel GND

IMPORTANT — why BOTH tests probe row 0, not some other row: for Oneshot,
show() (PANEL_REV==31) calls twopio_scan_frame(), which returns immediately
on the FIRST row that faults (display_scan_twopio.cpp: `for r in 0..19: if
(!twopio_scan_row(r, ...)) return false;`) — it does not continue sweeping
the remaining rows. Since the debug build's 5 us budget is far below a real
~45-50 us burst, row 0 (scanned first) always faults, and rows 1-19 are
never even attempted that pass. Triggered is separately row-0-only: the
tests below fire exactly one EINT edge, so only show_row(0) ever runs.

Both variants exercise the same wire-level setup: a Gray_16 all-on frame is
streamed to the arena (STREAM_FRAME, cmd 0x32) with the per-panel Oneshot
(0x30) or Triggered (0x32) command byte, like test_stream_trigger.py — BUT
the arena's CommandProcessor::patchDispMode() rewrites the per-panel command
byte of every streamed frame to match its configured panel display mode
(SET_PANEL_DISPLAY_MODE, 0x1B; default persist). So each test MUST set the
arena's mode first, or the panel silently receives 0x31 persist regardless
of what the stream carries — bench-confirmed via the SPI_DIAG heartbeat's
`cmd=` field. Persist can never demonstrate this finding: the panel then
re-attempts row 0 back-to-back (~80k faults/s livelock), so the row is
legitimately driven most of the time on fixed and unfixed builds alike.
"""

import sys
import time

import pytest

from .commands import (
    ALL_OFF_CMD,
    SET_PANEL_DISPLAY_MODE_CMD,
    STOP_DISPLAY_CMD,
    STREAM_FRAME_CMD,
)
from .test_stream_trigger import (
    PANEL_CMD_GS16_ONESHOT,
    PANEL_CMD_GS16_TRIGGERED,
    build_stream_command,
    make_grid,
)
from .transport import parse_response

# Arena panel-display-mode values for SET_PANEL_DISPLAY_MODE_CMD (0x1B).
MODE_ONESHOT   = 0
MODE_PERSIST   = 1
MODE_TRIGGERED = 2

_ALL_ON_GRID  = make_grid("all_on", level=15)
_ALL_OFF_GRID = make_grid("all_on", level=0)   # same shape, zero level

# Row 0 (GP20) is the reliable probe point for BOTH modes — see the
# "IMPORTANT" note in the module docstring.
ROW0_BOTTOM_PROBE_DIO  = 0   # GP20
EINT_DRIVE_DIO         = 1

# V1 Triggered's EINT sanity timeout (display.cpp: wait_eint_rising(1'000'000)).
# Wait past it so the row-stuck assertion isn't racing triggered_active_.
TRIGGERED_EINT_SANITY_TIMEOUT_S = 1.0


def _stream(transport, cmd_id, grid, duty):
    """Send a STREAM_FRAME command and return the parsed ack tuple."""
    transport._send(build_stream_command(cmd_id, grid, duty))
    raw = transport._recv_raw(timeout=4.0)
    return parse_response(raw)


def _set_panel_display_mode(transport, mode):
    """Set the arena's panel display mode (0x1B) — REQUIRED before streaming.

    patchDispMode() rewrites every streamed block's per-panel command byte to
    this mode, so the mode carried by build_stream_command()'s cmd_id is
    otherwise silently overridden (see module docstring).
    """
    st, _, payload, _ = transport.command(SET_PANEL_DISPLAY_MODE_CMD, bytes([mode]))
    assert st == 0, f"SET_PANEL_DISPLAY_MODE({mode}) failed: status={st}"
    assert payload and payload[0] == mode


def _stream_all_on(transport, cmd_id, mode, duty=255):
    """Set the arena's panel display mode, stream an all-on frame, and leave
    it streaming.

    Deliberately does NOT call STOP_DISPLAY_CMD: CommandProcessor::enterAllOff()
    (what STOP_DISPLAY_CMD triggers) overwrites frame_buf_ with an all-DARK
    frame and pushes it out over SPI three times immediately — calling it
    right after streaming all-on would blank every panel before anyone could
    observe anything, which is exactly why nothing appeared to turn on when
    this was tried with a STOP_DISPLAY_CMD here. Leaving the arena in
    STREAMING_FRAME state means it keeps re-delivering this same frame at its
    refresh rate (~300 Hz default for GS16) via transmitOnRefresh() until the
    `_restore` fixture's STOP_DISPLAY_CMD runs at teardown.
    """
    _set_panel_display_mode(transport, mode)
    st, echo, _, _ = _stream(transport, cmd_id, _ALL_ON_GRID, duty)
    assert st == 0, f"STREAM_FRAME (cmd={cmd_id:#x}) all-on failed: status={st}"
    assert echo == STREAM_FRAME_CMD


def _stream_all_off(transport, cmd_id):
    """Drive a fresh all-off frame — re-drives every row, self-healing any
    stuck-lit GPIO. Used only in teardown."""
    _stream(transport, cmd_id, _ALL_OFF_GRID, 0)
    transport.command(STOP_DISPLAY_CMD)


def _pause(message: str, settle_s: float = 5.0):
    """Show a step instruction and wait. Uses Enter when interactive (-s on a
    TTY), otherwise sleeps so the run never blocks under pytest capture."""
    print("\n" + message)
    if sys.stdin is not None and sys.stdin.isatty():
        input("    >>> press Enter when you've confirmed… ")
    else:
        print(f"    (no interactive TTY — pausing {settle_s:.0f}s; pass -s for prompts)")
        time.sleep(settle_s)


@pytest.fixture(autouse=True)
def _restore(transport):
    """Leave the debug-flashed panel dark (a fresh good frame self-heals any
    stuck-lit row) and the arena back in its default persist mode after each
    test."""
    yield
    _stream_all_off(transport, PANEL_CMD_GS16_ONESHOT)
    transport.command(ALL_OFF_CMD)
    transport.command(SET_PANEL_DISPLAY_MODE_CMD, bytes([MODE_PERSIST]))


# ---------------------------------------------------------------------------
# Guided-visual
# ---------------------------------------------------------------------------
@pytest.mark.visual
def test_visual_oneshot_stuck_row_after_timeout(transport):
    """PR#15 #1 — Oneshot: row should go dark, stays lit on the debug build."""
    print("\n" + "=" * 70)
    print("PR#15 finding #1 — Oneshot stuck-row check.")
    print("Panel under test must be flashed with pico_v031_twopiotimeouttest.")
    print("=" * 70)
    _stream_all_on(transport, PANEL_CMD_GS16_ONESHOT, MODE_ONESHOT)
    _pause(
        "Oneshot all-on frame streaming (arena keeps re-delivering it at its\n"
        "        refresh rate, ~300 Hz for GS16, until this test's teardown).\n"
        "        Watch the debug panel's row-0 pattern (a lit band whose exact\n"
        "        shape depends on the LED-matrix routing — bench: 2x2 squares).\n"
        "        BRIGHTNESS is the discriminator, not presence:\n"
        "        EXPECTED (fixed firmware): VERY DIM — the row is only driven\n"
        "        for the ~8 us fault window of each delivery (~0.2% duty).\n"
        "        BUG (unfixed firmware): STEADILY BRIGHT — the row stays\n"
        "        driven between deliveries because the self-heal never\n"
        "        drives the pins off (bench-verified: dim-vs-bright pair\n"
        "        confirmed side by side, plus PINS r=FFFFF between attempts\n"
        "        on the fixed build via the twopiotimeoutdiag heartbeat).\n"
        "        Confirm which you observe."
    )


@pytest.mark.visual
def test_visual_triggered_stuck_row_after_timeout(transport):
    """PR#15 #1 — Triggered: row should go dark once triggered_active_ drops."""
    print("\n" + "=" * 70)
    print("PR#15 finding #1 — Triggered stuck-row check.")
    print("Panel under test must be flashed with pico_v031_twopiotimeouttest.")
    print("=" * 70)
    _stream_all_on(transport, PANEL_CMD_GS16_TRIGGERED, MODE_TRIGGERED)
    _pause(
        "Triggered all-on frame streaming.\n"
        "        Now pulse the arena's external EINT input a few times\n"
        "        (by hand, or `panel/tools/ad3_trigger.py burst --edges 3`),\n"
        "        then wait >1 s with no further edges (the panel's internal\n"
        "        EINT sanity timeout fires and triggered_active_ goes false).\n"
        "        EXPECTED (fixed firmware): the debug panel's row 0 is DARK.\n"
        "        BUG (current firmware): row 0 stays LIT — confirm which you\n"
        "        observe.",
        settle_s=8.0,
    )


# ---------------------------------------------------------------------------
# AD3-instrumented
# ---------------------------------------------------------------------------
@pytest.mark.ad3
def test_ad3_oneshot_stuck_row_after_timeout(transport):
    """PR#15 #1 — Oneshot, AD3-probed row 0 (GP20, bottom row).

    Row 0: show()'s frame scan (twopio_scan_frame()) returns on the first
    row that faults without attempting the rest, and row 0 is always
    scanned first — see the module docstring's IMPORTANT note.
    """
    from . import ad3_row_probe

    _stream_all_on(transport, PANEL_CMD_GS16_ONESHOT, MODE_ONESHOT)
    time.sleep(0.2)  # let a few re-delivered scans run
    # Debounced: the arena keeps re-delivering this frame (~300 Hz), so on
    # fixed firmware row 0 is briefly, legitimately ON for the ~8 us fault
    # window within each otherwise-dark ~3.3 ms cycle. A single sample could
    # land in that window; majority-vote over several samples doesn't.
    level = ad3_row_probe.sample_row_pin_debounced(ROW0_BOTTOM_PROBE_DIO)
    assert level == 1, (
        "row 0 (GP20, bottom row) reads mostly LOW (driven ON) across "
        "repeated all-on Oneshot deliveries that should mostly leave it "
        "dark — PR#15 finding #1: twopio_prime()'s self-heal doesn't "
        "execute the corrective `out pins, 20`."
    )


@pytest.mark.ad3
def test_ad3_triggered_stuck_row_after_timeout(transport):
    """PR#15 #1 — Triggered, AD3-probed row 0 (GP20)."""
    from . import ad3_row_probe

    _stream_all_on(transport, PANEL_CMD_GS16_TRIGGERED, MODE_TRIGGERED)
    ad3_row_probe.drive_eint_burst(edges=1, dio_out=EINT_DRIVE_DIO)
    time.sleep(TRIGGERED_EINT_SANITY_TIMEOUT_S + 0.5)  # past the 1 s EINT sanity timeout
    level = ad3_row_probe.sample_row_pin(ROW0_BOTTOM_PROBE_DIO)
    assert level == 1, (
        "row 0 (GP20, bottom row) still reads LOW (driven ON) after "
        "triggered_active_ should have gone false with no corrective burst "
        "— PR#15 finding #1."
    )
