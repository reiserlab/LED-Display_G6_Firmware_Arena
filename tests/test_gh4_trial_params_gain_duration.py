"""Widened gain (int16) + controller-run Duration tests for trial_params (0x08)
— GitHub issue #4 remaining scope (canonical re-layout: gain widen + init_pos/
gain swap + Duration auto-stop).

  T1  test_widened_gain_accepted       : gain beyond the old int8 ceiling (e.g.
                                          2000) is accepted in Mode 4
  T2  test_duration_zero_no_autostop   : duration=0 -> controller never reaches
                                          ALL_OFF on its own
  T3  test_duration_autostop           : duration=N -> controller reaches
                                          ALL_OFF on its own once N ticks elapse

Duration is AC::constants::duration_tick_ms (10 ms) ticks. There's no opcode
that reports display state directly, so GET_SD_ARCHIVE_CMD (0x8A) is reused as
an indirect probe: it's gated on state_ == ArenaState::ALL_OFF
(CommandProcessor.cpp) and refuses with CE_DISPLAY_ACTIVE otherwise.

A full closed-loop fps-scaling readback isn't available on the wire today, so
T1 asserts acceptance + a sane response, not the resulting fps.

All three require an SD pattern; pass --pat.

Run:
    pixi run test-serial -- --pat arena.pat
    pixi run test-tcp   -- --ip 10.0.0.x --pat arena.pat
"""

import time
import struct

from .commands import (
    ALL_OFF_CMD,
    TRIAL_PARAMS_CMD,
)


def _trial_params(transport, mode: int, pat_id: int, frame_rate_hz: int = 0,
                  init_pos: int = 0, gain: int = 0, duration_ticks: int = 0):
    """Send a trial-params command using the post-relayout wire order:
    mode, pattern_id, frame_rate, init_pos, gain (int16), duration (uint16 ticks).
    """
    pat_bytes  = struct.pack("<H", pat_id)
    rate_bytes = struct.pack("<h", frame_rate_hz)
    pos_bytes  = struct.pack("<H", init_pos)
    gain_bytes = struct.pack("<h", gain)
    dur_bytes  = struct.pack("<H", duration_ticks)
    params = bytes([mode]) + pat_bytes + rate_bytes + pos_bytes + gain_bytes + dur_bytes
    return transport.command(TRIAL_PARAMS_CMD, params)


# ── T1: widened gain range ────────────────────────────────────────────────────

def test_widened_gain_accepted(transport, pat):
    """Mode 4 with gain well past the old int8 ceiling (127) must be accepted."""
    transport.command(ALL_OFF_CMD)

    st, _, _, _ = _trial_params(transport, mode=4, pat_id=pat, gain=2000)
    assert st == 0, f"trial-params (gain=2000, widened int16) rejected: status={st}"

    transport.command(ALL_OFF_CMD)


# ── T2: duration == 0 -> no auto-stop ────────────────────────────────────────

def test_duration_zero_no_autostop(transport, pat):
    """duration=0 must never auto-stop; GET_SD_ARCHIVE stays rejected (display active)."""
    transport.command(ALL_OFF_CMD)

    st, _, _, _ = _trial_params(transport, mode=2, pat_id=pat, frame_rate_hz=5,
                                duration_ticks=0)
    assert st == 0, f"trial-params (duration=0) failed: status={st}"

    time.sleep(2.0)  # well past any plausible trial length

    st2, _, _, _ = transport.get_sd_archive()
    assert st2 != 0, (
        f"GET_SD_ARCHIVE_CMD succeeded (status={st2}) — controller auto-stopped "
        "despite duration=0"
    )

    transport.command(ALL_OFF_CMD)


# ── T3: duration=N -> auto-stop ──────────────────────────────────────────────

def test_duration_autostop(transport, pat):
    """A short duration must cause the controller to revert to ALL_OFF on its own."""
    transport.command(ALL_OFF_CMD)

    DURATION_TICKS = 100  # 100 * 10 ms = 1 s
    st, _, _, _ = _trial_params(transport, mode=2, pat_id=pat, frame_rate_hz=5,
                                duration_ticks=DURATION_TICKS)
    assert st == 0, f"trial-params (duration={DURATION_TICKS}) failed: status={st}"

    time.sleep(1.5)  # past the 1 s trial length

    st2, _, _, _ = transport.get_sd_archive()
    assert st2 == 0, (
        f"GET_SD_ARCHIVE_CMD rejected (status={st2}) — controller did not reach "
        "ALL_OFF on its own after duration elapsed"
    )
