"""Signed frame_rate tests for trial_params (0x08) — GitHub issue #4.

  T1  test_forward_playback  : mode=2, frame_rate=+N → frame index advances forward
  T2  test_reverse_playback  : mode=2, frame_rate=-N → frame index advances backward
  T3  test_zero_rate_static  : mode=2, frame_rate=0  → frame index does not change

Direction is verified by reading the live frame index via GET_FRAME_POSITION
(0x72) before and after a sleep, not by counting SPI refresh activity. The rate
and window are chosen so the index advances only a few steps — well under the
pattern's frame count — so wrap is unambiguous.

All three require an SD pattern with several frames; pass --pat.

Run:
    pixi run test-serial -- --pat arena.pat
    pixi run test-tcp   -- --ip 10.0.0.x --pat arena.pat
"""

import time
import struct
import pytest

from .commands import (
    ALL_OFF_CMD,
    TRIAL_PARAMS_CMD,
    GET_FRAME_POSITION_CMD,
)

# Advance rate and observation window. RATE * SLEEP steps must stay safely below
# the pattern frame count so forward/reverse motion can't be confused by wrap.
RATE_HZ = 5
SLEEP_S = 0.6
EXPECTED_STEPS = RATE_HZ * SLEEP_S  # ~3


def _trial_params(transport, mode: int, pat_id: int, frame_rate_hz: int,
                  gain: int = 0, init_pos: int = 0):
    """Send a trial-params command. frame_rate_hz is int16 (negative = reverse)."""
    pat_bytes  = struct.pack("<H", pat_id)
    rate_bytes = struct.pack("<h", frame_rate_hz)  # signed int16
    gain_byte  = struct.pack("<b", gain)
    pos_bytes  = struct.pack("<H", init_pos)
    params = bytes([mode]) + pat_bytes + rate_bytes + gain_byte + pos_bytes + b"\x00\x00\x00"
    return transport.command(TRIAL_PARAMS_CMD, params)


def _get_position(transport):
    """Return (cur_frame_index, frame_count) from GET_FRAME_POSITION (0x72)."""
    st, _, payload, _ = transport.command(GET_FRAME_POSITION_CMD)
    assert st == 0, f"GET_FRAME_POSITION failed: status={st}"
    assert len(payload) >= 4, f"GET_FRAME_POSITION short payload: {bytes(payload).hex()}"
    idx, n = struct.unpack_from("<HH", bytes(payload))
    return idx, n


def _fwd_delta(before: int, after: int, n: int) -> int:
    """Steps taken if moving forward, modulo n (0..n-1)."""
    return (after - before) % n


# ── T1: forward playback ──────────────────────────────────────────────────────

def test_forward_playback(transport, pat):
    """Mode 2 at +RATE_HZ: the frame index must advance forward (mod N)."""
    transport.command(ALL_OFF_CMD)

    st, _, _, _ = _trial_params(transport, mode=2, pat_id=pat,
                                frame_rate_hz=RATE_HZ, init_pos=0)
    assert st == 0, f"trial-params (forward) failed: status={st}"

    before, n = _get_position(transport)
    if n < 4:
        pytest.skip(f"pattern has only {n} frame(s); need ≥4 to test direction")
    time.sleep(SLEEP_S)
    after, _ = _get_position(transport)

    fwd = _fwd_delta(before, after, n)
    # Forward motion: a small positive number of steps (not zero, not a near-full
    # wrap which would indicate reverse motion).
    assert 0 < fwd <= EXPECTED_STEPS + 2, (
        f"expected small forward advance, got before={before} after={after} "
        f"n={n} forward_delta={fwd}"
    )


# ── T2: reverse playback ──────────────────────────────────────────────────────

def test_reverse_playback(transport, pat):
    """Mode 2 at -RATE_HZ: the frame index must advance backward (mod N)."""
    transport.command(ALL_OFF_CMD)

    # Start mid-pattern so the index has room to count down (and to exercise the
    # wrap path it would hit if it ran long enough).
    st, _, payload, _ = _trial_params(transport, mode=2, pat_id=pat,
                                      frame_rate_hz=-RATE_HZ, init_pos=9)
    assert st == 0, f"trial-params (reverse, frame_rate=-{RATE_HZ}) rejected: status={st}"

    before, n = _get_position(transport)
    if n < 4:
        pytest.skip(f"pattern has only {n} frame(s); need ≥4 to test direction")
    time.sleep(SLEEP_S)
    after, _ = _get_position(transport)

    # Reverse motion shows up as a small *backward* delta, i.e. the forward delta
    # is a near-full wrap (n - small).
    rev = _fwd_delta(after, before, n)  # steps if moving backward
    assert 0 < rev <= EXPECTED_STEPS + 2, (
        f"expected small reverse advance, got before={before} after={after} "
        f"n={n} reverse_delta={rev}"
    )


# ── T3: zero rate static ──────────────────────────────────────────────────────

def test_zero_rate_static(transport, pat):
    """Mode 2 with frame_rate=0 must hold init_pos (no auto-advance)."""
    transport.command(ALL_OFF_CMD)

    INIT = 3
    st, _, _, _ = _trial_params(transport, mode=2, pat_id=pat,
                                frame_rate_hz=0, init_pos=INIT)
    assert st == 0, f"trial-params (zero rate) failed: status={st}"

    before, n = _get_position(transport)
    expected = INIT % n if n else 0
    assert before == expected, (
        f"index after load: expected {expected} (init_pos {INIT} mod {n}), got {before}"
    )
    time.sleep(0.5)
    after, _ = _get_position(transport)
    assert after == before, (
        f"frame index advanced with frame_rate=0: {before} → {after}"
    )
