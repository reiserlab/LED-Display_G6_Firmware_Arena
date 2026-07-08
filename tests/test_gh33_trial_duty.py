"""Per-trial duty tests — trial_params (0x08) param[11], issue #33 (stateless
redesign). Wire order matches the GH-4 re-layout (mode, pattern_id, frame_rate,
init_pos, gain int16, duration), with duty appended as an optional 12th byte.

  T1  duty_omitted_defaults_to_stored : 11-param message (no duty byte)
                                        accepted, means "stored duty"
  T2  explicit_zero_duty_is_noop      : 12-param message with duty=0 accepted,
                                        also means "stored duty"
  T3  duty_trial_accepted             : duty=0x40 trial starts across Modes 2
                                        and 3
  T4  stored_pattern_unmodified       : 0x88 reports the STORED duty before,
                                        during, and after a duty trial
                                        (transmit-time only)
  T5  set_pattern_id_clears_duty      : 0x03 after a duty trial starts a
                                        stored-duty Mode 3

Observing the duty byte on the SPI bus needs a logic analyser; these cover the
protocol layer and the SD-data invariant. All tests need --pat.

Run:
    pixi run test-serial -- --pat path/to/pattern.pat
    pixi run test-tcp -- --ip 10.0.0.x --pat path/to/pattern.pat
"""

import struct

import pytest

from .commands import (
    ALL_OFF_CMD,
    GET_PATTERN_INFO_CMD,
    SET_PATTERN_ID_CMD,
    TRIAL_PARAMS_CMD,
)


def _trial_params(transport, mode: int, pat_id: int, frame_rate_hz: int = 0,
                  init_pos: int = 0, gain: int = 0, duration_ticks: int = 0,
                  duty: int | None = None):
    """Send trial_params in post-relayout wire order: mode, pattern_id,
    frame_rate, init_pos, gain (int16), duration (uint16 ticks), and an
    optional trailing duty byte (param[11]; omitted means 'stored duty')."""
    params = (bytes([mode]) + struct.pack("<H", pat_id)
              + struct.pack("<h", frame_rate_hz) + struct.pack("<H", init_pos)
              + struct.pack("<h", gain) + struct.pack("<H", duration_ticks))
    if duty is not None:
        params += bytes([duty])
    return transport.command(TRIAL_PARAMS_CMD, params)


def _get_pattern_duty(transport, pat_id: int) -> int:
    """Stored frame-0 duty_cycle via GET_PATTERN_INFO (0x88) — last payload byte."""
    st, _, payload, _ = transport.command(GET_PATTERN_INFO_CMD,
                                          struct.pack("<H", pat_id))
    assert st == 0, f"GET_PATTERN_INFO failed: status={st}"
    return struct.unpack("<HBBBBBIB", bytes(payload))[-1]


@pytest.fixture(autouse=True)
def all_off_after(transport):
    """Park the arena dark after each test (also clears any per-trial duty)."""
    yield
    transport.command(ALL_OFF_CMD)


# ── T1: duty byte omitted defaults to stored duty ────────────────────────────

def test_duty_omitted_defaults_to_stored(transport, pat):
    """The base 11-param message (no duty byte) must still be accepted and
    mean 'stored duty'."""
    st, _, _, _ = _trial_params(transport, mode=3, pat_id=pat)
    assert st == 0, f"11-param trial_params (no duty) rejected: status={st}"


# ── T2: explicit duty=0 is also a no-op ──────────────────────────────────────

def test_explicit_zero_duty_is_noop(transport, pat):
    """A 12-param message with duty=0 is accepted and also means 'stored duty'."""
    st, _, _, _ = _trial_params(transport, mode=3, pat_id=pat, duty=0)
    assert st == 0, f"trial_params with duty=0 rejected: status={st}"


# ── T3: duty trial accepted in Modes 2 and 3 ─────────────────────────────────

def test_duty_trial_accepted(transport, pat):
    """A trial declaring duty=0x40 starts normally in show-frame and open-loop."""
    st, _, _, _ = _trial_params(transport, mode=3, pat_id=pat, duty=0x40)
    assert st == 0, f"mode 3 duty trial rejected: status={st}"
    st, _, _, _ = _trial_params(transport, mode=2, pat_id=pat,
                                frame_rate_hz=2, duty=0x40)
    assert st == 0, f"mode 2 duty trial rejected: status={st}"


# ── T4: SD pattern data is never modified ─────────────────────────────────────

def test_stored_pattern_unmodified(transport, pat):
    """0x88 must report the STORED duty before, during, and after a duty trial."""
    transport.command(ALL_OFF_CMD)
    stored = _get_pattern_duty(transport, pat)

    override = 0x11 if stored != 0x11 else 0x22
    st, _, _, _ = _trial_params(transport, mode=3, pat_id=pat, duty=override)
    assert st == 0, f"duty trial failed to start: status={st}"

    assert _get_pattern_duty(transport, pat) == stored, (
        "stored pattern duty changed while a duty trial was displaying"
    )

    transport.command(ALL_OFF_CMD)
    assert _get_pattern_duty(transport, pat) == stored, (
        "stored pattern duty changed after the duty trial ended"
    )


# ── T5: SET_PATTERN_ID declares no duty ──────────────────────────────────────

def test_set_pattern_id_clears_duty(transport, pat):
    """0x03 after a duty trial must start a stored-duty Mode 3 (0x03 carries no
    duty field, so any prior trial's duty must not leak into it)."""
    st, _, _, _ = _trial_params(transport, mode=3, pat_id=pat, duty=0x40)
    assert st == 0
    st, _, payload, _ = transport.command(SET_PATTERN_ID_CMD,
                                          struct.pack("<H", pat))
    assert st == 0, f"SET_PATTERN_ID after duty trial rejected: status={st}"
    # Protocol-level check only: the load succeeded and echoes the id. The
    # stored-duty rendering is a bench/logic-analyser observation.
    assert struct.unpack("<H", bytes(payload[:2]))[0] == pat
