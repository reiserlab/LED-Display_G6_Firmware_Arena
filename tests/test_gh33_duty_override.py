"""Duty-cycle override tests — SET_DUTY_OVERRIDE_CMD (0x1D) / GET (0x1E), issue #33.

  T1  disabled_reads_zero        : GET after an explicit disable returns [0, 0]
  T2  set_and_get_roundtrip      : SET accepts duty 0..255; SET echo + GET match
  T3  invalid_enable_rejected    : enable=2 / 255 return status=1, state unchanged
  T4  wrong_length_rejected      : 1- and 3-param-byte payloads return status=1
  T5  sticky_across_commands     : override survives an unrelated command round-trip
  T6  stored_pattern_unmodified  : override is transmit-time only — 0x88 keeps
                                   reporting the duty stored in the pattern file,
                                   even while the pattern is displayed overridden
                                   (needs --pat)

Verifying the override on the wire (last byte of each panel block on the SPI
bus) requires a logic analyser; these tests cover the protocol layer plus the
SD-data invariant.

Run:
    pixi run test-serial -- --pat path/to/pattern.pat
    pixi run test-tcp -- --ip 10.0.0.x --pat path/to/pattern.pat
"""

import struct

import pytest

from .commands import (
    ALL_OFF_CMD,
    GET_DUTY_OVERRIDE_CMD,
    GET_PATTERN_INFO_CMD,
    GET_REFRESH_RATE_CMD,
    SET_DUTY_OVERRIDE_CMD,
    TRIAL_PARAMS_CMD,
)


def _set_override(transport, enable: int, duty: int):
    st, _, payload, _ = transport.command(SET_DUTY_OVERRIDE_CMD,
                                          bytes([enable, duty]))
    return st, bytes(payload)


def _get_override(transport) -> tuple[int, int]:
    st, _, payload, _ = transport.command(GET_DUTY_OVERRIDE_CMD)
    assert st == 0, f"GET_DUTY_OVERRIDE failed: status={st}"
    assert len(payload) == 2, f"GET_DUTY_OVERRIDE payload: {bytes(payload).hex()}"
    return payload[0], payload[1]


def _get_pattern_duty(transport, pat_id: int) -> int:
    """Stored frame-0 duty_cycle via GET_PATTERN_INFO (0x88) — last payload byte."""
    st, _, payload, _ = transport.command(GET_PATTERN_INFO_CMD,
                                          struct.pack("<H", pat_id))
    assert st == 0, f"GET_PATTERN_INFO failed: status={st}"
    return struct.unpack("<HBBBBBIB", bytes(payload))[-1]


@pytest.fixture(autouse=True)
def restore_off(transport):
    """Restore the default (override off) after each test."""
    yield
    transport.command(SET_DUTY_OVERRIDE_CMD, bytes([0, 0]))


# ── T1: disabled state reads [0, 0] ──────────────────────────────────────────

def test_disabled_reads_zero(transport):
    """GET after an explicit disable must return [enable=0, duty=0]."""
    st, _ = _set_override(transport, 0, 0)
    assert st == 0, "disabling the override was rejected"
    assert _get_override(transport) == (0, 0)


# ── T2: round-trip ────────────────────────────────────────────────────────────

def test_set_and_get_roundtrip(transport):
    """SET accepts the full duty range; SET echo and GET both report it."""
    for duty in (0, 1, 0x40, 0x80, 0xFF):
        st, echo = _set_override(transport, 1, duty)
        assert st == 0, f"SET_DUTY_OVERRIDE duty={duty} rejected"
        assert echo == bytes([1, duty]), (
            f"SET echo: expected [1, {duty}], got {echo.hex()}"
        )
        assert _get_override(transport) == (1, duty)


# ── T3: invalid enable ────────────────────────────────────────────────────────

def test_invalid_enable_rejected(transport):
    """enable outside 0|1 must return status=1 and leave the state unchanged."""
    _set_override(transport, 1, 0x33)
    for bad in (2, 255):
        st, _ = _set_override(transport, bad, 0x99)
        assert st != 0, f"SET_DUTY_OVERRIDE enable={bad} should have been rejected"
    assert _get_override(transport) == (1, 0x33), (
        "rejected SET must not change the override state"
    )


# ── T4: wrong payload length ──────────────────────────────────────────────────

def test_wrong_length_rejected(transport):
    """A missing or extra parameter byte must return status=1."""
    for params in (bytes([1]), bytes([1, 0x40, 0x00])):
        st, _, _, _ = transport.command(SET_DUTY_OVERRIDE_CMD, params)
        assert st != 0, (
            f"SET_DUTY_OVERRIDE with {len(params)} param byte(s) should be rejected"
        )


# ── T5: sticky across unrelated commands ──────────────────────────────────────

def test_sticky_across_commands(transport):
    """Override is sticky — survives an unrelated command round-trip."""
    _set_override(transport, 1, 0x55)
    transport.command(GET_REFRESH_RATE_CMD)
    assert _get_override(transport) == (1, 0x55)


# ── T6: SD pattern data is never modified ─────────────────────────────────────

def test_stored_pattern_unmodified(transport, pat):
    """0x88 must report the STORED duty before, during, and after an overridden
    display — the override patches blocks at transmit time only."""
    transport.command(ALL_OFF_CMD)
    stored = _get_pattern_duty(transport, pat)

    # Pick an override value that differs from the stored duty.
    override = 0x11 if stored != 0x11 else 0x22
    st, _ = _set_override(transport, 1, override)
    assert st == 0

    # Display the pattern (Mode 3, static frame 0) so loadFrame() runs with the
    # override active, then confirm the file on SD still carries the old duty.
    params = (bytes([3]) + struct.pack("<H", pat) + struct.pack("<h", 0)
              + struct.pack("<b", 0) + struct.pack("<H", 0) + b"\x00\x00\x00")
    st, _, _, _ = transport.command(TRIAL_PARAMS_CMD, params)
    assert st == 0, f"trial-params (mode 3) failed: status={st}"

    assert _get_pattern_duty(transport, pat) == stored, (
        "stored pattern duty changed while the override was active"
    )

    transport.command(ALL_OFF_CMD)
    _set_override(transport, 0, 0)
    assert _get_pattern_duty(transport, pat) == stored, (
        "stored pattern duty changed after the override was disabled"
    )
