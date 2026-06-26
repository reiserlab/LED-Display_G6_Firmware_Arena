"""Panel display mode tests — SET_PANEL_DISPLAY_MODE_CMD (0x1B) / GET (0x1C).

  T1  default_is_persist          : GET before any SET returns 1 (persist)
  T2  set_and_get_roundtrip       : SET 0..3 all accepted; GET echoes the value
  T3  set_invalid_mode_rejected   : SET 4 / 255 return status=1
  T4  mode_persists_across_cmds   : mode survives an unrelated command round-trip

Full opcode verification (checking block[1] on the SPI bus) requires a logic
analyser; these tests cover the protocol layer only.

Run:
    pixi run test-serial
    pixi run test-tcp -- --ip 10.0.0.x
"""

import pytest

from .commands import (
    ALL_OFF_CMD,
    GET_PANEL_DISPLAY_MODE_CMD,
    GET_REFRESH_RATE_CMD,
    SET_PANEL_DISPLAY_MODE_CMD,
)


def _set_mode(transport, mode: int):
    st, _, payload, _ = transport.command(SET_PANEL_DISPLAY_MODE_CMD, bytes([mode]))
    return st, (payload[0] if payload else None)


def _get_mode(transport) -> int:
    st, _, payload, _ = transport.command(GET_PANEL_DISPLAY_MODE_CMD)
    assert st == 0, f"GET_PANEL_DISPLAY_MODE failed: status={st}"
    return payload[0]


@pytest.fixture(autouse=True)
def restore_persist(transport):
    """Restore default (persist=1) after each test."""
    yield
    transport.command(SET_PANEL_DISPLAY_MODE_CMD, bytes([1]))


# ── T1: default ───────────────────────────────────────────────────────────────

def test_default_is_persist(transport):
    """GET before any SET must return 1 (persist)."""
    transport.command(ALL_OFF_CMD)
    transport.command(SET_PANEL_DISPLAY_MODE_CMD, bytes([1]))  # explicit reset
    mode = _get_mode(transport)
    assert mode == 1, f"default panel display mode: expected 1 (persist), got {mode}"


# ── T2: round-trip ────────────────────────────────────────────────────────────

def test_set_and_get_roundtrip(transport):
    """SET 0..3 all accepted; GET returns exactly what was set."""
    mode_names = {0: "oneshot", 1: "persist", 2: "triggered", 3: "gated"}
    for m in range(4):
        st, echo = _set_mode(transport, m)
        assert st == 0, f"SET_PANEL_DISPLAY_MODE {m} ({mode_names[m]}) rejected"
        assert echo == m, f"SET echo: expected {m}, got {echo}"
        got = _get_mode(transport)
        assert got == m, (
            f"GET after SET {m} ({mode_names[m]}): expected {m}, got {got}"
        )


# ── T3: invalid mode ──────────────────────────────────────────────────────────

def test_set_invalid_mode_rejected(transport):
    """SET mode=4 and higher must return status=1."""
    for bad in (4, 5, 255):
        st, _ = _set_mode(transport, bad)
        assert st != 0, f"SET_PANEL_DISPLAY_MODE {bad} should have been rejected"


# ── T4: sticky across unrelated commands ──────────────────────────────────────

def test_mode_persists_across_commands(transport):
    """Mode is sticky — survives an unrelated command round-trip."""
    _set_mode(transport, 2)  # triggered
    transport.command(GET_REFRESH_RATE_CMD)
    got = _get_mode(transport)
    assert got == 2, f"mode changed after unrelated command: expected 2, got {got}"
