"""#135 io_ext hardware tests — DIO role machine (0xAC/0xAD), 0xAA role gating,
AO mode (0xA3), and the analog-in read (0xA4).

The instrument-free subset of webDisplayTools
docs/development/135-bench-checklist.md §E. Scope/meter items — the framescan
SPI envelope timing, AO/AI absolute accuracy, and the power-cycle boot-default
check — stay manual (tests here can't observe boot state mid-suite and can't
see BNC voltages).

NON-DESTRUCTIVE to the SD card: the frame_number ramp test uses the session
`pat` fixture (adds one 'conftest.pat', deletes nothing); everything else
touches only DIO/AO state and restores boot-equivalent roles afterwards.

Run:
    pixi run test-serial -- --port /dev/cu.usbmodemXXX          # roles/guards
    ... -- --pat .../g6_2x10/patterns/004_frame2_h_ccw_200f.pat # + AO ramp
"""

import struct

import pytest

from .commands import (
    GET_ANALOG_IN_CMD,
    GET_AO_VOLTAGE_CMD,
    GET_DIO_ROLE_CMD,
    GET_PATTERN_INFO_CMD,
    SET_AO_LUT_CMD,
    SET_AO_MODE_CMD,
    SET_AO_VOLTAGE_CMD,
    SET_DIGITAL_OUT_CMD,
    SET_DIO_ROLE_CMD,
    SET_FRAME_POSITION_CMD,
    STOP_DISPLAY_CMD,
    TRIAL_PARAMS_CMD,
)

ROLE_OFF = 0
ROLE_IN_TRIGGER = 1
ROLE_OUT_PROGRAMMABLE = 2
ROLE_OUT_FRAMESCAN = 3

# DAC quantization ~1.22 mV/LSB + integer scaling rounding.
_AO_TOLERANCE_MV = 4


def _dio_roles(transport):
    st, echo, payload, _ = transport.command(GET_DIO_ROLE_CMD)
    assert st == 0, "GET_DIO_ROLE failed"
    assert echo == GET_DIO_ROLE_CMD
    assert len(payload) == 4, f"expected [role1, level1, role2, level2], got {len(payload)}B"
    return tuple(payload)


def _set_role(transport, port, role):
    return transport.command(SET_DIO_ROLE_CMD, bytes([port, role]))


def _get_ao_mv(transport):
    st, _, payload, _ = transport.command(GET_AO_VOLTAGE_CMD)
    assert st == 0, "GET_AO_VOLTAGE failed"
    return struct.unpack("<H", bytes(payload[:2]))[0]


@pytest.fixture
def io_restore(transport):
    """Restore boot-equivalent I/O after each test: port 1 out_programmable,
    port 2 in_trigger (the EINT trigger route), AO programmable at 0 V."""
    yield
    transport.command(SET_AO_MODE_CMD, bytes([0]))
    transport.command(SET_AO_VOLTAGE_CMD, struct.pack("<H", 0))
    _set_role(transport, 1, ROLE_OUT_PROGRAMMABLE)
    _set_role(transport, 2, ROLE_IN_TRIGGER)


# ── DIO role state machine ────────────────────────────────────────────────────

def test_dio_role_readback_shape(transport, io_restore):
    r1, l1, r2, l2 = _dio_roles(transport)
    assert r1 <= 3 and r2 <= 3
    assert l1 in (0, 1) and l2 in (0, 1)


def test_dio_role_roundtrip_port1(transport, io_restore):
    """Port 1 accepts every role and reads it back (no EINT coupling on port 1)."""
    for role in (ROLE_OFF, ROLE_OUT_PROGRAMMABLE, ROLE_OUT_FRAMESCAN,
                 ROLE_IN_TRIGGER, ROLE_OUT_PROGRAMMABLE):
        st, _, payload, _ = _set_role(transport, 1, role)
        assert st == 0, f"SET_DIO_ROLE(1, {role}) refused: {bytes(payload)!r}"
        assert _dio_roles(transport)[0] == role


def test_dio_role_rejects_bad_args(transport, io_restore):
    # port 0 is the 0-based trap (web io: schema is 1-based) — must refuse.
    st, _, _, _ = _set_role(transport, 0, ROLE_OUT_PROGRAMMABLE)
    assert st != 0, "port 0 must be rejected (ports are 1-based)"
    st, _, _, _ = _set_role(transport, 3, ROLE_OUT_PROGRAMMABLE)
    assert st != 0, "port 3 must be rejected"
    st, _, _, _ = _set_role(transport, 1, 4)
    assert st != 0, "role 4 must be rejected"


# ── 0xAA role gating ─────────────────────────────────────────────────────────

def test_0xaa_auto_promotes_off_port(transport, io_restore):
    st, _, _, _ = _set_role(transport, 1, ROLE_OFF)
    assert st == 0
    st, _, payload, _ = transport.command(SET_DIGITAL_OUT_CMD, bytes([1, 1]))
    assert st == 0, f"0xAA on an off port must auto-promote: {bytes(payload)!r}"
    r1, l1, _, _ = _dio_roles(transport)
    assert r1 == ROLE_OUT_PROGRAMMABLE, "off port must promote to out_programmable"
    assert l1 == 1, "data pin must read HIGH after 0xAA state=1"
    st, _, _, _ = transport.command(SET_DIGITAL_OUT_CMD, bytes([1, 0]))
    assert st == 0
    assert _dio_roles(transport)[1] == 0, "data pin must read LOW after 0xAA state=0"


def test_0xaa_refuses_in_trigger_port(transport, io_restore):
    st, _, _, _ = _set_role(transport, 2, ROLE_IN_TRIGGER)
    assert st == 0
    st, _, payload, _ = transport.command(SET_DIGITAL_OUT_CMD, bytes([2, 1]))
    assert st != 0, "0xAA on an in_trigger port must refuse (protects the EINT route)"
    assert b"SET_DIO_ROLE" in bytes(payload), f"error must name the remedy: {bytes(payload)!r}"
    assert _dio_roles(transport)[2] == ROLE_IN_TRIGGER, "role must be unchanged after refusal"


def test_0xaa_refuses_framescan_port(transport, io_restore):
    st, _, _, _ = _set_role(transport, 1, ROLE_OUT_FRAMESCAN)
    assert st == 0
    st, _, payload, _ = transport.command(SET_DIGITAL_OUT_CMD, bytes([1, 1]))
    assert st != 0, "0xAA on a framescan port must refuse (would corrupt the scan gate)"
    assert b"SET_DIO_ROLE" in bytes(payload)


# ── AO mode (0xA3) ───────────────────────────────────────────────────────────

def test_ao_mode_guards(transport, io_restore):
    st, _, _, _ = transport.command(SET_AO_MODE_CMD, bytes([1]))
    assert st == 0, "SET_AO_MODE 1 (frame_number) refused"
    # 0xA0 and 0xA2 must refuse while frame_number owns the DAC.
    st, _, payload, _ = transport.command(SET_AO_VOLTAGE_CMD, struct.pack("<H", 2500))
    assert st != 0, "0xA0 must refuse in frame_number mode"
    assert b"SET_AO_MODE" in bytes(payload), f"error must name the remedy: {bytes(payload)!r}"
    lut = bytes([1]) + struct.pack("<H", 5) + struct.pack("<H", 2) + struct.pack("<HH", 0, 5000)
    st, _, _, _ = transport.command(SET_AO_LUT_CMD, lut)
    assert st != 0, "0xA2 must refuse in frame_number mode"
    # Back to programmable: 0xA0 works and reads back.
    st, _, _, _ = transport.command(SET_AO_MODE_CMD, bytes([0]))
    assert st == 0
    st, _, _, _ = transport.command(SET_AO_VOLTAGE_CMD, struct.pack("<H", 1234))
    assert st == 0
    assert abs(_get_ao_mv(transport) - 1234) <= _AO_TOLERANCE_MV


def test_ao_mode_rejects_bad_mode(transport, io_restore):
    st, _, _, _ = transport.command(SET_AO_MODE_CMD, bytes([2]))
    assert st != 0, "mode 2 must be rejected"


def test_ao_frame_number_tracks_position(transport, pat, io_restore):
    """The DAC must follow the frame index, 0 V = frame 0 .. 5 V = last frame.
    Mode 3 + SET_FRAME_POSITION makes it deterministic (no timing race)."""
    st, _, payload, _ = transport.command(GET_PATTERN_INFO_CMD, struct.pack("<H", pat))
    assert st == 0, "GET_PATTERN_INFO failed"
    frame_count = struct.unpack("<H", bytes(payload[:2]))[0]
    if frame_count < 8:
        pytest.skip(f"--pat has only {frame_count} frames; need >= 8 for a useful ramp")
    # Load in SHOW_FRAME mode at frame 0, then enable frame_number.
    tp = (bytes([3]) + struct.pack("<H", pat) + struct.pack("<H", 0)
          + bytes([0]) + struct.pack("<H", 0) + b"\x00\x00\x00")
    st, _, _, _ = transport.command(TRIAL_PARAMS_CMD, tp, timeout=10.0)
    assert st == 0, "trial-params mode=3 rejected"
    st, _, _, _ = transport.command(SET_AO_MODE_CMD, bytes([1]))
    assert st == 0
    try:
        for idx in (0, frame_count // 2, frame_count - 1):
            st, _, _, _ = transport.command(SET_FRAME_POSITION_CMD, struct.pack("<H", idx))
            assert st == 0, f"SET_FRAME_POSITION {idx} rejected"
            expected = round(idx * 5000 / (frame_count - 1))
            got = _get_ao_mv(transport)
            assert abs(got - expected) <= _AO_TOLERANCE_MV, (
                f"frame {idx}/{frame_count - 1}: AO={got} mV, expected ~{expected} mV")
    finally:
        transport.command(STOP_DISPLAY_CMD)


# ── Analog in (0xA4) ─────────────────────────────────────────────────────────

def test_get_analog_in_shape(transport, io_restore):
    st, echo, payload, _ = transport.command(GET_ANALOG_IN_CMD)
    assert st == 0, "GET_ANALOG_IN failed"
    assert echo == GET_ANALOG_IN_CMD
    assert len(payload) == 4, f"expected two int16 LE mV, got {len(payload)}B"
    a1, a2 = struct.unpack("<hh", bytes(payload))
    # Front-end maps ±10 V full scale; calibration TBD — range-check only
    # (floating inputs read an arbitrary mid value).
    assert -10500 <= a1 <= 10500, f"Analog In 1 out of range: {a1} mV"
    assert -10500 <= a2 <= 10500, f"Analog In 2 out of range: {a2} mV"
