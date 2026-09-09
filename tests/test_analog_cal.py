"""Per-board analog-input calibration (F2): GET_ANALOG_IN_RAW 0xA5, SET_ANALOG_CAL 0xA6,
GET_ANALOG_CAL 0xA7 and the 0xA4 flags byte (analog-input-plan § 3 / § 5.2).

Non-destructive by default: the record shape, the raw read, and a deadband
round trip that restores the previous value. Sampling the two points or clearing
a channel would overwrite a real calibration, so those run only with
AI_CAL_DESTRUCTIVE=1 in the environment (do that on a bench board: leave the
BNC open for the +10 V point; the 0 V point needs a ground cap — see the plan).
Persistence across a power cycle is a manual check (C2).
"""
import os
import struct

import pytest

from .commands import (
    GET_ANALOG_CAL_CMD,
    GET_ANALOG_IN_CMD,
    GET_ANALOG_IN_RAW_CMD,
    SET_ANALOG_CAL_CMD,
)

ACTION_SAMPLE_GND = 0
ACTION_SAMPLE_OPEN = 1
ACTION_SET_DEADBAND = 2
ACTION_CLEAR = 0xFF
FLAG_CH1_CAL, FLAG_CH2_CAL, FLAG_12BIT = 0x01, 0x02, 0x04
DESTRUCTIVE = os.environ.get("AI_CAL_DESTRUCTIVE") == "1"


def read_record(transport):
    st, echo, payload, _ = transport.command(GET_ANALOG_CAL_CMD)
    assert st == 0, "GET_ANALOG_CAL failed"
    assert echo == GET_ANALOG_CAL_CMD
    assert len(payload) == 18, f"expected the 18-byte record, got {len(payload)}B"
    version, adc_bits, source, flags = payload[0], payload[1], payload[2], payload[3]
    chans = []
    for i in range(2):
        valid, raw_open, raw_gnd, deadband = struct.unpack("<BHHH", bytes(payload[4 + 7 * i : 11 + 7 * i]))
        chans.append({"valid": valid, "raw_open": raw_open, "raw_gnd": raw_gnd, "deadband_mv": deadband})
    return {"version": version, "adc_bits": adc_bits, "source": source, "flags": flags, "ch": chans}


def test_get_analog_cal_shape(transport):
    rec = read_record(transport)
    assert rec["version"] == 1
    assert rec["adc_bits"] == 12, "F1 moved the raw scale to 12-bit; the record must say so"
    assert rec["source"] in (0, 1), f"source must be 0 (none) or 1 (eeprom), got {rec['source']}"
    assert not (rec["flags"] & ~0x01), f"unknown record flag bits: 0x{rec['flags']:02x}"
    for c in rec["ch"]:
        assert c["valid"] in (0, 1)
        assert c["deadband_mv"] <= 2000
        if c["valid"]:
            assert c["raw_open"] > c["raw_gnd"] + 99, "a valid channel has a real span"
            assert c["raw_open"] <= 4095 and c["raw_gnd"] <= 4095


def test_get_analog_in_raw_shape(transport):
    st, echo, payload, _ = transport.command(GET_ANALOG_IN_RAW_CMD)
    assert st == 0 and echo == GET_ANALOG_IN_RAW_CMD
    assert len(payload) == 4
    r1, r2 = struct.unpack("<HH", bytes(payload))
    assert 0 <= r1 <= 4095 and 0 <= r2 <= 4095, f"12-bit counts expected, got {r1}, {r2}"


def test_analog_in_flags_match_record(transport):
    rec = read_record(transport)
    st, _, payload, _ = transport.command(GET_ANALOG_IN_CMD)
    assert st == 0 and len(payload) == 5
    flags = payload[4]
    assert flags & FLAG_12BIT
    assert bool(flags & FLAG_CH1_CAL) == bool(rec["ch"][0]["valid"])
    assert bool(flags & FLAG_CH2_CAL) == bool(rec["ch"][1]["valid"])


def test_deadband_round_trip(transport):
    before = read_record(transport)["ch"][1]["deadband_mv"]
    try:
        st, echo, payload, _ = transport.command(SET_ANALOG_CAL_CMD, bytes([2, ACTION_SET_DEADBAND]) + struct.pack("<H", 123))
        assert st == 0 and echo == SET_ANALOG_CAL_CMD
        assert len(payload) == 18, "SET_ANALOG_CAL replies with the record"
        assert read_record(transport)["ch"][1]["deadband_mv"] == 123
        assert read_record(transport)["source"] == 1, "a save makes EEPROM the source"
    finally:
        transport.command(SET_ANALOG_CAL_CMD, bytes([2, ACTION_SET_DEADBAND]) + struct.pack("<H", before))
    assert read_record(transport)["ch"][1]["deadband_mv"] == before


def test_set_analog_cal_refused_while_displaying(transport):
    """Calibration is an inactive-state operation (it samples the Mode 4 input and
    writes EEPROM + SD): with the display running the firmware answers
    CE_DISPLAY_ACTIVE and leaves the record untouched."""
    from .commands import ALL_ON_CMD, ALL_OFF_CMD
    before = read_record(transport)
    st, _, _, _ = transport.command(ALL_ON_CMD)
    assert st == 0
    try:
        st, _, payload, _ = transport.command(SET_ANALOG_CAL_CMD, bytes([2, ACTION_SET_DEADBAND]) + struct.pack("<H", 321))
        assert st != 0, "SET_ANALOG_CAL must be refused while the display is active"
        assert b"Stop display" in bytes(payload)
    finally:
        transport.command(ALL_OFF_CMD)
    assert read_record(transport) == before


def test_set_analog_cal_rejects_bad_args(transport):
    st, _, _, _ = transport.command(SET_ANALOG_CAL_CMD, bytes([3, ACTION_SET_DEADBAND, 0, 0]))
    assert st != 0, "ch 3 must be refused"
    st, _, _, _ = transport.command(SET_ANALOG_CAL_CMD, bytes([1, 7]))
    assert st != 0, "unknown action must be refused"
    st, _, _, _ = transport.command(SET_ANALOG_CAL_CMD, bytes([1, ACTION_SET_DEADBAND]) + struct.pack("<H", 5000))
    assert st != 0, "deadband > 2000 mV must be refused"
    st, _, _, _ = transport.command(SET_ANALOG_CAL_CMD, bytes([1, ACTION_SET_DEADBAND]))
    assert st != 0, "deadband without a value must be refused"


@pytest.mark.skipif(not DESTRUCTIVE, reason="overwrites the board's calibration; set AI_CAL_DESTRUCTIVE=1")
def test_two_point_calibration_channel2(transport):
    """Bench: leave Analog In 2 OPEN for the +10 V point (the pull-up reads the
    reference), then the test pauses for the ground cap? No — pytest cannot pause,
    so this samples the open point only and checks the record stays INVALID
    (one point), then clears. The full two-point run is the Studio's job (S2)."""
    st, _, payload, _ = transport.command(SET_ANALOG_CAL_CMD, bytes([2, ACTION_SAMPLE_OPEN]))
    assert st == 0
    rec = read_record(transport)
    assert rec["ch"][1]["raw_open"] > 3000, "an open input sits near +10 V (top of the 12-bit range)"
    st, _, _, _ = transport.command(SET_ANALOG_CAL_CMD, bytes([2, ACTION_CLEAR]))
    assert st == 0
    rec = read_record(transport)
    assert rec["ch"][1]["valid"] == 0 and rec["ch"][1]["raw_open"] == 0
