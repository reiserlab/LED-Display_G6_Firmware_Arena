"""Transport-agnostic command tests.

Each test runs under whichever transport is selected via --transport. The full
suite runs twice (once per transport) via `pixi run test`.
"""

import struct

from .commands import (
    ALL_OFF_CMD,
    ALL_ON_CMD,
    GET_CONTROLLER_INFO_CMD,
    GET_DIAG_OUTPUT_CMD,
    GET_FILE_COUNT_CMD,
    GET_FRAME_POSITION_CMD,
    GET_FRAMES_SENT_CMD,
    GET_REFRESH_RATE_CMD,
    GET_SPI_CLOCK_CMD,
    RESET_FRAMES_SENT_CMD,
    SET_REFRESH_RATE_CMD,
    SET_SPI_CLOCK_CMD,
    STOP_DISPLAY_CMD,
    SWITCH_GRAYSCALE_CMD,
)


def test_all_off_acks(transport):
    st, echo, _, _ = transport.command(ALL_OFF_CMD)
    assert st == 0
    assert echo == ALL_OFF_CMD


def test_all_on_acks(transport):
    st, echo, _, _ = transport.command(ALL_ON_CMD)
    assert st == 0
    assert echo == ALL_ON_CMD


def test_stop_display_acks(transport):
    st, echo, _, _ = transport.command(STOP_DISPLAY_CMD)
    assert st == 0
    assert echo == STOP_DISPLAY_CMD


def test_get_controller_info(transport):
    # {version, capability, mac[6]} — MAC bytes are the tolerant extension
    # (webDisplayTools #135); capability bit 5 = io_ext (0xAC/0xAD/0xA3/0xA4).
    st, echo, payload, _ = transport.command(GET_CONTROLLER_INFO_CMD)
    assert st == 0
    assert echo == GET_CONTROLLER_INFO_CMD
    assert len(payload) == 8
    version, capability = payload[0], payload[1]
    assert version == 1
    assert capability & 0x01, "g6_mode bit must be set"
    assert capability & 0x20, "io_ext bit must be set (SET_DIO_ROLE et al.)"
    mac = payload[2:8]
    assert any(b != 0 for b in mac), "MAC must be non-zero (Teensy fuses)"
    assert any(b != 0xFF for b in mac), "MAC must not be all-FF"


def test_get_refresh_rate(transport):
    st, echo, payload, _ = transport.command(GET_REFRESH_RATE_CMD)
    assert st == 0
    assert echo == GET_REFRESH_RATE_CMD
    assert len(payload) == 2
    assert struct.unpack("<H", payload)[0] > 0


def test_round_trip_refresh_rate(transport):
    # Restore the pre-test rate — these are LIVE runtime settings on a bench
    # rig, and leaving 60 Hz behind masquerades as a wrong-build mystery.
    # (Caveat: restoring via SET keeps the firmware's refresh_rate_explicit_
    # flag latched until reboot — value is right, auto-default is disabled.)
    st0, _, p0, _ = transport.command(GET_REFRESH_RATE_CMD)
    assert st0 == 0
    before = struct.unpack("<H", p0)[0]
    try:
        target = 60
        transport.command(SET_REFRESH_RATE_CMD, struct.pack("<H", target))
        st, _, payload, _ = transport.command(GET_REFRESH_RATE_CMD)
        assert st == 0
        assert struct.unpack("<H", payload)[0] == target
    finally:
        transport.command(SET_REFRESH_RATE_CMD, struct.pack("<H", before))


def test_get_spi_clock(transport):
    st, echo, payload, _ = transport.command(GET_SPI_CLOCK_CMD)
    assert st == 0
    assert echo == GET_SPI_CLOCK_CMD
    assert len(payload) == 2


def test_round_trip_spi_clock(transport):
    # Restore the pre-test clock (see refresh-rate test — bench rigs keep
    # running after the suite; don't leave them at a diagnostic 10 MHz).
    st0, _, p0, _ = transport.command(GET_SPI_CLOCK_CMD)
    assert st0 == 0
    before = struct.unpack("<H", p0)[0]
    try:
        target = 10  # MHz
        st, _, payload, _ = transport.command(SET_SPI_CLOCK_CMD, struct.pack("<H", target))
        assert st == 0
        assert len(payload) == 2
        applied = struct.unpack("<H", payload)[0]
        st2, _, payload2, _ = transport.command(GET_SPI_CLOCK_CMD)
        assert st2 == 0
        assert struct.unpack("<H", payload2)[0] == applied
    finally:
        transport.command(SET_SPI_CLOCK_CMD, struct.pack("<H", before))


def test_get_frames_sent(transport):
    st, echo, payload, _ = transport.command(GET_FRAMES_SENT_CMD)
    assert st == 0
    assert echo == GET_FRAMES_SENT_CMD
    assert len(payload) == 4


def test_reset_frames_sent(transport):
    transport.command(ALL_OFF_CMD)
    st, echo, _, _ = transport.command(RESET_FRAMES_SENT_CMD)
    assert st == 0
    assert echo == RESET_FRAMES_SENT_CMD
    _, _, payload, _ = transport.command(GET_FRAMES_SENT_CMD)
    assert struct.unpack("<I", payload)[0] == 0


def test_get_diag_output(transport):
    st, echo, payload, _ = transport.command(GET_DIAG_OUTPUT_CMD)
    assert st == 0
    assert echo == GET_DIAG_OUTPUT_CMD
    assert len(payload) == 1
    assert payload[0] in (0, 1)


def test_get_file_count(transport):
    st, echo, payload, _ = transport.command(GET_FILE_COUNT_CMD)
    assert st == 0
    assert echo == GET_FILE_COUNT_CMD
    assert len(payload) == 2


def test_get_frame_position_shape(transport):
    """get-frame-position returns a well-formed, self-consistent (idx, count).

    Does not assume a fresh boot: the device persists state across runs and
    ALL_OFF does not clear the open pattern, so we only assert the response
    shape and that the index is within the reported frame count.
    """
    st, echo, payload, _ = transport.command(GET_FRAME_POSITION_CMD)
    assert st == 0
    assert echo == GET_FRAME_POSITION_CMD
    assert len(payload) == 4
    idx, count = struct.unpack("<HH", bytes(payload))
    if count == 0:
        assert idx == 0, f"count=0 but idx={idx}"
    else:
        assert idx < count, f"idx {idx} out of range for count {count}"


def test_switch_grayscale_dropped(transport):
    st, echo, _, _ = transport.command(SWITCH_GRAYSCALE_CMD)
    assert st == 1
    assert echo == SWITCH_GRAYSCALE_CMD
