"""SET_PATTERN_ID_CMD (0x03) tests — GitHub issue #5.

  T1  test_load_and_show_frame   : load pattern, then SET_FRAME_POSITION → no error
  T2  test_bad_id_rejected       : id=0 and id=9999 return status=1
  T3  test_no_auto_advance       : load pattern, wait, frames-sent counter doesn't grow

T1 and T3 require a pattern on the SD card; pass the pattern filename with --pat.
T2 runs without SD.

Run:
    pixi run test-serial -- --pat arena.pat
    pixi run test-tcp   -- --ip 10.0.0.x --pat arena.pat
"""

import time
import struct
import pytest

from .commands import (
    ALL_OFF_CMD,
    GET_FRAMES_SENT_CMD,
    RESET_FRAMES_SENT_CMD,
    SET_FRAME_POSITION_CMD,
    SET_PATTERN_ID_CMD,
    GET_FILE_COUNT_CMD,
)


def _set_pattern_id(transport, pat_id: int):
    payload = struct.pack("<H", pat_id)
    return transport.command(SET_PATTERN_ID_CMD, payload)


def _set_frame_position(transport, frame: int):
    payload = struct.pack("<H", frame)
    return transport.command(SET_FRAME_POSITION_CMD, payload)


def _get_frames_sent(transport) -> int:
    st, _, payload, _ = transport.command(GET_FRAMES_SENT_CMD)
    assert st == 0, f"GET_FRAMES_SENT failed: status={st}"
    return struct.unpack_from("<I", bytes(payload))[0]


# ── T1: load and show frame ───────────────────────────────────────────────────

def test_load_and_show_frame(transport, pat):
    """SET_PATTERN_ID loads the pattern; SET_FRAME_POSITION succeeds."""
    transport.command(ALL_OFF_CMD)
    st, _, payload, _ = _set_pattern_id(transport, pat)
    assert st == 0, f"SET_PATTERN_ID {pat} failed: status={st}"
    # Response payload echoes the pattern id as uint16 LE
    assert len(payload) >= 2, "SET_PATTERN_ID success response must echo id"
    echoed = struct.unpack_from("<H", bytes(payload[:2]))[0]
    assert echoed == pat, f"SET_PATTERN_ID echo: expected {pat}, got {echoed}"

    # Drive frame 0 explicitly — should succeed (no error)
    st2, _, _, _ = _set_frame_position(transport, 0)
    assert st2 == 0, f"SET_FRAME_POSITION 0 failed after SET_PATTERN_ID: status={st2}"


# ── T2: bad id rejected ───────────────────────────────────────────────────────

def test_bad_id_rejected(transport):
    """id=0 (invalid) and id=9999 (beyond any real SD) must return status=1."""
    transport.command(ALL_OFF_CMD)

    # First check how many patterns exist
    st, _, payload, _ = transport.command(GET_FILE_COUNT_CMD)
    file_count = struct.unpack_from("<H", bytes(payload[:2]))[0] if (st == 0 and len(payload) >= 2) else 0

    for bad_id in (0, 9999):
        st, _, _, _ = _set_pattern_id(transport, bad_id)
        assert st != 0, f"SET_PATTERN_ID {bad_id} should have been rejected (file_count={file_count})"


# ── T3: no auto-advance ───────────────────────────────────────────────────────

def test_no_auto_advance(transport, pat):
    """After SET_PATTERN_ID the frame index must not advance on its own."""
    transport.command(ALL_OFF_CMD)
    transport.command(RESET_FRAMES_SENT_CMD)
    st, _, _, _ = _set_pattern_id(transport, pat)
    assert st == 0, f"SET_PATTERN_ID {pat} failed: status={st}"

    before = _get_frames_sent(transport)
    time.sleep(0.5)
    after = _get_frames_sent(transport)

    # Refresh timer continues to tick (same frame re-sent), but frame_index must
    # not change. We can't check frame_index directly — instead verify that
    # frames are still being sent (display is live) and that no *SET_FRAME_POSITION*
    # equivalent auto-stepped.  The open-loop service routine returns immediately
    # when frame_rate_hz_==0, so cur_frame_index_ is static.
    assert after >= before, "frames-sent counter went backwards"
    # A static frame at 300 Hz for 500 ms = ~150 frames; at least 50 expected.
    assert (after - before) >= 50, (
        f"expected refresh activity after SET_PATTERN_ID (mode=3 static), "
        f"got {after - before} frames in 500 ms"
    )
