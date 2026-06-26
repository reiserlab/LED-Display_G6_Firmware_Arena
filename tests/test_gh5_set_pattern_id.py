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
    SET_FRAME_POSITION_CMD,
    SET_PATTERN_ID_CMD,
    GET_FILE_COUNT_CMD,
    GET_FRAME_POSITION_CMD,
)


def _set_pattern_id(transport, pat_id: int):
    payload = struct.pack("<H", pat_id)
    return transport.command(SET_PATTERN_ID_CMD, payload)


def _set_frame_position(transport, frame: int):
    payload = struct.pack("<H", frame)
    return transport.command(SET_FRAME_POSITION_CMD, payload)


def _get_position(transport):
    """Return (cur_frame_index, frame_count) from GET_FRAME_POSITION (0x72)."""
    st, _, payload, _ = transport.command(GET_FRAME_POSITION_CMD)
    assert st == 0, f"GET_FRAME_POSITION failed: status={st}"
    assert len(payload) >= 4, f"GET_FRAME_POSITION short payload: {bytes(payload).hex()}"
    return struct.unpack_from("<HH", bytes(payload))


# ── T1: load and show frame ───────────────────────────────────────────────────

def test_load_and_show_frame(transport, pat):
    """SET_PATTERN_ID loads the pattern parked at frame 0; SET_FRAME_POSITION moves it."""
    transport.command(ALL_OFF_CMD)
    st, _, payload, _ = _set_pattern_id(transport, pat)
    assert st == 0, f"SET_PATTERN_ID {pat} failed: status={st}"
    # Response payload echoes the pattern id as uint16 LE
    assert len(payload) >= 2, "SET_PATTERN_ID success response must echo id"
    echoed = struct.unpack_from("<H", bytes(payload[:2]))[0]
    assert echoed == pat, f"SET_PATTERN_ID echo: expected {pat}, got {echoed}"

    # Loaded pattern must be parked at frame 0.
    idx, n = _get_position(transport)
    assert idx == 0, f"SET_PATTERN_ID should park at frame 0, got {idx}"
    assert n >= 1, f"loaded pattern reports {n} frames"

    # Drive a specific frame and confirm the index actually moved there.
    target = min(2, n - 1)
    st2, _, _, _ = _set_frame_position(transport, target)
    assert st2 == 0, f"SET_FRAME_POSITION {target} failed after SET_PATTERN_ID: status={st2}"
    idx2, _ = _get_position(transport)
    assert idx2 == target, f"SET_FRAME_POSITION {target}: index is {idx2}"


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
    """After SET_PATTERN_ID (Mode 3) the frame index must not advance on its own."""
    transport.command(ALL_OFF_CMD)
    st, _, _, _ = _set_pattern_id(transport, pat)
    assert st == 0, f"SET_PATTERN_ID {pat} failed: status={st}"

    before, n = _get_position(transport)
    assert before == 0, f"SET_PATTERN_ID should park at frame 0, got {before}"
    time.sleep(0.5)
    after, _ = _get_position(transport)

    # Mode 3 has no auto-advance: frame_rate_hz_ is 0, so serviceOpenLoop is a
    # no-op and the index must be exactly where SET_PATTERN_ID left it.
    assert after == before, (
        f"frame index advanced without a command (Mode 3 should be static): "
        f"{before} → {after} (n={n})"
    )
