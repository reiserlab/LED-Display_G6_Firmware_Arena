"""GET_PATTERN_INFO (0x88) tests — cheap pattern metadata for preview (issue #18).

  T1  info_matches_header : upload --pat, call 0x88, assert every field matches
                            a host-side parse of the file (frame_count, gs, rows,
                            cols, arena_id, observer_id, file_size, stretch).
  T2  bad_index_rejected  : 0x88 with an out-of-range index returns status != 0.

0x88 is a small framed reply (12-byte payload) — it does NOT use the 0x84 bulk
path, so there are no stall/timeout concerns. T1 needs a real G6PT v2 pattern
(pass one with --pat); T2 runs against any connected controller.

Run:
    pixi run test-serial -- --pat path/to/pattern.pat
    pixi run test-tcp -- --ip 10.0.0.x --pat path/to/pattern.pat
"""

import struct

import pytest

from .commands import GET_PATTERN_INFO_CMD

# Wire layout of the 0x88 payload (little-endian), mirroring the field table in
# issue #18: frame_count u16 · gs u8 · rows u8 · cols u8 · arena u8 · observer u8
# · file_size u32 · stretch u8.
_PAYLOAD_FMT = "<HBBBBBIB"
_PAYLOAD_LEN = struct.calcsize(_PAYLOAD_FMT)  # 12

# G6PT v2 layout constants (see src/constants.h / g6_04-pattern-file-format.md).
_HEADER_LEN = 18
_FRAME_PREFIX_LEN = 4          # "FR" + frame index (LE16)
_BLOCK_GS2 = 53
_BLOCK_GS16 = 203


def _parse_header(data: bytes) -> dict:
    """Host-side parse of a G6PT v2 header + frame-0 panel-0 stretch byte."""
    assert data[:4] == b"G6PT", "--pat file is not a G6PT pattern"
    assert (data[4] >> 4) & 0x0F == 2, "--pat file is not header version 2"
    gs = data[10]
    block = _BLOCK_GS2 if gs == 1 else _BLOCK_GS16
    stretch_off = _HEADER_LEN + _FRAME_PREFIX_LEN + block - 1
    return {
        "frame_count": struct.unpack_from("<H", data, 6)[0],
        "gs_val": gs,
        "rows": data[8],
        "cols": data[9],
        "arena_id": ((data[4] & 0x0F) << 2) | (data[5] >> 6),
        "observer_id": data[5] & 0x3F,
        "file_size": len(data),
        "stretch": data[stretch_off],
    }


def test_info_matches_header(transport, pat, pat_data):
    """0x88 fields must match a host-side parse of the uploaded --pat file."""
    expected = _parse_header(pat_data)

    st, echo, payload, _ = transport.command(GET_PATTERN_INFO_CMD, struct.pack("<H", pat))
    assert st == 0, f"GET_PATTERN_INFO failed: status={st}"
    assert echo == GET_PATTERN_INFO_CMD
    assert len(payload) == _PAYLOAD_LEN, f"expected {_PAYLOAD_LEN}-byte payload, got {len(payload)}"

    (frame_count, gs_val, rows, cols, arena_id,
     observer_id, file_size, stretch) = struct.unpack(_PAYLOAD_FMT, bytes(payload))

    assert frame_count == expected["frame_count"]
    assert gs_val == expected["gs_val"]
    assert rows == expected["rows"]
    assert cols == expected["cols"]
    assert arena_id == expected["arena_id"]
    assert observer_id == expected["observer_id"]
    assert file_size == expected["file_size"]
    assert stretch == expected["stretch"]


def test_bad_index_rejected(transport):
    """An out-of-range pattern index must return a non-zero status, not a payload."""
    st, echo, payload, _ = transport.command(GET_PATTERN_INFO_CMD, struct.pack("<H", 0xFFFF))
    assert st != 0, "expected error status for out-of-range index"
    assert echo == GET_PATTERN_INFO_CMD
