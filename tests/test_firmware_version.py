"""GET_FIRMWARE_VERSION (0xCB) — compiled-in build identity.

Every controller build embeds its git short SHA, branch, working-tree dirty
flag, and UTC build date (scripts/build_version.py -> src/Version.h). 0xCB
reports them together with the build's arena geometry so a controller can be
pinned to a build after the fact (issue #50 could not be). The Studio writes
the same reply into every run log as `run_metadata.firmware`.

Read-only and O(1): the reply is a constant, so two reads must be identical.
Gated by the same GET_CONTROLLER_INFO capability bit 7 as GET_HEALTH.

Run:
    pixi run test-serial -- --port /dev/cu.usbmodemXXX
"""

import re
import struct
import pytest
from collections import namedtuple

from .commands import GET_CONTROLLER_INFO_CMD, GET_FIRMWARE_VERSION_CMD, GET_SD_INFO_CMD, SET_SD_DIAG_CMD

# Payload layout — mirrors CommandProcessor::handleGetFirmwareVersion() /
# src/Version.h. ASCII fields are right-padded with spaces (no NUL).
FW_VERSION_FMT = "<BBBB8s10s24s"
FW_VERSION_LEN = struct.calcsize(FW_VERSION_FMT)  # 46
FW_VERSION_VER = 1

FirmwareVersion = namedtuple(
    "FirmwareVersion", ["ver", "rows", "cols", "flags", "sha", "date", "branch"]
)

# flags bits
FLAG_DIRTY = 0x01
FLAG_DEBUG = 0x02
FLAG_TELEMETRY = 0x04  # telemetry ring compiled in (0xA8/0xA9 present) — hosts gate SET_TELEMETRY on this
FLAG_CRASHREPORT = 0x08  # GET_CRASHREPORT 0xCC + GET_HEALTH ver >= 2 present — hosts gate 0xCC on this
FLAG_FREERUN_REFRESH = 0x10  # free-running refresh timer variant (0x70 never disarms/re-arms the PIT)
FLAG_SD_FASTPATH = 0x20  # contiguous O(1) seeks + same-index read skip + FRAME 26 B + GET_SD_INFO 0xCD — hosts gate 0xCD on this
FLAG_SD_DIAG = 0x40  # SET_SD_DIAG 0xCE + STATE kind 14 — hosts gate 0xCE on this

# Arena geometry compiled into this branch (constants.h
# panel_count_per_frame_row / _col): G6_2x10.
EXPECTED_ROWS = 2
EXPECTED_COLS = 10

CAP_HEALTH = 0x80  # GET_CONTROLLER_INFO capability bit 7 (gates 0xCA and 0xCB)

SHA_RE = re.compile(r"^[0-9a-f]{7,8} ?$")  # --short=8 SHA; "unknown " without git
DATE_RE = re.compile(r"^\d{4}-\d{2}-\d{2}$")


def read_firmware_version(transport) -> FirmwareVersion:
    st, echo, payload, _ = transport.command(GET_FIRMWARE_VERSION_CMD)
    assert st == 0, f"GET_FIRMWARE_VERSION failed status={st}"
    assert echo == GET_FIRMWARE_VERSION_CMD
    assert len(payload) == FW_VERSION_LEN, (
        f"expected {FW_VERSION_LEN}B payload, got {len(payload)}B"
    )
    v = FirmwareVersion._make(struct.unpack(FW_VERSION_FMT, bytes(payload)))
    return v._replace(
        sha=v.sha.decode("ascii"), date=v.date.decode("ascii"), branch=v.branch.decode("ascii")
    )


def test_firmware_version_reply_shape(transport):
    v = read_firmware_version(transport)
    assert v.ver == FW_VERSION_VER
    assert v.rows == EXPECTED_ROWS
    assert v.cols == EXPECTED_COLS
    assert 0 <= v.flags <= (FLAG_DIRTY | FLAG_DEBUG | FLAG_TELEMETRY | FLAG_CRASHREPORT | FLAG_FREERUN_REFRESH | FLAG_SD_FASTPATH | FLAG_SD_DIAG), f"undefined flag bits set: {v.flags:#04x}"
    assert v.flags & FLAG_TELEMETRY, "this build compiles in the telemetry ring; bit2 must be set"
    assert v.flags & FLAG_CRASHREPORT, "this build has GET_CRASHREPORT + GET_HEALTH v2; bit3 must be set"
    assert v.flags & FLAG_FREERUN_REFRESH, "this build has the free-running refresh timer; bit4 must be set"
    assert v.flags & FLAG_SD_FASTPATH, "this build has the SD fast path + GET_SD_INFO; bit5 must be set"
    assert v.flags & FLAG_SD_DIAG, "this build has SET_SD_DIAG; bit6 must be set"


def test_firmware_version_sha(transport):
    v = read_firmware_version(transport)
    assert len(v.sha) == 8
    assert v.sha.isascii()
    assert SHA_RE.match(v.sha) or v.sha == "unknown ", f"unexpected sha field {v.sha!r}"


def test_firmware_version_date(transport):
    v = read_firmware_version(transport)
    assert len(v.date) == 10
    assert DATE_RE.match(v.date), f"unexpected date field {v.date!r}"


def test_firmware_version_branch(transport):
    v = read_firmware_version(transport)
    assert len(v.branch) == 24
    assert v.branch.isascii() and v.branch.isprintable(), f"non-printable branch {v.branch!r}"
    # Padding is trailing spaces only; the name itself is non-empty.
    assert v.branch.rstrip(" ") != ""
    assert v.branch == v.branch.rstrip(" ").ljust(24)


def test_firmware_version_is_constant(transport):
    # A compiled-in constant: two reads are byte-identical (no counters, no clock).
    assert read_firmware_version(transport) == read_firmware_version(transport)


def test_controller_info_advertises_firmware_version(transport):
    st, echo, payload, _ = transport.command(GET_CONTROLLER_INFO_CMD)
    assert st == 0 and echo == GET_CONTROLLER_INFO_CMD
    assert len(payload) >= 2
    assert payload[1] & CAP_HEALTH, "capability bit 7 (health) must be set when 0xCB exists"


# ── GET_SD_INFO (0xCD): card identity for stall attribution ─────────────────

SD_INFO_FMT = "<BBBBII16sBB"
SD_INFO_LEN = struct.calcsize(SD_INFO_FMT)  # 30


def test_sd_info_reply_shape_and_identity(transport):
    v = read_firmware_version(transport)
    assert v.flags & FLAG_SD_FASTPATH
    st, echo, payload, _ = transport.command(GET_SD_INFO_CMD)
    assert echo == GET_SD_INFO_CMD and len(payload) == SD_INFO_LEN
    ver, flags, card_type, fat_type, sectors, bpc, cid, maint, _res = struct.unpack(SD_INFO_FMT, bytes(payload))
    assert ver == 1
    if st == 0:
        assert flags & 0x01, "status 0 means a card is mounted"
        assert flags & 0x02 and flags & 0x04, "CID and CSD are cached by SdFat at mount"
        assert card_type in (1, 2, 3)
        assert fat_type in (12, 16, 32, 64)
        assert sectors > 0 and bpc in (512 << n for n in range(8))
        assert any(cid), "CID must not be all zero"  # MID 0x00 exists in the wild (the bench card: OEM "42" SD8GB)
        assert maint == 0xFF  # SD_STATUS maintenance bits are not read by SdFat 2.1.2
    else:
        assert not (flags & 0x01)
    # O(1) and constant: two reads are byte-identical.
    assert transport.command(GET_SD_INFO_CMD)[2] == payload


def test_sd_diag_switches_round_trip(transport):
    """SET_SD_DIAG (0xCE) bits echo back and show in GET_SD_INFO byte 29; both cleared afterwards."""
    v = read_firmware_version(transport)
    assert v.flags & FLAG_SD_FASTPATH
    _, _, info0, _ = transport.command(GET_SD_INFO_CMD)
    exfat = len(info0) >= 4 and bytes(info0)[3] == 64   # fat_type byte: 64 = exFAT
    try:
        for flags in (0x01, 0x02, 0x03, 0x00):
            st, echo, payload, _ = transport.command(SET_SD_DIAG_CMD, bytes([flags]))
            if exfat and (flags & 0x01):
                assert st == 1, "legacy seek is refused on exFAT (the library flags every file contiguous there)"
                continue
            assert st == 0 and echo == SET_SD_DIAG_CMD and bytes(payload) == bytes([flags])
            _, _, info, _ = transport.command(GET_SD_INFO_CMD)
            assert bytes(info)[29] & 0x03 == flags, "GET_SD_INFO byte 29 bits 0-1 report the requested diag flags"
        st, _, _, _ = transport.command(SET_SD_DIAG_CMD, bytes([0x04]))
        assert st == 1, "reserved bits are refused"
    finally:
        transport.command(SET_SD_DIAG_CMD, bytes([0x00]))


def test_sd_diag_legacy_seek_applies_on_same_pattern_restart(transport, pat):
    """The legacy-seek arm must take effect even when the SAME pattern is restarted after STOP
    (openPattern reuses the handle only under an unchanged seek mode) — reported as byte 29 bit 2."""
    from .commands import STOP_DISPLAY_CMD, TRIAL_PARAMS_CMD
    _, _, info0, _ = transport.command(GET_SD_INFO_CMD)
    if len(info0) >= 4 and bytes(info0)[3] == 64:
        pytest.skip("exFAT volume: the legacy-seek arm is refused there by design")
    tp = (bytes([3]) + struct.pack("<H", pat) + struct.pack("<h", 0) + struct.pack("<H", 0)
          + struct.pack("<h", 0) + struct.pack("<H", 0))
    try:
        assert transport.command(TRIAL_PARAMS_CMD, tp, timeout=10.0)[0] == 0
        assert bytes(transport.command(GET_SD_INFO_CMD)[2])[29] & 0x04 == 0, "fast path applied by default"
        transport.command(STOP_DISPLAY_CMD)
        assert transport.command(SET_SD_DIAG_CMD, bytes([0x01]))[0] == 0
        assert transport.command(TRIAL_PARAMS_CMD, tp, timeout=10.0)[0] == 0   # same pattern again
        assert bytes(transport.command(GET_SD_INFO_CMD)[2])[29] & 0x04, "legacy seek APPLIED after a same-pattern restart"
        transport.command(STOP_DISPLAY_CMD)
        assert transport.command(SET_SD_DIAG_CMD, bytes([0x00]))[0] == 0
        assert transport.command(TRIAL_PARAMS_CMD, tp, timeout=10.0)[0] == 0
        assert bytes(transport.command(GET_SD_INFO_CMD)[2])[29] & 0x04 == 0, "fast path re-applied"
    finally:
        transport.command(STOP_DISPLAY_CMD)
        transport.command(SET_SD_DIAG_CMD, bytes([0x00]))
