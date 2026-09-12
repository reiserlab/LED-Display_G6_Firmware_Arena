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
from collections import namedtuple

from .commands import GET_CONTROLLER_INFO_CMD, GET_FIRMWARE_VERSION_CMD

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
    assert 0 <= v.flags <= (FLAG_DIRTY | FLAG_DEBUG | FLAG_TELEMETRY), f"undefined flag bits set: {v.flags:#04x}"
    assert v.flags & FLAG_TELEMETRY, "this build compiles in the telemetry ring; bit2 must be set"


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
