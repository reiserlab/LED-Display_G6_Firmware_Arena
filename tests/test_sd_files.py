"""SD card file operation tests: upload, rename, download, delete.

Transport-agnostic — runs under both serial and TCP via --transport.
Each test starts with a freshly formatted SD (PURGE_MEMORY_CMD) via the
autouse fixture.

Upload/download flow:
  1. upload_file(idx=0, data)    → writes /patterns/pattern.temp (not yet listed)
  2. rename_file(idx=0, name)    → promotes temp to permanent; count becomes 1
  3. download_file(idx=1)        → streams raw file bytes back
  4. command(DELETE_PATTERN_FILE, pack("<H", 1)) → removes by 1-based index
"""

import os
import struct
import pytest

from .commands import (
    ALL_OFF_CMD,
    PURGE_MEMORY_CMD,
    DELETE_PATTERN_FILE_CMD,
    GET_FILE_COUNT_CMD,
    GET_PATTERN_FILE_CMD,
    GET_PATTERN_FILENAME_CMD,
    SET_PATTERN_FILE_CMD,
    SET_PATTERN_FILENAME_CMD,
)

# Small synthetic payload. GET_PATTERN_FILE streams raw bytes without parsing,
# so arbitrary content is fine for upload/download roundtrip.
TEST_DATA = bytes(range(256)) * 2   # 512 bytes, each value distinctive
TEST_NAME = "hil_test.pat"


def _file_count(transport) -> int:
    _, _, payload, _ = transport.command(GET_FILE_COUNT_CMD)
    return struct.unpack("<H", payload)[0]


@pytest.fixture(autouse=True)
def clean_sd(transport):
    """Format the SD card before each test."""
    transport.command(ALL_OFF_CMD)
    # A full SD.format() is much slower than the per-file delete it replaced,
    # and scales with card capacity rather than pattern count — give it a
    # generous timeout so a large/slow card doesn't desync the session.
    transport.command(PURGE_MEMORY_CMD, timeout=30.0)
    yield


# ── Basic SD state ──────────────────────────────────────────────────────────

def test_purge_memory_acks(transport):
    st, echo, _, _ = transport.command(PURGE_MEMORY_CMD, timeout=30.0)
    assert st == 0
    assert echo == PURGE_MEMORY_CMD
    assert _file_count(transport) == 0


# ── Upload (SET_PATTERN_FILE 0x85) ──────────────────────────────────────────

def test_upload_to_temp_acks(transport):
    """idx=0 writes to pattern.temp; count stays at 0 (temp is not listed)."""
    st, echo, _, _ = transport.upload_file(0, TEST_DATA)
    assert st == 0
    assert echo == SET_PATTERN_FILE_CMD
    assert _file_count(transport) == 0


# ── Rename (SET_PATTERN_FILENAME 0x83) ──────────────────────────────────────

def test_rename_temp_makes_file_permanent(transport):
    """rename_file(0, name) promotes pattern.temp and returns the new 1-based index."""
    transport.upload_file(0, TEST_DATA)
    st, echo, payload, _ = transport.rename_file(0, TEST_NAME)
    assert st == 0
    assert echo == SET_PATTERN_FILENAME_CMD
    new_idx = struct.unpack("<H", payload)[0]
    assert new_idx == 1
    assert _file_count(transport) == 1


# ── Filename query (GET_PATTERN_FILENAME 0x82) ──────────────────────────────

def test_get_filename_after_rename(transport):
    """GET_PATTERN_FILENAME at index 1 returns the name given at rename time."""
    transport.upload_file(0, TEST_DATA)
    transport.rename_file(0, TEST_NAME)
    st, echo, payload, _ = transport.command(
        GET_PATTERN_FILENAME_CMD, struct.pack("<H", 1)
    )
    assert st == 0
    assert echo == GET_PATTERN_FILENAME_CMD
    name_len = payload[0]
    name = payload[1:1 + name_len].decode("ascii")
    assert name == TEST_NAME


def test_get_filename_index_zero_errors(transport):
    """idx=0 is not a valid pattern index for GET_PATTERN_FILENAME."""
    st, _, _, _ = transport.command(GET_PATTERN_FILENAME_CMD, struct.pack("<H", 0))
    assert st != 0


def test_get_filename_out_of_range_errors(transport):
    """An index beyond the file count returns an error."""
    st, _, _, _ = transport.command(GET_PATTERN_FILENAME_CMD, struct.pack("<H", 99))
    assert st != 0


# ── Download (GET_PATTERN_FILE 0x84) ────────────────────────────────────────

def test_download_roundtrip(transport):
    """Bytes returned by download_file exactly match the bytes that were uploaded."""
    transport.upload_file(0, TEST_DATA)
    transport.rename_file(0, TEST_NAME)
    st, echo, data, _ = transport.download_file(1)
    assert st == 0
    assert echo == GET_PATTERN_FILE_CMD
    assert len(data) == len(TEST_DATA)
    assert data == TEST_DATA


def test_download_index_zero_errors(transport):
    """idx=0 is reserved for pattern.temp and is rejected by GET_PATTERN_FILE."""
    st, echo, _, _ = transport.download_file(0)
    assert st != 0
    assert echo == GET_PATTERN_FILE_CMD


def test_download_out_of_range_errors(transport):
    """An index beyond the file count returns an error."""
    st, echo, _, _ = transport.download_file(99)
    assert st != 0
    assert echo == GET_PATTERN_FILE_CMD


# ── Bulk-transfer size sweep (happy-path regression guard) ───────────────────
#
# Round-trips a range of sizes and checks byte-for-byte integrity — a guard against
# a regression that breaks large SD bulk transfers over serial.
#
# IMPORTANT: this does NOT reproduce the pre-fix "Break" crash. That crash needs the
# host READER to stall so the controller's USB TX FIFO backs up (the condition that
# tripped the old blocking Serial.write). This test's reader drains continuously and
# the OS buffers the CDC stream, so the FIFO never backs up — the sweep PASSES on both
# buggy and fixed firmware (verified 2026-07-01). The crash reproduces in the browser
# (its read loop stalls under render/blob-save load), not over fast raw serial. Treat
# this as "large transfers still work", not "the crash is fixed" — the latter rests on
# the code review + the browser before/after.
#
# The 0x85 upload is SD-write bound (slow) but completes over raw serial — hence the
# generous timeouts. Big *console* upload throttling is a separate browser issue.
#
# Default sweep runs to 2 MB. Override (comma-separated bytes):
#   HIL_BULK_SIZES="32768,65536,131072,262144" pixi run test-serial -- --port /dev/... -k bulk_roundtrip
_DEFAULT_SIZES = [64 * 1024, 256 * 1024, 512 * 1024, 1024 * 1024, 2 * 1024 * 1024]
BULK_SWEEP_SIZES = (
    [int(x) for x in os.environ["HIL_BULK_SIZES"].split(",")]
    if os.environ.get("HIL_BULK_SIZES")
    else _DEFAULT_SIZES
)


@pytest.mark.parametrize("size", BULK_SWEEP_SIZES, ids=lambda s: f"{s // 1024}KB")
def test_bulk_roundtrip(transport, size):
    """Upload + download `size` bytes over the wire and verify byte-for-byte.

    Run the set (-k bulk_roundtrip) to sweep and see where (if anywhere) it breaks;
    run one size with e.g. -k "bulk_roundtrip and 1024KB".
    """
    data = (bytes(range(256)) * ((size + 255) // 256))[:size]
    st, _, _, _ = transport.upload_file(0, data, timeout=240.0)
    assert st == 0, f"upload (0x85) failed at {size} B: status={st}"
    st, _, payload, _ = transport.rename_file(0, f"hil_{size}.pat")
    assert st == 0
    idx = struct.unpack("<H", payload)[0]
    st, echo, got, _ = transport.download_file(idx, timeout=240.0)
    assert st == 0, f"download (0x84) failed at {size} B: status={st}"
    assert echo == GET_PATTERN_FILE_CMD
    assert len(got) == size, f"short download at {size} B: {len(got)}/{size} bytes"
    assert got == data, f"corrupt download at {size} B"


# ── Delete (DELETE_PATTERN_FILE 0x86) ───────────────────────────────────────

def test_delete_permanent_file(transport):
    """Deleting by 1-based index removes the file and decrements the count."""
    transport.upload_file(0, TEST_DATA)
    transport.rename_file(0, TEST_NAME)
    assert _file_count(transport) == 1
    st, echo, _, _ = transport.command(DELETE_PATTERN_FILE_CMD, struct.pack("<H", 1))
    assert st == 0
    assert echo == DELETE_PATTERN_FILE_CMD
    assert _file_count(transport) == 0


def test_delete_temp_file(transport):
    """DELETE_PATTERN_FILE with idx=0 removes pattern.temp."""
    transport.upload_file(0, TEST_DATA)
    st, echo, _, _ = transport.command(DELETE_PATTERN_FILE_CMD, struct.pack("<H", 0))
    assert st == 0
    assert echo == DELETE_PATTERN_FILE_CMD
    assert _file_count(transport) == 0


def test_delete_invalid_index_errors(transport):
    """Deleting an out-of-range index returns a non-zero status."""
    st, echo, _, _ = transport.command(DELETE_PATTERN_FILE_CMD, struct.pack("<H", 99))
    assert st != 0
    assert echo == DELETE_PATTERN_FILE_CMD


def test_delete_temp_when_absent_errors(transport):
    """DELETE_PATTERN_FILE idx=0 returns error if pattern.temp does not exist."""
    # clean_sd already removed everything, so pattern.temp is absent.
    st, echo, _, _ = transport.command(DELETE_PATTERN_FILE_CMD, struct.pack("<H", 0))
    assert st != 0
    assert echo == DELETE_PATTERN_FILE_CMD
