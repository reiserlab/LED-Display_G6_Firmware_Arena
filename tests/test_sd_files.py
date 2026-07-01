"""SD card file operation tests: upload, rename, download, delete.

Transport-agnostic — runs under both serial and TCP via --transport.
Each test starts with a clean SD (DELETE_ALL_PATTERNS) via the autouse fixture.

Upload/download flow:
  1. upload_file(idx=0, data)    → writes /patterns/pattern.temp (not yet listed)
  2. rename_file(idx=0, name)    → promotes temp to permanent; count becomes 1
  3. download_file(idx=1)        → streams raw file bytes back
  4. command(DELETE_PATTERN_FILE, pack("<H", 1)) → removes by 1-based index
"""

import struct
import pytest

from .commands import (
    ALL_OFF_CMD,
    DELETE_ALL_PATTERNS_CMD,
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
    """Clear all pattern files (and pattern.temp) before each test."""
    transport.command(ALL_OFF_CMD)
    # SD directory iteration + manifest write can take several seconds on a
    # populated card — give it 10 s so stale bytes don't desync the session.
    transport.command(DELETE_ALL_PATTERNS_CMD, timeout=10.0)
    yield


# ── Basic SD state ──────────────────────────────────────────────────────────

def test_delete_all_patterns_acks(transport):
    st, echo, _, _ = transport.command(DELETE_ALL_PATTERNS_CMD)
    assert st == 0
    assert echo == DELETE_ALL_PATTERNS_CMD
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


# ── Large-payload bulk roundtrip (regression guard for the 0x84/0x85 bulk bug) ─
#
# The 512-byte TEST_DATA roundtrip above never fills the USB-CDC TX FIFO, so it
# passed even on firmware where GET_PATTERN_FILE (0x84) crashed the link ("Break")
# streaming a large file (blocking Serial.write with no availableForWrite()/yield()).
# This uses a payload well above the ~80 KB threshold where that bug appears:
#   buggy firmware → the download stalls/breaks → this FAILS (timeout / short read)
#   fixed firmware → bytes return byte-for-byte   → this PASSES
LARGE_SIZE = 200 * 1024  # 200 KB, comfortably above the ~80 KB crash threshold
LARGE_DATA = (bytes(range(256)) * ((LARGE_SIZE + 255) // 256))[:LARGE_SIZE]
LARGE_NAME = "hil_large.pat"


def test_large_bulk_roundtrip(transport):
    """Upload + download a 200 KB pattern and verify byte-for-byte integrity."""
    st, _, _, _ = transport.upload_file(0, LARGE_DATA, timeout=60.0)
    assert st == 0, f"large upload (0x85) failed: status={st}"
    st, _, payload, _ = transport.rename_file(0, LARGE_NAME)
    assert st == 0
    idx = struct.unpack("<H", payload)[0]
    st, echo, data, _ = transport.download_file(idx, timeout=60.0)
    assert st == 0, f"large download (0x84) failed: status={st}"
    assert echo == GET_PATTERN_FILE_CMD
    assert len(data) == LARGE_SIZE, f"short download: {len(data)}/{LARGE_SIZE} bytes"
    assert data == LARGE_DATA, "downloaded bytes differ from uploaded"


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
