"""HIL tests for cross-source transfer/mutation guards, PR #27 review point 8.

Source: reiserlab/LED-Display_G6_Firmware_Arena#27, review comment
pullrequestreview-4632788440. Findings and the fix implemented here:
debug/pr27-response.md, point 8 ("Mid-transfer cross-source mutations").

DELETE_PATTERN_FILE_CMD, DELETE_ALL_PATTERNS_CMD, ALL_ON_CMD, and
TRIAL_PARAMS_CMD had no awareness of an active dl_/ul_/ar_ transfer at all,
so a second source could delete or overwrite the exact file a download was
reading, or flip the display mode while an unrelated SD transfer was still
in flight. The fix is a hybrid, not one blanket rule everywhere:

  - DELETE_PATTERN_FILE_CMD and a same-index SET_PATTERN_FILE (0x85) are
    guarded INDEX-SPECIFICALLY (patternBusy()/dl_idx_ checks): only the one
    file actually in use is refused, unrelated indices still work. An
    upload is additionally refused outright while ANY archive is active
    (archive can be reading any pattern in the library at a given moment).
  - DELETE_ALL_PATTERNS_CMD, ALL_ON_CMD, and TRIAL_PARAMS_CMD are guarded
    BLANKET (any of dl_/ul_/ar_active_): "all" has no single index to
    check, and a display-mode transition isn't about a specific pattern at
    all, just whether the SD card is in use by anything right now.

  T1  test_delete_pattern_index_specific_while_downloading
        DELETE_PATTERN_FILE_CMD on the pattern being downloaded is refused;
        on an UNRELATED pattern, it still succeeds.
  T2  test_upload_index_specific_while_downloading
        SET_PATTERN_FILE to the pattern being downloaded is refused; to an
        unrelated slot (pattern.temp), it still succeeds.
  T3  test_delete_all_rejected_while_uploading
        DELETE_ALL_PATTERNS_CMD is refused while an unrelated upload runs.
  T4  test_upload_rejected_while_archiving
        SET_PATTERN_FILE is refused outright while an archive is active.
  T5  test_all_on_and_trial_params_rejected_during_transfer
        ALL_ON_CMD and TRIAL_PARAMS_CMD are both refused while an unrelated
        download is active.

All five drive the "busy" transfer over serial and fire the guarded command
on an independent TCP connection.

Run:
    pixi run test-serial -- -k pr27_transfer_mutation
"""

import struct
import threading
import time
from pathlib import Path

import pytest

from .commands import (
    ALL_OFF_CMD,
    ALL_ON_CMD,
    DELETE_ALL_PATTERNS_CMD,
    DELETE_PATTERN_FILE_CMD,
    GET_ETHERNET_IP_ADDRESS_CMD,
    GET_SD_ARCHIVE_CMD,
    SET_PATTERN_FILE_CMD,
    TRIAL_PARAMS_CMD,
)
from .transport import TCP_PORT, TcpTransport

DATA_DIR    = Path(__file__).parent / "data"
GRATING_PAT = DATA_DIR / "G6_2x10_grating_rotation_20px_50pct.pat"
SINE_PAT    = DATA_DIR / "G6_2x10_sine_rotation_200px.pat"  # ~8.1 MB


def _open_tcp_via_serial(transport) -> TcpTransport:
    """Auto-discover the controller's own IP over the serial link already in
    use (GET_ETHERNET_IP_ADDRESS_CMD, 0xC1) and open an independent TCP
    connection to it. Caller must close(). Ported from test_gh16_bulk_download.py."""
    st_ip, echo_ip, ip_payload, _ = transport.command(GET_ETHERNET_IP_ADDRESS_CMD, timeout=2.0)
    assert st_ip == 0, "could not read controller IP over serial"
    assert echo_ip == GET_ETHERNET_IP_ADDRESS_CMD
    ip = ip_payload.decode("ascii").rstrip("\x00")
    assert ip, "controller reported an empty IP address (no ethernet link?)"
    tcp = TcpTransport(ip, TCP_PORT)
    tcp.open()
    return tcp


def _upload(transport, path: Path, name: str) -> int:
    """Upload path's bytes to pattern.temp, promote to `name`, return 1-based idx."""
    data = path.read_bytes()
    st, _, _, _ = transport.upload_file(0, data, timeout=30.0)
    assert st == 0, f"setup: upload of {path.name} failed: status={st}"
    st2, _, payload, _ = transport.rename_file(0, name)
    assert st2 == 0, f"setup: rename of {path.name} failed: status={st2}"
    return struct.unpack("<H", payload)[0]


def _enc_trial_params(mode: int, pattern_id: int,
                      frame_rate: int = 10, gain: int = 0, init_pos: int = 0) -> bytes:
    """Build the 11-byte trial-params payload for a [0x0C, 0x08, ...] frame."""
    return (bytes([mode])
            + struct.pack("<H", pattern_id)
            + struct.pack("<H", frame_rate)
            + bytes([gain & 0xFF])
            + struct.pack("<H", init_pos)
            + b"\x00\x00\x00")


def _drain_download(transport, file_size: int, leftover: bytes, timeout: float = 30.0):
    """Finish a begin_download()'d transfer so dl_active_ clears cleanly,
    instead of leaving it to time out (kDownloadIdleTimeoutMs = 60 s)."""
    remaining = file_size - len(leftover)
    if remaining > 0:
        transport._recv_more(remaining, timeout)


# ── T1: delete-pattern guard is index-specific ────────────────────────────────

@pytest.mark.serial_only
def test_delete_pattern_index_specific_while_downloading(transport):
    """DELETE_PATTERN_FILE_CMD on the pattern being downloaded must be
    refused; on an unrelated pattern, it must still succeed."""
    transport.command(ALL_OFF_CMD)
    idx_busy = _upload(transport, GRATING_PAT, "pr27_busy.pat")
    idx_other = _upload(transport, GRATING_PAT, "pr27_other.pat")

    status, echo, file_size, leftover, diag = transport.begin_download(idx_busy, timeout=5.0)
    assert status == 0, f"begin_download failed: status={status}, diag={diag}"
    assert echo == 0x84

    tcp = _open_tcp_via_serial(transport)
    try:
        st_busy, echo_busy, payload_busy, _ = tcp.command(
            DELETE_PATTERN_FILE_CMD, struct.pack("<H", idx_busy), timeout=5.0
        )
        assert st_busy != 0, "deleting the pattern being downloaded was accepted"
        assert echo_busy == DELETE_PATTERN_FILE_CMD
        assert b"use" in bytes(payload_busy).lower()

        st_other, echo_other, _, _ = tcp.command(
            DELETE_PATTERN_FILE_CMD, struct.pack("<H", idx_other), timeout=5.0
        )
        assert st_other == 0, "deleting an UNRELATED pattern was refused"
        assert echo_other == DELETE_PATTERN_FILE_CMD
    finally:
        tcp.close()
        _drain_download(transport, file_size, leftover)


# ── T2: upload-vs-download guard is index-specific ────────────────────────────

@pytest.mark.serial_only
def test_upload_index_specific_while_downloading(transport):
    """SET_PATTERN_FILE to the pattern being downloaded must be refused; to
    an unrelated slot (pattern.temp), it must still succeed."""
    transport.command(ALL_OFF_CMD)
    idx_busy = _upload(transport, GRATING_PAT, "pr27_busy2.pat")

    status, echo, file_size, leftover, diag = transport.begin_download(idx_busy, timeout=5.0)
    assert status == 0, f"begin_download failed: status={status}, diag={diag}"
    assert echo == 0x84

    tcp = _open_tcp_via_serial(transport)
    try:
        small = GRATING_PAT.read_bytes()[:4096]

        st_busy, echo_busy, payload_busy, _ = tcp.upload_file(idx_busy, small, timeout=5.0)
        assert st_busy != 0, "overwriting the pattern being downloaded was accepted"
        assert echo_busy == SET_PATTERN_FILE_CMD
        assert b"download" in bytes(payload_busy).lower()

        st_other, echo_other, _, _ = tcp.upload_file(0, small, timeout=10.0)
        assert st_other == 0, "uploading to an UNRELATED slot (pattern.temp) was refused"
        assert echo_other == SET_PATTERN_FILE_CMD
    finally:
        tcp.close()
        _drain_download(transport, file_size, leftover)


# ── T3: delete-all guard is blanket ────────────────────────────────────────────

@pytest.mark.serial_only
def test_delete_all_rejected_while_uploading(transport):
    """DELETE_ALL_PATTERNS_CMD must be refused while an unrelated upload is
    active, even though "all" has no single index to check against."""
    transport.command(ALL_OFF_CMD)
    _upload(transport, GRATING_PAT, "pr27_keepme.pat")

    data = SINE_PAT.read_bytes()
    upload_timeout = len(data) / (50 * 1024) + 30.0
    result = {}

    def _big_upload():
        result["r"] = transport.upload_file(0, data, timeout=upload_timeout)

    th = threading.Thread(target=_big_upload, daemon=True)
    th.start()
    time.sleep(0.3)

    tcp = _open_tcp_via_serial(transport)
    try:
        st, echo, payload, _ = tcp.command(DELETE_ALL_PATTERNS_CMD, timeout=5.0)
        assert st != 0, "DELETE_ALL_PATTERNS_CMD was accepted during an active upload"
        assert echo == DELETE_ALL_PATTERNS_CMD
        assert b"progress" in bytes(payload).lower()
    finally:
        tcp.close()
        th.join(timeout=upload_timeout + 10.0)
        st_up, echo_up, _, diag_up = result["r"]
        assert st_up == 0, f"the unrelated upload itself failed: status={st_up}, diag={diag_up}"
        assert echo_up == SET_PATTERN_FILE_CMD


# ── T4: upload guard against an active archive is blanket ────────────────────

@pytest.mark.serial_only
def test_upload_rejected_while_archiving(transport):
    """SET_PATTERN_FILE must be refused outright while an archive is
    active, since it can be reading any pattern in the library."""
    transport.command(ALL_OFF_CMD)
    _upload(transport, GRATING_PAT, "pr27_archive_src.pat")

    status, echo, archive_size, leftover, diag = transport.begin_get_sd_archive(timeout=5.0)
    assert status == 0, f"begin_get_sd_archive failed: status={status}, diag={diag}"
    assert echo == GET_SD_ARCHIVE_CMD

    tcp = _open_tcp_via_serial(transport)
    try:
        small = GRATING_PAT.read_bytes()[:4096]
        st, echo_up, payload, _ = tcp.upload_file(0, small, timeout=5.0)
        assert st != 0, "SET_PATTERN_FILE was accepted while an archive was active"
        assert echo_up == SET_PATTERN_FILE_CMD
        assert b"archive" in bytes(payload).lower()
    finally:
        tcp.close()
        remaining = archive_size - len(leftover)
        if remaining > 0:
            transport._recv_more(remaining, timeout=30.0)


# ── T5: ALL_ON / TRIAL_PARAMS guards are blanket ──────────────────────────────

@pytest.mark.serial_only
def test_all_on_and_trial_params_rejected_during_transfer(transport):
    """ALL_ON_CMD and TRIAL_PARAMS_CMD must both be refused while an
    unrelated download is active, even though neither targets a pattern
    index the transfer is using."""
    transport.command(ALL_OFF_CMD)
    idx = _upload(transport, GRATING_PAT, "pr27_display_guard.pat")

    status, echo, file_size, leftover, diag = transport.begin_download(idx, timeout=5.0)
    assert status == 0, f"begin_download failed: status={status}, diag={diag}"
    assert echo == 0x84

    tcp = _open_tcp_via_serial(transport)
    try:
        st_on, echo_on, payload_on, _ = tcp.command(ALL_ON_CMD, timeout=5.0)
        assert st_on != 0, "ALL_ON_CMD was accepted during an active download"
        assert echo_on == ALL_ON_CMD
        assert b"progress" in bytes(payload_on).lower()

        params = _enc_trial_params(mode=2, pattern_id=idx, frame_rate=10)
        st_tp, echo_tp, payload_tp, _ = tcp.command(TRIAL_PARAMS_CMD, params, timeout=5.0)
        assert st_tp != 0, "TRIAL_PARAMS_CMD was accepted during an active download"
        assert echo_tp == TRIAL_PARAMS_CMD
        assert b"progress" in bytes(payload_tp).lower()
    finally:
        # Both guarded commands were refused, so state_ never changed and
        # there's nothing to stop; just release the TCP connection and the
        # download's dl_active_.
        tcp.close()
        _drain_download(transport, file_size, leftover)
