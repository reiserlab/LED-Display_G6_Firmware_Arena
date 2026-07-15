"""HIL tests for cross-source transfer/mutation guards, PR #27 review point 8.

Source: reiserlab/LED-Display_G6_Firmware_Arena#27, review comment
pullrequestreview-4632788440. Findings and the fix implemented here:
debug/pr27-response.md, point 8 ("Mid-transfer cross-source mutations").

DELETE_PATTERN_FILE_CMD, PURGE_MEMORY_CMD, ALL_ON_CMD, and
TRIAL_PARAMS_CMD had no awareness of an active dl_/ul_/ar_ transfer at all,
so a second source could delete or overwrite the exact file a download was
reading, or flip the display mode while an unrelated SD transfer was still
in flight. The fix is a hybrid, not one blanket rule everywhere:

  - DELETE_PATTERN_FILE_CMD and a same-index SET_PATTERN_FILE (0x85) are
    guarded INDEX-SPECIFICALLY (patternBusy()/dl_idx_ checks): only the one
    file actually in use is refused, unrelated indices still work. An
    upload is additionally refused outright while ANY archive is active
    (archive can be reading any pattern in the library at a given moment).
  - PURGE_MEMORY_CMD, ALL_ON_CMD, and TRIAL_PARAMS_CMD are guarded
    BLANKET (any of dl_/ul_/ar_active_): "all" has no single index to
    check, and a display-mode transition isn't about a specific pattern at
    all, just whether the SD card is in use by anything right now.

  T1  test_delete_pattern_index_specific_while_downloading
        DELETE_PATTERN_FILE_CMD on the pattern being downloaded is refused;
        on an UNRELATED pattern, it still succeeds.
  T2  test_upload_index_specific_while_downloading
        SET_PATTERN_FILE to the pattern being downloaded is refused; to an
        unrelated slot (pattern.temp), it still succeeds.
  T3  test_purge_memory_rejected_while_uploading
        PURGE_MEMORY_CMD is refused while an unrelated upload runs.
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

import io
import struct
import threading
import time
import zipfile
from pathlib import Path

import pytest

from .commands import (
    ALL_OFF_CMD,
    ALL_ON_CMD,
    PURGE_MEMORY_CMD,
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


def _enc_trial_params(mode: int, pattern_id: int, frame_rate: int = 10,
                      init_pos: int = 0, gain: int = 0,
                      duration_ticks: int = 0) -> bytes:
    """Build the 11-byte trial-params payload for a [0x0C, 0x08, ...] frame.
    Wire order: mode, pattern_id, frame_rate, init_pos, gain (int16), duration."""
    return (bytes([mode])
            + struct.pack("<H", pattern_id)
            + struct.pack("<h", frame_rate)
            + struct.pack("<H", init_pos)
            + struct.pack("<h", gain)
            + struct.pack("<H", duration_ticks))


# ── T1: delete-pattern guard is index-specific ────────────────────────────────

@pytest.mark.serial_only
def test_delete_pattern_index_specific_while_downloading(transport):
    """DELETE_PATTERN_FILE_CMD on the pattern being downloaded must be
    refused; on an unrelated pattern, it must still succeed."""
    transport.command(ALL_OFF_CMD)
    idx_busy = _upload(transport, GRATING_PAT, "pr27_busy.pat")
    idx_other = _upload(transport, GRATING_PAT, "pr27_other.pat")

    # Open the TCP connection FIRST, before arming the download: it queries
    # the controller's IP over THIS SAME serial connection, and
    # processCommand()'s skip-dispatch guard (dl_active_ && dl_source_ ==
    # &serial_) means that query never gets a response once serial itself
    # owns an active transfer. The read loop then just accumulates raw
    # file bytes and can misparse them as a garbage "response", corrupting
    # the stream for every test that runs afterward.
    #
    # begin_download() (header-only) stays on the MAIN thread so arming is
    # synchronous and deterministic: dl_active_ is guaranteed true by the
    # time the probes below fire. The body is then drained on a BACKGROUND
    # thread concurrently with the probes: serviceDownload() aborts the
    # whole transfer the moment a single sendRaw() call stalls
    # (SerialManager::sendRaw's 2 s kStallTimeoutMs), and the two TCP probes
    # below, each with their own multi-second timeout, are easily enough
    # idle time on serial to trip that if nothing drains meanwhile.
    file_size, leftover = 0, b""
    tcp = _open_tcp_via_serial(transport)
    th = None
    drained = {}
    try:
        status, echo, file_size, leftover, diag = transport.begin_download(idx_busy, timeout=5.0)
        assert status == 0, f"begin_download failed: status={status}, diag={diag}"
        assert echo == 0x84

        def _drain():
            remaining = file_size - len(leftover)
            drained["body"] = transport._recv_more(remaining, timeout=60.0) if remaining > 0 else b""

        th = threading.Thread(target=_drain, daemon=True)
        th.start()

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
        if tcp is not None:
            tcp.close()
        if th is not None:
            th.join(timeout=70.0)

    if th is not None:
        assert "body" in drained, "the background drain thread never finished"
        got = leftover + drained["body"]
        assert got == GRATING_PAT.read_bytes(), "the legitimate (serial) download was corrupted"


# ── T2: upload-vs-download guard is index-specific ────────────────────────────

@pytest.mark.serial_only
def test_upload_index_specific_while_downloading(transport):
    """SET_PATTERN_FILE to the pattern being downloaded must be refused; to
    an unrelated slot (pattern.temp), it must still succeed."""
    transport.command(ALL_OFF_CMD)
    idx_busy = _upload(transport, GRATING_PAT, "pr27_busy2.pat")

    # See T1's comment: begin_download() (header-only) stays on the main
    # thread so dl_active_ is guaranteed true by the time the probes below
    # fire; a background thread then drains the body concurrently with the
    # probes so it can't stall out after ~2 s of an idle serial link
    # (SerialManager::sendRaw's kStallTimeoutMs).
    file_size, leftover = 0, b""
    tcp = _open_tcp_via_serial(transport)
    th = None
    drained = {}
    try:
        status, echo, file_size, leftover, diag = transport.begin_download(idx_busy, timeout=5.0)
        assert status == 0, f"begin_download failed: status={status}, diag={diag}"
        assert echo == 0x84

        def _drain():
            remaining = file_size - len(leftover)
            drained["body"] = transport._recv_more(remaining, timeout=60.0) if remaining > 0 else b""

        th = threading.Thread(target=_drain, daemon=True)
        th.start()

        small = GRATING_PAT.read_bytes()[:4096]

        st_busy, echo_busy, payload_busy, _ = tcp.upload_file(idx_busy, small, timeout=5.0)
        assert st_busy != 0, "overwriting the pattern being downloaded was accepted"
        assert echo_busy == SET_PATTERN_FILE_CMD
        assert b"download" in bytes(payload_busy).lower()

        st_other, echo_other, _, _ = tcp.upload_file(0, small, timeout=10.0)
        assert st_other == 0, "uploading to an UNRELATED slot (pattern.temp) was refused"
        assert echo_other == SET_PATTERN_FILE_CMD
    finally:
        if tcp is not None:
            tcp.close()
        if th is not None:
            th.join(timeout=70.0)

    if th is not None:
        assert "body" in drained, "the background drain thread never finished"
        got = leftover + drained["body"]
        assert got == GRATING_PAT.read_bytes(), "the legitimate (serial) download was corrupted"


# ── T3: purge-memory guard is blanket ────────────────────────────────────────────

@pytest.mark.serial_only
def test_purge_memory_rejected_while_uploading(transport):
    """PURGE_MEMORY_CMD must be refused while an unrelated upload is
    active, even though "all" has no single index to check against."""
    transport.command(ALL_OFF_CMD)
    _upload(transport, GRATING_PAT, "pr27_keepme.pat")

    data = SINE_PAT.read_bytes()
    upload_timeout = len(data) / (50 * 1024) + 30.0
    result = {}

    def _big_upload():
        result["r"] = transport.upload_file(0, data, timeout=upload_timeout)

    th = threading.Thread(target=_big_upload, daemon=True)
    tcp = _open_tcp_via_serial(transport)
    try:
        th.start()
        time.sleep(0.3)

        st, echo, payload, _ = tcp.command(PURGE_MEMORY_CMD, timeout=5.0)
        assert st != 0, "PURGE_MEMORY_CMD was accepted during an active upload"
        assert echo == PURGE_MEMORY_CMD
        assert b"progress" in bytes(payload).lower()
    finally:
        if tcp is not None:
            tcp.close()
        th.join(timeout=upload_timeout + 10.0)
        if "r" in result:
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

    # See T1's comment: begin_get_sd_archive() (header-only) stays on the
    # main thread so ar_active_ is guaranteed true by the time the probe
    # below fires; a background thread then drains the ZIP body concurrently
    # so it can't stall out from an idle serial link (serviceArchive()
    # mirrors serviceDownload()'s abort-on-stall discipline).
    archive_size, leftover = 0, b""
    tcp = _open_tcp_via_serial(transport)
    th = None
    drained = {}
    try:
        status, echo, archive_size, leftover, diag = transport.begin_get_sd_archive(timeout=10.0)
        assert status == 0, f"begin_get_sd_archive failed: status={status}, diag={diag}"
        assert echo == GET_SD_ARCHIVE_CMD

        def _drain():
            remaining = archive_size - len(leftover)
            drained["body"] = transport._recv_more(remaining, timeout=60.0) if remaining > 0 else b""

        th = threading.Thread(target=_drain, daemon=True)
        th.start()

        small = GRATING_PAT.read_bytes()[:4096]
        st, echo_up, payload, _ = tcp.upload_file(0, small, timeout=5.0)
        assert st != 0, "SET_PATTERN_FILE was accepted while an archive was active"
        assert echo_up == SET_PATTERN_FILE_CMD
        assert b"archive" in bytes(payload).lower()
    finally:
        if tcp is not None:
            tcp.close()
        if th is not None:
            th.join(timeout=70.0)

    if th is not None:
        assert "body" in drained, "the background drain thread never finished"
        data_ar = leftover + drained["body"]
        zf = zipfile.ZipFile(io.BytesIO(data_ar))
        assert zf.testzip() is None, "archive has a CRC mismatch"


# ── T5: ALL_ON / TRIAL_PARAMS guards are blanket ──────────────────────────────

@pytest.mark.serial_only
def test_all_on_and_trial_params_rejected_during_transfer(transport):
    """ALL_ON_CMD and TRIAL_PARAMS_CMD must both be refused while an
    unrelated download is active, even though neither targets a pattern
    index the transfer is using."""
    transport.command(ALL_OFF_CMD)
    # SINE_PAT (~8.1 MB), not GRATING_PAT (~81 KB): unlike T1/T2, this test
    # fires TWO sequential TCP probes and both must land while the download
    # is still active. An 81 KB body drains in tens of milliseconds at this
    # link's throughput, so the second probe raced the download's completion
    # (observed on the bench: ALL_ON refused, then TRIAL_PARAMS accepted
    # status-0 because dl_active_ had legitimately cleared). The 8.1 MB body
    # keeps the download active for a second-plus — a comfortable window.
    idx = _upload(transport, SINE_PAT, "pr27_display_guard.pat")

    # See T1's comment: begin_download() (header-only) stays on the main
    # thread so dl_active_ is guaranteed true by the time the probes below
    # fire; a background thread then drains the body concurrently with the
    # probes so it can't stall out after ~2 s of an idle serial link
    # (SerialManager::sendRaw's kStallTimeoutMs).
    file_size, leftover = 0, b""
    tcp = _open_tcp_via_serial(transport)
    th = None
    drained = {}
    try:
        status, echo, file_size, leftover, diag = transport.begin_download(idx, timeout=5.0)
        assert status == 0, f"begin_download failed: status={status}, diag={diag}"
        assert echo == 0x84

        def _drain():
            remaining = file_size - len(leftover)
            drained["body"] = transport._recv_more(remaining, timeout=60.0) if remaining > 0 else b""

        th = threading.Thread(target=_drain, daemon=True)
        th.start()

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
        # there's nothing to stop; just release the TCP connection and join
        # the drain thread.
        if tcp is not None:
            tcp.close()
        if th is not None:
            th.join(timeout=70.0)

    if th is not None:
        assert "body" in drained, "the background drain thread never finished"
        got = leftover + drained["body"]
        assert got == SINE_PAT.read_bytes(), "the legitimate (serial) download was corrupted"
