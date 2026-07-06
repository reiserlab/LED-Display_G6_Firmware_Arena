"""HIL tests for the bulk-write consume fix, PR #27 review point 1.

Source: reiserlab/LED-Display_G6_Firmware_Arena#27, review comment
pullrequestreview-4632788440. Findings and the fix implemented here:
debug/pr27-response.md, point 1 ("Deferred-consume gate checks the global
ul_active_ instead of ownership").

Before this fix, processCommand() consumed a bulk command with
`if (!ul_active_) source.commandConsumed();`: the GLOBAL flag, not "is this
source the one doing the upload." A rejected SET_PATTERN_FILE (0x85) on
source B, while source A's upload made ul_active_ true, was never consumed,
so handleBulkWriteCommand() ran again on every loop() iteration: same
rejection, same drainBulkData() call, same duplicate error sent to B, for as
long as A's upload lasted. The SET_FIRMWARE_FILE (0xE0) path was worse and
had no guard against a concurrent transfer at all: it isn't migrated onto
the async chunked model, so it also risked starving (and false-timing-out) a
concurrent 0x84/0x85/0x8A transfer on the other source by blocking loop() for
its own full duration.

The fix: handleBulkWriteCommand()/handleSetFirmwareFile() now return whether
THIS call handed off to an async service loop; processCommand() consumes
based on that return value, not a re-check of ul_active_. handleSetFirmwareFile()
also gained an upfront guard rejecting the request outright while ANY of
dl_/ul_/ar_active_ is set.

  T1  test_rejected_upload_consumed_once_not_redispatched
        A rejected 0x85 on an independent connection, while an unrelated
        upload is active elsewhere, is rejected exactly once, not
        redispatched every loop() iteration.
  T2  test_firmware_upload_rejected_while_transfer_active
        A 0xE0 firmware-upload attempt, while an unrelated upload is active
        elsewhere, is rejected exactly once and never runs (no CRC success
        response), instead of blocking loop() and risking that transfer's
        timing.

Both drive the "busy" transfer over serial and fire the second, rejected
request on an independent TCP connection, then watch the RAW byte stream for
extra, unsolicited response frames (the redispatch bug's signature) rather
than trusting the single-frame-returning transport helpers, which would
silently discard any extra buffered frames.

Run:
    pixi run test-serial -- -k pr27_upload
"""

import socket
import struct
import threading
import time
from pathlib import Path

import pytest

from .commands import (
    ALL_OFF_CMD,
    GET_ETHERNET_IP_ADDRESS_CMD,
    SET_FIRMWARE_FILE_CMD,
    SET_PATTERN_FILE_CMD,
)
from .transport import TCP_PORT, TcpTransport, _parse_response_ex

DATA_DIR = Path(__file__).parent / "data"
SINE_PAT = DATA_DIR / "G6_2x10_sine_rotation_200px.pat"  # ~8.1 MB; see test_gh16_bulk_download.py


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


def _recv_for(tcp: TcpTransport, duration: float) -> bytes:
    """Accumulate every raw byte tcp receives over `duration` seconds
    (not just the first response frame). Used to prove nothing MORE than
    the one expected response arrives."""
    end = time.monotonic() + duration
    buf = bytearray()
    while True:
        remaining = end - time.monotonic()
        if remaining <= 0:
            break
        tcp._sock.settimeout(min(remaining, 1.0))
        try:
            chunk = tcp._sock.recv(4096)
            if chunk:
                buf.extend(chunk)
        except socket.timeout:
            pass
    return bytes(buf)


def _count_frames(buf: bytes) -> int:
    """Walk buf and count how many complete response frames it holds."""
    count = 0
    i = 0
    while i < len(buf):
        try:
            _, _, _, _, consumed = _parse_response_ex(buf[i:])
        except ValueError:
            break
        count += 1
        i += consumed
    return count


# ── T1: rejected 0x85 must not be redispatched ────────────────────────────────

@pytest.mark.serial_only
def test_rejected_upload_consumed_once_not_redispatched(transport):
    """A rejected SET_PATTERN_FILE (0x85), arriving on an independent
    connection while an unrelated upload is active on serial, must be
    rejected exactly once."""
    transport.command(ALL_OFF_CMD)

    # A real, slow-enough upload on serial so ul_active_ stays true for a
    # comfortable multi-second window (SINE_PAT is ~8.1 MB; matches
    # test_tcp_stays_responsive_during_serial_download's use of the same
    # file for the same reason: guaranteed overlap with the test window).
    data = SINE_PAT.read_bytes()
    upload_timeout = len(data) / (50 * 1024) + 30.0  # floor throughput + margin
    result = {}

    def _upload():
        result["r"] = transport.upload_file(0, data, timeout=upload_timeout)

    th = threading.Thread(target=_upload, daemon=True)
    th.start()
    time.sleep(0.3)  # let ul_active_ actually become true before firing T1's probe

    tcp = _open_tcp_via_serial(transport)
    try:
        rejected_payload = b"\x00" * 16
        header = bytes([SET_PATTERN_FILE_CMD]) + struct.pack("<HQ", 0, len(rejected_payload))
        tcp._send(header + rejected_payload)

        # Old (buggy) firmware redispatches the un-consumed rejection every
        # loop() iteration, producing a steady stream of duplicate "Upload
        # already in progress" frames for as long as the other upload runs.
        # This window is comfortably longer than one such redispatch cycle
        # needs to prove the point, and short relative to the real upload.
        raw = _recv_for(tcp, 4.0)
        frame_count = _count_frames(raw)
        assert frame_count == 1, (
            f"expected exactly one rejection response, got {frame_count} "
            f"frames in {len(raw)} bytes ({raw!r}); looks redispatched"
        )
        status, echo, _payload, _diag, _consumed = _parse_response_ex(raw)
        assert status != 0, "second concurrent upload attempt was accepted"
        assert echo == SET_PATTERN_FILE_CMD
    finally:
        tcp.close()

    th.join(timeout=upload_timeout + 10.0)
    st, echo, _, _ = result["r"]
    assert st == 0, f"legitimate (serial) upload failed: status={st}"
    assert echo == SET_PATTERN_FILE_CMD


# ── T2: 0xE0 must be refused, not run, while another transfer is active ──────

@pytest.mark.serial_only
def test_firmware_upload_rejected_while_transfer_active(transport):
    """A SET_FIRMWARE_FILE (0xE0) attempt, arriving on an independent
    connection while an unrelated upload is active on serial, must be
    rejected outright (not run, not redispatched) so it can't block loop()
    and starve the other transfer's idle deadline."""
    transport.command(ALL_OFF_CMD)

    data = SINE_PAT.read_bytes()
    upload_timeout = len(data) / (50 * 1024) + 30.0
    result = {}

    def _upload():
        result["r"] = transport.upload_file(0, data, timeout=upload_timeout)

    th = threading.Thread(target=_upload, daemon=True)
    th.start()
    time.sleep(0.3)

    tcp = _open_tcp_via_serial(transport)
    try:
        fw_payload = b"\x00" * 8  # content is irrelevant: the guard fires before any I/O
        header = bytes([SET_FIRMWARE_FILE_CMD]) + struct.pack("<Q", len(fw_payload))
        tcp._send(header + fw_payload)

        raw = _recv_for(tcp, 4.0)
        frame_count = _count_frames(raw)
        assert frame_count == 1, (
            f"expected exactly one rejection response, got {frame_count} "
            f"frames in {len(raw)} bytes ({raw!r}); looks redispatched"
        )
        status, echo, _payload, _diag, _consumed = _parse_response_ex(raw)
        assert status != 0, (
            "firmware upload was accepted while another transfer was active "
            "(status 0 means it ran to completion instead of being refused)"
        )
        assert echo == SET_FIRMWARE_FILE_CMD
    finally:
        tcp.close()

    th.join(timeout=upload_timeout + 10.0)
    st, echo, _, _ = result["r"]
    assert st == 0, f"legitimate (serial) upload failed: status={st}"
    assert echo == SET_PATTERN_FILE_CMD
