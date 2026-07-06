"""HIL tests for GET_SD_ARCHIVE (0x8A) async streaming, PR #27 review point 2.

Source: reiserlab/LED-Display_G6_Firmware_Arena#27, review comment
pullrequestreview-4632788440. Findings and the alternative implemented here:
debug/pr27-response.md, point 2's "Alternative approaches" (full async port).

Before this fix, handleGetSdArchive() streamed the whole ZIP inline via
sendRaw() calls whose return value it never checked. sendRaw() can return
short after a stall (issue #16 fix 1) instead of blocking forever, so a host
that paused mid-archive got silently truncated ZIP bytes with status 0
already sent, a corrupt archive with no error reported. This is now ported
onto an async chunked model (serviceArchive(), mirroring serviceDownload()):
each phase (headers, file data, data descriptor, central directory, EOCD)
streams one bounded sendRaw() per loop() call and aborts the WHOLE transfer
on the first short write, so the corruption case can no longer happen.

  T1  test_archive_download_is_valid_zip
        The archive is a structurally valid ZIP (testzip() finds no bad CRCs)
        and contains the just-uploaded pattern with byte-identical content.
  T2  test_archive_blocked_while_display_active
        GET_SD_ARCHIVE_CMD is still rejected (CE_DISPLAY_ACTIVE) while a
        pattern is displaying, unaffected by the async port.
  T3  test_second_archive_rejected_while_one_in_flight
        A second 0x8A on an independent connection, while the first is still
        streaming, is rejected immediately (not silently re-run every
        loop() iteration the way the PR #27 review's point 1 describes for
        0x85: this command has no bulk payload to desync, so plain
        immediate consumption is enough).
  T4  test_stalled_archive_recovers
        Regression test for the actual bug point 2 fixes: a host that stops
        draining mid-archive must not wedge the controller. Even with issue
        #16 fix 1's sendRaw() stall bound already in place, the OLD
        handleGetSdArchive() ignored every sendRaw() return and just kept
        calling it again for the next buffer, so a stalled host would burn a
        fresh ~2 s stall at EVERY one of its many sendRaw() calls, all inside
        ONE blocking invocation, keeping loop() unresponsive for far longer
        than a single stall period. serviceArchive() aborts on the FIRST
        short write, so loop() is blocked for at most one stall period no
        matter how many entries/chunks remain.

Uses the small pattern in tests/data/ (not the multi-MB ones test_gh16 uses)
since the async archive itself isn't the thing under stress test here; T4's
stall doesn't depend on file size, only on the host never draining.

Run:
    pixi run test-serial -- -k pr27
    pixi run test-tcp    -- -k pr27   # T2 only; T1/T3/T4 need serial
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
    GET_CONTROLLER_INFO_CMD,
    GET_ETHERNET_IP_ADDRESS_CMD,
    GET_SD_ARCHIVE_CMD,
    STOP_DISPLAY_CMD,
)
from .transport import TCP_PORT, TcpTransport

CE_DISPLAY_ACTIVE = 10

DATA_DIR    = Path(__file__).parent / "data"
GRATING_PAT = DATA_DIR / "G6_2x10_grating_rotation_20px_50pct.pat"


def _upload(transport, path: Path, name: str) -> int:
    """Upload path's bytes to pattern.temp, promote to `name`, return 1-based idx."""
    data = path.read_bytes()
    st, _, _, _ = transport.upload_file(0, data, timeout=30.0)
    assert st == 0, f"setup: upload of {path.name} failed: status={st}"
    st2, _, payload, _ = transport.rename_file(0, name)
    assert st2 == 0, f"setup: rename of {path.name} failed: status={st2}"
    return struct.unpack("<H", payload)[0]


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


# ── T1: the async-streamed archive is a valid, byte-correct ZIP ──────────────

def test_archive_download_is_valid_zip(transport):
    """The archive serviceArchive() streams must be a structurally valid ZIP
    (no CRC mismatches) and must contain the uploaded pattern unmodified."""
    transport.command(ALL_OFF_CMD)
    _upload(transport, GRATING_PAT, "pr27_archive.pat")

    st, echo, data, diag = transport.get_sd_archive(timeout=60.0)
    assert st == 0, f"GET_SD_ARCHIVE failed: status={st}, diag={diag}"
    assert echo == GET_SD_ARCHIVE_CMD

    zf = zipfile.ZipFile(io.BytesIO(data))
    bad_entry = zf.testzip()
    assert bad_entry is None, f"CRC mismatch in archive entry: {bad_entry}"

    names = set(zf.namelist())
    assert "MANIFEST.bin" in names or "MANIFEST.txt" in names, (
        f"archive is missing both manifest files: {sorted(names)}"
    )
    matches = [n for n in names if n.endswith("pr27_archive.pat")]
    assert matches, f"uploaded pattern not found in archive: {sorted(names)}"
    assert zf.read(matches[0]) == GRATING_PAT.read_bytes(), (
        "extracted pattern bytes differ from the uploaded file"
    )


# ── T2: ALL_OFF guard survives the async port ─────────────────────────────────

def test_archive_blocked_while_display_active(transport):
    """GET_SD_ARCHIVE_CMD must still be rejected (CE_DISPLAY_ACTIVE) while a
    pattern is displaying, unchanged by porting the stream onto serviceArchive()."""
    st_on, _, _, _ = transport.command(ALL_ON_CMD)
    assert st_on == 0

    st, echo, data, _ = transport.get_sd_archive(timeout=10.0)
    assert st == CE_DISPLAY_ACTIVE, f"expected CE_DISPLAY_ACTIVE, got status={st}"
    assert echo == GET_SD_ARCHIVE_CMD
    assert data == b""

    transport.command(STOP_DISPLAY_CMD)


# ── T3: a second, concurrent 0x8A is rejected, not silently re-run ───────────

@pytest.mark.serial_only
def test_second_archive_rejected_while_one_in_flight(transport):
    """While one archive streams over serial, a second GET_SD_ARCHIVE_CMD on
    an independent TCP connection must be rejected immediately. This proves
    ar_active_/ar_source_ gates a second transfer the way dl_active_ does
    for 0x84, without reproducing PR #27 review point 1's re-dispatch bug
    (0x8A has no bulk payload, so it's always consumed right away either way)."""
    transport.command(ALL_OFF_CMD)
    _upload(transport, GRATING_PAT, "pr27_concurrent.pat")

    tcp = _open_tcp_via_serial(transport)
    try:
        result = {}

        def _archive():
            result["r"] = transport.get_sd_archive(timeout=60.0)

        th = threading.Thread(target=_archive, daemon=True)
        th.start()

        # get_sd_archive()'s first action is to send the request; give the
        # controller a moment to receive it, run handleGetSdArchive() (which
        # sends the header response and sets ar_active_), and start
        # serviceArchive() streaming before firing the second request on TCP.
        # The rejection this proves is about ar_active_ already being true,
        # not a race at request time, so this only needs to be comfortably
        # longer than one loop() iteration.
        time.sleep(0.2)

        st2, echo2, data2, _ = tcp.command(GET_SD_ARCHIVE_CMD, timeout=5.0)
        th.join(timeout=65.0)

        assert st2 != 0, "second concurrent GET_SD_ARCHIVE_CMD was accepted"
        assert echo2 == GET_SD_ARCHIVE_CMD
        assert data2 == b""

        st1, echo1, data1, diag1 = result["r"]
        assert st1 == 0, f"first (serial) archive failed: status={st1}, diag={diag1}"
        assert echo1 == GET_SD_ARCHIVE_CMD
        zf = zipfile.ZipFile(io.BytesIO(data1))
        assert zf.testzip() is None, "first archive has a CRC mismatch"

        # The controller must be free to serve a fresh archive once the first
        # one has actually finished. Uses get_sd_archive() (not command()) so
        # this fully drains the ZIP body too, leaving the connection clean
        # before it's closed below.
        st3, echo3, data3, diag3 = tcp.get_sd_archive(timeout=60.0)
        assert st3 == 0, f"archive rejected after the in-flight one finished: status={st3}, diag={diag3}"
        assert echo3 == GET_SD_ARCHIVE_CMD
        zf3 = zipfile.ZipFile(io.BytesIO(data3))
        assert zf3.testzip() is None, "second archive has a CRC mismatch"
    finally:
        tcp.close()


def _drain_serial(transport, quiet_seconds: float = 0.5):
    """Read and discard bytes until the serial link has been quiet for
    `quiet_seconds`. Used to resynchronize after a test deliberately leaves
    an aborted transfer's tail bytes unread. Ported from
    test_gh16_bulk_download.py."""
    ser = transport._ser
    quiet_deadline = time.monotonic() + quiet_seconds
    while time.monotonic() < quiet_deadline:
        n = ser.in_waiting
        if n:
            ser.read(n)
            quiet_deadline = time.monotonic() + quiet_seconds
        else:
            time.sleep(0.02)


# ── T4: regression test for point 2's actual bug (stalled host, ignored return) ──

@pytest.mark.serial_only
def test_stalled_archive_recovers(transport):
    """A host that stops draining mid-archive entirely must not wedge the
    controller.

    This is the regression test for the bug PR #27 review point 2 actually
    describes: the pre-port handleGetSdArchive() called sendRaw() at many
    sites per entry (LFH+name, each file-data chunk, the data descriptor,
    later the central-directory entry) all inside ONE processCommand() call,
    and never checked any of those returns. issue #16 fix 1 already bounds a
    single sendRaw() call to a ~2 s stall (SerialManager::sendRaw's
    kStallTimeoutMs) instead of spinning forever, but the old archive code
    would just call sendRaw() again for the next buffer on a still-stalled
    host, so it would burn a FRESH ~2 s stall at every remaining call within
    that same blocking invocation. For an archive with several entries and
    file-data chunks, that adds up to many multiples of the stall bound,
    still keeping loop() (and therefore TCP) unresponsive for far longer than
    a single stall period, even with fix 1 in place.

    serviceArchive() (this fix) checks the return and aborts the WHOLE
    transfer on the FIRST short write, so loop() is blocked for at most one
    ~2 s stall no matter how many entries/chunks remain. This drives the
    stall over serial and polls an INDEPENDENT TCP connection throughout,
    the same responsiveness signal test_gh16_bulk_download.py's
    test_stalled_download_recovers uses for 0x84.
    """
    transport.command(ALL_OFF_CMD)
    _upload(transport, GRATING_PAT, "pr27_stall.pat")

    tcp = _open_tcp_via_serial(transport)
    try:
        status, echo, archive_size, _leftover, diag = transport.begin_get_sd_archive(timeout=5.0)
        assert status == 0, f"GET_SD_ARCHIVE header failed: status={status}, diag={diag}"
        assert echo == GET_SD_ARCHIVE_CMD
        assert archive_size > 0

        # Deliberately never drain the serial archive body (a hard stall,
        # not a throttle) while polling TCP for a window comfortably longer
        # than one stall period, but far shorter than the many-stalls total
        # the OLD code would need to even reach the end of the archive.
        latencies = []
        t_end = time.monotonic() + 8.0
        while time.monotonic() < t_end:
            t0 = time.monotonic()
            st, tcp_echo, _, _ = tcp.command(GET_CONTROLLER_INFO_CMD, timeout=4.0)
            latencies.append(time.monotonic() - t0)
            assert st == 0
            assert tcp_echo == GET_CONTROLLER_INFO_CMD
            time.sleep(0.2)

        assert latencies, "no TCP polls completed during the stall"
        assert max(latencies) < 3.0, (
            f"controller stopped answering TCP while the serial archive was "
            f"stalled: max latency {max(latencies):.2f}s, looks wedged"
        )
    finally:
        tcp.close()
        # The serial archive body was never drained (that's the point of the
        # test), so leftover bytes from the aborted transfer are still
        # sitting in the OS read buffer. transport is session-scoped, so
        # discard them now before the next test's response parsing.
        _drain_serial(transport)
