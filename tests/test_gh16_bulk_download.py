"""HIL tests for issue #16 — 0x84 large pattern download hang/truncation.

Source: reiserlab/LED-Display_G6_Firmware_Arena#16.
Findings and fix numbering: debug/issue-16-bulk-download-notes.md.

Each test below targets one of the four candidate fixes actually implemented:

  fix 1  SerialManager::sendRaw bounds its room<=0 spin to a short stall
         deadline instead of spinning forever when the host stops draining.
         -> test_stalled_download_recovers

  fix 2  GET_PATTERN_FILE_CMD streams one chunk per loop() iteration
         (CommandProcessor::serviceDownload) instead of blocking inside a
         single processCommand() call, so the rest of loop() keeps running.
         -> test_tcp_stays_responsive_during_serial_download

  fix 3  The per-chunk idle deadline resets on every successfully drained
         chunk instead of being a single wall-clock-from-start deadline, so a
         slow-but-progressing host doesn't trip a false timeout.
         -> test_throttled_download_survives_idle_deadline

  fix 4  GET_PATTERN_FILE_CMD now has the same ALL_OFF/CE_DISPLAY_ACTIVE guard
         as GET_SD_ARCHIVE_CMD and SET_PATTERN_FILE_CMD.
         -> test_download_blocked_while_display_active

Fixes 5 (DBG_PRINTF stall instrumentation) and 6 (webDisplayTools console
timeout) are out of scope here — 5 isn't black-box testable and 6 lives in a
different repo.

Uses the real .pat files in tests/data/ (not synthetic payloads) so sizes/paces
match the notes' own worked examples:
  G6_2x10_grating_rotation_20px_50pct.pat   ~81 KB  (fast — guard test)
  G6_2x10_bright-bar_5pct.pat              ~813 KB (fix 3's own example size)
  G6_2x10_sine_rotation_200px.pat          ~8.1 MB (long enough to stall/observe)

Run:
    pixi run test-serial -- -k gh16          # all (serial-only tests included)
    pixi run test-tcp    -- -k gh16          # fix 4 only; others need serial
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
    GET_CONTROLLER_INFO_CMD,
    GET_ETHERNET_IP_ADDRESS_CMD,
    GET_PATTERN_FILE_CMD,
    GET_PATTERN_INFO_CMD,
    STOP_DISPLAY_CMD,
)
from .transport import TCP_PORT, TcpTransport

# ── error codes (ControllerError enum, src/constants.h) ──────────────────────
CE_DISPLAY_ACTIVE = 10

DATA_DIR     = Path(__file__).parent / "data"
GRATING_PAT  = DATA_DIR / "G6_2x10_grating_rotation_20px_50pct.pat"
BRIGHTBAR_PAT = DATA_DIR / "G6_2x10_bright-bar_5pct.pat"
SINE_PAT     = DATA_DIR / "G6_2x10_sine_rotation_200px.pat"

# ── size-scaled timeouts ──────────────────────────────────────────────────────
#
# Rather than hand-picking a fixed timeout per test (which either flakes on
# slower hardware/SD cards or wastes time on faster ones), scale it from the
# transfer size and a floor throughput well below anything observed, plus a
# fixed margin. Being generous costs at most a few extra seconds of wait on a
# real hang — it never causes a false pass, since the tests that need a
# genuinely bounded, SHORT abort (fix 1's stall, fix 3's idle deadline) assert
# on the CONTROLLER's own timing, not on how long the test itself waits.
#
# The floor is deliberately far below the small-sample rates measured early in
# this session (uploads 1.3-4.7 MB/s @ 128-813 KB; downloads 8.2 MB/s @
# 128 KB): a *sustained* 8.1 MB upload, run later in a long test session
# (after several other patterns had already cycled through SD), failed to
# even sustain 300 kB/s — plausibly SD wear-leveling/cache exhaustion
# catching up after many preceding read/write cycles, not a per-call fluke.
# One floor for both directions, set low enough to absorb that rather than
# re-guessing per test.
_MIN_BYTES_PER_SEC = 50 * 1024
_TIMEOUT_MARGIN_S = 15.0


def _size_timeout(size_bytes: int, min_bytes_per_sec: int = _MIN_BYTES_PER_SEC,
                  margin_s: float = _TIMEOUT_MARGIN_S) -> float:
    """Generous timeout for a size_bytes transfer at >= min_bytes_per_sec.

    When a test deliberately paces itself (a throttled download's own
    bytes_per_sec), pass min(that rate, _MIN_BYTES_PER_SEC): the deliberate
    rate is a ceiling the test is aiming for, not a guaranteed floor the
    controller/SD can actually sustain under load (see module docstring
    above) — never assume better than the general floor just because the
    test asked for a slower pace.
    """
    return size_bytes / min_bytes_per_sec + margin_s


def _pattern_file_size(transport, idx: int) -> int:
    """Query GET_PATTERN_INFO (0x88) for pattern idx's file_size (u32 LE at
    payload offset 7 — frame_count u16, gs/rows/cols/arena/observer u8 each,
    then file_size u32, duty_cycle u8; see test_gh18_pattern_info.py)."""
    st, echo, payload, diag = transport.command(
        GET_PATTERN_INFO_CMD, struct.pack("<H", idx), timeout=5.0,
    )
    assert st == 0, f"GET_PATTERN_INFO(0x88) failed for idx={idx}: status={st}, diag={diag}"
    assert echo == GET_PATTERN_INFO_CMD
    return struct.unpack_from("<I", payload, 7)[0]


def _download_timeout(transport, idx: int) -> float:
    """Generous download timeout derived from the pattern's actual on-SD size
    (via 0x88, not local knowledge) and the general floor throughput."""
    return _size_timeout(_pattern_file_size(transport, idx))


def _upload(transport, path: Path, name: str) -> int:
    """Upload path's bytes to pattern.temp, promote to `name`, return 1-based idx."""
    data = path.read_bytes()
    timeout = _size_timeout(len(data))
    st, _, _, _ = transport.upload_file(0, data, timeout=timeout)
    assert st == 0, f"setup: upload of {path.name} failed: status={st}"
    st2, _, payload, _ = transport.rename_file(0, name)
    assert st2 == 0, f"setup: rename of {path.name} failed: status={st2}"
    return struct.unpack("<H", payload)[0]


def _open_tcp_via_serial(transport) -> TcpTransport:
    """Auto-discover the controller's own IP over the serial link already in
    use (GET_ETHERNET_IP_ADDRESS_CMD, 0xC1) — rather than requiring a --ip
    flag — and open an independent TCP connection to it. Caller must close()."""
    st_ip, echo_ip, ip_payload, _ = transport.command(GET_ETHERNET_IP_ADDRESS_CMD, timeout=2.0)
    assert st_ip == 0, "could not read controller IP over serial"
    assert echo_ip == GET_ETHERNET_IP_ADDRESS_CMD
    ip = ip_payload.decode("ascii").rstrip("\x00")
    assert ip, "controller reported an empty IP address (no ethernet link?)"
    tcp = TcpTransport(ip, TCP_PORT)
    tcp.open()
    return tcp


# ── fix 4: ALL_OFF guard ──────────────────────────────────────────────────────

def test_download_blocked_while_display_active(transport):
    """GET_PATTERN_FILE_CMD must be rejected (CE_DISPLAY_ACTIVE) while a pattern
    is displaying, mirroring the existing SET_PATTERN_FILE_CMD guard
    (test_lab79_sd.py::test_upload_blocked_while_display_active)."""
    idx = _upload(transport, GRATING_PAT, "gh16_guard.pat")

    st_on, _, _, _ = transport.command(ALL_ON_CMD)
    assert st_on == 0

    st, echo, data, _ = transport.download_file(idx, timeout=10.0)
    assert st == CE_DISPLAY_ACTIVE, f"expected CE_DISPLAY_ACTIVE, got status={st}"
    assert echo == GET_PATTERN_FILE_CMD
    assert data == b""

    transport.command(STOP_DISPLAY_CMD)
    st2, echo2, data2, _ = transport.download_file(idx, timeout=_download_timeout(transport, idx))
    assert st2 == 0, f"download failed after stop: status={st2}"
    assert echo2 == GET_PATTERN_FILE_CMD
    assert len(data2) == GRATING_PAT.stat().st_size
    assert data2 == GRATING_PAT.read_bytes()


# ── fix 3: idle-based (not wall-clock) deadline ───────────────────────────────

@pytest.mark.serial_only
def test_throttled_download_survives_idle_deadline(transport):
    """An 813 KB file paced slow enough to comfortably exceed 60 s total must
    still complete, rather than truncate at the old fixed wall-clock deadline.

    The notes' own worked example paces this file at ~13 kB/s (~61 s total)
    — deliberately not used here: that leaves only ~1 s of margin over the
    60 s deadline, and pacing jitter alone was enough to land the transfer
    just under 60 s even on the OLD firmware (confirmed empirically — that
    rate does not reliably distinguish old vs. fixed behavior). ~10 kB/s
    (~79 s total) gives a solid margin either way.
    """
    data = BRIGHTBAR_PAT.read_bytes()
    idx = _upload(transport, BRIGHTBAR_PAT, "gh16_throttle.pat")

    throttle_rate = 10 * 1024
    file_size = _pattern_file_size(transport, idx)
    assert file_size == len(data)
    st, echo, got, _ = transport.download_file_throttled(
        idx, bytes_per_sec=throttle_rate,
        overall_timeout=_size_timeout(file_size, min(throttle_rate, _MIN_BYTES_PER_SEC)),
    )
    assert st == 0
    assert echo == GET_PATTERN_FILE_CMD
    assert len(got) == len(data), f"truncated at {len(got)}/{len(data)} bytes"
    assert got == data


# ── fix 1: stalled-host recovery ──────────────────────────────────────────────

@pytest.mark.serial_only
def test_stalled_download_recovers(transport):
    """A host that stops draining mid-download entirely must not wedge the
    controller.

    Note on what actually distinguishes buggy vs. fixed firmware here: this
    link's observed serial throughput (~8 MB/s) means that once a stalled
    reader resumes, even the *old* unbounded sendRaw() finishes the whole
    8.1 MB file in about a second — "does the transfer eventually complete"
    does NOT distinguish old from fixed firmware (confirmed empirically: the
    old code passed that check too). What actually differs is whether the
    REST of the controller stays responsive *during* the stall, before the
    host ever resumes. Old firmware blocks the entire loop() inside
    sendRaw()'s unbounded spin for as long as the stall lasts; fixed firmware
    (fix 1) gives up on that download after its stall bound (2 s — see
    SerialManager::sendRaw's kStallTimeoutMs, widened from an initial 1.5 s
    after on-hardware testing showed that was tight enough to false-trigger
    on a genuinely slow-but-progressing link too; see fix 3's test) regardless
    of whether the host ever resumes, freeing loop() to keep running.

    So this drives the stall itself over serial and polls an INDEPENDENT TCP
    connection throughout — the same responsiveness signal
    test_tcp_stays_responsive_during_serial_download uses for a merely slow
    (but progressing) download, applied here to a hard stall (no draining at
    all) instead.
    """
    idx = _upload(transport, SINE_PAT, "gh16_stall.pat")

    tcp = _open_tcp_via_serial(transport)
    try:
        status, echo, file_size, _leftover, _diag = transport.begin_download(idx, timeout=2.0)
        assert status == 0
        assert echo == GET_PATTERN_FILE_CMD
        assert file_size == SINE_PAT.stat().st_size

        # Deliberately never drain the serial download body — a hard stall,
        # not a throttle — while polling TCP for the whole stall window.
        # Window and per-poll timeout both sized with margin over the ~2 s
        # firmware stall bound so a poll landing right at that boundary
        # measures as "slow" rather than timing out as an exception.
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
            f"controller stopped answering TCP while the serial download was "
            f"stalled: max latency {max(latencies):.2f}s — looks wedged"
        )
    finally:
        tcp.close()
        # We never drained the serial download body (that's the point of the
        # test), so leftover bytes from the aborted transfer are still sitting
        # in the OS read buffer. transport is session-scoped — discard them
        # now so they don't corrupt the next test's response parsing.
        _drain_serial(transport)


def _drain_serial(transport, quiet_seconds: float = 0.5):
    """Read and discard bytes until the serial link has been quiet for
    `quiet_seconds` — used to resynchronize after a test deliberately leaves
    an aborted transfer's tail bytes unread."""
    ser = transport._ser
    quiet_deadline = time.monotonic() + quiet_seconds
    while time.monotonic() < quiet_deadline:
        n = ser.in_waiting
        if n:
            ser.read(n)
            quiet_deadline = time.monotonic() + quiet_seconds
        else:
            time.sleep(0.02)


# ── fix 2: download doesn't block loop() ─────────────────────────────────────

@pytest.mark.serial_only
def test_tcp_stays_responsive_during_serial_download(transport):
    """While a long download streams over serial, TCP round-trips on an
    independent connection must stay fast.

    Opens a second TCP connection (see _open_tcp_via_serial) and polls
    GET_CONTROLLER_INFO on it while the serial download is in flight.

    Deliberately uses the plain, unthrottled download_file() rather than
    download_file_throttled() here: running the token-bucket pacer
    concurrently with this test's own polling thread was observed (via
    GET_CONTROLLER_INFO's debug fields) to make the BACKGROUND thread fall
    far behind its target rate under real GIL/scheduling contention between
    the two threads — the controller finished sending the whole file cleanly
    (dl_remaining_ reached 0, no abort) each time, but the Python reader
    simply couldn't keep pace, timing out around ~50% regardless of how
    generous the timeout was. That's a test-harness threading artifact, not
    something this test is trying to measure — the property under test is
    TCP latency (asserted below), not the download's own duration, so the
    simpler, unpaced reader sidesteps the interaction entirely.
    """
    idx = _upload(transport, SINE_PAT, "gh16_concurrent.pat")
    download_timeout = _download_timeout(transport, idx)

    tcp = _open_tcp_via_serial(transport)
    try:
        latencies = []
        stop = threading.Event()
        download_result = {}

        def _download():
            try:
                download_result["result"] = transport.download_file(
                    idx, timeout=download_timeout,
                )
            finally:
                stop.set()

        th = threading.Thread(target=_download, daemon=True)
        th.start()
        while not stop.is_set():
            t0 = time.monotonic()
            st, echo, _, _ = tcp.command(GET_CONTROLLER_INFO_CMD, timeout=2.0)
            latencies.append(time.monotonic() - t0)
            assert st == 0
            assert echo == GET_CONTROLLER_INFO_CMD
            time.sleep(0.2)
        th.join(timeout=5.0)

        assert latencies, "no TCP polls completed during the download"
        assert max(latencies) < 1.0, (
            f"TCP stalled during serial download: max latency {max(latencies):.2f}s "
            f"({len(latencies)} polls)"
        )

        st, echo, got, _ = download_result["result"]
        assert st == 0
        assert echo == GET_PATTERN_FILE_CMD
        assert len(got) == SINE_PAT.stat().st_size
        assert got == SINE_PAT.read_bytes()
    finally:
        tcp.close()
