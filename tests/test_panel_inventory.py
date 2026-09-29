"""PANEL_INVENTORY_SCAN (0xD0) / GET_PANEL_INVENTORY (0xD1): per-panel presence +
firmware fingerprint.

Wire format is specified in src/PanelInventory.h. Run against a flashed arena;
the fingerprint tests need at least one responding panel.
"""

import re
import struct
import time

import pytest

from .commands import (
    ALL_OFF_CMD,
    ALL_ON_CMD,
    G6_VERIFY_PANEL_CMD,
    FEATURE_PANEL_INVENTORY,
    GET_CONTROLLER_INFO_CMD,
    GET_FIRMWARE_INFO_CMD,
    GET_FIRMWARE_VERSION_CMD,
    GET_PANEL_INVENTORY_CMD,
    PANEL_INVENTORY_SCAN_CMD,
    SYSTEM_RESET_CMD,
    controller_features,
)

CE_DISPLAY_ACTIVE = 10

FLAG_PRESENCE_VALID = 0x01
FLAG_FP_VALID = 0x02
FLAG_FP_IN_PROGRESS = 0x04
FLAG_REF_PRESENT = 0x08
FLAG_FP_PREFIX = 0x10

ABSENT, PRESENT, FW_MATCH, FW_DIFFERS, FW_NO_REF, FW_FAILED = 1, 2, 3, 4, 5, 6
FINGERPRINTED = {FW_MATCH, FW_DIFFERS, FW_NO_REF}

# version, count, flags, first, n, ref_crc, fp_len, age_ms, scan_id
HEADER = struct.Struct("<BBBBBIIIB")


def _parse_page(st, payload):
    if st != 0:
        return st, None, None
    assert len(payload) >= HEADER.size, payload.hex()
    (version, count, flags, first_echo, n, ref_crc, fp_len, age_ms,
     scan_id) = HEADER.unpack_from(payload)
    entries = [struct.unpack_from("<BI", payload, HEADER.size + 5 * k) for k in range(n)]
    assert len(payload) == HEADER.size + 5 * n
    hdr = dict(version=version, count=count, flags=flags, first=first_echo, n=n,
               ref_crc=ref_crc, fp_len=fp_len, age_ms=age_ms, scan_id=scan_id)
    return st, hdr, entries


def read_page(transport, first=0):
    st, echo, payload, _ = transport.command(GET_PANEL_INVENTORY_CMD, bytes([first]))
    assert echo == GET_PANEL_INVENTORY_CMD
    return _parse_page(st, payload)


def scan(transport, action, timeout=10.0):
    st, echo, payload, _ = transport.command(PANEL_INVENTORY_SCAN_CMD, bytes([action]), timeout=timeout)
    assert echo == PANEL_INVENTORY_SCAN_CMD
    return _parse_page(st, payload)


def build_panel_count(transport):
    st, _, payload, _ = transport.command(GET_FIRMWARE_VERSION_CMD)
    assert st == 0
    return payload[1] * payload[2]  # rows x cols


def read_all(transport, retries=3):
    """Read every page; re-read from the start when a scan lands between pages."""
    for _ in range(retries):
        st, hdr, entries = read_page(transport, 0)
        assert st == 0
        consistent = True
        while len(entries) < hdr["count"]:
            st, hdr2, more = read_page(transport, len(entries))
            assert st == 0 and more
            if hdr2["scan_id"] != hdr["scan_id"]:
                consistent = False
                break
            entries += more
        if consistent:
            return hdr, entries
    pytest.fail("scan_id kept changing between pages")


@pytest.fixture
def display_off(transport):
    transport.command(ALL_OFF_CMD)
    yield
    transport.command(ALL_OFF_CMD)


def test_feature_bit_advertised(transport):
    st, _, payload, _ = transport.command(GET_CONTROLLER_INFO_CMD)
    assert st == 0
    assert controller_features(payload) & (1 << FEATURE_PANEL_INVENTORY)
    assert not payload[1] & 0x40, "capability bit 6 is ai_cal, not panel inventory"


def test_default_request_reads_first_page(transport):
    st, echo, payload, _ = transport.command(GET_PANEL_INVENTORY_CMD)
    assert st == 0 and echo == GET_PANEL_INVENTORY_CMD
    version, count, flags, first, n = payload[:5]
    assert version == 1
    assert count == build_panel_count(transport)
    assert flags & FLAG_PRESENCE_VALID, "boot presence scan should have run"
    assert first == 0 and n == min(32, count)


def test_pages_cover_every_panel_once(transport):
    hdr, entries = read_all(transport)
    assert len(entries) == hdr["count"]
    st, hdr2, past_end = read_page(transport, hdr["count"])
    assert st == 0 and hdr2["n"] == 0 and past_end == []
    assert hdr2["scan_id"] == hdr["scan_id"], "a pure read must not change scan_id"


def test_entries_are_well_formed(transport):
    _, entries = read_all(transport)
    for i, (status, crc) in enumerate(entries):
        assert 0 <= status <= FW_FAILED, f"panel {i + 1}: status {status}"
        if status not in FINGERPRINTED:
            assert crc == 0, f"panel {i + 1}: crc {crc:#x} with status {status}"


def test_bad_action_rejected(transport, display_off):
    st, _, _ = scan(transport, 2)
    assert st == 1


def test_scan_requires_action_byte(transport, display_off):
    st, echo, _, _ = transport.command(PANEL_INVENTORY_SCAN_CMD)
    assert echo == PANEL_INVENTORY_SCAN_CMD and st == 1


def test_rescan_refused_while_display_runs_but_read_allowed(transport, display_off):
    assert transport.command(ALL_ON_CMD)[0] == 0
    st, _, _ = scan(transport, 0)
    assert st == CE_DISPLAY_ACTIVE
    st, hdr, _ = read_page(transport)
    assert st == 0 and hdr["flags"] & FLAG_PRESENCE_VALID


def test_presence_rescan(transport, display_off):
    before, _ = read_all(transport)
    st, hdr, _ = scan(transport, 0)
    assert st == 0 and hdr["first"] == 0
    assert hdr["flags"] & FLAG_PRESENCE_VALID
    assert hdr["scan_id"] != before["scan_id"], "a presence scan must change scan_id"
    assert hdr["age_ms"] < 2000


def test_presence_rescan_keeps_fingerprints(transport, display_off):
    hdr0, before = fingerprint_sweep(transport)
    st, hdr1, _ = scan(transport, 0)
    assert st == 0
    assert hdr1["flags"] & FLAG_FP_VALID, "action 0 must not discard a completed sweep"
    assert not hdr1["flags"] & FLAG_FP_IN_PROGRESS
    assert hdr1["ref_crc"] == hdr0["ref_crc"] and hdr1["fp_len"] == hdr0["fp_len"]
    _, after = read_all(transport)
    for i, ((s0, crc0), (s1, crc1)) in enumerate(zip(before, after)):
        if s0 in FINGERPRINTED and s1 != ABSENT:
            assert (s1, crc1) == (s0, crc0), f"panel {i + 1} lost its fingerprint"


def test_fingerprint_rescan_discards_fingerprints(transport, display_off):
    fingerprint_sweep(transport)
    st, hdr, entries = scan(transport, 1)
    assert st == 0
    assert hdr["flags"] & FLAG_FP_IN_PROGRESS and not hdr["flags"] & FLAG_FP_VALID
    for i, (status, crc) in enumerate(entries):
        assert status in (ABSENT, PRESENT) and crc == 0, f"panel {i + 1}: stale {status}/{crc:#x}"


def _reconnect_after_reset(transport, first_cmd, deadline_s=20.0):
    """Reopen the port after SYSTEM_RESET and make `first_cmd` the first command the controller
    processes. The controller answers nothing until setup() ends (~4 s on the 2x10), so probes
    sent before that are answered late; those replies are drained afterwards so the transport
    stays in sync (a stale reply would otherwise shift every later echo by one)."""
    transport.close()
    time.sleep(0.5)
    deadline = time.monotonic() + deadline_s
    while True:
        try:
            transport.open()
            st, echo, _, _ = transport.command(first_cmd, timeout=0.5)
            if st == 0 and echo == first_cmd:
                break
        except Exception:  # noqa: BLE001 — the CDC node comes and goes during a reboot
            pass
        try:
            transport.close()
        except Exception:  # noqa: BLE001
            pass
        if time.monotonic() > deadline:
            raise RuntimeError("controller did not come back after SYSTEM_RESET")
        time.sleep(0.2)
    time.sleep(0.5)  # late replies to the probes that timed out arrive now
    transport._ser.reset_input_buffer()
    st, echo, _, _ = transport.command(GET_CONTROLLER_INFO_CMD)
    assert st == 0 and echo == GET_CONTROLLER_INFO_CMD, "transport out of sync after reset"


def test_display_started_before_boot_scan_cancels_it(transport, display_off):
    """A display the host starts before the 3.5 s late boot step must cancel the boot scan —
    otherwise the scan lands in the display's first inter-trial ALL_OFF and delays the next
    trial. presence_valid stays clear until the host rescans with 0xD0. ~8 s."""
    st, _, payload, _ = transport.command(SYSTEM_RESET_CMD)
    assert st == 0 and bytes(payload) == b"rebooting"
    _reconnect_after_reset(transport, ALL_ON_CMD)  # ALL_ON is the first command processed
    st, hdr, _ = read_page(transport)
    assert st == 0
    if hdr["flags"] & FLAG_PRESENCE_VALID:
        # setup() (power settle, SD, Ethernet) outlasted the 3.5 s late-boot mark: the boot
        # scan ran on the first loop() pass, before ALL_ON was processed (2x10 performance
        # build: first reply ~4.0 s after reset). Nothing to cancel on this build.
        pytest.skip("boot scan ran before the controller processed its first command")
    time.sleep(2.0)  # past the late boot step while the display runs
    assert transport.command(ALL_OFF_CMD)[0] == 0
    time.sleep(1.0)  # a still-pending scan would run here
    st, hdr, _ = read_page(transport)
    assert st == 0
    assert not hdr["flags"] & FLAG_PRESENCE_VALID, "boot scan ran after the display started"
    st, hdr, _ = scan(transport, 0)
    assert st == 0 and hdr["flags"] & FLAG_PRESENCE_VALID, "host rescan must still work"


def fingerprint_sweep(transport, timeout_s=90.0):
    st, hdr, _ = scan(transport, 1)
    assert st == 0 and hdr["flags"] & FLAG_FP_IN_PROGRESS
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        hdr, entries = read_all(transport)
        if hdr["flags"] & FLAG_FP_VALID:
            return hdr, entries
        time.sleep(0.5)
    pytest.fail("fingerprint sweep did not complete")


def test_fingerprint_sweep_completes(transport, display_off):
    hdr, entries = fingerprint_sweep(transport)
    assert not hdr["flags"] & FLAG_FP_IN_PROGRESS
    assert bool(hdr["flags"] & FLAG_REF_PRESENT) != bool(hdr["flags"] & FLAG_FP_PREFIX)
    assert hdr["fp_len"] > 0
    for i, (status, _) in enumerate(entries):
        assert status != PRESENT, f"panel {i + 1} responded but was never fingerprinted"


def test_reference_matches_sd_footer(transport, display_off):
    hdr, _ = fingerprint_sweep(transport)
    st, _, footer, _ = transport.command(GET_FIRMWARE_INFO_CMD)
    if st != 0 or not hdr["flags"] & FLAG_REF_PRESENT:
        pytest.skip("no /firmware/panel.bin on SD")
    crc, size = struct.unpack_from("<II", footer, 24)
    assert hdr["ref_crc"] == crc and hdr["fp_len"] == size


def test_fingerprint_agrees_with_verify_panel(transport, display_off):
    hdr, entries = fingerprint_sweep(transport)
    if not hdr["flags"] & FLAG_REF_PRESENT:
        pytest.skip("0xC9 needs /firmware/panel.bin on SD")
    idx = next((i for i, (s, _) in enumerate(entries) if s in FINGERPRINTED), None)
    if idx is None:
        pytest.skip("no fingerprinted panel")
    st, _, msg, _ = transport.command(G6_VERIFY_PANEL_CMD, bytes([idx + 1]), timeout=10.0)
    m = re.search(rb"running-app CRC=0x([0-9A-Fa-f]{8})", msg)
    assert m, msg
    assert int(m.group(1), 16) == entries[idx][1]
