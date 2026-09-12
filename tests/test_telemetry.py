"""Telemetry ring — SET_TELEMETRY (0xA8) + GET_TELEMETRY_BLOCK (0xA9), issue #50 follow-on.

The controller appends a small binary record for every dispatched command,
every displayed frame change, and every state transition to a 64 KiB OCRAM
ring (src/Telemetry.h); the host drains it with framed, chunked 0xA9 replies
and an ACK CURSOR. These tests check the wire contract end to end:

  * both SET_TELEMETRY forms are accepted, bad lengths rejected;
  * the 18-byte block header decodes; whole records only (a split record or a
    returned PAD fails the strict parser); record bytes == payload - 18;
  * seq is contiguous across a full drain; CMD records match the commands we
    sent (opcode, params, reply status), in order;
  * `more` is 1 exactly until the last chunk; re-asking with the same ack
    returns the same first_seq and the same bytes (lossless); an ack frees
    records and a lower ack never resurrects them;
  * disabling stops recording (and records that it did); re-enabling resumes;
  * the synthetic producer (flags bit7 + rate) emits cmd-0xFE records at the
    requested rate with a contiguous counter; overfilling the ring with it
    evicts the OLDEST records: `dropped` grows, the drain shows a seq gap of
    exactly that size, and one STATE(ring_overrun) marks the episode;
  * a 0x70 storm in Mode 3 yields FRAME records whose idx sequence follows the
    commanded indices (needs --pat, multi-frame pattern);
  * optionally (TELEMETRY_RESET_OK=1 in the environment — it reboots the
    controller): the ring survives SYSTEM_RESET and the first record after the
    reboot is STATE(boot) with the next seq.

NON-DESTRUCTIVE by default. The frame-storm test uses the session `pat`
fixture (adds 'conftest.pat', deletes nothing) and skips without --pat.

Run:
    pixi run test-serial -- --port /dev/cu.usbmodemXXX
    ... -- --pat .../g6_2x10/patterns/004_frame2_h_ccw_200f.pat     # + frame storm
    TELEMETRY_RESET_OK=1 pixi run test-serial -- tests/test_telemetry.py -k reset
"""

import os
import struct
import time

import pytest

from .commands import (
    GET_CONTROLLER_INFO_CMD,
    GET_FIRMWARE_VERSION_CMD,
    GET_FRAME_POSITION_CMD,
    GET_PANEL_DISPLAY_MODE_CMD,
    GET_PATTERN_INFO_CMD,
    GET_REFRESH_RATE_CMD,
    GET_TELEMETRY_BLOCK_CMD,
    SET_FRAME_POSITION_CMD,
    SET_TELEMETRY_CMD,
    STOP_DISPLAY_CMD,
    SYSTEM_RESET_CMD,
    TRIAL_PARAMS_CMD,
)
from .telemetry_codec import (
    ARENA_STATE_NAMES,
    BLOCK_HEADER_LEN,
    FLAG_EVENTS_ENABLED,
    FLAG_HEAP_COLLISION,
    FLAG_SURVIVED_REBOOT,
    FLAG_SYNTHETIC_ON,
    NO_ACK,
    OVERRUN_CODE_EVICTED,
    RECORD_BYTES_MAX,
    REC_CMD,
    REC_FRAME,
    REC_STATE,
    SET_FLAG_EVENTS,
    SET_FLAG_SYNTHETIC,
    ST_BOOT,
    ST_RING_OVERRUN,
    ST_SD_OPEN,
    ST_STATE_CHANGE,
    ST_TELEMETRY,
    SYNTHETIC_CMD,
    build_get_block,
    build_set_telemetry,
    parse_block,
)

CAP_HEALTH = 0x80  # capability bit 7 gates 0xCA/0xCB; the ring itself is GET_FIRMWARE_VERSION flags bit 2
FW_FLAG_TELEMETRY = 0x04
STATE_ALL_OFF = ARENA_STATE_NAMES.index("ALL_OFF")
STATE_SHOW_FRAME = ARENA_STATE_NAMES.index("SHOW_FRAME")
OP_CMD = 4  # Health::LastOp::OP_CMD (a commanded reset's breadcrumb)
RING_DATA_BYTES = 0x10000 - 32


# ── helpers ───────────────────────────────────────────────────────────────────

def get_block(transport, ack_seq=NO_ACK, max_bytes=RECORD_BYTES_MAX):
    """One 0xA9 round trip → (header, records, raw_payload)."""
    st, echo, payload, _ = transport.command(GET_TELEMETRY_BLOCK_CMD,
                                             build_get_block(ack_seq, max_bytes))
    assert st == 0, f"GET_TELEMETRY_BLOCK failed status={st} payload={bytes(payload)!r}"
    assert echo == GET_TELEMETRY_BLOCK_CMD
    payload = bytes(payload)
    assert len(payload) >= BLOCK_HEADER_LEN, f"block header must be {BLOCK_HEADER_LEN} B"
    hdr, recs = parse_block(payload)  # strict: raises on split/PAD/miscount
    assert sum(len(r.raw) for r in recs) == len(payload) - BLOCK_HEADER_LEN
    assert len(payload) - BLOCK_HEADER_LEN <= min(max_bytes, RECORD_BYTES_MAX)
    assert not (hdr.flags & FLAG_HEAP_COLLISION), "heap guard must never trip on the bench"
    return hdr, recs, payload


def drain(transport, ack=True, max_bytes=RECORD_BYTES_MAX):
    """Loop on `more` until the ring is empty. Returns (records, headers).

    With ack=True each request acknowledges the previous block's last seq and a
    final ack-only request frees the last block, leaving the ring empty.
    """
    records, headers = [], []
    ack_seq = NO_ACK
    for _ in range(4096):  # 4096 × 178 B > the whole 64 KiB ring
        hdr, recs, _ = get_block(transport, ack_seq, max_bytes)
        headers.append(hdr)
        records.extend(recs)
        if recs and ack:
            ack_seq = recs[-1].seq
        if not hdr.more:
            break
    else:
        pytest.fail("drain did not terminate — `more` never cleared")
    if ack and ack_seq != NO_ACK:
        hdr, recs, _ = get_block(transport, ack_seq, max_bytes)  # frees the last block
        assert not recs and hdr.first_seq == 0 and hdr.more == 0
    return records, headers


def set_telemetry(transport, flags, rate=0):
    st, echo, payload, _ = transport.command(SET_TELEMETRY_CMD, build_set_telemetry(flags, rate))
    assert st == 0 and echo == SET_TELEMETRY_CMD
    assert len(payload) == 0, "SET_TELEMETRY reply carries no payload"


def enable(transport, on=True):
    set_telemetry(transport, SET_FLAG_EVENTS if on else 0, 0)  # rate 0 → synthetic off too


def assert_contiguous(recs):
    for a, b in zip(recs, recs[1:]):
        assert b.seq == a.seq + 1, f"seq gap: {a.seq} -> {b.seq}"


@pytest.fixture(autouse=True)
def _telemetry_on(transport):
    """Every test starts with events enabled, synthetic off, display stopped."""
    st, _, _, _ = transport.command(STOP_DISPLAY_CMD)
    assert st == 0
    enable(transport, True)
    yield
    enable(transport, True)


# ── capability + request forms ────────────────────────────────────────────────

def test_firmware_version_advertises_telemetry_flag(transport):
    # The 0xC2 capability byte is full; hosts gate SET_TELEMETRY on 0xCB flags bit2.
    st, echo, payload, _ = transport.command(GET_CONTROLLER_INFO_CMD)
    assert st == 0 and echo == GET_CONTROLLER_INFO_CMD
    assert payload[1] & CAP_HEALTH, "0xCB itself is gated by capability bit 7 (health)"
    st, echo, payload, _ = transport.command(GET_FIRMWARE_VERSION_CMD)
    assert st == 0 and echo == GET_FIRMWARE_VERSION_CMD
    assert payload[3] & FW_FLAG_TELEMETRY, "0xCB flags bit2 (telemetry ring compiled in) must be set"


def test_set_telemetry_accepts_both_forms_and_rejects_bad_length(transport):
    st, _, payload, _ = transport.command(SET_TELEMETRY_CMD, build_set_telemetry(1, 0))     # [04 A8 f lo hi]
    assert st == 0 and len(payload) == 0
    st, _, payload, _ = transport.command(SET_TELEMETRY_CMD, build_set_telemetry(1, None))  # [02 A8 f]
    assert st == 0 and len(payload) == 0
    st, _, _, _ = transport.command(SET_TELEMETRY_CMD, bytes([1, 0]))                       # [03 A8 f x] — neither
    assert st == 1
    st, _, _, _ = transport.command(GET_TELEMETRY_BLOCK_CMD, b"\x00\x00")                  # too short
    assert st == 1


def test_block_header_shape_and_flags(transport):
    hdr, recs, payload = get_block(transport)
    assert hdr.flags & FLAG_EVENTS_ENABLED
    assert not (hdr.flags & FLAG_SYNTHETIC_ON)
    assert bool(hdr.flags & FLAG_SURVIVED_REBOOT) == (hdr.boot_count > 0)
    assert hdr.more in (0, 1)
    assert hdr.t_now_us > 0
    if recs:
        assert hdr.first_seq == recs[0].seq > 0
    else:
        assert hdr.first_seq == 0 and hdr.n_records == 0


# ── lossless drain: seq, CMD contents, more, re-ask, ack ─────────────────────

def _probe_commands():
    # Innocuous read-only commands with distinguishable params/echoes.
    return [
        (GET_FRAME_POSITION_CMD, b""),
        (GET_REFRESH_RATE_CMD, b""),
        (GET_PANEL_DISPLAY_MODE_CMD, b""),
        (SET_TELEMETRY_CMD, build_set_telemetry(1, 0x1234)),  # recorded, with its 3 param bytes; synth stays off (bit7 clear)
    ]


def test_drain_is_contiguous_and_cmd_records_match_sent_commands(transport):
    drain(transport)  # start empty
    sent = []
    for i in range(25):
        cmd, params = _probe_commands()[i % 4]
        st, _, _, _ = transport.command(cmd, params)
        sent.append((cmd, params, st))
    recs, headers = drain(transport)  # acked: > 1 block, so an unacked loop could never finish
    assert recs, "commands must produce CMD records"
    assert_contiguous(recs)
    cmds = [r for r in recs if r.type == REC_CMD]
    # Our probes are the CMD records here (the 0xA9 drain itself is never recorded;
    # SET_TELEMETRY also emits a STATE(telemetry) record between the CMDs).
    ours = [r for r in cmds if r.fields["cmd"] != GET_TELEMETRY_BLOCK_CMD]
    assert len(ours) >= len(sent)
    tail = ours[-len(sent):]
    for r, (cmd, params, st) in zip(tail, sent):
        assert r.fields["cmd"] == cmd
        assert r.fields["status"] == st
        assert bytes.fromhex(r.fields["params"]) == params[:8]
    assert not any(r.type == REC_CMD and r.fields["cmd"] == GET_TELEMETRY_BLOCK_CMD for r in recs), \
        "GET_TELEMETRY_BLOCK must not be recorded"
    tel = [r for r in recs if r.type == REC_STATE and r.fields["kind"] == ST_TELEMETRY]
    assert tel and tel[-1].fields["rate_hz"] == 0x1234 and tel[-1].fields["events"]
    # seq is APPEND order, not t_us order: a CMD record carries the dispatch-entry
    # time but is appended after the STATE records its handler produced. Assert the
    # receipt -> effect relation instead: each SET_TELEMETRY's STATE(telemetry) is
    # appended just before its CMD, stamped no earlier than the command's receipt.
    for i, r in enumerate(recs):
        if r.type == REC_CMD and r.fields["cmd"] == SET_TELEMETRY_CMD:
            eff = recs[i - 1]
            assert eff.type == REC_STATE and eff.fields["kind"] == ST_TELEMETRY
            assert (eff.t_us - r.t_us) & 0xFFFFFFFF < 5_000_000, "effect stamped after receipt"
    assert all(h.dropped == headers[0].dropped for h in headers), "no evictions during a small test"


def test_more_semantics_and_reask_is_lossless(transport):
    drain(transport)
    # > 178 B of records: 30 × 13 B CMD records = 390 B → at least 3 chunks.
    for _ in range(30):
        transport.command(GET_FRAME_POSITION_CMD)
    hdr1, recs1, pay1 = get_block(transport)        # no ack
    assert hdr1.more == 1, "first chunk of > 178 B must announce more"
    assert recs1 and hdr1.first_seq == recs1[0].seq
    hdr2, recs2, pay2 = get_block(transport)        # re-ask, same (no) ack
    assert hdr2.first_seq == hdr1.first_seq, "re-asking without an ack returns the same block"
    assert pay2[BLOCK_HEADER_LEN:] == pay1[BLOCK_HEADER_LEN:], "same bytes: nothing was freed"
    # Full drain with acks: contiguous, and `more` is 1 on every chunk but the last.
    recs, headers = drain(transport, ack=True)
    assert recs[0].seq == hdr1.first_seq
    assert_contiguous(recs)
    assert all(h.more == 1 for h in headers[:-1]) and headers[-1].more == 0
    assert len(headers) >= 3
    assert sum(1 for r in recs if r.type == REC_CMD and r.fields["cmd"] == GET_FRAME_POSITION_CMD) >= 30


def test_ack_frees_and_lower_ack_does_not_resurrect(transport):
    drain(transport)
    for _ in range(12):
        transport.command(GET_REFRESH_RATE_CMD)
    hdr, recs, _ = get_block(transport)
    assert len(recs) >= 12
    k = recs[5].seq
    hdr2, recs2, _ = get_block(transport, ack_seq=k)
    assert hdr2.first_seq == k + 1, "ack frees exactly the records with seq <= ack"
    assert recs2[0].seq == k + 1
    hdr3, recs3, _ = get_block(transport, ack_seq=recs[0].seq)  # lower ack: no-op
    assert hdr3.first_seq == k + 1
    hdr4, recs4, _ = get_block(transport, ack_seq=recs2[-1].seq)
    assert hdr4.first_seq == 0 and hdr4.n_records == 0 and hdr4.more == 0


def test_small_max_bytes_returns_whole_records_only(transport):
    drain(transport)
    for _ in range(4):
        transport.command(GET_REFRESH_RATE_CMD)
    hdr, recs, payload = get_block(transport, max_bytes=20)   # room for one 13 B CMD, not two
    assert len(recs) == 1 and hdr.more == 1
    hdr0, recs0, payload0 = get_block(transport, max_bytes=0)   # header only, nothing freed
    assert recs0 == [] and hdr0.first_seq == 0 and hdr0.more == 1
    assert len(payload0) == BLOCK_HEADER_LEN
    drain(transport)


# ── enable / disable ──────────────────────────────────────────────────────────

def test_disable_stops_recording_and_records_the_transition(transport):
    drain(transport)
    enable(transport, False)
    for _ in range(5):
        transport.command(GET_FRAME_POSITION_CMD)
    hdr, recs, _ = get_block(transport)
    assert not (hdr.flags & FLAG_EVENTS_ENABLED)
    # Only the disabling SET_TELEMETRY's own records may be present: its
    # STATE(telemetry, flags=0) (recorded before the gate closed) and its CMD.
    for r in recs:
        if r.type == REC_CMD:
            assert r.fields["cmd"] == SET_TELEMETRY_CMD
        elif r.type == REC_STATE:
            assert r.fields["kind"] == ST_TELEMETRY and r.fields["code"] == 0
        else:
            pytest.fail(f"unexpected record while disabled: {r.as_dict()}")
    assert not any(r.type == REC_CMD and r.fields["cmd"] == GET_FRAME_POSITION_CMD for r in recs)
    enable(transport, True)
    recs, _ = drain(transport)
    tel = [r for r in recs if r.type == REC_STATE and r.fields["kind"] == ST_TELEMETRY]
    assert tel and tel[-1].fields["events"], "re-enable records STATE(telemetry, 1)"
    hdr, _, _ = get_block(transport)
    assert hdr.flags & FLAG_EVENTS_ENABLED


# ── synthetic producer (T1) + overwrite-oldest ───────────────────────────────

def test_synthetic_producer_rate_and_counter(transport):
    drain(transport)
    rate = 500
    set_telemetry(transport, SET_FLAG_EVENTS | SET_FLAG_SYNTHETIC, rate)
    hdr, _, _ = get_block(transport)
    assert hdr.flags & FLAG_SYNTHETIC_ON
    t0 = time.monotonic()
    time.sleep(1.0)
    set_telemetry(transport, SET_FLAG_EVENTS, 0)  # synthetic off (rate 0 also forces off)
    elapsed = time.monotonic() - t0
    hdr, _, _ = get_block(transport)
    assert not (hdr.flags & FLAG_SYNTHETIC_ON)
    recs, headers = drain(transport)
    assert_contiguous(recs)
    synth = [r for r in recs if r.type == REC_CMD and r.fields["cmd"] == SYNTHETIC_CMD]
    expected = rate * elapsed
    assert abs(len(synth) - expected) <= 0.1 * expected + 5, f"{len(synth)} synthetic records for {elapsed:.2f} s at {rate}/s"
    assert all(r.fields["status"] == 0 for r in synth)
    counters = [r.fields["counter"] for r in synth]
    assert counters == list(range(len(counters))), "counter restarts at 0 on enable and is contiguous"
    # Period from the controller's own timestamps: 2 ms ± 10 %.
    dts = [(b.t_us - a.t_us) & 0xFFFFFFFF for a, b in zip(synth, synth[1:])]
    assert dts and abs(sum(dts) / len(dts) - 1_000_000 / rate) < 0.1 * 1_000_000 / rate
    assert headers[-1].dropped == headers[0].dropped, "1 s at 500/s (8.5 KB) must not overfill 64 KiB"


def test_overfill_evicts_oldest_and_reports_dropped(transport):
    recs0, _ = drain(transport)
    hdr0, _, _ = get_block(transport)
    dropped_before = hdr0.dropped
    # Leave a known marker record at the head, then overfill without draining:
    # 17 B synthetic records × 8000/s × 1.0 s ≈ 136 KB > 65,504 B.
    transport.command(GET_REFRESH_RATE_CMD)
    hdr_m, marker, _ = get_block(transport)
    head_seq = marker[0].seq
    # Phase A — modest overflow (~5000 × 17 B = 85 KB into a 65.5 KB ring): the
    # marker written at the FIRST eviction sits ~3850 records in and is still in
    # the ring when we drain, because fewer than a ring-full of records followed it.
    set_telemetry(transport, SET_FLAG_EVENTS | SET_FLAG_SYNTHETIC, 5000)
    time.sleep(1.0)
    set_telemetry(transport, SET_FLAG_EVENTS, 0)
    hdr1, recs1, _ = get_block(transport)
    evicted = hdr1.dropped - dropped_before
    assert evicted > 0, "overfilling must evict"
    assert hdr1.first_seq > head_seq, "the OLDEST records were evicted, the marker among them"
    # The seq gap between the last record we hold and the first still in the
    # ring is exactly the eviction count (PADs are not records).
    assert hdr1.first_seq - head_seq == evicted
    recs, headers = drain(transport)
    assert_contiguous(recs)
    assert len(recs) * 13 <= RING_DATA_BYTES
    overruns = [r for r in recs if r.type == REC_STATE and r.fields["kind"] == ST_RING_OVERRUN]
    assert overruns, "the first eviction leaves a STATE(ring_overrun) marker (near the first overflow)"
    assert all(r.fields["code"] == OVERRUN_CODE_EVICTED for r in overruns)
    assert not any(r.fields["heap_collision"] for r in overruns)
    synth = [r for r in recs if r.type == REC_CMD and r.fields["cmd"] == SYNTHETIC_CMD]
    first_marker_pos = recs.index(overruns[0])
    assert first_marker_pos < len(recs) // 2, "marker belongs to the start of the overflow, not its end"
    # The newest synthetic records survived (overwrite-oldest, not drop-newest).
    assert synth and synth[-1].fields["counter"] == max(r.fields["counter"] for r in synth)
    assert synth[-1].fields["counter"] + 1 >= len(synth) + evicted - 10, "counter accounts for evicted records"

    # Phase B — sustained overflow (~16000 records, > 4 ring-fulls): the marker is
    # NOT guaranteed to survive; the contract is dropped + seq gaps. The marker's
    # own eviction is counted in `dropped`, so gap == dropped still holds exactly.
    transport.command(GET_REFRESH_RATE_CMD)
    hdr_m, marker, _ = get_block(transport)
    head_seq = marker[0].seq
    dropped_before = hdr_m.dropped
    set_telemetry(transport, SET_FLAG_EVENTS | SET_FLAG_SYNTHETIC, 8000)
    time.sleep(2.0)
    set_telemetry(transport, SET_FLAG_EVENTS, 0)
    hdr2, _, _ = get_block(transport)
    evicted2 = hdr2.dropped - dropped_before
    assert evicted2 > 3 * len(recs), "sustained overflow evicts several ring-fulls"
    assert hdr2.first_seq - head_seq == evicted2, "seq gap == dropped, with or without a surviving marker"
    recs2, _ = drain(transport)
    assert_contiguous(recs2)


# ── frame storm (Mode 3) ──────────────────────────────────────────────────────

def test_frame_storm_yields_frame_records_in_index_order(transport, pat):
    st, _, payload, _ = transport.command(GET_PATTERN_INFO_CMD, struct.pack("<H", pat))
    assert st == 0
    frame_count = struct.unpack_from("<H", bytes(payload))[0]
    if frame_count < 2:
        pytest.skip(f"--pat has only {frame_count} frame(s); need >= 2")

    drain(transport)
    tp = (bytes([3]) + struct.pack("<H", pat) + struct.pack("<h", 0)
          + struct.pack("<H", 0) + struct.pack("<h", 0) + struct.pack("<H", 0))
    st, _, _, _ = transport.command(TRIAL_PARAMS_CMD, tp, timeout=10.0)
    assert st == 0, "TRIAL_PARAMS (mode 3) failed"
    n = 200
    sent_idx = []
    try:
        for i in range(n):
            idx = i % frame_count
            st, _, _, _ = transport.command(SET_FRAME_POSITION_CMD, struct.pack("<H", idx))
            assert st == 0
            sent_idx.append(idx)
            time.sleep(0.005)  # > one 300 Hz refresh period, so each index gets displayed
        time.sleep(0.05)
    finally:
        transport.command(STOP_DISPLAY_CMD)

    recs, headers = drain(transport)
    assert_contiguous(recs)
    assert headers[-1].dropped == headers[0].dropped, "7.2 KB of records must not overfill a 64 KiB ring"

    states = [r for r in recs if r.type == REC_STATE]
    assert any(r.fields["kind"] == ST_SD_OPEN and r.fields["code"] == 0 and r.fields["arg"] == pat
               for r in states), "trial start records STATE(sd_open, ok, pattern)"
    assert any(r.fields["kind"] == ST_STATE_CHANGE and r.fields["code"] == STATE_SHOW_FRAME
               for r in states), "trial start records STATE(state_change, SHOW_FRAME)"
    assert states[-1].fields["kind"] == ST_STATE_CHANGE and states[-1].fields["code"] == STATE_ALL_OFF, \
        "STOP records STATE(state_change, ALL_OFF) last"

    cmd70 = [r for r in recs if r.type == REC_CMD and r.fields["cmd"] == SET_FRAME_POSITION_CMD]
    assert len(cmd70) == n
    assert [struct.unpack("<H", bytes.fromhex(r.fields["params"]))[0] for r in cmd70] == sent_idx
    assert all(r.fields["status"] == 0 for r in cmd70)

    frames = [r for r in recs if r.type == REC_FRAME]
    assert len(frames) >= 150, f"expected ~{n} FRAME records at a 5 ms pace, got {len(frames)}"
    assert all(len(r.raw) == 20 for r in frames), "FRAME record is 20 B (sd_load_us is u32)"
    got_idx = [r.fields["idx"] for r in frames]
    # The displayed sequence is an in-order subsequence of the commanded one
    # (two 0x70 inside one refresh period display only the second), and it ends
    # on the last commanded index. The trial-start frame (index 0) leads.
    assert got_idx[0] == 0
    assert got_idx[-1] == sent_idx[-1]
    it = iter(sent_idx)
    for idx in got_idx[1:]:
        assert any(s == idx for s in it), f"FRAME idx {idx} is not in commanded order"
    assert all(r.fields["pattern"] == pat for r in frames)
    assert all(r.fields["sd_load_us"] > 0 for r in frames), "each frame came from an SD read"
    assert all(r.fields["spi_us"] > 0 for r in frames)
    # Causality: every FRAME follows a CMD(0x70) for its index.
    seen = {}
    for r in recs:
        if r.type == REC_CMD and r.fields["cmd"] == SET_FRAME_POSITION_CMD:
            seen[struct.unpack("<H", bytes.fromhex(r.fields["params"]))[0]] = r.t_us
        elif r.type == REC_FRAME and r.fields["idx"] in seen:
            assert (r.t_us - seen[r.fields["idx"]]) & 0xFFFFFFFF < 5_000_000


# ── crash-dump semantics (opt-in: reboots the controller) ────────────────────

@pytest.mark.serial_only
@pytest.mark.skipif(not os.environ.get("TELEMETRY_RESET_OK"),
                    reason="reboots the controller — set TELEMETRY_RESET_OK=1 to run")
def test_ring_survives_system_reset(transport):
    drain(transport)
    hdr, recs, _ = get_block(transport)
    assert recs == []
    # Leave one pre-reset record unacked so we can see it survive.
    st, _, payload, _ = transport.command(GET_REFRESH_RATE_CMD)
    hdr, recs, _ = get_block(transport)
    last_seq = recs[-1].seq
    st, _, payload, _ = transport.command(SYSTEM_RESET_CMD)
    assert st == 0 and bytes(payload) == b"rebooting"
    transport.close()
    time.sleep(3.0)
    deadline = time.monotonic() + 20.0
    while True:
        try:
            transport.open()
            transport.command(GET_CONTROLLER_INFO_CMD)
            break
        except Exception:  # noqa: BLE001 — the CDC node comes and goes during a reboot
            if time.monotonic() > deadline:
                raise
            time.sleep(0.5)
    hdr, recs, _ = get_block(transport)
    assert hdr.flags & FLAG_SURVIVED_REBOOT and hdr.boot_count >= 1
    assert recs, "the kept ring must still hold the pre-reset records"
    assert recs[0].type == REC_CMD and recs[0].fields["cmd"] == GET_REFRESH_RATE_CMD
    # Pre-reset records first, then STATE(boot) with the next seq.
    boot = [r for r in recs if r.type == REC_STATE and r.fields["kind"] == ST_BOOT]
    assert boot, "STATE(boot) is appended at every boot"
    assert boot[0].seq == last_seq + 1, "seq continues across the reboot (0x01 itself is not recorded)"
    assert boot[0].fields["code"] & 0x40, "SRC_SRSR LOCKUP_SYSRESETREQ (software reset) in the low byte"
    assert boot[0].fields["prev_breadcrumb_valid"] == 1
    assert boot[0].fields["prev_breadcrumb_op"] == OP_CMD
    drain(transport)
