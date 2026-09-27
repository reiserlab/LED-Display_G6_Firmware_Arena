"""Offline (no hardware) fixtures for the telemetry ring codec — STATE kinds 8/9/10, FRAME v1/v2, kinds 11-13.

Runs inside the HIL suite but needs no transport. Builds records byte-for-byte per
src/Telemetry.h and checks the decoder's derived fields.
"""

import struct

import pytest

from .telemetry_codec import (
    FRAME_RECORD_LEN,
    ISR_NAMES,
    ST_PREV_ISR_COUNT,
    ST_SD_LAYOUT,
    ST_SD_READS,
    ST_SD_READS_CKPT,
    ST_SD_SLOW,
    ST_SD_SLOW_CTX,
    ST_TIMER_FAIL,
    ST_WDOG_CONTEXT,
    classify_exc_return,
    parse_block,
)


def _state(seq, kind, code, arg, t_us=1234):
    return bytes([14, 3]) + struct.pack("<II", seq, t_us) + struct.pack("<BBH", kind, code, arg)


def _frame(seq, idx, pattern, sd_load_us, spi_us, req_age_us=None, superseded=0, flags=0, t_us=5678):
    body = struct.pack("<HHIH", idx, pattern, sd_load_us, spi_us)
    if req_age_us is not None:
        body += struct.pack("<IBB", req_age_us, superseded, flags)
    return bytes([10 + len(body), 2]) + struct.pack("<II", seq, t_us) + body


def _block(*recs):
    body = b"".join(recs)
    hdr = struct.pack("<IIHIBBH", 99, struct.unpack_from("<I", recs[0], 2)[0], len(recs), 0, 0, 0x03, 1)
    return parse_block(hdr + body)[1]


def test_wdog_context_thread_preempted_no_fp():
    # EXC_RETURN 0xFFFFFFF9: thread mode, no FP state; IPSR 0; prior isr 0 (main loop)
    (r,) = _block(_state(1, ST_WDOG_CONTEXT, 0xF9, 0))
    f = r.fields
    assert f["kind_name"] == "wdog_context" and f["preempted"] == "thread"
    assert f["ipsr"] == 0 and f["ipsr_irq"] is None and f["prior_isr"] == 0 and f["prior_isr_name"] == "none"
    assert f["fp_stacked"] is False


def test_wdog_context_handler_preempted_with_fp_pit_storm():
    # EXC_RETURN 0xFFFFFFE1: handler mode, FP state stacked; IPSR 138 (= IRQ 122 PIT); prior isr 7 (pit)
    arg = 138 | (7 << 9)
    (r,) = _block(_state(2, ST_WDOG_CONTEXT, 0xE1, arg))
    f = r.fields
    assert f["preempted"] == "handler" and f["fp_stacked"] is True
    assert f["ipsr"] == 138 and f["ipsr_irq"] == 122
    assert f["prior_isr"] == 7 and f["prior_isr_name"] == "pit"


def test_wdog_context_thread_with_fp():
    (r,) = _block(_state(3, ST_WDOG_CONTEXT, 0xE9, 0 | (4 << 9)))
    assert r.fields["preempted"] == "thread" and r.fields["prior_isr_name"] == "usb"


def test_classify_exc_return_table():
    assert [classify_exc_return(x) for x in (0xF9, 0xE9, 0xFD, 0xED)] == ["thread"] * 4
    assert [classify_exc_return(x) for x in (0xF1, 0xE1)] == ["handler"] * 2
    assert classify_exc_return(0x00) == "unknown" and classify_exc_return(0xF5) == "unknown"


def test_prev_isr_count_units():
    (r,) = _block(_state(4, ST_PREV_ISR_COUNT, 7, 1000))
    assert r.fields["kind_name"] == "prev_isr_count"
    assert r.fields["isr_name"] == "pit" and r.fields["entries_approx"] == 1000 << 12
    assert ISR_NAMES[7] == "pit"


def test_timer_fail_is_not_a_telemetry_config():
    (r,) = _block(_state(5, ST_TIMER_FAIL, 0, 300))
    assert r.fields["kind_name"] == "timer_fail" and r.fields["requested_hz"] == 300
    assert "events" not in r.fields  # must not be mistaken for STATE(telemetry, flags)


# ── FRAME v1 (20 B) / v2 (26 B) ─────────────────────────────────────────────

def test_frame_v1_record_decodes_without_extras():
    (r,) = _block(_frame(6, 78, 36, 1961, 812))
    assert len(r.raw) == 20
    assert r.fields == {"idx": 78, "pattern": 36, "sd_load_us": 1961, "spi_us": 812}


def test_frame_v2_record_carries_request_age_and_flags():
    (r,) = _block(_frame(7, 152, 36, 88_700, 771, req_age_us=91_200, superseded=3, flags=0x03))
    assert len(r.raw) == FRAME_RECORD_LEN == 26
    f = r.fields
    assert f["req_age_us"] == 91_200, "u32: a 30–90 ms card stall must be representable (u16 clips at 65 ms)"
    assert f["superseded"] == 3 and f["flags"] == 3
    assert f["sd_read"] is True and f["contiguous"] is True
    assert "extra" not in f


def test_frame_longer_than_v2_keeps_the_tail_raw():
    rec = _frame(8, 1, 5, 620, 770, req_age_us=4700, superseded=0, flags=1)
    rec = bytes([rec[0] + 2]) + rec[1:] + b"\xaa\xbb"
    (r,) = _block(rec)
    assert r.fields["req_age_us"] == 4700 and r.fields["extra"] == "aabb"


def test_frame_shorter_than_v1_is_rejected():
    rec = bytes([18, 2]) + struct.pack("<II", 9, 1) + struct.pack("<HHI", 1, 5, 620)
    with pytest.raises(ValueError):
        _block(rec)


# ── STATE kinds 11-13 + sd_slow phase byte (ring v2) ────────────────────────

def test_sd_slow_phase_and_error_flag():
    (r,) = _block(_state(10, ST_SD_SLOW, 0x02, 887))          # body phase, no error, 88.7 ms
    assert r.fields["kind_name"] == "sd_slow" and r.fields["read_us"] == 88_700
    assert r.fields["phase"] == "body" and r.fields["read_error"] is False
    (r,) = _block(_state(11, ST_SD_SLOW, 0x81, 12))            # seek phase, read returned an error
    assert r.fields["phase"] == "seek" and r.fields["read_error"] is True
    (r,) = _block(_state(12, ST_SD_SLOW, 0x00, 330))           # ring-v1 firmware: code 0
    assert r.fields["phase"] == "unknown" and r.fields["read_us"] == 33_000


def test_sd_layout_slow_ctx_and_reads():
    (a, b, c, d) = _block(_state(13, ST_SD_LAYOUT, 0x01, 8),
                          _state(14, ST_SD_SLOW_CTX, 0, 0x0001),
                          _state(15, ST_SD_READS, 0, 24_573),
                          _state(16, ST_SD_READS, 3, 45_000))   # 360,000 reads, shift 3
    assert a.fields["kind_name"] == "sd_layout" and a.fields["contiguous"] is True
    assert a.fields["exfat"] is False and a.fields["sectors_per_cluster"] == 8
    assert a.fields["legacy_seek"] is False and a.fields["no_same_index_skip"] is False
    (arm,) = _block(_state(18, ST_SD_LAYOUT, 0x0C, 8))   # legacy seek + no skip, file not flagged contiguous
    assert arm.fields["legacy_seek"] is True and arm.fields["no_same_index_skip"] is True and arm.fields["contiguous"] is False
    assert b.fields["kind_name"] == "sd_slow_ctx" and b.fields["card_error_code"] == 0
    assert b.fields["irqstat_hi"] == 1 and b.fields["driver_saw_error"] is False
    assert c.fields["kind_name"] == "sd_reads" and c.fields["reads"] == 24_573 and c.fields["checkpoint"] is False
    assert d.fields["reads"] == 360_000
    (e,) = _block(_state(17, ST_SD_READS_CKPT, 0, 30_000))
    assert e.fields["kind_name"] == "sd_reads_ckpt" and e.fields["reads"] == 30_000 and e.fields["checkpoint"] is True
    (f,) = _block(_state(18, ST_SD_READS, 0x80, 30_000))   # legacy encoding (kind 13, code bit 7) still a checkpoint
    assert f.fields["kind_name"] == "sd_reads" and f.fields["reads"] == 30_000 and f.fields["checkpoint"] is True
