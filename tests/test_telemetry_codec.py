"""Offline (no hardware) fixtures for the telemetry ring codec — STATE kinds 8/9/10.

Runs inside the HIL suite but needs no transport. Builds records byte-for-byte per
src/Telemetry.h and checks the decoder's derived fields.
"""

import struct

from .telemetry_codec import (
    ISR_NAMES,
    ST_PREV_ISR_COUNT,
    ST_TIMER_FAIL,
    ST_WDOG_CONTEXT,
    classify_exc_return,
    parse_block,
)


def _state(seq, kind, code, arg, t_us=1234):
    return bytes([14, 3]) + struct.pack("<II", seq, t_us) + struct.pack("<BBH", kind, code, arg)


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
