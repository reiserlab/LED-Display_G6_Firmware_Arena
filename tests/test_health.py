"""GET_HEALTH (0xCA) — issue #50 controller health telemetry.

Read-only opcode a soak harness polls at ~1 Hz (and after a fault) to see
controller-side state during Mode-3 host streaming: loop timing, SD read
timing + SdFat error code, SPI frame/ISR counters, 0x70 count, ArenaState,
reset cause, and the previous boot's reset-surviving breadcrumb.

Nothing is clear-on-read: counters are cumulative since boot, so tests here
diff two samples rather than asserting absolute values.

NON-DESTRUCTIVE: the 0x70 counter test uses the session `pat` fixture (adds
one 'conftest.pat', deletes nothing) and skips without --pat.

Run:
    pixi run test-serial -- --port /dev/cu.usbmodemXXX
    ... -- --pat .../g6_2x10/patterns/004_frame2_h_ccw_200f.pat   # + 0x70 count
"""

import struct
import time
from collections import namedtuple

import pytest

from .commands import (
    GET_CONTROLLER_INFO_CMD,
    GET_FRAMES_SENT_CMD,
    GET_HEALTH_CMD,
    GET_PATTERN_INFO_CMD,
    SET_FRAME_POSITION_CMD,
    STOP_DISPLAY_CMD,
    TRIAL_PARAMS_CMD,
)

# Payload layout — mirrors CommandProcessor::handleGetHealth() (all LE).
HEALTH_FMT = "<BBIIIIIIBIIIIBHIBIBBIBI"
HEALTH_LEN = struct.calcsize(HEALTH_FMT)  # 66
HEALTH_VER = 1

Health = namedtuple(
    "Health",
    [
        "ver", "flags", "uptime_ms", "loop_count", "loop_max_us", "loop_max_1s_us",
        "sd_reads", "sd_read_max_us", "sd_err", "sd_err_data",
        "frames_sent", "isr_count", "cmd70_count", "state", "cur_frame", "reset_cause",
        "prev_breadcrumb", "prev_breadcrumb_us", "prev_breadcrumb_arg",
        "prev_slow_op", "prev_slow_us", "slow_op", "slow_us",
    ],
)

# flags bits
FLAG_SD_MOUNTED = 0x01
FLAG_PATTERN_OPEN = 0x02
FLAG_DISPLAY_ACTIVE = 0x04
FLAG_BREADCRUMB_VALID = 0x08

# ArenaState (CommandProcessor.h)
STATE_ALL_OFF = 0
STATE_SHOW_FRAME = 4
STATE_MAX = 7  # ERROR_DISPLAY

# Health::LastOp
OP_IDLE, OP_SD_READ, OP_SPI_FRAME, OP_USB_WRITE, OP_CMD, OP_SD_OPEN = range(6)

CAP_HEALTH = 0x80  # GET_CONTROLLER_INFO capability bit 7


def read_health(transport) -> Health:
    st, echo, payload, _ = transport.command(GET_HEALTH_CMD)
    assert st == 0, f"GET_HEALTH failed status={st}"
    assert echo == GET_HEALTH_CMD
    assert len(payload) == HEALTH_LEN, f"expected {HEALTH_LEN}B payload, got {len(payload)}B"
    return Health._make(struct.unpack(HEALTH_FMT, bytes(payload)))


# ── shape + capability ────────────────────────────────────────────────────────

def test_health_reply_shape(transport):
    h = read_health(transport)
    assert h.ver == HEALTH_VER
    assert h.uptime_ms > 0
    assert h.loop_count > 0
    assert h.state <= STATE_MAX
    assert h.slow_op <= OP_SD_OPEN
    assert h.prev_breadcrumb <= OP_SD_OPEN
    assert h.prev_slow_op <= OP_SD_OPEN
    # A live loop has measured at least one iteration by the time a host talks to it.
    assert h.loop_max_us > 0


def test_controller_info_advertises_health(transport):
    st, echo, payload, _ = transport.command(GET_CONTROLLER_INFO_CMD)
    assert st == 0 and echo == GET_CONTROLLER_INFO_CMD
    assert len(payload) >= 2
    assert payload[1] & CAP_HEALTH, "capability bit 7 (health) must be set when 0xCA exists"


# ── counters are live, cumulative, never cleared by a read ────────────────────

def test_uptime_and_loop_count_advance(transport):
    a = read_health(transport)
    time.sleep(0.25)
    b = read_health(transport)
    assert b.uptime_ms > a.uptime_ms
    assert b.uptime_ms - a.uptime_ms >= 200, "uptime should track wall clock (~250 ms elapsed)"
    assert b.loop_count > a.loop_count
    # Cumulative maxima never decrease between reads (no clear-on-read).
    assert b.loop_max_us >= a.loop_max_us
    assert b.sd_read_max_us >= a.sd_read_max_us
    assert b.sd_reads >= a.sd_reads
    assert b.slow_us >= a.slow_us


def test_loop_max_1s_window_is_populated(transport):
    # The rolling window publishes the max of the last COMPLETED 1 s window; after
    # >1 s of uptime it is non-zero and bounded by the since-boot max.
    h = read_health(transport)
    if h.uptime_ms < 1500:
        time.sleep(1.5)
        h = read_health(transport)
    assert h.loop_max_1s_us > 0
    assert h.loop_max_1s_us <= h.loop_max_us


def test_frames_sent_matches_get_frames_sent(transport):
    # Both read the same SpiManager counter; with the display stopped (no
    # refresh traffic) two consecutive reads must agree exactly.
    st, _, _, _ = transport.command(STOP_DISPLAY_CMD)
    assert st == 0
    h = read_health(transport)
    st, echo, payload, _ = transport.command(GET_FRAMES_SENT_CMD)
    assert st == 0 and echo == GET_FRAMES_SENT_CMD
    (frames_sent,) = struct.unpack("<I", bytes(payload[:4]))
    assert h.frames_sent == frames_sent
    assert h.isr_count >= 0  # exists; value depends on how long refresh ran


def test_sd_healthy(transport):
    h = read_health(transport)
    assert h.flags & FLAG_SD_MOUNTED, "SD card must be mounted for the HIL suite"
    assert h.sd_err == 0, f"SdFat card errorCode={h.sd_err:#x} data={h.sd_err_data:#x}"


def test_state_all_off_after_stop(transport):
    st, _, _, _ = transport.command(STOP_DISPLAY_CMD)
    assert st == 0
    h = read_health(transport)
    assert h.state == STATE_ALL_OFF
    assert not (h.flags & FLAG_DISPLAY_ACTIVE)


def test_reset_cause_is_plausible(transport):
    # SRC_SRSR: some cause bit is always set after any reset (POR, pin, or
    # LOCKUP_SYSRESETREQ for the 0x01 software reset). The defined flags are
    # bits 0..8 (imxrt.h SRC_SRSR_*); the firmware clears the register after
    # capture, so a value here is THIS boot's cause, not an accumulation.
    h = read_health(transport)
    assert h.reset_cause & 0x1FF, f"no defined SRSR cause bit set: {h.reset_cause:#x}"


def test_breadcrumb_consistency(transport):
    # A breadcrumb harvested from the previous boot must be self-consistent;
    # without one (power-on), all prev_* fields must read as zero.
    h = read_health(transport)
    if h.flags & FLAG_BREADCRUMB_VALID:
        assert h.prev_slow_op <= OP_SD_OPEN
        if h.prev_breadcrumb != OP_CMD:
            assert h.prev_breadcrumb_arg == 0
    else:
        assert h.prev_breadcrumb == 0
        assert h.prev_breadcrumb_us == 0
        assert h.prev_breadcrumb_arg == 0
        assert h.prev_slow_op == 0
        assert h.prev_slow_us == 0


# ── 0x70 counter (needs a pattern open) ──────────────────────────────────────

def test_cmd70_count_increments_per_set_frame_position(transport, pat):
    st, _, payload, _ = transport.command(GET_PATTERN_INFO_CMD, struct.pack("<H", pat))
    assert st == 0, "GET_PATTERN_INFO failed"
    frame_count = struct.unpack_from("<H", bytes(payload))[0]
    if frame_count < 2:
        pytest.skip(f"--pat has only {frame_count} frame(s); need >= 2")

    # Mode 3, no auto-advance, no duration auto-stop: park on frame 0.
    tp = (bytes([3]) + struct.pack("<H", pat) + struct.pack("<h", 0)
          + struct.pack("<H", 0) + struct.pack("<h", 0) + struct.pack("<H", 0))
    st, _, _, _ = transport.command(TRIAL_PARAMS_CMD, tp, timeout=10.0)
    assert st == 0, "TRIAL_PARAMS (mode 3) failed"
    try:
        before = read_health(transport)
        assert before.flags & FLAG_PATTERN_OPEN
        assert before.state == STATE_SHOW_FRAME
        assert before.flags & FLAG_DISPLAY_ACTIVE

        n = 20
        for i in range(n):
            st, _, _, _ = transport.command(SET_FRAME_POSITION_CMD,
                                            struct.pack("<H", i % frame_count))
            assert st == 0, f"SET_FRAME_POSITION #{i} failed"

        after = read_health(transport)
        assert after.cmd70_count - before.cmd70_count == n
        assert after.sd_reads - before.sd_reads >= n, "each 0x70 reads one frame from SD"
        assert after.sd_read_max_us > 0
        assert after.cur_frame == (n - 1) % frame_count
        assert after.sd_err == 0
    finally:
        transport.command(STOP_DISPLAY_CMD)
