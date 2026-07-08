"""LAB-82 AO LUT playback tests.

Verifies the SET_AO_LUT_CMD (0xA2) implementation:

  T1  lut_frame_locked_ramp    : upload a 5-entry ramp, step frames via
                                  SET_FRAME_POSITION, verify DAC follows LUT[frame%5]
  T2  lut_time_based_advances  : upload a time-based LUT at 5 Hz, wait,
                                  confirm DAC has moved beyond LUT[0]
  T3  lut_cleared_by_set_ao    : upload a LUT, then 0xA0 — DAC goes static,
                                  subsequent frame advance must not change it
  T4  lut_wrap                 : upload a 3-entry LUT, step frame index past end,
                                  confirm modulo wrap (frame 3 → LUT[0], etc.)

T1 and T4 require a multi-frame .pat file via --pat (frame_count >= 8).
T2 and T3 run without SD access.

Run:
    pixi run test-serial                       # T2 + T3 only
    pixi run test-serial -- --pat arena.pat    # all four tests
    pixi run test-tcp    -- --ip 10.0.0.x --pat arena.pat
"""

import struct
import time

import pytest

from .commands import (
    ALL_OFF_CMD,
    DELETE_ALL_PATTERNS_CMD,
    GET_AO_VOLTAGE_CMD,
    SET_AO_LUT_CMD,
    SET_AO_VOLTAGE_CMD,
    SET_FRAME_POSITION_CMD,
    STOP_DISPLAY_CMD,
    TRIAL_PARAMS_CMD,
)

# DAC quantization: 12-bit over 0-5000 mV → ~1.22 mV/LSB; allow 3 mV tolerance.
_AO_TOLERANCE_MV = 3


def _enc_lut(mode: int, step_hz: int, mv_list: list) -> bytes:
    """Build the SET_AO_LUT_CMD params: [mode, step_hz LE, count LE, mv... LE]."""
    params = bytes([mode]) + struct.pack("<H", step_hz) + struct.pack("<H", len(mv_list))
    for mv in mv_list:
        params += struct.pack("<H", mv)
    return params


def _enc_trial_params(mode: int, pattern_id: int, frame_rate: int = 10,
                      init_pos: int = 0, gain: int = 0,
                      duration_ticks: int = 0) -> bytes:
    """Wire order: mode, pattern_id, frame_rate, init_pos, gain (int16), duration."""
    return (bytes([mode])
            + struct.pack("<H", pattern_id)
            + struct.pack("<h", frame_rate)
            + struct.pack("<H", init_pos)
            + struct.pack("<h", gain)
            + struct.pack("<H", duration_ticks))


def _get_ao_mv(transport) -> int:
    """Read the MCP4725 DAC register and return the current level in mV."""
    st, _, payload, _ = transport.command(GET_AO_VOLTAGE_CMD)
    assert st == 0, f"GET_AO_VOLTAGE failed: status={st}"
    return struct.unpack("<H", payload[:2])[0]


# ── fixtures ──────────────────────────────────────────────────────────────────

@pytest.fixture(autouse=True)
def reset_ao(transport):
    """Before each test: stop display and drive AO to 0 (clears any active LUT)."""
    transport.command(ALL_OFF_CMD)
    st, _, _, _ = transport.command(SET_AO_VOLTAGE_CMD, struct.pack("<H", 0))
    assert st == 0, "reset_ao: SET_AO_VOLTAGE 0 mV failed — MCP4725 I²C not responding"
    yield
    transport.command(STOP_DISPLAY_CMD)
    transport.command(SET_AO_VOLTAGE_CMD, struct.pack("<H", 0))


@pytest.fixture
def uploaded_pat(transport, pat_data):
    """Upload --pat to SD, promote it, yield its 1-based pattern index.

    The pattern must have at least 8 frames for T1 / T4 to be meaningful.
    """
    if pat_data is None:
        pytest.skip("--pat not provided; pass a multi-frame .pat to run this test")
    transport.command(DELETE_ALL_PATTERNS_CMD, timeout=15.0)
    st, _, _, _ = transport.upload_file(0, pat_data, timeout=120.0)
    assert st == 0, "setup: upload_file failed"
    st2, _, payload, _ = transport.rename_file(0, "lab82.pat")
    assert st2 == 0, "setup: rename_file failed"
    yield struct.unpack("<H", payload)[0]
    transport.command(DELETE_ALL_PATTERNS_CMD, timeout=15.0)


# ── T1: frame-locked ramp ─────────────────────────────────────────────────────

def test_lut_frame_locked_ramp(transport, uploaded_pat):
    """Frame-locked: DAC must follow LUT[frame_index % 5] at each frame position."""
    lut = [0, 1250, 2500, 3750, 5000]

    # Enter SHOW_FRAME mode (mode=3) at frame 0.
    params = _enc_trial_params(mode=3, pattern_id=uploaded_pat, init_pos=0)
    st, _, _, _ = transport.command(TRIAL_PARAMS_CMD, params, timeout=10.0)
    assert st == 0, "trial-params mode=3 rejected"

    # Upload frame-locked LUT (mode=0, step_hz ignored).
    st, _, payload, _ = transport.command(SET_AO_LUT_CMD, _enc_lut(0, 0, lut))
    assert st == 0, "SET_AO_LUT rejected"
    count_back = struct.unpack("<H", payload[:2])[0]
    assert count_back == len(lut)

    # Check each frame → LUT mapping via SET_FRAME_POSITION.
    for frame_idx in [0, 1, 2, 3, 4]:
        st_fp, _, _, _ = transport.command(
            SET_FRAME_POSITION_CMD, struct.pack("<H", frame_idx))
        assert st_fp == 0, f"SET_FRAME_POSITION {frame_idx} rejected"
        mv = _get_ao_mv(transport)
        expected = lut[frame_idx % len(lut)]
        assert abs(mv - expected) <= _AO_TOLERANCE_MV, (
            f"frame {frame_idx}: AO={mv} mV, expected LUT[{frame_idx % len(lut)}]={expected} mV"
        )


# ── T2: time-based LUT advances ───────────────────────────────────────────────

def test_lut_time_based_advances(transport):
    """Time-based: DAC must advance beyond LUT[0] without any display running."""
    lut = [0, 1000, 2000, 3000, 4000]

    # Upload time-based LUT (mode=1) at 5 Hz. LUT[0] = 0 mV applied immediately.
    st, _, _, _ = transport.command(SET_AO_LUT_CMD, _enc_lut(1, 5, lut))
    assert st == 0, "SET_AO_LUT (time-based) rejected"

    mv_initial = _get_ao_mv(transport)
    assert mv_initial == 0, f"expected LUT[0]=0 immediately after upload, got {mv_initial} mV"

    # Wait for at least one step (period = 200 ms at 5 Hz; sleep 400 ms for margin).
    time.sleep(0.4)

    mv_after = _get_ao_mv(transport)
    assert mv_after > 0, (
        f"expected AO to advance from 0 after 400 ms at 5 Hz, still at {mv_after} mV"
    )


# ── T3: LUT cleared by SET_AO_VOLTAGE ────────────────────────────────────────

def test_lut_cleared_by_set_ao_voltage(transport):
    """SET_AO_VOLTAGE (0xA0) must stop the LUT and drive a static level."""
    lut = [0, 500, 1000, 1500, 2000]

    # Start a time-based LUT.
    st, _, _, _ = transport.command(SET_AO_LUT_CMD, _enc_lut(1, 10, lut))
    assert st == 0, "SET_AO_LUT rejected"

    # Override with a static level — this clears ao_lut_len_.
    target_mv = 2500
    st_ao, _, payload, _ = transport.command(SET_AO_VOLTAGE_CMD, struct.pack("<H", target_mv))
    assert st_ao == 0, f"SET_AO_VOLTAGE {target_mv} mV rejected"
    echo_mv = struct.unpack("<H", payload[:2])[0]
    assert abs(echo_mv - target_mv) <= _AO_TOLERANCE_MV, (
        f"SET_AO_VOLTAGE response: got {echo_mv} mV, expected {target_mv} mV"
    )

    # Wait long enough for multiple LUT steps that would have fired — AO must stay put.
    time.sleep(0.3)
    mv = _get_ao_mv(transport)
    assert abs(mv - target_mv) <= _AO_TOLERANCE_MV, (
        f"AO drifted after SET_AO_VOLTAGE: got {mv} mV, expected static {target_mv} mV"
    )


# ── T4: wrap ──────────────────────────────────────────────────────────────────

def test_lut_wrap(transport, uploaded_pat):
    """Frame-locked: LUT must wrap with modulo indexing when frame_index >= count."""
    lut = [0, 2500, 5000]   # 3 entries

    params = _enc_trial_params(mode=3, pattern_id=uploaded_pat, init_pos=0)
    st, _, _, _ = transport.command(TRIAL_PARAMS_CMD, params, timeout=10.0)
    assert st == 0, "trial-params mode=3 rejected"

    st, _, _, _ = transport.command(SET_AO_LUT_CMD, _enc_lut(0, 0, lut))
    assert st == 0, "SET_AO_LUT rejected"

    # frame 0 → LUT[0]=0, frame 3 → LUT[3%3=0]=0, frame 4 → LUT[4%3=1]=2500
    wrap_cases = [(0, 0), (3, 0), (4, 2500), (5, 5000), (6, 0)]
    for frame_idx, expected_mv in wrap_cases:
        st_fp, _, _, _ = transport.command(
            SET_FRAME_POSITION_CMD, struct.pack("<H", frame_idx))
        assert st_fp == 0, f"SET_FRAME_POSITION {frame_idx} rejected"
        mv = _get_ao_mv(transport)
        assert abs(mv - expected_mv) <= _AO_TOLERANCE_MV, (
            f"frame {frame_idx}: AO={mv} mV, expected LUT[{frame_idx % len(lut)}]={expected_mv} mV"
        )
