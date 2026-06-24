"""LAB-79 SD card verification tests.

Complements test_sd_files.py (which covers basic upload/rename/download/delete
with a small payload) with the four items needed to close LAB-79:

  T1  upload_throughput          : 128 kB write, asserts status=0, prints kB/s
  T2  download_throughput        : 128 kB read,  asserts status=0, prints kB/s
  T3  large_roundtrip            : uploaded and downloaded bytes are bit-perfect (MD5)
  T4  concurrency_guard          : CE_DISPLAY_ACTIVE while display is active;
                                   same upload succeeds immediately after stop
  T5  trial_params_mode2_from_sd : firmware accepts trial-params mode=2 for an SD
                                   pattern (status=0); visual confirmation of panels
                                   is still manual — skipped if --pat not supplied

Run:
    pixi run test-serial              # T1-T4 (T5 skipped without --pat)
    pixi run test-serial -- -s        # same + throughput numbers in stdout
    pixi run test-serial -- --pat /path/to/arena.pat   # also runs T5
    pixi run test-tcp    -- --ip 10.0.0.x --pat /path/to/arena.pat
"""

import hashlib
import struct
import time

import pytest

from .commands import (
    ALL_OFF_CMD,
    ALL_ON_CMD,
    DELETE_ALL_PATTERNS_CMD,
    SET_PATTERN_FILE_CMD,
    STOP_DISPLAY_CMD,
    TRIAL_PARAMS_CMD,
)

# ── error codes (ControllerError enum, src/constants.h) ──────────────────────
CE_DISPLAY_ACTIVE = 10

# ── 128 kB deterministic payload ─────────────────────────────────────────────
_PAYLOAD_KB  = 128
_PAYLOAD     = bytes(range(256)) * (_PAYLOAD_KB * 4)   # 131 072 bytes
_PAYLOAD_MD5 = hashlib.md5(_PAYLOAD).hexdigest()


# ── helpers ───────────────────────────────────────────────────────────────────

def _enc_trial_params(mode: int, pattern_id: int,
                      frame_rate: int = 10, gain: int = 0, init_pos: int = 0) -> bytes:
    """Build the 11-byte trial-params payload for a [0x0C, 0x08, …] frame."""
    return (bytes([mode])
            + struct.pack("<H", pattern_id)
            + struct.pack("<H", frame_rate)
            + bytes([gain & 0xFF])
            + struct.pack("<H", init_pos)
            + b"\x00\x00\x00")


# ── fixtures ──────────────────────────────────────────────────────────────────

@pytest.fixture(autouse=True)
def clean_sd(transport):
    """Clear all patterns (and pattern.temp) before each test."""
    transport.command(ALL_OFF_CMD)
    transport.command(DELETE_ALL_PATTERNS_CMD, timeout=15.0)
    yield


@pytest.fixture
def uploaded_idx(transport):
    """Upload _PAYLOAD to pattern.temp, promote to 'lab79.pat', yield 1-based index."""
    st, _, _, _ = transport.upload_file(0, _PAYLOAD, timeout=60.0)
    assert st == 0, "setup: upload_file to temp failed"
    st2, _, payload, _ = transport.rename_file(0, "lab79.pat")
    assert st2 == 0, "setup: rename_file failed"
    yield struct.unpack("<H", payload)[0]


# ── T1: upload throughput ─────────────────────────────────────────────────────

def test_upload_throughput(transport):
    """Upload 128 kB to pattern.temp; assert status=0 and report write kB/s (use -s)."""
    t0 = time.monotonic()
    st, _, _, _ = transport.upload_file(0, _PAYLOAD, timeout=60.0)
    elapsed = time.monotonic() - t0
    assert st == 0
    print(f"\n  write {_PAYLOAD_KB} kB in {elapsed * 1000:.0f} ms"
          f" → {_PAYLOAD_KB / elapsed:.1f} kB/s")


# ── T2: download throughput ───────────────────────────────────────────────────

def test_download_throughput(transport, uploaded_idx):
    """Download 128 kB from SD; assert status=0 and report read kB/s (use -s)."""
    t0 = time.monotonic()
    st, _, data, _ = transport.download_file(uploaded_idx, timeout=60.0)
    elapsed = time.monotonic() - t0
    assert st == 0
    print(f"\n  read {len(data) // 1024} kB in {elapsed * 1000:.0f} ms"
          f" → {len(data) / 1024 / elapsed:.1f} kB/s")


# ── T3: byte-for-byte round-trip ──────────────────────────────────────────────

def test_large_roundtrip(transport, uploaded_idx):
    """128 kB upload → download must be bit-perfect (MD5 match)."""
    _, _, data, _ = transport.download_file(uploaded_idx, timeout=60.0)
    assert len(data) == len(_PAYLOAD), (
        f"size mismatch: got {len(data)}, expected {len(_PAYLOAD)}"
    )
    assert hashlib.md5(data).hexdigest() == _PAYLOAD_MD5, "MD5 mismatch — bytes differ"


# ── T4: concurrency guard ─────────────────────────────────────────────────────

def test_upload_blocked_while_display_active(transport):
    """SET_PATTERN_FILE must be rejected (CE_DISPLAY_ACTIVE=10) in any non-ALL_OFF state."""
    # Put display into ALL_ON — no SD required.
    st_on, _, _, _ = transport.command(ALL_ON_CMD)
    assert st_on == 0

    # Upload attempt while active — must fail.
    st, echo, _, _ = transport.upload_file(0, bytes(64), timeout=10.0)
    assert st == CE_DISPLAY_ACTIVE, (
        f"Expected CE_DISPLAY_ACTIVE ({CE_DISPLAY_ACTIVE}), got status={st}"
    )
    assert echo == SET_PATTERN_FILE_CMD

    # Stop display; upload must now succeed.
    transport.command(STOP_DISPLAY_CMD)
    st2, echo2, _, _ = transport.upload_file(0, bytes(64), timeout=10.0)
    assert st2 == 0, f"upload failed after stop: status={st2}"
    assert echo2 == SET_PATTERN_FILE_CMD


# ── T5: SD playback — command level (visual check still manual) ───────────────

def test_trial_params_mode2_from_sd(transport, pat_data):
    """Upload --pat, issue trial-params mode=2, assert firmware accepts it (status=0).

    Whether the panels show the correct pattern is a manual visual check.
    Automatically skipped when --pat is not supplied.
    """
    if pat_data is None:
        pytest.skip("--pat not provided; pass a .pat valid for this arena to run T5")

    st, _, _, _ = transport.upload_file(0, pat_data, timeout=120.0)
    assert st == 0, "upload of --pat file failed"

    st2, _, rp, _ = transport.rename_file(0, "lab79_playback.pat")
    assert st2 == 0, "rename of --pat file failed"
    idx = struct.unpack("<H", rp)[0]

    params = _enc_trial_params(mode=2, pattern_id=idx, frame_rate=10)
    st_tp, echo, _, diag = transport.command(TRIAL_PARAMS_CMD, params, timeout=10.0)

    assert st_tp == 0, f"trial-params rejected: status={st_tp}, diag={diag}"
    assert echo == TRIAL_PARAMS_CMD

    # Hold the display for 5 s so the operator can observe the panels, then stop.
    print("\n  *** observe panels for 5 s — pattern should be visible ***")
    time.sleep(5)
    transport.command(STOP_DISPLAY_CMD)
