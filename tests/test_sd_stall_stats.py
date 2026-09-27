"""Offline tests for scripts/sd_stall_test.py's incremental statistics (no hardware).

Codex round-4 findings (2026-09-13): exact aggregates must not depend on the bounded stall detail;
legacy kind-13 checkpoints (code bit 7, emitted by the never-flashed f6c11d2 build) must still decode
as checkpoints; the controller clock must tolerate a pre-wrap record appended after a post-wrap one.
"""
from __future__ import annotations

import importlib.util
import pathlib
import sys
from types import SimpleNamespace

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tests import telemetry_codec as tc  # noqa: E402
from tests.test_telemetry_codec import _block, _state  # noqa: E402

_spec = importlib.util.spec_from_file_location("sd_stall_test", ROOT / "scripts" / "sd_stall_test.py")
sst = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sst)


def _slow(seq, ms, t_us, seg_boot=False):
    return SimpleNamespace(type=tc.REC_STATE, seq=seq, t_us=t_us,
                           fields={"kind": tc.ST_SD_SLOW, "code": 1, "arg": int(ms * 10), "phase": "seek"})


def _st(seq, kind, code, arg, t_us=1000, **extra):
    f = {"kind": kind, "code": code, "arg": arg}
    f.update(extra)
    return SimpleNamespace(type=tc.REC_STATE, seq=seq, t_us=t_us, fields=f)


def test_worst_and_clusters_are_exact_beyond_the_detail_cap():
    s = sst.Stats(gap_ms=10.0)
    s.note_block(0)
    t = 0
    for i in range(sst.STALL_KEEP + 1):          # one stall per cluster, 6 s apart
        t += 6_000_000
        s.note_block(t)
        s.feed(_slow(i, 20.0, t))
    t += 6_000_000
    s.note_block(t)
    s.feed(_slow(99_999, 900.0, t))              # the worst stall arrives AFTER the cap
    out = s.summary()
    assert out["stalls_over_gap"] == sst.STALL_KEEP + 2
    assert out["clusters"] == sst.STALL_KEEP + 2
    assert out["worst_ms"] == 900.0
    assert out["stall_detail_truncated"] is True


def test_clusters_group_stalls_within_the_gap_and_split_on_boot():
    s = sst.Stats(gap_ms=10.0)
    s.note_block(0)
    for i, ms in enumerate((23, 33, 41)):        # one cluster: 0.1 s apart
        s.feed(_slow(i, ms, 1_000_000 + i * 100_000))
    s.feed(_slow(10, 67, 30_000_000))            # 29 s later: second cluster
    s.feed(_st(11, tc.ST_BOOT, 0, 0, t_us=30_500_000))
    s.feed(_slow(12, 89, 30_600_000))            # after a boot: never merged, spacing not computed
    cl = s.clusters()
    assert [c["n"] for c in cl] == [3, 1, 1]
    assert cl[0]["ms"] == [23.0, 33.0, 41.0]
    out = s.summary()
    assert out["clusters"] == 3 and out["spacing_s_median"] == 29.0


def test_legacy_kind13_checkpoint_still_decodes_as_checkpoint():
    (legacy,) = _block(_state(17, tc.ST_SD_READS, 0x80, 30_000))
    assert legacy.fields["reads"] == 30_000 and legacy.fields["checkpoint"] is True
    (final,) = _block(_state(18, tc.ST_SD_READS, 1, 30_000))
    assert final.fields["reads"] == 60_000 and final.fields["checkpoint"] is False
    s = sst.Stats(gap_ms=10.0)
    s.note_block(0)
    s.feed(_st(1, tc.ST_SD_OPEN, 0, 36))
    s.feed(legacy)
    assert s.reads_fw == 30_000 and s.summary()["reads_fw_is_lower_bound"] is True
    s.feed(_st(3, tc.ST_SD_READS, 0, 35_000, reads=35_000, checkpoint=False))
    s.feed(_st(4, tc.ST_SD_OPEN, 0, 5))
    s.feed(_st(5, tc.ST_SD_READS_CKPT, 0, 100, reads=100, checkpoint=True))
    assert s.reads_fw == 35_100                  # finals summed + the current open's checkpoint


def test_controller_clock_tolerates_out_of_order_records_across_a_wrap():
    s = sst.Stats(gap_ms=10.0)
    s.note_block(0xFFFF_F000)                    # just before the u32 micros wrap
    t_pre = s.ctl_time_s(0xFFFF_F800)
    s.note_block(0x0000_1000)                    # header wrapped
    t_post = s.ctl_time_s(0x0000_0800)           # a post-wrap STATE
    t_late = s.ctl_time_s(0xFFFF_FC00)           # a pre-wrap CMD appended after it
    assert abs((t_post - t_pre) - 0x1000 / 1e6) < 1e-6
    assert t_pre < t_late < t_post               # not 4295 s in the future


def test_arm_consistency_is_tracked_from_sd_layout():
    s = sst.Stats(gap_ms=10.0)
    s.note_block(0)
    s.feed(_st(1, tc.ST_SD_LAYOUT, 0x05, 8, contiguous=False, exfat=False, sectors_per_cluster=8,
               legacy_seek=True, no_same_index_skip=False))
    assert s.arm_seen == {(True, False)}
    assert s.arm_matches((True, False)) and not s.arm_matches((False, False))
    s.feed(_st(2, tc.ST_SD_LAYOUT, 0x01, 8, contiguous=True, exfat=False, sectors_per_cluster=8,
               legacy_seek=False, no_same_index_skip=False))
    assert not s.arm_matches((True, False))       # the run changed arm mid-way
