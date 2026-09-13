#!/usr/bin/env python3
"""sd_stall_test.py — SD-card stall characterisation / card-comparison driver (no browser).

Reproduces the Mode-3 closed-loop access pattern on the wire — TRIAL_PARAMS (0x08, mode 3) then
SET_FRAME_POSITION (0x70) at a fixed rate with a FicTrac-like random-walk index (plus periodic
jumps) — while draining the controller's telemetry ring (0xA9, ack cursor) into a behavior_v2
NDJSON log with the SAME row shapes the Arena Studio + bridge write:

    {"type":"frame_schema","level":"behavior_v2",...,"t0":<epoch ms>}
    {"type":"log","event":"run_metadata","tool":"sd_stall_test","firmware":"…","sd_card":{…},
     "controller_settings":{…},"args":{…}}
    {"event":"stream_schema","streams":{"cc":…,"cf":…,"cs":…}}
    ["a",  t_off_ms, dt_ms, "<request hex>", status|null, rx_off_ms]        host command echo
    ["cc", rx_ms, t_us, seq, cmd, status, "<req hex>"]                       controller CMD record
    ["cf", rx_ms, t_us, seq, idx, pattern, sd_load_us, spi_us, req_age_us, superseded, flags]
    ["cs", rx_ms, t_us, seq, kind, code, arg]
    {"type":"log","event":"sd_stall_summary", …}                             inline summary at the end

so one analyzer serves both: webDisplayTools/scripts/telemetry-report.py. The inline summary
answers the card question on its own: stall clusters, their spacing in reads, durations and
phases, SD read cost by index step, request→presentation age, and the card identity
(GET_SD_INFO 0xCD) they belong to.

Measurement discipline (Codex reviews 2026-09-13, rounds 2+3). NOTE: the workload is response-paced (single-flight):
it slows down when the controller slows down, so it compares SD behaviour under a controlled command stream — it is not
a replay of externally paced FicTrac traffic (the Studio + bridge path is).
  * the workload is RESPONSE-PACED (single-flight request/response): a missed send slot is skipped,
    never caught up with a burst; missed slots, achieved rate and long send intervals are reported;
  * only records with seq > the boundary taken after the pre-run backlog drain enter the summary
    (the backlog stays in the log, tagged by seq);
  * capture validity is tracked (drain errors, ring disabled/heap collision, seq gaps, dropped) and
    an invalid capture is reported and exits 4 — a run with no telemetry is never "0 stalls";
  * stall clusters are grouped on the CONTROLLER clock (unwrapped t_us), segmented at boot records;
  * statistics are incremental (bounded reservoirs) — nothing grows with run length;
  * reads are counted from the controller (STATE sd_reads) with index changes as the running proxy.

Why: the card stalls 23–89 ms every ~24.5k reads of a large pattern (card-internal read-count
maintenance, 2026-09-13). Acceptable display freeze: 5 ms target / 10 ms worst case. A 20-minute
run at 200 Hz (≈ 5 baseline cycles) SCREENS a card; qualification needs hours and a second specimen.

Usage:
    python3 scripts/sd_stall_test.py --port /dev/cu.usbmodem123456 --pattern 36 --label sandisk-32g
    python3 scripts/sd_stall_test.py --port … --pattern 36 --window 50 --minutes 20   # working-set test
    python3 scripts/sd_stall_test.py --port … --pattern 36 --reads 150000              # stop after ~N reads

Requires pyserial (system python3 has it; the pixi env may not). Exit codes: 0 normal end,
1 startup failure (no controller / pattern / telemetry), 2 bad arguments, 3 controller stopped
answering (fault), 4 capture invalid (telemetry incomplete — the log is complete but the summary
must not be used for a card comparison).
"""

from __future__ import annotations

import argparse
import datetime as _dt
import json
import os
import random
import statistics
import struct
import sys
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)

from tests.commands import (  # noqa: E402
    GET_CONTROLLER_INFO_CMD,
    GET_FIRMWARE_VERSION_CMD,
    GET_PANEL_DISPLAY_MODE_CMD,
    GET_PATTERN_INFO_CMD,
    GET_REFRESH_RATE_CMD,
    GET_SD_INFO_CMD,
    GET_SPI_CLOCK_CMD,
    GET_TELEMETRY_BLOCK_CMD,
    SET_FRAME_POSITION_CMD,
    SET_SD_DIAG_CMD,
    SET_TELEMETRY_CMD,
    STOP_DISPLAY_CMD,
    TRIAL_PARAMS_CMD,
)
from tests import telemetry_codec as tc  # noqa: E402
from tests.transport import SerialTransport, build_frame, parse_response  # noqa: E402

CAP_HEALTH = 0x80
FW_FLAG_TELEMETRY = 0x04
FW_FLAG_SD_FASTPATH = 0x20
FW_FLAG_SD_DIAG = 0x40             # SET_SD_DIAG 0xCE present
STALL_KEEP = 2000                   # bounded per-stall detail kept in memory (every stall is in the log)
FW_SD_SLOW_THRESHOLD_MS = 10.0     # ring v2 firmware emits sd_slow above this (20 ms on v1)
CLUSTER_GAP_S = 5.0
RESERVOIR = 20000                  # bounded per-class latency samples
SD_MANUFACTURERS = {0x01: "Panasonic", 0x02: "Toshiba/Kioxia", 0x03: "SanDisk", 0x09: "ATP", 0x13: "KingMax",
                    0x1B: "Samsung", 0x1D: "ADATA", 0x27: "Phison", 0x28: "Lexar", 0x31: "Silicon Power",
                    0x41: "Kingston", 0x5D: "Swissbit", 0x6F: "STMicro", 0x74: "Transcend", 0x76: "Patriot",
                    0x82: "Sony/Gobe", 0x9C: "Angelbird/Hoodman"}
SD_CARD_TYPES = ["none", "SD1", "SD2", "SDHC/SDXC"]


def now_ms() -> float:
    return time.time() * 1000.0


def hexstr(b: bytes) -> str:
    return " ".join(f"{x:02x}" for x in b)


def pct(values, p):
    if not values:
        return None
    s = sorted(values)
    return s[min(len(s) - 1, int(p * len(s)))]


def decode_firmware_version(payload: bytes) -> dict:
    ver, rows, cols, flags, sha, date, branch = struct.unpack("<BBBB8s10s24s", bytes(payload[:46]))
    sha = sha.decode("ascii", "replace").strip()
    label = f"{sha}{'*' if flags & 1 else ''} {rows}x{cols} {date.decode('ascii', 'replace').strip()} {branch.decode('ascii', 'replace').strip()}"
    if flags & 0x10:
        label += " freerun"
    if flags & FW_FLAG_SD_FASTPATH:
        label += " sdfast"
    return {"label": label, "flags": flags, "sha": sha}


def decode_sd_info(payload: bytes) -> dict:
    p = bytes(payload)
    ver, flags, card_type, fat_type, sectors, bpc = struct.unpack_from("<BBBBII", p, 0)
    cid = p[12:28]
    d = {
        "ver": ver, "mounted": bool(flags & 1), "cid_valid": bool(flags & 2), "csd_valid": bool(flags & 4),
        "card_type": SD_CARD_TYPES[card_type] if card_type < 4 else f"type_{card_type}",
        "fat": "exFAT" if fat_type == 64 else (f"FAT{fat_type}" if fat_type else "unknown"),
        "sectors": sectors, "capacity_gb": round(sectors * 512 / 1e7) / 100, "bytes_per_cluster": bpc,
        "cid_hex": cid.hex(), "mid": cid[0],
        "manufacturer": SD_MANUFACTURERS.get(cid[0], f"mid_0x{cid[0]:02x}"),
        "oid": cid[1:3].decode("ascii", "replace").strip(), "pnm": cid[3:8].decode("ascii", "replace").strip(),
        "prv": f"{cid[8] >> 4}.{cid[8] & 15}", "psn_hex": cid[9:13].hex(),
        "mdt": f"{2000 + (((cid[13] & 15) << 4) | (cid[14] >> 4))}-{cid[14] & 15:02d}",
        "sd_status_maint": p[28],
        "sd_diag": p[29], "legacy_seek_requested": bool(p[29] & 1), "no_same_index_skip": bool(p[29] & 2),
        "legacy_seek_applied": bool(p[29] & 4),
    }
    d["label"] = ("no card" if not d["mounted"] else
                  f"{d['manufacturer']} {d['pnm']} {d['prv']} sn {d['psn_hex']} ({d['mdt']}) {d['capacity_gb']} GB "
                  f"{d['card_type']} {d['fat']} {bpc // 1024 if bpc >= 1024 else bpc}{' KiB' if bpc >= 1024 else ' B'} clusters")
    return d


class Walker:
    """FicTrac-like heading random walk in frames, wrapped, with periodic jumps; optionally
    confined to the first `window` frames (working-set test on the same file)."""

    def __init__(self, frames: int, sigma: float, jump_every: int, jump_frames: float, seed: int, window: int):
        self.n = max(1, window if window and window < frames else frames)
        self.rng = random.Random(seed)
        self.sigma = sigma
        self.jump_every = max(0, jump_every)
        self.jump_frames = jump_frames
        self.pos = 0.0
        self.i = 0

    def next(self) -> int:
        self.i += 1
        self.pos += self.rng.gauss(0.0, self.sigma)
        if self.jump_every and self.i % self.jump_every == 0:
            self.pos += self.rng.choice((-1, 1)) * self.jump_frames
        self.pos %= self.n
        return int(round(self.pos)) % self.n


class Log:
    def __init__(self, path: str):
        self.path = path
        self.f = open(path, "w", buffering=1)
        self.t0 = now_ms()
        self.write({"type": "frame_schema", "level": "behavior_v2",
                    "cols": ["ms", "fc", "idx", "ft", "x", "y", "hd"],
                    "arena_cols": ["t_off", "dt", "hex", "status", "rx_off"], "t0": self.t0})

    def write(self, obj) -> None:
        self.f.write(json.dumps(obj, separators=(",", ":")) + "\n")

    def event(self, name: str, **kw) -> None:
        d = {"type": "log", "event": name, "ms": now_ms()}
        d.update(kw)
        self.write(d)

    def close(self) -> None:
        self.f.close()


class Reservoir:
    """Bounded uniform sample (Algorithm R) with exact count/max."""

    def __init__(self, cap: int = RESERVOIR):
        self.cap = cap
        self.buf: list = []
        self.n = 0
        self.max = 0
        self.rng = random.Random(7)

    def add(self, v) -> None:
        self.n += 1
        if v > self.max:
            self.max = v
        if len(self.buf) < self.cap:
            self.buf.append(v)
        else:
            j = self.rng.randrange(self.n)
            if j < self.cap:
                self.buf[j] = v

    def summary(self) -> dict:
        if not self.n:
            return {"n": 0}
        return {"n": self.n, "p50": statistics.median(self.buf), "p90": pct(self.buf, 0.9),
                "p99": pct(self.buf, 0.99), "max": self.max}


class Stats:
    """Incremental analysis of the CURRENT run's ring records (seq > boundary)."""

    def __init__(self, gap_ms: float):
        self.gap_ms = gap_ms
        self.cmds70 = 0
        self.index_changes = 0
        self.last_idx = None
        self.frames = 0
        self.reads_fw_final = 0         # sum of STATE sd_reads (kind 13) finals since the boundary
        self.reads_fw_ckpt = 0          # last STATE sd_reads_ckpt (kind 14) of the current open (lower bound)
        self.opens = 0
        self.slow_reads = 0             # every sd_slow record (firmware threshold)
        self.stalls: list = []          # bounded detail (STALL_KEEP) — everything the summary reports is exact
        self.stall_count = 0
        self.worst_ms = 0.0             # exact running max (not from the bounded detail)
        self._clusters: list = []       # exact online clustering: one dict per cluster (t_s, seg, cmds, idx, n, ms[≤8], worst)
        self._last_stall = None         # (t_s, seg) of the previous over-gap stall
        self.age_over_gap = 0           # exact counter (the reservoir is for percentiles only)
        self.arm_seen: set = set()      # (legacy_seek, no_skip) tuples from sd_layout records
        self.step = {k: Reservoir() for k in ("+1", "-1", "small", "jump")}
        self.ages = Reservoir()
        self.superseded_frames = 0
        self.superseded_total = 0
        self.contiguous_frames = 0
        self.v2_frames = 0
        self.prev_idx = None
        self.layout = None
        # controller clock: unwrap u32 micros against the block header's t_now_us (monotone per drain),
        # tolerating records that carry an earlier timestamp than a later-appended one (CMD after its
        # handler's STATEs); segment at boot records
        self.epoch = 0                  # multiples of 2^32 accumulated from t_now_us wraps
        self.t_now_last = None
        self.segment = 0
        self.boots = 0
        self.ring_disabled = False

    def note_block(self, t_now_us: int) -> None:
        if self.t_now_last is not None and t_now_us < self.t_now_last and (self.t_now_last - t_now_us) > 0x80000000:
            self.epoch += 1 << 32
        self.t_now_last = t_now_us

    def ctl_time_s(self, t_us: int) -> float:
        anchor = self.t_now_last if self.t_now_last is not None else t_us
        t = self.epoch + t_us
        if t_us > anchor + 0x80000000:      # record from before the wrap the anchor already passed
            t -= 1 << 32
        return t / 1e6

    def feed(self, r) -> None:
        f = r.fields
        t_s = self.ctl_time_s(r.t_us)
        if r.type == tc.REC_CMD:
            if f["cmd"] == SET_FRAME_POSITION_CMD and f["status"] == 0:
                self.cmds70 += 1
                p = f["params"]
                if len(p) >= 4:
                    idx = int(p[0:2], 16) | (int(p[2:4], 16) << 8)
                    if idx != self.last_idx:
                        self.index_changes += 1
                        self.last_idx = idx
        elif r.type == tc.REC_FRAME:
            self.frames += 1
            if self.last_idx is None:
                self.last_idx = f["idx"]
            if self.prev_idx is not None:
                d = f["idx"] - self.prev_idx
                cls = "+1" if d == 1 else "-1" if d == -1 else "small" if abs(d) < 10 else "jump"
                self.step[cls].add(f["sd_load_us"])
            self.prev_idx = f["idx"]
            if "req_age_us" in f:
                self.v2_frames += 1
                self.ages.add(f["req_age_us"])
                if f["req_age_us"] > self.gap_ms * 1000:
                    self.age_over_gap += 1
                if f["superseded"]:
                    self.superseded_frames += 1
                    self.superseded_total += f["superseded"]
                if f["contiguous"]:
                    self.contiguous_frames += 1
        elif r.type == tc.REC_STATE:
            k = f["kind"]
            if k == tc.ST_SD_SLOW:
                ms = f["arg"] / 10.0
                self.slow_reads += 1
                if ms > self.gap_ms:
                    self.stall_count += 1
                    if ms > self.worst_ms:
                        self.worst_ms = ms
                    last = self._last_stall
                    if last is not None and last[1] == self.segment and (t_s - last[0]) < CLUSTER_GAP_S:
                        c = self._clusters[-1]
                        c["n"] += 1
                        if len(c["ms"]) < 8:
                            c["ms"].append(ms)
                            c["phases"].append(f.get("phase", "unknown"))
                        c["worst"] = max(c["worst"], ms)
                    else:
                        self._clusters.append({"t_s": t_s, "seg": self.segment, "cmds": self.cmds70, "idx": self.index_changes,
                                               "n": 1, "ms": [ms], "phases": [f.get("phase", "unknown")], "worst": ms})
                    self._last_stall = (t_s, self.segment)
                    if len(self.stalls) < STALL_KEEP:
                        self.stalls.append({"t_s": t_s, "seg": self.segment, "ms": ms, "phase": f.get("phase", "unknown"),
                                            "err": bool(f.get("read_error")), "cmds": self.cmds70, "idx": self.index_changes})
            elif k == tc.ST_SD_READS and f.get("checkpoint"):   # legacy checkpoint encoding (kind 13, code bit 7)
                self.reads_fw_ckpt = max(self.reads_fw_ckpt, f.get("reads", f["arg"]))
            elif k == tc.ST_SD_READS:                       # final count for the open being left
                self.reads_fw_final += f.get("reads", f["arg"])
                self.reads_fw_ckpt = 0
            elif k == tc.ST_SD_READS_CKPT:                  # cumulative so far for the current open
                self.reads_fw_ckpt = max(self.reads_fw_ckpt, f.get("reads", f["arg"]))
            elif k == tc.ST_SD_OPEN:
                if f["code"] == 0:
                    self.opens += 1
                    self.reads_fw_ckpt = 0
            elif k == tc.ST_SD_LAYOUT:
                self.layout = {"contiguous": f["contiguous"], "exfat": f["exfat"], "sectors_per_cluster": f["sectors_per_cluster"],
                               "legacy_seek": f.get("legacy_seek", False), "no_same_index_skip": f.get("no_same_index_skip", False)}
                self.arm_seen.add((self.layout["legacy_seek"], self.layout["no_same_index_skip"]))
            elif k == tc.ST_BOOT:
                self.boots += 1
                self.segment += 1       # clusters / spacings never bridge a boot (the block clock re-anchors itself)
            elif k == tc.ST_RING_OVERRUN and f["code"] == tc.OVERRUN_CODE_HEAP_COLLISION:
                self.ring_disabled = True

    def clusters(self) -> list:
        """Exact cluster list (one dict per cluster; stall detail inside a cluster bounded to 8 durations)."""
        return self._clusters

    def arm_matches(self, expected) -> bool:
        """True when every sd_layout record of the run reports the expected (legacy_seek, no_skip) arm."""
        if not self.arm_seen:
            return True     # no trial opened yet — capture() requires opens >= 1 separately
        return self.arm_seen == {tuple(expected)}

    @property
    def reads_fw(self):
        """Controller-counted reads since the boundary: finals + the current open's last checkpoint (lower bound)."""
        v = self.reads_fw_final + self.reads_fw_ckpt
        return v if v else None

    def summary(self) -> dict:
        cl = self.clusters()
        spacing_cmds = [c["cmds"] - p["cmds"] for p, c in zip(cl, cl[1:]) if c["seg"] == p["seg"]]
        spacing_idx = [c["idx"] - p["idx"] for p, c in zip(cl, cl[1:]) if c["seg"] == p["seg"]]
        spacing_s = [round(c["t_s"] - p["t_s"], 1) for p, c in zip(cl, cl[1:]) if c["seg"] == p["seg"]]
        return {
            "accepted_0x70": self.cmds70,
            "index_changes": self.index_changes,
            "reads_fw": self.reads_fw,
            "reads_fw_is_lower_bound": self.reads_fw_ckpt > 0,
            "opens": self.opens,
            "arms_seen": sorted(["legacy-seek" if a[0] else "fast-seek", "no-skip" if a[1] else "skip"] for a in self.arm_seen),
            "frames": self.frames,
            "reads_per_cmd": round((self.reads_fw or self.index_changes) / self.cmds70, 3) if self.cmds70 else None,
            "layout": self.layout,
            "slow_reads_fw_threshold": self.slow_reads,
            "stalls_over_gap": self.stall_count,
            "stall_detail_truncated": self.stall_count > len(self.stalls),
            "clusters": len(cl),
            "clusters_per_1e5_cmds": round(len(cl) * 1e5 / self.cmds70, 2) if self.cmds70 else None,
            "cluster_ms": [[round(m, 1) for m in c["ms"]] + (["…"] if c["n"] > len(c["ms"]) else []) for c in cl[:200]],
            "cluster_sizes": [c["n"] for c in cl[:200]],
            "cluster_phases": [c["phases"] for c in cl[:200]],
            "cluster_t_s": [round(c["t_s"], 1) for c in cl[:200]],
            "spacing_cmds": spacing_cmds[:200],
            "spacing_cmds_median": statistics.median(spacing_cmds) if spacing_cmds else None,
            "spacing_index_changes_median": statistics.median(spacing_idx) if spacing_idx else None,
            "spacing_s_median": statistics.median(spacing_s) if spacing_s else None,
            "worst_ms": self.worst_ms,
            "step_cost_us": {k: v.summary() for k, v in self.step.items() if v.n},
            "req_age_us": dict(self.ages.summary(), over_gap=self.age_over_gap) if self.v2_frames else None,
            "superseded_share": round(self.superseded_frames / self.v2_frames, 3) if self.v2_frames else None,
            "contiguous_share": round(self.contiguous_frames / self.v2_frames, 3) if self.v2_frames else None,
            "controller_boots_during_run": self.boots,
        }


class Drainer:
    """Ack-cursor ring drain: records are freed only by the NEXT request's ack (a lost reply is
    re-served). Capture validity is tracked; records with seq <= boundary are logged but not analysed."""

    def __init__(self, t: SerialTransport, log: Log, timeout: float, stats: Stats):
        self.t = t
        self.log = log
        self.timeout = timeout
        self.stats = stats
        self.ack = tc.NO_ACK
        self.tracker = tc.SeqTracker()
        self.rows = {"cc": 0, "cf": 0, "cs": 0}
        self.errors = 0
        self.blocks = 0
        self.boundary = None            # seq of the last backlog record; analysis starts after it
        self.last_header = None
        self.dropped_delta = 0
        self.disabled_seen = False
        self.events_off_seen = False
        self.incarnations = 0
        self.post_boundary_records = 0

    def drain(self, max_chunks: int = 10) -> int:
        n = 0
        for _ in range(max_chunks):
            try:
                st, echo, payload, _ = self.t.command(GET_TELEMETRY_BLOCK_CMD,
                                                      tc.build_get_block(self.ack, tc.RECORD_BYTES_MAX),
                                                      timeout=self.timeout)
            except Exception:
                self.errors += 1
                return n
            if st != 0 or echo != GET_TELEMETRY_BLOCK_CMD:
                self.errors += 1
                return n
            rx = now_ms()
            try:
                hdr, recs = tc.parse_block(bytes(payload))
            except ValueError:
                self.errors += 1
                return n
            self.blocks += 1
            self.stats.note_block(hdr.t_now_us)
            if self.last_header is not None and hdr.boot_count != self.last_header.boot_count:
                self.incarnations += 1               # ring re-initialised / controller rebooted mid-run
            if self.last_header is not None and hdr.dropped > self.last_header.dropped:
                self.dropped_delta += hdr.dropped - self.last_header.dropped
            self.last_header = hdr
            if hdr.flags & tc.FLAG_HEAP_COLLISION:
                self.disabled_seen = True
            if not (hdr.flags & tc.FLAG_EVENTS_ENABLED):
                self.events_off_seen = True
            self.tracker.feed(recs)
            for r in recs:
                f = r.fields
                if r.type == tc.REC_CMD:
                    self.log.write(["cc", rx, r.t_us, r.seq, f["cmd"], f["status"], f["params"]])
                    self.rows["cc"] += 1
                elif r.type == tc.REC_FRAME:
                    row = ["cf", rx, r.t_us, r.seq, f["idx"], f["pattern"], f["sd_load_us"], f["spi_us"]]
                    if "req_age_us" in f:
                        row += [f["req_age_us"], f["superseded"], f["flags"]]
                    self.log.write(row)
                    self.rows["cf"] += 1
                else:
                    self.log.write(["cs", rx, r.t_us, r.seq, f["kind"], f["code"], f["arg"]])
                    self.rows["cs"] += 1
                if self.boundary is not None and ((r.seq - self.boundary) & 0xFFFFFFFF) < 0x80000000 and r.seq != self.boundary:
                    self.stats.feed(r)
                    self.post_boundary_records += 1
                n += 1
            if recs:
                self.ack = recs[-1].seq
            if not hdr.more:
                break
        return n

    def set_boundary(self) -> None:
        self.boundary = self.ack if self.ack != tc.NO_ACK else 0

    def capture(self, accepted: int = 0, expect_final_reads: bool = False, expected_arm=None) -> dict:
        """Measurement completeness, not just transport health: records after the boundary, a trial that
        opened and closed (sd_open + final sd_reads), controller command count consistent with the host's
        accepted count (final capture only — mid-run the ring drain lags by up to one poll), the whole run
        under ONE arm equal to the requested one, no ring loss, no reboot / re-initialisation, no
        events-off / heap-disabled block."""
        gaps_after = [g for g in self.tracker.gaps if self.boundary is None or ((g[1] - self.boundary) & 0xFFFFFFFF) < 0x80000000]
        st = self.stats
        cmds_ok = accepted == 0 or abs(st.cmds70 - accepted) <= 2       # ≤ the in-flight command
        complete = (st.opens >= 1 and st.frames > 0 and (st.reads_fw_final > 0 or not expect_final_reads))
        arm_ok = len(st.arm_seen) <= 1 and (expected_arm is None or st.arm_matches(expected_arm))
        valid = (self.errors == 0 and not self.disabled_seen and not self.events_off_seen
                 and self.dropped_delta == 0 and not gaps_after and self.blocks > 0
                 and self.post_boundary_records > 0 and self.incarnations == 0 and cmds_ok and complete and arm_ok)
        return {"valid": valid, "drain_errors": self.errors, "blocks": self.blocks, "rows": dict(self.rows),
                "records_after_boundary": self.post_boundary_records, "ring_disabled": self.disabled_seen,
                "events_off": self.events_off_seen, "dropped_during_run": self.dropped_delta, "seq_gaps": len(gaps_after),
                "incarnations": self.incarnations, "controller_cmds_vs_accepted": [st.cmds70, accepted],
                "trial_opened": st.opens >= 1, "final_reads_seen": st.reads_fw_final > 0, "arm_consistent": arm_ok,
                "cmds_reconciled": accepted > 0,
                "boot_count": self.last_header.boot_count if self.last_header else None}


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", required=True, help="serial device, e.g. /dev/cu.usbmodem123456 or COM7")
    p.add_argument("--pattern", type=int, required=True, help="1-based SD pattern index (the large pattern)")
    p.add_argument("--frames", type=int, default=0, help="override the frame count (default: GET_PATTERN_INFO)")
    p.add_argument("--hz", type=float, default=200.0, help="0x70 rate (default 200)")
    p.add_argument("--minutes", type=float, default=20.0, help="run length (default 20; 0 = until --reads / Ctrl-C)")
    p.add_argument("--reads", type=int, default=0, help="stop after ~N SD reads (index-changing 0x70s; 0 = no limit)")
    p.add_argument("--sigma", type=float, default=1.6, help="random-walk step sigma, frames per sample (default 1.6)")
    p.add_argument("--jump-every", type=int, default=100, help="one jump every N samples (0 = none; default 100)")
    p.add_argument("--jump-deg", type=float, default=90.0, help="jump size in degrees of the pattern's 360° (default 90)")
    p.add_argument("--window", type=int, default=0, help="confine indices to the first N frames (working-set test; 0 = all)")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--gap-ms", type=float, default=10.0, help="stall threshold for the summary (default 10; below the firmware's 10 ms only FRAME ages are covered)")
    p.add_argument("--sd-diag", type=int, default=0, help="A/B arm via SET_SD_DIAG 0xCE: 0 production, 1 legacy FAT-chain seek, 2 no same-index skip, 3 both (needs 0xCB bit 6)")
    p.add_argument("--allow-v1", action="store_true", help="accept firmware without the SD fast path (20 ms sd_slow threshold, no req_age): coverage is reduced")
    p.add_argument("--label", default="", help="free text for the log name / metadata (card, experiment)")
    p.add_argument("--log-dir", default="./soak-logs")
    p.add_argument("--timeout", type=float, default=0.5, help="reply timeout, s (default 0.5)")
    p.add_argument("--drain-every", type=float, default=0.1, help="ring drain period, s (default 0.1)")
    p.add_argument("--progress-every", type=float, default=60.0)
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.hz <= 0 or args.gap_ms <= 0:
        print("--hz and --gap-ms must be positive", file=sys.stderr)
        return 2
    coverage_note = None
    if args.gap_ms < FW_SD_SLOW_THRESHOLD_MS:
        coverage_note = (f"--gap-ms {args.gap_ms} is below the firmware sd_slow threshold ({FW_SD_SLOW_THRESHOLD_MS} ms): "
                         f"reads between the two are only visible through FRAME req_age_us")
        print("WARNING:", coverage_note, file=sys.stderr)
    os.makedirs(args.log_dir, exist_ok=True)
    stamp = _dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    label = "".join(ch if ch.isalnum() or ch in "-_" else "-" for ch in args.label) or "run"
    log = Log(os.path.join(args.log_dir, f"sdstall-{stamp}-{label}.jsonl"))
    print(f"log: {log.path}")

    t = SerialTransport(args.port)
    try:
        t.open()
    except Exception as e:
        print(f"cannot open {args.port}: {e}", file=sys.stderr)
        return 1
    stats = Stats(args.gap_ms)
    drainer = None
    exit_code = 0
    shutdown_verified = False
    sent = accepted = missed_slots = echo_mismatch = long_intervals = resyncs = resync_failures = 0
    prev_diag = None          # SET_SD_DIAG flags found on the controller before this run (restored in finally)
    diag_set = False
    expected_arm = (bool(args.sd_diag & 1), bool(args.sd_diag & 2))

    def resync() -> bool:
        """After a timeout a late reply to the timed-out 0x70 may still arrive; opcode-only matching would hand it
        to the NEXT request. Send ONE probe of a different opcode and read replies until ITS echo comes back."""
        try:
            t._send(build_frame(GET_CONTROLLER_INFO_CMD))
        except Exception:
            return False
        for _ in range(4):
            try:
                _, echo_r, _, _ = parse_response(t._recv_raw(0.3))
            except Exception:
                return False
            if echo_r == GET_CONTROLLER_INFO_CMD:
                return True
        return False
    period = 1.0 / args.hz
    try:
        st, _, _, _ = t.command(STOP_DISPLAY_CMD, timeout=2.0)
        st, _, info, _ = t.command(GET_CONTROLLER_INFO_CMD, timeout=2.0)
        if st != 0 or len(info) < 2 or not (info[1] & CAP_HEALTH):
            print("controller does not advertise health/version (0xC2 cap bit 7)", file=sys.stderr)
            return 1
        st, _, fvp, _ = t.command(GET_FIRMWARE_VERSION_CMD, timeout=2.0)
        if st != 0 or len(fvp) < 46:
            print("GET_FIRMWARE_VERSION failed", file=sys.stderr)
            return 1
        fw = decode_firmware_version(bytes(fvp))
        if not (fw["flags"] & FW_FLAG_TELEMETRY):
            print(f"firmware {fw['label']} has no telemetry ring", file=sys.stderr)
            return 1
        if not (fw["flags"] & FW_FLAG_SD_FASTPATH) and not args.allow_v1:
            print(f"firmware {fw['label']} lacks the SD fast path (0xCB bit 5): sd_slow threshold is 20 ms and FRAME has no "
                  f"req_age — pass --allow-v1 to measure anyway (reduced coverage)", file=sys.stderr)
            return 1
        if args.sd_diag not in (0, 1, 2, 3):
            print("--sd-diag must be 0..3", file=sys.stderr)
            return 2
        if args.sd_diag and not (fw["flags"] & FW_FLAG_SD_DIAG):
            print(f"firmware {fw['label']} has no SET_SD_DIAG (0xCB bit 6): cannot run arm {args.sd_diag}", file=sys.stderr)
            return 1
        sd_card = None
        if fw["flags"] & FW_FLAG_SD_FASTPATH:
            st, _, sdp, _ = t.command(GET_SD_INFO_CMD, timeout=2.0)
            if len(sdp) >= 30:
                sd_card = decode_sd_info(bytes(sdp))
        if fw["flags"] & FW_FLAG_SD_DIAG:
            # The legacy-seek arm only reproduces the FAT-chain walk on FAT16/32: on exFAT the library flags
            # the file contiguous at open, so arms 1/3 would silently equal arms 0/2 (Codex rounds 3/4).
            if (args.sd_diag & 1) and sd_card and sd_card["fat"] == "exFAT":
                print(f"card is exFAT: the legacy-seek arm has no effect there — arm {args.sd_diag} refused", file=sys.stderr)
                return 1
            prev_diag = sd_card["sd_diag"] & 0x03 if sd_card else None
            st, echo, dp, _ = t.command(SET_SD_DIAG_CMD, bytes([args.sd_diag]), timeout=2.0)
            if st != 0 or echo != SET_SD_DIAG_CMD or bytes(dp)[:1] != bytes([args.sd_diag]):
                print(f"SET_SD_DIAG({args.sd_diag}) not applied (status {st})", file=sys.stderr)
                return 1
            diag_set = True
        # runtime settings that change the timing (Codex: identical CLI args ≠ identical controller)
        settings = {}
        for name, cmd, fmt in (("refresh_hz", GET_REFRESH_RATE_CMD, "<H"), ("spi_mhz", GET_SPI_CLOCK_CMD, "<H"),
                               ("panel_disp_mode", GET_PANEL_DISPLAY_MODE_CMD, "<B")):
            try:
                st, _, pl, _ = t.command(cmd, timeout=2.0)
                settings[name] = struct.unpack_from(fmt, bytes(pl))[0] if st == 0 and len(pl) >= struct.calcsize(fmt) else None
            except Exception:
                settings[name] = None
        st, _, pip, _ = t.command(GET_PATTERN_INFO_CMD, struct.pack("<H", args.pattern), timeout=2.0)
        if st != 0 or len(pip) < 2:
            print(f"GET_PATTERN_INFO({args.pattern}) failed status={st}", file=sys.stderr)
            return 1
        frames = args.frames or struct.unpack_from("<H", bytes(pip))[0]
        if frames < 2:
            print("pattern has < 2 frames", file=sys.stderr)
            return 1
        jump_frames = frames * args.jump_deg / 360.0
        walker = Walker(frames, args.sigma, args.jump_every, jump_frames, args.seed, args.window)
        log.event("run_metadata", tool="sd_stall_test", firmware=fw["label"], sd_card=sd_card, pattern=args.pattern,
                  frames=frames, log_format="behavior_v2", telemetry="ring-10hz", controller_settings=settings,
                  sd_diag_arm=args.sd_diag, fw_sd_slow_threshold_ms=FW_SD_SLOW_THRESHOLD_MS if fw["flags"] & FW_FLAG_SD_FASTPATH else 20.0,
                  args={k: v for k, v in vars(args).items() if k != "port"}, gap_threshold_ms=args.gap_ms,
                  coverage_note=coverage_note)
        log.write({"event": "stream_schema", "streams": {
            "cc": {"cols": ["rx", "t_us", "seq", "cmd", "status", "req"]},
            "cf": {"cols": ["rx", "t_us", "seq", "idx", "pattern", "sd_load_us", "spi_us", "req_age_us", "superseded", "flags"]},
            "cs": {"cols": ["rx", "t_us", "seq", "kind", "code", "arg"], "kinds": tc.STATE_KIND_NAMES}}})
        print(f"firmware: {fw['label']}")
        print(f"sd card : {sd_card['label'] if sd_card else '(firmware without GET_SD_INFO)'}")
        print(f"settings: {settings} · arm {args.sd_diag} ({'legacy-seek ' if args.sd_diag & 1 else ''}{'no-skip' if args.sd_diag & 2 else ''}{'production' if not args.sd_diag else ''})")
        print(f"pattern {args.pattern}: {frames} frames, window {walker.n}, {args.hz} Hz, sigma {args.sigma}, "
              f"jump {args.jump_deg}° every {args.jump_every}, {args.minutes} min")

        st, _, _, _ = t.command(SET_TELEMETRY_CMD, tc.build_set_telemetry(tc.SET_FLAG_EVENTS, None), timeout=2.0)
        if st != 0:
            print("SET_TELEMETRY refused", file=sys.stderr)
            return 1
        drainer = Drainer(t, log, args.timeout, stats)
        # Pre-run backlog: kept in the log, excluded from the summary. A full 64 KiB ring of 26 B FRAMEs is
        # > 400 blocks, so drain until the header says nothing is left — a boundary set with records still
        # queued would count the previous run's reads as this one's (whole-stack review, 2026-09-13).
        for _ in range(40):
            drainer.drain(max_chunks=400)
            if drainer.errors or (drainer.last_header is not None and not drainer.last_header.more):
                break
        if drainer.errors or drainer.blocks == 0:
            print("telemetry ring did not answer at start — refusing to run a blind measurement", file=sys.stderr)
            return 1
        if drainer.last_header is not None and drainer.last_header.more:
            print("telemetry backlog did not drain (ring still reports more) — refusing to set a boundary", file=sys.stderr)
            return 1
        drainer.set_boundary()
        log.event("telemetry_backlog_drained", boundary_seq=drainer.boundary, blocks=drainer.blocks)

        tp = (bytes([3]) + struct.pack("<H", args.pattern) + struct.pack("<h", 0) + struct.pack("<H", 0)
              + struct.pack("<h", 0) + struct.pack("<H", 0))
        st, _, _, _ = t.command(TRIAL_PARAMS_CMD, tp, timeout=10.0)
        if st != 0:
            print(f"TRIAL_PARAMS failed status={st}", file=sys.stderr)
            return 1
        applied = None
        if fw["flags"] & FW_FLAG_SD_DIAG:
            _, _, sdp2, _ = t.command(GET_SD_INFO_CMD, timeout=2.0)
            if len(sdp2) >= 30:
                applied = bytes(sdp2)[29]
                if bool(applied & 4) != bool(args.sd_diag & 1):
                    print(f"arm not applied: requested legacy_seek={bool(args.sd_diag & 1)}, applied={bool(applied & 4)}", file=sys.stderr)
                    return 1
        log.event("runner", phase="trial_start", pattern=args.pattern, mode=3, sd_diag_applied=applied)

        t_start = time.perf_counter()
        t_end = t_start + args.minutes * 60.0 if args.minutes > 0 else None
        next_send = t_start
        next_drain = t_start + args.drain_every
        next_progress = t_start + args.progress_every
        last_send = None
        fails = []
        while True:
            nowp = time.perf_counter()
            if t_end and nowp >= t_end:
                break
            if args.reads and max(stats.index_changes, stats.reads_fw or 0) >= args.reads:
                break
            if nowp < next_send:
                time.sleep(min(next_send - nowp, 0.002))
                continue
            # Missed-slot policy: never burst to catch up — skip the expired slots and count them.
            if nowp - next_send >= period:
                skipped = int((nowp - next_send) // period)
                missed_slots += skipped
                next_send += skipped * period
            next_send += period
            if last_send is not None and (nowp - last_send) > 1.5 * period:
                long_intervals += 1
            last_send = nowp
            idx = walker.next()
            req = build_frame(SET_FRAME_POSITION_CMD, struct.pack("<H", idx))
            t_off = now_ms() - log.t0
            t1 = time.perf_counter()
            try:
                st, echo, _, _ = t.command(SET_FRAME_POSITION_CMD, struct.pack("<H", idx), timeout=args.timeout)
                dt = (time.perf_counter() - t1) * 1000.0
                if echo != SET_FRAME_POSITION_CMD:
                    echo_mismatch += 1
                    ok = False
                    log.write(["a", round(t_off, 3), round(dt, 3), hexstr(req), None, round(now_ms() - log.t0, 3), f"echo 0x{echo:02x}"])
                else:
                    ok = st == 0
                    log.write(["a", round(t_off, 3), round(dt, 3), hexstr(req), st, round(now_ms() - log.t0, 3)])
            except Exception as e:
                dt = (time.perf_counter() - t1) * 1000.0
                log.write(["a", round(t_off, 3), round(dt, 3), hexstr(req), None, round(now_ms() - log.t0, 3), str(e)[:40]])
                ok = False
            if not ok:
                resyncs += 1
                if not resync():
                    resync_failures += 1
                    log.event("runner", phase="resync_failed", count=resync_failures)
                    if resync_failures >= 3:
                        print("link out of sync after 3 failed resyncs — aborting the run", file=sys.stderr)
                        exit_code = 3
                        break
            sent += 1
            if ok:
                accepted += 1
            fails.append(0 if ok else 1)
            if len(fails) > 10:
                fails.pop(0)
            if sum(fails) >= 3:
                log.event("fault", reason="controller_unresponsive", sent=sent, accepted=accepted)
                print("FAULT: >= 3 failed 0x70 in the last 10 — stopping", file=sys.stderr)
                exit_code = 3
                break
            if nowp >= next_drain:
                next_drain = nowp + args.drain_every
                drainer.drain(max_chunks=10)
            if nowp >= next_progress:
                next_progress = nowp + args.progress_every
                el = nowp - t_start
                ncl = len(stats.clusters())
                log.event("soak_progress", sent=sent, accepted=accepted, achieved_hz=round(sent / el, 1) if el else None,
                          missed_slots=missed_slots, stalls=stats.stall_count, clusters=ncl,
                          worst_ms=stats.worst_ms, resyncs=resyncs,
                          reads_fw=stats.reads_fw, capture=drainer.capture(expected_arm=expected_arm))
                print(f"  {el:6.0f}s {sent} sent / {accepted} ok · {sent / el:.1f} Hz achieved · missed slots {missed_slots} · "
                      f"reads≈{stats.index_changes} (fw {stats.reads_fw}) · stalls>{args.gap_ms}ms {stats.stall_count} in {ncl} clusters · "
                      f"worst {stats.worst_ms} ms · seq gaps {len(drainer.tracker.gaps)} · drain errors {drainer.errors}")
    except KeyboardInterrupt:
        print("interrupted")
    except Exception as e:                       # anything outside the per-command handler: the run is not usable
        exit_code = 3
        print(f"run aborted: {e!r}", file=sys.stderr)
        if 'log' in dir():
            log.event("runner", phase="error", error=repr(e)[:200])
    finally:
        try:
            st, echo, _, _ = t.command(STOP_DISPLAY_CMD, timeout=2.0)
            shutdown_verified = (st == 0 and echo == STOP_DISPLAY_CMD)
            log.event("runner", phase="trial_stop", verified=shutdown_verified)
        except Exception:
            log.event("runner", phase="trial_stop", verified=False)
        if diag_set and prev_diag is not None and prev_diag != args.sd_diag:
            # Do not leave the arm on the controller for whoever connects next (Codex round 4).
            try:
                st_d, echo_d, _, _ = t.command(SET_SD_DIAG_CMD, bytes([prev_diag]), timeout=2.0)
                restored = st_d == 0 and echo_d == SET_SD_DIAG_CMD
            except Exception:
                restored = False
            log.event("runner", phase="sd_diag_restore", flags=prev_diag, ok=restored)
            if not restored:
                print(f"WARNING: could not restore SET_SD_DIAG({prev_diag}) — the controller keeps arm {args.sd_diag}", file=sys.stderr)
        if drainer is not None:
            try:
                time.sleep(0.2)
                drainer.drain(max_chunks=400)   # the trailing sd_reads / state_change records
            except Exception:
                pass
            elapsed = time.perf_counter() - t_start if 't_start' in dir() else None
            cap = drainer.capture(accepted, expect_final_reads=shutdown_verified and bool(fw["flags"] & FW_FLAG_SD_FASTPATH),
                                  expected_arm=expected_arm)
            s = stats.summary()
            s.update({"sent": sent, "accepted": accepted, "achieved_hz": round(sent / elapsed, 1) if elapsed else None,
                      "missed_slots": missed_slots, "long_send_intervals": long_intervals, "echo_mismatch": echo_mismatch,
                      "resyncs": resyncs, "resync_failures": resync_failures, "sd_diag_arm": args.sd_diag,
                      "sd_diag_restored_to": prev_diag if diag_set else None, "capture": cap, "shutdown_verified": shutdown_verified,
                      "coverage_note": coverage_note, "response_paced": True,
                      "measurement_usable": cap["valid"] and shutdown_verified and exit_code == 0})
            log.event("sd_stall_summary", **s)
            print("\n=== SD stall summary ===")
            print(json.dumps(s, indent=1))
            if not s["measurement_usable"]:
                print("MEASUREMENT NOT USABLE — " + ("capture incomplete" if not cap["valid"] else "STOP unverified / fault")
                      + "; do not use this summary for a card comparison", file=sys.stderr)
                if exit_code == 0:
                    exit_code = 4
            print(f"\nfull report: python3 webDisplayTools/scripts/telemetry-report.py {log.path}")
        log.close()
        t.close()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
