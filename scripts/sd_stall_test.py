#!/usr/bin/env python3
"""sd_stall_test.py — SD-card stall characterisation / card-comparison driver (no browser).

Reproduces the Mode-3 closed-loop access pattern on the wire — TRIAL_PARAMS (0x08, mode 3) then
SET_FRAME_POSITION (0x70) at a fixed rate with a FicTrac-like random-walk index (plus periodic
jumps) — while draining the controller's telemetry ring (0xA9, ack cursor) into a behavior_v2
NDJSON log with the SAME row shapes the Arena Studio + bridge write:

    {"type":"frame_schema","level":"behavior_v2",...,"t0":<epoch ms>}
    {"type":"log","event":"run_metadata","tool":"sd_stall_test","firmware":"…","sd_card":{…},"args":{…}}
    {"event":"stream_schema","streams":{"cc":…,"cf":…,"cs":…}}
    ["a",  t_off_ms, dt_ms, "<request hex>", status|null, rx_off_ms]        host command echo
    ["cc", rx_ms, t_us, seq, cmd, status, "<req hex>"]                       controller CMD record
    ["cf", rx_ms, t_us, seq, idx, pattern, sd_load_us, spi_us, req_age_us, superseded, flags]
    ["cs", rx_ms, t_us, seq, kind, code, arg]
    {"type":"log","event":"sd_stall_summary", …}                             inline summary at the end

so one analyzer serves both: webDisplayTools/scripts/telemetry-report.py. The inline summary
answers the card question on its own: stall clusters, their spacing in reads, durations and
phases, SD read cost by index step, and the card identity (GET_SD_INFO 0xCD) they belong to.

Why: the card stalls 23–89 ms every ~24.5k reads of a large pattern (card-internal read-count
maintenance, 2026-09-13). Acceptable display freeze: 5 ms target / 10 ms worst case. A 20-minute
run at 200 Hz (≈ 5 baseline cycles) SCREENS a card; qualification needs hours and a second specimen.

Usage:
    # 20 min at 200 Hz on SD pattern 36 (the 200-frame bar), 90° jumps every 100 samples
    python3 scripts/sd_stall_test.py --port /dev/cu.usbmodem123456 --pattern 36 --label sandisk-32g

    # working-set test on the same file: indices restricted to the first 50 frames
    python3 scripts/sd_stall_test.py --port … --pattern 36 --window 50 --minutes 20

    # stop after N accepted reads instead of a time budget
    python3 scripts/sd_stall_test.py --port … --pattern 36 --reads 150000

Requires pyserial (system python3 has it; the pixi env may not). Exit codes: 0 normal end,
1 startup failure (no controller / pattern / telemetry), 2 bad arguments, 3 controller stopped
answering (fault) — the log is complete either way.
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
    GET_PATTERN_INFO_CMD,
    GET_SD_INFO_CMD,
    GET_TELEMETRY_BLOCK_CMD,
    SET_FRAME_POSITION_CMD,
    SET_TELEMETRY_CMD,
    STOP_DISPLAY_CMD,
    TRIAL_PARAMS_CMD,
)
from tests import telemetry_codec as tc  # noqa: E402
from tests.transport import SerialTransport, build_frame  # noqa: E402

CAP_HEALTH = 0x80
FW_FLAG_TELEMETRY = 0x04
FW_FLAG_SD_FASTPATH = 0x20
CLUSTER_GAP_S = 5.0
SD_MANUFACTURERS = {0x01: "Panasonic", 0x02: "Toshiba/Kioxia", 0x03: "SanDisk", 0x09: "ATP", 0x13: "KingMax",
                    0x1B: "Samsung", 0x1D: "ADATA", 0x27: "Phison", 0x28: "Lexar", 0x31: "Silicon Power",
                    0x41: "Kingston", 0x5D: "Swissbit", 0x6F: "STMicro", 0x74: "Transcend", 0x76: "Patriot",
                    0x82: "Sony/Gobe", 0x9C: "Angelbird/Hoodman"}
SD_CARD_TYPES = ["none", "SD1", "SD2", "SDHC/SDXC"]


def now_ms() -> float:
    return time.time() * 1000.0


def hexstr(b: bytes) -> str:
    return " ".join(f"{x:02x}" for x in b)


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


class Drainer:
    """Ack-cursor ring drain (lossless): records are freed only by the NEXT request's ack."""

    def __init__(self, t: SerialTransport, log: Log, timeout: float):
        self.t = t
        self.log = log
        self.timeout = timeout
        self.ack = tc.NO_ACK
        self.tracker = tc.SeqTracker()
        self.rows = {"cc": 0, "cf": 0, "cs": 0}
        self.errors = 0
        self.records = []   # (rec, rx_ms) kept for the inline summary

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
                self.records.append((r, rx))
                n += 1
            if recs:
                self.ack = recs[-1].seq
            if not hdr.more:
                break
        return n


def summarize(records, gap_ms: float, accepted_reads: int) -> dict:
    """Inline analysis (a subset of webDisplayTools/scripts/telemetry-report.py)."""
    stalls = []
    step = {}
    prev_idx = None
    cmds = 0
    frames = 0
    ages = []
    for r, rx in records:
        f = r.fields
        if r.type == tc.REC_CMD and f["cmd"] == SET_FRAME_POSITION_CMD and f["status"] == 0:
            cmds += 1
        elif r.type == tc.REC_STATE and f["kind"] == tc.ST_SD_SLOW:
            ms = f["arg"] / 10.0
            if ms > gap_ms:
                stalls.append({"rx": rx, "ms": ms, "phase": f.get("phase", "unknown"), "cmds": cmds})
        elif r.type == tc.REC_FRAME:
            frames += 1
            if prev_idx is not None:
                d = f["idx"] - prev_idx
                cls = "+1" if d == 1 else "-1" if d == -1 else "small" if abs(d) < 10 else "jump"
                step.setdefault(cls, []).append(f["sd_load_us"])
            prev_idx = f["idx"]
            if "req_age_us" in f:
                ages.append(f["req_age_us"])
    clusters = []
    for s in stalls:
        if clusters and (s["rx"] - clusters[-1][-1]["rx"]) / 1000.0 < CLUSTER_GAP_S:
            clusters[-1].append(s)
        else:
            clusters.append([s])
    spacing = [c[0]["cmds"] - p[0]["cmds"] for p, c in zip(clusters, clusters[1:])]
    return {
        "accepted_0x70": accepted_reads,
        "frames": frames,
        "stalls_over_gap": len(stalls),
        "clusters": len(clusters),
        "clusters_per_1e5_cmds": round(len(clusters) * 1e5 / cmds, 2) if cmds else None,
        "cluster_ms": [[round(s["ms"], 1) for s in c] for c in clusters],
        "cluster_phases": [[s["phase"] for s in c] for c in clusters],
        "spacing_cmds_median": statistics.median(spacing) if spacing else None,
        "spacing_cmds": spacing,
        "worst_ms": max((s["ms"] for s in stalls), default=0.0),
        "step_cost_us_p50": {k: statistics.median(v) for k, v in step.items()},
        "step_cost_us_p99": {k: sorted(v)[min(len(v) - 1, int(0.99 * len(v)))] for k, v in step.items()},
        "req_age_us": {"p50": statistics.median(ages) if ages else None,
                       "p99": sorted(ages)[min(len(ages) - 1, int(0.99 * len(ages)))] if ages else None,
                       "max": max(ages) if ages else None,
                       "over_gap": sum(1 for a in ages if a > gap_ms * 1000)},
    }


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", required=True, help="serial device, e.g. /dev/cu.usbmodem123456 or COM7")
    p.add_argument("--pattern", type=int, required=True, help="1-based SD pattern index (the large pattern)")
    p.add_argument("--frames", type=int, default=0, help="override the frame count (default: GET_PATTERN_INFO)")
    p.add_argument("--hz", type=float, default=200.0, help="0x70 rate (default 200)")
    p.add_argument("--minutes", type=float, default=20.0, help="run length (default 20; 0 = until --reads / Ctrl-C)")
    p.add_argument("--reads", type=int, default=0, help="stop after N accepted 0x70s (0 = no limit)")
    p.add_argument("--sigma", type=float, default=1.6, help="random-walk step sigma, frames per sample (default 1.6)")
    p.add_argument("--jump-every", type=int, default=100, help="one jump every N samples (0 = none; default 100)")
    p.add_argument("--jump-deg", type=float, default=90.0, help="jump size in degrees of the pattern's 360° (default 90)")
    p.add_argument("--window", type=int, default=0, help="confine indices to the first N frames (working-set test; 0 = all)")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--gap-ms", type=float, default=10.0, help="stall threshold for the summary (default 10)")
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
    accepted = 0
    exit_code = 0
    drainer = None
    try:
        t.command(STOP_DISPLAY_CMD, timeout=2.0)
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
        sd_card = None
        if fw["flags"] & FW_FLAG_SD_FASTPATH:
            st, _, sdp, _ = t.command(GET_SD_INFO_CMD, timeout=2.0)
            if len(sdp) >= 30:
                sd_card = decode_sd_info(bytes(sdp))
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
                  frames=frames, log_format="behavior_v2", telemetry="ring-10hz",
                  args={k: v for k, v in vars(args).items() if k != "port"}, gap_threshold_ms=args.gap_ms)
        log.write({"event": "stream_schema", "streams": {
            "cc": {"cols": ["rx", "t_us", "seq", "cmd", "status", "req"]},
            "cf": {"cols": ["rx", "t_us", "seq", "idx", "pattern", "sd_load_us", "spi_us", "req_age_us", "superseded", "flags"]},
            "cs": {"cols": ["rx", "t_us", "seq", "kind", "code", "arg"], "kinds": tc.STATE_KIND_NAMES}}})
        print(f"firmware: {fw['label']}")
        print(f"sd card : {sd_card['label'] if sd_card else '(firmware without GET_SD_INFO)'}")
        print(f"pattern {args.pattern}: {frames} frames, window {walker.n}, {args.hz} Hz, sigma {args.sigma}, "
              f"jump {args.jump_deg}° every {args.jump_every}, {args.minutes} min")

        st, _, _, _ = t.command(SET_TELEMETRY_CMD, tc.build_set_telemetry(tc.SET_FLAG_EVENTS, None), timeout=2.0)
        if st != 0:
            print("SET_TELEMETRY refused", file=sys.stderr)
            return 1
        drainer = Drainer(t, log, args.timeout)
        drainer.drain(max_chunks=400)  # backlog from before this run (kept in the log, tagged by seq)
        log.event("telemetry_backlog_drained", records=len(drainer.records))

        tp = (bytes([3]) + struct.pack("<H", args.pattern) + struct.pack("<h", 0) + struct.pack("<H", 0)
              + struct.pack("<h", 0) + struct.pack("<H", 0))
        st, _, _, _ = t.command(TRIAL_PARAMS_CMD, tp, timeout=10.0)
        if st != 0:
            print(f"TRIAL_PARAMS failed status={st}", file=sys.stderr)
            return 1
        log.event("runner", phase="trial_start", pattern=args.pattern, mode=3)

        period = 1.0 / args.hz
        t_end = time.perf_counter() + args.minutes * 60.0 if args.minutes > 0 else None
        next_send = time.perf_counter()
        next_drain = next_send + args.drain_every
        next_progress = next_send + args.progress_every
        fails = []  # rolling window of the last 10 outcomes
        sent = 0
        while True:
            nowp = time.perf_counter()
            if t_end and nowp >= t_end:
                break
            if args.reads and accepted >= args.reads:
                break
            if nowp < next_send:
                time.sleep(min(next_send - nowp, 0.002))
                continue
            next_send += period
            idx = walker.next()
            req = build_frame(SET_FRAME_POSITION_CMD, struct.pack("<H", idx))
            t_off = now_ms() - log.t0
            t1 = time.perf_counter()
            try:
                st, _, _, _ = t.command(SET_FRAME_POSITION_CMD, struct.pack("<H", idx), timeout=args.timeout)
                dt = (time.perf_counter() - t1) * 1000.0
                log.write(["a", round(t_off, 3), round(dt, 3), hexstr(req), st, round(now_ms() - log.t0, 3)])
                ok = st == 0
            except Exception as e:
                dt = (time.perf_counter() - t1) * 1000.0
                log.write(["a", round(t_off, 3), round(dt, 3), hexstr(req), None, round(now_ms() - log.t0, 3), str(e)[:40]])
                ok = False
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
                s = summarize(drainer.records, args.gap_ms, accepted)
                log.event("soak_progress", sent=sent, accepted=accepted, stalls=s["stalls_over_gap"],
                          clusters=s["clusters"], worst_ms=s["worst_ms"])
                print(f"  {sent} sent / {accepted} accepted · stalls>{args.gap_ms}ms {s['stalls_over_gap']} in "
                      f"{s['clusters']} clusters · worst {s['worst_ms']} ms · seq gaps {len(drainer.tracker.gaps)}")
    except KeyboardInterrupt:
        print("interrupted")
    finally:
        try:
            t.command(STOP_DISPLAY_CMD, timeout=2.0)
            log.event("runner", phase="trial_stop")
        except Exception:
            pass
        if drainer is not None:
            try:
                time.sleep(0.2)
                drainer.drain(max_chunks=400)
            except Exception:
                pass
            s = summarize(drainer.records, args.gap_ms, accepted)
            s.update({"seq_gaps": len(drainer.tracker.gaps), "drain_errors": drainer.errors, "rows": drainer.rows})
            log.event("sd_stall_summary", **s)
            print("\n=== SD stall summary ===")
            print(json.dumps(s, indent=1))
            print(f"\nfull report: python3 webDisplayTools/scripts/telemetry-report.py {log.path}")
        log.close()
        t.close()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
