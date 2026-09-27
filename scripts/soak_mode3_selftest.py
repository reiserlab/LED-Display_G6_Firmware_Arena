#!/usr/bin/env python3
"""Self-test for soak_mode3.py: exercises the whole fault lifecycle against the
in-process FakeLink (no hardware, no pyserial needed).

Run 1  --on-fault halt:            fake wedges after 150 commands; expect the
                                   schema line, >= 150 ["a", ...] rows, exactly
                                   one `fault`, >= 1 `probe`, soak_end fault-halt,
                                   exit code 2.
Run 2  --on-fault reset-continue:  same wedge; expect `system_reset`,
                                   `post_reset_probe` (with prev_boot summary),
                                   `soak_resumed`, more ["a"] rows (incl. 0x70s)
                                   after it, soak_end max-commands, exit code 0.

    python3 soak_mode3_selftest.py
"""

import json
import os
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
DRIVER = os.path.join(HERE, "soak_mode3.py")

COMMON = [
    "--dry-run", "--dry-run-wedge-after", "150", "--dry-run-wedge-latency", "0.6",
    "--hz", "200", "--timeout", "0.25", "--health-every", "0.3", "--progress-every", "1",
    "--quiet-seconds", "0.3", "--probe-seconds", "2", "--probe-interval", "1",
    "--probe-timeout", "3", "--seed", "7",
]


def run(label, extra):
    log_dir = tempfile.mkdtemp(prefix=f"soak-selftest-{label}-")
    cmd = [sys.executable, DRIVER, *COMMON, "--log-dir", log_dir, *extra]
    t0 = time.monotonic()
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    files = [f for f in os.listdir(log_dir) if f.endswith(".jsonl")]
    assert len(files) == 1, f"{label}: expected one log file, got {files}"
    path = os.path.join(log_dir, files[0])
    lines = [json.loads(l) for l in open(path, encoding="utf-8") if l.strip()]
    print(f"[{label}] exit={proc.returncode} {time.monotonic() - t0:.1f}s lines={len(lines)} -> {path}")
    for l in proc.stderr.strip().splitlines():
        print(f"[{label}] stderr: {l}")
    return proc.returncode, lines, files[0]


def rows(lines):
    return [l for l in lines if isinstance(l, list) and l[0] == "a"]


def events(lines, name):
    return [l for l in lines if isinstance(l, dict) and l.get("event") == name]


def check(cond, msg):
    if not cond:
        raise AssertionError(msg)
    print(f"  ok: {msg}")


def main():
    # ---- run 1: halt ---------------------------------------------------------
    rc, lines, name = run("halt", ["--on-fault", "halt"])
    check(name.startswith("soak-") and name.endswith("-mode3-200hz.jsonl"), f"log name {name}")
    check(lines[0].get("type") == "frame_schema" and lines[0].get("level") == "behavior_v2",
          "first line is the behavior_v2 frame_schema")
    check(lines[0].get("arena_cols") == ["t_off", "dt", "hex", "status", "rx_off"], "arena_cols match bridge")
    meta = events(lines, "run_metadata")
    check(len(meta) == 1 and meta[0]["controller"]["mac"] == "04:E9:E5:12:91:C0", "run_metadata with controller mac")
    a = rows(lines)
    check(len(a) >= 150, f"{len(a)} arena rows (>= 150)")
    check(all(isinstance(r[1], int) and isinstance(r[5], int) for r in a), "t_off/rx_off are ints")
    r70 = [r for r in a if r[3].startswith("0370")]
    check(len(r70) >= 100, f"{len(r70)} 0x70 rows")
    tp = [r for r in a if r[3].startswith("0d08")]
    check(len(tp) == 1 and tp[0][3] == "0d08" + "03" + "0100" + "0000" + "0000" + "0000" + "0000" + "00",
          f"TRIAL_PARAMS row {tp[0][3] if tp else None}")
    timeouts = [r for r in a if len(r) == 7 and r[4] is None]
    check(len(timeouts) >= 3, f"{len(timeouts)} timeout rows carry status null + error string")
    check(len(events(lines, "fault")) == 1, "exactly one fault event")
    f = events(lines, "fault")[0]
    check(f["failures"] >= 3 and f["window"] <= 10, f"fault rule {f['failures']}/{f['window']}")
    check(len(events(lines, "health")) >= 1, "health ticks logged")
    h = events(lines, "health")[-1]["health"]
    check(h is None or ("slow_us" in h and "prev_slow_op_name" in h), "66-byte health decode incl. tail")
    probes = events(lines, "probe")
    check(len(probes) >= 1, f"{len(probes)} probe events")
    names = {p["name"] for p in probes}
    check({"controller_info", "frames_sent", "frame_position", "health", "pattern_info_1", "firmware_info"} <= names,
          f"probe set {sorted(names)}")
    fw = [p for p in probes if p["name"] == "firmware_info"][0]
    check(fw["status"] == 1 and fw["dt_ms"] is not None, "0xE3 status 1 logged with latency")
    check(all(p["dt_ms"] is None or p["dt_ms"] >= 500 for p in probes), "probes see the degraded latency")
    end = events(lines, "soak_end")
    check(len(end) == 1 and end[0]["reason"] == "fault-halt", "soak_end reason fault-halt")
    check(rc == 2, f"exit code {rc} == 2")
    check(len(events(lines, "system_reset")) == 0, "halt policy never sends SYSTEM_RESET")
    check(len(events(lines, "soak_progress")) >= 1, "soak_progress emitted")

    # ---- run 2: reset-continue ----------------------------------------------
    rc, lines, name = run("reset", ["--on-fault", "reset-continue", "--max-commands", "300",
                                   "--reset-wait", "0.2", "--reconnect-seconds", "2"])
    check(len(events(lines, "fault")) == 1, "one fault")
    check(len(events(lines, "system_reset")) == 1, "one system_reset")
    pr = events(lines, "post_reset_probe")
    check(len(pr) == 1, "one post_reset_probe")
    pb = pr[0]["prev_boot"]
    check(pb and pb["prev_slow_op"] == "sd_read" and pb["prev_slow_us"] == 987 and pb["prev_last_op"] == "command"
          and pb["prev_last_op_arg"] == "0x70", f"prev_boot summary {pb}")
    check(len(events(lines, "soak_resumed")) == 1, "soak_resumed")
    idx = lines.index(pr[0])
    after = rows(lines[idx:])
    after70 = [r for r in after if r[3].startswith("0370") and r[4] == 0]
    check(len(after70) >= 50, f"{len(after70)} successful 0x70 rows after the reset")
    check(len(events(lines, "trial_params")) == 2, "pattern re-opened after reset")
    end = events(lines, "soak_end")
    check(len(end) == 1 and end[0]["reason"] == "max-commands", "soak_end reason max-commands")
    check(end[0]["resets"] == 1 and end[0]["sent70"] >= 300, f"end counters {end[0]}")
    check(rc == 0, f"exit code {rc} == 0")
    print("\nSELFTEST PASSED")


if __name__ == "__main__":
    main()
