"""3-port CIPO bench capture: drive all-on on the Teensy AND tee both panels'
SPI_DIAG heartbeats at the same time.

Each device is on its own USB-CDC port (no contention). One reader thread per
port tees each stream to log/<ts>_<label>_<role>.log as text -- ASCII
diagnostics stay readable and binary framing bytes are rendered as \\xNN hex
escapes (not saved raw). The main thread sends the all-on command to the
Teensy, waits, then sends all-off. Everything is parsed from the logs at the
end.

    pixi run python scripts/multi_port_capture.py \
        --teensy /dev/cu.usbmodem121699401 \
        --panel v021=/dev/cu.usbmodem11401 --panel v031=/dev/cu.usbmodem11301 \
        --label B --duration 50

Panel answer ("did it get the frames"): heartbeats fire every 1000 received
msgs, so #heartbeats*1000 ~= frames; the last gap_us bin-sum is the cumulative
msg count; gate p/l/pr/cmd/cc is the per-sample validity; PARITY lines flag bad
frames; fskip/qdrop are cumulative drops.

Requires diagnostic firmware on BOTH ends -- the production builds emit none of
the strings parsed below:
  * Panels: build with -DSPI_DIAG=1 (Panel-Firmware env pico_v021_spidiag /
    pico_v031_spidiag; flash via `pixi run deploy21a-diag` / `deploy31a-diag`).
    Gates the gap_us / gate / PARITY / txCIPO / qdrop / fskip heartbeats in
    panel/src/messenger.cpp. Without it summarize_panel() reports 0 heartbeats.
  * Teensy/arena: the `[spi] CIPO` rows are gated on -DDEBUG_SERIAL (the
    DBG_PRINTF macro in src/constants.h, emitted from src/SpiManager.cpp). Build
    with env teensy41-printf and flash via `pixi run deploy-printf`. The default
    `pixi run deploy` (env teensy41) compiles them out, so summarize_teensy()
    finds no rows.
"""

import argparse
import re
import threading
import time

from cipo_common import (
    ALL_OFF,
    ALL_ON,
    CIPO_RE,
    classify,
    enable_diagnostics,
    open_cdc,
    safe_text,
    timestamped_log,
)

GAP_RE = re.compile(r"gap_us \[.*?\]:\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+max=(\d+)")
GATE_RE = re.compile(r"gate p/l/pr/cmd/cc=(\d)/(\d)/(\d)/(\d)/(\d).*?hdr=0x([0-9A-Fa-f]+)\s+cmd=0x([0-9A-Fa-f]+)")
FSKIP_RE = re.compile(r"qdrop=(\d+).*?fskip=(\d+)")
TXCIPO_RE = re.compile(r"txCIPO=([0-9A-Fa-f]{2})\s+([0-9A-Fa-f]{2})\s+([0-9A-Fa-f]{2})")


def reader(ser, path, stop):
    with open(path, "w", encoding="ascii") as f:
        while not stop.is_set():
            chunk = ser.read(4096)
            if chunk:
                f.write(safe_text(chunk)); f.flush()


def summarize_teensy(path, label):
    rows, tally = 0, {}
    active = {}
    with open(path, "rb") as f:
        for raw in f:
            m = CIPO_RE.search(raw.decode("ascii", "replace"))
            if not m: continue
            rows += 1
            setn = m.group(1)
            for grp in ((3, 4, 5), (6, 7, 8)):
                c = classify(*(m.group(g) for g in grp))
                tally[c] = tally.get(c, 0) + 1
                if c != "BLOCKED":
                    active.setdefault(f"set{setn}/cs={m.group(2)}", {})
                    active[f"set{setn}/cs={m.group(2)}"][c] = \
                        active[f"set{setn}/cs={m.group(2)}"].get(c, 0) + 1
    print(f"\n=== TEENSY [{label}] === CIPO rows={rows}  tally={tally}")
    for k in sorted(active):
        print(f"    {k}: {active[k]}")


def summarize_panel(path, name):
    hb = 0
    last_gap = None
    gate_lines = []
    parity_miss = 0
    last_fskip = last_qdrop = None
    last_txcipo = None
    with open(path, "rb") as f:
        for raw in f:
            line = raw.decode("ascii", "replace")
            if "gap_us [" in line:
                hb += 1
                g = GAP_RE.search(line)
                if g: last_gap = sum(int(g.group(i)) for i in range(1, 7))
            if "gate p/l" in line:
                g = GATE_RE.search(line)
                if g: gate_lines.append(g.groups())
                fq = FSKIP_RE.search(line)
                if fq: last_qdrop, last_fskip = fq.group(1), fq.group(2)
                tx = TXCIPO_RE.search(line)
                if tx: last_txcipo = " ".join(tx.groups())
            if "PARITY calc=" in line:
                parity_miss += 1
    frames_est = hb * 1000
    print(f"\n=== PANEL {name} ===")
    print(f"    heartbeats={hb}  -> ~{frames_est} frames received "
          f"(last cumulative msg_count={last_gap})")
    if gate_lines:
        g = gate_lines[-1]
        allpass = sum(1 for gl in gate_lines if gl[:5] == ('1','1','1','1','1'))
        print(f"    last gate p/l/pr/cmd/cc={'/'.join(g[:5])} hdr=0x{g[5]} cmd=0x{g[6]}"
              f"   ({allpass}/{len(gate_lines)} sampled frames fully passed)")
    print(f"    PARITY-miss heartbeats={parity_miss}   last qdrop={last_qdrop} fskip={last_fskip}")
    print(f"    last txCIPO (panel intends to send)={last_txcipo}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--teensy", required=True)
    ap.add_argument("--panel", action="append", default=[], help="name=port")
    ap.add_argument("--label", default="run")
    ap.add_argument("--duration", type=float, default=50.0)
    ap.add_argument("--logdir", default="log")
    args = ap.parse_args()

    panels = [p.split("=", 1) for p in args.panel]

    ports = {}
    ports["teensy"] = (open_cdc(args.teensy),
                       timestamped_log(args.logdir, args.label, "teensy"))
    for name, dev in panels:
        ports[name] = (open_cdc(dev),
                       timestamped_log(args.logdir, args.label, name))

    stop = threading.Event()
    threads = []
    for name, (ser, path) in ports.items():
        t = threading.Thread(target=reader, args=(ser, path, stop), daemon=True)
        t.start(); threads.append(t)

    teensy = ports["teensy"][0]
    enable_diagnostics(teensy)  # undo any prior web-serial mute so CIPO rows flow
    print(f"[{args.label}] all_on; capturing {args.duration:.0f}s across "
          f"{len(ports)} ports ({', '.join(ports)})")
    time.sleep(0.3)
    teensy.write(bytes([0x01, ALL_ON])); teensy.flush()
    time.sleep(args.duration)
    teensy.write(bytes([0x01, ALL_OFF])); teensy.flush()
    time.sleep(0.3)
    stop.set()
    for t in threads: t.join(timeout=2)
    for ser, _ in ports.values(): ser.close()

    summarize_teensy(ports["teensy"][1], args.label)
    for name, _ in panels:
        summarize_panel(ports[name][1], name)
    print("\nlogs:", ", ".join(p for _, p in ports.values()))


if __name__ == "__main__":
    main()
