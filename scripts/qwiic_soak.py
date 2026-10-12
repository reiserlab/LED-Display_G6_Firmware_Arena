"""Bus-limit soak for the Qwiic / STEMMA QT jack: hammer every device on the
chain with read-only register transfers, count I2C errors by status, report
throughput and latency, and read GET_HEALTH (0xCA) before and during the run
so the cost to the controller's loop (loop_max_1s_us) is visible.

    pixi run qwiic-soak                              # auto-detect port, 20 s
    pixi run qwiic-soak -- --seconds 60 --rlen 16    # longer, bigger reads
    pixi run qwiic-soak -- --scan-every 50           # also re-scan the bus every 50 rounds
    pixi run qwiic-soak -- --csv soak.csv            # per-round latency log

LAB-214 "technical limits": run with 1, 3, 5 ... devices and the 100 / 200 mm
cables, at 100 kHz and with a QWIIC_I2C_CLOCK_HZ=400000 build. Zero errors
and a flat latency histogram at the target device count is the pass; the
first NACK / bus error (status 2-4) or a drifting scan marks the limit.

Reads are of ID / status registers only, so the run disturbs no sensor
configuration: SHT4x gets its serial-number command (two legs), the others a
plain register read.
"""

import argparse
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from qwiic_probe import open_transport  # noqa: E402  (adds repo root to sys.path)
from tests.qwiic_peripherals import soak_transfer  # noqa: E402
from tests.qwiic_sensors import (  # noqa: E402
    I2CError,
    NoQwiicJack,
    describe,
    i2c_scan,
    locate_devices,
)
from tests.test_health import read_health  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    ap.add_argument("--ip")
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--rlen", type=int, default=0, choices=range(0, 65), metavar="0..64",
                    help="read at least this many bytes per transfer (default: the ID register's size; "
                         "64 is the firmware's I2C_TRANSFER cap)")
    ap.add_argument("--scan-every", type=int, default=0, help="re-scan the bus every N rounds and flag any change")
    ap.add_argument("--csv", help="write per-transfer latency rows here")
    args = ap.parse_args()

    t, link = open_transport(args)
    try:
        try:
            root = i2c_scan(t)
        except NoQwiicJack as e:
            sys.exit(str(e))
        devices = locate_devices(t)
        if not devices:
            sys.exit(f"nothing on the Qwiic bus ({link})")
        print(f"# {link}: {len(root)} root device(s), {len(devices)} reachable:")
        for d in devices:
            print(f"#   0x{d.addr:02X} @ {d.where():14s} {describe(d.addr)}")

        h0 = read_health(t)
        print(f"# health before: loop_max_1s_us={h0.loop_max_1s_us} loop_max_us={h0.loop_max_us}")

        errors = {}            # (addr, status) -> count
        lat = {d: [] for d in devices}
        rounds = 0
        scan_changes = 0
        loop_max_1s = []
        csv = open(args.csv, "w") if args.csv else None
        if csv:
            csv.write("round,addr,where,latency_ms,error\n")
        t_start = time.monotonic()
        next_health = t_start + 1.0
        try:
            while time.monotonic() - t_start < args.seconds:
                rounds += 1
                for d in devices:
                    err = ""
                    try:
                        dt = soak_transfer(t, d, args.rlen)
                        lat[d].append(dt)
                    except I2CError as e:
                        dt = float("nan")
                        err = str(e.status)
                        errors[(d.addr, e.status)] = errors.get((d.addr, e.status), 0) + 1
                    if csv:
                        csv.write(f"{rounds},0x{d.addr:02X},{d.where()},{dt * 1000:.3f},{err}\n")
                if args.scan_every and rounds % args.scan_every == 0:
                    now = i2c_scan(t)
                    if now != root:
                        scan_changes += 1
                        print(f"! scan changed at round {rounds}: {[hex(a) for a in now]}")
                if time.monotonic() >= next_health:
                    h = read_health(t)
                    loop_max_1s.append(h.loop_max_1s_us)
                    next_health += 1.0
        except KeyboardInterrupt:
            print("# interrupted")
        finally:
            if csv:
                csv.close()
        elapsed = time.monotonic() - t_start
        h1 = read_health(t)

        total = sum(len(v) for v in lat.values()) + sum(errors.values())
        print(f"\n# {rounds} rounds, {total} transfers in {elapsed:.1f} s = {total / elapsed:.0f} transfers/s"
              f" ({rounds / elapsed:.1f} rounds/s over {len(devices)} devices)")
        print(f"# errors: {sum(errors.values())}"
              + ("" if not errors else "  " + ", ".join(f"0x{a:02X} status {s}: {n}" for (a, s), n in sorted(errors.items()))))
        if scan_changes:
            print(f"# !! bus scan changed {scan_changes} time(s) during the run")
        print("# latency per device (host round trip, ms):")
        for d in devices:
            v = sorted(lat[d])
            if not v:
                print(f"#   0x{d.addr:02X} @ {d.where():14s} no successful transfer")
                continue
            p99 = v[min(len(v) - 1, int(len(v) * 0.99))]
            print(f"#   0x{d.addr:02X} @ {d.where():14s} n={len(v):6d} mean {statistics.mean(v) * 1000:6.2f}"
                  f"  p50 {v[len(v) // 2] * 1000:6.2f}  p99 {p99 * 1000:6.2f}  max {v[-1] * 1000:6.2f}")
        if loop_max_1s:
            print(f"# controller loop_max_1s_us while soaking: median {int(statistics.median(loop_max_1s))}"
                  f"  max {max(loop_max_1s)}  (before: {h0.loop_max_1s_us}; boot max now {h1.loop_max_us})")
        sys.exit(1 if errors or scan_changes else 0)
    finally:
        t.close()


if __name__ == "__main__":
    main()
