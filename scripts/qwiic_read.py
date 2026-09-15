"""Continuous readout of the light sensors on the Qwiic jack — one line per
sample until Ctrl-C, then min / max / mean per channel.

    pixi run qwiic-read                                  # auto-detect port, as fast as fresh data allows
    pixi run qwiic-read -- --interval 0.5                # extra pause between samples
    pixi run qwiic-read -- --gain high --atime 300       # TSL2591 gain low|med|high|max, integration ms
    pixi run qwiic-read -- --as-gain 256                 # AS7343 gain 0.5..2048 (default 64)
    pixi run qwiic-read -- --csv > light.csv             # machine-readable
    pixi run qwiic-read -- --count 50                    # stop after 50 samples

Reads every TSL2591, VEML7700 and AS7343 found (root bus or behind a
PCA9548). The sensors integrate in parallel: each round starts the AS7343
frame, reads the continuously-integrating TSL2591 / VEML7700 while it runs,
then collects the AS7343 — so a round takes the slowest integration time
(~150 ms with the AS7343 at its defaults, 100 ms without it), not the sum.
Rounds are paced to that time so every line is fresh data.

Lux values are the vendor approximations and AS7343 counts are raw ADC
values (12 spectral channels by centre wavelength plus the clear VIS
channel), not calibrated irradiance; "SAT" marks a saturated reading.
"""

import argparse
import statistics
import sys
import time

from qwiic_probe import open_transport  # noqa: F401  (adds repo root to sys.path)
from tests.qwiic_sensors import (  # noqa: E402
    AS7343,
    TSL2591,
    VEML7700,
    NoQwiicJack,
    locate_devices,
    reach,
)

TSL_CEILING = {100: 36863}  # other integration times saturate at 65535
VEML_IT_MS = 100


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    ap.add_argument("--ip")
    ap.add_argument("--interval", type=float, default=0.0,
                    help="extra seconds between rounds (default 0: paced by the slowest integration)")
    ap.add_argument("--count", type=int, default=0, help="stop after N rounds (default: until Ctrl-C)")
    ap.add_argument("--gain", choices=list(TSL2591.GAIN), default="med", help="TSL2591 gain (default med = x25)")
    ap.add_argument("--atime", type=int, choices=list(TSL2591.ATIME), default=100, help="TSL2591 integration ms")
    ap.add_argument("--as-gain", type=float, choices=list(AS7343.GAIN_CODE), default=64,
                    help="AS7343 gain multiplier (default 64)")
    ap.add_argument("--csv", action="store_true", help="CSV to stdout instead of aligned text")
    args = ap.parse_args()

    t, link = open_transport(args)
    try:
        try:
            devices = locate_devices(t)
        except NoQwiicJack as e:
            sys.exit(str(e))
        sensors = []
        period_ms = 0.0
        for dev in devices:
            if dev.addr == TSL2591.ADDR:
                s = TSL2591(t)
                with reach(t, dev):
                    s.configure(gain=args.gain, atime_ms=args.atime)
                sensors.append((dev, s, ["full", "ir", "lux", "sat"]))
                period_ms = max(period_ms, args.atime)
            elif dev.addr == VEML7700.ADDR:
                s = VEML7700(t)
                with reach(t, dev):
                    s.configure()
                sensors.append((dev, s, ["als", "white", "lux"]))
                period_ms = max(period_ms, VEML_IT_MS)
            elif dev.addr == AS7343.ADDR:
                s = AS7343(t)
                with reach(t, dev):
                    s.configure(gain=args.as_gain)
                sensors.append((dev, s, list(AS7343.SPECTRAL) + ["VIS", "sat"]))
                period_ms = max(period_ms, s.frame_ms)
        if not sensors:
            sys.exit(f"no TSL2591 / VEML7700 / AS7343 on the Qwiic bus ({link}); found "
                     + (", ".join(f"0x{d.addr:02X}" for d in devices) or "nothing"))

        tags = [f"{type(s).__name__}[{d.where()}]" for d, s, _ in sensors]
        columns = [f"{tag}.{c}" for (d, s, cols), tag in zip(sensors, tags) for c in cols]
        history = {c: [] for c in columns}
        spectral = [(d, s) for d, s, _ in sensors if isinstance(s, AS7343)]
        integrating = [(d, s) for d, s, _ in sensors if not isinstance(s, AS7343)]
        if args.csv:
            print("time," + ",".join(columns))
        else:
            print(f"# {link}; TSL2591 gain={args.gain} atime={args.atime} ms; AS7343 gain={args.as_gain:g}; "
                  f"round = {period_ms:.0f} ms (~{1000 / period_ms:.1f} Hz) + {args.interval:g} s; Ctrl-C for stats",
                  file=sys.stderr)

        ceiling = TSL_CEILING.get(args.atime, 65535)
        n = 0
        t_first = time.monotonic()
        try:
            while not args.count or n < args.count:
                round_start = time.monotonic()
                for dev, s in spectral:
                    with reach(t, dev):
                        s.start()
                results = {}
                for dev, s in integrating:
                    with reach(t, dev):
                        results[id(s)] = s.read()
                for dev, s in spectral:
                    with reach(t, dev):
                        results[id(s)] = s.collect()

                stamp = time.strftime("%H:%M:%S") + f".{int(time.time() * 1000) % 1000:03d}"
                values, cells = [], []
                for (dev, s, cols), tag in zip(sensors, tags):
                    r = results[id(s)]
                    if isinstance(s, TSL2591):
                        sat = r["full"] >= ceiling or r["ir"] >= ceiling
                        lux = None if sat else r["lux_approx"]
                        values += [r["full"], r["ir"], lux, int(sat)]
                        lux_text = f"{lux:8.1f}" if lux is not None else "     n/a"
                        cells.append(f"{tag} full={r['full']:5d} ir={r['ir']:5d} lux={lux_text}"
                                     + ("  SAT" if sat else ""))
                    elif isinstance(s, VEML7700):
                        values += [r["als"], r["white"], r["lux_approx"]]
                        cells.append(f"{tag} als={r['als']:5d} white={r['white']:5d} lux={r['lux_approx']:8.1f}")
                    else:
                        values += [r["spectral"][k] for k in AS7343.SPECTRAL] + [r["vis"], int(r["sat"])]
                        cells.append(f"{tag} " + " ".join(f"{k.split('_')[1]}:{v:5d}" for k, v in r["spectral"].items())
                                     + f" VIS:{r['vis']:5.0f}"
                                     + ("  SAT" if r["sat"] else "") + (" (VIS sat)" if r["sat_vis"] else ""))
                for c, v in zip(columns, values):
                    if v is not None and not c.endswith(".sat"):
                        history[c].append(v)
                if args.csv:
                    print(stamp + "," + ",".join("" if v is None else f"{v:.3f}" if isinstance(v, float) else str(v)
                                                 for v in values), flush=True)
                else:
                    print(f"{stamp}  " + "\n              ".join(cells), flush=True)
                n += 1
                # Pace to the slowest integration so the next round is fresh data.
                wait = period_ms / 1000.0 - (time.monotonic() - round_start) + args.interval
                if wait > 0:
                    time.sleep(wait)
        except KeyboardInterrupt:
            pass
        finally:
            elapsed = time.monotonic() - t_first
            for dev, s, _ in sensors:
                if isinstance(s, (TSL2591, AS7343)):
                    with reach(t, dev):
                        s.power_off()

        if n:
            print(f"\n# {n} rounds in {elapsed:.1f} s ({n / elapsed:.1f} Hz)", file=sys.stderr)
            for c in columns:
                h = history[c]
                if c.endswith(".lux") and len(h) < n:
                    print(f"#   {c:28s} {n - len(h)} of {n} samples saturated — lower --gain / --atime",
                          file=sys.stderr)
                if h:
                    mean = statistics.mean(h)
                    spread = (max(h) - min(h)) / mean * 100 if mean else 0.0
                    print(f"#   {c:28s} min {min(h):9.1f}  max {max(h):9.1f}  mean {mean:9.1f}"
                          f"  spread {spread:5.1f} %", file=sys.stderr)
    finally:
        t.close()


if __name__ == "__main__":
    main()
