"""Evaluate the light sensors daisy-chained on the Qwiic jack.

One command, three modes:

    pixi run qwiic-eval                      # 10 s capture from every sensor -> report
    pixi run qwiic-eval -- --seconds 30
    pixi run qwiic-eval -- --ab              # baseline capture, change the light, second capture,
                                             #   per-channel change report (LED colour checks)
    pixi run qwiic-eval -- --live            # in-place dashboard with bars, Ctrl-C to stop

The report says which of the expected parts (TSL2591 0x29, VEML7700 0x10,
AS7343 0x39) answered, the achieved sample rate, and per sensor: mean, std,
noise (CV %), min/max, saturated samples, and ADC headroom with a gain
suggestion. The AS7343 spectrum is drawn as bars by centre wavelength. A
cross-check section compares the sensors' uncalibrated lux and IR fractions.

Options --gain/--atime (TSL2591) and --as-gain (AS7343) match qwiic_read.py.
Exit code 0 = every expected sensor present and no saturation, 1 = a sensor
is missing or the firmware lacks the bridge, 2 = saturation during capture.
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
    Sampler,
    flatten,
)

BAR = "█"
BAR_WIDTH = 32


def bar(value, scale, width=BAR_WIDTH) -> str:
    n = 0 if scale <= 0 else int(round(width * min(1.0, value / scale)))
    return BAR * n + "·" * (width - n)


def capture(sampler, seconds, label=None):
    """Return {(tag, metric): [values]} plus per-sensor sat counts and the rate."""
    series, sat = {}, {}
    n, t0 = 0, time.monotonic()
    while time.monotonic() - t0 < seconds:
        for dev, s, r in sampler.round():
            tag = f"{type(s).__name__}[{dev.where()}]"
            flat = flatten(s, r)
            sat[tag] = sat.get(tag, 0) + flat.pop("sat")
            for k, v in flat.items():
                if v is not None:
                    series.setdefault((tag, k), []).append(v)
        n += 1
        if label and sys.stdout.isatty():
            print(f"\r  {label}: {n} rounds, {time.monotonic() - t0:4.1f}/{seconds:.0f} s", end="", flush=True)
    if label and sys.stdout.isatty():
        print("\r" + " " * 60 + "\r", end="")
    return series, sat, n, time.monotonic() - t0


def stats(values):
    m = statistics.mean(values)
    sd = statistics.pstdev(values) if len(values) > 1 else 0.0
    return m, sd, (sd / m * 100 if m else 0.0), min(values), max(values)


def sensor_header(s, settings) -> str:
    if isinstance(s, TSL2591):
        g = settings["tsl_gain"]
        return f"gain {g} (x{TSL2591.GAIN[g][1]:g}), {settings['tsl_atime']} ms, ceiling {36863 if settings['tsl_atime'] == 100 else 65535}"
    if isinstance(s, VEML7700):
        return "gain x1, 100 ms, ceiling 65535"
    return f"gain x{settings['as_gain']:g}, {s.tint_ms:.0f} ms x 3 cycles, full scale {s.full_scale}"


def headroom_line(s, settings, peak_metric, peak) -> str:
    if isinstance(s, TSL2591):
        full = 36863 if settings["tsl_atime"] == 100 else 65535
        order = list(TSL2591.GAIN)
        cur = order.index(settings["tsl_gain"])
        frac = peak / full
        hint = ""
        if frac > 0.9:
            hint = f" -> saturating; use --gain {order[cur - 1]}" if cur > 0 else " -> saturating at lowest gain; shorten --atime"
        elif frac < 0.02 and cur + 1 < len(order):
            hint = f" -> plenty of headroom; --gain {order[cur + 1]} would help resolution"
        return f"  headroom: {peak_metric} at {frac * 100:.1f} % of ceiling{hint}"
    if isinstance(s, VEML7700):
        return f"  headroom: {peak_metric} at {peak / 65535 * 100:.1f} % of ceiling"
    frac = peak / s.full_scale
    gains = sorted(AS7343.GAIN_CODE)
    cur = settings["as_gain"]
    hint = ""
    if frac > 0.9:
        lower = [g for g in gains if g < cur]
        hint = f" -> saturating; use --as-gain {lower[-1]:g}" if lower else " -> saturating at minimum gain"
    elif frac < 0.25:
        target = cur * 0.5 / max(frac, 1e-9)
        higher = [g for g in gains if cur < g <= target]
        if higher:
            hint = f" -> --as-gain {higher[-1]:g} would put the peak near 50 %"
    return f"  headroom: peak {peak_metric} at {frac * 100:.1f} % of full scale{hint}"


def print_report(sampler, series, sat, rounds, elapsed):
    settings = sampler.settings
    any_sat = False
    for dev, s in sampler.sensors:
        tag = f"{type(s).__name__}[{dev.where()}]"
        print(f"\n{tag}  ({sensor_header(s, settings)})")
        print(f"  {'metric':10s} {'mean':>10s} {'std':>8s} {'cv%':>6s} {'min':>9s} {'max':>9s}")
        peak_metric, peak = None, -1
        metrics = [k for (t, k) in series if t == tag]
        for k in metrics:
            m, sd, cv, lo, hi = stats(series[(tag, k)])
            print(f"  {k:10s} {m:10.1f} {sd:8.2f} {cv:6.2f} {lo:9.0f} {hi:9.0f}")
            if k not in ("lux~",) and m > peak:
                peak_metric, peak = k, m
        if isinstance(s, AS7343):
            spec = [(k, statistics.mean(series[(tag, k)])) for k in AS7343.SPECTRAL if (tag, k) in series]
            top = max(v for _, v in spec) or 1
            print("  spectrum (relative to the strongest channel):")
            for k, v in spec:
                print(f"    {k.split('_')[1]:>3s} nm  {bar(v, top)} {v:7.0f}")
        if peak_metric is not None:
            print(headroom_line(s, settings, peak_metric, peak))
        if sat.get(tag):
            any_sat = True
            print(f"  !! {sat[tag]} of {rounds} samples saturated")
    return any_sat


def cross_check(sampler, series):
    tags = {type(s).__name__: f"{type(s).__name__}[{dev.where()}]" for dev, s in sampler.sensors}

    def mean(name, k):
        key = (tags.get(name), k)
        return statistics.mean(series[key]) if key in series else None

    lines = []
    tl, vl = mean("TSL2591", "lux~"), mean("VEML7700", "lux~")
    if tl and vl:
        lines.append(f"  TSL2591 lux / VEML7700 lux     = {tl / vl:5.2f}   (uncalibrated; depends on geometry + spectrum)")
    tf, ti = mean("TSL2591", "full"), mean("TSL2591", "ir")
    if tf:
        lines.append(f"  TSL2591 IR fraction (ir/full)  = {ti / tf:5.2f}   (~0.2-0.3 LED/fluorescent, >0.5 incandescent/daylight)")
    nir, vis = mean("AS7343", "NIR_855"), mean("AS7343", "VIS")
    if nir is not None and vis:
        lines.append(f"  AS7343 NIR_855 / VIS           = {nir / vis:5.2f}")
    red, blue = mean("AS7343", "F6_640"), mean("AS7343", "FZ_450")
    if red is not None and blue:
        lines.append(f"  AS7343 F6_640 / FZ_450         = {red / blue:5.2f}   (red:blue balance)")
    if lines:
        print("\nCross-check")
        print("\n".join(lines))


def run_capture(sampler, seconds, label):
    series, sat, rounds, elapsed = capture(sampler, seconds, label)
    return series, sat, rounds, elapsed


def mode_report(sampler, args):
    print(f"\nCapturing {args.seconds:.0f} s ...")
    series, sat, rounds, elapsed = run_capture(sampler, args.seconds, "capture")
    print(f"Capture: {rounds} rounds in {elapsed:.1f} s ({rounds / elapsed:.1f} Hz; "
          f"round time set by the slowest integration, {sampler.period_s * 1000:.0f} ms)")
    any_sat = print_report(sampler, series, sat, rounds, elapsed)
    cross_check(sampler, series)
    return any_sat


def mode_ab(sampler, args):
    print(f"\nA: baseline — leave the light as it is. Capturing {args.seconds:.0f} s ...")
    a, sat_a, n_a, _ = run_capture(sampler, args.seconds, "A")
    input("\nB: change the light (cover the sensors, switch the arena pattern/colour, ...) then press Enter ")
    print(f"Capturing {args.seconds:.0f} s ...")
    b, sat_b, n_b, _ = run_capture(sampler, args.seconds, "B")
    any_sat = False
    for dev, s in sampler.sensors:
        tag = f"{type(s).__name__}[{dev.where()}]"
        print(f"\n{tag}")
        print(f"  {'metric':10s} {'A mean':>10s} {'B mean':>10s} {'B/A':>7s}   change")
        rows = [k for (t, k) in a if t == tag]
        ratios = {}
        for k in rows:
            ma = statistics.mean(a[(tag, k)])
            mb = statistics.mean(b[(tag, k)]) if (tag, k) in b else float("nan")
            ratio = mb / ma if ma else float("inf")
            ratios[k] = ratio
            print(f"  {k:10s} {ma:10.1f} {mb:10.1f} {ratio:7.3f}   {bar(min(ratio, 4.0), 4.0, 16) if ratio == ratio else ''}")
        if isinstance(s, AS7343):
            spec = [(k, ratios[k]) for k in AS7343.SPECTRAL if k in ratios]
            hi = max(spec, key=lambda kv: kv[1])
            lo = min(spec, key=lambda kv: kv[1])
            print(f"  most increased: {hi[0]} (x{hi[1]:.2f});  least: {lo[0]} (x{lo[1]:.2f})")
        if sat_a.get(tag) or sat_b.get(tag):
            any_sat = True
            print(f"  !! saturated samples: A {sat_a.get(tag, 0)}/{n_a}, B {sat_b.get(tag, 0)}/{n_b}")
    return any_sat


def mode_live(sampler, args):
    t0 = time.monotonic()
    n = 0
    try:
        while not args.seconds or time.monotonic() - t0 < args.seconds:
            rows = sampler.round()
            n += 1
            out = [f"Qwiic live  {time.strftime('%H:%M:%S')}  round {n}  ({n / (time.monotonic() - t0 + 1e-9):.1f} Hz)  Ctrl-C to stop\n"]
            for dev, s, r in rows:
                tag = f"{type(s).__name__}[{dev.where()}]"
                flat = flatten(s, r)
                sat = flat.pop("sat")
                out.append(f"{tag}{'   SAT' if sat else ''}")
                if isinstance(s, AS7343):
                    top = max(r["spectral"].values()) or 1
                    for k, v in r["spectral"].items():
                        out.append(f"  {k.split('_')[1]:>3s} nm  {bar(v, top)} {v:6d}")
                    out.append(f"  VIS {r['vis']:6.0f}   full scale {s.full_scale}")
                elif isinstance(s, TSL2591):
                    ceil = 36863 if sampler.settings["tsl_atime"] == 100 else 65535
                    out.append(f"  full {bar(flat['full'], ceil)} {flat['full']:6d}")
                    out.append(f"  ir   {bar(flat['ir'], ceil)} {flat['ir']:6d}")
                    out.append(f"  lux~ {flat['lux~']:.1f}" if flat['lux~'] is not None else "  lux~ n/a (saturated)")
                else:
                    out.append(f"  als   {bar(flat['als'], 65535)} {flat['als']:6d}")
                    out.append(f"  white {bar(flat['white'], 65535)} {flat['white']:6d}")
                    out.append(f"  lux~ {flat['lux~']:.1f}")
                out.append("")
            sys.stdout.write("\x1b[H\x1b[2J" + "\n".join(out))
            sys.stdout.flush()
    except KeyboardInterrupt:
        print()
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    ap.add_argument("--ip")
    ap.add_argument("--seconds", type=float, default=10.0, help="capture length per phase (default 10; live: 0 = until Ctrl-C)")
    ap.add_argument("--ab", action="store_true", help="two-phase A/B comparison")
    ap.add_argument("--live", action="store_true", help="in-place dashboard")
    ap.add_argument("--gain", choices=list(TSL2591.GAIN), default="med", help="TSL2591 gain (default med)")
    ap.add_argument("--atime", type=int, choices=list(TSL2591.ATIME), default=100, help="TSL2591 integration ms")
    ap.add_argument("--as-gain", type=float, choices=list(AS7343.GAIN_CODE), default=64, help="AS7343 gain (default 64)")
    ap.add_argument("--allow-missing", action="store_true", help="evaluate whatever is present instead of requiring all three")
    args = ap.parse_args()
    if args.live and args.seconds == 10.0:
        args.seconds = 0

    t, link = open_transport(args)
    try:
        try:
            sampler = Sampler(t, tsl_gain=args.gain, tsl_atime=args.atime, as_gain=args.as_gain)
        except NoQwiicJack as e:
            sys.exit(str(e))
        except RuntimeError as e:
            sys.exit(f"{e} — firmware lacks the Qwiic bridge (0xB0/0xB1)?")

        print(f"Qwiic sensor evaluation on {link}")
        found = ", ".join(f"{type(s).__name__} 0x{d.addr:02X} @ {d.where()}" for d, s in sampler.sensors) or "none"
        print(f"Sensors: {found}")
        missing = sampler.missing()
        if missing:
            print(f"Missing: {', '.join(missing)}")
            if not args.allow_missing:
                sys.exit("expected all three LAB-211 sensors on the chain (use --allow-missing to evaluate a subset)")
        if not sampler.sensors:
            sys.exit(1)

        try:
            if args.live:
                any_sat = mode_live(sampler, args)
            elif args.ab:
                any_sat = mode_ab(sampler, args)
            else:
                any_sat = mode_report(sampler, args)
        finally:
            sampler.close()

        print()
        if any_sat:
            print("Result: sensors respond, but some samples saturated — see headroom hints")
            sys.exit(2)
        print("Result: all sensors present and reading within range")
    finally:
        t.close()


if __name__ == "__main__":
    main()
