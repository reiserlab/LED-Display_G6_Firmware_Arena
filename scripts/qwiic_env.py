"""Environmental / tilt / bump readout from the LAB-214 peripherals on the
Qwiic jack -- one line per sample (default 1 Hz) until Ctrl-C, then stats.
This is the host-side shape of "environmental monitoring" from LAB-214: the
same transfers a Studio trial log would issue at trial start / end.

    pixi run qwiic-env                               # auto-detect port, 1 Hz
    pixi run qwiic-env -- --interval 0.2 --count 50  # faster, fixed length
    pixi run qwiic-env -- --bump-g 0.1               # more sensitive bump latch (default 0.25 g)
    pixi run qwiic-env -- --csv > env.csv            # machine-readable

Reads every SHT4x (temperature, humidity) and LSM6DSOX (tilt from the gravity
vector, latched "bumped since the last line" flag, die temperature) found on
the root bus or behind a PCA9548. Tap the IMU breakout between lines to see
the bump flag set once and clear itself on the next line.
"""

import argparse
import statistics
import sys
import time

from qwiic_probe import open_transport  # noqa: F401  (adds repo root to sys.path)
from tests.qwiic_peripherals import LSM6DSOX, SHT4x  # noqa: E402
from tests.qwiic_sensors import I2CError, NoQwiicJack, locate_devices, reach  # noqa: E402


def stamp_now() -> str:
    return time.strftime("%H:%M:%S") + f".{int(time.time() * 1000) % 1000:03d}"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    ap.add_argument("--ip")
    ap.add_argument("--interval", type=float, default=1.0, help="seconds between samples (default 1)")
    ap.add_argument("--count", type=int, default=0, help="stop after N samples (default: until Ctrl-C)")
    ap.add_argument("--bump-g", type=float, default=0.25, help="wake-up threshold in g (default 0.25)")
    ap.add_argument("--csv", action="store_true")
    args = ap.parse_args()

    t, link = open_transport(args)
    try:
        try:
            devices = locate_devices(t)
        except NoQwiicJack as e:
            sys.exit(str(e))
        sensors = []
        for dev in devices:
            if dev.addr == SHT4x.ADDR:
                s = SHT4x(t)
                with reach(t, dev):
                    ident = s.identify()
                sensors.append((dev, s, f"SHT4x[{dev.where()}]"))
                print(f"# {sensors[-1][2]} serial 0x{ident['serial']:08X}", file=sys.stderr)
            elif dev.addr in (LSM6DSOX.ADDR, LSM6DSOX.ADDR_ALT):
                s = LSM6DSOX(t, dev.addr)
                with reach(t, dev):
                    ident = s.identify()
                    if not ident["id_ok"]:
                        print(f"# 0x{dev.addr:02X} @ {dev.where()}: WHO_AM_I 0x{ident['who_am_i']:02X} != 0x6C, skipped",
                              file=sys.stderr)
                        continue
                    s.configure(odr_hz=104)
                    s.arm_bump(args.bump_g)
                sensors.append((dev, s, f"LSM6DSOX[{dev.where()}]"))
                print(f"# {sensors[-1][2]} WHO_AM_I 0x6C, bump latch armed at {args.bump_g:g} g", file=sys.stderr)
        if not sensors:
            sys.exit(f"no SHT4x / LSM6DSOX on the Qwiic bus ({link}); found "
                     + (", ".join(f"0x{d.addr:02X}" for d in devices) or "nothing"))

        columns = []
        for _, s, tag in sensors:
            if isinstance(s, SHT4x):
                columns += [f"{tag}.temp_c", f"{tag}.rh_pct"]
            else:
                columns += [f"{tag}.from_vertical_deg", f"{tag}.pitch_deg", f"{tag}.roll_deg",
                            f"{tag}.accel_norm_g", f"{tag}.bumped", f"{tag}.temp_c"]
        history = {c: [] for c in columns}
        if args.csv:
            print("time," + ",".join(columns))
        else:
            print(f"# {link}; every {args.interval:g} s; Ctrl-C for stats", file=sys.stderr)

        n = 0
        bumps = 0
        t_first = time.monotonic()
        try:
            while not args.count or n < args.count:
                t0 = time.monotonic()
                values, cells = [], []
                for dev, s, tag in sensors:
                    width = 2 if isinstance(s, SHT4x) else 6
                    try:
                        with reach(t, dev):
                            if isinstance(s, SHT4x):
                                r = s.read()
                                values += [r["temp_c"], r["rh_pct"]]
                                cells.append(f"{tag} {r['temp_c']:6.2f} C {r['rh_pct']:5.1f} %RH" + ("" if r["crc_ok"] else " CRC!"))
                            else:
                                r = s.read()
                                b = s.bump_flag()
                                bumps += b["bumped"]
                                values += [r["from_vertical_deg"], r["pitch_deg"], r["roll_deg"],
                                           r["accel_norm_g"], int(b["bumped"]), r["temp_c"]]
                                cells.append(f"{tag} tilt {r['from_vertical_deg']:5.1f} deg (pitch {r['pitch_deg']:+6.1f}"
                                             f" roll {r['roll_deg']:+6.1f}) |a| {r['accel_norm_g']:.3f} g"
                                             + (f"  BUMPED[{b['axes']}]" if b["bumped"] else ""))
                    except I2CError as e:
                        # One bad transfer must not end a logging run: blank cells, note, carry on.
                        values += [None] * width
                        cells.append(f"{tag} ERROR {e}")
                        print(f"# {stamp_now()} {tag}: {e}", file=sys.stderr)
                stamp = stamp_now()
                for c, v in zip(columns, values):
                    if v is not None:
                        history[c].append(v)
                if args.csv:
                    print(stamp + "," + ",".join("" if v is None else f"{v:.3f}" if isinstance(v, float) else str(v)
                                                 for v in values), flush=True)
                else:
                    print(f"{stamp}  " + "   ".join(cells), flush=True)
                n += 1
                wait = args.interval - (time.monotonic() - t0)
                if wait > 0 and (not args.count or n < args.count):
                    time.sleep(wait)
        except KeyboardInterrupt:
            pass
        finally:
            elapsed = time.monotonic() - t_first
            for dev, s, _ in sensors:
                if isinstance(s, LSM6DSOX):
                    with reach(t, dev):
                        s.power_off()
        if n:
            print(f"\n# {n} samples in {elapsed:.1f} s; {bumps} bump event(s)", file=sys.stderr)
            for c in columns:
                h = history[c]
                if c.endswith(".bumped") or not h:
                    continue
                print(f"#   {c:36s} min {min(h):8.3f}  max {max(h):8.3f}  mean {statistics.mean(h):8.3f}"
                      f"  sd {statistics.pstdev(h):7.3f}", file=sys.stderr)
    finally:
        t.close()


if __name__ == "__main__":
    main()
