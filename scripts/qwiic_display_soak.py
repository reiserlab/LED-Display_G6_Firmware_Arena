"""Does Qwiic sensor traffic disturb a running display? Three 10 s phases with
the controller in ALL_ON (the refresh timer pushes a frame to the panel bus
every tick, panels or not): (A) display alone, (B) display + the bus soak
(every attached device, back-to-back read-only transfers -- far harsher than
any logger), (C) display + a realistic 1 Hz environmental poll (SHT4x read,
IMU read + bump flag). Each phase reports frames pushed vs. expected from the
refresh rate, and the controller's worst 1 s loop gap.

    pixi run qwiic-display-soak                          # auto-detect port
    pixi run qwiic-display-soak -- --seconds 20          # longer phases

Leaves the display stopped (ALL_OFF). LAB-214 bench, 2026-10-12.
"""

import argparse
import statistics
import struct
import sys
import time

from qwiic_probe import open_transport  # noqa: F401  (adds repo root to sys.path)
from tests.commands import ALL_ON_CMD, GET_FRAMES_SENT_CMD, STOP_DISPLAY_CMD  # noqa: E402
from tests.qwiic_peripherals import LSM6DSOX, SHT4x, soak_transfer  # noqa: E402
from tests.qwiic_sensors import I2CError, NoQwiicJack, locate_devices, reach  # noqa: E402
from tests.test_health import read_health  # noqa: E402


def frames_sent(t) -> int:
    st, _, payload, _ = t.command(GET_FRAMES_SENT_CMD)
    assert st == 0
    return struct.unpack("<I", bytes(payload[:4]))[0]


def phase(t, label, seconds, work):
    """Run `work()` repeatedly for `seconds`; sample health once a second."""
    time.sleep(1.2)                              # let the previous phase leave the 1 s window
    f0, t0 = frames_sent(t), time.monotonic()
    loop_max, errors, calls = [], 0, 0
    next_health = t0 + 1.0
    while time.monotonic() - t0 < seconds:
        try:
            work()
            calls += 1
        except I2CError:
            errors += 1
        if time.monotonic() >= next_health:
            loop_max.append(read_health(t).loop_max_1s_us)
            next_health += 1.0
    f1, t1 = frames_sent(t), time.monotonic()
    return {"label": label, "frames": f1 - f0, "seconds": t1 - t0, "fps": (f1 - f0) / (t1 - t0),
            "loop_max_1s_median": int(statistics.median(loop_max)) if loop_max else None,
            "loop_max_1s_max": max(loop_max) if loop_max else None, "calls": calls, "errors": errors}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    ap.add_argument("--ip")
    ap.add_argument("--seconds", type=float, default=10.0, help="length of each phase (default 10)")
    args = ap.parse_args()

    t, link = open_transport(args)
    try:
        try:
            devices = locate_devices(t)
        except NoQwiicJack as e:
            sys.exit(str(e))
        if not devices:
            sys.exit("nothing on the Qwiic bus")
        sht = [d for d in devices if d.addr == SHT4x.ADDR]
        imu = [d for d in devices if d.addr in (LSM6DSOX.ADDR, LSM6DSOX.ADDR_ALT)]
        imu_drv = LSM6DSOX(t, imu[0].addr) if imu else None
        if imu_drv:
            with reach(t, imu[0]):
                imu_drv.configure(odr_hz=104)
                imu_drv.arm_bump(0.25)

        st, _, _, _ = t.command(ALL_ON_CMD)
        assert st == 0, "ALL_ON refused"
        time.sleep(0.5)
        print(f"# {link}: {len(devices)} device(s); display ALL_ON, phases of {args.seconds:g} s")

        def idle():
            time.sleep(0.05)

        def soak():
            for d in devices:
                soak_transfer(t, d, 0)

        def env_poll():
            t_next = time.monotonic() + 1.0
            for d in sht:
                with reach(t, d):
                    SHT4x(t).read()
            if imu_drv:
                with reach(t, imu[0]):
                    imu_drv.read()
                    imu_drv.bump_flag()
            time.sleep(max(0.0, t_next - time.monotonic()))

        results = [phase(t, "A display alone", args.seconds, idle),
                   phase(t, "B display + bus soak", args.seconds, soak),
                   phase(t, "C display + 1 Hz env poll", args.seconds, env_poll)]
        t.command(STOP_DISPLAY_CMD)
        if imu_drv:
            with reach(t, imu[0]):
                imu_drv.power_off()

        base = results[0]["fps"]
        print(f"\n{'phase':28s} {'frames':>7s} {'fps':>8s} {'vs A':>7s} {'loop_max_1s med/max us':>24s} {'transfers':>10s} {'err':>4s}")
        for r in results:
            print(f"{r['label']:28s} {r['frames']:7d} {r['fps']:8.2f} {r['fps'] / base * 100:6.2f}%"
                  f" {str(r['loop_max_1s_median']):>11s}/{str(r['loop_max_1s_max']):<12s} {r['calls']:10d} {r['errors']:4d}")
        worst = min(r["fps"] for r in results) / base
        print(f"\n# frame rate held to {worst * 100:.2f} % of the display-alone rate"
              + (" — no frames lost to sensor traffic" if worst > 0.995 else " — frames LOST, investigate"))
        sys.exit(0 if worst > 0.995 and all(r["errors"] == 0 for r in results) else 1)
    finally:
        try:
            t.command(STOP_DISPLAY_CMD)
        finally:
            t.close()


if __name__ == "__main__":
    main()
