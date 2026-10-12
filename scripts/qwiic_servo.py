"""PCA9685 servo check over the Qwiic bridge (SparkFun Servo pHAT DEV-15316 or
Adafruit 815 at 0x40). Host-side "go to position": a 4-byte register write.

    pixi run qwiic-servo                              # identify, set 50 Hz, report; outputs untouched
    pixi run qwiic-servo -- --channel 0 --us 1500     # center one servo and hold
    pixi run qwiic-servo -- --channel 0 --sweep       # 1000 -> 2000 -> 1000 us in 100 us steps, then off
    pixi run qwiic-servo -- --all-off                 # release every channel

POWER: the servo rail (V+) must come from the driver board's own supply (the
pHAT's USB-C, the Adafruit board's screw terminal), never from the Qwiic
3.3 V -- that is the Teensy's regulator. Only the logic side is on the bus.

The PCA9685 also answers the all-call address 0x70 (reads included) until
MODE1.ALLCALL is cleared; configure() clears it so a PCA9548 mux at 0x70 can
coexist. The pHAT's logic side is NOT powered from the Qwiic 3.3 V: without
its USB-C supply the board is simply absent from the bus. Bench-verified
2026-10-12 on the pHAT with a micro servo on ch0 (prescale, pulse read-back,
full-OFF, ALLCALL off, --sweep moves the arm; ~49 mA idle / ~75 mA peak on the
pHAT's own supply) -- docs/development/qwiic-bench-2026-10-12.md.
"""

import argparse
import sys
import time

from qwiic_probe import open_transport  # noqa: F401  (adds repo root to sys.path)
from tests.qwiic_peripherals import PCA9685  # noqa: E402
from tests.qwiic_sensors import NoQwiicJack, i2c_scan  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    ap.add_argument("--ip")
    ap.add_argument("--addr", type=lambda x: int(x, 0), default=PCA9685.ADDR)
    ap.add_argument("--freq", type=float, default=50.0, help="PWM frame rate, Hz (default 50)")
    ap.add_argument("--channel", type=int, help="servo channel 0..15")
    ap.add_argument("--us", type=float, help="pulse width to set and hold, microseconds")
    ap.add_argument("--sweep", action="store_true", help="sweep 1000..2000 us and back, then release")
    ap.add_argument("--all-off", action="store_true", help="release every channel")
    args = ap.parse_args()

    t, link = open_transport(args)
    try:
        try:
            scan = i2c_scan(t)
        except NoQwiicJack as e:
            sys.exit(str(e))
        if args.addr not in scan:
            sys.exit(f"no device at 0x{args.addr:02X} on the Qwiic bus ({link}); scan: "
                     + (", ".join(f"0x{a:02X}" for a in scan) or "nothing"))
        pwm = PCA9685(t, args.addr)
        ident = pwm.identify()
        print(f"PCA9685 @ 0x{args.addr:02X}: MODE1 0x{ident['mode1']:02X} prescale {ident['prescale']}"
              f" ({ident['freq_hz']:.1f} Hz){' SLEEP' if ident['sleeping'] else ''}{' ALLCALL' if ident['allcall'] else ''}")
        if args.all_off:
            pwm.all_off()
            print("all channels released")
            return
        if args.channel is None:
            print("no --channel given; outputs untouched")
            return
        pwm.configure(args.freq)
        ident = pwm.identify()
        print(f"configured: {pwm.freq_hz:.1f} Hz, MODE1 0x{ident['mode1']:02X}"
              f"{' (all-call still on!)' if ident['allcall'] else ', all-call off'}")
        if args.sweep:
            steps = list(range(1000, 2001, 100)) + list(range(1900, 999, -100))
            for us in steps:
                off = pwm.set_pulse_us(args.channel, us)
                on_rb, off_rb = pwm.read_counts(args.channel)
                print(f"  ch{args.channel} {us:4d} us -> OFF count {off:4d} (read back {off_rb})")
                time.sleep(0.3)
            pwm.set_off(args.channel)
            print(f"  ch{args.channel} released")
        elif args.us is not None:
            off = pwm.set_pulse_us(args.channel, args.us)
            on_rb, off_rb = pwm.read_counts(args.channel)
            print(f"  ch{args.channel} {args.us:.0f} us -> OFF count {off} (read back {off_rb}); holding")
    finally:
        t.close()


if __name__ == "__main__":
    main()
