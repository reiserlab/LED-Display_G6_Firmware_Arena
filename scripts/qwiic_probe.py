"""Bench probe for the Qwiic / STEMMA QT jack (J2) on arena_12-18.

Scans the controller's Wire1 bus through GET_I2C_SCAN (0xA6), maps any
PCA9548 mux channels, identifies the LAB-211 sensors (AS7343, TSL2591,
VEML7700) and takes one raw light reading from each. Optionally repeats the
readings so you can wave a hand / toggle the arena LEDs and watch them move.

    pixi run qwiic-probe                         # auto-detect the USB-CDC port
    pixi run qwiic-probe -- --port /dev/cu.usbmodem1234
    pixi run qwiic-probe -- --ip 10.0.0.x        # over TCP
    pixi run qwiic-probe -- --loop 20 --interval 0.5

Exit codes: 0 all identified sensors OK, 1 no controller / firmware lacks
0xA6 / no Qwiic jack, 2 a sensor answered but failed its ID check.
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tests.commands import GET_CONTROLLER_INFO_CMD  # noqa: E402
from tests.qwiic_sensors import (  # noqa: E402
    AS7343,
    MCP4725_ADDR,
    TSL2591,
    VEML7700,
    I2CError,
    NoQwiicJack,
    describe,
    i2c_scan,
    is_mux,
    locate_devices,
    reach,
)
from tests.transport import SerialTransport, TcpTransport  # noqa: E402


def find_ports() -> list[str]:
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    named = [p.device for p in ports
             if any(s and ("Arena" in s or "Reiser" in s)
                    for s in (p.manufacturer, p.product, p.description))]
    if named:
        return named
    return [p.device for p in ports if p.vid == 0x16C0]  # PJRC


def open_transport(args):
    if args.ip:
        t = TcpTransport(args.ip)
        t.open()
        return t, f"tcp {args.ip}"
    port = args.port
    if not port:
        ports = find_ports()
        if not ports:
            sys.exit("no Arena USB-CDC port found — pass --port or --ip")
        if len(ports) > 1:
            print(f"several candidate ports {ports}; using {ports[0]}")
        port = ports[0]
    t = SerialTransport(port)
    t.open()
    return t, f"serial {port}"


def identify_all(t, devices) -> tuple[list, list[str]]:
    """Returns (sensors, failures): sensors = [(Located, driver)]."""
    sensors, failures = [], []
    for dev in devices:
        cls = {VEML7700.ADDR: VEML7700, TSL2591.ADDR: TSL2591, AS7343.ADDR: AS7343}.get(dev.addr)
        if cls is None:
            continue
        with reach(t, dev):
            s = cls(t)
            try:
                ident = s.identify()
            except I2CError as e:
                failures.append(f"{cls.__name__} @ {dev.where()}: {e}")
                continue
            if isinstance(s, TSL2591):
                ok = ident["id_ok"]
                detail = f"ID 0x{ident['id']:02X} (expect 0x50)"
            elif isinstance(s, AS7343):
                ok = ident["id_ok"]
                detail = (f"ID 0x{ident['id']:02X} (expect 0x81) rev 0x{ident['revid']:02X}"
                          f" aux 0x{ident['auxid']:02X}")
            else:
                ok = True  # early VEML7700 silicon has no ID register
                detail = (f"ID reg 0x{ident['id_raw']:04X}"
                          + (" ok" if ident["device_id_ok"] else " (0x81 expected on current silicon)"))
            print(f"  {cls.__name__:9s} @ {dev.where():16s} {'OK  ' if ok else 'FAIL'} {detail}")
            if ok:
                sensors.append((dev, s))
            else:
                failures.append(f"{cls.__name__} @ {dev.where()}: {detail}")
    return sensors, failures


def configure_all(t, sensors):
    for dev, s in sensors:
        with reach(t, dev):
            if isinstance(s, TSL2591):
                s.configure(gain="med", atime_ms=100)
            elif isinstance(s, VEML7700):
                s.configure()
            elif isinstance(s, AS7343):
                if not s.enable_roundtrip():
                    print(f"  AS7343 @ {dev.where()}: ENABLE.PON did not read back")
                s.configure(gain=64)


def read_all(t, sensors) -> list[str]:
    cells = []
    for dev, s in sensors:
        with reach(t, dev):
            if isinstance(s, TSL2591):
                r = s.read()
                lux = f"{r['lux_approx']:.1f} lx" if r["lux_approx"] is not None else "sat/0"
                cells.append(f"TSL2591[{dev.where()}] full={r['full']:5d} ir={r['ir']:5d} ~{lux}")
            elif isinstance(s, VEML7700):
                r = s.read()
                cells.append(f"VEML7700[{dev.where()}] als={r['als']:5d} white={r['white']:5d}"
                             f" ~{r['lux_approx']:.1f} lx")
            elif isinstance(s, AS7343):
                r = s.read()
                cells.append(f"AS7343[{dev.where()}] gain64 "
                             + " ".join(f"{k.split('_')[1]}nm={v}" for k, v in r["spectral"].items())
                             + f" VIS={r['vis']:.0f}"
                             + ("  SAT" if r["sat"] else ""))
    return cells


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="USB-CDC device (auto-detected if omitted)")
    ap.add_argument("--ip", help="controller IP for TCP instead of serial")
    ap.add_argument("--loop", type=int, default=1, help="number of reading rounds (default 1)")
    ap.add_argument("--interval", type=float, default=0.5, help="seconds between rounds")
    args = ap.parse_args()

    t, link = open_transport(args)
    try:
        st, _, payload, _ = t.command(GET_CONTROLLER_INFO_CMD)
        if st != 0 or len(payload) < 2:
            sys.exit(f"GET_CONTROLLER_INFO failed over {link}: status={st}")
        print(f"controller on {link}: protocol v{payload[0]} capabilities 0x{payload[1]:02X}")

        try:
            root = i2c_scan(t)
        except NoQwiicJack as e:
            sys.exit(f"{e} — flash the arena_12-18 build (pixi run deploy-12-18)")
        except RuntimeError as e:
            sys.exit(f"{e} — firmware predates the Qwiic bridge (0xA6/0xA7)?")

        print(f"\nQwiic bus scan ({len(root)} device{'s' if len(root) != 1 else ''}):")
        for a in root:
            print(f"  0x{a:02X}  {describe(a)}")
        if not root:
            print("  (nothing ACKed — check the cable, sensor power LED, and jack orientation)")
        if MCP4725_ADDR in root:
            print("  !! 0x60 is the AO DAC: the bridge is on Wire, not the Qwiic Wire1 bus")

        devices = locate_devices(t)
        behind = [d for d in devices if d.mux is not None]
        if any(is_mux(a) for a in root):
            print("\nPCA9548 channel map:")
            for mux in sorted({d.mux for d in behind} | {a for a in root if is_mux(a)}):
                for ch in range(8):
                    on_ch = [d.addr for d in behind if d.mux == mux and d.channel == ch]
                    if on_ch:
                        print(f"  0x{mux:02X} ch{ch}: " + ", ".join(f"0x{a:02X} {describe(a)}" for a in on_ch))

        print("\nIdentify:")
        sensors, failures = identify_all(t, devices)
        if not sensors and not failures:
            print("  no LAB-211 sensors found")

        if sensors:
            configure_all(t, sensors)
            print("\nReadings:")
            for i in range(args.loop):
                stamp = time.strftime("%H:%M:%S")
                for cell in read_all(t, sensors):
                    print(f"  {stamp}  {cell}")
                if i + 1 < args.loop:
                    time.sleep(args.interval)
            for dev, s in sensors:
                if isinstance(s, (TSL2591, AS7343)):
                    with reach(t, dev):
                        s.power_off()

        print()
        if failures:
            print("FAIL:", *failures, sep="\n  ")
            sys.exit(2)
        print(f"I2C path OK: {len(root)} device(s) on the jack, {len(sensors)} sensor(s) identified")
    finally:
        t.close()


if __name__ == "__main__":
    main()
