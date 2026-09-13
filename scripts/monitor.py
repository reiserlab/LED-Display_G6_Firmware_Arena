"""Cross-platform USB serial monitor that also logs to log/<timestamp>_<variant>.log.

Wraps `pio device monitor` so the monitor-* pixi tasks work the same on Linux,
macOS and Windows (no bash, tee or date needed), and finds the arena's serial
port by its USB descriptor rather than a Linux-only /dev/serial/by-id glob.

    pixi run monitor-10-10                  # auto-detect the arena
    pixi run monitor-10-10 -- --port COM5   # explicit port
    pixi run monitor-12-18 -- --filter time # anything else is passed to pio

Port detection is shared with the pytest conftests, see scripts/arena_port.py.
"""

import argparse
import datetime
import os
import subprocess
import sys

from arena_port import describe_usb_ports, find_arena_port


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("-e", "--environment", required=True, help="platformio.ini env")
    ap.add_argument("-p", "--port", help="serial port; auto-detected when omitted")
    args, passthrough = ap.parse_known_args()

    port = args.port or find_arena_port()
    if not port:
        sys.exit(
            "monitor.py: no G6 arena found; pass --port <device>. "
            f"USB serial ports seen: {describe_usb_ports()}"
        )

    variant = args.environment.removeprefix("teensy41-")
    os.makedirs("log", exist_ok=True)
    log_path = os.path.join("log", f"{datetime.datetime.now():%Y%m%d_%H%M%S}_{variant}.log")

    cmd = [sys.executable, "-m", "platformio", "device", "monitor",
           "-e", args.environment, "--port", port, *passthrough]
    print(f"monitor.py: port {port}, logging to {log_path}", flush=True)

    with open(log_path, "wb") as log, subprocess.Popen(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT
    ) as proc:
        try:
            for chunk in iter(lambda: proc.stdout.read1(4096), b""):
                sys.stdout.buffer.write(chunk)
                sys.stdout.buffer.flush()
                log.write(chunk)
                log.flush()
        except KeyboardInterrupt:
            pass
        return proc.wait()


if __name__ == "__main__":
    sys.exit(main())
