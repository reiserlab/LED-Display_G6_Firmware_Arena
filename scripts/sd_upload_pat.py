#!/usr/bin/env python3
"""Upload a .pat to the controller's SD card (browser-free) and print its 1-based index.

    python3 scripts/sd_upload_pat.py --port /dev/cu.usbmodemXXXX --file soak-patterns/sine_2000f_gs16.pat --name sine_2000f_gs16.pat

Stops the display first (SD writes are refused while displaying), uploads to the staging slot (0x85 idx 0),
promotes it with 0x83, then reads GET_PATTERN_INFO 0x88 back and prints "index frames". Exit 1 on any failure.
"""
import argparse, os, struct, sys, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from tests.commands import GET_PATTERN_INFO_CMD, STOP_DISPLAY_CMD  # noqa: E402
from tests.transport import SerialTransport  # noqa: E402

p = argparse.ArgumentParser()
p.add_argument("--port", required=True); p.add_argument("--file", required=True); p.add_argument("--name", required=True)
p.add_argument("--timeout", type=float, default=600.0)
a = p.parse_args()
data = open(a.file, "rb").read()
t = SerialTransport(a.port); t.open()
try:
    st, _, _, _ = t.command(STOP_DISPLAY_CMD, timeout=2.0)
    t0 = time.perf_counter()
    st, echo, _, _ = t.upload_file(0, data, timeout=a.timeout)
    dt = time.perf_counter() - t0
    if st != 0:
        print(f"upload failed status={st}", file=sys.stderr); sys.exit(1)
    st2, _, payload, _ = t.rename_file(0, a.name)
    if st2 != 0:
        print(f"rename failed status={st2}", file=sys.stderr); sys.exit(1)
    idx = struct.unpack_from("<H", bytes(payload[:2]))[0]
    st3, _, pip, _ = t.command(GET_PATTERN_INFO_CMD, struct.pack("<H", idx), timeout=5.0)
    frames = struct.unpack_from("<H", bytes(pip))[0] if st3 == 0 and len(pip) >= 2 else -1
    print(f"{idx} {frames}  ({len(data)} B in {dt:.1f} s = {len(data)/dt/1e6:.2f} MB/s)")
finally:
    t.close()
