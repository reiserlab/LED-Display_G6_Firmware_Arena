"""Measure the analog voltage on the shared MISO_B0 bus node (far side of R192).

Purpose: distinguish wired-OR *contention* from a clean signal or an open on the
arena's MISO_B0 bus. U96 pin 4 drives a clean 0/3.3 V `01 30 62`, but Teensy
pin 12 reads 00. If multiple B0 buffers are enabled at once, their push-pull
outputs fight through their 33 Ω series resistors and the shared node settles to
a *divider* level (e.g. ~0.66 V for one '1' vs four '0's) — below logic
threshold, so it decodes as 0. A logic analyzer can't see that; an oscilloscope
can. This captures the shared node with the AD3 *scope* and reports the voltage
the data '1' bits actually reach.

Interpretation of the reported node-high level:
  - ~3.0-3.3 V  -> clean full swing. NOT contention; the loss is an OPEN
                  between this node and Teensy pin 12 (trace continuity).
  - ~0.4-1.5 V  -> CONTENTION: other enabled buffers are pulling the bus down.
  - ~0 V (flat) -> node hard-pulled low (stuck driver / short / pulldown).

Wiring — KEEP your existing logic hookup (DIO0=SCK, DIO1=MOSI, DIO2=MISO,
DIO3=CS); this script doesn't use them but they don't interfere. ADD the two
AD3 *scope* (analog) channels:

    Scope 1+  -> far side of R192  (the shared MISO_B0 node; the R192 end that
                 is NOT on U96 pin 4)
    Scope 1-  -> GND
    Scope 2+  -> the CS signal you already trigger on (same net as DIO3)
    Scope 2-  -> GND

Run (with All-On streaming from the controller):
    pixi run -e debugad3 python debug/measure_shared_node.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import dwfpy as dwf
import numpy as np

CH_NODE = 0          # Scope CH1 -> shared MISO_B0 node (R192 far side)
CH_CS = 1            # Scope CH2 -> CS (trigger reference)

SAMPLE_RATE = 100e6  # 10 samples/bit at 10 MHz SCK; well within the AD3's 125 MS/s
BUFFER_SIZE = 16384  # ~164 us window — easily spans the 3-byte (~2.4 us) reply
RANGE_V = 5.0        # ±range covering a 0-3.3 V logic signal
TRIG_LEVEL_V = 1.6   # CS mid-threshold
LOGIC_THRESH_V = 1.6 # 3.3 V-logic high threshold, for the pass/fail verdict

OUTPUT_FILE = Path(__file__).with_name("shared_node_capture.npz")


def main() -> int:
    if not list(dwf.Device.enumerate()):
        print("No Digilent device found. Is the AD3 plugged in?", file=sys.stderr)
        return 1

    with dwf.Device() as device:
        print(f"Connected: {device.name} sn={device.serial_number}")
        scope = device.analog_input

        scope.setup_channel(CH_NODE, range=RANGE_V, offset=0.0)
        scope.setup_channel(CH_CS, range=RANGE_V, offset=0.0)
        # Trigger on the CS falling edge (start of a transaction). Centered
        # trigger is fine: the post-trigger half (~82 us) dwarfs the ~2.4 us
        # confirmation slot, so the reply is always captured.
        scope.setup_edge_trigger(
            channel=CH_CS, slope="falling", level=TRIG_LEVEL_V, hysteresis=0.2
        )

        window_us = BUFFER_SIZE / SAMPLE_RATE * 1e6
        print(
            f"Arming scope: {SAMPLE_RATE/1e6:.0f} MS/s x {BUFFER_SIZE} "
            f"({window_us:.0f} us). Trigger = CH2 (CS) falling. "
            f"Run scripts/all_on.py to drive traffic..."
        )
        scope.single(
            sample_rate=SAMPLE_RATE,
            buffer_size=BUFFER_SIZE,
            configure=True,
            start=True,  # blocks until triggered + data read
        )
        node = np.asarray(scope.channels[CH_NODE].get_data(), dtype=np.float64)
        cs = np.asarray(scope.channels[CH_CS].get_data(), dtype=np.float64)

    # The trigger sits at buffer center; analyze the post-trigger region where
    # the panel drives its confirmation.
    trig = BUFFER_SIZE // 2
    post = node[trig:]

    node_min, node_max = float(post.min()), float(post.max())
    # Robust "high level": median of the post-trigger samples that are in the
    # upper part of the swing (the '1' bits). Falls back to max if all-low.
    hi_samples = post[post > (node_min + node_max) / 2.0]
    node_high = float(np.median(hi_samples)) if hi_samples.size else node_max
    frac_above_thresh = float((post > LOGIC_THRESH_V).mean()) * 100.0

    np.savez(
        OUTPUT_FILE,
        node=node,
        cs=cs,
        sample_rate=np.float64(SAMPLE_RATE),
        trigger_index=np.int64(trig),
    )

    print("\n--- Shared MISO_B0 node (R192 far side) ---")
    print(f"  CS swing (trigger ch): {float(cs.min()):.2f} .. {float(cs.max()):.2f} V"
          f"  {'(CS toggling - good)' if cs.max()-cs.min() > 1.0 else '(CS flat - check CH2 / trigger!)'}")
    print(f"  node min / max:        {node_min:.2f} / {node_max:.2f} V")
    print(f"  node '1'-bit level:    {node_high:.2f} V")
    print(f"  samples above {LOGIC_THRESH_V:.1f} V: {frac_above_thresh:.1f}%")

    print("\n  Verdict:")
    if node_high >= 2.5:
        print("    Full-swing (~3.3 V) here but 00 at Teensy pin 12 -> the bus is")
        print("    CLEAN at this node; the fault is an OPEN downstream. Trace")
        print("    continuity from this node to Teensy pin 12.")
    elif node_high >= 0.4:
        print(f"    CONTENTION confirmed: '1' bits only reach ~{node_high:.2f} V")
        print("    (a resistor-divider level, below the ~1.6 V logic threshold).")
        print("    Multiple B0 buffers are enabled and fighting on the shared bus.")
        print("    Fix: ensure only the selected port's buffer is enabled (CS->OE")
        print("    gating), or disable the others (U88/U92/U100/U104 OE -> +3.3 V).")
    else:
        print("    Node sits near 0 V (no swing) -> hard-pulled low: a stuck")
        print("    driver, short, or pulldown on the shared node.")
    print(f"\nRaw capture saved to {OUTPUT_FILE}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
