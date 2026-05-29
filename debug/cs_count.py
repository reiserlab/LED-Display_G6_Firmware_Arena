"""Count CS-falling edges seen on the panel's CS line over many refresh frames.

Goal: confirm whether the v0.3.1 panel sees one CS-down event per controller
refresh frame (the expected 300 Hz) or more (which would explain the
50%-failure PE03 pattern as the panel responding to extra/spurious CS edges).

Wiring (matches debug/spi_capture.py so flywires don't need to move):

    AD3 DIO 0  -> Teensy SCK   (D13 for SPI-B0)
    AD3 DIO 3  -> Teensy CS    (D0 for panel set 0, or whichever pin the
                                panel is actually wired to)
    AD3 GND    -> Teensy GND.

The capture is a 50 ms single-shot at 1 MS/s (1 us resolution — way finer
than the 3.33 ms refresh period and the 162 us transaction length). It is
not high enough to decode bytes, but that's fine — we only need to count
edges and measure inter-edge timing.

Output: per-edge timestamps, inter-edge gaps, and a one-line summary like
    "n=16 CS-falls over 50 ms; inter-edge p50=3331 us p90=3340 us min=15 us
     max=3401 us"
A `p50` close to 3333 us with `min` also near 3333 us = clean 300 Hz.
A bimodal distribution (some gaps ~3.3 ms, some <100 us, some ~6.6 ms) =
extra/spurious edges; the count > 15 in a 50 ms window confirms it.

Run from project root:

    pixi run -e debugad3 cs-count

…while a steady all-on stream is running.
"""

from __future__ import annotations

import sys
from pathlib import Path

import dwfpy as dwf
import numpy as np


SCK_BIT = 0
CS_BIT  = 3

SAMPLE_RATE  = 1_000_000          # 1 MS/s = 1 us resolution
BUFFER_SIZE  = 50_000             # 50 000 samples = 50 ms window
WINDOW_S     = BUFFER_SIZE / SAMPLE_RATE

OUTPUT_FILE  = Path(__file__).with_name("cs_count.npz")


def main() -> int:
    devices = list(dwf.Device.enumerate())
    if not devices:
        print("No Digilent device found. Is the AD3 plugged in?", file=sys.stderr)
        return 1

    with dwf.Device() as device:
        print(f"Connected: {device.name} sn={device.serial_number}")
        logic = device.digital_input

        # Trigger on the first CS falling edge so the window aligns to a
        # transaction boundary; that makes the per-frame edge count easy
        # to read off without an off-by-one ambiguity.
        logic.setup_edge_trigger(channel=CS_BIT, edge="falling")
        logic.trigger.prefill = 0
        logic.trigger.position = BUFFER_SIZE

        print(
            f"Arming: {SAMPLE_RATE / 1e6:.1f} MS/s x {BUFFER_SIZE} samples "
            f"({WINDOW_S * 1e3:.1f} ms window). "
            f"Trigger = DIO{CS_BIT} falling. "
            "Start an all-on stream now if not already running..."
        )

        samples = logic.single(
            sample_rate=SAMPLE_RATE,
            sample_format=16,
            buffer_size=BUFFER_SIZE,
            configure=True,
            start=True,
        )

    samples = np.asarray(samples, dtype=np.uint16)
    actual_window_ms = samples.size / SAMPLE_RATE * 1e3
    print(f"\nActual sample count: {samples.size} "
          f"(requested {BUFFER_SIZE}; covers {actual_window_ms:.2f} ms at "
          f"{SAMPLE_RATE/1e6:.2f} MS/s)")

    cs  = ((samples >> CS_BIT)  & 1).astype(np.uint8)
    sck = ((samples >> SCK_BIT) & 1).astype(np.uint8)

    # CS falling edges = transitions 1 -> 0.
    falls = np.nonzero((cs[1:] == 0) & (cs[:-1] == 1))[0] + 1

    print(f"\nCaptured {falls.size} CS-falling edges in {actual_window_ms:.2f} ms.")
    if falls.size:
        print(f"  First fall at sample {falls[0]} = {falls[0] / SAMPLE_RATE * 1e3:.2f} ms")
        print(f"  Last  fall at sample {falls[-1]} = {falls[-1] / SAMPLE_RATE * 1e3:.2f} ms")
        print(f"  CS LOW fraction: {(cs == 0).mean():.3f}")
    if falls.size < 2:
        print("Not enough edges to compute gaps. Is the controller streaming?")
        print(f"CS levels: low_fraction={(cs == 0).mean():.3f}  "
              f"sck_low_fraction={(sck == 0).mean():.3f}")
        return 0

    gaps_us = np.diff(falls)  # 1 sample = 1 us at 1 MS/s
    print("\nInter-edge gap stats (us):")
    print(f"  n={gaps_us.size}  min={gaps_us.min()}  max={gaps_us.max()}  "
          f"mean={gaps_us.mean():.0f}  median={np.median(gaps_us):.0f}")
    print(f"  p10={np.percentile(gaps_us, 10):.0f}  "
          f"p50={np.percentile(gaps_us, 50):.0f}  "
          f"p90={np.percentile(gaps_us, 90):.0f}")

    # Bucket the gaps so a bimodal pattern jumps out.
    buckets = [
        (0,      100,   "<100us  (back-to-back, suspicious)"),
        (100,    500,   "100-500us"),
        (500,    2000,  "500us-2ms"),
        (2000,   4000,  "2-4ms   (one controller period @300Hz = 3333us)"),
        (4000,   8000,  "4-8ms"),
        (8000,   16000, "8-16ms"),
        (16000,  64000, ">16ms"),
    ]
    print("\nCS gap distribution:")
    for lo, hi, label in buckets:
        n = int(((gaps_us >= lo) & (gaps_us < hi)).sum())
        if n:
            print(f"  {label:<40s} {n}")

    # SCK cross-check: count SCK transitions to confirm whether the SCK line
    # shows the expected ~15 bursts of activity (one per refresh frame at 300 Hz).
    sck_falls = np.nonzero((sck[1:] == 0) & (sck[:-1] == 1))[0] + 1
    sck_rises = np.nonzero((sck[1:] == 1) & (sck[:-1] == 0))[0] + 1
    print(f"\nSCK channel (cross-check):")
    print(f"  rising edges: {sck_rises.size}   falling edges: {sck_falls.size}")
    print(f"  SCK HIGH fraction: {(sck == 1).mean():.3f}  "
          f"(MODE3 idle-HIGH → expect ~95% during steady streaming)")

    # Cluster CS-falls into bursts so we can tell whether the 3000+ edges
    # are one tight burst per real controller event or many.
    BURST_GAP_US = 200  # gaps within a single ringing burst are <<200 us
    burst_starts = np.concatenate(([0], np.nonzero(gaps_us > BURST_GAP_US)[0] + 1))
    burst_count  = burst_starts.size
    print(f"\nCS-fall clustering (burst boundary = gap > {BURST_GAP_US} us):")
    print(f"  {burst_count} bursts in {WINDOW_S * 1e3:.1f} ms = "
          f"{burst_count / WINDOW_S:.1f} Hz")
    if burst_count > 1:
        # Edges per burst.
        ends = np.append(burst_starts[1:], falls.size)
        sizes = ends - burst_starts
        print(f"  edges per burst: min={sizes.min()} max={sizes.max()} "
              f"median={int(np.median(sizes))} mean={sizes.mean():.1f}")
        # Inter-burst spacing (start to start).
        burst_t = falls[burst_starts]
        if burst_t.size > 1:
            burst_gaps_us = np.diff(burst_t)
            print(f"  inter-burst gap (us): min={burst_gaps_us.min()} "
                  f"max={burst_gaps_us.max()} "
                  f"median={int(np.median(burst_gaps_us))}")

    # Save raw for follow-up inspection in WaveForms GUI.
    np.savez(
        OUTPUT_FILE,
        samples=samples,
        cs=cs,
        sck=sck,
        falls=falls,
        gaps_us=gaps_us,
        sample_rate=SAMPLE_RATE,
    )
    print(f"\nSaved {OUTPUT_FILE} ({OUTPUT_FILE.stat().st_size / 1024:.1f} KB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
