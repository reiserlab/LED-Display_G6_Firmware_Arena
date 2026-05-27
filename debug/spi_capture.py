"""Capture one panel-set SPI transaction from the G6 arena.

Hook the Analog Discovery 3 flywires to the Teensy 4.1 SPI-B0 pins:

    AD3 DIO 0  -> Teensy D13 (SCK_B0)
    AD3 DIO 1  -> Teensy D11 (MOSI_B0)
    AD3 DIO 2  -> Teensy D12 (MISO_B0)
    AD3 DIO 3  -> Teensy D0  (CS for panel set 0)
    AD3 GND    -> Teensy GND (any of pin 1 / 34 / 47 / 64)

Triggers on CS falling edge, captures 16k samples at 125 MS/s (~131 us window,
enough to swallow one 53- or 203-byte panel block at 30 MHz SPI), software-
decodes Mode-3 / MSB-first SPI, and saves the raw + decoded capture so you
can also drop it into the WaveForms GUI for visual inspection.

Run from the project root:

    pixi run -e debugad3 spi-capture

…and in another terminal trigger traffic with `python scripts/all_on.py`.
"""

from __future__ import annotations

import sys
from pathlib import Path

import dwfpy as dwf
import numpy as np

# Bit assignments in the 16-bit logic-input word.
SCK_BIT  = 0
MOSI_BIT = 1
MISO_BIT = 2
CS_BIT   = 3

SAMPLE_RATE = 125e6
BUFFER_SIZE = 16_384            # 131 us window at 125 MS/s
PRE_TRIGGER_FRACTION = 0.05     # 5 % before the CS falling edge

OUTPUT_FILE = Path(__file__).with_name("spi_capture.npz")


def decode_spi_mode3(samples: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Decode SPI Mode 3, MSB-first.

    In Mode 3 the controller drives MOSI on the SCK falling edge and the
    slave samples on the rising edge, so the bit value is stable across the
    full HIGH phase — sampling MOSI/MISO at the SCK rising edge while CS is
    LOW is the canonical decoder.
    """
    cs   = ((samples >> CS_BIT)   & 1).astype(np.uint8)
    sck  = ((samples >> SCK_BIT)  & 1).astype(np.uint8)
    mosi = ((samples >> MOSI_BIT) & 1).astype(np.uint8)
    miso = ((samples >> MISO_BIT) & 1).astype(np.uint8)

    rising = ((sck[1:] == 1) & (sck[:-1] == 0) & (cs[1:] == 0))
    idx = np.nonzero(rising)[0] + 1
    if idx.size < 8:
        return np.zeros(0, dtype=np.uint8), np.zeros(0, dtype=np.uint8)

    n_bits = (idx.size // 8) * 8
    return (
        np.packbits(mosi[idx[:n_bits]]),  # MSB-first (numpy default)
        np.packbits(miso[idx[:n_bits]]),
    )


def configure_trigger(logic: "dwf.DigitalInput") -> None:
    """Edge trigger on CS falling. dwfpy 1.0 high-level API.

    `setup_edge_trigger` handles both the trigger source (detector-digital-in)
    and the trigger mask in one call. The trigger position controls where the
    trigger event lands inside the buffer: `position` = number of samples to
    capture AFTER the trigger. dwfpy's default is 0 (entire buffer is
    pre-trigger), which would leave the actual SPI transaction outside the
    capture window — we want most of the buffer to be post-trigger so the CS-
    low burst lands inside it.
    """
    logic.setup_edge_trigger(channel=CS_BIT, edge="falling")
    pre = int(BUFFER_SIZE * PRE_TRIGGER_FRACTION)
    logic.trigger.prefill = pre
    logic.trigger.position = BUFFER_SIZE - pre


def main() -> int:
    devices = list(dwf.Device.enumerate())
    if not devices:
        print("No Digilent device found. Is the AD3 plugged in?", file=sys.stderr)
        return 1

    with dwf.Device() as device:
        print(f"Connected: {device.name} sn={device.serial_number}")

        logic = device.digital_input
        configure_trigger(logic)

        window_us = BUFFER_SIZE / SAMPLE_RATE * 1e6
        print(
            f"Arming: {SAMPLE_RATE / 1e6:.0f} MS/s x {BUFFER_SIZE} samples "
            f"({window_us:.1f} us). Trigger = DIO{CS_BIT} falling. "
            f"Run scripts/all_on.py to drive traffic..."
        )
        samples = logic.single(
            sample_rate=SAMPLE_RATE,
            sample_format=16,
            buffer_size=BUFFER_SIZE,
            configure=True,
            start=True,
        )

    samples = np.asarray(samples, dtype=np.uint16)

    # Per-channel diagnostics — catches wiring problems before the user
    # tries to interpret an empty SPI decode.
    print("Per-channel diagnostics:")
    for name, bit in (("SCK ", SCK_BIT), ("MOSI", MOSI_BIT),
                      ("MISO", MISO_BIT), ("CS  ", CS_BIT)):
        bits = ((samples >> bit) & 1).astype(np.uint8)
        edges = int(np.abs(np.diff(bits.astype(np.int8))).sum())
        high_pct = float(bits.mean()) * 100.0
        print(f"  DIO{bit} {name}: high {high_pct:5.1f}%   edges {edges}")

    mosi, miso = decode_spi_mode3(samples)
    np.savez(
        OUTPUT_FILE,
        samples=samples,
        mosi=mosi,
        miso=miso,
        sample_rate=np.float64(SAMPLE_RATE),
        bit_sck=np.uint8(SCK_BIT),
        bit_mosi=np.uint8(MOSI_BIT),
        bit_miso=np.uint8(MISO_BIT),
        bit_cs=np.uint8(CS_BIT),
    )

    print(f"Captured {mosi.size} SPI bytes per direction.")
    head = min(mosi.size, 16)
    if head:
        print(f"  MOSI[:{head}] = {' '.join(f'{b:02X}' for b in mosi[:head])}")
        print(f"  MISO[:{head}] = {' '.join(f'{b:02X}' for b in miso[:head])}")
    else:
        print("  (no rising SCK edges seen while CS was low)")
        print("  Likely causes — see per-channel diagnostics above:")
        print("    - SCK channel shows 0 edges       -> SCK probe not connected, "
              "or all_on.py didn't fire during capture window")
        print("    - SCK toggles but CS stays HIGH   -> trigger fired on noise; "
              "the capture missed a real CS pulse")
        print("    - all channels at 0.0% high      -> probes not connected; "
              "DIO inputs floating to the AD3's 1 Mohm pulldowns")
    print(f"Raw + decoded saved to {OUTPUT_FILE}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
