"""Simultaneously scope U96 pin 4 (buffer output) and the shared MISO_B0 node /
Teensy pin 12, on the SAME timebase, to settle whether the bus actually sags
below the buffer output (a real downstream sink/load) or whether they track
(no sink — the earlier 2.42 V vs 1.05 V was an artifact of two separate runs).

Both AD3 scope channels are measurement points here; we trigger on CH1's rising
edge (a confirmation '1' bit) so no CS reference is needed.

Wiring (AD3 *scope* / analog channels):
    Scope 1+ -> U96 pin 4   (buffer Y output, before R192)
    Scope 1- -> GND
    Scope 2+ -> Teensy pin 12  (= shared MISO_B0 node, after R192)
    Scope 2- -> GND

Run (controller streaming All-On):
    pixi run -e debugad3 python debug/compare_pin4_node.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import dwfpy as dwf
import numpy as np

CH_PIN4 = 0          # Scope CH1 -> U96 pin 4
CH_NODE = 1          # Scope CH2 -> Teensy pin 12 / shared node

SAMPLE_RATE = 100e6
BUFFER_SIZE = 16384
RANGE_V = 5.0
TRIG_LEVEL_V = 1.2   # rising edge of a pin-4 '1' bit (pin 4 reaches ~2.4 V)
HI_THRESH_V = 1.0    # samples above this on CH1 are treated as '1' bits

OUTPUT_FILE = Path(__file__).with_name("compare_pin4_node.npz")


def main() -> int:
    if not list(dwf.Device.enumerate()):
        print("No Digilent device found. Is the AD3 plugged in?", file=sys.stderr)
        return 1

    with dwf.Device() as device:
        print(f"Connected: {device.name} sn={device.serial_number}")
        scope = device.analog_input
        scope.setup_channel(CH_PIN4, range=RANGE_V, offset=0.0)
        scope.setup_channel(CH_NODE, range=RANGE_V, offset=0.0)
        # Trigger on CH1 (pin 4) rising — catches a confirmation '1' bit.
        scope.setup_edge_trigger(
            channel=CH_PIN4, slope="rising", level=TRIG_LEVEL_V, hysteresis=0.2
        )
        print(
            f"Arming scope: {SAMPLE_RATE/1e6:.0f} MS/s x {BUFFER_SIZE}. "
            f"CH1=U96 pin4, CH2=pin12. Trigger=CH1 rising. Run all_on.py..."
        )
        scope.single(
            sample_rate=SAMPLE_RATE, buffer_size=BUFFER_SIZE,
            configure=True, start=True,
        )
        pin4 = np.asarray(scope.channels[CH_PIN4].get_data(), dtype=np.float64)
        node = np.asarray(scope.channels[CH_NODE].get_data(), dtype=np.float64)

    # Sample both channels only where pin4 is a '1' bit (same instants).
    hi = pin4 > HI_THRESH_V
    if not hi.any():
        print("No '1' bits seen on CH1 (pin 4). Is U96 driving / All-On running?")
        return 1
    pin4_hi = float(np.median(pin4[hi]))
    node_at_hi = float(np.median(node[hi]))   # CH2 at the SAME instants pin4 is high
    drop = pin4_hi - node_at_hi

    np.savez(OUTPUT_FILE, pin4=pin4, node=node, sample_rate=np.float64(SAMPLE_RATE))

    print("\n--- Same-transaction comparison (median over pin-4 '1' bits) ---")
    print(f"  U96 pin 4  (CH1): {pin4_hi:.2f} V")
    print(f"  node/pin12 (CH2): {node_at_hi:.2f} V   (at the same instants)")
    print(f"  drop across R192: {drop:.2f} V  ->  {drop/33.0*1000:.0f} mA through 33 Ohm")
    print("\n  Verdict:")
    if drop > 0.5:
        print("    REAL sink: the node sags well below the buffer output in the")
        print("    SAME transaction. Something downstream of R192 draws current.")
        print(f"    With all B0 siblings disabled, the only thing on that net is")
        print("    the Teensy pin 12 -> the MCU pin is loading/holding MISO low.")
        print("    Next: confirm by scoping pin 12 with U96 also disabled, and")
        print("    recheck the LPSPI MISO ownership / any keeper/pull at the pin.")
    elif node_at_hi >= 1.6:
        print("    NO sink: node tracks pin 4 and is ABOVE the 1.6 V logic")
        print("    threshold. U96 drives the bus fine -> the Teensy *should* read")
        print("    a logic 1, but reports 00. That points at the LPSPI RECEIVE")
        print("    not latching -> do the loopback test (jumper pin 11->12).")
    else:
        print("    Node tracks pin 4 but both sit LOW (<1.6 V) -> U96 itself is")
        print("    only driving to a sub-threshold level (weak buffer / OE not")
        print("    solidly low / VCC). Check U96 pin 5 (VCC=3.3 V) and the OE tie.")
    print(f"\nRaw capture saved to {OUTPUT_FILE}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
