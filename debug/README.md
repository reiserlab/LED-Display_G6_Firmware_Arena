# Debug — Analog Discovery 3 probe of G6 SPI

Bench setup for probing the Teensy 4.1 → arena SPI bus with a Digilent Analog
Discovery 3 (AD3). Use this when the firmware logs say the controller is
faithfully transmitting but the panels don't respond — capturing the wire
disambiguates whether the bug is on the controller side (SPI / DMA library
bug) or downstream (level translator, ribbon, panel firmware not running).

## Prerequisites

1. Install the Digilent WaveForms runtime system-wide:
   `digilent.waveforms_<version>_amd64.deb` from
   <https://digilent.com/shop/software/digilent-waveforms/>. This is the
   library `dwfpy` loads at import time.
2. Materialize the isolated debug env:
   ```
   pixi install -e debugad3
   pixi run -e debugad3 python -c "import dwfpy as d; print(d.Application.get_version())"
   ```
3. Plug the AD3 into a USB-C port on the host that runs the script. WaveForms
   GUI and `dwfpy` cannot share the device — close the GUI before running the
   script and vice versa.

## Wiring

Five flywires from the AD3 2×15 MTE flycable to the Teensy 4.1 board.
Pin numbers below are the Teensy silk labels (e.g. `D0`, `D11`).

| AD3 flywire | Teensy 4.1 pin | Net | Notes |
|---|---|---|---|
| DIO 0 | **D13** | `SCK_B0`  | Shared with the on-board LED — fine to probe |
| DIO 1 | **D11** | `MOSI_B0` | |
| DIO 2 | **D12** | `MISO_B0` | The signal we're checking |
| DIO 3 | **D0**  | `CS` for panel set 0 | Trigger source — falling edge fires the capture |
| GND   | any GND pin (1, 34, 47, 64) | `GND` | **Required** — without it the LA reads garbage |

The Teensy outputs 3.3 V LVCMOS; AD3 DIO is 5 V-tolerant 3.3 V LVCMOS, so no
level shifter is needed.

## Option A — WaveForms GUI (fastest, recommended for first look)

This gives you the SPI byte stream decoded next to the raw waveforms in
under three minutes.

1. Launch the WaveForms application. The AD3 should auto-connect; the
   bottom status bar shows its serial number.
2. **Welcome → Logic Analyzer**.
3. **Add the four signals** (Add ▸ Signal):
   - `SCK` on DIO 0
   - `MOSI` on DIO 1
   - `MISO` on DIO 2
   - `CS` on DIO 3
4. **Add a SPI bus interpreter** (Add ▸ Bus ▸ SPI):
   - Select: DIO 3
   - Clock: DIO 0
   - MOSI: DIO 1
   - MISO: DIO 2
   - Mode: **3** (CPOL=1, CPHA=1) — must match `g6_01-panel-protocol.md`
   - Bit order: **MSB first**
   - Word size: 8
   - Active: low (CS)
5. **Trigger setup** (Trigger panel at top):
   - Source: Digital
   - Type: Edge
   - Channel: DIO 3
   - Slope: **falling**
   - Mode: Normal (wait for trigger)
6. **Sample rate**: 125 MS/s. **Buffer**: 16 KiS (≈ 131 µs window — long
   enough to swallow one 203-byte GS16 panel block at 30 MHz SCK).
7. Click **Run / Single**.
8. In another terminal: `python scripts/all_on.py`. The CS falling edge for
   panel set 0 should arm a single capture immediately.
9. The Logic Analyzer view shows the four raw signals plus a decoded
   "MOSI / MISO" byte stream from the SPI interpreter.

What to look for:

- **SCK idles HIGH and pulses LOW** between the leading falling edge and the
  trailing rising edge — that's Mode 3. If SCK idles LOW instead, our
  controller is actually emitting Mode 0 despite the `SPI_MODE3` setting.
- **MOSI[0..7]** should be `81 30 FF FF FF FF FF FF` for an ALL_ON GS16
  oneshot — matches what the firmware printed for `first_block:`.
- **MISO[0..2]** is the panel's CIPO confirmation slot. Three cases:
  - `81 00 00` — empty-buffer sentinel: panel **is alive** and the controller-
    side firmware MISO capture has a bug (the `SPI.transfer(buf, retbuf, …)`
    call isn't actually filling `retbuf`).
  - `?1 30 ??` — echo of our last cmd with a CRC8: panel is alive and
    accepting GS16 oneshots; the dark LEDs are a panel-firmware / hardware
    issue (LED drivers, BCM engine, etc.).
  - `00 00 00` — MISO genuinely stays low at the Teensy header: panel isn't
    driving the line. Suspect: panel unpowered, ribbon disconnected, wrong
    stack slot (CS not reaching the panel), or panel firmware compiled with
    `STAGE2_SELFTEST=1` (skips `messenger.initialize()`).
  - MISO **floating / noisy** with no clean digital levels: arena's
    `74LVC1G125` MISO buffer isn't enabling (its OE is gated by the panel's
    CS lines per `miso_enable.kicad_sch`), or its 3.3 V rail is missing.

## Option B — `dwfpy` script (reproducible, headless)

Same wiring, same trigger logic, no GUI required.

```bash
pixi run -e debugad3 spi-capture          # ≡ python debug/spi_capture.py
```

Output is `debug/spi_capture.npz` containing `samples` (16-bit packed
digital), `mosi`, and `miso`. Re-load with `numpy` for further analysis:

```python
import numpy as np
d = np.load("debug/spi_capture.npz")
print("MOSI[:32]:", " ".join(f"{b:02X}" for b in d["mosi"][:32]))
print("MISO[:32]:", " ".join(f"{b:02X}" for b in d["miso"][:32]))
```

If you want to inspect waveforms visually, load the `.npz` into the
WaveForms GUI via File ▸ Open (it accepts the `.csv` it exports; convert
the `samples` array with `numpy.savetxt(..., fmt="%d")` if needed).

## Common gotchas

- **Forgetting GND.** AD3 inputs are 1 MΩ // 24 pF — without a common GND
  the levels float and the LA reports random transitions.
- **Both WaveForms GUI and dwfpy open at once** — they collide on the USB
  device handle. Close the GUI before running the script.
- **`OSError: cannot open shared object file: libdwf.so`** — the WaveForms
  runtime isn't installed. Install the `.deb` from Digilent.
- **Trigger never fires.** Check that all_on.py is actually running and the
  controller is in `STATE=1` (`[cmd] display tick ...` in the printf
  monitor). CS pulses ~300 Hz once streaming starts; a single shot should
  arm within milliseconds.
