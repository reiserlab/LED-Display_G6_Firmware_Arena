# Qwiic / STEMMA QT jack bring-up — bench results, 2026-09-15

Scope: validate the I2C path from a host, through the arena_12-18 controller,
to the LAB-211 light sensors on the board's Qwiic jack (J2). Not in scope:
optical calibration (fixed geometry, reference detector) — that is LAB-211.

## Setup

- arena_12-18 v1.0 controller, USB-CDC (`/dev/cu.usbmodem208523401`), no panels,
  no SD card. Firmware: this branch, `teensy41-12-18` env (DEBUG_SERIAL, diag muted).
- Sensors (Adafruit STEMMA QT breakouts, 100 mm cables): TSL2591 (0x29),
  VEML7700 (0x10), AS7343 (0x39). Boards lying loose on the bench under room
  light — **no controlled geometry**, so absolute values and cross-sensor
  ratios below say nothing about the sensors' accuracy.
- Not tested: PCA9548 mux, duplicate addresses behind it, 200 mm cables.

## Jack pinout (net-traced from `arena_12-18/teensy.kicad_sch`, hardware repo `origin/main`)

| J2 pin | Net | Teensy 4.1 |
|---|---|---|
| 1 | GND | |
| 2 | +3.3V | |
| 3 | Qwiic_SDA | D17 = SDA1 |
| 4 | Qwiic_SDL | D16 = SCL1 |

`Wire1`; separate from the MCP4725 AO DAC on `Wire` (D18/D19). No pull-ups on
the board (the breakouts carry 10k each; the Teensy pads have 22k). Pins 24/25
(Wire2) are CS_10/CS_11 and must not be used. Bus clock 100 kHz for bring-up.

## Firmware bridge

`GET_I2C_SCAN` (0xB0) and `I2C_TRANSFER` (0xB1) — generic, sensor-agnostic; see
README § Qwiic. Compiled into all variants; only `ARENA_HW_12_18` has
`qwiic_present = true`, the others answer `status 1`.

## Results

### Bus level (`tests/test_qwiic_i2c.py`, nothing sensor-specific)

| Check | Result |
|---|---|
| Scan payload ascending, unique, 0x08–0x77 | pass |
| AO DAC (0x60) **not** visible on the Qwiic bus | pass — bridge is on Wire1 |
| Bad framing (short frame, wlen mismatch, rlen > 64, 8-bit addr) → status 1 | pass |
| Empty address → status 2 (address NACK) on write, read, ACK-probe; no hang | pass |
| Every scanned address also ACKs a probe | pass |

### Per sensor

| Sensor | Identify | Register round-trip | Reading |
|---|---|---|---|
| TSL2591 | ID `0x50` at reg 0x12 | ENABLE/CONTROL write, AVALID set | full 7265 / IR 1765 @ gain ×25, 100 ms (~680 lx approx) |
| VEML7700 | ID reg `0xC481` (device id 0x81) | ALS_CONF_0 200 ms ↔ 100 ms reads back | als 8542 / white 10450 @ gain ×1, 100 ms (~492 lx approx) |
| AS7343 | ID `0x81` at reg 0x5A via `CFG0.REG_BANK=1`, rev 0x00, aux 0x00 | ENABLE.PON reads back | 18-channel auto-SMUX frame, see below |

All three together on one cable: scan `0x10 0x29 0x39`, all identified, 9/9 tests.

### TSL2591 gain sweep (same light)

| gain | full | ir | lux approx |
|---|---|---|---|
| low (×1) | 294 | 69 | 703 |
| med (×25) | 7204 | 1729 | 679 |
| high (×428) | 37888 | 37888 | saturated (both channels pinned) |

low→med ratio **24.5×** — the datasheet's medium-gain value (nominal 25). The
lux figures differ 3 % only because the formula uses the nominal 25.

### AS7343 spectrum (room light, gain ×64, 30 × 600 × 2.78 µs = 50 ms/cycle, full scale 18000)

```
405 nm  ████····························     138
425 nm  ██████████······················     348
450 nm  ████████████████················     540
475 nm  ███████████████·················     518
515 nm  ████████████████████············     675
550 nm  ██████████······················     329
555 nm  ████████████████████████████████    1081
600 nm  █████████████████████████████···     989
640 nm  ███████████████████████████·····     902
690 nm  ████████████████████············     673
745 nm  ███████·························     243
855 nm  ██████████······················     321
VIS (clear)                                 1343
```

Slots 4/5 of each SMUX cycle are **VIS (clear)** and **FD (flicker detect)**,
not two clear diodes as Adafruit's `VIS_TL/VIS_BR` enum suggests: the FD slot
reads full scale (17999) in every cycle regardless of light, the VIS slot
repeats within one count across the three cycles (1216/1216/1217). FD is
excluded from the saturation flag. Peak channel at 7.5 % of full scale →
gain ×256 would put it near 50 % under this light.

### Noise and rate (`qwiic-eval`, 5 s capture, all three sensors)

31 rounds in 5.1 s = **6.1 Hz**, set by the AS7343 frame (3 × 50 ms).
CV over the capture: VEML7700 0.03 %, TSL2591 0.06 %, AS7343 0.00–0.21 %
per channel. A/B null test (no light change between phases): every ratio
1.000 ± 0.003.

### Link overhead

| transfer | time |
|---|---|
| ACK probe (0 data bytes) | 0.21 ms |
| 4-byte register read | 0.80 ms |
| 37-byte AS7343 block read | 3.87 ms |

USB round trip ≈ 0.2 ms; the remainder is wire time at 100 kHz (~0.1 ms/byte).
Each transfer blocks the controller's main loop for its wire time. Reading all
three sensors is ~7 ms of bus per round against 100–150 ms integrations (bus
~5 % busy) — daisy-chaining does not limit the rate; the sensors' integration
does (TSL2591 ≥ 100 ms, VEML7700 ≥ 25 ms, AS7343 configurable).

### Cross-sensor (uncalibrated, loose geometry — indicative only)

TSL2591 lux / VEML7700 lux = 0.43; TSL2591 IR fraction 0.25; AS7343 NIR/VIS 0.24;
F6_640/FZ_450 = 1.67. The IR fractions agree between the two sensors that
report one (LED/fluorescent room light, not incandescent); the lux disagreement
is expected for two uncalibrated photometers with different spectral responses.

## Open items

- PCA9548 mux with the duplicate-address pairs and a 200 mm cable (code and
  tests exist, untested against the part).
- Register 0xB0/0xB1 in `g6_03-controller.md` (docs repo) and decide on a
  capability bit; both are shared wire-protocol fields.
- 400 kHz once the long-cable case passes (cuts the AS7343 block read to ~1 ms).
- Sensor choice is open (LAB-211). The firmware is sensor-agnostic; the
  part-specific code is `tests/qwiic_sensors.py` only.
