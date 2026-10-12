# Qwiic peripherals (LAB-214) — bench results and decisions, 2026-10-12

Scope: validate the LAB-214 parts on the arena_12-18 Qwiic jack through the
sensor-agnostic bridge from PR #58 (`GET_I2C_SCAN` 0xB0 / `I2C_TRANSFER` 0xB1),
measure how many devices the one jack carries and what sensor traffic costs
the controller, and settle how sensors are configured and logged. Follows
`qwiic-bench-2026-09-15.md` (LAB-211 light sensors).

## Setup

- arena_12-18 v1.0 controller, USB-CDC `/dev/cu.usbmodem208523401`, no panels.
  Firmware: branch `qwiic/lab-214-peripherals` (= PR #58 + `main` at 12bba7e),
  `teensy41-12-18` env; a second build with `-DQWIIC_I2C_CLOCK_HZ=400000`.
- Chain (one daisy chain on J2, Adafruit STEMMA QT breakouts, as plugged on the bench):
  VEML7700 0x10, TSL2591 0x29, AS7343 0x39, **SHT4x 0x44**, **LSM6DSOX 0x6A**.
- Added later in the session: **SparkFun Pi Servo pHAT (PCA9685 0x40)** at the end
  of the chain, USB-C powered (power meter inline), one micro servo on ch0.
- Not on the bench: the PCA9548 mux, a second SHT4x (duplicate address). Their
  code paths are written but **unverified** — marked below.
- Everything is host-side. The only firmware edit is the `QWIIC_I2C_CLOCK_HZ`
  build-flag override (default unchanged, 100 kHz).

## Parts identified

| Addr | Part | Identify | Reading on the bench |
|---|---|---|---|
| 0x44 | SHT4x (Adafruit 5776/5665) | serial 0x14DCF709, CRC-8 OK on serial and data | 22.7 °C, 59–61 %RH; high/med/low precision 13.9 / 9.2 / 5.0 ms host round trip (mostly the mandated wait) |
| 0x6A | LSM6DSOX-class IMU | WHO_AM_I 0x6C (LSM6DSOX and LSM6DSO32 share it — check the breakout label) | at rest \|a\| = 0.997–1.001 g, tilt 10.5° from vertical (board on its cable), die 22.8 °C; 14-byte temp+gyro+accel block read 1.8 ms |

| 0x40 (+0x70 all-call) | PCA9685, SparkFun Pi Servo pHAT | MODE1 0x11 / prescale 30 at power-on, as the datasheet says | `configure(50)`: prescale 121 = 50.03 Hz, wake + auto-increment, ALLCALL cleared; ch0 1500 µs → OFF count 307 reads back; full-OFF bit sets |

The ticket's accelerometer was the LIS3DH (0x18); the part on the bench is an
LSM6DSOX (0x6A). The driver is written for what arrived; 0x18/0x19 are in the
address book without a driver.

### LSM6DSOX latched "was it bumped" flag — what actually works

The host-side bump detector is the chip's wake-up function (acceleration
slope over WK_THS on any axis sets `WAKE_UP_SRC.WU_IA`). Register-level bench
findings, each tested without physical motion by dropping the threshold to 0
(noise fires it) and raising it to the maximum before reading:

| TAP_CFG0 | MD1_CFG.INT1_WU | Flag after the event stops | Clears on reading WAKE_UP_SRC |
|---|---|---|---|
| LIR | off | **gone** (live status only) | n/a |
| LIR | on | held | **never** |
| LIR + INT_CLR_ON_READ | on | held | **yes, first read** |
| LIR + INT_CLR_ON_READ | off | gone | n/a |

So `arm_bump()` sets LIR + INT_CLR_ON_READ and routes the wake-up to INT1
(the pin is not wired to anything; the routing is what enables the latch).
With that, `bump_flag()` is a clean read-and-clear: `test_lsm6dsox_bump_flag_
latches_and_clears_on_read` passes. Threshold resolution is FS/64 = 31 mg at
±2 g; 0.25 g default stayed clear over 2 s at rest. **Still to do by hand:**
`pixi run qwiic-env`, tap the breakout between lines, expect one `BUMPED[..]`
line then clear.

### PCA9685 / Servo pHAT — findings

- **Needs its own power even for the logic side.** Plugged into the chain with
  no USB-C the board never appeared on the bus (no 0x40, no 0x70; the five
  breakouts unaffected). With a 5 V USB-C adapter on the pHAT it scans at once.
  The pHAT's Qwiic 3.3 V pin does not feed the PCA9685; only ground and the two
  bus lines are shared with the arena. Its presence on the bus is therefore
  power-dependent: unplugging its USB-C (done once to insert a power meter)
  removes 0x40/0x70 from the scan with no effect on the other devices.
- **The all-call address 0x70 ACKs reads** (returns MODE1), contrary to the
  write-only assumption in the first draft. A 1-byte-read probe therefore
  cannot tell it from a PCA9548; `locate_devices()` instead reads the
  PCA9685's MODE1 and, while ALLCALL is set, keeps 0x70 a root device and maps
  no channels through it (verified: `test_pca9685_allcall_shadow_is_not_
  mistaken_for_a_mux`). The LAB-211 mux tests in `test_qwiic_i2c.py` apply the
  same exclusion. After `configure()` clears ALLCALL, 0x70 leaves the scan. A real mux at 0x70 next to an un-configured PCA9685 would still be
  ambiguous: configure the PCA9685 first, or strap the mux to 0x71–0x77.
- **Servo motion verified.** Micro servo on ch0, pHAT on a 5 V USB-C adapter
  with a power meter inline: `pixi run qwiic-servo -- --channel 0 --sweep`
  (1000 → 2000 → 1000 µs in 100 µs steps, 0.3 s each, counts 205 → 410 at
  50.03 Hz, every count read back, channel released at the end) moved the arm
  through its range and back, three runs. Meter on the pHAT's USB-C: ≈ 49 mA
  idle (PCA9685 + LEDs + servo holding), ≈ 75 mA peak while moving; nothing
  of it passes through the arena, whose USB link stayed up throughout.

### Hands-on sanity check (all five sensors, 1 Hz, 50 s)

Covering the light sensors: VEML7700 / TSL2591 ~30–40 lx → ~2 lx, AS7343
clear 196 → 3 counts, all three in step; ~75 lx after. Breath / fingertip on
the SHT4x: 22.7 °C / 59 %RH → 29.6 °C / 93 %RH within seconds, relaxing after.
IMU tilt 13° → 73° and back while handled, bump latch fired with the right
axes on every jostle (including the cable tug when the neighboring boards were
covered) and cleared on each read. No I2C errors.

## Bus limits — six devices on one jack

`pixi run qwiic-soak`: round-robin read-only register transfers over every
device (SHT4x = serial-number command, two legs), error count by I2C status,
host round-trip latency, and `GET_HEALTH.loop_max_1s_us` sampled every second.

| Build | Transfer | Transfers | Errors | Latency p50 (ms) register / SHT4x | loop_max_1s_us median |
|---|---|---|---|---|---|
| 100 kHz | 1–2 B ID reads | 16,345 (15 s) | 0 | 0.53 / 2.38 | 666 |
| 100 kHz | 32 B reads | 3,120 (10 s) | 0 | — | 3,241 |
| 100 kHz | + full scan every 20 rounds | 9,575 (10 s) | 0 | 0.53 / 2.39 | 12,257 (the scan) |
| 400 kHz | 1–2 B ID reads | 26,780 (15 s) | 0 | 0.22 / 1.96 | 186 |
| 400 kHz | 32 B reads | 6,555 (8 s) | 0 | — | 882 |
| 400 kHz | + full scan every 10 rounds | 12,350 (8 s) | 0 | — | 9,102 (the scan) |
| 100 kHz, **+ Servo pHAT** (6 boards, 7 addresses incl. the 0x70 shadow) | 1–2 B ID reads | 19,334 (15 s) | 0 | 0.52 / 2.38 | 666 |

HIL suites (`test_qwiic_i2c.py`, `test_qwiic_peripherals.py`): 15 passed,
4 skipped (mux, PCA9685) at both clocks with five devices; with the pHAT added
(100 kHz, USB-C powered) the two PCA9685 tests pass as well — 17 passed,
2 skipped (mux).

Reading: the controller's loop stalls for exactly the wire time of each
transfer (≈ 0.1 ms/byte at 100 kHz, 0.025 at 400 kHz) plus ~0.1 ms of
overhead; nothing accumulates across devices. The 9–12 ms outlier is
`GET_I2C_SCAN` (112 addresses), which Studio should issue only at setup, never
while a display runs. USB round trip (~0.2 ms) is now the floor at 400 kHz.

### Sensor traffic against a running display (`pixi run qwiic-display-soak`)

Controller in ALL_ON (refresh timer pushing a frame to the panel bus every
tick, 400 Hz on the 12-18 build), six boards on the chain, 10 s phases:

| Phase | Frames | fps | Loop max per 1 s window (median / max) | Transfers | Errors |
|---|---|---|---|---|---|
| A display alone | 4002 | 399.98 | 1.87 / 1.87 ms (the SPI frame push) | — | — |
| B display + bus soak (every device, back-to-back) | 4004 | 400.00 | 2.53 / 2.53 ms | 639 | 0 |
| C display + 1 Hz env poll (SHT4x + IMU + bump) | 4017 | 400.00 | 3.46 / 3.46 ms | 10 | 0 |

No frame was lost in any phase. The cost is loop latency only: a transfer
that lands in the same loop pass as a frame push extends that pass by its
wire time (phase C's 3.5 ms is push + the SHT4x read leg), so a frame can
leave up to ~1.5 ms late but is never skipped — the refresh timer is
free-running. Frame *jitter* was not measured directly (a static ALL_ON frame
produces no FRAME telemetry records); if a future application needs sub-ms
frame timing with sensors polled, measure it with a changing pattern and the
telemetry ring, and prefer 400 kHz.

### Current drawn from the arena

Power meter on the arena controller's USB supply (5.19 V): 0.623 A with the
five-breakout chain attached, 0.618 A with it unplugged — **≈ 5 mA, ≈ 1 mA per
board** (mostly the breakouts' power LEDs). The Servo pHAT draws nothing from
the arena (own USB-C). The controller board itself idles near 620 mA
(Teensy 4.1 + Ethernet PHY + board logic). Current is not what limits the
device count; the pull-up stack is.

Side observation (not Qwiic): during the ALL_ON phases the 12 panels on the
bench 12-18 drew **3.20–3.26 A** on the panel supply (Michael's meter), i.e.
≈ 270 mA per panel at full white with this firmware; the display is stopped
(ALL_OFF) at the end of the script.

### What bounds the device count

The firmware has no limit (the scan returns up to 112 addresses). The bus does:

- **Current:** ≈ 1 mA per board from the Teensy's 3.3 V regulator (measured,
  above) — not a constraint.
- **Pull-up stacking.** Adafruit QT boards carry 10 kΩ on SDA/SCL each, the
  Teensy pads 22 kΩ; the five boards here give ≈ 1.8 kΩ. The I²C 3 mA sink
  floor at 3.3 V is ≈ 1 kΩ, so ~9 Adafruit-class boards is the ceiling.
  **SparkFun boards (incl. the Servo pHAT) carry 2.2 kΩ each** — one is fine,
  two put the bus at the floor: cut their pull-up jumpers.
- **Rise time at 400 kHz.** ≤ 300 ns allowed; estimated ≈ 230 ns for this
  chain (1.8 kΩ × ~150 pF of boards + cables). Fine today, shrinking margin
  per added board or metre. Measure SCL rise with the AD3 on the longest
  intended chain before the default moves to 400 kHz.
- **Addresses.** Fixed-address duplicates (a second SHT4x at 0x44) need the
  PCA9548 mux — supported by the host library, still unverified on the part.
  PCA9685 answers (reads included) at its all-call 0x70 until MODE1.ALLCALL is
  cleared, which is the mux's default address; `PCA9685.configure()` clears it
  and the probe warns (see § PCA9685 findings).

## Decisions (proposed for LAB-214)

1. **How many at once.** Support a *list*, not a fixed number: the tested
   envelope is five devices on one chain at 100 and 400 kHz with zero errors
   (six with the pHAT at 100 kHz);
   the practical ceiling is the pull-up budget (~8 Adafruit-class boards, fewer
   with SparkFun boards unless their jumpers are cut). Studio should not
   hard-cap below 8 (one per mux channel) but should show the chain's device
   count after the setup scan.
2. **Logging stays host-side**, controller sensor-agnostic. Per-trial
   snapshot (trial start/end, LAB-214 § 1a) and 1 Hz interval logging both fit
   over 0xB1: one SHT4x read is 2.4 ms of host time and ~1.2 ms of bus; one IMU
   read + bump flag 2.3 ms. `scripts/qwiic_env.py --csv` is the column shape
   for an `env.csv` sidecar next to the trial log. The firmware path
   (`0xB2/0xB3` + a telemetry record) is only warranted if logging must
   continue with Studio disconnected or bump events need controller-clock
   timestamps; neither is asked for yet — keep `0xB2–0xBF` reserved, unused.
3. **Configuration** in the webDisplayTools rig config (`configs/rigs/*.yaml`)
   under `peripherals:` as sketched in LAB-214 (`kind`, `address`, optional
   `mux`, `poll: trial_start | interval: 1.0 | on_demand`), drivers ported from
   `tests/qwiic_peripherals.py` + `tests/qwiic_sensors.py` into the Studio
   register-map library. That is the next session.
4. **Bus clock** stays 100 kHz by default. 400 kHz is validated for this chain
   and cuts the per-transfer loop cost ~2.5×; switch the default after the
   AD3 rise-time check with the servo board and the 200 mm cable in the chain.
5. **Scan only at setup.** 9–12 ms of loop stall; Studio probes once when the
   rig connects and otherwise addresses devices from the YAML.
6. **One owner for read-to-clear state.** The IMU bump flag clears on read;
   exactly one process (the trial logger) reads it. `qwiic-probe` therefore no
   longer arms it; `qwiic-env` does.
7. **Self-powered peripherals.** A board that is not fed by the Qwiic 3.3 V
   (the Servo pHAT) is present only while its own supply is on; Studio should
   treat "declared in YAML but absent from the setup scan" as a clear error,
   not retry.

## Open items

- PCA9548 mux with a second SHT4x behind it; 200 mm cable position in the chain.
- Confirm the IMU breakout: LSM6DSOX (±2–16 g) vs LSM6DSO32 (±4–32 g) — same
  WHO_AM_I; only the full-scale table differs (driver assumes LSM6DSOX).
- AD3 SCL rise-time measurement before changing the default clock.
- Docs repo `g6_03-controller.md`: nothing new to register (no new opcodes);
  note the `QWIIC_I2C_CLOCK_HZ` build flag if the clock becomes a host concern.

## Commands

```
pixi run qwiic-probe -- --port /dev/cu.usbmodem208523401            # scan, identify, one reading each
pixi run qwiic-env   -- --port /dev/cu.usbmodem208523401 --csv      # 1 Hz T/RH/tilt/bump log
pixi run qwiic-soak  -- --port /dev/cu.usbmodem208523401 --seconds 30
pixi run qwiic-display-soak -- --port /dev/cu.usbmodem208523401        # ALL_ON refresh vs. sensor traffic
pixi run pytest tests/test_qwiic_peripherals.py tests/test_qwiic_i2c.py -v --transport=serial -s
PLATFORMIO_BUILD_FLAGS=-DQWIIC_I2C_CLOCK_HZ=400000 pixi run deploy-12-18   # 400 kHz bench build
```
