# CIPO doesn't reach the Teensy — investigation & bench-experiment plan

**Status:** root cause hypothesis identified; cheap firmware experiment defined, not yet run.
**Symptom (Frank):** the panel's CIPO/MISO confirmation reaches the arena's MISO buffer but
never makes it through to the Teensy — the controller reads `00`.

This is a self-contained handoff: a fresh session (or person) should be able to execute the
experiment below from this file alone.

---

## TL;DR

- The arena does **not** wire panel CIPO straight to the Teensy. Each panel's CIPO goes
  through a per-panel **tri-state buffer** (`74LVC1G125`) whose `/OE` is decoded from chip-select
  lines, then onto a **shared** `MISO_B0`/`MISO_B1` node (5 buffers wired-OR'd per bus) and on to
  the Teensy. The 33 Ω series resistors are source termination only — they do **not** prevent
  contention.
- For CIPO to reach the Teensy, **exactly one** buffer per bus may drive at a time. That requires
  every CS line feeding the enable decode to be at a defined level.
- **The controller firmware drives only 10 of the board's 20 CS lines** and leaves the rest as
  floating inputs (power-on default). The per-column MISO-enable is the AND of **four** CS lines,
  so a floating CS that drifts low spuriously enables that column's buffer → multiple buffers
  drive the shared node → a logic-`1` collapses to a sub-threshold divider voltage → Teensy
  latches `00`.
- **Both firmwares are doing their job.** The controller correctly routes CIPO to the hardware
  MISO pins; the panel (PR #7) drives the CIPO confirmation aligned to CS. The fault is the arena
  board's shared/gated MISO return network, aggravated by the firmware leaving CS lines floating.

---

## Confirmed facts (with evidence)

### Controller firmware (this repo, `LED-Display_G6_Firmware_Arena`, tip `7702c36`)
- CIPO is routed to the Teensy 4.1 hardware MISO pins **12 (SPI/B0)** and **1 (SPI1/B1)** before
  `begin()` — `src/SpiManager.cpp:31-34`, `src/constants.h:73` (`region_cipo_pins = {12, 1}`).
  A prior wrong-SDI-pin bug was already fixed here; CIPO still read `00` afterward → loss is
  downstream of the MCU.
- **Exactly 10 CS pins are ever driven**, one at a time:
  - The only pin-drive calls in all of `src/` are `SpiManager.cpp:38-39` (init the 10 pins
    `OUTPUT`/`HIGH`), `:133` (assert one LOW), `:159` (deassert), plus the boot LED.
  - The 10 pins come from the hardcoded `panel_sets[10]` table — `src/ArenaConfig.h:32-48`:
    **`{0, 2, 5, 6, 9, 10, 23, 28, 29, 32}`**.
  - `transferFrame()` (`SpiManager.cpp:127-161`) asserts one set's CS LOW, transfers, raises it
    HIGH, then moves to the next → **at most one CS low at any instant**.
  - This is fixed at **compile time** and does **not** depend on how many panels are installed
    (no detection/enumeration). README: *"Arena hardcoded to G6_2x10 — the panel-set table and CS
    pin map are baked in."*
- Debug build (`DEBUG_SERIAL`, `pixi run deploy-printf`) clocks the panel reply back via a
  blocking `SPI.transfer(tx, rx, n)` and prints `[spi] CIPO setN cs=.. B0=.. B1=..` every 300
  frames — `SpiManager.cpp:80-86, 139-149, 163-176`. This is the readout the experiment uses.

### Panel firmware (`LED-Display_G6_Firmware_Panel`, PR #7 "align CIPO message to COPI and CS")
- The panel is the SPI **peripheral**; it only **reads** its CS pin
  (`gpio_set_function(SPI_CS_PIN, GPIO_FUNC_SPI)`, `gpio_get(SPI_CS_PIN)` for framing). It never
  drives CS.
- PR #7 writes the 3-byte CIPO confirmation directly into the PL022 TX FIFO so it clocks out
  aligned to byte 0 of the CS window (`panel_spi_custom.cpp: prime_tx_confirmation()`), with two
  `0x00` fillers so the CRC isn't truncated on TX underflow.
- The panel keeps its PL022 **continuously enabled** and does **not** tri-state SDO between
  frames — which is exactly why the arena needs per-panel tri-state buffers to gate MISO.

### Arena hardware (`LED-Display_G6_Hardware_Arena`, `arena_10-10/arena_10-10_v1`)
- `fan_out.kicad_sch` instantiates `miso_enable.kicad_sch` **×10**, one per panel position:
  P1–P5 collapse onto **`MISO_B0`**, P6–P10 onto **`MISO_B1`** (5 wired-OR buffers per bus).
- Each `miso_enable` instance (verified by parsing the schematic):
  `MISO_IN → 74LVC2G17 Schmitt (always on) → 33 Ω → 74LVC1G125 tri-state (/OE active-low, 5 kΩ
  pull-up = disabled by default) → 33 Ω → shared MISO_Bx`.
- Each buffer's `/OE` is pulled low by an **`SN74HCS08` AND-gate decode of 4 CS lines**; enables
  when **any** of its group's 4 CS is asserted. **No pull-ups on the decode inputs.** Grouping:
  P1/P6←CS_00..03, P2/P7←CS_04..07, P3/P8←CS_08..11, P4/P9←CS_12..15, P5/P10←CS_16..19.
- There are **20 CS nets** (`TNY.CS_00..19`), each to its own Teensy GPIO. The firmware drives 10
  of them; the other ~10 float into the decode.
- **Confirmed-floating decode inputs** (hard pin-adjacent match in `teensy.kicad_sch`):
  **Arduino pin 30 (`CS_15`)** and **pin 31 (`CS_14`)** — both in the P4/P9 group, not in the
  firmware's driven set.

---

## Root-cause hypothesis (to be tested)

The ~10 undriven CS lines float into the per-column MISO-enable AND-gates. A floating input that
reads low enables that column's `74LVC1G125` onto the shared `MISO_Bx` node. With multiple buffers
enabled, their push-pull outputs fight through the 33 Ω resistors and a logic-`1` settles to a
resistor-divider level (~0.6–1.1 V) below the Teensy's ~1.6 V threshold → reads `00`.

This matches the bench observation noted in `debug/measure_shared_node.py`
("U96 pin 4 drives a clean 0/3.3 V, but Teensy pin 12 reads 00") and is independent of how many
panels are physically installed.

**Alternative causes not yet excluded:** an open trace between the shared node and Teensy pin 12;
or the LPSPI receive path not latching. The scope scripts (`compare_pin4_node.py`,
`measure_shared_node.py`) distinguish these.

---

## Cheap firmware experiment — "drive the idle CS lines HIGH"

Holding every idle CS line HIGH (deasserted) makes the per-column enables strictly one-hot
(only the one firmware-driven CS goes low at a time), which should eliminate the floating-CS
contention. The change is **purely additive, reversible, and cannot affect the display path.**

### Prerequisites
- Arena with the Teensy 4.1 + at least one G6 panel installed and powered (panel running current
  firmware, e.g. PR #7).
- Build/flash via pixi (`pixi run deploy-printf`, `pixi run monitor`); traffic via
  `python scripts/all_on.py`. See `debug/README.md` for the CIPO readout interpretation.

### Step 0 — Baseline (reproduce)
1. `pixi run deploy-printf`
2. `pixi run monitor` (separate terminal)
3. `python scripts/all_on.py`
4. **Record** the `[spi] CIPO setN ...` lines. Expect `B0=00 00 00` on addressed sets = the bug.
   (`debug/README.md`: all-zero = line held low at the Teensy.)

### Step 1 — Get the exact idle-CS pin list (authoritative)
Run KiCad's netlister on the hardware repo (locally):
```
kicad-cli sch export netlist \
  arena_10-10/arena_10-10_v1/arena_10_of_10_v1r1.kicad_sch -o cs.net
```
From `cs.net`, list the Teensy `U1` pin on each net named `CS_00 … CS_19` ("… fan out to Px"),
then compute:

> **idle_cs_pins = { Teensy GPIOs on CS_00..19 }  −  { 0, 2, 5, 6, 9, 10, 23, 28, 29, 32 }**

Known members already: **30, 31**. Expect ~10 total. (Committing `cs.net` to the branch lets the
exact remaining pins be read off directly.)

### Step 2 — Minimal firmware change (`src/SpiManager.cpp`, `begin()`)
```cpp
// Idle CS lines: wired into the per-column MISO-enable decode in hardware but not
// used by the 2x10 panel-set table. Held HIGH (deasserted) so a floating input
// can't spuriously enable a column's MISO buffer and contend on MISO_Bx.
// Fill from Step 1 (kicad netlist): CS_00..19 GPIOs minus panel_sets pins.
constexpr uint8_t idle_cs_pins[] = { 30, 31, /* …the rest from Step 1… */ };

void SpiManager::begin() {
  // … existing setMISO + SPI.begin loop, and the panel_sets CS init loop …
  for (uint8_t p : idle_cs_pins) {        // NEW
    pinMode(p, OUTPUT);
    digitalWriteFast(p, HIGH);
  }
}
```
**Keep-out — do NOT add these (confirmed functional from the PCB U1 map):** SPI B0 = 11/12/13,
SPI B1 = 26/27/1, analog = 14/15, I²C = 18/19, level translators = 35/37.

### Step 3 — Build & flash
`pixi run deploy-printf` → `python scripts/all_on.py` → `pixi run monitor`.

### Step 4 — A/B verdict
| `[spi] CIPO` now shows | Meaning |
|---|---|
| `B0=?1 30 ??` (panel echo) | **Hypothesis confirmed.** Floating CS → contention; firmware mitigation works. |
| still `00` | Not (only) floating CS. Cause is downstream: open trace to pin 12, or LPSPI receive. Go to scope tests / loopback. |

### Step 5 — Positive control (proves the mechanism, pure firmware)
Temporarily, for one capture frame, assert a **second** B0 set's `cs_pin` LOW at the same time as
the target (two buffers enabled on `MISO_B0`). A previously-good echo should collapse back to
`00`, directly demonstrating wired-OR contention. Revert afterward.

### Step 6 — Decision
- Works → keep the idle-CS init (cheap firmware fix); log the durable hardware fix for the next
  board spin (one-hot decoder / 8:1 mux per bus + defined idle level on the shared node).
- Doesn't work → idle-CS init is harmless; move to open-trace / receive-path tests with the AD3
  (`compare_pin4_node.py`).

### Notes & limits
- No extra hardware for Steps 0,1,4,5 — the Teensy is both stimulus and reader via the
  `DEBUG_SERIAL` CIPO readback. Scope only if Step 4 fails.
- Teensy reads MISO as logic 0/1 only (pins 12/1 are not ADC-capable), so this is a binary
  works/doesn't — enough to confirm cause via A/B + Step 5, but not to *measure* the divider
  voltage (use `compare_pin4_node.py` for that).
- The change can't disturb the display (COPI) path.

---

## Durable hardware fix (for the next board revision)

Don't wired-OR independent tri-state buffers. Either:
1. **Rely on the peripheral to tri-state** — wire panel CIPOs directly to `MISO_Bx` + one pull at
   the MCU. Only valid if the RP2350 PL022 slave actually releases SDO on CS-high (the arena
   designers assumed it does not — verify before relying on it).
2. **Multiplexer (recommended)** — one 8:1 mux per bus (`74HC151` / CBT bus-switch), select =
   encoded active-column index. One-hot by construction; contention impossible.
3. **Keep the tri-states but make enables provably one-hot** — drive each `/OE` from exactly one
   CS, hold all idle CS HIGH (pull-ups + firmware), and add one defined pull on each shared node.

---

## References
- Controller firmware: `src/SpiManager.cpp`, `src/ArenaConfig.h`, `src/constants.h`; debug
  scripts `debug/README.md`, `debug/measure_shared_node.py`, `debug/compare_pin4_node.py`,
  `debug/spi_capture.py`.
- Panel firmware: `reiserlab/LED-Display_G6_Firmware_Panel` PR #7
  (`panel/src/panel_spi_custom.cpp`, `messenger.cpp`).
- Hardware: `reiserlab/LED-Display_G6_Hardware_Arena`,
  `arena_10-10/arena_10-10_v1/{fan_out,miso_enable,column_buffer,teensy}.kicad_sch` and
  `arena_10_of_10_v1r1.kicad_pcb`.
