# G6 Arena Slim

G6 Arena Controller firmware for the **arena_10-10 v1** hardware (Teensy 4.1, 20 panels in a 2x10 grid, two SPI buses, G6 v1 panel protocol).

Structurally based on G4.1-ArenaSlim: single main loop, no QP framework, same response framing, same G4 host command opcodes (with G6-dropped commands rejected).

Current capabilities:

- **G4 display Modes 2, 3, 4, and 5.** Mode 5 streams full arena frames over TCP/USB; Modes 2/3/4 play `.pat` files from the SD card (open-loop auto-advance, host-commanded frame, and AIN0 closed-loop velocity).
- **Two command transports** — TCP (port 62222) and USB-CDC serial — sharing one parser and command set.
- **`get-controller-info` (0xC2)** capability handshake and a **controller error display** ("CE / NN" glyph) for SD/CRC/parameter faults.
- **`get-health` (0xCA)** read-only telemetry (loop/SD/SPI/USB counters, reset cause) plus a **reset-surviving breadcrumb** for soak-testing the Mode-3 streaming wedge (issue #50) — see [Health + breadcrumb](#health--breadcrumb-get-health-0xca).
- **`get-firmware-version` (0xCB)** compiled-in **build identity** (git SHA, branch, dirty flag, UTC build date, arena rows×cols) so any controller can be pinned to the exact build it runs — see [Build identity](#build-identity-get-firmware-version-0xcb).
- **Telemetry ring (`SET_TELEMETRY` 0xA8 / `GET_TELEMETRY_BLOCK` 0xA9)** — a 64 KiB reset-surviving OCRAM event log (every command with its receive time + reply status, every displayed frame change with SD-load and SPI times, every state change / error glyph / slow SD read; overwrite-oldest, heap-guarded) drained live over USB-CDC with framed chunks and an ack cursor — the last seconds before a hang, without touching the SD card — see [Telemetry ring](#telemetry-ring-set_telemetry-0xa8--get_telemetry_block-0xa9).
- **Arena hardcoded to G6_2x10** — the panel-set table and CS pin map are baked in. Multi-arena lookup via [`g6_arena_configs.h`](https://github.com/reiserlab/Modular-LED-Display/blob/main/docs/development/g6_arena_configs.h) is deferred.
- **10 MHz SPI**, MSB-first, **CPOL=1 / CPHA=1 (Mode 3)** per [`g6_01-panel-protocol.md`](https://github.com/reiserlab/Modular-LED-Display/blob/main/docs/development/g6_01-panel-protocol.md) § SPI framing. (Panels accept up to 30 MHz; the clock is held at 10 MHz during bring-up — see `spi_clock_speed` in `constants.h`.)
- **G6 v2 `.pat` format** ([`g6_04-pattern-file-format.md`](https://github.com/reiserlab/Modular-LED-Display/blob/main/docs/development/g6_04-pattern-file-format.md)) is the on-disk file format the SD reader consumes.
- **No CIPO confirmation validation.** Panel echoes are clocked out on every transfer but not inspected.

> **Docs live in a separate repo.** The protocol/spec documents referenced here are in
> [`reiserlab/Modular-LED-Display`](https://github.com/reiserlab/Modular-LED-Display) under `docs/development/`, not in this firmware repo. Links below are absolute.

## Quickstart

**You only need [pixi](https://pixi.sh/) installed.** Everything else — PlatformIO, the Teensy
compiler, Python — is fetched automatically the first time you run a `pixi run` task. The one
optional extra is a **Chromium-based browser** (Chrome / Edge / Opera / Brave) for the
[Arena Console](https://reiserlab.github.io/webDisplayTools/arena_console.html) (a browser-based
Web Serial control panel) in step 4.

1. **Connect the controller.** Plug the Teensy 4.1 on the G6 arena into your computer via USB.
2. **Flash the controller firmware:**
   ```
   pixi run deploy-10-10
   ```
3. **Install the panels.** Add G6 panels running the most recent panel firmware from
   [`reiserlab/LED-Display_G6_Firmware_Panel`](https://github.com/reiserlab/LED-Display_G6_Firmware_Panel).
4. **Open the [Arena Console](https://reiserlab.github.io/webDisplayTools/arena_console.html)**
   in a **Chromium-based** browser (Chrome / Edge / Opera / Brave — Web Serial is not in Firefox
   or Safari).
5. **Connect to the arena.** Click **Connect to G6 Arena** and pick the correct serial port from
   the chooser (typically `/dev/ttyACM*` on Linux, `COM*` on Windows, `/dev/cu.usbmodem*` on
   macOS). Close any other program holding the port (e.g. `pixi run monitor-10-10`) first.
6. **Light it up.** Click any command — **All On** is a good first test. From there try the
   other buttons, the **Stream** presets, or upload a `.bin`/`.pat`.

## Build

The toolchain (PlatformIO, the Teensy compiler, Python, …) is installed automatically by pixi the
first time you run a task — you don't install any of it yourself.

```
pixi run build-10-10     # compile (arena_10-10)
pixi run deploy-10-10    # compile and upload (arena_10-10)
pixi run monitor-10-10   # USB serial monitor (logs to log/)

pixi run build-12-18     # compile (arena_12-18)
pixi run deploy-12-18    # compile and upload (arena_12-18)
pixi run monitor-12-18   # USB serial monitor (logs to log/)

pixi run build-2-10      # compile (G6_2x10 -- the CSHL course controllers)
pixi run deploy-2-10     # compile and upload (G6_2x10)
pixi run monitor-2-10    # USB serial monitor (logs to log/)
```

Each hardware target also has a `-performance` variant (`deploy-10-10-performance`,
`deploy-12-18-performance`, `deploy-2-10-performance`) that builds without `DEBUG_SERIAL`.

## Source files

All source files live in `src/`.

| File | Purpose |
|------|---------|
| `main.cpp` | Setup, main loop, interrupt priorities, SD mount |
| `MessageSource.h` | Abstract command-source / response interface |
| `NetworkManager.h/.cpp` | TCP server, G4 binary protocol parsing, response buffer |
| `SerialManager.h/.cpp` | USB-CDC command source (same framing as TCP) |
| `SpiManager.h/.cpp` | Dual SPI bus setup, panel-set table iteration, parallel transfers |
| `SdManager.h/.cpp` | SD mount, `/patterns/*.pat` listing, v2 header + frame CRC validation |
| `CommandProcessor.h/.cpp` | Arena state machine, command dispatch, Mode 2/3/4/5 service, refresh timer |
| `Crc.h` | CRC-8/AUTOSAR (header) + CRC-16/CCITT (per-frame) |
| `ErrorGlyph.h/.cpp` | Composes the 20x20 "CE / NN" controller error frame |
| `G6PanelProtocol.h` | G6 v1 header/parity, opcodes, block sizes |
| `ArenaConfig.h` | Hardcoded G6_2x10 panel-set table |
| `constants.h` | Hardware constants, panel geometry, timing, SD/Mode-4/error constants |
| `commands.h` | `ArenaCommands` enum (G4-compatible, G6-dropped commands marked) |
| `Version.h` | Build identity constants (git SHA / branch / dirty / date) injected by `scripts/build_version.py`; reported by `GET_FIRMWARE_VERSION` (0xCB) |
| `Telemetry.h/.cpp` | 64 KiB OCRAM telemetry ring (CMD / FRAME / STATE records), crash-dump keep-or-init at boot, ack-cursor drain for `SET_TELEMETRY` (0xA8) / `GET_TELEMETRY_BLOCK` (0xA9) |

## Host command protocol

Same wire framing as G4.1-ArenaSlim, accepted on both TCP and USB serial:

- **Incoming binary:** `[length, cmd, params...]`
- **Incoming stream:** `[0x32, len_lo, len_hi, frame_data...]` — no `analog_x`/`analog_y` bytes (G6 dropped these)
- **Response:** `[length, status(0=ok), echo_cmd, payload...]` — `payload` is an ASCII message for most commands, or raw bytes for machine-readable ones (e.g. `GET_CONTROLLER_INFO`)

Unknown opcodes reply with `status = 1` and raise a `CE 01` error glyph.

The full, current opcode list (host→controller and controller→panel) lives in
[`g6_03-controller.md`](https://github.com/reiserlab/Modular-LED-Display/blob/main/docs/development/g6_03-controller.md#command-registry)
§ Command Registry — not duplicated here, so it can't drift out of sync with `commands.h` the
way an inline table would. `commands.h` is the source of truth for opcode values; the spec doc
tracks it and is updated alongside firmware changes.

### Display modes

| Mode | Name | Behavior |
|---|---|---|
| 2 | Open loop | Load frames from SD and auto-advance at `frame_rate` |
| 3 | Show frame | Host sets the frame index via `SET_FRAME_POSITION` |
| 4 | Closed loop | Integrate AIN0 (D14) velocity at 500 Hz to advance frames (`fps = V · gain/10`) |
| 5 | Streaming | Host streams raw arena frames (the `0x32` path); no SD access |

Mode is selected by the `TRIAL_PARAMS` payload (`mode`, `pattern_id`, `frame_rate`, `gain`, `init_pos`; see `commands.h` / `CommandProcessor.cpp` for the byte layout, which is still being reconciled with the host). Mode 1 (TSI Position Function) is **not** implemented — it is a v2 / PSRAM feature.

### Health + breadcrumb (`GET_HEALTH`, 0xCA)

[Issue #50](https://github.com/reiserlab/LED-Display_G6_Firmware_Arena/issues/50): during Mode-3
host streaming (`SET_FRAME_POSITION` 0x70 at 100–286 Hz over USB-CDC) the controller sometimes
degrades from ~2 ms to 100–500 ms per command and never recovers without a power cycle.
`GET_HEALTH` is the read-only, O(1), no-SD-I/O opcode a soak harness polls (~1 Hz, and after a
fault) to see the controller's side of that. Advertised by **capability bit 7 (`0x80`)** in
`GET_CONTROLLER_INFO` (0xC2). Request `[01 CA]`; framed reply `status 0` + a 66-byte
little-endian payload (`src/Health.h`, `CommandProcessor::handleGetHealth`; Python decoder in
`tests/test_health.py`):

| Off | Type | Field | Meaning |
|---|---|---|---|
| 0 | u8 | `ver` | payload schema version, `1` |
| 1 | u8 | `flags` | bit0 `sd_mounted`, bit1 `pattern_open`, bit2 `display_active` (state ≠ ALL_OFF), bit3 `breadcrumb_valid` |
| 2 | u32 | `uptime_ms` | `millis()` |
| 6 | u32 | `loop_count` | `loop()` iterations since boot |
| 10 | u32 | `loop_max_us` | longest `loop()` iteration since boot |
| 14 | u32 | `loop_max_1s_us` | longest iteration in the most recent *completed* 1 s window (rolling, firmware-maintained) |
| 18 | u32 | `sd_reads` | `SdManager::readFrame()` calls since boot |
| 22 | u32 | `sd_read_max_us` | longest `readFrame()` since boot |
| 26 | u8 | `sd_err` | SdFat card `errorCode()` (0 = none; the driver's cached last error, not a card query) |
| 27 | u32 | `sd_err_data` | SdFat card `errorData()` |
| 31 | u32 | `frames_sent` | frames pushed to panels (same counter as `GET_FRAMES_SENT` 0x33) |
| 35 | u32 | `isr_count` | refresh-timer ISRs since boot |
| 39 | u32 | `cmd70_count` | `SET_FRAME_POSITION` commands received since boot (rejected ones included) |
| 43 | u8 | `state` | `ArenaState` (0 ALL_OFF, 1 ALL_ON, 2 STREAMING_FRAME, 3 OPEN_LOOP, 4 SHOW_FRAME, 5 CLOSED_LOOP, 6 PSRAM_PLAY, 7 ERROR_DISPLAY) |
| 44 | u16 | `cur_frame` | current frame index |
| 46 | u32 | `reset_cause` | `SRC_SRSR` captured once at boot, then cleared so the next boot reports only its own cause |
| 50 | u8 | `prev_breadcrumb` | previous boot's `last_op` (see codes below; 0 = idle / none) |
| 51 | u32 | `prev_breadcrumb_us` | `micros()` stamp of that op in the previous boot |
| 55 | u8 | `prev_breadcrumb_arg` | opcode when `prev_breadcrumb` = 4 (command dispatch), else 0 |
| 56 | u8 | `prev_slow_op` | previous boot's single slowest op |
| 57 | u32 | `prev_slow_us` | …and its duration |
| 61 | u8 | `slow_op` | this boot's single slowest op so far |
| 62 | u32 | `slow_us` | …and its duration |

Breadcrumb op codes (`Health::LastOp`): `0` idle, `1` SD `readFrame`, `2` SPI `transferFrame`,
`3` USB-CDC response write (`flushResponses`), `4` command dispatch (opcode in the arg byte),
`5` SD `openPattern`.

Semantics:

- **Nothing is clear-on-read.** Counters and maxima are cumulative since boot; a poller diffs
  successive samples. `loop_max_1s_us` is the only rolling value.
- **Breadcrumb.** Each potentially blocking call site stores its op code + `micros()` immediately
  before the call and resets to idle after (innermost call wins when nested; the dispatch mark
  brackets every handler). The record lives in uninitialized OCRAM (one cache line just below
  PJRC's `CrashReport` area, flushed on every write) with a magic word + checksum, so it
  **survives `SYSTEM_RESET` (0x01, `SCB_AIRCR` SYSRESETREQ) and a CPU lockup reset, but not a
  power-on** — after a power cycle `breadcrumb_valid` is 0 and every `prev_*` field is 0. Because
  a host-commanded reset is itself a dispatched command, `prev_breadcrumb` after a 0x01 reads
  `4` / `0x01`; the `*slow_op` / `*slow_us` fields carry the "what was wedging" answer.
- Hot-path cost per mark/clear is a few stores, one `micros()`, and a one-line dcache flush;
  `handleSetFramePosition`'s SD/timer logic is untouched beyond the counter and marks.

### Watchdog, sub-op / ISR breadcrumbs, CrashReport passthrough (diagnostic build, 2026-09-12)

The #50 wedge reproduced on the ring build (`c47ee68`, 15:44 ET) and was captured through the
bootloader route: breadcrumb `OP_CMD`/`0x70` stamped 3.3 ms after the last `FRAME` record, no
STATE records or slow SD reads in the tail, total USB silence, **no reboot** (so not a plain CPU
fault — the core's fault handler reboots 8 s later). `clear()` after a nested op leaves `OP_IDLE`,
so `OP_CMD`/`0x70` means the loop stopped **between `Health::mark(OP_CMD)` in
`handleBinaryCommand` and `Health::mark(OP_SD_READ)` in `loadFrame`** — the `handleSetFramePosition`
preamble (`patternOpen` check, `spi_.disarmRefreshTimer()` = `IntervalTimer::end`, index store) — OR
the main loop was preempted forever by an ISR-level spin / an interrupts-disabled wait / LOCKUP.
This commit adds three things so the next occurrence answers that:

**1. Hardware watchdog (RTWDOG / `WDOG3`, 2 s).** `Health::watchdogBegin()` runs LAST in `setup()`;
`Health::loopTick()` refreshes it every `loop()` iteration — **never from an ISR**; the bounded
main-loop spins that can outlast 2 s (`sendRaw`'s 5 s/2 s waits, `drainBulkData` up to 15 s on a
rejected 0x85/0xE0, the synchronous 0xE0 upload loop with its 30 s idle deadline) kick from inside,
deadlines unchanged. The one **unbounded** spin, `SpiManager::transferPanelSet`'s
`while (!dmaComplete_)`, is deliberately NOT kicked — if the SPI DMA completion ever fails to
arrive, the watchdog is what gets us out, and `wdog_pc` will point at it. On expiry the
RTWDOG asserts a system reset through the SRC (`SRC_SRSR` bit 7 `wdog3_rst_b`, already in
`GET_HEALTH.reset_cause` and in the telemetry `STATE(boot).code`), which is the same reset domain as
`SYSRESETREQ`: OCRAM2 (`0x20200000..`) is not powered down, so the **telemetry ring and the
breadcrumb survive exactly as they do for `SYSTEM_RESET`** (PJRC's CrashReport relies on the same
property). Bench verification: `SET_TELEMETRY` flags **bit4 + bit5 = starve** stops the kicks (the check lives inside
`watchdogKick()`, so every kick path honours it) → reset in 2 s → `GET_HEALTH` must show
`wdog_flags` bit1 (previous reset was the watchdog), bit2 (PC captured), `breadcrumb_valid`, and the
ring's `boot_count` incremented with the pre-reset records intact. **Clock gate — the bench finding on the first flash (`wdog_flags = 0x49`, config failed):** the RTWDOG's
bus clock is gated at boot (`imxrt.h`: *WDOG3 requires `CCM_CCGR5_WDOG3`*) and the Teensy core's
`startup.c` never opens it, so every unlock/CS write was silently ignored. `rtwdogProgram()` now sets
`CCM_CCGR5 |= CCM_CCGR5_WDOG3(ON)` first, records the CS reset default once (`wdog_cs_boot`), then
follows the NXP order — unlock key(s) → `TOVAL`, `WIN`, `CS` **immediately** (no polling inside the
128-bus-clock window) → wait `RCS` (≤ 2 LPO clocks) — and verifies the readback (`EN`, `TOVAL`)
before declaring success; the refresh key width follows the `CMD32EN` bit actually in effect
(32-bit `0xB480A602`, or `0xA602`/`0xB480`). Programming: `CS =
CMD32EN | CLK(LPO 32 kHz) | PRES(/256) | UPDATE | INT | EN`, `TOVAL = 250` (125 Hz ticks → 2.000 s),
unlock `0xD928C520` (or the 16-bit pair `0xC520`, `0xD928` when CMD32EN is clear);

**Fourth bench round (320e26d) — two points fit a line; the 500 Hz conclusion was wrong.** `TOVAL`
1000 expired **6.37 s** after the last kick; `TOVAL` 254 had expired in 0.52 s. Slope
(1000 − 254)/(6.37 − 0.52) = **127.5 Hz** — the comparator runs at exactly the rate `WDOG3_CNT` reads
(32.768 kHz ÷ 256; the CNT measurement was right) — and intercept **≈ 190 ticks (~1.5 s)**: expiry ≈
(`TOVAL` − 190)/127.5 s, as if the counter already held ~190 ticks at the moment kicks stop. The
mechanism is not identified yet (does the refresh not zero the counter? are refreshes only honoured
intermittently — e.g. one per N counter ticks?), so this build (a) **calibrates**
`TOVAL = tick_hz × seconds + 190` with the tick rate measured at boot (`444` ≈ 2.0 s, `4000` ≈ 30 s;
fallback 127 Hz), and (b) **instruments the kick path**: `WDOG3_CNT` is read immediately before and
after every refresh and the min/max since boot are in the ver 5 tail. Expected readings if refreshes
zero the counter: `cnt_after_min/max` = 0..1 and `cnt_before_max` = the longest loop gap in ticks
(a 129 ms SD read ≈ 16). `cnt_after_max` ≈ 190 would mean the refresh does *not* zero the counter
(it resets to some base); `cnt_before_max` ≈ 190 with small `after` values would mean refreshes are
being ignored for ~1.5 s at a time. Bench check: a starve must now drop the port ≈ 2.0 s after the
reply. The paragraph below records the (wrong) intermediate conclusion for the audit trail.

**Third bench round (54b57d0) — the counter you can read is not the counter that expires.** With
the CNT-measured rate (127 Hz = 32.768 kHz ÷ 256) `TOVAL` was set to 254 for "2.0 s", yet the starve
reset still came 0.52 s after the reply — `TOVAL` 254 expiring in ≤ 0.5 s means the comparator runs at
≥ 500 Hz (= 128 kHz ÷ 256, or 32.768 kHz ÷ 64), **4× the rate `WDOG3_CNT` reads advance at**. The RT1060
RM calls `CLK = 01` the 32 kHz LPO, which matches what CNT shows but not what the timeout does; whether
CNT reads are decimated or the comparator is fed differently cannot be told from software. Conclusion
and what the firmware does: `TOVAL` is derived from the **empirical expiry rate**
`Health::watchdog_tick_hz = 500` (`1000` ticks = 2.0 s, `15000` = 30 s); the CNT measurement stays
in the health tail as a diagnostic (`wdog_tick_hz`, expect ~127). Bench check for the constant: a
starve (`SET_TELEMETRY 0x31`) must now drop the CDC port **~2.0 s** after the reply (+0.3 s to
re-enumerate); if it is 8 s the two rates are the same after all and the constant should be 127.

**Second bench round (86eeb4a) — two more findings, both fixed here.** (a) **The tick is ~500 Hz, not
128 Hz:** the "LPO" feeding the RTWDOG on this silicon is 128 kHz, so `TOVAL = 250` expired in
~0.5 s (the starve test's CDC port vanished 0.5 s after the reply) — short enough to clip SdFat's
1 s busy timeouts. `Health::begin()` now arms the RTWDOG early with a long provisional `TOVAL`
(`0xFFFF`, > 2 min) so the whole boot is protected but never clipped, and `watchdogBegin()` (end of
`setup()`) measures the counter (`WDOG3_CNT` edge-synced, ≥ 64 ticks or 400 ms) — reported as
`wdog_tick_hz` — and programs `TOVAL` for 2.0 s / 30 s (see the third round above for why the
measured rate is a diagnostic only and the timing uses the empirical 500 Hz). (b) **`wdog_flags` bit6 was a false negative:** `RCS` is already 1 in the reset
default and after any earlier configuration, so a spin on it returned before the reconfiguration
latched and the `TOVAL` readback still showed the old value. Success is now judged from the `EN` +
`TOVAL` readback after a 300 µs settle (RCS is advisory), with one retry using the other unlock key
width. Observed CS values: reset default `0x2520`; after programming **`0x35E0`** (CMD32EN, PRES,
RCS, CLK=LPO, EN, INT, UPDATE); after a **watchdog reset `0x31E0`** — the block keeps its registers
across the reset it causes (EN still set, RCS clear), which is why the early arm first refreshes a
still-running watchdog before reprogramming it. `UPDATE=1` keeps it reconfigurable: **runtime disable** (flags bit4 + bit6) and the
nesting-safe **long-operation window** `watchdogSuspend()/Resume()`, which re-programs `TOVAL` live
to **30 s** (3750 ticks) for the operation and back to 2 s afterwards — the watchdog is **never
fully off** during SD format / ISP / image upload, only slower. Every reprogramming (unlock →
TOVAL → CS → RCS) is a few µs with IRQs masked and refreshes the counter immediately. Both spins
are bounded — a mis-programmed RTWDOG can never brick boot. If a reprogramming FAILS the hardware
state is unknown: it is reported as still armed (`wdog_flags` bit6 "last reprogramming failed"),
`watchdogSuspend/Resume/SetEnabled` return false, and the caller proceeds anyway after appending a
telemetry `STATE(telemetry, code 0xEE, arg = opcode)` so the trace shows it.
Compile-time switch `HEALTH_WATCHDOG` (default 1). **Pre-reset PC capture:** `INT=1` raises
`IRQ_RTWDOG` (priority 0) 128 bus clocks before the reset; the naked ISR stores the stacked
**PC/LR of the preempted context** (the hung main loop, or the ISR that was spinning) into the
**separate ISR/watchdog record** (`wdog_pc`/`wdog_lr`/`wdog_stamp_us`, `wdog_fired = 1`,
`isr_last = 3`) and seals it. Limits: an equal-priority
ISR (the LPSPI IRQs are also priority 0) or an interrupts-disabled spin cannot be preempted — then
only the reset happens and `isr_last`/`prev_breadcrumb` carry the answer. Long synchronous handlers
open the 30 s window for their dispatch: `PURGE_MEMORY` 0x8F (SD format), `G6_PROGRAM_PANEL` 0xC8 /
`G6_VERIFY_PANEL` 0xC9 (ISP), `GET_SD_ARCHIVE` 0x8A (entry collection), `SET_FIRMWARE_FILE` 0xE0
(synchronous image upload). Everything else must finish in well under 2 s — a 129 ms SD read is
fine. **Watchdog policy bits are applied only when `SET_TELEMETRY` bit4 ("watchdog bits present")
is set**; a plain logging enable/disable (`0x01` / `0x00`) never touches the watchdog.

**2. Finer breadcrumbs.** Sub-ops inside the 0x70 handler, `op_arg = 0x70`: `6 OP_CMD_DISARM`
(around `disarmRefreshTimer`), `7 OP_CMD_PRELOAD` (before `loadFrame`), `8 OP_CMD_ARM`
(`armRefreshTimer`), `9 OP_CMD_RESPOND` (`sendResponse`). The main-loop breadcrumb keeps its v1
layout. **ISR / watchdog record**: a SEPARATE 32-byte OCRAM line at `0x2027FF20` (below the
breadcrumb at `0x2027FF40`) with its own magic `'H6IR'` + checksum — `isr_last` (set at entry /
cleared at exit of `SpiManager::refreshISR` = 1 and `SpiManager::dmaISR` = 2, 3 = watchdog ISR),
`isr_count`, `wdog_fired`, `wdog_pc`, `wdog_lr`, `wdog_stamp_us`. It is written and sealed ONLY
from interrupt context (under a brief IRQ mask so nested ISRs cannot leave a stale checksum) and
harvested at boot independently of the breadcrumb: a main loop caught mid-`mark()` (checksummed
fields dirty) can no longer invalidate the PC capture, and vice versa. SdFat/USB ISRs live in the
core and are not hooked.
`GET_HEALTH` is now **ver 5, 114 B** — offsets never move; tails appended (ver 2: 66–88, ver 3: 89–96, ver 4: 97–105, ver 5: 106–113):

| Off | Type | Field | Meaning |
|---|---|---|---|
| 66 | u8 | `prev_isr_last` | ISR the previous boot was inside when it died (0 none, 1 refresh, 2 dma, 3 wdog, 4 usb, 5 sdhc, 6 lpspi, 7 pit) |
| 67 | u32 | `prev_isr_count` | ISR entries in the previous boot |
| 71 | u32 | `prev_wdog_pc` | stacked PC captured by the watchdog pre-reset IRQ (valid when `wdog_flags` bit2) |
| 75 | u32 | `prev_wdog_lr` | stacked LR at that moment |
| 79 | u8 | `wdog_flags` | bit0 armed, bit1 previous reset was the watchdog, bit2 previous boot captured pc, bit3 compiled in, bit4 long-op window (30 s) active, bit5 starving (test), bit6 last RTWDOG reprogramming failed (state unknown) |
| 80 | u8 | `isr_last` | this boot: ISR currently inside (live) |
| 81 | u32 | `isr_count` | this boot: ISR entries |
| 85 | u32 | `wdog_kicks` | this boot: watchdog refreshes |
| 89 | u32 | `wdog_cs_boot` | (ver 3) raw `WDOG3_CS` as found before the first programming — this silicon's reset default, expected `0x2520` (UPDATE, CLK=LPO, RCS, CMD32EN) |
| 93 | u32 | `wdog_cs_now` | (ver 3) live `WDOG3_CS` readback: EN bit7, UPDATE bit5, INT bit6, CLK bits 8–9, RCS bit10, ULK bit11, PRES bit12, CMD32EN bit13, FLG bit14 |
| 97 | u32 | `wdog_tick_hz` | (ver 4) measured `WDOG3_CNT` tick rate — ~127 Hz on this silicon, and per the two-point bench fit also the comparator's rate (fallback 127 if unmeasurable, `wdog_verify` bit3) |
| 101 | u32 | `wdog_toval_now` | (ver 4) live `WDOG3_TOVAL` = `tick_hz × s + 190` offset ticks: `444` = 2.0 s normally, `4000` = 30 s inside a long-op window |
| 105 | u8 | `wdog_verify` | (ver 4) last programming: bit0 RCS timeout (advisory), bit1 EN mismatch, bit2 TOVAL mismatch, bit3 tick-rate measurement fell back to 127 Hz, bit4 retried with the other unlock key width |
| 106 | u16 | `wdog_cnt_before_max` | (ver 5) `WDOG3_CNT` read just **before** a refresh, max since boot — the longest gap between kicks, in ticks |
| 108 | u16 | `wdog_cnt_after_min` | (ver 5) `WDOG3_CNT` read just **after** a refresh, min since boot — 0..1 if the refresh zeroes the counter |
| 110 | u16 | `wdog_cnt_after_max` | (ver 5) … max since boot — anything large means refreshes are not zeroing / not being honoured; this is the field that should explain the ~190-tick anomaly |
| 112 | u16 | `wdog_cnt_now` | (ver 5) live `WDOG3_CNT` |

The telemetry `STATE(boot).arg` low byte gains **bit1 = previous boot ended in the watchdog ISR**
(bit0 stays `prev_valid`; high byte = previous breadcrumb op).

**3. `GET_CRASHREPORT` 0xCC.** `[01 CC]` → the raw **128 B** at `0x2027FF80..0x20280000`: PJRC's
`arm_fault_info_struct` `{len@0, ipsr@4, cfsr@8, hfsr@12, mmfar@16, bfar@20, ret@24, xpsr@28,
temp(float)@32, time@36, crc@40}` (`len` is in **words**: 11 when a fault is recorded, 0 when none) followed by PJRC's own breadcrumb
words at `0x2027FFC0`. Readable without `DEBUG_SERIAL`; **never cleared by this read**. Caveat: the
`DEBUG_SERIAL` boot banner prints `CrashReport`, and the core's `printTo` clears it — so the debug
build hands over an already-cleared record; the performance build preserves it until the next fault.
Gate on **0xCB `flags` bit 3** (`crashreport`, also implies `GET_HEALTH` ver 2) — not on bit 2: a
rollback to the ring build c47ee68 has the ring but neither 0xCC nor the v2 tail.

**Deferred (recorded, not planned for this diagnostic build):** temp-file pattern replacement;
a recovery-mode boot path / consecutive-watchdog-reset failure counter; a versioned crash envelope
wrapping breadcrumb + ISR record + CrashReport; ISR hooks bracketing core driver code (SdFat, USB,
LPSPI/DMA); dual ring headers.

**Investigated and dropped:** the ELF string `beginCycles` from `IntervalTimer::beginCycles` is the
symbol name; `cores/teensy4/debug/printf.h` compiles `printf(...)` to nothing unless
`PRINT_DEBUG_STUFF` is defined, and `objdump` shows no call out of `beginCycles` — no `_write`
override is needed. `platformio.ini`: `-DDEBUG_SERIAL` belongs to `[env:teensy41]` only; the
performance env carries no `build_flags`.

### Free-running refresh timer (wedge #5 fix candidate, 2026-09-13)

**Evidence.** Wedge #5 (23:51, build 4860fef) was the first one caught by the watchdog:
`prev_breadcrumb = OP_CMD_DISARM / 0x70`, `prev_isr_last = 3` (the core was preemptible),
`prev_wdog_pc = 0x212F4` = `IntervalTimer::end()` at `str r1, [r3, #8]` — the `channel->TCTRL = 0`
store into the PIT — `lr = 0x212EB`. Two readings fit: the **leading mechanism** is the core's `IntervalTimer::end()` race (see "The core
race" below — a null-callback PIT interrupt storm starving the main context exactly at that store);
the **alternative** is a peripheral-bus hang on the PIT write itself (USB and SD queue behind it, no
fault because nothing faults). The kind 8/9 ring records at the next capture decide between them. Independently, at 286 Hz
`SET_FRAME_POSITION` the display was starving: every 0x70 disarmed and re-armed the refresh timer,
which **restarts the refresh period** on each command, so with a 3.5 ms command interval against a
3.33 ms period the tick rarely fired before the next restart (~20 displayed frames/s).

**Change.** `SpiManager::armRefreshTimer` is now **idempotent** — it tracks the rate the PIT channel is
running at and does nothing when asked for the same rate; a different rate re-begins the channel.
A **valid, steady-state 0x70 never touches the PIT**: `handleSetFramePosition` validates,
`loadFrame`s into `frame_buf_`, sets `state_ = SHOW_FRAME`, arms only when entering SHOW_FRAME from
another state or when the refresh rate differs, and replies. `showError()` (a rejected 0x70), rate
changes and state transitions still program the PIT — now through the guarded disarm below. FRAME telemetry is unchanged (it is emitted at the
transfer). **Audit of disarm/arm sites:**

| Site | Before | Now | Why |
|---|---|---|---|
| `handleSetFramePosition` (0x70, per command) | disarm + arm | **none in steady state** (idempotent arm at state entry / rate change only; a rejected 0x70 still goes through `showError`) | the hot path; the churn that wedges #2–#5 sat in |
| `handleDisplayPsramIndex` (0x3A, per index) | disarm + arm | idempotent arm only | same per-command churn on the PSRAM path |
| `handlePsramPlay` (0x3B, per play start) | disarm + arm | idempotent arm only | buffer built synchronously in `loop()` |
| `SET_REFRESH_RATE` (0x16) | disarm + arm | arm (re-begins only if the rate changed) | rate change is the point |
| `enterPatternMode` (trial start, 0x08/0x03) | disarm + arm | **kept** (guarded) | explicit transition semantics — the display pauses while the pattern opens and the geometry (GS2/GS16) may change; not a concurrency guard (the PSRAM paths change `block_byte_count_` without disarming) |
| `enterStreamingFrame` (0x32 size/state change) | disarm + arm | kept (guarded; already gated on `need_rearm`) | explicit transition: stream geometry / rate change |
| `enterAllOn`, `showError`, `enterAllOff` (STOP) | disarm (+ arm) | kept (guarded); STOP's disarm carries the `OP_CMD_DISARM` crumb (arg 0) | explicit transitions: all-on / glyph / display off |

**ISR breadcrumb coverage (same commit).** The one interpretation-killer for a watchdog PC is an
interrupt storm at a priority the main loop cannot preempt — the core's `usb_isr` (USB-CDC), USDHC
(SdFat's SDIO), or an LPSPI IRQ. At the end of `setup()`, `wrapDriverVectors()` saves the handler
found in `_VectorsRam` for `IRQ_USB1` / `IRQ_SDHC1` / `IRQ_LPSPI3` / `IRQ_LPSPI4` and installs a thin
trampoline that marks `isr_last` = **4 usb / 5 sdhc / 6 lpspi** (and **7 pit** via `Health::wrapPitVector`, see below) on entry (nesting-safe: exit restores
the previous id) and counts entries. These use `Health::isrEnterLite`: a byte store, a count, and an
incremental XOR update of the record checksum — **no cache flush** per interrupt; the memory copy is
refreshed by the sealing refresh/DMA hooks (≥ 300 Hz) and by the watchdog ISR. A vector still pointing
at the core's `unused_interrupt_vector` is left alone (nothing attached — the `DEBUG_SERIAL` banner
prints which were wrapped; on this firmware expect usb = 1, sdhc = 1, lpspi3/4 = 0 because the SPI
DMA path completes through DMA-channel ISRs, not LPSPI IRQs). **`setupInterruptPriorities()` no longer
pins LPSPI3/4 to priority 0**: nothing attaches those vectors, so it was dead configuration, and at
priority 0 such an interrupt could not have been preempted by the watchdog IRQ (also 0) — they stay
at the core default 128.

**The core race — leading mechanism candidate (2026-09-13).** Verified in the installed core
(framework-arduinoteensy 1.160.0, `cores/teensy4/IntervalTimer.cpp`): `end()` stores
`funct_table[index] = nullptr` **before** `channel->TCTRL = 0; channel->TFLG = 1;`, and `pit_isr()`
clears a channel's `TFLG` **only** when its `funct_table` entry is non-null. If the PIT interrupt is
taken between the two stores, `pit_isr` runs with a null callback, never clears `TFLG`, and re-enters
forever at priority 128 — the main context is starved with its stacked PC exactly at the `TCTRL`
store (`0x212F4`, what wedge #5 captured). SDHC runs at priority 96 (`setupInterruptPriorities`) and
preempts the storm outright; USB (IRQ 113, priority 128) wins the equal-priority NVIC tie-break
against the PIT (IRQ 122, lower number first) between storm iterations — which is why USB stayed
enumerated and the bootloader route worked. The free-running
change above **reduces exposure** by removing the hot-path disarm (the 286 Hz × 3.3 ms lottery);
`SpiManager::disarmRefreshTimer()` now **closes the remaining sites** (`enterPatternMode`,
`enterStreamingFrame`, `enterAllOn`, `showError`, `enterAllOff`) by running `IntervalTimer::end()`
with **only the PIT masked at the NVIC** (`NVIC_DISABLE_IRQ(IRQ_PIT)` → `dsb; isb` → `end()` → `dsb` →
`NVIC_ENABLE_IRQ(IRQ_PIT)`), nothing else inside. Not PRIMASK: that would also mask the watchdog IRQ,
and if the alternative reading (a genuinely stalled peripheral store) were true we would lose the PC
capture exactly where it matters. A PIT interrupt that pends during the window runs `pit_isr`
afterwards with a null callback and an already-cleared `TFLG` — no storm.
`begin()`'s order (`funct_table` set before `TCTRL = 3`) has no such window. Two more probes make the
next capture conclusive: **ISR id 7 = PIT** — the core's `pit_isr` vector (IRQ 122) is wrapped by an
`isrEnterLite` trampoline via `Health::wrapPitVector()`, re-installed from `armRefreshTimer()` after
every `IntervalTimer::begin()` (which re-attaches `pit_isr` and would otherwise remove the wrapper);
the ISR record now keeps **per-id entry counts**, and after a watchdog reset the ring receives
`STATE(9 prev_isr_count)` per busy id (arg in units of 4096 entries — 300 Hz × 60 s ≈ 4 units, a
multi-MHz storm for 2 s ≈ 1000+). **Watchdog context** — the pre-reset IRQ also stores the stacked
xPSR and its EXC_RETURN in the ISR record (grown to 96 B at `0x2027FEC0`; `GET_HEALTH` unchanged), and
the ring receives `STATE(8 wdog_context)` with code = EXC_RETURN low byte (`0xF9` = thread mode was
preempted, `0xF1` = a handler was) and arg = IPSR in bits 0–8 (0 = thread, `138` = the PIT interrupt)
**| the `isr_last` that was active when the watchdog fired, in bits 9–15** (the watchdog IRQ itself
writes `isr_last = 3`, so the prior value is captured first — wedge #5 only told us "watchdog"). A
PIT storm would read `wdog_context code 0xF1, IPSR 138, prior isr 7` + `prev_isr_count id 7 ≫
uptime × 300 Hz`. Both the lite hooks and the full refresh/DMA hooks update the record under a
PRIMASK-preserving critical section and save/restore the previous id (SDHC at priority 96 preempts
them; a callback inside the wrapped `pit_isr` must not clobber the PIT trampoline's marker). The
record's **line 0 carries the context plus its own checksum** and is sealed and flushed **first** by
the watchdog ISR, the count lines after — a capture cut short by the reset still yields
pc/lr/xPSR/EXC_RETURN (counts then read 0). The old 32-byte record location (`0x2027FF20`, builds
fb11681–eca07f6) has its magic cleared at boot so a rolled-back firmware never harvests it as "the
previous boot"; kinds 8/9 are emitted only when **this boot's** `SRC_SRSR` says watchdog. A failed
`IntervalTimer::begin()` (no free PIT channel) leaves the timer un-armed and appends
`STATE(10 timer_fail, arg = rate)`. Decoders classify EXC_RETURN by **bit 3** (`0xF9/0xE9/0xFD/0xED`
= thread preempted, `0xF1/0xE1` = a handler; the `E` forms mean FP state was stacked, normal on this
floating-point firmware) — offline fixtures in `tests/test_telemetry_codec.py`.

**Race analysis (why the disarm was never needed for buffer safety).** `SpiManager::refreshISR`
only sets `refreshFlag`. The transfer (`transmitOnRefresh → transferFrame`, which spins for SPI-DMA
completion) and `loadFrame` (SD → `frame_buf_`) both run in `loop()`, so they are strictly serialized
by construction: a tick that lands during `loadFrame` merely defers the transfer to the next
`serviceDisplay()`. The disarm/arm pairs that remain are **explicit transition semantics** (pause the display while a
pattern opens, switch to the glyph, go dark), not concurrency protection — the PSRAM paths already
changed `block_byte_count_` without disarming and were never unsafe for that reason. No double
buffer is required. Those remaining sites now run `IntervalTimer::end()` under the guarded critical
section described under "The core race".

**Expected bench effects.** At 286 Hz commands the displayed-frame rate should rise from ~20/s to
≈ min(refresh 300 Hz, distinct requested frames ≈ 200/s). Semantics are **latest-request-wins**: the
next free-running tick transfers whatever `frame_buf_` holds; command→display latency is SD load +
loop work + the wait for the next tick + the transfer — a quantity to be **measured** from the
CMD/FRAME telemetry pairs, not a guaranteed bound. At the next wedge the watchdog PC either **moves**
(→ the alternative bus-hang reading, PC in `loadFrame`/SD or the USB write) or the wedge
**disappears** / the kind 8/9 records show the PIT storm (→ the core race). Identity: 0xCB `flags`
bit4.

### SD fast path + stall attribution (2026-09-13, 0xCB flags bit 5)

**Why.** A night of Mode-3 soak logs (fw #50 campaign, 9.9 M `SET_FRAME_POSITION`s) split the SD read
cost by index step: a sequential +1 frame costs ~620 µs (the card's open multi-block stream), any other
step ~1.18 ms on an 81 KB pattern but 1.46–2.0 ms on an 813 KB one — the extra is SdFat's
`FatFile::seekSet` re-walking the FAT chain from the first cluster on backward seeks (an extra FAT-sector
read whenever the chain spans more than one FAT sector; the ARM build keeps a separate FAT cache, so
this is a chain-length effect, not data-read eviction). Separately, the card itself stalls for
quantised 23/33/41/67/89 ms every ~24.5k reads of the large file (clusters of 3–4, card-internal
read-count maintenance; not firmware-addressable — see `webDisplayTools/docs/development/sd-read-jitter-2026-09-13.md`).

**What changed.**
- `SdManager` holds the pattern as an SdFat `FsFile` and calls `contiguousRange()` once at open: on a
  contiguous file SdFat sets `FILE_FLAG_CONTIGUOUS` and every later seek is arithmetic. A fragmented
  file keeps the old path; `STATE(sd_layout)` says which (code bit0) plus the cluster size.
- A `SET_FRAME_POSITION` for the index already in `frame_buf_` (loaded by a successful `loadFrame`,
  not overwritten since — glyph / all-on / dark / stream / PSRAM / AO-mode / AO-LUT changes invalidate
  it) is answered **without an SD read** (`Health::stats.cmd70_same_index`, RAM only). ~24 % of
  closed-loop commands repeat the index.
- `readFrame` times seek / body / CRC-trailer separately; a read over **10 ms** records `sd_slow` with
  the slowest phase in the code byte and an `sd_slow_ctx` record (SdFat `errorCode()`, `errorData()`)
  so a card hold can be told from a driver error/retry.
- FRAME records grow to **26 B** (`req_age_us`, `superseded`, `flags`; ring layout **v2** — a kept
  ring of the other version is re-initialised at boot rather than truncated). `STATE(sd_reads)` at
  STOP / the next trial start reports the read count of the pattern being left (the host cannot
  derive it once same-index commands skip the read).

### SD card identity (`GET_SD_INFO`, 0xCD)

Every stall measurement and card comparison must be attributable to a specific card. Request `[01 CD]`;
framed reply status 0 (1 when no card is mounted — the payload is still sent) + a **30-byte** payload,
O(1) (SdFat caches CID/CSD at mount; no card traffic). Gated on 0xCB flags bit 5.

| Off | Type | Field | Meaning |
|---|---|---|---|
| 0 | u8 | `ver` | payload schema version, `1` |
| 1 | u8 | `flags` | bit0 mounted, bit1 CID valid, bit2 CSD valid |
| 2 | u8 | `card_type` | SdFat `type()`: 0 none, 1 SD1, 2 SD2, 3 SDHC/SDXC |
| 3 | u8 | `fat_type` | 12 / 16 / 32, 64 = exFAT, 0 unknown |
| 4 | u32 | `sectors` | capacity in 512 B sectors (CSD) |
| 8 | u32 | `bytes_per_cluster` | volume cluster size |
| 12 | u8[16] | `cid` | raw CID register: MID @0, OID @1–2, PNM @3–7, PRV @8, PSN @9–12 (BE), MDT @13–14, CRC @15 |
| 28 | u8 | `sd_status_maint` | SD 6.0 §4.18 maintenance-support bits (SD_STATUS b328/329); `0xFF` = not read (SdFat 2.1.2 has no ACMD13 reader) |
| 29 | u8 | `sd_diag` | `SET_SD_DIAG` (0xCE): bit0 legacy seek requested (next open skips `contiguousRange()` → FAT-chain-walking seeks), bit1 no same-index skip, **bit2 legacy seek applied to the open file**. Bench A/B switches for the causal test of the card stalls; both OFF at boot; each arm switch also leaves a `STATE(telemetry, code 0xCE, arg = flags)` marker and every `sd_layout` record carries bits 2/3 |

Arena Studio reads it at link-up into `run_metadata.sd_card`; `tests/test_firmware_version.py` decodes it.

### SD fast-path A/B switches (`SET_SD_DIAG`, 0xCE) — bench only

`[02 CE flags]`, reply status 0 + the flags in force (1 B). bit0 **legacy seek**: the next `openPattern` skips
`contiguousRange()`, so `FatFile::seekSet` walks the FAT chain again (the pre-fast-path behaviour); bit1 **no
same-index skip**: every `SET_FRAME_POSITION` reads its frame. Both OFF at boot; bits 2–7 refused. Each switch
appends `STATE(telemetry, code 0xCE, arg = flags)`; every later `sd_layout` record carries bits 2/3;
`GET_SD_INFO` byte 29 reports the flags (bits 0–1 requested, **bit 2 = legacy seek applied to the currently open
file** — `openPattern` reopens a same-pattern restart when the requested seek mode differs). On exFAT volumes the
library sets the contiguous flag at open by itself, so the legacy arm only reproduces the chain walk on FAT16/32
(`sd_layout` bit1). Purpose: the causal test of the card stalls on one build without reflashing (webDisplayTools
`docs/development/sd-stall-causal-test-plan-2026-09-13.md`). **Gate on 0xCB flags bit 6** (`sd_diag`), not bit 5.

### Build identity (`GET_FIRMWARE_VERSION`, 0xCB)

Every build embeds the git identity of the checkout it was compiled from, so a controller in the
field can be pinned to an exact build (issue #50 could not be: `GET_CONTROLLER_INFO` 0xC2 carries
only a protocol version byte — always `1` — plus the capability bitmap and MAC, and 0xE3 describes
the *panel* image on the SD card, not the controller). Request `[01 CB]`; framed reply `status 0`
+ a **46-byte** payload. Read-only, O(1), no SD I/O — every field is a compile-time constant
(`src/Version.h`, `CommandProcessor::handleGetFirmwareVersion`; Python decoder in
`tests/test_firmware_version.py`). ASCII fields are right-padded with spaces, never NUL-terminated:

| Off | Type | Field | Meaning |
|---|---|---|---|
| 0 | u8 | `ver` | payload schema version, `1` |
| 1 | u8 | `rows` | `panel_count_per_frame_row` this build was compiled for |
| 2 | u8 | `cols` | `panel_count_per_frame_col` |
| 3 | u8 | `flags` | bit0 `dirty` (tracked files modified at build time), bit1 `debug` (`DEBUG_SERIAL` build), bit2 `telemetry` (telemetry ring compiled in — `SET_TELEMETRY` 0xA8 / `GET_TELEMETRY_BLOCK` 0xA9 answer; **hosts gate 0xA8 on this bit**, not on 0xC2 bit 7), bit3 `crashreport` (`GET_CRASHREPORT` 0xCC + `GET_HEALTH` ver ≥ 2 present; **hosts gate 0xCC on this bit** — a rollback to c47ee68 has bit2 but not bit3), bit4 `freerun_refresh` (free-running refresh timer: a valid steady-state `SET_FRAME_POSITION` never disarms/re-arms the PIT; transitions/glyph/rate changes use the guarded disarm — variant marker so run logs can tell which build ran), bit5 `sd_fastpath` (contiguous-file O(1) seeks, same-index `SET_FRAME_POSITION` skips the SD read, FRAME records 26 B = ring v2, STATE kinds 11–13, `GET_SD_INFO` 0xCD answers — **hosts gate 0xCD on this bit**), bit6 `sd_diag` (`SET_SD_DIAG` 0xCE + STATE kind 14 — **hosts gate 0xCE on this bit**) |
| 4 | char[8] | `sha` | short git SHA, lowercase hex (`git rev-parse --short=8`); `unknown ` when git was unavailable |
| 12 | char[10] | `date` | build date, UTC, `YYYY-MM-DD` |
| 22 | char[24] | `branch` | git branch, truncated to 24; `detached` for a detached HEAD; `unknown` when unavailable |

How it gets in: `scripts/build_version.py` is a PlatformIO `pre:` extra script (listed in
`platformio.ini` after the USB-string and port-finder scripts). On every `pio run` it runs
`git rev-parse --short=8 HEAD`, `git rev-parse --abbrev-ref HEAD`, and
`git status --porcelain --untracked-files=no` (untracked files do not make a build dirty), stamps
the UTC date, and appends `FW_GIT_SHA` / `FW_GIT_BRANCH` / `FW_BUILD_DATE` / `FW_GIT_DIRTY` as
`-D` macros; `src/Version.h` provides `"unknown"` fallbacks so a build without git still compiles.
Because pre: scripts re-run on every build, **rebuilding after a commit picks up the new SHA** —
and because the macros are on the compile command line, that rebuild is a full one (SCons
re-compiles every object when its command changes), not incremental. The `DEBUG_SERIAL` boot
banner prints the same identity (`=== G6 arena controller fw <sha> (<branch>) built <date> … ===`).

Hosts: Arena Studio reads 0xCB at connect and records it in every run log as
`run_metadata.firmware`. 0xCB is gated by the same **capability bit 7 (`health`)** in 0xC2 as
`GET_HEALTH` — both shipped in the same build, and older firmware flashes a `CE 01` error glyph
on any unknown opcode, so a host must check the bit before asking.

### Telemetry ring (`SET_TELEMETRY` 0xA8 / `GET_TELEMETRY_BLOCK` 0xA9)

The breadcrumb above says what the controller was doing at the instant it wedged; the telemetry
ring says what led up to it — and, in normal runs, gives the exact controller-side timing of
every command and every displayed frame (the data the
[ring-buffer proposal](https://github.com/reiserlab/webDisplayTools/blob/main/docs/development/controller-telemetry-ring-buffer-proposal.md)
§ 3.1 asks for). Every dispatched command, every displayed frame *change*, and every state
transition appends a small binary record to a **64 KiB byte ring in OCRAM**; a host drains it live
over the single USB-CDC link with **framed, chunked replies ("framing A")** and an **ack cursor**,
so the drain is lossless regardless of link hiccups. Nothing here touches the SD card. Both
opcodes were reserved by fw PR #47 for exactly this stream. There is no 0xC2 capability bit for
them (the byte is full; bit 6 is `ai_cal`, bit 7 `health` is the last) — a host must gate
`SET_TELEMETRY` on **`GET_FIRMWARE_VERSION` (0xCB) `flags` bit2 `telemetry`**: health-only firmware
(22b756d) answers an unknown 0xA8 with a `CE 01` error glyph on the arena.
Source: `src/Telemetry.h` (layout, all constants), `src/Telemetry.cpp`,
`CommandProcessor::handleSetTelemetry` / `handleGetTelemetryBlock`; Python codec in
`tests/telemetry_codec.py`; HIL tests in `tests/test_telemetry.py`; bench drainer
`scripts/telemetry_drain.py`.

**Memory (crash-dump semantics).** Fixed OCRAM region, in no linker section (like the breadcrumb):
ring base `0x2026F000`, size `0x10000`, end `0x2027F000` — below the breadcrumb (`0x2027FF40`)
and PJRC's `CrashReport` (`0x2027FF80`). At boot: if the header's magic + checksum validate and the
cursors are in range, the contents are **KEPT** (`boot_count++`), the record chain from `read_off`
is walked and **truncated at the first inconsistent record** (a partially written trailing record
is ignored — a valid header is never a reason to wipe), and a `STATE(boot)` record is appended.
That is the crash dump after a `SYSTEM_RESET`, a lockup reset, or the bootloader's program-button
reboot (RAM is preserved). A power-on leaves garbage → the ring is initialised.

**Heap guard (the address-range assumption, and how it is enforced).** OCRAM ("RAM2") is
`0x20200000..0x20280000`; `.bss.dma` (`DMAMEM`) occupies its bottom 55.7 KB in this build and the
heap grows up from there (`_heap_start` = end of `.bss.dma`) — but the Teensy linker script puts
`_heap_end` at the **top** of OCRAM, so `malloc` can in principle grow into the fixed region. This
firmware's heap use is small and static (SdFat, Wire); the heap would need ~390 KiB to reach the
ring. Nothing enforces that, so `Telemetry::begin()` **and every append** (and `loop()`'s
`Telemetry::service()`) compare the live heap break (`__brkval`, the core's `_sbrk` cursor) against
`ring base − 4096`. If the heap gets that close the ring is **disabled for the rest of the boot**
(events off, no ring access at all — the header at the ring base is the first thing the heap would
clobber), a `STATE(ring_overrun, code 0xFF)` is appended first if the ring is still intact, and the
0xA9 reply reports **`flags` bit2 = disabled: heap collision** (header only, counters 0). At boot,
a heap already inside the guard band leaves the region untouched (a valid ring stays in RAM for the
next boot).

**Overflow = overwrite oldest.** The ring is first a crash recorder: the lead-up to a hang matters
more than unread old history. When a record does not fit, `read_off` is advanced past the oldest
record(s) until it does and `dropped` counts every evicted record; the host sees the **seq gap**
plus the counter, and one `STATE(ring_overrun, code 0, arg = evicted since the last marker)` marks
each eviction *episode* (evictions ≥ 1 s apart start a new episode). A host that polls at ~10 Hz
and loops on `more` never lets it fill.

**Commit ordering (reset safety).** An append (1) publishes any eviction first — the header with the
advanced `read_off` is flushed *before* a byte of the old records is overwritten; (2) writes the
PAD (if any) and the record bytes beyond `write_off` and flushes them — invisible until (3) the
header (`write_off`, `next_seq`, `checksum`) is flushed. A reset at any point leaves the header
pointing at the last complete record.

**`telemetry_flags` / cache note.** OCRAM is mapped write-back (`MEM_CACHE_WBWA` in the Teensy
core's `startup.c`), so an unflushed append could sit in a dirty D-cache line and be lost when a
reset invalidates the cache — exactly the most recent records a crash dump wants. Every append
therefore `arm_dcache_flush`es the record bytes just written (1–2 lines) **and the 32-byte header
line** (`write_off` / `next_seq` / `flags` / `dropped`, sealed by the checksum); `ack` flushes the
header again when `read_off` moves. Same reasoning and cost class as the breadcrumb: ~0.2 µs per
record, < 0.02 % CPU at 286 Hz Mode-3 streaming. Making the region uncached via an MPU region was
rejected (the base is not 64 KiB-aligned, and the drain reads faster cached).

**Layout** (all little-endian). Header, 32 B at the ring base:

```
RingHeader (at base, 32 B): magic u32 = 0x47365452 ('G6TR'), ver u8 = 2 (1 = 20 B FRAME builds; a kept ring of another version is re-initialised at boot), flags u8 (bit0 events_enabled), reserved u16,
  write_off u32, read_off u32 (host ack cursor), next_seq u32, dropped u32, boot_count u32, checksum u32 (sum of the other fields)
Records (from base+32 to base+0x10000, byte ring, wrap-around; a record never splits: if the tail can't fit
  the next record a PAD record `len=remaining, type=0` fills it and writing wraps to 0)
Record: len u8 (total incl. this byte), type u8, seq u32, t_us u32 (micros()), payload
  type 1 CMD   : cmd u8, status u8, plen u8, payload[plen ≤ 8]      (recorded after dispatch; t_us = receipt time before dispatch)
  type 2 FRAME : idx u16, pattern u16, sd_load_us u32, spi_us u16, req_age_us u32, superseded u8, flags u8   (26 B; recorded in transmitOnRefresh when cur_frame_index_ or pattern changed since the last FRAME record)
  type 3 STATE : kind u8, code u8, arg u16
      kinds: 1 boot (code = reset_cause & 0xFF, arg = prev breadcrumb op<<8 | prev_valid), 2 state_change (code = new ArenaState, arg = pattern_id),
             3 error_glyph (code = CE code, arg = 0), 4 sd_slow (code=0, arg = read µs/100; when a readFrame > 20 ms),
             5 ring_overrun (code 0: arg = records evicted since the last marker; code 0xFF: ring disabled, heap collision, arg 0),
             6 telemetry (code = SET_TELEMETRY flags, arg = synthetic rate), 7 sd_open (code = CE result, arg = pattern_id),
             8 wdog_context (boot after a watchdog reset: code = EXC_RETURN & 0xFF — bit 3 set = thread preempted (F9/E9/FD/ED), clear = a handler (F1/E1);
                             arg bits 0-8 = xPSR IPSR (0 = thread, else exception number — PIT = 138), arg bits 9-15 = isr_last when the watchdog fired),
             9 prev_isr_count (boot after a watchdog reset, one per ISR id with entries: code = ISR id, arg = min(65535, entries >> 12)),
             10 timer_fail (IntervalTimer::begin() failed, timer left un-armed: code = 0, arg = requested refresh Hz),
             11 sd_layout (after every sd_open: code bit0 = pattern file contiguous, bit1 = exFAT, bit2 = legacy seek forced (0xCE), bit3 = same-index skip disabled (0xCE); arg = sectors per cluster),
             12 sd_slow_ctx (follows every sd_slow: code = SdFat card errorCode() — sticky, 0 = no driver error this boot; arg = errorData() >> 16 = USDHC IRQSTAT error bits 16-31 saved at the driver's LAST error, may predate this read),
             13 sd_reads (at STOP / next trial start: reads = arg << code, readFrame calls while that pattern was open),
             14 sd_reads_ckpt (every 30k reads while a pattern is open: cumulative so far = arg << code — a lower bound if the run dies before kind 13)
      sd_slow code byte (ring v2): bits 0-1 = slowest phase of the read (1 seek, 2 body, 3 CRC trailer), bit7 = the read returned an error; threshold 10 ms (was 20 ms)
```

| Off | Type | Header field | Meaning |
|---|---|---|---|
| 0 | u32 | `magic` | `0x47365452` (`'G6TR'`) |
| 4 | u8 | `ver` | `2` (ring v2, 26 B FRAME; `1` = 20 B FRAME builds) |
| 5 | u8 | `flags` | bit0 `events_enabled` (forced on at every boot) |
| 6 | u16 | `reserved` | 0 |
| 8 | u32 | `write_off` | next byte to write, offset into the 65,504-byte record area |
| 12 | u32 | `read_off` | host ack cursor — oldest unacked byte |
| 16 | u32 | `next_seq` | seq of the next record (starts at 1; `first_seq = 0` in a block means "none") |
| 20 | u32 | `dropped` | records evicted (overwrite-oldest) since init |
| 24 | u32 | `boot_count` | boots that kept this ring (0 = initialised this boot) |
| 28 | u32 | `checksum` | sum of the seven u32 words above |

Record offsets: `len` @0, `type` @1, `seq` @2 (u32), `t_us` @6 (u32), payload @10.
CMD payload: `cmd` @10, `status` @11, `plen` @12, request bytes @13.. (the first ≤ 8 bytes after
`[len, cmd]`); CMD records are 13–21 B. FRAME payload: `idx` @10 (u16), `pattern` @12 (u16),
`sd_load_us` @14 (**u32** — 129 ms SD reads have been observed; u16 would clip at 65 ms),
`spi_us` @18 (u16), `req_age_us` @20 (**u32**: dispatch of the `SET_FRAME_POSITION` that requested this
frame — `loadFrame` entry for Mode 2/4 — to the SPI start; excludes USB/host queueing; u32 so a 30–90 ms
card stall is representable), `superseded` @24 (u8: frames loaded into `frame_buf_` but replaced before
any transfer since the last FRAME record), `flags` @25 (bit0 frame came from an SD read, bit1 the
pattern file is contiguous = O(1) seek path); **26 B total** (ring v2; 20 B in v1 — decoders read the
first 10 payload bytes and treat the rest as optional). STATE payload: `kind` @10, `code` @11, `arg`
@12 (u16); 14 B total. `t_us` is raw `micros()` (wraps every 71.6 min; the host unwraps using `seq` and the block's
`t_now_us`).

**What gets recorded** (producer hooks, all in `CommandProcessor.cpp`):

- `CMD` — in `handleBinaryCommand`: `t_us` = `micros()` at receipt (before dispatch); after the
  dispatch switch the record carries the opcode, the reply's status byte
  (`MessageSource::lastResponseStatus()`, 0xFF if none was queued) and the first ≤ 8 request bytes.
  `GET_TELEMETRY_BLOCK` (0xA9) itself is **not** recorded (a poller looping on `more` would fill the
  ring with records of reading the ring); 0xC2/0xCA/0xCB/0xA8 are. Not recorded: 0x32 stream frames
  and the 0x85/0xE0 bulk-upload headers (different dispatch paths), and `SYSTEM_RESET` 0x01 (it
  resets before the record would be appended — the next boot's `STATE(boot)` marks it).
- `FRAME` — in `transmitOnRefresh`, when `cur_frame_index_` or `pattern_id_` differs from the last
  FRAME recorded (reset at every `enterPatternMode`, so the first frame of a trial is always
  recorded). `t_us` = start of `SpiManager::transferFrame` (what the panels latch), `spi_us` = its
  duration, `sd_load_us` = the most recent `readFrame` duration (`Health::stats.last_sd_read_us`),
  `req_age_us` = SPI start − the request's dispatch time (`superseded` counts loads that never reached
  the panels). A held frame re-sent every refresh tick costs nothing.
- `STATE` — `boot` in `Telemetry::begin()`; `sd_open` (+ `state_change` on success) in
  `enterPatternMode`; `state_change` in `enterAllOff` (STOP / ALL_OFF / trial timer / glyph
  timeout), `enterAllOn`, `enterStreamingFrame`, and on an actual change in `handleSetFramePosition`
  / the PSRAM handlers; `error_glyph` in `showError`; `sd_slow` (+ `sd_slow_ctx`) in `loadFrame` when
  a `readFrame` exceeds 10 ms; `sd_layout` after a successful `sd_open`; `sd_reads` in `enterAllOff` /
  the next `enterPatternMode` for the pattern being left; `telemetry` in `SET_TELEMETRY`;
  `ring_overrun` from the ring itself.
- **Synthetic producer** (bench test T1, drain throughput): `SET_TELEMETRY` flags **bit7** turns on
  a dummy CMD-type record — `cmd 0xFE, status 0, plen 4, payload = counter u32` (restarts at 0 on
  each enable) — generated from `loop()` (`Telemetry::service()`) at `rate` records/s, paced by
  `micros()` accumulation (no timer; catch-up after a loop stall is capped at 256 records per
  call). Not gated by events bit0, so a pure synthetic stream is a valid setup. **Off at boot.**

**Opcodes.**

- `SET_TELEMETRY` **0xA8** — `[04 A8 flags rate_lo rate_hi]`, or the 2-byte form `[02 A8 flags]`.
  `flags` bit0 = record events (default **ON** at every boot — recording costs nothing until
  drained), bit7 = synthetic producer on at `rate` records/s (u16; `rate` 0 = off), bits 1..6
  reserved (analog ticks come later). The 2-byte form leaves `rate` unchanged. Reply: status 0, no
  payload. Records `STATE(telemetry, flags, rate)` — before the gate closes on a disable, after it
  opens on an enable — so the transition is always in the log. Ignored (stays disabled) once the
  heap guard has tripped.
- `GET_TELEMETRY_BLOCK` **0xA9** — `[08 A9 ack_seq u32 LE, max_bytes u16 LE, flags u8]` (`flags`
  reserved; the byte may be omitted). The controller **first advances `read_off` past every record
  with `seq ≤ ack_seq`** (`ack_seq = 0xFFFFFFFF` means "no ack"), **then replies** with the header
  plus as many *whole* records from `read_off` as fit in `min(max_bytes, 178)` bytes **without
  advancing `read_off`** — they are freed only by a later ack. PAD records are skipped, never
  returned. Reply payload = 18-byte header + records verbatim:

| Off | Type | Field | Meaning |
|---|---|---|---|
| 0 | u32 | `t_now_us` | controller `micros()` at reply time (pair with the host receive time to fit a clock) |
| 4 | u32 | `first_seq` | seq of the first returned record; 0 if none |
| 8 | u16 | `n_records` | records in this block |
| 10 | u32 | `dropped` | records evicted since the ring was initialised |
| 14 | u8 | `more` | 1 if unread records remain after this block — ask again immediately |
| 15 | u8 | `flags` | bit0 `events_enabled`, bit1 contents survived a reboot (`boot_count > 0`), bit2 ring disabled: heap collision, bit3 synthetic producer on |
| 16 | u16 | `boot_count` | boots that kept this ring (saturates at 65535) |

  Why 178 and not 180: `sendResponse` accepts a payload only while `3 + payload_len < 200`
  (`RESP_BUF_SIZE`), i.e. ≤ 196 B, and the header takes 18. A maximal CMD record is 21 B, so a
  block always carries at least one record when any is pending.

**Ack semantics / lossless drain.** The host keeps the last seq it has *stored* and sends it as
`ack_seq` on the **next** request: a reply that is lost or times out is re-served unchanged on the
re-ask (same `first_seq`, same bytes), because nothing is freed until the host says so (or the ring
overflows — then the seq gap and `dropped` say exactly what was lost). A lower or repeated ack is a
no-op; acking frees exactly the records with `seq ≤ ack_seq`. Drain loop:
`ack = 0xFFFFFFFF; loop { reply = 0xA9(ack, 178); store records; ack = last seq; if !more: break }`,
then one final `0xA9(ack, 0)` frees the last block. At 286 Hz Mode-3 streaming the ring fills at
~10 KB/s, so a poller must loop on `more` (~56 chunks/s), not take one chunk per poll.

**Cost.** Recording is a few dozen byte stores + two one-to-three-line dcache flushes per record
(see the cache note above) and one heap-break compare, gated by one DTCM boolean; nothing runs when
events are disabled. Ring capacity: 65,504 B ≈ 3,000–4,000 records ≈ 6 s at the worst case above,
≥ 50 s of a typical Mode-2 run, indefinitely for an idle controller.

**Semantics and known limitations (read before interpreting a drain).**

- **FRAME records describe SD-playback frames only** (Modes 2/3/4: `loadFrame` → `transmitOnRefresh`).
  Streamed (0x32, Mode 5) and PSRAM (0x3A/0x3B) frames are not tracked — they re-send a held
  buffer whose `cur_frame_index_`/`pattern_id_` do not change. Known limitation; frame-generation
  tracking for those paths is deferred ([#54](https://github.com/reiserlab/LED-Display_G6_Firmware_Arena/issues/54)).
- **`t_us` semantics.** CMD `t_us` = **dispatch entry** (`handleBinaryCommand` start), not the byte's
  arrival on USB; FRAME `t_us` = **start of the SPI transfer**, not the moment the LEDs change (add
  `spi_us` + the panel latch). **`seq` is the append order, and `t_us` may be non-monotonic within
  one dispatch**: a CMD record is appended *after* its handler ran, so the STATE records that
  handler produced (e.g. `sd_open`, `state_change`, `telemetry`) precede the CMD in `seq` while
  carrying a later `t_us`. Sort by `seq` for causality, by `t_us` for timing — do not assert one from
  the other.
- **Retention.** At the ~10 KB/s worst case (286 Hz Mode-3, CMD + FRAME per step) the ring holds
  ≈ 6 s. Under sustained overflow the `STATE(ring_overrun)` marker written at the first eviction is
  itself evicted after another ring-full — it is **not guaranteed to survive**. The durable contract
  is `dropped` in the block header plus the `seq` gap (gap == records evicted, PADs excluded; the
  marker's own eviction is counted).
- **Single-copy header.** The 32-byte header is one copy, flushed after every append. A reset that
  lands *during* the header write (a ~100 ns window) can leave a header that fails the checksum, and
  the boot then initialises the ring — losing the history. Accepted for this diagnostic build: the
  #50 wedge is a *hang*, the program-button reboot happens minutes later with no append in flight.
  Dual headers / a linker-reserved region / 64-bit cursors / per-transport (TCP) read cursors are
  recorded as deferred, tracked in [#54](https://github.com/reiserlab/LED-Display_G6_Firmware_Arena/issues/54).
- **Ack guard.** An `ack_seq ≥ next_seq` (never generated by this incarnation — e.g. a host still
  holding an ack from before a power cycle re-initialised the ring) is ignored, so a stale ack can
  never free unseen records; `seq` 0 and 0xFFFFFFFF are never emitted. The heap guard runs on the
  ack/peek/configure paths as well as on append.

**Host tooling.** `scripts/telemetry_drain.py --port …` is a pyserial drainer that prints every
record as a JSON line, tracks seq gaps, and summarises on Ctrl-C (`--raw-out` also writes the raw
blocks with a host receive stamp — the proposal's § 7 sidecar); `tests/telemetry_codec.py` is the
decoder both it and the HIL tests use.
### Qwiic / STEMMA QT jack (I2C bridge)

arena_12-18 has a Qwiic jack, J2: GND, +3.3 V, SDA → Teensy D17 (SDA1), SCL → D16 (SCL1),
i.e. `Wire1` — separate from the MCP4725 AO DAC on `Wire` (D18/D19). The board has no pull-ups
on it (the breakouts carry 10k each); the firmware runs it at 100 kHz. Two commands expose the
bus to a host so sensors can be validated without sensor-specific firmware:

- `GET_I2C_SCAN` (0xB0): `[01 B0]` → `[count, addr...]`, the 7-bit addresses (0x08–0x77) that ACK.
- `I2C_TRANSFER` (0xB1): `[len B1 addr wlen w... rlen]` → the `rlen` bytes read. Writes `wlen`
  bytes, then reads `rlen` under a repeated start; `wlen = 0` is a plain read, `rlen = 0` a plain
  write, both zero an ACK probe. `rlen ≤ 64`. Status: 1 bad framing, 2 address NACK, 3 data NACK,
  4 bus error/timeout, 5 short read.

arena_10-10 builds (no jack) answer both with `status = 1`. Both block the control loop for the
transaction (≲ 12 ms for a full scan) — bench use, not the display hot path. `pixi run qwiic-probe`
(`scripts/qwiic_probe.py`) scans the bus, maps PCA9548 mux channels, identifies the LAB-211 sensors
(AS7343, TSL2591, VEML7700) and takes readings; `pixi run qwiic-read` streams all of them
continuously (paced to the slowest integration, ~6.7 Hz with the AS7343, Ctrl-C for stats);
`tests/test_qwiic_i2c.py` covers the same under pytest and skips whatever is not plugged in.

## SD pattern playback (Modes 2/3/4)

Patterns are `/patterns/*.pat` files on the built-in SD card, in the v2 `G6PT` format
([`g6_04-pattern-file-format.md`](https://github.com/reiserlab/Modular-LED-Display/blob/main/docs/development/g6_04-pattern-file-format.md)):
18-byte header + per-frame `"FR"` prefix + panel blocks + CRC-16 trailer. Files are sorted
alphabetically and addressed by a **1-based pattern ID** (`TRIAL_PARAMS` `pattern_id`). The reader
validates the **header CRC-8** on open and the **per-frame CRC-16** on each frame read, and checks
that the pattern geometry matches this controller's 2x10 arena.

On any SD/CRC/parameter fault the controller shows a **"CE / NN" error glyph** on all panels for ≥750 ms
(codes in `constants.h` `ControllerError`, e.g. `04` = no card, `06` = header CRC, `07` = frame CRC,
`08` = arena geometry mismatch) and replies with `status = 1`.

## Hardware-in-the-loop tests

`tests/` is a pytest suite that drives a live controller over USB-CDC or TCP
(see `tests/conftest.py` for the transport options):

```sh
pixi run test-serial            # automated suite over USB-CDC
pixi run test-tcp -- --ip 10.0.0.x
pixi run test-serial-visual     # opt-in guided tests (human at the arena; -s already set)
pixi run -e debugad3 test-serial-ad3   # opt-in AD3-instrumented tests (Analog Discovery 3)
```

Guided-visual tests are marked `visual` (need `--visual`), AD3-instrumented
tests `ad3` (need `--ad3` plus the `debugad3` pixi environment and the
Digilent WaveForms runtime; wiring in `tests/ad3_row_probe.py`). Both are
skipped by default.

`tests/test_pr15_stuck_row_timeout.py` additionally needs one panel flashed
with Panel-Firmware's `pico_v031_twopiotimeouttest` forced-fault build; its
module docstring is the full walkthrough (flash command, why the arena's
display-mode patching makes the `0x1B` step mandatory, and why brightness,
not shape, is the pass/fail discriminator).

## Host tooling

- **[Arena Console](https://reiserlab.github.io/webDisplayTools/arena_console.html)** — the
  browser-based Web Serial control panel (buttons for every command, stream presets, raw-hex
  send, and `.bin` / `.pat` upload-and-stream), in webDisplayTools.
- `scripts/all_on.py`, `controller_info.py`, `play_pattern.py`, `probe.py` — standalone
  TCP clients (no `arena_interface` dependency).
- `scripts/all_on_serial.py`, `scripts/multi_port_capture.py` — USB-CDC bench tools for the
  CIPO diagnostic (`DEBUG_SERIAL` builds): drive all-on over serial and capture/parse the
  `[spi] CIPO` stream on the same pipe. `multi_port_capture.py` additionally taps both panels'
  `SPI_DIAG` heartbeats on their own ports to confirm per-panel frame reception.
- `scripts/telemetry_drain.py` — USB-CDC drainer for the telemetry ring (0xA9 with ack cursor):
  JSON-lines records, seq-gap tracking, Ctrl-C summary; `scripts/soak_mode3.py` — browser-free
  Mode-3 soak driver for issue #50.

## TCP transport — known throughput limitation

TCP upload throughput (host → controller → SD) is capped at approximately **84 kB/s** — around 6× slower than the USB-CDC serial path (~1370 kB/s) — and is unaffected by reducing LWIP's delayed-ACK timer (`TCP_TMR_INTERVAL`). The root cause lies inside QNEthernet's connection receive-buffer management: each call to `client_.available()` / `client_.read()` appears to expose only one LWIP pbuf worth of data at a time (~1 TCP segment ≈ 1460 bytes), regardless of how many segments have actually arrived from the host. Because the firmware must cycle through `readBulkBytes` → SD write (~3 ms) → `readBulkBytes` for every such chunk, and each cycle triggers two `Ethernet.loop()` passes whose overhead is non-trivial, throughput stalls well below what the network or SD card could sustain. The download direction (controller → host, ~5000 kB/s) is unaffected because there the bottleneck is the 100 Mbps Ethernet link, not the receive path. Closing the upload gap would require either patching QNEthernet's `ConnectionManager` to deliver all buffered pbufs in a single `read()` call, or restructuring the file-transfer path to use UDP datagrams (eliminating the receive-buffer issue entirely at the cost of application-level sequencing).
