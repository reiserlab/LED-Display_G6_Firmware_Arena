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
   pixi run deploy
   ```
3. **Install the panels.** Add G6 panels running the most recent panel firmware from
   [`reiserlab/LED-Display_G6_Firmware_Panel`](https://github.com/reiserlab/LED-Display_G6_Firmware_Panel).
4. **Open the [Arena Console](https://reiserlab.github.io/webDisplayTools/arena_console.html)**
   in a **Chromium-based** browser (Chrome / Edge / Opera / Brave — Web Serial is not in Firefox
   or Safari).
5. **Connect to the arena.** Click **Connect to G6 Arena** and pick the correct serial port from
   the chooser (typically `/dev/ttyACM*` on Linux, `COM*` on Windows, `/dev/cu.usbmodem*` on
   macOS). Close any other program holding the port (e.g. `pixi run monitor`) first.
6. **Light it up.** Click any command — **All On** is a good first test. From there try the
   other buttons, the **Stream** presets, or upload a `.bin`/`.pat`.

## Build

The toolchain (PlatformIO, the Teensy compiler, Python, …) is installed automatically by pixi the
first time you run a task — you don't install any of it yourself.

```
pixi run build           # compile
pixi run deploy          # compile and upload
pixi run deploy-printf   # compile with DEBUG_SERIAL, and upload
pixi run monitor         # USB serial monitor (logs to log/)
```

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
| 3 | u8 | `flags` | bit0 `dirty` (tracked files modified at build time), bit1 `debug` (`DEBUG_SERIAL` build), bit2 `telemetry` (telemetry ring compiled in — `SET_TELEMETRY` 0xA8 / `GET_TELEMETRY_BLOCK` 0xA9 answer; **hosts gate 0xA8 on this bit**, not on 0xC2 bit 7) |
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
RingHeader (at base, 32 B): magic u32 = 0x47365452 ('G6TR'), ver u8 = 1, flags u8 (bit0 events_enabled), reserved u16,
  write_off u32, read_off u32 (host ack cursor), next_seq u32, dropped u32, boot_count u32, checksum u32 (sum of the other fields)
Records (from base+32 to base+0x10000, byte ring, wrap-around; a record never splits: if the tail can't fit
  the next record a PAD record `len=remaining, type=0` fills it and writing wraps to 0)
Record: len u8 (total incl. this byte), type u8, seq u32, t_us u32 (micros()), payload
  type 1 CMD   : cmd u8, status u8, plen u8, payload[plen ≤ 8]      (recorded after dispatch; t_us = receipt time before dispatch)
  type 2 FRAME : idx u16, pattern u16, sd_load_us u32, spi_us u16   (recorded in transmitOnRefresh when cur_frame_index_ or pattern changed since the last FRAME record)
  type 3 STATE : kind u8, code u8, arg u16
      kinds: 1 boot (code = reset_cause & 0xFF, arg = prev breadcrumb op<<8 | prev_valid), 2 state_change (code = new ArenaState, arg = pattern_id),
             3 error_glyph (code = CE code, arg = 0), 4 sd_slow (code=0, arg = read µs/100; when a readFrame > 20 ms),
             5 ring_overrun (code 0: arg = records evicted since the last marker; code 0xFF: ring disabled, heap collision, arg 0),
             6 telemetry (code = SET_TELEMETRY flags, arg = synthetic rate), 7 sd_open (code = CE result, arg = pattern_id)
```

| Off | Type | Header field | Meaning |
|---|---|---|---|
| 0 | u32 | `magic` | `0x47365452` (`'G6TR'`) |
| 4 | u8 | `ver` | `1` |
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
`spi_us` @18 (u16); **20 B total**. STATE payload: `kind` @10, `code` @11, `arg` @12 (u16); 14 B
total. `t_us` is raw `micros()` (wraps every 71.6 min; the host unwraps using `seq` and the block's
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
  duration, `sd_load_us` = the most recent `readFrame` duration (`Health::stats.last_sd_read_us`).
  A held frame re-sent every refresh tick costs nothing.
- `STATE` — `boot` in `Telemetry::begin()`; `sd_open` (+ `state_change` on success) in
  `enterPatternMode`; `state_change` in `enterAllOff` (STOP / ALL_OFF / trial timer / glyph
  timeout), `enterAllOn`, `enterStreamingFrame`, and on an actual change in `handleSetFramePosition`
  / the PSRAM handlers; `error_glyph` in `showError`; `sd_slow` in `loadFrame` when a `readFrame`
  exceeds 20 ms; `telemetry` in `SET_TELEMETRY`; `ring_overrun` from the ring itself.
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

**Host tooling.** `scripts/telemetry_drain.py --port …` is a pyserial drainer that prints every
record as a JSON line, tracks seq gaps, and summarises on Ctrl-C (`--raw-out` also writes the raw
blocks with a host receive stamp — the proposal's § 7 sidecar); `tests/telemetry_codec.py` is the
decoder both it and the HIL tests use.

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
