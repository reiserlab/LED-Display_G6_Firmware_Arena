# Arena-Firmware guidance for AI assistants

G6 Arena Controller firmware (Teensy 4.1). All builds, flashes, tests, and
bench tools run through **pixi** -- there is no separate manual toolchain
setup. See `README.md` for protocol/architecture details; this file is
about how to actually run things and how the repo is wired together for
that purpose.

## Prerequisites

Only [pixi](https://pixi.sh/) needs to be installed. The first `pixi run
<task>` in a fresh checkout fetches PlatformIO, the Teensy toolchain, and
Python automatically.

## Hardware variants

This firmware targets more than one physical arena board, selected at
**build time** by an `-DARENA_HW_*` flag -- there is no runtime detection
or fallback. Building without the flag fails to compile on purpose (see
the `#error` in `src/constants.h` and `src/ArenaConfig.h`), so a
host/firmware hardware mismatch is a build- or handshake-time error
instead of a silently wrong or frozen display.

| Variant | Flag | Panel grid | Per-board config |
|---|---|---|---|
| arena_10-10 (current production board) | `ARENA_HW_10_10` | 4 rows x 10 cols (40 panels) | `src/hw/ArenaConfig_10_10.h` |
| arena_12-18 | `ARENA_HW_12_18` | 4 rows x 12 cols (48 panels) | `src/hw/ArenaConfig_12_18.h` |

`src/ArenaConfig.h` is a thin selector that includes the right per-board
header based on the flag. `src/constants.h` selects `panel_count_per_frame_row/col`
the same way. Everything else (SPI mode, block byte counts, refresh
defaults, command opcodes) is shared and lives outside the `#if` blocks.

## pixi tasks

Every hardware variant has the same four-task pattern; keep them in sync
if you add a fifth variant or change what any of them does.

```
pixi run build-10-10               # compile only
pixi run deploy-10-10               # compile and upload
pixi run deploy-10-10-performance   # compile and upload, no DEBUG_SERIAL
pixi run monitor-10-10              # USB serial monitor, logs to log/

pixi run build-12-18
pixi run deploy-12-18
pixi run deploy-12-18-performance
pixi run monitor-12-18
```

These map directly to `platformio.ini` environments (`teensy41-10-10`,
`teensy41-10-10-performance`, `teensy41-12-18`, `teensy41-12-18-performance`),
all of which `extends = env:teensy41-base` and only differ by their
`build_flags`. `DEBUG_SERIAL` gates the `[spi] CIPO` diagnostic prints and
other `DBG_PRINTF` output (see `constants.h`'s `DBG_PRINTF` macro) -- use
the non-`-performance` task when you need those. A `DEBUG_SERIAL` build is
safe to leave on the bench: the SPI transfer's blocking CIPO capture only
runs when diagnostics are unmuted at runtime (`SET_DIAG_OUTPUT`, 0xC3,
default off), so with diagnostics muted it idles at the same timing as a
`-performance` build (see the comment in `SpiManager.cpp`'s `transferFrame`).
Reach for `-performance` when you specifically want the diagnostic path
compiled out entirely, not for routine timing concerns.

Other pixi tasks:

```
pixi run test                       # = test-serial (default automated suite)
pixi run test-serial                # HIL pytest suite over USB-CDC, excludes visual
pixi run test-tcp -- --ip 10.0.0.x  # same, over TCP
pixi run test-serial-visual         # guided visual tests; needs a human at the arena
pixi run test-tcp-visual -- --ip 10.0.0.x
pixi run -e debugad3 test-serial-ad3   # AD3-instrumented tests (needs Digilent WaveForms)
pixi run webserial                  # serve the Arena Console web UI + open a browser
pixi run all-on-serial -- --port /dev/ttyACM0   # USB-CDC CIPO bench tool
pixi run multi-capture              # dual-port CIPO + panel SPI_DIAG capture
```

All test/tooling tasks are hardware-variant-agnostic at the pixi layer --
they just talk to whatever is plugged in and flashed. The pytest suite
does not know which `ARENA_HW_*` build is on the device; a wrong-variant
mismatch shows up as a failed `status == 0` assertion on the first
`STREAM_FRAME_CMD` ("Bad stream-frame size"), not a pytest skip.

## Adding a new hardware variant

1. Derive the new board's CS/panel map (see `src/hw/ArenaConfig_12_18.h`'s
   header comment for how that one was net-traced from the KiCad
   schematics) and its panel grid geometry.
2. Add `src/hw/ArenaConfig_<name>.h` with `panel_set_count` and
   `panel_sets[]`, following the existing two files' shape exactly (they
   rely on being `#include`d inside `ArenaConfig.h`'s `namespace AC { ... }`
   block, with `PanelSet` already declared -- don't include them directly).
3. Add an `#elif defined(ARENA_HW_<NAME>)` branch to `src/ArenaConfig.h`
   and to the geometry block in `src/constants.h`.
4. Add `teensy41-<name>` / `teensy41-<name>-performance` environments to
   `platformio.ini`, extending `env:teensy41-base` with
   `-DARENA_HW_<NAME>` (plus `-DDEBUG_SERIAL` for the debug variant).
5. Add the matching `build-<name>` / `deploy-<name>[-performance]` /
   `monitor-<name>` tasks to `pixi.toml`, copying the existing quartet.
6. Update the variant table above and `README.md`'s Build section.

## Gotchas specific to this repo

- **Frame-size constants are part of the wire protocol.** `panel_count_per_frame`
  (derived from the per-board `panel_count_per_frame_row/col`) determines
  the exact byte length `STREAM_FRAME_CMD` will accept
  (`stream_frame_byte_count_gs2/gs16` in `constants.h`). If you change
  a board's geometry, any host software that streams frames (webDisplayTools,
  maDisplayTools, the pytest suite's frame builders) must match, or every
  stream gets silently rejected and the firmware falls back to whatever
  was last in `frame_buf_` -- which can look like a hardware fault rather
  than a version mismatch. Exposing the flashed `ARENA_HW_*` variant
  through `GET_CONTROLLER_INFO_CMD`'s `controller_capability_bitmap`
  (`CommandProcessor.cpp`'s `handleGetControllerInfo()`) would close this
  gap, but that's a shared wire-protocol field (spec'd in `g6_03-controller.md`
  § 5 in the docs repo) -- update both sides together, not just this repo.
- **SCK on bus B0 shares a pin with `LED_BUILTIN`** (Teensy D13) on both
  current hardware variants. Never `digitalWrite(LED_BUILTIN, ...)` for
  status -- it glitches the SPI clock. Use `ETH_LED` or another spare GPIO.
- **`enterAllOff()` pushes an explicit all-dark frame three times**
  rather than just stopping transmission, because panels run in
  Persistent mode and hold their last received frame. If you add a new
  idle/safe-state path, follow the same pattern rather than assuming
  "stop sending" is enough to blank the display.
- **No runtime hardware autodetection, and no runtime-visible indicator of
  which `ARENA_HW_*` variant is flashed either** -- neither the boot
  sequence nor `GET_CONTROLLER_INFO_CMD` (0xC2) currently reports it. If a
  board reports unexpected behavior, first double check which `pixi run
  deploy-<variant>` task was actually used last, rather than assuming a
  code bug. Reporting the variant in `GET_CONTROLLER_INFO_CMD`'s
  capability bitmap would close this gap and is worth doing before this
  bites someone.
- **Flashing from Windows takes two attempts — expect the first to fail.**
  Observed on every upload during the 2026-09-10 bench day (Windows 11,
  PlatformIO 6.1, `teensy_loader_cli` 2.2): the first `pixi run
  deploy-<variant>` reboots the board into HalfKay and then dies with
  `error writing to Teensy`; running the identical command again finds the
  bootloader already up and programs in ~4 s. Just re-run it. Three more
  Windows facts: (1) `scripts/find_teensy.py` only globs
  `/dev/serial/by-id/...`, so pass the port explicitly --
  `pixi run deploy-12-18-performance -- --upload-port COM5` (find it with
  `Get-PnpDevice | ? InstanceId -match VID_16C0` and take the `Status OK`
  entry; stale `Unknown` COM entries from previously plugged controllers
  linger); (2) `teensy_loader_cli` prints `Soft reboot is not implemented
  for Win32` and simply waits -- the board only entered the bootloader once
  the **arena was powered**, so an unpowered arena looks like a hang;
  (3) nothing else may hold the COM port (Web Serial in the Studio, a
  `monitor-*` task) or the reboot never happens.
