# G6 Arena Slim

G6 Arena Controller firmware for the **arena_10-10 v1** hardware (Teensy 4.1, 20 panels in a 2x10 grid, two SPI buses, G6 v1 panel protocol).

Structurally based on G4.1-ArenaSlim: single main loop, no QP framework, same response framing, same G4 host command opcodes (with G6-dropped commands rejected).

Current capabilities:

- **G4 display Modes 2, 3, 4, and 5.** Mode 5 streams full arena frames over TCP/USB; Modes 2/3/4 play `.pat` files from the SD card (open-loop auto-advance, host-commanded frame, and AIN0 closed-loop velocity).
- **Two command transports** — TCP (port 62222) and USB-CDC serial — sharing one parser and command set.
- **`get-controller-info` (0xC2)** capability handshake and a **controller error display** ("CE / NN" glyph) for SD/CRC/parameter faults.
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

## TCP transport — known throughput limitation

TCP upload throughput (host → controller → SD) is capped at approximately **84 kB/s** — around 6× slower than the USB-CDC serial path (~1370 kB/s) — and is unaffected by reducing LWIP's delayed-ACK timer (`TCP_TMR_INTERVAL`). The root cause lies inside QNEthernet's connection receive-buffer management: each call to `client_.available()` / `client_.read()` appears to expose only one LWIP pbuf worth of data at a time (~1 TCP segment ≈ 1460 bytes), regardless of how many segments have actually arrived from the host. Because the firmware must cycle through `readBulkBytes` → SD write (~3 ms) → `readBulkBytes` for every such chunk, and each cycle triggers two `Ethernet.loop()` passes whose overhead is non-trivial, throughput stalls well below what the network or SD card could sustain. The download direction (controller → host, ~5000 kB/s) is unaffected because there the bottleneck is the 100 Mbps Ethernet link, not the receive path. Closing the upload gap would require either patching QNEthernet's `ConnectionManager` to deliver all buffered pbufs in a single `read()` call, or restructuring the file-transfer path to use UDP datagrams (eliminating the receive-buffer issue entirely at the cost of application-level sequencing).
