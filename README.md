# G6 Arena Slim

G6 Arena Controller firmware for the **arena_10-10 v1** hardware (Teensy 4.1, 20 panels in a 2x10 grid, two SPI buses, G6 v1 panel protocol).

Structurally based on G4.1-ArenaSlim: single main loop, no QP framework, same response framing, same G4 host command opcodes (with G6-dropped commands rejected).

Current capabilities:

- **G4 display Modes 2, 3, 4, and 5.** Mode 5 streams full arena frames over TCP/USB; Modes 2/3/4 play `.pat` files from the SD card (open-loop auto-advance, host-commanded frame, and AIN0 closed-loop velocity).
- **Two command transports** — TCP (port 62222) and USB-CDC serial — sharing one parser and command set.
- **`get-controller-info` (0x67)** capability handshake and a **controller error display** ("CE / NN" glyph) for SD/CRC/parameter faults.
- **Arena hardcoded to G6_2x10** — the panel-set table and CS pin map are baked in. Multi-arena lookup via [`g6_arena_configs.h`](https://github.com/reiserlab/Modular-LED-Display/blob/main/docs/development/g6_arena_configs.h) is deferred.
- **10 MHz SPI**, MSB-first, **CPOL=1 / CPHA=1 (Mode 3)** per [`g6_01-panel-protocol.md`](https://github.com/reiserlab/Modular-LED-Display/blob/main/docs/development/g6_01-panel-protocol.md) § SPI framing. (Panels accept up to 30 MHz; the clock is held at 10 MHz during bring-up — see `spi_clock_speed` in `constants.h`.)
- **G6 v2 `.pat` format** ([`g6_04-pattern-file-format.md`](https://github.com/reiserlab/Modular-LED-Display/blob/main/docs/development/g6_04-pattern-file-format.md)) is the on-disk file format the SD reader consumes.
- **No CIPO confirmation validation.** Panel echoes are clocked out on every transfer but not inspected.

> **Docs live in a separate repo.** The protocol/spec documents referenced here are in
> [`reiserlab/Modular-LED-Display`](https://github.com/reiserlab/Modular-LED-Display) under `docs/development/`, not in this firmware repo. Links below are absolute.

## Quickstart

**You only need [pixi](https://pixi.sh/) installed.** Everything else — PlatformIO, the Teensy
compiler, Python, the local web server — is fetched automatically the first time you run a `pixi
run` task. The one optional extra is a **Chromium-based browser** (Chrome / Edge / Opera / Brave)
for the Web Serial UI in step 4.

1. **Connect the controller.** Plug the Teensy 4.1 on the G6 arena into your computer via USB.
2. **Flash the controller firmware:**
   ```
   pixi run deploy
   ```
3. **Install the panels.** Add G6 panels running the most recent panel firmware from
   [`reiserlab/LED-Display_G6_Firmware_Panel`](https://github.com/reiserlab/LED-Display_G6_Firmware_Panel).
4. **Launch the Web Serial UI:**
   ```
   pixi run webserial
   ```
   This serves the page and opens a browser. If no browser opens, open a **Chromium-based**
   browser (Chrome / Edge / Opera / Brave — Web Serial is not in Firefox or Safari) and go to
   <http://localhost:8000>.
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
pixi run webserial       # serve scripts/web-serial and open a browser
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

## G4 host command protocol

Same wire framing as G4.1-ArenaSlim, accepted on both TCP and USB serial:

- **Incoming binary:** `[length, cmd, params...]`
- **Incoming stream:** `[0x32, len_lo, len_hi, frame_data...]` — no `analog_x`/`analog_y` bytes (G6 dropped these)
- **Response:** `[length, status(0=ok), echo_cmd, payload...]` — `payload` is an ASCII message for most commands, or raw bytes for `GET_CONTROLLER_INFO`

| Command | Code | Supported | Notes |
|---|---|---|---|
| `ALL_OFF`        | `0x00` | ✓ | Stops refresh, holds dark |
| `TRIAL_PARAMS`   | `0x08` | ✓ | "Combined command" — selects display mode + SD pattern (Modes 2/3/4) |
| `SET_REFRESH_RATE` | `0x16` | ✓ | Host override of GS-derived default (300 Hz GS16 / 1000 Hz GS2) |
| `STOP_DISPLAY`   | `0x30` | ✓ | Alias for ALL_OFF |
| `STREAM_FRAME`   | `0x32` | ✓ | Mode 5. Frame size `4 + 20*53` (GS2 = 1064) or `4 + 20*203` (GS16 = 4064) bytes |
| `GET_ETHERNET_IP_ADDRESS` | `0x66` | ✓ | Returns DHCP-resolved IP as ASCII |
| `GET_CONTROLLER_INFO` | `0x67` | ✓ | Returns `{version, capability_bitmap}` (bit 0 `g6_mode` = 1) |
| `SET_FRAME_POSITION` | `0x70` | ✓ | Mode 3 — show a specific frame of the open pattern |
| `ALL_ON`         | `0xFF` | ✓ | Synthesizes a full-bright GS16 oneshot on every panel |
| `SYSTEM_RESET`   | `0x01` | ✓ | Software system reset — acks then reboots (SCB_AIRCR SYSRESETREQ) |
| `SWITCH_GRAYSCALE` | `0x06` | ✗ | Dropped for G6 — `gs_val` is derived from the stream size / pattern header |

Unknown opcodes reply with `status = 1` and raise a `CE 01` error glyph.

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

## Host tooling

- `scripts/web-serial/` — browser UI (Web Serial) with buttons for every command, the
  webDisplayTools preset patterns, and `.bin` / `.pat` upload-and-stream. `pixi run webserial`
  serves it and opens a browser. See its [README](scripts/web-serial/README.md).
- `scripts/all_on.py`, `controller_info.py`, `play_pattern.py`, `probe.py` — standalone
  TCP clients (no `arena_interface` dependency).
- `scripts/all_on_serial.py`, `scripts/multi_port_capture.py` — USB-CDC bench tools for the
  CIPO diagnostic (`DEBUG_SERIAL` builds): drive all-on over serial and capture/parse the
  `[spi] CIPO` stream on the same pipe. `multi_port_capture.py` additionally taps both panels'
  `SPI_DIAG` heartbeats on their own ports to confirm per-panel frame reception.
