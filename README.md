# G6 Arena Slim

G6 Arena Controller firmware for the **arena_10-10 v1** hardware (Teensy 4.1, 20 panels in a 2x10 grid, two SPI buses, G6 v1 panel protocol).

This is a deliberately small first cut. Initial scope:

- **Mode 5 only** — host streams full arena frames over TCP; controller dispatches pre-formatted panel blocks to the panels. No SD card playback, no closed loop, no Triggered/Gated, no PSRAM (v2), no ISP, no controller error display.
- **Arena hardcoded to G6_2x10** — the panel-set table and CS pin map are baked in. Multi-arena lookup via `g6_arena_configs.h` is deferred.
- **30 MHz SPI**, MSB-first, **CPOL=1 / CPHA=1 (Mode 3)** per [`g6_01-panel-protocol.md`](../../public/docs/development/g6_01-panel-protocol.md) § SPI framing.
- **G6 v2 `.pat` format** is the target file format for future SD playback work (not exercised yet).
- **No CIPO confirmation validation.** Panel echoes are clocked out on every transfer but not inspected.

Structurally based on G4.1-ArenaSlim. Same main loop shape, same response framing, same G4 host command opcodes (with G6-dropped commands rejected).

## Build

Requires [PlatformIO](https://platformio.org/) managed via [pixi](https://pixi.sh/).

```
pixi run build           # compile
pixi run deploy          # compile and upload
pixi run deploy-printf   # compile with DEBUG_SERIAL, and upload
```

## Source files

All source files live in `src/`.

| File | Purpose |
|------|---------|
| `main.cpp` | Setup, main loop, interrupt priorities |
| `NetworkManager.h/.cpp` | TCP server, G4 binary protocol parsing, response buffer |
| `SpiManager.h/.cpp` | Dual SPI bus setup, panel-set table iteration, parallel transfers |
| `CommandProcessor.h/.cpp` | Arena state machine, stream-frame handling, refresh timer |
| `G6PanelProtocol.h` | G6 v1 header/parity, opcodes, block sizes |
| `ArenaConfig.h` | Hardcoded G6_2x10 panel-set table |
| `constants.h` | Hardware constants, panel geometry, timing, network port |
| `commands.h` | `ArenaCommands` enum (G4-compatible, G6-dropped commands marked) |

## G6_2x10 hardware (arena_10-10 v1)

Source: [`g6_07-arena-firmware-interface.md`](../../public/docs/development/g6_07-arena-firmware-interface.md) +
[`g6_arena_configs.h`](../../public/docs/development/g6_arena_configs.h).

- **SPI B0** (`SPI`):  MOSI = D11, MISO = D12, SCK = D13 — serves columns 0..4 (silk P1..P5)
- **SPI B1** (`SPI1`): MOSI = D26, MISO = D1,  SCK = D27 — serves columns 5..9 (silk P6..P10)
- **10 CS pins, gating both buses simultaneously** (4 sub-CS per panel column, only 2 used for 2x10):
  - Col 0 / Col 5: D0 (row 0), D2 (row 1)
  - Col 1 / Col 6: D5, D6
  - Col 2 / Col 7: D9, D10
  - Col 3 / Col 8: D28, D29
  - Col 4 / Col 9: D32, D23

`LED_BUILTIN` (D13) is shared with SCK_B0 — the boot blink runs **before** `SPI.begin()` and the firmware never drives D13 after that.

## G4 host command protocol

Same wire framing as G4.1-ArenaSlim:

- **Incoming binary:** `[length, cmd, params...]`
- **Incoming stream:** `[0x32, len_lo, len_hi, frame_data...]` — no `analog_x`/`analog_y` bytes (G6 dropped these)
- **Response:** `[length, status(0=ok), echo_cmd, ascii_message...]`

| Command | Code | Supported | Notes |
|---|---|---|---|
| `ALL_OFF`        | `0x00` | ✓ | Stops refresh, holds dark |
| `STOP_DISPLAY`   | `0x30` | ✓ | Alias for ALL_OFF |
| `ALL_ON`         | `0xFF` | ✓ | Synthesizes a full-bright GS16 oneshot on every panel |
| `STREAM_FRAME`   | `0x32` | ✓ | Frame size: `4 + 20*53` (GS2 = 1064) or `4 + 20*203` (GS16 = 4064) bytes |
| `SET_REFRESH_RATE` | `0x16` | ✓ | Host override of GS-derived default (300 Hz GS16 / 1000 Hz GS2) |
| `GET_ETHERNET_IP_ADDRESS` | `0x66` | ✓ | Returns DHCP-resolved IP as ASCII |
| `DISPLAY_RESET`  | `0x01` | ✗ | Dropped for G6 — responds with a clear error message |
| `SWITCH_GRAYSCALE` | `0x06` | ✗ | Dropped for G6 — `gs_val` is now derived from the streamed payload size or pattern header |
| `TRIAL_PARAMS`   | `0x08` | ✗ | Needs Modes 2/3/4 (not in scope) |
| `SET_FRAME_POSITION` | `0x70` | ✗ | Needs Modes 2/3 (not in scope) |

## Stream-frame interpretation

The host sends **pre-formatted G6 v1 panel blocks** (each 53 bytes for GS2 or 203 bytes for GS16),
already carrying a correct parity bit in the panel-protocol header byte. The 4-byte frame prefix
(`"FR"` + 16-bit little-endian frame index, mirroring [`g6_04-pattern-file-format.md`](../../public/docs/development/g6_04-pattern-file-format.md)) is consumed and ignored.

Frame size for the 2x10 arena (20 panels, row-major panel order: panels `0..9` row 0, `10..19` row 1):

- GS2:  `4 + 20*53  = 1064` bytes
- GS16: `4 + 20*203 = 4064` bytes

The grayscale mode is inferred from the payload size — there is no `SWITCH_GRAYSCALE` command in G6.

The frame buffer is re-transmitted at the configured refresh rate (default: 300 Hz for GS16,
1000 Hz for GS2) using the Oneshot opcodes carried in the host-supplied blocks. If the host
chooses to send Persistent (`0x11`/`0x31`) blocks, re-transmission is harmless overhead.

## Out of scope (intentional)

- SD-card pattern playback (Modes 2 & 3) and the G6 v2 `.pat` reader
- Closed-loop Mode 4 (AIN0 sampling via OPA2277)
- Mode 1 (TSI Position Function) and v2 PSRAM workflow
- v1 Triggered (`0x12`/`0x32`) and Gated (`0x13`/`0x33`) — needs EINT (D33) wiring
- ISP / panel firmware update
- CIPO confirmation validation (panel echoes + CRC-8)
- Controller error display

Pickable as separate follow-up work without churn to the Mode 5 transmission path.
