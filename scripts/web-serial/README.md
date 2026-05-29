# Web Serial G6 Arena Controller

A tiny browser UI that talks to the Teensy 4.1 controller over its USB CDC
serial port using the [Web Serial API](https://developer.mozilla.org/en-US/docs/Web/API/Web_Serial_API).

Compared to the TCP-over-Direct-Sockets sibling in `../web/`, this page
needs no Isolated Web App, no special install — any modern Chromium-based
browser running on a desktop OS can open it from disk (`file://`) or any
HTTPS origin and pick the Teensy port from the system chooser.

## Prerequisites

- Teensy 4.1 flashed with the G6-ArenaSlim firmware that includes the USB
  serial command path (`SerialManager`). The same build serves both TCP
  and USB CDC.
- A Chromium-based browser ≥ v89: Chrome, Edge, Opera, Brave, Arc. **Firefox
  and Safari do not implement Web Serial** and will show a warning banner.
- The Teensy USB cable plugged into the same machine running the browser.

## Usage

1. Open `index.html` in Chrome. Either:
   - `file://` open is fine — Web Serial works from `file://` origins.
   - or serve the directory: `python -m http.server -d scripts/web-serial`.
2. Click **Connect to Teensy**. The browser shows the OS serial-port
   chooser; pick the Teensy port (typically `/dev/ttyACM*` on Linux,
   `COM*` on Windows, `/dev/cu.usbmodem*` on macOS).
3. Drive the controller with the buttons:
   - **All On** / **All Off** / **Stop** — `0xFF` / `0x00` / `0x30`.
   - **Get IP** (`0x66`) — prints the DHCP-resolved address.
   - **Get Controller Info** (`0x67`) — prints `{version, capability}` and
     decodes the capability bitmap (`g6_mode`, `v2_local_storage`, …).
   - **Set Refresh** (`0x16`) — host override of the re-transmit rate.
   - **SD pattern playback** — pick a Mode (2 open-loop / 3 show-frame /
     4 closed-loop), a 1-based **Pattern ID** (a `*.pat` file in `/patterns`
     on the SD card), frame rate, gain, and initial position, then
     **Send trial-params** (`0x08`). In Mode 3, jump to a specific frame with
     **Set frame position** (`0x70`).
   - **Stream test frame** (`0x32`, Mode 5) — builds a full frame of
     parity-correct panel blocks in the browser (GS2/GS16 all-on or a GS2
     checkerboard) and streams it, exercising the host-streaming path.
   - Or paste a raw hex string (e.g. `01 16 64 00` = set refresh to 100 Hz)
     and click **Send bytes**.
4. Responses from the controller are decoded and printed in the log pane.

> SD pattern playback needs a FAT-formatted microSD card in the Teensy 4.1
> slot with v2 `.pat` files under `/patterns`. Without a card (or with a bad
> pattern) the controller shows a "CE / NN" error glyph on the arena and
> replies with a non-zero status.

## Wire format

Identical to the TCP path:

- Binary commands: `[length, cmd, params...]` — `length` counts everything
  after itself, so `01 FF` is "1 remaining byte: cmd 0xFF (ALL_ON)".
- Stream frames: `[0x32, len_lo, len_hi, ...payload]` — full G6 v1 panel
  blocks concatenated, no analog_x/analog_y.
- Responses: `[length, status, echo_cmd, ...ascii_message]`.

See the firmware's `src/commands.h` and the project root `CLAUDE.md` for the
full command list.

## Notes

- `pixi run webserial` serves this directory with caching disabled
  (`Cache-Control: no-store` + conditional-request stripping), so edits to
  `index.html` / `main.js` always show up on reload. If you opened the page
  before this was in place and still see an old layout, hard-reload once
  (Ctrl+Shift+R).
- Web Serial is **per-origin**. If you open the page from `file://` and
  later from a local HTTP server, you'll be prompted to authorize the port
  again for the new origin.
- The `baudRate` passed to `port.open()` is meaningless on USB CDC — the
  Teensy honors any value. We use `115200` as a conventional placeholder.
- In `DEBUG_SERIAL` firmware builds, the same USB CDC pipe carries
  `DBG_PRINTF` diagnostic text. The framing parser will not crash on it
  but will treat the first byte of any unexpected text as a length and
  log nonsense until it resyncs after a fresh command. For interactive
  use prefer a non-`DEBUG_SERIAL` build.
- Only one process can hold the USB serial port at a time. If `pio device
  monitor`, `dwfpy`, or another terminal is connected, disconnect it
  before clicking **Connect** in the browser.
