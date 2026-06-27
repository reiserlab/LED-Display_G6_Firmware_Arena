#pragma once

#include <Arduino.h>
#include "SpiManager.h"

// ───────────────────────────────────────────────────────────────────────────
// IspController — controller-side driver for in-system programming of a single
// panel over SPI. Host commands: g6-program-panel (0xC8), g6-verify-panel (0xC9).
//
// Reads /firmware/panel.bin from the controller SD, validates its 32-byte
// footer, then runs the ISP sequence against ONE panel (selected by CS):
//
//   ISP_ENTER  → stage image into panel PSRAM via ISP_WRITE_PAGE×N
//              → ISP_VERIFY_STAGED → ISP_COMMIT
//
//   ISP_COMMIT makes the panel write the verified image to its LittleFS + an OTA
//   command and reboot; the panel's OTA stub then flashes it at early-boot (see
//   panel isp.{h,cpp}). There is no controller-side post-commit step — the panel
//   reboots itself. Use g6-verify-panel afterwards (ISP_ENTER + ISP_VERIFY_CRC
//   over the running app) to confirm the installed image's CRC.
//
// WIRE PROTOCOL (MUST stay in sync with the panel's isp.{h,cpp}).
//   Panel protocol v1 header (0x01 / 0x81, even-parity bit), opcode block
//   0xE4–0xE9. Each command is one CS-asserted transaction (phase A: COPI). A
//   command that returns data is followed by a SECOND CS-asserted read
//   transaction (phase B) of a fixed length, during which the panel — now in a
//   "response-armed" state — drives its reply on CIPO. Reply framing:
//       [status(1)] [payload...] [crc8]
//   where crc8 is CRC-8/AUTOSAR over status+payload, and status 0 == OK.
//
//   ISP is run at a conservative SPI clock (kIspClockMhz) so the wired-OR CIPO
//   return path needs no high-clock 1-bit realign.
// ───────────────────────────────────────────────────────────────────────────

namespace AC {

// ISP opcodes (panel-protocol v1 namespace). Must match panel isp.h.
enum IspOpcode : uint8_t {
  ISP_ENTER         = 0xE4,
  ISP_WRITE_PAGE    = 0xE5,
  ISP_VERIFY_STAGED = 0xE6,
  ISP_COMMIT        = 0xE7,
  ISP_VERIFY_CRC    = 0xE8,
  ISP_EXIT_REBOOT   = 0xE9,
};

}  // namespace AC

class IspController {
 public:
  explicit IspController(SpiManager &spi) : spi_(spi) {}

  // Reflash one panel from /firmware/panel.bin. Returns true on success. On
  // return, `msg` holds a short human-readable result / failure reason (the
  // last step reached), suitable for the 0xC8 host response.
  bool programPanel(uint8_t panel_index, char *msg, size_t msg_len);

  // CRC the panel's RUNNING application flash (region [0, image_size) read via
  // XIP on the panel) against the /firmware/panel.bin footer CRC. Returns true
  // on MATCH (that image is what's installed). Does ISP_ENTER + ISP_VERIFY_CRC;
  // no reboot. Useful to confirm an OTA flash actually took.
  bool verifyPanel(uint8_t panel_index, char *msg, size_t msg_len);

 private:
  SpiManager &spi_;

  // ISP runs slow on purpose: the buffered wired-OR CIPO return path slips ~1
  // bit per ~100 ns of fixed delay, so a low clock shrinks the slip toward zero
  // and gives big timing margin for the panel's reply (speed is irrelevant for a
  // one-shot reflash). Bench-tunable — raise once readback is proven reliable.
  static constexpr uint16_t kIspClockMhz   = 2;    // conservative ISP SCK
  static constexpr uint16_t kPageBytes     = 256;  // flash program granule
  static constexpr uint32_t kCommitWaitMs  = 12000; // first commit may format LittleFS + write ~130 KB before the receipt
  static constexpr uint32_t kRebootWaitMs  = 8000; // reboot + OTA-stub copy (~96 KB) + boot

  // 16-byte ISP-enter sentinel + 4-byte unlock token. Must match panel isp.h.
  static constexpr char     kSentinel[17]  = "G6PANELISPENTER";  // 15 chars + NUL = 16 bytes
  static constexpr uint8_t  kUnlock[4]     = { 'I', 'S', 'P', '!' };

  uint32_t session_nonce_ = 0;

  // Build a v1 ISP COPI message [hdr][cmd][payload...] into `out`, stamping the
  // even-parity header bit. Returns total message length.
  size_t buildMsg(uint8_t *out, uint8_t cmd, const uint8_t *payload, size_t plen);

  // Phase A: clock a COPI message to the panel (CIPO ignored).
  bool sendCmd(uint8_t panel, const uint8_t *msg, size_t len);

  // Phase B: clock `resp_len` zero bytes and capture the panel's armed reply.
  // Verifies the trailing CRC-8 and returns the panel status byte via *status
  // (and any data payload via resp[1..]). Returns false on bus/CRC failure.
  // `wait_ms` is how long to let the panel finish processing the phase-A command
  // and park in its reply-drive loop before we clock the read. Default 2 ms is
  // fine for fast commands; the CRC-over-image steps (VERIFY_STAGED/VERIFY_CRC)
  // need much longer because the panel scans ~100 KB of PSRAM/flash first.
  bool readResp(uint8_t panel, uint8_t *resp, size_t resp_len, uint8_t *status,
                uint16_t wait_ms = 2);
};
