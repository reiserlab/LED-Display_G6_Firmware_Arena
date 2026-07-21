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
//   ISP uses split SPI clocks: phase A (COPI command) at kIspClockAMhz, phase
//   B (CIPO readback) at a conservative kIspClockBMhz so the wired-OR CIPO
//   return path needs no realign beyond readResp's 2-bit tolerance.
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

  // Split ISP clocks. Phase B (CIPO readback) runs slow on purpose: the
  // buffered wired-OR CIPO return path slips ~1 bit per ~100 ns of fixed
  // delay, so a low clock shrinks the slip toward zero. Phase A (COPI
  // command) has no such constraint; display streaming proves 25 MHz COPI on
  // the same bus. 25 MHz fleet-validated for ISP (20/20 flash + CRC MATCH,
  // arena bench; 10 and 16 MHz also passed on the way up). Do NOT raise
  // kIspClockBMhz.
  static constexpr uint16_t kIspClockAMhz  = 25;   // phase-A COPI clock (fleet-validated)
  static constexpr uint16_t kIspClockBMhz  = 2;    // phase-B CIPO clock (keep conservative)
  static constexpr uint16_t kPageBytes     = 256;  // flash program granule

  // Inter-transaction pacing. The pre-command gap keeps a command from being
  // clocked before the panel is back in panel_spi_read (a missed command
  // makes the panel parse the following phase-B zeros as a bad-header
  // command); the reply wait lets the panel park in its reply-drive loop with
  // the TX FIFO preloaded before phase B.
  //
  // Slow one-shot commands (ENTER / VERIFY_STAGED / COMMIT) keep the proven
  // generous gap: VERIFY_STAGED is built instantly after the last WRITE_PAGE
  // reply and historically exposed the missed-phase-A failure at short gaps
  // (recurred at a uniform 1000 us as a 1-in-20 verify miss on the bench).
  // Three such commands per flash, so generosity costs ~6 ms total.
  static constexpr uint32_t kCmdGapUs = 2000;
  // Per-page pacing (533 per flash, where the stream time lives). Page
  // building (SD read + page CRC) adds its own natural gap on top. 500/500 is
  // fleet-validated (20/20 flash + CRC MATCH, ~616 ms stream; 1000/1000 also
  // passed). No page-loop failure floor was found down to 500; go lower only
  // with a fresh bench sweep.
  static constexpr uint32_t kPageGapUs       = 500;  // pre-command gap, WRITE_PAGE only
  static constexpr uint32_t kPageReplyWaitUs = 500;  // readResp default pre-read wait

  // Poll cadences + ceilings replacing the old fixed 12 s commit and 8 s
  // reboot delays. The ceilings keep the conservative worst cases (first-ever
  // commit formats LittleFS before writing ~130 KB; reboot includes the OTA
  // stub's app-region copy); the polls stop as soon as the panel answers.
  static constexpr uint32_t kVerifyPollMs    = 25;
  static constexpr uint32_t kVerifyTimeoutMs = 3000;
  static constexpr uint32_t kCommitPollMs    = 250;
  static constexpr uint32_t kCommitTimeoutMs = 15000;
  static constexpr uint32_t kAlivePollMs     = 250;
  static constexpr uint32_t kAliveTimeoutMs  = 12000;

  // Panel-protocol v1 COMM_CHECK opcode (payload = bytes 0..199), used by
  // pollPanelAlive. Deliberately not a display command: COMM_CHECK is exempt
  // from the panel's boot-indicator retire filter, so liveness polling leaves
  // the post-flash smiley in place.
  static constexpr uint8_t  kCommCheckCmd  = 0x01;

  // 16-byte ISP-enter sentinel + 4-byte unlock token. Must match panel isp.h.
  static constexpr char     kSentinel[17]  = "G6PANELISPENTER";  // 15 chars + NUL = 16 bytes
  static constexpr uint8_t  kUnlock[4]     = { 'I', 'S', 'P', '!' };

  uint32_t session_nonce_ = 0;

  // Build a v1 ISP COPI message [hdr][cmd][payload...] into `out`, stamping the
  // even-parity header bit. Returns total message length.
  size_t buildMsg(uint8_t *out, uint8_t cmd, const uint8_t *payload, size_t plen);

  // Phase A: clock a COPI message to the panel (CIPO ignored). gap_us is the
  // settle delay before asserting CS: kCmdGapUs (default) for the slow
  // one-shot commands, kPageGapUs for the WRITE_PAGE loop.
  bool sendCmd(uint8_t panel, const uint8_t *msg, size_t len,
               uint32_t gap_us = kCmdGapUs);

  // Phase B: clock `resp_len` zero bytes and capture the panel's armed reply.
  // Verifies the trailing CRC-8 and returns the panel status byte via *status
  // (and any data payload via resp[1..]). Returns false on bus/CRC failure.
  // `wait_us` is how long to let the panel finish processing the phase-A
  // command and park in its reply-drive loop before we clock the read. The
  // default covers fast commands; slow commands (VERIFY_STAGED, COMMIT) use
  // pollResp with wait_us=0 instead of a long blocking wait.
  bool readResp(uint8_t panel, uint8_t *resp, size_t resp_len, uint8_t *status,
                uint32_t wait_us = kPageReplyWaitUs);

  // Repeated phase-B reads every poll_ms until the CRC-8 validates or
  // timeout_ms elapses (first attempt immediate). Early polls that land while
  // the panel is still processing are simply missed by it and read as
  // CRC-invalid garbage here, so they are harmless. On timeout, resp holds
  // the LAST raw capture for diagnostics. polls_out (optional) reports the
  // attempt count for timing prints.
  bool pollResp(uint8_t panel, uint8_t *resp, size_t resp_len, uint8_t *status,
                uint32_t poll_ms, uint32_t timeout_ms, uint32_t *polls_out = nullptr);

  // Post-reboot liveness: clock full COMM_CHECK frames (at the phase-B clock)
  // and validate the 3-byte confirmation that rides the NEXT frame's CIPO.
  // NEVER poll with bare zero-reads: a running panel parses those as a
  // bad-header command and raises an error glyph, clobbering the post-flash
  // smiley.
  bool pollPanelAlive(uint8_t panel, uint32_t poll_ms, uint32_t timeout_ms);
};
