#include "IspController.h"

#include <SD.h>
#include <string.h>
#include <stdio.h>
#include "Crc.h"
#include "G6PanelProtocol.h"
#include "constants.h"

// Out-of-class definitions for the static constexpr members (needed when their
// address is taken, e.g. memcpy of kSentinel/kUnlock).
constexpr char    IspController::kSentinel[17];
constexpr uint8_t IspController::kUnlock[4];

size_t IspController::buildMsg(uint8_t *out, uint8_t cmd,
                               const uint8_t *payload, size_t plen) {
  out[0] = G6::header_version_v1;  // parity stamped below
  out[1] = cmd;
  if (plen) memcpy(out + 2, payload, plen);
  size_t len = 2 + plen;
  G6::stamp_header_parity(out, len);
  return len;
}

bool IspController::sendCmd(uint8_t panel, const uint8_t *msg, size_t len,
                            uint32_t gap_us) {
  // Let the panel settle back into its receive loop (parked waiting for CS)
  // before we assert CS for this command. A command sent immediately after the
  // previous reply read can arrive before the panel is listening again and be
  // missed entirely — the panel then reads the following phase-B zeros as a
  // bad-header command (PE04) and never processes the real command. WRITE_PAGE
  // happens to be slow enough to build (SD read + per-page CRC) to hide this
  // (and passes kPageGapUs); VERIFY_STAGED is built instantly and exposed it,
  // so the slow one-shot commands keep the generous kCmdGapUs default.
  delayMicroseconds(gap_us);
  // Phase A: clock the COPI message at the fast A-clock; ignore CIPO.
  spi_.setSpiClockMhz(kIspClockAMhz);
  return spi_.transferSinglePanel(panel, msg, nullptr, len);
}

bool IspController::readResp(uint8_t panel, uint8_t *resp, size_t resp_len,
                             uint8_t *status, uint32_t wait_us) {
  // Give the panel time to finish processing the phase-A command and park in
  // panel_spi_drive_response() with its TX FIFO pre-loaded, BEFORE we drop CS
  // for the read. If the panel is even slightly late it skips the whole window
  // (its "wait for CS idle" gate) and the read returns the empty line. The
  // default is generous (bench-tunable); pollResp passes wait_us=0 and retries
  // on a cadence instead.
  if (wait_us) delayMicroseconds(wait_us);
  // Phase B captures CIPO: drop to the conservative B-clock (see header).
  spi_.setSpiClockMhz(kIspClockBMhz);

  // Phase B: clock the reply (zeros on COPI; panel drives CIPO). The buffered
  // wired-OR CIPO return path slips by a fixed sub-bit that shows up as a whole
  // 1-bit left shift even at 10 MHz (the display path only realigns ≥20 MHz —
  // not enough here). Clock one EXTRA byte so a 1-bit shift can pull in the
  // trailing byte's MSB, then accept whichever of {no shift, 1-bit shift}
  // validates the trailing CRC-8. Reply = [status][payload...][crc8].
  const size_t n = resp_len + 1;
  uint8_t raw[21] = {0};
  uint8_t tx[21]  = {0};
  if (n > sizeof(raw)) return false;
  if (!spi_.transferSinglePanel(panel, tx, raw, n)) return false;

  for (uint8_t bits = 0; bits <= 2; ++bits) {
    for (size_t i = 0; i < resp_len; ++i) {
      resp[i] = bits ? (uint8_t)((raw[i] << bits) | (raw[i + 1] >> (8 - bits))) : raw[i];
    }
    if (G6::crc8_autosar(resp, resp_len - 1) == resp[resp_len - 1]) {
      if (status) *status = resp[0];
      return true;
    }
  }
  // Neither alignment validated — leave resp as the raw capture for diagnostics.
  for (size_t i = 0; i < resp_len; ++i) resp[i] = raw[i];
  return false;
}

bool IspController::pollResp(uint8_t panel, uint8_t *resp, size_t resp_len,
                             uint8_t *status, uint32_t poll_ms,
                             uint32_t timeout_ms, uint32_t *polls_out) {
  uint32_t t0 = millis();
  uint32_t polls = 0;
  for (;;) {
    ++polls;
    if (readResp(panel, resp, resp_len, status, /*wait_us=*/0)) {
      if (polls_out) *polls_out = polls;
      return true;
    }
    if (millis() - t0 >= timeout_ms) break;
    // Small quasi-random skew so the poll grid never locks onto the panel's
    // reply-parking instant. The panel's drive_response has a microsecond
    // window (RX drain + TX FIFO preload) during which a poll's CS edge
    // consumes the receipt half-driven; with a fixed grid and a deterministic
    // compute time (e.g. the ~51 ms VERIFY_STAGED CRC) an unlucky panel hits
    // that window on EVERY attempt (seen on the bench: identical
    // "verify-staged garbled CIPO: 81 .." failures back to back). The skew
    // makes each poll, and each retry, sample a different phase.
    delay(poll_ms + (millis() % 5));
  }
  if (polls_out) *polls_out = polls;
  return false;
}

bool IspController::pollPanelAlive(uint8_t panel, uint32_t poll_ms,
                                   uint32_t timeout_ms) {
  // Canonical COMM_CHECK frame: [hdr][0x01][0..199], parity-stamped.
  uint8_t cc[2 + 200];
  cc[0] = G6::header_version_v1;
  cc[1] = kCommCheckCmd;
  for (int i = 0; i < 200; ++i) cc[2 + i] = (uint8_t)i;
  G6::stamp_header_parity(cc, sizeof(cc));

  // Expected 3-byte confirmation the panel arms for the NEXT transaction:
  // chk = CRC-8/AUTOSAR over the frame with the header parity bit masked;
  // hdr carries even parity over the {version, cmd, chk} triple (panel
  // message.cpp: calculate_crc8 / header_with_parity_for_3byte).
  uint8_t masked0 = (uint8_t)(cc[0] & 0x7F);
  uint8_t stamped0 = cc[0];
  cc[0] = masked0;
  uint8_t exp_chk = G6::crc8_autosar(cc, sizeof(cc));
  cc[0] = stamped0;
  unsigned ones = (unsigned)__builtin_popcount(masked0) +
                  (unsigned)__builtin_popcount(kCommCheckCmd) +
                  (unsigned)__builtin_popcount(exp_chk);
  uint8_t exp_hdr = (uint8_t)(masked0 | ((ones & 1u) << 7));

  uint8_t rx[sizeof(cc)] = {0};
  uint32_t t0 = millis();
  bool primed = false;  // becomes true once a frame may have been ingested
  for (;;) {
    spi_.setSpiClockMhz(kIspClockBMhz);  // CIPO capture: conservative clock
    if (spi_.transferSinglePanel(panel, cc, rx, sizeof(cc)) && primed) {
      // Same 0..2-bit CIPO realign tolerance as readResp.
      for (uint8_t bits = 0; bits <= 2; ++bits) {
        uint8_t b0 = bits ? (uint8_t)((rx[0] << bits) | (rx[1] >> (8 - bits))) : rx[0];
        uint8_t b1 = bits ? (uint8_t)((rx[1] << bits) | (rx[2] >> (8 - bits))) : rx[1];
        uint8_t b2 = bits ? (uint8_t)((rx[2] << bits) | (rx[3] >> (8 - bits))) : rx[2];
        if (b0 == exp_hdr && b1 == kCommCheckCmd && b2 == exp_chk) return true;
      }
    }
    primed = true;
    if (millis() - t0 >= timeout_ms) return false;
    delay(poll_ms);
  }
}

bool IspController::programPanel(uint8_t panel_index, char *msg, size_t msg_len) {
  auto setMsg = [&](const char *m) { snprintf(msg, msg_len, "%s", m); };

  // --- 1. Open image + validate the 32-byte footer ---------------------------
  File f = SD.open(AC::constants::firmware_path, FILE_READ);
  if (!f) { setMsg("no firmware on SD"); return false; }
  uint32_t file_size = (uint32_t)f.size();
  constexpr uint8_t FOOT = AC::constants::firmware_footer_byte_count;  // 32
  if (file_size <= FOOT) { f.close(); setMsg("firmware too small"); return false; }

  uint8_t footer[FOOT];
  f.seek(file_size - FOOT);
  if (f.read(footer, FOOT) != (size_t)FOOT) { f.close(); setMsg("footer read failed"); return false; }
  if (memcmp(footer, "G6PANFW", 7) != 0) { f.close(); setMsg("bad footer magic"); return false; }
  uint32_t image_crc32, image_size;
  memcpy(&image_crc32, footer + 24, 4);  // u32 LE
  memcpy(&image_size,  footer + 28, 4);  // u32 LE
  if (image_size != file_size - FOOT) { f.close(); setMsg("footer size mismatch"); return false; }

  // --- 2. Save the caller's SPI clock (per-phase clocks are set inside
  //        sendCmd/readResp; see kIspClockAMhz/kIspClockBMhz) ------------------
  uint16_t saved_mhz = spi_.getSpiClockMhz();
  spi_.setSpiClockMhz(kIspClockBMhz);  // safe default between phases

  bool ok = false;
  uint8_t resp[20];
  uint8_t status = 0xFF;
  uint8_t msgbuf[2 + 3 + 4 + kPageBytes + 4];  // largest message = WRITE_PAGE
  size_t mlen;

  // Per-phase wall-clock instrumentation (DBG_PRINTF lines + a compact summary
  // in the host-visible result msg) so pacing/clock changes are measurable.
  uint32_t t_total0 = millis();
  uint32_t stream_ms = 0, verify_ms = 0, commit_ms = 0, boot_ms = 0;
  uint32_t pages_sent = 0;

  do {
    // --- 3. ISP_ENTER --------------------------------------------------------
    uint8_t enter_payload[20];
    memcpy(enter_payload, kSentinel, 16);     // 15 chars + NUL = 16 bytes
    memcpy(enter_payload + 16, kUnlock, 4);
    mlen = buildMsg(msgbuf, AC::ISP_ENTER, enter_payload, sizeof(enter_payload));
    // reply: status(1) nonce(4) flash(4) page(2) sector(2) appcrc(4) bootrom(1) crc8 = 19
    // One internal retry: a just-flashed panel doing its one-time boot-indicator
    // flag erase misses commands for up to a few hundred ms (kEnterRetryDelayMs).
    bool crc_ok = false;
    bool send_ok = true;
    for (uint8_t attempt = 1; attempt <= 2 && !crc_ok && send_ok; ++attempt) {
      if (attempt == 2) delay(kEnterRetryDelayMs);
      send_ok = sendCmd(panel_index, msgbuf, mlen);
      if (!send_ok) break;
      memset(resp, 0, sizeof(resp));
      crc_ok = readResp(panel_index, resp, 19, &status);
    }
    if (!send_ok) { setMsg("panel not in arena map"); break; }
    if (!crc_ok) {
      // Distinguish a silent panel from a garbled return so the operator knows
      // where to look. All-zero CIPO == nothing driven back.
      // A genuine reply carries a nonzero payload (nonce + flash/page/sector
      // constants) in bytes 2..18. All-zero there means the panel drove nothing
      // back — almost always the addressed panel isn't running this ISP firmware
      // (or isn't wired / CIPO bus dead). `C0 80 …` etc. is just idle-line residue.
      bool no_payload = true;
      for (int i = 2; i < 19; ++i) { if (resp[i]) { no_payload = false; break; } }
      if (no_payload) {
        setMsg("ISP_ENTER: no valid reply — is the panel at this index running this ISP firmware? (also check wiring/CIPO)");
        break;
      }
      // Garbled but non-empty: a real alignment problem. Dump the raw 19 CIPO
      // bytes so the offset/corruption is visible.
      // Aligned reply = 00 <nonce4> 00 00 20 00 00 01 00 10 00 00 00 00 00 <crc8>.
      char hx[3 * 19 + 1];
      int o = 0;
      for (int i = 0; i < 19; ++i) o += snprintf(hx + o, sizeof(hx) - o, "%02X ", resp[i]);
      snprintf(msg, msg_len, "ISP_ENTER garbled CIPO: %s", hx);
      break;
    }
    if (status != 0) {
      snprintf(msg, msg_len, "ISP_ENTER rejected by panel (status=%u — bad sentinel/token or PSRAM alloc)",
               (unsigned)status);
      break;
    }
    memcpy(&session_nonce_, resp + 1, 4);

    DBG_PRINTF("[isp] enter ok, %lu ms\n", (unsigned long)(millis() - t_total0));

    // --- 4. Stream the image into panel PSRAM --------------------------------
    uint32_t t_stream0 = millis();
    uint32_t pages = (image_size + kPageBytes - 1) / kPageBytes;
    f.seek(0);
    bool page_ok = true;
    for (uint32_t p = 0; p < pages; ++p) {
      uint8_t page[kPageBytes];
      memset(page, 0xFF, sizeof(page));
      uint32_t remain = image_size - p * kPageBytes;
      uint32_t want   = remain < kPageBytes ? remain : kPageBytes;
      if (f.read(page, want) != want) { setMsg("SD read error"); page_ok = false; break; }
      uint32_t pcrc = G6::crc32_update(0xFFFFFFFFu, page, kPageBytes) ^ 0xFFFFFFFFu;
      uint8_t pl[3 + 4 + kPageBytes + 4];
      pl[0] = p & 0xFF; pl[1] = (p >> 8) & 0xFF; pl[2] = (p >> 16) & 0xFF;
      memcpy(pl + 3, &session_nonce_, 4);
      memcpy(pl + 7, page, kPageBytes);
      memcpy(pl + 7 + kPageBytes, &pcrc, 4);
      mlen = buildMsg(msgbuf, AC::ISP_WRITE_PAGE, pl, sizeof(pl));
      if (!sendCmd(panel_index, msgbuf, mlen, kPageGapUs)) { setMsg("write-page send failed"); page_ok = false; break; }
      if (!readResp(panel_index, resp, 2, &status) || status != 0) {
        snprintf(msg, msg_len, "ISP_WRITE_PAGE %lu failed", (unsigned long)p);
        page_ok = false; break;
      }
    }
    if (!page_ok) break;
    pages_sent = pages;
    stream_ms = millis() - t_stream0;
    DBG_PRINTF("[isp] stream %lu pages in %lu ms (%lu us/page)\n",
               (unsigned long)pages, (unsigned long)stream_ms,
               (unsigned long)(pages ? stream_ms * 1000ul / pages : 0));

    // --- 5. ISP_VERIFY_STAGED (PSRAM staging buffer) -------------------------
    {
      uint32_t t_verify0 = millis();
      uint32_t polls = 0;
      uint8_t pl[4 + 3 + 4];
      memcpy(pl, &session_nonce_, 4);
      pl[4] = image_size & 0xFF; pl[5] = (image_size >> 8) & 0xFF; pl[6] = (image_size >> 16) & 0xFF;
      memcpy(pl + 7, &image_crc32, 4);
      mlen = buildMsg(msgbuf, AC::ISP_VERIFY_STAGED, pl, sizeof(pl));
      if (!sendCmd(panel_index, msgbuf, mlen)) { setMsg("verify-staged send failed"); break; }
      // Panel CRCs the whole staged image in PSRAM before replying; poll for
      // the receipt instead of one long blocking wait.
      bool got = pollResp(panel_index, resp, 6, &status,
                          kVerifyPollMs, kVerifyTimeoutMs, &polls);
      verify_ms = millis() - t_verify0;
      DBG_PRINTF("[isp] verify-staged %lu ms (%lu polls)\n",
                 (unsigned long)verify_ms, (unsigned long)polls);
      if (!got) {
        // Dump the raw 6-byte capture so we can see whether the panel drove a
        // reply at all (empty/residue → panel faulted/hung in the CRC scan;
        // misaligned data → CIPO realign issue).
        char hx[3 * 6 + 1];
        int o = 0;
        for (int i = 0; i < 6; ++i) o += snprintf(hx + o, sizeof(hx) - o, "%02X ", resp[i]);
        snprintf(msg, msg_len, "verify-staged garbled CIPO: %s", hx);
        break;
      }
      if (status != 0) {
        // resp = [status][computed_crc32 LE ×4][crc8]. Surface the panel's
        // computed CRC vs expected so a length/range/PSRAM issue is diagnosable.
        uint32_t got = (uint32_t)resp[1] | ((uint32_t)resp[2] << 8) |
                       ((uint32_t)resp[3] << 16) | ((uint32_t)resp[4] << 24);
        snprintf(msg, msg_len, "staged CRC mismatch: panel=0x%08lX expected=0x%08lX (len=%lu)",
                 (unsigned long)got, (unsigned long)image_crc32, (unsigned long)image_size);
        break;
      }
    }

    // --- 6. ISP_COMMIT — panel stages the verified image into its LittleFS + an
    //     OTA command, replies, then reboots so the core's boot-time OTA stub
    //     (flash offset 0) copies it into the app region and boots the new
    //     firmware. The panel writes ~96 KB to LittleFS before replying (first
    //     commit may format the FS first); the receipt is POLLED with a
    //     kCommitTimeoutMs ceiling. (The non-destructive scratch probe,
    //     ISP_PROBE_SCRATCH 0xEA, remains in panel firmware for manual diagnosis.)
    {
      uint32_t t_commit0 = millis();
      uint32_t polls = 0;
      uint8_t pl[4 + 3];
      memcpy(pl, &session_nonce_, 4);
      pl[4] = image_size & 0xFF; pl[5] = (image_size >> 8) & 0xFF; pl[6] = (image_size >> 16) & 0xFF;
      mlen = buildMsg(msgbuf, AC::ISP_COMMIT, pl, sizeof(pl));
      if (!sendCmd(panel_index, msgbuf, mlen)) { setMsg("commit send failed"); break; }
      bool got = pollResp(panel_index, resp, 2, &status,
                          kCommitPollMs, kCommitTimeoutMs, &polls);
      commit_ms = millis() - t_commit0;
      DBG_PRINTF("[isp] commit receipt %lu ms (%lu polls)\n",
                 (unsigned long)commit_ms, (unsigned long)polls);
      if (!got) {
        setMsg("commit: no receipt within 15 s; run g6-verify-panel to check whether the flash took");
        break;
      }
      if (status == 8) { setMsg("commit: OTA staging failed on panel (LittleFS/space?)"); break; }
      if (status != 0) { snprintf(msg, msg_len, "commit rejected (status %u)", (unsigned)status); break; }
    }
    // Panel is rebooting; the OTA stub copies the staged image into flash and
    // boots. Poll for the panel to answer a COMM_CHECK instead of a fixed wait.
    {
      uint32_t t_boot0 = millis();
      bool alive = pollPanelAlive(panel_index, kAlivePollMs, kAliveTimeoutMs);
      boot_ms = millis() - t_boot0;
      DBG_PRINTF("[isp] reboot -> alive %lu ms (%s)\n",
                 (unsigned long)boot_ms, alive ? "ok" : "TIMEOUT");
      if (!alive) {
        setMsg("panel did not come back within 12 s after OTA reboot; check power/wiring, then g6-verify-panel");
        break;
      }
    }
    snprintf(msg, msg_len,
             "panel %u flashed via OTA (%lu bytes) in %lu.%01lu s "
             "(stream %lu + verify %lu + commit %lu + boot %lu ms, %lu pages)",
             (unsigned)(panel_index + 1), (unsigned long)image_size,  // report 1-based
             (unsigned long)((millis() - t_total0) / 1000ul),
             (unsigned long)(((millis() - t_total0) % 1000ul) / 100ul),
             (unsigned long)stream_ms, (unsigned long)verify_ms,
             (unsigned long)commit_ms, (unsigned long)boot_ms,
             (unsigned long)pages_sent);
    ok = true;
  } while (false);

  f.close();
  spi_.setSpiClockMhz(saved_mhz);  // restore the caller's SPI clock
  return ok;
}

bool IspController::verifyPanel(uint8_t panel_index, char *msg, size_t msg_len) {
  auto setMsg = [&](const char *m) { snprintf(msg, msg_len, "%s", m); };

  // Read the SD footer (expected image_size + CRC) — no need to stream the image.
  File f = SD.open(AC::constants::firmware_path, FILE_READ);
  if (!f) { setMsg("no firmware on SD"); return false; }
  uint32_t file_size = (uint32_t)f.size();
  constexpr uint8_t FOOT = AC::constants::firmware_footer_byte_count;  // 32
  if (file_size <= FOOT) { f.close(); setMsg("firmware too small"); return false; }
  uint8_t footer[FOOT];
  f.seek(file_size - FOOT);
  size_t got_n = f.read(footer, FOOT);
  f.close();
  if (got_n != (size_t)FOOT) { setMsg("footer read failed"); return false; }
  if (memcmp(footer, "G6PANFW", 7) != 0) { setMsg("bad footer magic"); return false; }
  uint32_t image_crc32, image_size;
  memcpy(&image_crc32, footer + 24, 4);
  memcpy(&image_size,  footer + 28, 4);

  uint16_t saved_mhz = spi_.getSpiClockMhz();
  spi_.setSpiClockMhz(kIspClockBMhz);  // safe default between phases

  bool ok = false;
  uint8_t resp[20];
  uint8_t status = 0xFF;
  uint8_t msgbuf[2 + 3 + 4 + kPageBytes + 4];
  size_t mlen;

  do {
    // ISP_ENTER (get session nonce; no reboot, panel keeps running)
    uint8_t ep[20];
    memcpy(ep, kSentinel, 16);
    memcpy(ep + 16, kUnlock, 4);
    mlen = buildMsg(msgbuf, AC::ISP_ENTER, ep, sizeof(ep));
    // Same one-shot ENTER retry as programPanel (post-flash flag-erase window).
    bool crc_ok = false;
    bool send_ok = true;
    for (uint8_t attempt = 1; attempt <= 2 && !crc_ok && send_ok; ++attempt) {
      if (attempt == 2) delay(kEnterRetryDelayMs);
      send_ok = sendCmd(panel_index, msgbuf, mlen);
      if (!send_ok) break;
      memset(resp, 0, sizeof(resp));
      crc_ok = readResp(panel_index, resp, 19, &status);
    }
    if (!send_ok) { setMsg("panel not in arena map"); break; }
    if (!crc_ok || status != 0) {
      setMsg("ISP_ENTER: no valid reply (is the panel running this ISP firmware?)");
      break;
    }
    memcpy(&session_nonce_, resp + 1, 4);

    // ISP_VERIFY_CRC over the RUNNING app region [0, image_size) (read via XIP).
    uint8_t pl[3 + 3 + 4 + 4];
    pl[0] = 0; pl[1] = 0; pl[2] = 0;  // start = 0
    pl[3] = image_size & 0xFF; pl[4] = (image_size >> 8) & 0xFF; pl[5] = (image_size >> 16) & 0xFF;
    memcpy(pl + 6, &session_nonce_, 4);
    memcpy(pl + 10, &image_crc32, 4);
    mlen = buildMsg(msgbuf, AC::ISP_VERIFY_CRC, pl, sizeof(pl));
    if (!sendCmd(panel_index, msgbuf, mlen)) { setMsg("verify-crc send failed"); break; }
    // Panel CRCs the app region via XIP before replying; poll for the receipt.
    if (!pollResp(panel_index, resp, 6, &status, kVerifyPollMs, kVerifyTimeoutMs)) {
      setMsg("verify-crc: no/garbled reply");
      break;
    }
    uint32_t got = (uint32_t)resp[1] | ((uint32_t)resp[2] << 8) |
                   ((uint32_t)resp[3] << 16) | ((uint32_t)resp[4] << 24);
    bool match = (status == 0);
    snprintf(msg, msg_len,
             "panel %u running-app CRC=0x%08lX expected=0x%08lX -> %s",
             (unsigned)(panel_index + 1), (unsigned long)got, (unsigned long)image_crc32,  // 1-based
             match ? "MATCH (this firmware is installed)"
                   : "MISMATCH (a different firmware is running)");
    ok = match;
  } while (false);

  spi_.setSpiClockMhz(saved_mhz);
  return ok;
}
