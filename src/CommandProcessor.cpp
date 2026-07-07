#include "CommandProcessor.h"
#include "Crc.h"
#include "ErrorGlyph.h"
#include <Wire.h>

using namespace AC;
using namespace AC::constants;

void CommandProcessor::begin() {
  Wire.begin();
  Wire.setClock(400000);

  // Digital IO boot roles (#135). Port 1 ("Digital IO 1 (5V)", J3): a driven-
  // LOW programmable output, as this firmware has always booted. Port 2
  // ("Digital IO 2 (5V)", J4): in_trigger — U3 in B→A so the BNC feeds the
  // panels' EINT net through the J30 shunt (external trigger for the
  // triggered/gated panel display modes). This REPLACES the old pair of
  // main.cpp setupExternalTriggerInput() + unconditional output init here,
  // which together left D35 driving the EINT net against U3 (boot contention)
  // — applyDioRole tri-states the data pin before flipping direction.
  // Teensy pin 33 (R25→EINT) stays at its power-on INPUT default so nothing
  // else drives the EINT net.
  applyDioRole(1, DioRole::kOutProgrammable);
  applyDioRole(2, DioRole::kInTrigger);
}

// Apply a DIO role with contention-safe pin-transition ordering. The
// SN74LVC1T45 translator drives its A side (our data pin's net) whenever
// DIR=LOW, so: to an output role, flip DIR HIGH FIRST (translator releases the
// A net; the BNC floats for ~µs) and only then drive the data pin; to an
// input/off role, tri-state the data pin FIRST and only then flip DIR LOW.
// Note for rigs with the J30 shunt installed: port 2's A net feeds the panels'
// EINT fanout through R216 regardless of role — an out_programmable port 2
// pulses the panel trigger net whenever it toggles (hardware property, not
// firmware-preventable).
void CommandProcessor::applyDioRole(uint8_t port, DioRole role) {
  const uint8_t data = (port == 1) ? do1_data_pin : do2_data_pin;
  const uint8_t dir  = (port == 1) ? do1_dir_pin  : do2_dir_pin;
  pinMode(dir, OUTPUT);
  if (role == DioRole::kOutProgrammable || role == DioRole::kOutFramescan) {
    digitalWrite(dir, HIGH);   // translator A side becomes an input (stops driving)
    pinMode(data, OUTPUT);
    digitalWrite(data, LOW);   // defined level before anything observes the BNC
  } else {
    pinMode(data, INPUT);      // we stop driving FIRST
    digitalWrite(dir, LOW);    // then the translator turns around (BNC → A net)
  }
  dio_role_[port - 1] = role;
  // Keep the SpiManager frame-scan gate in sync (one slot per port).
  spi_.setFramescanGatePins(
      dio_role_[0] == DioRole::kOutFramescan ? (int16_t)do1_data_pin : (int16_t)-1,
      dio_role_[1] == DioRole::kOutFramescan ? (int16_t)do2_data_pin : (int16_t)-1);
  DBG_PRINTF("[cmd] dio role port=%u -> %s\n", (unsigned)port, dioRoleName(role));
}

const char *CommandProcessor::dioRoleName(DioRole role) const {
  switch (role) {
    case DioRole::kOff:             return "off";
    case DioRole::kInTrigger:       return "in_trigger";
    case DioRole::kOutProgrammable: return "out_programmable";
    case DioRole::kOutFramescan:    return "out_debug_framescan";
  }
  return "?";
}

// ---------------------------------------------------------------------------
// Command dispatch
// ---------------------------------------------------------------------------

void CommandProcessor::processCommand() {
  // Drain one command from each source per loop iteration. Net first
  // (lower-latency TCP path), then serial. Each source uses its own
  // response buffer; current_source_ tells the handlers which one to
  // send the reply back through.
  //
  // A source with an active 0x84 download (dl_active_, issue #16 fix 2),
  // 0x85 upload (ul_active_), or 0x8A archive (ar_active_, PR #27 review
  // point 2) is skipped here even if it has a newly-parsed command waiting.
  //
  // Downloads and archives: sendRaw() flushes any queued response frame
  // before writing raw bytes, so handling a new command on that same source
  // would splice a framed response into the middle of the in-flight raw
  // byte stream, corrupting it from the client's point of view.
  //
  // Uploads: a command that hands off to serviceUpload() is deliberately
  // left un-consumed (hasCommand() stays true) so parseIncoming() doesn't
  // mistake its still-arriving raw file bytes for a new framed command.
  // handleBulkWriteCommand()'s return value says whether THIS call handed
  // off (true) or was fully, synchronously handled already (false). See
  // the ul_* fields' comment in CommandProcessor.h (PR #27 review point 1)
  // for why consuming based on the stale global ul_active_ flag instead let
  // a REJECTED command get redispatched, and re-rejected, on every loop()
  // iteration for as long as an UNRELATED transfer on the other source ran.
  //
  // Either way, the command stays queued until the transfer finishes. The
  // OTHER source is unaffected — that's the concurrency fix 2 is for.
  if (net_.hasCommand() && !(dl_active_ && dl_source_ == &net_)
                        && !(ul_active_ && ul_source_ == &net_)
                        && !(ar_active_ && ar_source_ == &net_)) {
    current_source_ = &net_;
    const ParsedCommand &cmd = net_.command();
    if (cmd.is_bulk) {
      bool async_handoff = handleBulkWriteCommand(cmd);
      if (!async_handoff) net_.commandConsumed();
    } else if (cmd.is_stream) {
      handleStreamCommand(cmd);
      net_.commandConsumed();
    } else {
      handleBinaryCommand(cmd);
      net_.commandConsumed();
    }
  }
  if (serial_.hasCommand() && !(dl_active_ && dl_source_ == &serial_)
                           && !(ul_active_ && ul_source_ == &serial_)
                           && !(ar_active_ && ar_source_ == &serial_)) {
    current_source_ = &serial_;
    const ParsedCommand &cmd = serial_.command();
    if (cmd.is_bulk) {
      bool async_handoff = handleBulkWriteCommand(cmd);
      if (!async_handoff) serial_.commandConsumed();
    } else if (cmd.is_stream) {
      handleStreamCommand(cmd);
      serial_.commandConsumed();
    } else {
      handleBinaryCommand(cmd);
      serial_.commandConsumed();
    }
  }
  current_source_ = nullptr;
}

void CommandProcessor::handleBinaryCommand(const ParsedCommand &cmd) {
  const uint8_t *buf = cmd.data;
  uint8_t claimed_len = buf[0];
  if (cmd.data_len - 1 != claimed_len) {
    // Malformed framing — drop silently to keep the wire quiet.
    return;
  }

  uint8_t pos = 1;
  uint8_t command_byte = buf[pos++];
  DBG_PRINTF("[cmd] handleBinary cmd=0x%02X len=%u\n",
             command_byte, (unsigned)cmd.data_len);

  switch (command_byte) {
    case ALL_OFF_CMD:
      enterAllOff();
      current_source_->sendResponse(command_byte, 0, "All-Off Received");
      break;

    case ALL_ON_CMD:
      // PR #27 review point 8: a state transition mid-transfer doesn't
      // disrupt the transfer itself (serviceDownload()/serviceUpload()/
      // serviceArchive() don't read state_ at all) or corrupt anything (SD
      // is SDIO, not shared with the panel SPI path, and loop() is
      // single-threaded so accesses are naturally serialized), but it's
      // still confusing at the bench to have the arena visibly change while
      // an SD transfer nobody can see is still in flight, so refuse it.
      if (dl_active_ || ul_active_ || ar_active_) {
        current_source_->sendResponse(command_byte, 1, "SD transfer in progress");
        break;
      }
      enterAllOn();
      current_source_->sendResponse(command_byte, 0, "All-On Received");
      break;

    case STOP_DISPLAY_CMD:
      enterAllOff();
      current_source_->sendResponse(command_byte, 0, "Display has been stopped");
      break;

    case SET_REFRESH_RATE_CMD: {
      if (claimed_len != 3) {
        current_source_->sendResponse(command_byte, 1, "Expected [03 16 hz_lo hz_hi]");
        break;
      }
      uint16_t rate;
      memcpy(&rate, buf + pos, sizeof(rate));
      if (rate > 0) {
        refresh_rate_hz_ = rate;
        refresh_rate_explicit_ = true;
        if (state_ != ArenaState::ALL_OFF) {
          spi_.disarmRefreshTimer();
          spi_.armRefreshTimer(refresh_rate_hz_);
        }
      }
      current_source_->sendResponse(command_byte, 0, "");
      break;
    }

    case GET_REFRESH_RATE_CMD: {
      uint16_t hz = (uint16_t)refresh_rate_hz_;
      uint8_t payload[2] = { (uint8_t)(hz), (uint8_t)(hz >> 8) };
      current_source_->sendResponse(command_byte, 0, payload, sizeof(payload));
      break;
    }

    case SET_PANEL_DISPLAY_MODE_CMD: {
      // [02, 1B, mode]  mode uint8: 0=oneshot 1=persist 2=triggered 3=gated
      if (claimed_len != 2) {
        current_source_->sendResponse(command_byte, 1, "Expected [02 1B mode]");
        break;
      }
      uint8_t mode = buf[pos];
      if (mode > 3) {
        current_source_->sendResponse(command_byte, 1,
                                      "mode must be 0-3 (oneshot/persist/triggered/gated)");
        break;
      }
      panel_disp_mode_ = mode;
      patchDispMode();  // apply immediately to the currently-buffered frame
      current_source_->sendResponse(command_byte, 0, &mode, 1);
      DBG_PRINTF("[cmd] set-panel-display-mode mode=%u\n", (unsigned)mode);
      break;
    }

    case GET_PANEL_DISPLAY_MODE_CMD:
      current_source_->sendResponse(command_byte, 0, &panel_disp_mode_, 1);
      break;

    case GET_ETHERNET_IP_ADDRESS_CMD:
      current_source_->sendResponse(command_byte, 0, net_.ipAddress());
      break;

    case GET_CONTROLLER_INFO_CMD:
      handleGetControllerInfo();
      break;

    case SET_DIAG_OUTPUT_CMD:
      // [len=2, 0xC3, on]: mute (0) / unmute (non-zero) DEBUG_SERIAL
      // diagnostics on the shared USB-CDC pipe. Always accepted so the wire
      // protocol is uniform across builds; only has an effect when DEBUG_SERIAL
      // is compiled in. Missing arg defaults to on.
      g_dbg_on = (claimed_len >= 2) ? (buf[pos] != 0) : true;
      current_source_->sendResponse(command_byte, 0, g_dbg_on ? "diag on" : "diag off");
      break;

    case TRIAL_PARAMS_CMD:
      handleTrialParams(cmd);
      break;

    case SET_FRAME_POSITION_CMD:
      handleSetFramePosition(cmd);
      break;

    case GET_FRAME_POSITION_CMD: {
      // [01 72] → payload: cur_frame_index (uint16 LE), frame_count (uint16 LE).
      // Reflects the live index in any pattern mode (2/3/4) and 0/0 when no
      // pattern is open. Lets a host poll playback position and direction.
      uint16_t idx = cur_frame_index_;
      uint16_t n   = frame_count_;
      uint8_t payload[4] = {
          (uint8_t)(idx), (uint8_t)(idx >> 8),
          (uint8_t)(n),   (uint8_t)(n >> 8),
      };
      current_source_->sendResponse(command_byte, 0, payload, sizeof(payload));
      break;
    }

    case DISPLAY_PSRAM_INDEX_CMD:
      handleDisplayPsramIndex(cmd);
      break;

    case PSRAM_PLAY_CMD:
      handlePsramPlay(cmd);
      break;

    case GET_FRAMES_SENT_CMD: {
      uint32_t n = spi_.framesSent();
      uint8_t payload[4] = {
          (uint8_t)(n),
          (uint8_t)(n >> 8),
          (uint8_t)(n >> 16),
          (uint8_t)(n >> 24),
      };
      current_source_->sendResponse(command_byte, 0, payload, sizeof(payload));
      break;
    }

    case RESET_FRAMES_SENT_CMD:
      spi_.resetFramesSent();
      current_source_->sendResponse(command_byte, 0, "");
      break;

    case GET_FILE_COUNT_CMD: {
      uint16_t n = sd_.patternCount();
      uint8_t payload[2] = { (uint8_t)(n), (uint8_t)(n >> 8) };
      current_source_->sendResponse(command_byte, 0, payload, sizeof(payload));
      break;
    }

    case GET_PATTERN_FILENAME_CMD: {
      if (claimed_len != 3) {
        current_source_->sendResponse(command_byte, 1, "Expected [03 82 idx_lo idx_hi]");
        break;
      }
      uint16_t idx;
      memcpy(&idx, buf + pos, sizeof(idx));
      const char *name = sd_.patternName(idx);
      if (!name) {
        current_source_->sendResponse(command_byte, 1, "Index out of range");
        break;
      }
      // Response: 1-byte length + filename chars (no null terminator).
      uint8_t len = (uint8_t)strlen(name);
      uint8_t payload[1 + AC::constants::pattern_name_byte_count];
      payload[0] = len;
      memcpy(payload + 1, name, len);
      current_source_->sendResponse(command_byte, 0, payload, 1 + len);
      break;
    }

    case GET_PATTERN_INFO_CMD: {
      // [03 88 idx_lo idx_hi] — cheap pattern metadata for preview (no bulk
      // download, no ALL_OFF). Framed reply: 12-byte little-endian payload
      //   frame_count u16 · gs u8 · rows u8 · cols u8 · arena u8 · observer u8
      //   · file_size u32 · duty_cycle u8
      if (claimed_len != 3) {
        current_source_->sendResponse(command_byte, 1, "Expected [03 88 idx_lo idx_hi]");
        break;
      }
      uint16_t idx;
      memcpy(&idx, buf + pos, sizeof(idx));
      SdManager::PatternMeta m;
      uint8_t err = sd_.readPatternInfo(idx, m);
      if (err != CE_NONE) {
        current_source_->sendResponse(command_byte, err, "pattern info read failed");
        break;
      }
      uint8_t payload[12];
      payload[0] = (uint8_t)(m.frame_count);
      payload[1] = (uint8_t)(m.frame_count >> 8);
      payload[2] = m.gs_val;
      payload[3] = m.rows;
      payload[4] = m.cols;
      payload[5] = m.arena_id;
      payload[6] = m.observer_id;
      memcpy(payload + 7, &m.file_size, sizeof(m.file_size));  // u32 LE
      payload[11] = m.duty_cycle;
      current_source_->sendResponse(command_byte, 0, payload, sizeof(payload));
      break;
    }

    case SET_PATTERN_FILENAME_CMD: {
      // [len, 0x83, idx_lo, idx_hi, name_len, char0..charN]
      if (claimed_len < 4) {
        current_source_->sendResponse(command_byte, 1, "Too short");
        break;
      }
      uint16_t idx;
      memcpy(&idx, buf + pos, sizeof(idx));
      pos += 2;

      uint8_t name_len = buf[pos++];
      if (name_len == 0 || name_len >= AC::constants::pattern_name_byte_count ||
          claimed_len < (uint8_t)(4 + name_len)) {
        current_source_->sendResponse(command_byte, 1, "Bad name length");
        break;
      }
      if (idx > sd_.patternCount()) {
        current_source_->sendResponse(command_byte, 1, "Index out of range");
        break;
      }

      char new_name[AC::constants::pattern_name_byte_count];
      memcpy(new_name, buf + pos, name_len);
      new_name[name_len] = '\0';

      uint16_t new_idx = 0;
      uint8_t  err     = sd_.renamePattern(idx, new_name, &new_idx);
      if (err != CE_NONE) {
        current_source_->sendResponse(command_byte, err, "Rename failed");
        break;
      }
      uint8_t payload[2] = { (uint8_t)new_idx, (uint8_t)(new_idx >> 8) };
      current_source_->sendResponse(command_byte, 0, payload, sizeof(payload));
      DBG_PRINTF("[cmd] set-pattern-filename idx=%u -> '%s' new_idx=%u\n",
                 (unsigned)idx, new_name, (unsigned)new_idx);
      break;
    }

    case GET_PATTERN_FILE_CMD: {
      // [03 84 idx_lo idx_hi]  Response: [0x0A, 0, 0x84, size_b0..b7], then raw
      // bytes streamed asynchronously by serviceDownload() — see fixes 1-3 in
      // debug/issue-16-bulk-download-notes.md.
      if (state_ != ArenaState::ALL_OFF) {
        current_source_->sendResponse(command_byte, CE_DISPLAY_ACTIVE,
                                      "Stop display first");
        break;
      }
      if (dl_active_) {
        current_source_->sendResponse(command_byte, 1, "Download already in progress");
        break;
      }
      if (claimed_len != 3) {
        current_source_->sendResponse(command_byte, 1, "Expected [03 84 idx_lo idx_hi]");
        break;
      }
      uint16_t idx;
      memcpy(&idx, buf + pos, sizeof(idx));
      if (idx == 0 || idx > sd_.patternCount()) {
        current_source_->sendResponse(command_byte, 1, "Index out of range");
        break;
      }
      const char* name = sd_.patternName(idx);
      if (!name) {
        current_source_->sendResponse(command_byte, 1, "Pattern not found");
        break;
      }
      char path[sizeof(AC::constants::pattern_dir) + AC::constants::pattern_name_byte_count + 1];
      snprintf(path, sizeof(path), "%s/%s", AC::constants::pattern_dir, name);
      File f = SD.open(path, FILE_READ);
      if (!f) {
        current_source_->sendResponse(command_byte, 1, "File open failed");
        break;
      }
      uint32_t file_size = (uint32_t)f.size();
      // Send header: [0x0A, 0, 0x84, size_b0..b7] (8-byte uint64 LE; upper 4 bytes = 0)
      uint8_t size_buf[8] = {};
      memcpy(size_buf, &file_size, sizeof(file_size));
      current_source_->sendResponse(command_byte, 0, size_buf, sizeof(size_buf));
      // Hand off to serviceDownload() (driven from loop(), like serviceDisplay())
      // to stream the body one ~4 KB chunk per loop iteration instead of blocking
      // here for the whole file — the old version starved everything else in
      // loop() (TCP/USB command intake, display refresh, response flushing) for
      // as long as the transfer ran.
      dl_file_      = f;
      dl_source_    = current_source_;
      dl_idx_       = idx;
      dl_remaining_ = file_size;
      dl_deadline_  = millis() + kDownloadIdleTimeoutMs;
      dl_active_    = true;
      DBG_PRINTF("[cmd] get-pattern-file idx=%u name=%s size=%lu (async)\n",
                 (unsigned)idx, name, (unsigned long)file_size);
      break;
    }

    case DELETE_PATTERN_FILE_CMD: {
      // [03 86 idx_lo idx_hi]
      if (claimed_len != 3) {
        current_source_->sendResponse(command_byte, 1, "Expected [03 86 idx_lo idx_hi]");
        break;
      }
      uint16_t idx;
      memcpy(&idx, buf + pos, sizeof(idx));
      // Refuse deleting the one file an active download/upload has open
      // (PR #27 review point 8) rather than racing it. A download degrades
      // gracefully if its file vanishes anyway, but refusing here avoids
      // silently stranding that client instead of just telling it no.
      if (patternBusy(idx)) {
        current_source_->sendResponse(command_byte, 1, "Pattern is in use");
        break;
      }
      uint8_t err = sd_.deletePattern(idx);
      if (err != CE_NONE) {
        current_source_->sendResponse(command_byte, err, "Delete failed");
        break;
      }
      DBG_PRINTF("[cmd] delete-pattern-file idx=%u\n", (unsigned)idx);
      current_source_->sendResponse(command_byte, 0, "");
      break;
    }

    case DELETE_ALL_PATTERNS_CMD: {
      // Same reasoning as DELETE_PATTERN_FILE_CMD above, but "all" has no
      // single index to check against, so refuse outright while either
      // mechanism is active on any pattern (PR #27 review point 8).
      if (dl_active_ || ul_active_) {
        current_source_->sendResponse(command_byte, 1, "A transfer is in progress");
        break;
      }
      uint8_t err = sd_.deleteAllPatterns();
      if (err != CE_NONE) {
        current_source_->sendResponse(command_byte, err, "Delete-all failed");
        break;
      }
      DBG_PRINTF("[cmd] delete-all-patterns\n");
      current_source_->sendResponse(command_byte, 0, "");
      break;
    }

    case GET_SD_ARCHIVE_CMD:
      if (state_ != ArenaState::ALL_OFF) {
        current_source_->sendResponse(command_byte, CE_DISPLAY_ACTIVE,
                                      "Stop display first");
        break;
      }
      if (ar_active_) {
        current_source_->sendResponse(command_byte, 1, "Archive already in progress");
        break;
      }
      handleGetSdArchive();
      break;

    case GET_FIRMWARE_INFO_CMD:
      handleGetFirmwareInfo();
      break;

    case G6_PROGRAM_PANEL_CMD:
      if (state_ != ArenaState::ALL_OFF) {
        current_source_->sendResponse(command_byte, CE_DISPLAY_ACTIVE,
                                      "Stop display before flashing a panel");
        break;
      }
      handleProgramPanel(cmd);
      break;

    case G6_VERIFY_PANEL_CMD:
      if (state_ != ArenaState::ALL_OFF) {
        current_source_->sendResponse(command_byte, CE_DISPLAY_ACTIVE,
                                      "Stop display before verifying a panel");
        break;
      }
      handleVerifyPanel(cmd);
      break;

    case GET_DIAG_OUTPUT_CMD: {
      uint8_t val = g_dbg_on ? 1 : 0;
      current_source_->sendResponse(command_byte, 0, &val, 1);
      break;
    }

    case SET_SPI_CLOCK_CMD: {
      if (claimed_len != 3) {
        current_source_->sendResponse(command_byte, 1, "Expected [03 C5 mhz_lo mhz_hi]");
        break;
      }
      uint16_t mhz;
      memcpy(&mhz, buf + pos, sizeof(mhz));
      spi_.setSpiClockMhz(mhz);  // clamps 1..30 internally
      uint16_t applied = spi_.getSpiClockMhz();
      uint8_t payload[2] = { (uint8_t)(applied), (uint8_t)(applied >> 8) };
      current_source_->sendResponse(command_byte, 0, payload, sizeof(payload));
      DBG_PRINTF("[cmd] set-spi-clock req=%u applied=%u MHz\n",
                 (unsigned)mhz, (unsigned)applied);
      break;
    }

    case GET_SPI_CLOCK_CMD: {
      uint16_t mhz = spi_.getSpiClockMhz();
      uint8_t payload[2] = { (uint8_t)(mhz), (uint8_t)(mhz >> 8) };
      current_source_->sendResponse(command_byte, 0, payload, sizeof(payload));
      break;
    }

    case SET_PATTERN_ID_CMD: {
      // [03, 0x03, id_lo, id_hi] — load a 1-based pattern ID into SHOW_FRAME
      // (Mode 3) parked at frame 0, without starting auto-advance.
      // Pairs with SET_FRAME_POSITION (0x70) for clean G4-style frame addressing.
      if (claimed_len != 3) {
        current_source_->sendResponse(command_byte, 1, "Expected [03 03 id_lo id_hi]");
        break;
      }
      uint16_t pattern_id;
      memcpy(&pattern_id, buf + pos, sizeof(pattern_id));
      if (!enterPatternMode(ArenaState::SHOW_FRAME, pattern_id, 0, 0, 0)) {
        current_source_->sendResponse(command_byte, 1, "SET_PATTERN_ID: load failed");
        break;
      }
      uint8_t payload[2] = { (uint8_t)pattern_id, (uint8_t)(pattern_id >> 8) };
      current_source_->sendResponse(command_byte, 0, payload, sizeof(payload));
      DBG_PRINTF("[cmd] set-pattern-id id=%u frames=%u\n",
                 (unsigned)pattern_id_, (unsigned)frame_count_);
      break;
    }

    case SYSTEM_RESET_CMD:
      // Ack first so the host receives confirmation before the USB/TCP link drops.
      current_source_->sendResponse(command_byte, 0, "rebooting");
      net_.flushResponses();
      serial_.flushResponses();
      delay(10);
      SCB_AIRCR = 0x05FA0004;  // ARM AIRCR SYSRESETREQ
      break;

    case GET_DIGITAL_OUT_CMD: {
      uint8_t state1 = digitalRead(do1_data_pin) ? 1 : 0;
      uint8_t state2 = digitalRead(do2_data_pin) ? 1 : 0;
      uint8_t payload[2] = { state1, state2 };
      current_source_->sendResponse(command_byte, 0, payload, sizeof(payload));
      DBG_PRINTF("[cmd] get-digital-out do1=%u do2=%u\n",
                 (unsigned)state1, (unsigned)state2);
      break;
    }

    case SET_DIGITAL_OUT_CMD: {
      if (claimed_len != 3) {
        current_source_->sendResponse(command_byte, 1, "Expected [03 AA channel state]");
        break;
      }
      uint8_t channel = buf[pos++];
      uint8_t state   = buf[pos];
      if (channel != 1 && channel != 2) {
        current_source_->sendResponse(command_byte, 1, "channel must be 1 or 2");
        break;
      }
      // Role gate (#135): an `off` (unconfigured) port auto-promotes to
      // out_programmable — bare bench pokes keep working. A port explicitly
      // configured as in_trigger or out_debug_framescan REFUSES: flipping its
      // direction here would silently destroy the trigger route / scan gate
      // (the pre-role firmware did exactly that on channel 2).
      DioRole role = dio_role_[channel - 1];
      if (role == DioRole::kOff) {
        applyDioRole(channel, DioRole::kOutProgrammable);
        role = DioRole::kOutProgrammable;
      }
      if (role != DioRole::kOutProgrammable) {
        char msg[96];
        snprintf(msg, sizeof(msg),
                 "Digital IO %u role is %s - SET_DIO_ROLE (0xAC) it to out_programmable first",
                 (unsigned)channel, dioRoleName(role));
        current_source_->sendResponse(command_byte, 1, msg);
        break;
      }
      digitalWrite(channel == 1 ? do1_data_pin : do2_data_pin, state ? HIGH : LOW);
      current_source_->sendResponse(command_byte, 0, "");
      DBG_PRINTF("[cmd] set-digital-out ch=%u state=%u\n",
                 (unsigned)channel, (unsigned)state);
      break;
    }

    case SET_DIO_ROLE_CMD: {
      // [03 AC port role] — see DioRole in CommandProcessor.h. Explicit role
      // changes are the ONLY way into in_trigger / out_debug_framescan.
      if (claimed_len != 3) {
        current_source_->sendResponse(command_byte, 1, "Expected [03 AC port role]");
        break;
      }
      uint8_t port = buf[pos++];
      uint8_t role = buf[pos];
      if (port != 1 && port != 2) {
        current_source_->sendResponse(command_byte, 1, "port must be 1 or 2");
        break;
      }
      if (role > 3) {
        current_source_->sendResponse(
            command_byte, 1,
            "role must be 0=off 1=in_trigger 2=out_programmable 3=out_debug_framescan");
        break;
      }
      applyDioRole(port, (DioRole)role);
      current_source_->sendResponse(command_byte, 0, "");
      break;
    }

    case GET_DIO_ROLE_CMD: {
      // [01 AD] → [role1, level1, role2, level2]. `level` is a live read of
      // the data pin: the driven latch in output roles, the BNC level (through
      // the translator) in input roles — a free trigger-line readback.
      uint8_t payload[4] = {
          (uint8_t)dio_role_[0], (uint8_t)(digitalRead(do1_data_pin) ? 1 : 0),
          (uint8_t)dio_role_[1], (uint8_t)(digitalRead(do2_data_pin) ? 1 : 0),
      };
      current_source_->sendResponse(command_byte, 0, payload, sizeof(payload));
      DBG_PRINTF("[cmd] get-dio-role p1=%s/%u p2=%s/%u\n",
                 dioRoleName(dio_role_[0]), (unsigned)payload[1],
                 dioRoleName(dio_role_[1]), (unsigned)payload[3]);
      break;
    }

    case SET_AO_VOLTAGE_CMD: {
      if (claimed_len != 3) {
        current_source_->sendResponse(command_byte, 1, "Expected [03 A0 mv_lo mv_hi]");
        break;
      }
      uint16_t mv;
      memcpy(&mv, buf + pos, sizeof(mv));
      if (mv > 5000) {
        current_source_->sendResponse(command_byte, 1, "mv out of range (0-5000)");
        break;
      }
      if (ao_mode_ != 0) {
        current_source_->sendResponse(
            command_byte, 1, "AO is in frame_number mode - SET_AO_MODE (0xA3) 0 first");
        break;
      }
      if (!writeDacMv(mv)) {
        current_source_->sendResponse(command_byte, 1, "I2C write failed");
        break;
      }
      ao_lut_len_ = 0;  // stop any active LUT playback
      uint8_t payload[2] = { (uint8_t)(mv), (uint8_t)(mv >> 8) };
      current_source_->sendResponse(command_byte, 0, payload, sizeof(payload));
      DBG_PRINTF("[cmd] set-ao-voltage mv=%u\n", (unsigned)mv);
      break;
    }

    case SET_AO_MODE_CMD: {
      // [02 A3 mode] — 0 = programmable (0xA0/0xA2), 1 = frame_number: the
      // DAC tracks the SD-pattern frame index, 0 V = frame 0 .. 5 V = last
      // frame (normalized per pattern; updated in loadFrame, Modes 2/3/4).
      if (claimed_len != 2) {
        current_source_->sendResponse(command_byte, 1, "Expected [02 A3 mode]");
        break;
      }
      uint8_t new_mode = buf[pos];
      if (new_mode > 1) {
        current_source_->sendResponse(command_byte, 1,
                                      "mode must be 0 (programmable) or 1 (frame_number)");
        break;
      }
      ao_mode_ = new_mode;
      if (ao_mode_ == 1) {
        ao_lut_len_ = 0;  // frame_number owns the DAC — stop LUT playback
        // Reflect the current position immediately if a pattern is open.
        if (frame_count_ > 1) {
          writeDacMv((uint16_t)((uint32_t)cur_frame_index_ * 5000 / (frame_count_ - 1)));
        } else if (frame_count_ == 1) {
          writeDacMv(0);
        }
      }
      current_source_->sendResponse(command_byte, 0, "");
      DBG_PRINTF("[cmd] set-ao-mode %u\n", (unsigned)ao_mode_);
      break;
    }

    case GET_ANALOG_IN_CMD: {
      // [01 A4] → [ain1 int16 LE mV, ain2 int16 LE mV]. Both BNCs ("Analog In
      // 1 (±10V)" J28/D14, "Analog In 2 (±10V)" J29/D15) share the OPA2277
      // front-end mapping ±10 V → 0..3.3 V at the ADC, midscale = 0 V — the
      // same math Mode 4 uses. Front-end offset/scale calibration is TBD
      // (g6_03 § Mode 4); this is a bench diagnostic, not a precision read.
      int16_t mv[2];
      const uint8_t pins[2] = { mode4_ain_pin, ain2_pin };
      for (int i = 0; i < 2; ++i) {
        int raw = analogRead(pins[i]);
        float frac = (float)raw / (float)adc_full_scale_counts;  // 0..1
        float v = (frac - 0.5f) * 2.0f * mode4_ain_input_range_volts;
        mv[i] = (int16_t)lroundf(v * 1000.0f);
      }
      uint8_t payload[4] = {
          (uint8_t)((uint16_t)mv[0]), (uint8_t)((uint16_t)mv[0] >> 8),
          (uint8_t)((uint16_t)mv[1]), (uint8_t)((uint16_t)mv[1] >> 8),
      };
      current_source_->sendResponse(command_byte, 0, payload, sizeof(payload));
      DBG_PRINTF("[cmd] get-analog-in ain1=%d mV ain2=%d mV\n", (int)mv[0], (int)mv[1]);
      break;
    }

    case GET_AO_VOLTAGE_CMD: {
      // Read back directly from MCP4725 DAC register (3-byte read response):
      //   byte 0: [RDY, POR, x, x, PD1, PD0, x, x]
      //   byte 1: [D11..D4]
      //   byte 2: [D3..D0, x, x, x, x]
      uint8_t n = Wire.requestFrom((uint8_t)0x60, (uint8_t)3);
      if (n < 3) {
        current_source_->sendResponse(command_byte, 1, "I2C read failed");
        break;
      }
      uint8_t status = Wire.read();     // [RDY, POR, x, x, PD1, PD0, x, x]
      uint8_t hi = Wire.read();         // D11..D4
      uint8_t lo = Wire.read();         // D3..D0 in upper nibble
      uint16_t dac_code = ((uint16_t)hi << 4) | (lo >> 4);
      uint16_t mv = (uint16_t)((uint32_t)dac_code * 5000 / 4095);
      uint8_t pd = (status >> 2) & 0x03;  // PD1:PD0 from bits 3:2
      ao_mv_ = mv;
      uint8_t payload[2] = { (uint8_t)(mv), (uint8_t)(mv >> 8) };
      current_source_->sendResponse(command_byte, 0, payload, sizeof(payload));
      DBG_PRINTF("[cmd] get-ao-voltage hw_dac=%u hw_mv=%u status=0x%02X pd=%u%s\n",
                 (unsigned)dac_code, (unsigned)mv, (unsigned)status, (unsigned)pd,
                 pd ? " POWER-DOWN!" : "");
      break;
    }

    case SET_AO_LUT_CMD: {
      // [len, 0xA2, mode, step_hz_lo, step_hz_hi, count_lo, count_hi, mv[0..n-1]×2]
      //   mode     uint8:  0 = frame-locked, 1 = time-based
      //   step_hz  uint16 LE: step rate for mode 1 (ignored for mode 0; max 1000 Hz)
      //   count    uint16 LE: LUT entry count
      //   mv[i]    uint16 LE each: 0–5000 mV
      // claimed_len = 1(cmd) + 1(mode) + 2(step_hz) + 2(count) + 2*count = 6 + 2*count
      if (claimed_len < 6) {
        current_source_->sendResponse(command_byte, 1, "Too short (need mode+step_hz+count)");
        break;
      }
      if (ao_mode_ != 0) {
        current_source_->sendResponse(
            command_byte, 1, "AO is in frame_number mode - SET_AO_MODE (0xA3) 0 first");
        break;
      }
      uint8_t  lut_mode = buf[pos++];
      uint16_t step_hz;
      memcpy(&step_hz, buf + pos, sizeof(step_hz)); pos += 2;
      uint16_t count;
      memcpy(&count,   buf + pos, sizeof(count));   pos += 2;

      if (lut_mode > 1) {
        current_source_->sendResponse(command_byte, 1, "mode must be 0 (frame-locked) or 1 (time-based)");
        break;
      }
      if (count == 0 || count > kAoLutMaxLen) {
        current_source_->sendResponse(command_byte, 1, "count out of range (1-4096)");
        break;
      }
      // Verify the framing length matches the declared count.
      uint32_t needed = 6u + (uint32_t)count * 2;
      if (needed > 255u || (uint8_t)needed != claimed_len) {
        current_source_->sendResponse(command_byte, 1, "count/length mismatch");
        break;
      }
      bool mv_error = false;
      for (uint16_t i = 0; i < count; ++i) {
        uint16_t mv;
        memcpy(&mv, buf + pos, sizeof(mv)); pos += 2;
        if (mv > 5000) {
          current_source_->sendResponse(command_byte, 1, "mv out of range (0-5000)");
          mv_error = true;
          break;
        }
        ao_lut_[i] = mv;
      }
      if (mv_error) break;

      ao_lut_len_     = count;
      ao_lut_mode_    = lut_mode;
      ao_lut_step_hz_ = step_hz;
      ao_lut_idx_     = 0;
      ao_lut_last_us_ = micros();
      if (!applyAoLut(0)) {
        ao_lut_len_ = 0;  // undo — DAC not responding
        current_source_->sendResponse(command_byte, 1, "I2C error writing LUT[0]");
        break;
      }

      uint8_t payload[2] = { (uint8_t)count, (uint8_t)(count >> 8) };
      current_source_->sendResponse(command_byte, 0, payload, sizeof(payload));
      DBG_PRINTF("[cmd] set-ao-lut mode=%u step=%u count=%u lut[0]=%u mV\n",
                 (unsigned)lut_mode, (unsigned)step_hz,
                 (unsigned)count, (unsigned)ao_lut_[0]);
      break;
    }

    // G6-dropped command. Echo with an explanatory message so a legacy G4 host
    // gets a clear signal rather than silent failure.
    case SWITCH_GRAYSCALE_CMD:
      current_source_->sendResponse(command_byte, 1,
                        "SWITCH_GRAYSCALE dropped for G6; mode inferred from stream size");
      break;

    default:
      showError(CE_UNKNOWN_CMD);
      current_source_->sendResponse(command_byte, 1, "Unknown command");
      break;
  }
}

void CommandProcessor::handleStreamCommand(const ParsedCommand &cmd) {
#ifdef DEBUG_SERIAL
  uint32_t t0 = micros();
#endif
  const uint8_t *buf = cmd.data;
  uint32_t frame_byte_count = cmd.data_len - stream_header_byte_count;

  uint16_t block_size;
  if (frame_byte_count == stream_frame_byte_count_gs2) {
    block_size = G6::block_byte_count_gs2;
  } else if (frame_byte_count == stream_frame_byte_count_gs16) {
    block_size = G6::block_byte_count_gs16;
  } else {
    DBG_PRINTF("[stream] bad frame size %lu (expected gs2=%u gs16=%u)\n",
               (unsigned long)frame_byte_count,
               (unsigned)stream_frame_byte_count_gs2,
               (unsigned)stream_frame_byte_count_gs16);
    enterAllOff();
    current_source_->sendResponse(STREAM_FRAME_CMD, 1, "Bad stream-frame size");
    return;
  }

  // Copy the frame payload into our owned buffer so the network layer can
  // recycle its receive slot.
  memcpy(frame_buf_, buf + stream_header_byte_count, frame_byte_count);
  frame_byte_count_  = (uint16_t)frame_byte_count;
  block_byte_count_  = block_size;
  patchDispMode();

  // First-time entry into streaming, or grayscale-mode change → re-arm timer.
  bool need_rearm = (state_ != ArenaState::STREAMING_FRAME);

  if (!refresh_rate_explicit_) {
    uint32_t want = defaultRefreshFor(block_size);
    if (want != refresh_rate_hz_) {
      refresh_rate_hz_ = want;
      need_rearm = true;
    }
  }

  if (need_rearm) {
    enterStreamingFrame(block_size);
  }

  current_source_->sendResponse(STREAM_FRAME_CMD, 0, "");
  DBG_PRINTF("[stream] bytes=%lu block=%u refresh=%lu Hz dt=%lu us\n",
             (unsigned long)frame_byte_count, (unsigned)block_size,
             (unsigned long)refresh_rate_hz_, (unsigned long)(micros() - t0));
}

// ---------------------------------------------------------------------------
// get-controller-info (0xC2)
// ---------------------------------------------------------------------------

void CommandProcessor::handleGetControllerInfo() {
  // Response payload {version_byte, capability_bitmap, mac[6]} (g6_03 § 5).
  // The trailing 6 raw MAC bytes are the controller's physical-setup identity
  // (Teensy 4.1 burned-in unique ID, via QNEthernet — valid even when the
  // Ethernet link is down). Tolerant, additive extension: hosts that predate
  // it read only the first two bytes; webDisplayTools' decodeControllerInfo
  // reports mac:null when the payload is 2 bytes, so version stays 1.
  uint8_t payload[8] = {
      controller_info_version,
      controller_capability_bitmap,
  };
  net_.macBytes(payload + 2);
  current_source_->sendResponse(GET_CONTROLLER_INFO_CMD, 0, payload, sizeof(payload));
  DBG_PRINTF("[cmd] controller-info v=%u cap=0x%02X mac=%02X:%02X:%02X:%02X:%02X:%02X\n",
             (unsigned)payload[0], (unsigned)payload[1], payload[2], payload[3],
             payload[4], payload[5], payload[6], payload[7]);
}

// ---------------------------------------------------------------------------
// trial-params (0x08) — selects display mode 2/3/4 and the SD pattern.
//
// Payload layout (after the [len, 0x08] framing), parsed defensively:
//   param[0]   mode        (2 = open loop, 3 = show frame, 4 = closed loop)
//   param[1:2] pattern_id  uint16 LE (1-based index into /patterns/*.pat)
//   param[3:4] frame_rate  int16 LE  (Hz; Mode 2: positive = forward, negative = reverse)
//   param[5]   gain        int8       (Mode 4 velocity scaling, 10x fps/V)
//   param[6:7] init_pos    uint16 LE  (initial frame index, 0-based)
//   param[8:]  reserved    (legacy G4 fields; ignored)
//
// NOTE: the exact 12-byte G4 trial-params layout is host-canonical and still
// being reconciled for G6 (g6_03 § Modify). This layout covers every field
// the G6 doc names; confirm offsets with the host during bring-up.
// ---------------------------------------------------------------------------

void CommandProcessor::handleTrialParams(const ParsedCommand &cmd) {
  // PR #27 review point 8: trial-start opens a pattern via sd_.openPattern(),
  // an SD read, mid-transfer of an unrelated dl_/ul_/ar_ operation. Same
  // reasoning as the ALL_ON guard above: not corrupting (SD is SDIO, and
  // loop() is single-threaded so accesses are naturally serialized), but
  // confusing to have the display change while an SD transfer is still in
  // flight, so refuse it rather than let it interleave.
  if (dl_active_ || ul_active_ || ar_active_) {
    current_source_->sendResponse(TRIAL_PARAMS_CMD, 1, "SD transfer in progress");
    return;
  }

  const uint8_t *p = cmd.data + 2;          // first param byte
  uint8_t param_len = cmd.data[0] - 1;       // claimed_len minus the cmd byte
  if (param_len < 8) {
    showError(CE_BAD_PAYLOAD_LEN);
    current_source_->sendResponse(TRIAL_PARAMS_CMD, 1, "TRIAL_PARAMS too short");
    return;
  }

  uint8_t  mode       = p[0];
  uint16_t pattern_id = (uint16_t)p[1] | ((uint16_t)p[2] << 8);
  int16_t  frame_rate = (int16_t)((uint16_t)p[3] | ((uint16_t)p[4] << 8));
  int8_t   gain       = (int8_t)p[5];
  uint16_t init_pos   = (uint16_t)p[6] | ((uint16_t)p[7] << 8);

  ArenaState target;
  switch (mode) {
    case display_mode_open_loop:   target = ArenaState::OPEN_LOOP;   break;
    case display_mode_show_frame:  target = ArenaState::SHOW_FRAME;  break;
    case display_mode_closed_loop: target = ArenaState::CLOSED_LOOP; break;
    default:
      showError(CE_BAD_PARAM);
      current_source_->sendResponse(TRIAL_PARAMS_CMD, 1,
                        "TRIAL_PARAMS: mode must be 2/3/4");
      return;
  }

  if (enterPatternMode(target, pattern_id, frame_rate, gain, init_pos)) {
    current_source_->sendResponse(TRIAL_PARAMS_CMD, 0, "");
  } else {
    // enterPatternMode already raised the error display + parked in ALL_OFF.
    current_source_->sendResponse(TRIAL_PARAMS_CMD, 1, "TRIAL_PARAMS: load failed");
  }
}

// ---------------------------------------------------------------------------
// set-frame-position (0x70) — Mode 3: show a specific frame of the open pattern.
// ---------------------------------------------------------------------------

void CommandProcessor::handleSetFramePosition(const ParsedCommand &cmd) {
  uint8_t param_len = cmd.data[0] - 1;
  if (param_len < 2) {
    showError(CE_BAD_PAYLOAD_LEN);
    current_source_->sendResponse(SET_FRAME_POSITION_CMD, 1,
                      "SET_FRAME_POSITION too short");
    return;
  }
  uint16_t index = (uint16_t)cmd.data[2] | ((uint16_t)cmd.data[3] << 8);

  if (!sd_.patternOpen()) {
    showError(CE_BAD_PARAM);
    current_source_->sendResponse(SET_FRAME_POSITION_CMD, 1,
                      "SET_FRAME_POSITION: no pattern selected (send trial-params first)");
    return;
  }
  if (index >= frame_count_) {
    showError(CE_BAD_PARAM);
    current_source_->sendResponse(SET_FRAME_POSITION_CMD, 1,
                      "SET_FRAME_POSITION: index out of range");
    return;
  }

  spi_.disarmRefreshTimer();
  cur_frame_index_ = index;
  if (!loadFrame(cur_frame_index_)) {
    current_source_->sendResponse(SET_FRAME_POSITION_CMD, 1,
                      "SET_FRAME_POSITION: frame read failed");
    return;
  }
  state_ = ArenaState::SHOW_FRAME;
  if (!refresh_rate_explicit_) refresh_rate_hz_ = defaultRefreshFor(block_byte_count_);
  spi_.armRefreshTimer(refresh_rate_hz_);
  current_source_->sendResponse(SET_FRAME_POSITION_CMD, 0, "");
}

// ---------------------------------------------------------------------------
// V2 PSRAM display (0x3A single index / 0x3B auto-advance play) — LAB-41/42.
//
// The panel holds the frames in its own PSRAM (loaded locally for the demo).
// We synthesize a frame of identical V2 "display PSRAM index N" blocks (one per
// panel, all the same index → whole arena shows frame N) and let the refresh
// timer retransmit it. 0x3B advances the index at frame_rate_hz_, exactly like
// the Mode-2 open-loop player but with no SD access.
// ---------------------------------------------------------------------------

void CommandProcessor::buildPsramFrame(uint16_t index) {
  memset(frame_buf_, 0, sizeof(frame_buf_));
  frame_buf_[0] = 'F';
  frame_buf_[1] = 'R';
  frame_buf_[2] = (uint8_t)(index & 0xFF);
  frame_buf_[3] = (uint8_t)(index >> 8);

  uint16_t blk = G6::block_byte_count_psram;
  for (uint8_t p = 0; p < panel_count_per_frame; ++p) {
    uint8_t *block = frame_buf_ + stream_frame_prefix_byte_count
                     + (uint32_t)p * G6::block_byte_count_psram;
    blk = G6::build_psram_index_block(block, index, psram_cmd_id_, 0);
  }
  block_byte_count_ = blk;
  frame_byte_count_ = (uint16_t)(stream_frame_prefix_byte_count
                                 + panel_count_per_frame * block_byte_count_);
}

void CommandProcessor::handleDisplayPsramIndex(const ParsedCommand &cmd) {
  uint8_t param_len = cmd.data[0] - 1;
  if (param_len < 2) {
    showError(CE_BAD_PAYLOAD_LEN);
    current_source_->sendResponse(DISPLAY_PSRAM_INDEX_CMD, 1, "DISPLAY_PSRAM too short");
    return;
  }
  uint16_t index = (uint16_t)cmd.data[2] | ((uint16_t)cmd.data[3] << 8);

  spi_.disarmRefreshTimer();
  psram_cmd_id_      = G6::disp_opcode_with_mode(G6::cmd_disp_psram_oneshot,
                                                 panel_disp_mode_);
  psram_start_index_ = index;
  psram_play_count_  = 1;          // static single index
  psram_play_offset_ = 0;
  buildPsramFrame(index);
  state_ = ArenaState::PSRAM_PLAY;
  if (!refresh_rate_explicit_) refresh_rate_hz_ = defaultRefreshFor(block_byte_count_);
  spi_.armRefreshTimer(refresh_rate_hz_);
  current_source_->sendResponse(DISPLAY_PSRAM_INDEX_CMD, 0, "");
  DBG_PRINTF("[cmd] DISPLAY_PSRAM index=%u\n", (unsigned)index);
}

void CommandProcessor::handlePsramPlay(const ParsedCommand &cmd) {
  uint8_t param_len = cmd.data[0] - 1;
  if (param_len < 6) {
    showError(CE_BAD_PAYLOAD_LEN);
    current_source_->sendResponse(PSRAM_PLAY_CMD, 1, "PSRAM_PLAY too short");
    return;
  }
  const uint8_t *p = cmd.data + 2;
  uint16_t start = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
  uint16_t count = (uint16_t)p[2] | ((uint16_t)p[3] << 8);
  uint16_t fps   = (uint16_t)p[4] | ((uint16_t)p[5] << 8);
  if (count == 0) count = 1;

  spi_.disarmRefreshTimer();
  psram_cmd_id_      = G6::disp_opcode_with_mode(G6::cmd_disp_psram_oneshot,
                                                 panel_disp_mode_);
  psram_start_index_ = start;
  psram_play_count_  = count;
  psram_play_offset_ = 0;
  frame_rate_hz_     = fps;          // animation advance rate (separate from refresh)
  buildPsramFrame(start);
  state_ = ArenaState::PSRAM_PLAY;
  last_advance_us_ = micros();
  // Retransmit (refresh) faster than the animation advances so each index is
  // sent several times; the panel renders Persistent so a missed retransmit
  // just holds the last frame.
  uint32_t r = refresh_rate_explicit_ ? refresh_rate_hz_
                                      : defaultRefreshFor(block_byte_count_);
  spi_.armRefreshTimer(r);
  current_source_->sendResponse(PSRAM_PLAY_CMD, 0, "");
  DBG_PRINTF("[cmd] PSRAM_PLAY start=%u count=%u fps=%u\n",
             (unsigned)start, (unsigned)count, (unsigned)fps);
}

// ---------------------------------------------------------------------------
// serviceDisconnects: called every loop() iteration, before the per-
// transfer service functions. PR #27 review point 5: a TCP client can
// disconnect out from under an active 0x84/0x85/0x8A transfer, and
// NetworkManager::serviceTcp() will happily accept a brand-new client on
// the very same source afterward. Without this check, dl_/ul_/ar_active_
// stay set with a stale dl_/ul_/ar_source_ pointer, so the NEW client's
// bytes (or, for downloads/archives, its next command) get fed into or
// blocked by a transfer it never started. isConnected() defaults to true
// on MessageSource (serial has no equivalent failure mode, matching the
// review's USB exclusion), so this is a no-op for serial_-owned transfers.
// ---------------------------------------------------------------------------

void CommandProcessor::serviceDisconnects() {
  if (dl_active_ && !dl_source_->isConnected()) endDownload();
  if (ul_active_ && !ul_source_->isConnected()) abortUpload();
  if (ar_active_ && !ar_source_->isConnected()) abortArchive();
}

// ---------------------------------------------------------------------------
// Display service — called every loop iteration
// ---------------------------------------------------------------------------

void CommandProcessor::transmitOnRefresh() {
  if (spi_.refreshFlag) {
    spi_.refreshFlag = false;
    spi_.transferFrame(frame_buf_, block_byte_count_);
  }
}

void CommandProcessor::serviceDisplay() {
  // Service time-based AO LUT regardless of display state (independent of frames).
  if (ao_lut_len_ > 0 && ao_lut_mode_ == 1 && ao_lut_step_hz_ > 0) {
    uint32_t now = micros();
    uint32_t period_us = 1000000UL / ao_lut_step_hz_;
    if (now - ao_lut_last_us_ >= period_us) {
      ao_lut_last_us_ += period_us;
      ao_lut_idx_ = (uint16_t)((ao_lut_idx_ + 1) % ao_lut_len_);
      applyAoLut(ao_lut_idx_);
    }
  }

  switch (state_) {
    case ArenaState::ALL_OFF:
      break;

    case ArenaState::ALL_ON:
    case ArenaState::STREAMING_FRAME:
    case ArenaState::SHOW_FRAME:
      transmitOnRefresh();
      break;

    case ArenaState::OPEN_LOOP:
      serviceOpenLoop();
      transmitOnRefresh();
      break;

    case ArenaState::CLOSED_LOOP:
      serviceClosedLoop();
      transmitOnRefresh();
      break;

    case ArenaState::PSRAM_PLAY:
      servicePsramPlay();
      transmitOnRefresh();
      break;

    case ArenaState::ERROR_DISPLAY:
      transmitOnRefresh();
      if ((int32_t)(millis() - error_until_ms_) >= 0) {
        enterAllOff();  // glyph held long enough → revert to a safe dark state
      }
      break;
  }
}

// ---------------------------------------------------------------------------
// serviceDownload — one GET_PATTERN_FILE (0x84) chunk per loop() call.
//
// Issue #16 fixes 1-3: the old handler streamed the whole file inline inside
// processCommand(), blocking net_.serviceTcp(), serial_.serviceUsb(),
// serviceDisplay(), and both flushResponses() for the entire transfer (fix 2).
// This drains one ~4 KB chunk per call instead, so those all keep running.
//
// dl_deadline_ resets on every successfully drained chunk rather than being
// set once for the whole transfer (fix 3), so a slow-but-progressing host
// doesn't trip the idle ceiling just because the transfer runs long. A short
// return from sendRaw() (fix 1's stall bound) means the host has stopped
// draining — abort immediately rather than waiting out the idle deadline.
// ---------------------------------------------------------------------------
void CommandProcessor::serviceDownload() {
  if (!dl_active_) return;

  if ((int32_t)(millis() - dl_deadline_) >= 0) {
    DBG_PRINTF("[cmd] get-pattern-file idle TIMEOUT, %lu bytes remaining\n",
               (unsigned long)dl_remaining_);
    endDownload();
    return;
  }

  if (dl_remaining_ == 0) {
    endDownload();
    return;
  }

  static uint8_t chunk[4096];
  size_t to_read = (dl_remaining_ < sizeof(chunk)) ? (size_t)dl_remaining_ : sizeof(chunk);
  // File::read() returns int: -1 on I/O error, 0 on EOF, else bytes read.
  // Check the sign BEFORE narrowing to size_t — casting -1 straight to
  // size_t (as an earlier version of this code did) yields SIZE_MAX, which
  // then gets handed to sendRaw() as the length of a 4096-byte buffer:
  // an out-of-bounds read that crashes/reboots the controller instead of
  // cleanly aborting the download.
  int rd = dl_file_.read(chunk, to_read);
  if (rd <= 0) {  // unexpected EOF/read error before dl_remaining_ hit 0
    DBG_PRINTF("[cmd] get-pattern-file EOF/read-error rd=%d, %lu bytes remaining\n",
               rd, (unsigned long)dl_remaining_);
    endDownload();
    return;
  }
  size_t n = (size_t)rd;

  size_t sent = dl_source_->sendRaw(chunk, n);
  dl_remaining_ -= sent;
  if (sent < n) {
    DBG_PRINTF("[cmd] get-pattern-file STALL sent=%lu/%lu, %lu bytes remaining\n",
               (unsigned long)sent, (unsigned long)n, (unsigned long)dl_remaining_);
    endDownload();
    return;
  }

  dl_deadline_ = millis() + kDownloadIdleTimeoutMs;
  if (dl_remaining_ == 0) {
    endDownload();
  }
}

// Common teardown for serviceDownload(), on completion, timeout, read error,
// stall, or (PR #27 review point 5) the owning source disconnecting. No
// response is sent either way: the header response already promised the
// file's total size; the client infers success or failure from how many
// bytes it actually received.
void CommandProcessor::endDownload() {
  dl_file_.close();
  dl_active_ = false;
}

// ---------------------------------------------------------------------------
// serviceUpload — one SET_PATTERN_FILE (0x85) chunk per loop() call.
//
// Mirrors serviceDownload() above (issue #16): reads+writes one ~4 KB chunk
// per call instead of blocking inside processCommand() for the whole
// transfer, and ul_deadline_ resets on every chunk actually read off the
// wire rather than staying fixed from the start, so a slow-but-progressing
// host doesn't trip the idle ceiling just because the transfer runs long.
//
// Unlike the download side, a failure here (idle timeout or SD write error)
// doesn't mean the transfer can just stop: the host already committed to
// sending total_len bytes, and whatever it hasn't sent yet will still land
// on the wire and get misparsed as a bogus command unless something reads
// and discards it first. ul_phase_'s kDraining value is that second phase:
// same chunked service loop, just discarding instead of writing, with its
// own bound (kUploadDrainTimeoutMs) since by then the transfer is already a
// lost cause and this is purely resync, not recovery. See the ul_* fields'
// comment in CommandProcessor.h (PR #27 review point 3) for why this is an
// explicit phase transition rather than a bool checked at two sites: an
// earlier version of this function used the latter and, on the FIRST
// write-phase timeout, fell straight into the same "give up" response this
// function now only reaches on a SECOND (drain-phase) timeout, never
// reading the bytes still in flight, so they landed on the wire and got
// misparsed as bogus commands once the source was freed up.
// ---------------------------------------------------------------------------
void CommandProcessor::serviceUpload() {
  if (!ul_active_) return;

  if (ul_phase_ == UploadPhase::kWriting) {
    if ((int32_t)(millis() - ul_deadline_) >= 0) {
      // First (write-phase) timeout: transition into draining instead of
      // finalizing here. ul_drain_deadline_ is an absolute cap from now,
      // not idle-reset; see the field comment in CommandProcessor.h.
      ul_file_.close();
      SD.remove(ul_path_);
      ul_fail_status_ = 1;
      strncpy(ul_fail_msg_, "Upload timeout", sizeof(ul_fail_msg_) - 1);
      ul_phase_ = UploadPhase::kDraining;
      ul_drain_deadline_ = millis() + kUploadDrainTimeoutMs;
      return;
    }
  } else {  // kDraining
    if ((int32_t)(millis() - ul_drain_deadline_) >= 0) {
      // Draining also timed out — give up on resync too; the next framed
      // byte the parser sees may be garbage, but continuing to wait
      // indefinitely isn't better.
      ul_source_->sendResponse(SET_PATTERN_FILE_CMD, ul_fail_status_, ul_fail_msg_);
      ul_active_ = false;
      ul_source_->commandConsumed();
      return;
    }
  }

  static uint8_t chunk[4096];
  size_t want = (ul_remaining_ < sizeof(chunk)) ? (size_t)ul_remaining_ : sizeof(chunk);
  size_t got = ul_source_->readBulkBytes(chunk, want);
  if (got == 0) return;  // nothing new this tick; the checks above cover staleness

  if (ul_phase_ == UploadPhase::kWriting) {
    size_t written = ul_file_.write(chunk, got);
    if (written != got) {
      ul_file_.close();
      SD.remove(ul_path_);
      ul_fail_status_ = 1;
      strncpy(ul_fail_msg_, "SD write error (card full?)", sizeof(ul_fail_msg_) - 1);
      ul_phase_ = UploadPhase::kDraining;
      ul_drain_deadline_ = millis() + kUploadDrainTimeoutMs;
    }
  }
  ul_remaining_ -= (uint32_t)got;
  // kDraining's deadline is an absolute cap set once above, not reset here.
  if (ul_phase_ == UploadPhase::kWriting) {
    ul_deadline_ = millis() + kUploadIdleTimeoutMs;
  }

  if (ul_remaining_ == 0) {
    switch (ul_phase_) {
      case UploadPhase::kDraining:
        ul_source_->sendResponse(SET_PATTERN_FILE_CMD, ul_fail_status_, ul_fail_msg_);
        break;
      case UploadPhase::kWriting: {
        uint32_t elapsed_ms = millis() - ul_start_ms_;
        uint32_t kbps = elapsed_ms > 0 ? (uint32_t)(ul_total_ / elapsed_ms) : 0;
        DBG_PRINTF("[cmd] set-pattern-file idx=%u wrote %lu bytes in %lu ms (%lu kB/s) to %s\n",
                   (unsigned)ul_idx_, (unsigned long)ul_total_,
                   (unsigned long)elapsed_ms, (unsigned long)kbps, ul_path_);
        ul_file_.close();
        ul_source_->sendResponse(SET_PATTERN_FILE_CMD, 0, "");
        break;
      }
    }
    ul_active_ = false;
    ul_source_->commandConsumed();
  }
}

// Teardown for an upload whose owning source disconnected (PR #27 review
// point 5), distinct from serviceUpload()'s own timeout/error paths: those
// still expect the host to resume (kWriting -> kDraining) or have already
// finished, so they respond and consume normally. A disconnected source has
// nothing left to respond to and nothing left to resync, so this always
// deletes the partial file and fully deactivates in one step; there's no
// draining phase to enter since no more bytes will ever arrive on this
// connection.
void CommandProcessor::abortUpload() {
  ul_file_.close();
  SD.remove(ul_path_);
  ul_active_ = false;
  ul_source_->commandConsumed();
}

void CommandProcessor::serviceOpenLoop() {
  if (frame_rate_hz_ == 0 || frame_count_ <= 1) return;  // static frame
  uint32_t rate_hz   = (frame_rate_hz_ < 0) ? (uint32_t)(-frame_rate_hz_)
                                             : (uint32_t)frame_rate_hz_;
  uint32_t period_us = microseconds_per_second / rate_hz;
  uint32_t now = micros();
  if ((now - last_advance_us_) < period_us) return;

  // Advance by however many whole periods have elapsed (catch up if the loop
  // was busy), then load the new frame once.
  uint16_t steps = 0;
  while ((now - last_advance_us_) >= period_us) {
    last_advance_us_ += period_us;
    if (++steps >= frame_count_) { steps = frame_count_; break; }  // clamp
  }
  int32_t idx = (int32_t)cur_frame_index_
                + (frame_rate_hz_ > 0 ? (int32_t)steps : -(int32_t)steps);
  idx %= (int32_t)frame_count_;
  if (idx < 0) idx += (int32_t)frame_count_;
  cur_frame_index_ = (uint16_t)idx;
  loadFrame(cur_frame_index_);
}

void CommandProcessor::servicePsramPlay() {
  if (frame_rate_hz_ <= 0 || psram_play_count_ <= 1) return;  // static index
  uint32_t period_us = microseconds_per_second / (uint32_t)frame_rate_hz_;
  uint32_t now = micros();
  if ((now - last_advance_us_) < period_us) return;

  uint16_t steps = 0;
  while ((now - last_advance_us_) >= period_us) {
    last_advance_us_ += period_us;
    if (++steps >= psram_play_count_) { steps = psram_play_count_; break; }
  }
  psram_play_offset_ =
      (uint16_t)((psram_play_offset_ + steps) % psram_play_count_);
  buildPsramFrame((uint16_t)(psram_start_index_ + psram_play_offset_));
}

void CommandProcessor::serviceClosedLoop() {
  if (frame_count_ == 0) return;
  uint32_t sample_period_us = microseconds_per_second / mode4_sample_rate_hz;
  uint32_t now = micros();
  if ((now - last_sample_us_) < sample_period_us) return;

  uint32_t dt_us = now - last_sample_us_;
  last_sample_us_ = now;

  // Reconstruct the bipolar BNC input voltage from the ADC reading. The
  // OPA2277 front-end maps +/-10 V at J28 to 0..3.3 V at the ADC, midscale =
  // 0 V (g6_06). Front-end offset/scale is hardware calibration — flagged as
  // lowest priority / TBD in g6_03 § Mode 4.
  int raw = analogRead(mode4_ain_pin);
  float adc_frac = (float)raw / (float)adc_full_scale_counts;       // 0..1
  float v_in = (adc_frac - 0.5f) * 2.0f * mode4_ain_input_range_volts;
  // fps = v_in * (gain/10) fps/V (e.g. gain=-20 -> -2.0 fps/V; g6_03 § Mode 4).
  float fps = v_in * ((float)gain_ / 10.0f);
  frame_accum_ += fps * ((float)dt_us / (float)microseconds_per_second);

  bool changed = false;
  while (frame_accum_ >= 1.0f) {
    frame_accum_ -= 1.0f;
    cur_frame_index_ = (uint16_t)((cur_frame_index_ + 1) % frame_count_);
    changed = true;
  }
  while (frame_accum_ <= -1.0f) {
    frame_accum_ += 1.0f;
    cur_frame_index_ =
        (uint16_t)((cur_frame_index_ + frame_count_ - 1) % frame_count_);
    changed = true;
  }
  if (changed) loadFrame(cur_frame_index_);
}

bool CommandProcessor::loadFrame(uint16_t frame_index) {
  uint8_t err = sd_.readFrame(frame_index, frame_buf_, sizeof(frame_buf_));
  if (err != CE_NONE) {
    DBG_PRINTF("[cmd] loadFrame %u failed err=%u\n",
               (unsigned)frame_index, (unsigned)err);
    showError(err);
    return false;
  }
  frame_byte_count_ = (uint16_t)(stream_frame_prefix_byte_count
                                 + (uint32_t)sd_.info().num_panels * block_byte_count_);
  patchDispMode();
  if (ao_mode_ == 1) {
    // frame_number AO (#135): DAC tracks the frame position, 0 V = frame 0 ..
    // 5 V = last frame. loadFrame only runs when the index CHANGES, so the
    // ~100 µs blocking I2C write costs nothing while a frame is held.
    writeDacMv(frame_count_ > 1
                   ? (uint16_t)((uint32_t)frame_index * 5000 / (frame_count_ - 1))
                   : (uint16_t)0);
  } else if (ao_lut_len_ > 0 && ao_lut_mode_ == 0) {
    ao_lut_idx_ = (uint16_t)(frame_index % ao_lut_len_);
    applyAoLut(ao_lut_idx_);
  }
  return true;
}

// ---------------------------------------------------------------------------
// State transitions
// ---------------------------------------------------------------------------

void CommandProcessor::enterAllOff() {
  spi_.disarmRefreshTimer();
  // Panels run in Persistent mode and HOLD their last received frame, so simply
  // stopping transmission leaves them lit. Push an all-dark frame so they
  // actually blank, then go quiet. Sent a few times for reliability on the
  // shared (wired-OR CIPO) bus; one dropped frame would otherwise stay lit.
  fillFrameBufferDark();
  for (uint8_t i = 0; i < 3; ++i) {
    spi_.transferFrame(frame_buf_, block_byte_count_);
  }
  state_ = ArenaState::ALL_OFF;
  frame_byte_count_ = 0;
}

void CommandProcessor::enterAllOn() {
  spi_.disarmRefreshTimer();
  block_byte_count_ = G6::block_byte_count_gs16;
  if (!refresh_rate_explicit_) {
    refresh_rate_hz_ = refresh_rate_gs16_default;
  }
  fillFrameBufferAllOn(block_byte_count_);
  state_ = ArenaState::ALL_ON;
  spi_.armRefreshTimer(refresh_rate_hz_);
#ifdef DEBUG_SERIAL
  DBG_PRINTF("[cmd] enterAllOn block=%u refresh=%lu Hz frame_bytes=%u\n",
             (unsigned)block_byte_count_,
             (unsigned long)refresh_rate_hz_,
             (unsigned)frame_byte_count_);
  const uint8_t *p = frame_buf_ + AC::constants::stream_frame_prefix_byte_count;
  DBG_PRINTF("[cmd] first_block: %02X %02X %02X %02X %02X %02X %02X %02X\n",
             p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);
#endif
}

void CommandProcessor::enterStreamingFrame(uint16_t block_byte_count) {
  spi_.disarmRefreshTimer();
  block_byte_count_ = block_byte_count;
  state_ = ArenaState::STREAMING_FRAME;
  spi_.armRefreshTimer(refresh_rate_hz_);
}

bool CommandProcessor::enterPatternMode(ArenaState mode, uint16_t pattern_id,
                                        int16_t frame_rate_hz, int8_t gain,
                                        uint16_t init_frame) {
  spi_.disarmRefreshTimer();

  uint8_t err = sd_.openPattern(pattern_id);
  if (err != CE_NONE) {
    DBG_PRINTF("[cmd] openPattern %u failed err=%u\n",
               (unsigned)pattern_id, (unsigned)err);
    showError(err);
    return false;
  }

  pattern_id_      = pattern_id;
  frame_count_     = sd_.info().frame_count;
  block_byte_count_ = sd_.info().block_size;
  frame_rate_hz_   = frame_rate_hz;
  gain_            = gain;
  cur_frame_index_ = (frame_count_ > 0) ? (uint16_t)(init_frame % frame_count_) : 0;
  frame_accum_     = 0.0f;

  if (!loadFrame(cur_frame_index_)) return false;  // showError already raised

  if (!refresh_rate_explicit_) refresh_rate_hz_ = defaultRefreshFor(block_byte_count_);

  uint32_t now = micros();
  last_advance_us_ = now;
  last_sample_us_  = now;
  state_ = mode;
  spi_.armRefreshTimer(refresh_rate_hz_);
  DBG_PRINTF("[cmd] enterPatternMode state=%u id=%u frames=%u rate=%u gain=%d\n",
             (unsigned)mode, (unsigned)pattern_id_, (unsigned)frame_count_,
             (unsigned)frame_rate_hz_, (int)gain_);
  return true;
}

void CommandProcessor::showError(uint8_t code) {
  spi_.disarmRefreshTimer();
  block_byte_count_ = G6::block_byte_count_gs16;
  frame_byte_count_ = G6Error::buildErrorFrame(frame_buf_, code, panel_count_per_frame);
  state_ = ArenaState::ERROR_DISPLAY;
  error_until_ms_ = millis() + error_display_hold_ms;
  uint32_t r = refresh_rate_explicit_ ? refresh_rate_hz_
                                      : defaultRefreshFor(block_byte_count_);
  spi_.armRefreshTimer(r);
  DBG_PRINTF("[cmd] showError code=%u hold=%lu ms\n",
             (unsigned)code, (unsigned long)error_display_hold_ms);
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

uint8_t CommandProcessor::dispOpcodeFor(bool gs16) const {
  uint8_t base = gs16 ? G6::cmd_disp_16lvl_oneshot : G6::cmd_disp_2lvl_oneshot;
  return G6::disp_opcode_with_mode(base, panel_disp_mode_);
}

void CommandProcessor::patchDispMode() {
  if (block_byte_count_ == 0 || frame_byte_count_ <= stream_frame_prefix_byte_count) return;
  uint8_t *base = frame_buf_ + stream_frame_prefix_byte_count;
  uint16_t count = (uint16_t)((frame_byte_count_ - stream_frame_prefix_byte_count)
                               / block_byte_count_);
  for (uint16_t p = 0; p < count; ++p) {
    uint8_t *block = base + (uint32_t)p * block_byte_count_;
    // Preserve the opcode family (high 6 bits — GS2 0x1x / GS16 0x3x / PSRAM
    // 0x5x / 0x6x); rewrite only the mode in the low 2 bits. Works for any
    // display block without assuming the grayscale level from block size.
    block[1] = G6::disp_opcode_with_mode((uint8_t)(block[1] & 0xFC), panel_disp_mode_);
    G6::stamp_header_parity(block, block_byte_count_);
  }
}

// Write a 0-5000 mV level to the MCP4725 (fast-mode 2-byte write). Shared by
// SET_AO_VOLTAGE, LUT playback, and the frame_number AO mode. Blocking I2C at
// 400 kHz ≈ 100 µs — called only from the main loop, never an ISR.
bool CommandProcessor::writeDacMv(uint16_t mv) {
  uint16_t dac_code = (uint16_t)((uint32_t)mv * 4095 / 5000);
  Wire.beginTransmission(0x60);
  Wire.write((uint8_t)((dac_code >> 8) & 0x0F));
  Wire.write((uint8_t)(dac_code & 0xFF));
  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    DBG_PRINTF("[dac] I2C error %u writing mv=%u\n", (unsigned)err, (unsigned)mv);
    return false;
  }
  ao_mv_ = mv;
  return true;
}

bool CommandProcessor::applyAoLut(uint16_t idx) {
  if (ao_lut_len_ == 0) return true;
  return writeDacMv(ao_lut_[idx % ao_lut_len_]);
}

uint32_t CommandProcessor::defaultRefreshFor(uint16_t block_byte_count) const {
  return (block_byte_count == G6::block_byte_count_gs2)
             ? refresh_rate_gs2_default
             : refresh_rate_gs16_default;
}

void CommandProcessor::fillFrameBufferAllOn(uint16_t block_byte_count) {
  // Synthesize an all-max-pixel frame using the current panel display mode.
  // Block layout: [header][cmd][200 pixel bytes = 0xFF][duty_cycle=0xFF].
  // Parity is recomputed per block.
  memset(frame_buf_, 0, sizeof(frame_buf_));

  // Frame prefix: "FR" + frame index 0 — informational only, matches the
  // stream payload layout the SPI dispatcher expects.
  frame_buf_[0] = 'F';
  frame_buf_[1] = 'R';
  frame_buf_[2] = 0;
  frame_buf_[3] = 0;

  uint8_t cmd = dispOpcodeFor(block_byte_count == G6::block_byte_count_gs16);

  for (uint8_t p = 0; p < panel_count_per_frame; ++p) {
    uint8_t *block = frame_buf_ + stream_frame_prefix_byte_count
                     + (uint32_t)p * block_byte_count;
    block[0] = G6::header_version_v1;  // parity stamped below
    block[1] = cmd;
    // Pixel data + duty_cycle = 0xFF.
    for (uint16_t i = 2; i < block_byte_count; ++i) {
      block[i] = 0xFF;
    }
    G6::stamp_header_parity(block, block_byte_count);
  }

  frame_byte_count_ = (uint16_t)(stream_frame_prefix_byte_count
                                 + panel_count_per_frame * block_byte_count);
}

void CommandProcessor::fillFrameBufferDark() {
  // GS2 Persistent frame, all pixels + duty_cycle = 0 → panels scan dark and
  // HOLD dark after transmission stops. A fixed Persistent opcode (not
  // panel_disp_mode_) guarantees blanking even from Triggered/Gated modes,
  // mirroring how the error-glyph path uses a mode-independent opcode.
  memset(frame_buf_, 0, sizeof(frame_buf_));
  frame_buf_[0] = 'F';
  frame_buf_[1] = 'R';

  const uint16_t blk = G6::block_byte_count_gs2;
  for (uint8_t p = 0; p < panel_count_per_frame; ++p) {
    uint8_t *block = frame_buf_ + stream_frame_prefix_byte_count
                     + (uint32_t)p * blk;
    block[0] = G6::header_version_v1;          // parity stamped below
    block[1] = G6::cmd_disp_2lvl_persist;      // 0x11 — Persistent, holds dark
    // pixel bytes + duty_cycle remain 0 from the memset above.
    G6::stamp_header_parity(block, blk);
  }
  block_byte_count_ = blk;
  frame_byte_count_ = (uint16_t)(stream_frame_prefix_byte_count
                                 + panel_count_per_frame * blk);
}

// ---------------------------------------------------------------------------
// handleBulkWriteCommand — set-pattern-file (0x85)
//
// Returns true only on the one path that hands off to serviceUpload():
// the caller must leave that command unconsumed so parseIncoming() doesn't
// mistake the still-arriving raw bytes for a new command. Every other path
// is rejected, drained, and responded to synchronously right here, so it
// returns false: the caller must consume the command immediately regardless
// of ul_active_'s value for an unrelated transfer on the OTHER source (PR
// #27 review point 1; see the ul_* fields' comment in CommandProcessor.h).
// ---------------------------------------------------------------------------

bool CommandProcessor::handleBulkWriteCommand(const ParsedCommand &cmd) {
  if (cmd.cmd == SET_FIRMWARE_FILE_CMD) {
    return handleSetFirmwareFile(cmd);
  }
  if (cmd.cmd != SET_PATTERN_FILE_CMD) {
    drainBulkData(cmd.bulk_payload_len);
    current_source_->sendResponse(cmd.cmd, 1, "Unknown bulk command");
    return false;
  }

  // Header: [0x85, idx_lo, idx_hi, len_b0..b7] (already parsed by transport)
  uint16_t idx;
  memcpy(&idx, cmd.data + 1, sizeof(idx));
  uint32_t total_len = cmd.bulk_payload_len;

  // A 0-byte pattern file isn't a valid pattern (no header, no frames);
  // reject it outright rather than opening/keeping an empty file around for
  // something to fail on later (PR #27 review point 7). No drainBulkData()
  // needed: total_len == 0 means no payload bytes follow the header at all.
  if (total_len == 0) {
    current_source_->sendResponse(SET_PATTERN_FILE_CMD, 1, "Empty upload not supported");
    return false;
  }

  if (state_ != ArenaState::ALL_OFF) {
    drainBulkData(total_len);
    current_source_->sendResponse(SET_PATTERN_FILE_CMD, CE_DISPLAY_ACTIVE,
                                  "Stop display before writing to SD");
    return false;
  }

  if (ul_active_) {
    drainBulkData(total_len);
    current_source_->sendResponse(SET_PATTERN_FILE_CMD, 1, "Upload already in progress");
    return false;
  }

  // The write below (SD.remove() + reopen for write) would race whichever
  // OTHER transfer already has this same file open for reading: a
  // download's dl_file_, or, since an archive can be reading any pattern in
  // the library at a given moment, any active archive at all (PR #27 review
  // point 8). Unlike a delete (which only frees clusters a reader degrades
  // gracefully without), a concurrent write reallocates them, so this one
  // is refused outright rather than tolerated.
  if (dl_active_ && dl_idx_ == idx) {
    drainBulkData(total_len);
    current_source_->sendResponse(SET_PATTERN_FILE_CMD, 1, "Pattern is being downloaded");
    return false;
  }

  if (ar_active_) {
    drainBulkData(total_len);
    current_source_->sendResponse(SET_PATTERN_FILE_CMD, 1, "Archive in progress");
    return false;
  }

  if (idx > sd_.patternCount()) {
    drainBulkData(total_len);
    current_source_->sendResponse(SET_PATTERN_FILE_CMD, 1, "Index out of range");
    return false;
  }

  // Build destination path.
  char path[sizeof(ul_path_)];
  if (idx == 0) {
    snprintf(path, sizeof(path), "%s/pattern.temp", AC::constants::pattern_dir);
  } else {
    const char *name = sd_.patternName(idx);
    snprintf(path, sizeof(path), "%s/%s", AC::constants::pattern_dir, name);
  }

  // Remove any existing file so we get a clean write.
  SD.remove(path);
  File f = SD.open(path, FILE_WRITE);
  if (!f) {
    drainBulkData(total_len);
    current_source_->sendResponse(SET_PATTERN_FILE_CMD, 1, "SD open failed");
    return false;
  }

  // Hand off to serviceUpload() (driven from loop(), like serviceDownload())
  // to read+write one ~4 KB chunk per loop iteration instead of blocking here
  // for the whole transfer — see the ul_* fields' comment in
  // CommandProcessor.h for why this doesn't desync the command parser.
  snprintf(ul_path_, sizeof(ul_path_), "%s", path);
  ul_file_      = f;
  ul_source_    = current_source_;
  ul_idx_       = idx;
  ul_remaining_ = total_len;
  ul_total_     = total_len;
  ul_start_ms_  = millis();
  ul_deadline_  = millis() + kUploadIdleTimeoutMs;
  ul_phase_     = UploadPhase::kWriting;
  ul_active_    = true;
  return true;
}

// ---------------------------------------------------------------------------
// handleSetFirmwareFile — set-firmware-file (0xE0)
// ---------------------------------------------------------------------------
// Streams the uploaded panel firmware image to /firmware/panel.bin (single
// image, no index) and replies with the uint32 LE CRC-32 of the stored bytes
// so the host can confirm the upload before issuing g6-program-panel.
//
// Still a single blocking call, unlike 0x84/0x85/0x8A, which all stream one
// chunk per loop() call. While this runs, nothing else in loop() executes,
// including serviceDownload()/serviceUpload()/serviceArchive() for an
// unrelated transfer on the OTHER source. That transfer's idle deadline
// measures wall-clock time since its last successfully drained chunk, not
// whether its own host stopped sending, so a long firmware flash here could
// starve it long enough to trip a false timeout and delete its partial
// file (PR #27 review point 1's follow-on hazard, beyond the redispatch bug
// the ownership fix targets). Refusing to start while ANY of
// dl_/ul_/ar_active_ is set avoids that outright, rather than letting this
// run and silently wreck an unrelated transfer's timing. Always returns
// false (no async handoff): every path here, success or failure, is fully
// synchronous by the time it returns.

bool CommandProcessor::handleSetFirmwareFile(const ParsedCommand &cmd) {
  uint32_t total_len = cmd.bulk_payload_len;

  if (state_ != ArenaState::ALL_OFF) {
    drainBulkData(total_len);
    current_source_->sendResponse(SET_FIRMWARE_FILE_CMD, CE_DISPLAY_ACTIVE,
                                  "Stop display before writing to SD");
    return false;
  }

  if (dl_active_ || ul_active_ || ar_active_) {
    drainBulkData(total_len);
    current_source_->sendResponse(SET_FIRMWARE_FILE_CMD, 1,
                                  "Bulk transfer already in progress");
    return false;
  }

  SD.mkdir(AC::constants::firmware_dir);
  SD.remove(AC::constants::firmware_path);
  File f = SD.open(AC::constants::firmware_path, FILE_WRITE);
  if (!f) {
    drainBulkData(total_len);
    current_source_->sendResponse(SET_FIRMWARE_FILE_CMD, 1, "SD open failed");
    return false;
  }

  static constexpr size_t CHUNK = 4096;
  static uint8_t chunk[CHUNK];
  uint32_t remaining = total_len;
  uint32_t crc       = 0xFFFFFFFFu;  // CRC-32/ISO-HDLC running state
  uint32_t t_idle    = millis();
  uint32_t t_start   = millis();
  bool ok            = true;
  const char *fail_msg = "Upload timeout";

  while (remaining > 0) {
    size_t want = (remaining < CHUNK) ? (size_t)remaining : CHUNK;
    size_t got  = current_source_->readBulkBytes(chunk, want);
    if (got > 0) {
      size_t written = f.write(chunk, got);
      if (written != got) {
        fail_msg = "SD write error (card full?)";
        ok = false;
        break;
      }
      crc = G6::crc32_update(crc, chunk, got);
      remaining -= (uint32_t)got;
      t_idle = millis();
    } else {
      if ((uint32_t)(millis() - t_idle) > 30000U) { ok = false; break; }
      yield();
    }
  }

  uint32_t elapsed_ms = millis() - t_start;
  f.close();
  if (!ok) {
    SD.remove(AC::constants::firmware_path);
    // Drain undelivered payload so leftover bytes don't desync the command loop.
    drainBulkData(remaining);
    current_source_->sendResponse(SET_FIRMWARE_FILE_CMD, 1, fail_msg);
    return false;
  }

  crc ^= 0xFFFFFFFFu;  // CRC-32 final XOR
  uint32_t kbps = elapsed_ms > 0 ? (uint32_t)(total_len / elapsed_ms) : 0;
  DBG_PRINTF("[cmd] set-firmware-file wrote %lu bytes in %lu ms (%lu kB/s) crc=0x%08lX\n",
             (unsigned long)total_len, (unsigned long)elapsed_ms,
             (unsigned long)kbps, (unsigned long)crc);
  uint8_t crc_buf[4];
  memcpy(crc_buf, &crc, sizeof(crc));  // uint32 LE
  current_source_->sendResponse(SET_FIRMWARE_FILE_CMD, 0, crc_buf, sizeof(crc_buf));
  return false;
}

// ---------------------------------------------------------------------------
// handleGetFirmwareInfo — get-firmware-info (0xE3)
// ---------------------------------------------------------------------------
// Replies with the 32-byte footer at the end of /firmware/panel.bin:
//   {magic[8], version[16], image_crc32(u32 LE), image_size(u32 LE)}.

void CommandProcessor::handleGetFirmwareInfo() {
  File f = SD.open(AC::constants::firmware_path, FILE_READ);
  if (!f) {
    current_source_->sendResponse(GET_FIRMWARE_INFO_CMD, 1, "No firmware image present");
    return;
  }
  uint32_t sz = (uint32_t)f.size();
  constexpr uint8_t FOOT = AC::constants::firmware_footer_byte_count;
  if (sz < FOOT) {
    f.close();
    current_source_->sendResponse(GET_FIRMWARE_INFO_CMD, 1, "Firmware image too small");
    return;
  }
  uint8_t footer[FOOT];
  f.seek(sz - FOOT);
  size_t n = f.read(footer, FOOT);
  f.close();
  if (n != (size_t)FOOT) {
    current_source_->sendResponse(GET_FIRMWARE_INFO_CMD, 1, "Footer read failed");
    return;
  }
  current_source_->sendResponse(GET_FIRMWARE_INFO_CMD, 0, footer, FOOT);
}

// ---------------------------------------------------------------------------
// handleProgramPanel — g6-program-panel (0xC8): reflash one panel over SPI ISP
// ---------------------------------------------------------------------------
// [0x02, 0xC8, panel_index]. Caller (the switch) has already required ALL_OFF.
// Blocks for several seconds while the panel stages → commits → reboots.

void CommandProcessor::handleProgramPanel(const ParsedCommand &cmd) {
  if (cmd.data_len < 3) {
    current_source_->sendResponse(G6_PROGRAM_PANEL_CMD, 1, "Expected [02 C8 panel_number]");
    return;
  }
  // Payload is the 1-based panel NUMBER (matches the panel-map labels); the arena
  // config is 0-based (panel_index = row*10 + col), so convert here.
  uint8_t panel_number = cmd.data[2];
  if (panel_number < 1) {
    current_source_->sendResponse(G6_PROGRAM_PANEL_CMD, 1, "panel numbers are 1-based (1..N)");
    return;
  }
  uint8_t panel_index = (uint8_t)(panel_number - 1);
  DBG_PRINTF("[cmd] g6-program-panel panel=%u (idx=%u) — starting ISP\n",
             (unsigned)panel_number, (unsigned)panel_index);
  char msg[160];  // room for a raw-CIPO hex dump on garbled-reply diagnostics
  bool ok = isp_.programPanel(panel_index, msg, sizeof(msg));
  DBG_PRINTF("[cmd] g6-program-panel panel=%u %s: %s\n",
             (unsigned)panel_number, ok ? "OK" : "FAIL", msg);
  current_source_->sendResponse(G6_PROGRAM_PANEL_CMD, ok ? 0 : 1, msg);
}

// ---------------------------------------------------------------------------
// handleVerifyPanel — g6-verify-panel (0xC9): CRC the panel's RUNNING app flash
// against /firmware/panel.bin (ISP_ENTER + ISP_VERIFY_CRC). Confirms an OTA took.
// [0x02, 0xC9, panel_number]. Caller (the switch) has already required ALL_OFF.
// ---------------------------------------------------------------------------
void CommandProcessor::handleVerifyPanel(const ParsedCommand &cmd) {
  if (cmd.data_len < 3) {
    current_source_->sendResponse(G6_VERIFY_PANEL_CMD, 1, "Expected [02 C9 panel_number]");
    return;
  }
  // 1-based panel number (matches the panel-map) → 0-based arena index.
  uint8_t panel_number = cmd.data[2];
  if (panel_number < 1) {
    current_source_->sendResponse(G6_VERIFY_PANEL_CMD, 1, "panel numbers are 1-based (1..N)");
    return;
  }
  uint8_t panel_index = (uint8_t)(panel_number - 1);
  char msg[160];
  bool ok = isp_.verifyPanel(panel_index, msg, sizeof(msg));
  DBG_PRINTF("[cmd] g6-verify-panel panel=%u %s: %s\n",
             (unsigned)panel_number, ok ? "MATCH" : "NOMATCH", msg);
  current_source_->sendResponse(G6_VERIFY_PANEL_CMD, ok ? 0 : 1, msg);
}

// Reads and discards a rejected/undelivered bulk payload so the host's
// trailing bytes don't get misparsed as the start of a new command. Two
// give-up conditions, whichever comes first (PR #27 review point 6):
//   - idle: no bytes at all for kIdleTimeoutMs
//   - absolute: kAbsoluteTimeoutMs total, from when draining started,
//     regardless of how many times the idle deadline got reset
// The idle-only bound let a sender pacing bytes just under it keep this call
// "making progress" indefinitely, blocking loop() for minutes over a single
// rejected upload. This is purely resync after a decision that's already
// been made (the command was already rejected), nothing is being preserved,
// so there's no reason to tolerate an unbounded drain the way a real
// transfer's own idle-reset deadline legitimately does.
void CommandProcessor::drainBulkData(uint32_t remaining) {
  static constexpr uint32_t kIdleTimeoutMs     = 5000UL;
  static constexpr uint32_t kAbsoluteTimeoutMs = 15000UL;
  uint8_t buf[256];
  uint32_t start = millis();
  uint32_t t0 = start;
  while (remaining > 0) {
    if ((uint32_t)(millis() - start) > kAbsoluteTimeoutMs) break;
    size_t want = (remaining < sizeof(buf)) ? (size_t)remaining : sizeof(buf);
    size_t got = current_source_->readBulkBytes(buf, want);
    if (got > 0) {
      remaining -= (uint32_t)got;
      t0 = millis();
    } else {
      if ((uint32_t)(millis() - t0) > kIdleTimeoutMs) break;
      yield();
    }
  }
}

// ---------------------------------------------------------------------------
// handleGetSdArchive: 0x8A, arms a full-SD-content ZIP (store mode) stream.
// Sends a bulk-read response header (8-byte uint64 LE total size) here,
// synchronously (entry collection only stats files, open+size+close; it
// never calls sendRaw() and can't stall a transport). The ZIP bytes
// themselves (headers, file data, data descriptors, central directory,
// EOCD) are streamed asynchronously afterward by serviceArchive(), one
// bounded sendRaw() per loop() call; see the ar_* fields' comment in
// CommandProcessor.h for why (PR #27 review point 2).
//
// ZIP uses data descriptors (flags=0x0008) so CRC/sizes are deferred — the
// total size can be computed before streaming any file data.
// ---------------------------------------------------------------------------

void CommandProcessor::handleGetSdArchive() {
  uint16_t entry_count = 0;

  // --- Collect root manifest files ---
  static const char *const kRootFiles[] = { "MANIFEST.bin", "MANIFEST.txt" };
  for (uint8_t i = 0; i < 2; ++i) {
    ZipEntry &e = ar_entries_[entry_count];
    e.name_len = (uint8_t)strlen(kRootFiles[i]);
    memcpy(e.zip_name, kRootFiles[i], e.name_len + 1);
    snprintf(e.sd_path, sizeof(e.sd_path), "/%s", kRootFiles[i]);
    File f = SD.open(e.sd_path, FILE_READ);
    if (!f) continue;
    e.file_size = (uint32_t)f.size();
    f.close();
    e.crc32 = 0;
    e.lhf_offset = 0;
    ++entry_count;
  }

  // --- Collect pattern files ---
  uint16_t pc = sd_.patternCount();
  for (uint16_t i = 1; i <= pc && entry_count < kArchiveMaxEntries; ++i) {
    const char *name = sd_.patternName(i);
    if (!name) continue;
    ZipEntry &e = ar_entries_[entry_count];
    snprintf(e.zip_name, sizeof(e.zip_name), "patterns/%s", name);
    e.name_len = (uint8_t)strlen(e.zip_name);
    snprintf(e.sd_path, sizeof(e.sd_path), "%s/%s", AC::constants::pattern_dir, name);
    File f = SD.open(e.sd_path, FILE_READ);
    if (!f) continue;
    e.file_size = (uint32_t)f.size();
    f.close();
    e.crc32 = 0;
    e.lhf_offset = 0;
    ++entry_count;
  }

  // --- Pre-compute total ZIP size and per-entry LFH offsets ---
  // Each entry: LFH(30+nlen) + data(file_size) + DataDescriptor(16)
  // Central directory: sum of (46+nlen) per entry
  // EOCD: 22 bytes
  uint32_t offset  = 0;
  uint32_t cd_size = 0;
  for (uint16_t i = 0; i < entry_count; ++i) {
    ar_entries_[i].lhf_offset = offset;
    offset  += 30u + ar_entries_[i].name_len + ar_entries_[i].file_size + 16u;
    cd_size += 46u + ar_entries_[i].name_len;
  }
  uint32_t cd_offset  = offset;
  uint64_t total_size = (uint64_t)cd_offset + cd_size + 22u;

  // --- Send bulk response header (8-byte uint64 LE size, then raw ZIP) ---
  uint8_t size_buf[8] = {};
  memcpy(size_buf, &total_size, sizeof(total_size));
  current_source_->sendResponse(GET_SD_ARCHIVE_CMD, 0, size_buf, sizeof(size_buf));

  // --- Arm the async stream; serviceArchive() sends the ZIP body from loop() ---
  ar_entry_count_ = entry_count;
  ar_entry_idx_    = 0;
  ar_phase_        = ArchivePhase::kLocalHeader;
  ar_cd_offset_    = cd_offset;
  ar_cd_size_      = cd_size;
  ar_source_       = current_source_;
  ar_deadline_     = millis() + kArchiveIdleTimeoutMs;
  ar_active_       = true;

  DBG_PRINTF("[cmd] get-sd-archive armed: %u files, %lu bytes\n",
             (unsigned)entry_count, (unsigned long)total_size);
}

// ---------------------------------------------------------------------------
// serviceArchive: one bounded step of the GET_SD_ARCHIVE (0x8A) ZIP stream
// per loop() call. Mirrors serviceDownload()/serviceUpload() (issue #16):
// each phase does at most one sendRaw() (file data is additionally chunked
// to kChunkBytes, same as serviceDownload), checks its return, and aborts
// the WHOLE transfer on the first short write. It never retries a partial
// send inline, per MessageSource::sendRaw()'s documented contract.
// ---------------------------------------------------------------------------

void CommandProcessor::serviceArchive() {
  if (!ar_active_) return;

  if ((int32_t)(millis() - ar_deadline_) >= 0) {
    DBG_PRINTF("[cmd] get-sd-archive idle TIMEOUT at entry %u/%u phase=%u\n",
               (unsigned)ar_entry_idx_, (unsigned)ar_entry_count_,
               (unsigned)ar_phase_);
    abortArchive();
    return;
  }

  static constexpr size_t kChunkBytes = 4096;
  static uint8_t chunk[kChunkBytes];
  static const uint8_t kZeros[kChunkBytes] = {};

  switch (ar_phase_) {
    case ArchivePhase::kLocalHeader: {
      ZipEntry &e = ar_entries_[ar_entry_idx_];
      uint8_t buf[30 + sizeof(e.zip_name)] = {};
      buf[0]=0x50; buf[1]=0x4B; buf[2]=0x03; buf[3]=0x04;  // PK\x03\x04
      buf[4]=20;                                              // version needed 2.0
      buf[6]=0x08;                                            // flags: data descriptor
      buf[26]=(uint8_t)e.name_len;                            // file name length LE
      memcpy(buf + 30, e.zip_name, e.name_len);
      size_t total = 30u + e.name_len;
      size_t sent = ar_source_->sendRaw(buf, total);
      if (sent < total) { abortArchive(); return; }

      // Hand off to kFileData: open (or fail to open) the entry's file now,
      // once, rather than re-attempting it every tick.
      ar_crc_           = 0xFFFFFFFFu;
      ar_file_remaining_ = e.file_size;
      ar_file_          = SD.open(e.sd_path, FILE_READ);
      ar_phase_         = ArchivePhase::kFileData;
      ar_deadline_      = millis() + kArchiveIdleTimeoutMs;
      break;
    }

    case ArchivePhase::kFileData: {
      if (ar_file_remaining_ == 0) {
        if (ar_file_) ar_file_.close();
        ar_entries_[ar_entry_idx_].crc32 = ar_crc_ ^ 0xFFFFFFFFu;
        ar_phase_ = ArchivePhase::kDataDescriptor;
        break;
      }
      size_t want = (ar_file_remaining_ < kChunkBytes) ? (size_t)ar_file_remaining_ : kChunkBytes;
      if (ar_file_) {
        size_t got = (size_t)ar_file_.read(chunk, want);
        if (got > 0) {
          ar_crc_ = G6::crc32_update(ar_crc_, chunk, got);
          size_t sent = ar_source_->sendRaw(chunk, got);
          if (sent < got) { abortArchive(); return; }
          ar_file_remaining_ -= (uint32_t)got;
          ar_deadline_ = millis() + kArchiveIdleTimeoutMs;
          break;
        }
        // EOF/read-error before ar_file_remaining_ hit 0, so fall back to
        // zero-padding the rest so the ZIP's promised file_size still holds
        // (mirrors the old code's short-file/unavailable-file fallback).
        ar_file_.close();
      }
      // No file open (never opened, or just closed above): zero-pad this tick.
      ar_crc_ = G6::crc32_update(ar_crc_, kZeros, want);
      size_t sent = ar_source_->sendRaw(kZeros, want);
      if (sent < want) { abortArchive(); return; }
      ar_file_remaining_ -= (uint32_t)want;
      ar_deadline_ = millis() + kArchiveIdleTimeoutMs;
      break;
    }

    case ArchivePhase::kDataDescriptor: {
      ZipEntry &e = ar_entries_[ar_entry_idx_];
      uint8_t dd[16];
      dd[0]=0x50; dd[1]=0x4B; dd[2]=0x07; dd[3]=0x08;
      dd[4] =(uint8_t)e.crc32;          dd[5] =(uint8_t)(e.crc32>>8);
      dd[6] =(uint8_t)(e.crc32>>16);    dd[7] =(uint8_t)(e.crc32>>24);
      dd[8] =(uint8_t)e.file_size;      dd[9] =(uint8_t)(e.file_size>>8);
      dd[10]=(uint8_t)(e.file_size>>16);dd[11]=(uint8_t)(e.file_size>>24);
      dd[12]=(uint8_t)e.file_size;      dd[13]=(uint8_t)(e.file_size>>8);
      dd[14]=(uint8_t)(e.file_size>>16);dd[15]=(uint8_t)(e.file_size>>24);
      size_t sent = ar_source_->sendRaw(dd, sizeof(dd));
      if (sent < sizeof(dd)) { abortArchive(); return; }

      ++ar_entry_idx_;
      ar_deadline_ = millis() + kArchiveIdleTimeoutMs;
      if (ar_entry_idx_ < ar_entry_count_) {
        ar_phase_ = ArchivePhase::kLocalHeader;
      } else {
        ar_entry_idx_ = 0;  // reused below as the Central Directory index
        ar_phase_ = ArchivePhase::kCentralDir;
      }
      break;
    }

    case ArchivePhase::kCentralDir: {
      ZipEntry &e = ar_entries_[ar_entry_idx_];
      uint8_t buf[46 + sizeof(e.zip_name)] = {};
      buf[0]=0x50; buf[1]=0x4B; buf[2]=0x01; buf[3]=0x02;  // PK\x01\x02
      buf[4]=20;   buf[6]=20;                                // version made by / needed
      buf[8]=0x08;                                           // flags
      buf[16]=(uint8_t)e.crc32;           buf[17]=(uint8_t)(e.crc32>>8);
      buf[18]=(uint8_t)(e.crc32>>16);     buf[19]=(uint8_t)(e.crc32>>24);
      buf[20]=(uint8_t)e.file_size;       buf[21]=(uint8_t)(e.file_size>>8);
      buf[22]=(uint8_t)(e.file_size>>16); buf[23]=(uint8_t)(e.file_size>>24);
      buf[24]=(uint8_t)e.file_size;       buf[25]=(uint8_t)(e.file_size>>8);
      buf[26]=(uint8_t)(e.file_size>>16); buf[27]=(uint8_t)(e.file_size>>24);
      buf[28]=(uint8_t)e.name_len;                           // file name length
      buf[42]=(uint8_t)e.lhf_offset;      buf[43]=(uint8_t)(e.lhf_offset>>8);
      buf[44]=(uint8_t)(e.lhf_offset>>16);buf[45]=(uint8_t)(e.lhf_offset>>24);
      memcpy(buf + 46, e.zip_name, e.name_len);
      size_t total = 46u + e.name_len;
      size_t sent = ar_source_->sendRaw(buf, total);
      if (sent < total) { abortArchive(); return; }

      ++ar_entry_idx_;
      ar_deadline_ = millis() + kArchiveIdleTimeoutMs;
      if (ar_entry_idx_ >= ar_entry_count_) ar_phase_ = ArchivePhase::kEocd;
      break;
    }

    case ArchivePhase::kEocd: {
      uint8_t eocd[22] = {};
      eocd[0]=0x50; eocd[1]=0x4B; eocd[2]=0x05; eocd[3]=0x06;  // PK\x05\x06
      eocd[8] =(uint8_t)ar_entry_count_;  eocd[9] =(uint8_t)(ar_entry_count_>>8);
      eocd[10]=(uint8_t)ar_entry_count_;  eocd[11]=(uint8_t)(ar_entry_count_>>8);
      eocd[12]=(uint8_t)ar_cd_size_;      eocd[13]=(uint8_t)(ar_cd_size_>>8);
      eocd[14]=(uint8_t)(ar_cd_size_>>16);eocd[15]=(uint8_t)(ar_cd_size_>>24);
      eocd[16]=(uint8_t)ar_cd_offset_;    eocd[17]=(uint8_t)(ar_cd_offset_>>8);
      eocd[18]=(uint8_t)(ar_cd_offset_>>16);eocd[19]=(uint8_t)(ar_cd_offset_>>24);
      size_t sent = ar_source_->sendRaw(eocd, sizeof(eocd));
      if (sent < sizeof(eocd)) { abortArchive(); return; }

      DBG_PRINTF("[cmd] get-sd-archive complete: %u files\n", (unsigned)ar_entry_count_);
      ar_active_ = false;
      break;
    }
  }
}

// Common teardown for an aborted (stalled or timed-out) archive stream.
// serviceArchive() calls this instead of retrying a short sendRaw() inline,
// per MessageSource::sendRaw()'s documented contract.
void CommandProcessor::abortArchive() {
  if (ar_file_) ar_file_.close();
  ar_active_ = false;
}
