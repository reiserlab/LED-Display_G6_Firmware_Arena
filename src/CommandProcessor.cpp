#include "CommandProcessor.h"
#include "Crc.h"
#include "ErrorGlyph.h"
#include <Wire.h>

using namespace AC;
using namespace AC::constants;

void CommandProcessor::begin() {
  Wire.begin();
  Wire.setClock(400000);

  // Digital outputs: both channels start driving LOW, direction = Teensy→BNC.
  pinMode(do1_data_pin, OUTPUT); digitalWrite(do1_data_pin, LOW);
  pinMode(do1_dir_pin,  OUTPUT); digitalWrite(do1_dir_pin,  HIGH);
  pinMode(do2_data_pin, OUTPUT); digitalWrite(do2_data_pin, LOW);
  pinMode(do2_dir_pin,  OUTPUT); digitalWrite(do2_dir_pin,  HIGH);
}

// ---------------------------------------------------------------------------
// Command dispatch
// ---------------------------------------------------------------------------

void CommandProcessor::processCommand() {
  // Drain one command from each source per loop iteration. Net first
  // (lower-latency TCP path), then serial. Each source uses its own
  // response buffer; current_source_ tells the handlers which one to
  // send the reply back through.
  if (net_.hasCommand()) {
    current_source_ = &net_;
    const ParsedCommand &cmd = net_.command();
    if (cmd.is_bulk) {
      handleBulkWriteCommand(cmd);
    } else if (cmd.is_stream) {
      handleStreamCommand(cmd);
    } else {
      handleBinaryCommand(cmd);
    }
    net_.commandConsumed();
  }
  if (serial_.hasCommand()) {
    current_source_ = &serial_;
    const ParsedCommand &cmd = serial_.command();
    if (cmd.is_bulk) {
      handleBulkWriteCommand(cmd);
    } else if (cmd.is_stream) {
      handleStreamCommand(cmd);
    } else {
      handleBinaryCommand(cmd);
    }
    serial_.commandConsumed();
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
      // [03 84 idx_lo idx_hi]  Response: [0x0A, 0, 0x84, size_b0..b7], then raw bytes.
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
      // Stream raw file bytes via sendRaw (flushes the header on the first call).
      static uint8_t chunk[4096];
      uint32_t remaining = file_size;
      uint32_t deadline = millis() + 60000UL;
      uint32_t t_start = millis();
      bool timed_out = false;
      while (remaining > 0) {
        if (millis() > deadline) { timed_out = true; break; }
        size_t to_read = (remaining < sizeof(chunk)) ? (size_t)remaining : sizeof(chunk);
        size_t n = (size_t)f.read(chunk, to_read);
        if (n == 0) break;
        current_source_->sendRaw(chunk, n);
        remaining -= n;
      }
      uint32_t elapsed_ms = millis() - t_start;
      uint32_t kbps = elapsed_ms > 0 ? (uint32_t)(file_size / elapsed_ms) : 0;
      f.close();
      DBG_PRINTF("[cmd] get-pattern-file idx=%u name=%s size=%lu elapsed=%lu ms (%lu kB/s)%s\n",
                 (unsigned)idx, name, (unsigned long)file_size,
                 (unsigned long)elapsed_ms, (unsigned long)kbps,
                 timed_out ? " (TIMEOUT)" : "");
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
      uint8_t data_pin, dir_pin;
      if (channel == 1) {
        data_pin = do1_data_pin;
        dir_pin  = do1_dir_pin;
      } else if (channel == 2) {
        data_pin = do2_data_pin;
        dir_pin  = do2_dir_pin;
      } else {
        current_source_->sendResponse(command_byte, 1, "channel must be 1 or 2");
        break;
      }
      digitalWrite(dir_pin,  HIGH);
      digitalWrite(data_pin, state ? HIGH : LOW);
      current_source_->sendResponse(command_byte, 0, "");
      DBG_PRINTF("[cmd] set-digital-out ch=%u state=%u\n",
                 (unsigned)channel, (unsigned)state);
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
      uint16_t dac_code = (uint16_t)((uint32_t)mv * 4095 / 5000);
      Wire.beginTransmission(0x60);
      Wire.write((uint8_t)((dac_code >> 8) & 0x0F));
      Wire.write((uint8_t)(dac_code & 0xFF));
      uint8_t i2c_err = Wire.endTransmission();
      if (i2c_err != 0) {
        current_source_->sendResponse(command_byte, 1, "I2C write failed");
        break;
      }
      ao_mv_ = mv;
      ao_lut_len_ = 0;  // stop any active LUT playback
      uint8_t payload[2] = { (uint8_t)(mv), (uint8_t)(mv >> 8) };
      current_source_->sendResponse(command_byte, 0, payload, sizeof(payload));
      DBG_PRINTF("[cmd] set-ao-voltage mv=%u dac=%u\n",
                 (unsigned)mv, (unsigned)dac_code);
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
  // Response payload {version_byte, capability_bitmap} (g6_03 § 5).
  const uint8_t payload[2] = {
      controller_info_version,
      controller_capability_bitmap,
  };
  current_source_->sendResponse(GET_CONTROLLER_INFO_CMD, 0, payload, sizeof(payload));
  DBG_PRINTF("[cmd] controller-info v=%u cap=0x%02X\n",
             (unsigned)payload[0], (unsigned)payload[1]);
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
  if (ao_lut_len_ > 0 && ao_lut_mode_ == 0) {
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

bool CommandProcessor::applyAoLut(uint16_t idx) {
  if (ao_lut_len_ == 0) return true;
  uint16_t mv = ao_lut_[idx % ao_lut_len_];
  uint16_t dac_code = (uint16_t)((uint32_t)mv * 4095 / 5000);
  Wire.beginTransmission(0x60);
  Wire.write((uint8_t)((dac_code >> 8) & 0x0F));
  Wire.write((uint8_t)(dac_code & 0xFF));
  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    DBG_PRINTF("[ao_lut] I2C error %u at idx=%u mv=%u\n",
               (unsigned)err, (unsigned)idx, (unsigned)mv);
    return false;
  }
  ao_mv_ = mv;
  return true;
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
// ---------------------------------------------------------------------------

void CommandProcessor::handleBulkWriteCommand(const ParsedCommand &cmd) {
  if (cmd.cmd == SET_FIRMWARE_FILE_CMD) {
    handleSetFirmwareFile(cmd);
    return;
  }
  if (cmd.cmd != SET_PATTERN_FILE_CMD) {
    drainBulkData(cmd.bulk_payload_len);
    current_source_->sendResponse(cmd.cmd, 1, "Unknown bulk command");
    return;
  }

  // Header: [0x85, idx_lo, idx_hi, len_b0..b7] (already parsed by transport)
  uint16_t idx;
  memcpy(&idx, cmd.data + 1, sizeof(idx));
  uint32_t total_len = cmd.bulk_payload_len;

  if (state_ != ArenaState::ALL_OFF) {
    drainBulkData(total_len);
    current_source_->sendResponse(SET_PATTERN_FILE_CMD, CE_DISPLAY_ACTIVE,
                                  "Stop display before writing to SD");
    return;
  }

  if (idx > sd_.patternCount()) {
    drainBulkData(total_len);
    current_source_->sendResponse(SET_PATTERN_FILE_CMD, 1, "Index out of range");
    return;
  }

  // Build destination path.
  char path[AC::constants::pattern_name_byte_count + 16];
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
    return;
  }

  static constexpr size_t CHUNK = 4096;
  static uint8_t chunk[CHUNK];
  uint32_t remaining = total_len;
  uint32_t t_idle = millis();   // tracks last data arrival (timeout)
  uint32_t t_start = millis();  // start of write phase (throughput)
  bool ok = true;
  const char *fail_msg = "Upload timeout";

  while (remaining > 0) {
    size_t want = (remaining < CHUNK) ? (size_t)remaining : CHUNK;
    size_t got = current_source_->readBulkBytes(chunk, want);
    if (got > 0) {
      size_t written = f.write(chunk, got);
      if (written != got) {
        fail_msg = "SD write error (card full?)";
        ok = false;
        break;
      }
      remaining -= (uint32_t)got;
      t_idle = millis();
    } else {
      if ((uint32_t)(millis() - t_idle) > 30000U) {
        ok = false;
        break;
      }
      yield();
    }
  }

  uint32_t elapsed_ms = millis() - t_start;
  f.close();
  if (!ok) {
    SD.remove(path);
    // Consume the undelivered payload before returning to the command loop; otherwise
    // the leftover file bytes are parsed as bogus commands and desync the link.
    drainBulkData(remaining);
    current_source_->sendResponse(SET_PATTERN_FILE_CMD, 1, fail_msg);
    return;
  }

  uint32_t kbps = elapsed_ms > 0 ? (uint32_t)(total_len / elapsed_ms) : 0;
  DBG_PRINTF("[cmd] set-pattern-file idx=%u wrote %lu bytes in %lu ms (%lu kB/s) to %s\n",
             (unsigned)idx, (unsigned long)total_len,
             (unsigned long)elapsed_ms, (unsigned long)kbps, path);
  current_source_->sendResponse(SET_PATTERN_FILE_CMD, 0, "");
}

// ---------------------------------------------------------------------------
// handleSetFirmwareFile — set-firmware-file (0xE0)
// ---------------------------------------------------------------------------
// Streams the uploaded panel firmware image to /firmware/panel.bin (single
// image, no index) and replies with the uint32 LE CRC-32 of the stored bytes
// so the host can confirm the upload before issuing g6-program-panel.

void CommandProcessor::handleSetFirmwareFile(const ParsedCommand &cmd) {
  uint32_t total_len = cmd.bulk_payload_len;

  if (state_ != ArenaState::ALL_OFF) {
    drainBulkData(total_len);
    current_source_->sendResponse(SET_FIRMWARE_FILE_CMD, CE_DISPLAY_ACTIVE,
                                  "Stop display before writing to SD");
    return;
  }

  SD.mkdir(AC::constants::firmware_dir);
  SD.remove(AC::constants::firmware_path);
  File f = SD.open(AC::constants::firmware_path, FILE_WRITE);
  if (!f) {
    drainBulkData(total_len);
    current_source_->sendResponse(SET_FIRMWARE_FILE_CMD, 1, "SD open failed");
    return;
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
    return;
  }

  crc ^= 0xFFFFFFFFu;  // CRC-32 final XOR
  uint32_t kbps = elapsed_ms > 0 ? (uint32_t)(total_len / elapsed_ms) : 0;
  DBG_PRINTF("[cmd] set-firmware-file wrote %lu bytes in %lu ms (%lu kB/s) crc=0x%08lX\n",
             (unsigned long)total_len, (unsigned long)elapsed_ms,
             (unsigned long)kbps, (unsigned long)crc);
  uint8_t crc_buf[4];
  memcpy(crc_buf, &crc, sizeof(crc));  // uint32 LE
  current_source_->sendResponse(SET_FIRMWARE_FILE_CMD, 0, crc_buf, sizeof(crc_buf));
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

void CommandProcessor::drainBulkData(uint32_t remaining) {
  uint8_t buf[256];
  uint32_t t0 = millis();
  while (remaining > 0) {
    size_t want = (remaining < sizeof(buf)) ? (size_t)remaining : sizeof(buf);
    size_t got = current_source_->readBulkBytes(buf, want);
    if (got > 0) {
      remaining -= (uint32_t)got;
      t0 = millis();
    } else {
      if ((uint32_t)(millis() - t0) > 5000U) break;
      yield();
    }
  }
}

// ---------------------------------------------------------------------------
// handleGetSdArchive — 0x8A: stream full SD content as a ZIP (store mode).
// Sends a bulk-read response: 8-byte uint64 LE total size, then ZIP bytes.
// ZIP uses data descriptors (flags=0x0008) so CRC/sizes are deferred — the
// total size can be computed before streaming any file data.
// ---------------------------------------------------------------------------

void CommandProcessor::handleGetSdArchive() {
  struct ZipEntry {
    char     zip_name[80];  // path inside ZIP (e.g. "patterns/foo.pat")
    char     sd_path[80];   // full path on SD  (e.g. "/patterns/foo.pat")
    uint32_t file_size;
    uint32_t crc32;
    uint32_t lhf_offset;    // byte offset of this entry's Local File Header
    uint8_t  name_len;
  };
  static ZipEntry entries[258];  // 2 manifest + up to 256 patterns; ~44 KB BSS
  uint16_t entry_count = 0;

  // --- Collect root manifest files ---
  static const char *const kRootFiles[] = { "MANIFEST.bin", "MANIFEST.txt" };
  for (uint8_t i = 0; i < 2; ++i) {
    ZipEntry &e = entries[entry_count];
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
  for (uint16_t i = 1; i <= pc && entry_count < 258; ++i) {
    const char *name = sd_.patternName(i);
    if (!name) continue;
    ZipEntry &e = entries[entry_count];
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
    entries[i].lhf_offset = offset;
    offset  += 30u + entries[i].name_len + entries[i].file_size + 16u;
    cd_size += 46u + entries[i].name_len;
  }
  uint32_t cd_offset  = offset;
  uint64_t total_size = (uint64_t)cd_offset + cd_size + 22u;

  // --- Send bulk response header (8-byte uint64 LE size, then raw ZIP) ---
  uint8_t size_buf[8] = {};
  memcpy(size_buf, &total_size, sizeof(total_size));
  current_source_->sendResponse(GET_SD_ARCHIVE_CMD, 0, size_buf, sizeof(size_buf));

  // --- Stream each entry: LFH + data + DataDescriptor ---
  static uint8_t chunk[512];
  static const uint8_t kZeros[64] = {};

  for (uint16_t i = 0; i < entry_count; ++i) {
    ZipEntry &e = entries[i];

    // Local File Header (30 bytes, flags=0x0008 → data descriptor follows)
    uint8_t lfh[30] = {};
    lfh[0]=0x50; lfh[1]=0x4B; lfh[2]=0x03; lfh[3]=0x04;  // PK\x03\x04
    lfh[4]=20;                                              // version needed 2.0
    lfh[6]=0x08;                                            // flags: data descriptor
    lfh[26]=(uint8_t)e.name_len;                            // file name length LE
    current_source_->sendRaw(lfh, sizeof(lfh));
    current_source_->sendRaw((const uint8_t *)e.zip_name, e.name_len);

    // Stream file data, accumulate CRC-32
    uint32_t crc      = 0xFFFFFFFFu;
    uint32_t leftover = e.file_size;
    File f = SD.open(e.sd_path, FILE_READ);
    if (f) {
      uint32_t deadline = millis() + 60000UL;
      while (leftover > 0) {
        if (millis() > deadline) break;
        size_t want = (leftover < sizeof(chunk)) ? (size_t)leftover : sizeof(chunk);
        size_t got  = (size_t)f.read(chunk, want);
        if (got == 0) break;
        crc = G6::crc32_update(crc, chunk, got);
        current_source_->sendRaw(chunk, got);
        leftover -= (uint32_t)got;
      }
      f.close();
    }
    // Pad with zeros if file is short or unavailable (preserves ZIP structure)
    while (leftover > 0) {
      size_t n = (leftover < sizeof(kZeros)) ? (size_t)leftover : sizeof(kZeros);
      crc = G6::crc32_update(crc, kZeros, n);
      current_source_->sendRaw(kZeros, n);
      leftover -= (uint32_t)n;
    }
    e.crc32 = crc ^ 0xFFFFFFFFu;

    // Data Descriptor (16 bytes): PK\x07\x08 + crc32 + comp_size + uncomp_size
    uint8_t dd[16];
    dd[0]=0x50; dd[1]=0x4B; dd[2]=0x07; dd[3]=0x08;
    dd[4] =(uint8_t)e.crc32;         dd[5] =(uint8_t)(e.crc32>>8);
    dd[6] =(uint8_t)(e.crc32>>16);   dd[7] =(uint8_t)(e.crc32>>24);
    dd[8] =(uint8_t)e.file_size;     dd[9] =(uint8_t)(e.file_size>>8);
    dd[10]=(uint8_t)(e.file_size>>16);dd[11]=(uint8_t)(e.file_size>>24);
    dd[12]=(uint8_t)e.file_size;     dd[13]=(uint8_t)(e.file_size>>8);
    dd[14]=(uint8_t)(e.file_size>>16);dd[15]=(uint8_t)(e.file_size>>24);
    current_source_->sendRaw(dd, sizeof(dd));
  }

  // --- Central Directory (one entry per file) ---
  for (uint16_t i = 0; i < entry_count; ++i) {
    ZipEntry &e = entries[i];
    uint8_t cde[46] = {};
    cde[0]=0x50; cde[1]=0x4B; cde[2]=0x01; cde[3]=0x02;  // PK\x01\x02
    cde[4]=20;   cde[6]=20;                                // version made by / needed
    cde[8]=0x08;                                           // flags
    cde[16]=(uint8_t)e.crc32;          cde[17]=(uint8_t)(e.crc32>>8);
    cde[18]=(uint8_t)(e.crc32>>16);    cde[19]=(uint8_t)(e.crc32>>24);
    cde[20]=(uint8_t)e.file_size;      cde[21]=(uint8_t)(e.file_size>>8);
    cde[22]=(uint8_t)(e.file_size>>16);cde[23]=(uint8_t)(e.file_size>>24);
    cde[24]=(uint8_t)e.file_size;      cde[25]=(uint8_t)(e.file_size>>8);
    cde[26]=(uint8_t)(e.file_size>>16);cde[27]=(uint8_t)(e.file_size>>24);
    cde[28]=(uint8_t)e.name_len;                           // file name length
    cde[42]=(uint8_t)e.lhf_offset;     cde[43]=(uint8_t)(e.lhf_offset>>8);
    cde[44]=(uint8_t)(e.lhf_offset>>16);cde[45]=(uint8_t)(e.lhf_offset>>24);
    current_source_->sendRaw(cde, sizeof(cde));
    current_source_->sendRaw((const uint8_t *)e.zip_name, e.name_len);
  }

  // --- End of Central Directory Record (22 bytes) ---
  uint8_t eocd[22] = {};
  eocd[0]=0x50; eocd[1]=0x4B; eocd[2]=0x05; eocd[3]=0x06;  // PK\x05\x06
  eocd[8] =(uint8_t)entry_count;      eocd[9] =(uint8_t)(entry_count>>8);
  eocd[10]=(uint8_t)entry_count;      eocd[11]=(uint8_t)(entry_count>>8);
  eocd[12]=(uint8_t)cd_size;          eocd[13]=(uint8_t)(cd_size>>8);
  eocd[14]=(uint8_t)(cd_size>>16);    eocd[15]=(uint8_t)(cd_size>>24);
  eocd[16]=(uint8_t)cd_offset;        eocd[17]=(uint8_t)(cd_offset>>8);
  eocd[18]=(uint8_t)(cd_offset>>16);  eocd[19]=(uint8_t)(cd_offset>>24);
  current_source_->sendRaw(eocd, sizeof(eocd));

  DBG_PRINTF("[cmd] get-sd-archive %u files %lu bytes\n",
             (unsigned)entry_count, (unsigned long)total_size);
}
