#include "CommandProcessor.h"
#include "ErrorGlyph.h"

using namespace AC;
using namespace AC::constants;

void CommandProcessor::begin() {
  // No-op; state is default-initialized. SD is mounted by main() via sd_.begin().
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
    if (cmd.is_stream) {
      handleStreamCommand(cmd);
    } else {
      handleBinaryCommand(cmd);
    }
    net_.commandConsumed();
  }
  if (serial_.hasCommand()) {
    current_source_ = &serial_;
    const ParsedCommand &cmd = serial_.command();
    if (cmd.is_stream) {
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

    case GET_ETHERNET_IP_ADDRESS_CMD:
      current_source_->sendResponse(command_byte, 0, net_.ipAddress());
      break;

    case GET_CONTROLLER_INFO_CMD:
      handleGetControllerInfo();
      break;

    case TRIAL_PARAMS_CMD:
      handleTrialParams(cmd);
      break;

    case SET_FRAME_POSITION_CMD:
      handleSetFramePosition(cmd);
      break;

    case DISPLAY_PSRAM_INDEX_CMD:
      handleDisplayPsramIndex(cmd);
      break;

    case PSRAM_PLAY_CMD:
      handlePsramPlay(cmd);
      break;

    // G6-dropped commands. Echo them with an explanatory message so a legacy
    // G4 host gets a clear signal rather than silent failure.
    case DISPLAY_RESET_CMD:
      current_source_->sendResponse(command_byte, 1, "DISPLAY_RESET dropped for G6");
      break;
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
// get-controller-info (0x67)
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
//   param[3:4] frame_rate  uint16 LE (Hz; Mode 2 frame-advance rate)
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
  uint16_t frame_rate = (uint16_t)p[3] | ((uint16_t)p[4] << 8);
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
// V2 PSRAM display (0x71 single index / 0x72 auto-advance play) — LAB-41/42.
//
// The panel holds the frames in its own PSRAM (loaded locally for the demo).
// We synthesize a frame of identical V2 "display PSRAM index N" blocks (one per
// panel, all the same index → whole arena shows frame N) and let the refresh
// timer retransmit it. 0x72 advances the index at frame_rate_hz_, exactly like
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
  psram_cmd_id_      = G6::cmd_disp_psram_persist;
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
  psram_cmd_id_      = G6::cmd_disp_psram_persist;
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
  uint32_t period_us = microseconds_per_second / frame_rate_hz_;
  uint32_t now = micros();
  if ((now - last_advance_us_) < period_us) return;

  // Advance by however many whole periods have elapsed (catch up if the loop
  // was busy), then load the new frame once.
  uint16_t steps = 0;
  while ((now - last_advance_us_) >= period_us) {
    last_advance_us_ += period_us;
    if (++steps >= frame_count_) { steps = frame_count_; break; }  // clamp
  }
  cur_frame_index_ = (uint16_t)((cur_frame_index_ + steps) % frame_count_);
  loadFrame(cur_frame_index_);
}

void CommandProcessor::servicePsramPlay() {
  if (frame_rate_hz_ == 0 || psram_play_count_ <= 1) return;  // static index
  uint32_t period_us = microseconds_per_second / frame_rate_hz_;
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
  // 0 V (g6_07). Front-end offset/scale is hardware calibration — flagged as
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
  return true;
}

// ---------------------------------------------------------------------------
// State transitions
// ---------------------------------------------------------------------------

void CommandProcessor::enterAllOff() {
  spi_.disarmRefreshTimer();
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
                                        uint16_t frame_rate_hz, int8_t gain,
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

uint32_t CommandProcessor::defaultRefreshFor(uint16_t block_byte_count) const {
  return (block_byte_count == G6::block_byte_count_gs2)
             ? refresh_rate_gs2_default
             : refresh_rate_gs16_default;
}

void CommandProcessor::fillFrameBufferAllOn(uint16_t block_byte_count) {
  // Synthesize a frame of GS16 oneshot blocks with all-max pixels.
  // Block layout: [header][cmd=0x30][200 pixel bytes = 0xFF][duty_cycle=0xFF].
  // Parity is recomputed per block.
  memset(frame_buf_, 0, sizeof(frame_buf_));

  // Frame prefix: "FR" + frame index 0 — informational only, matches the
  // stream payload layout the SPI dispatcher expects.
  frame_buf_[0] = 'F';
  frame_buf_[1] = 'R';
  frame_buf_[2] = 0;
  frame_buf_[3] = 0;

  uint8_t cmd = (block_byte_count == G6::block_byte_count_gs2)
                    ? G6::cmd_disp_2lvl_oneshot
                    : G6::cmd_disp_16lvl_oneshot;

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
