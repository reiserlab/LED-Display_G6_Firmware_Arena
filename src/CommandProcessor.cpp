#include "CommandProcessor.h"

using namespace AC;
using namespace AC::constants;

void CommandProcessor::begin() {
  // No-op; state is default-initialized.
}

// ---------------------------------------------------------------------------
// Command dispatch
// ---------------------------------------------------------------------------

void CommandProcessor::processCommand() {
  if (!net_.hasCommand()) return;

  const ParsedCommand &cmd = net_.command();
  if (cmd.is_stream) {
    handleStreamCommand(cmd);
  } else {
    handleBinaryCommand(cmd);
  }
  net_.commandConsumed();
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
      net_.sendResponse(command_byte, 0, "All-Off Received");
      break;

    case ALL_ON_CMD:
      enterAllOn();
      net_.sendResponse(command_byte, 0, "All-On Received");
      break;

    case STOP_DISPLAY_CMD:
      enterAllOff();
      net_.sendResponse(command_byte, 0, "Display has been stopped");
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
      net_.sendResponse(command_byte, 0, "");
      break;
    }

    case GET_ETHERNET_IP_ADDRESS_CMD:
      net_.sendResponse(command_byte, 0, net_.ipAddress());
      break;

    // G6-dropped commands. Echo them with an explanatory message so a legacy
    // G4 host gets a clear signal rather than silent failure.
    case DISPLAY_RESET_CMD:
      net_.sendResponse(command_byte, 1, "DISPLAY_RESET dropped for G6");
      break;
    case SWITCH_GRAYSCALE_CMD:
      net_.sendResponse(command_byte, 1,
                        "SWITCH_GRAYSCALE dropped for G6; mode inferred from stream size");
      break;

    // Unsupported in this Mode-5-only build.
    case TRIAL_PARAMS_CMD:
      net_.sendResponse(command_byte, 1,
                        "TRIAL_PARAMS not supported (Modes 2/3/4 out of scope)");
      break;
    case SET_FRAME_POSITION_CMD:
      net_.sendResponse(command_byte, 1,
                        "SET_FRAME_POSITION not supported (Modes 2/3 out of scope)");
      break;

    default:
      net_.sendResponse(command_byte, 1, "Unknown command");
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
    net_.sendResponse(STREAM_FRAME_CMD, 1, "Bad stream-frame size");
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
    uint32_t want = (block_size == G6::block_byte_count_gs2)
                        ? refresh_rate_gs2_default
                        : refresh_rate_gs16_default;
    if (want != refresh_rate_hz_) {
      refresh_rate_hz_ = want;
      need_rearm = true;
    }
  }

  if (need_rearm) {
    enterStreamingFrame(block_size);
  }

  net_.sendResponse(STREAM_FRAME_CMD, 0, "");
  DBG_PRINTF("[stream] bytes=%lu block=%u refresh=%lu Hz dt=%lu us\n",
             (unsigned long)frame_byte_count, (unsigned)block_size,
             (unsigned long)refresh_rate_hz_, (unsigned long)(micros() - t0));
}

// ---------------------------------------------------------------------------
// Display service — called every loop iteration
// ---------------------------------------------------------------------------

void CommandProcessor::serviceDisplay() {
  switch (state_) {
    case ArenaState::ALL_OFF:
      break;

    case ArenaState::ALL_ON:
    case ArenaState::STREAMING_FRAME:
      if (spi_.refreshFlag) {
        spi_.refreshFlag = false;
        spi_.transferFrame(frame_buf_, block_byte_count_);
#ifdef DEBUG_SERIAL
        static uint32_t tick = 0;
        if ((++tick % 300) == 0) {
          DBG_PRINTF("[cmd] display tick %lu (state=%u)\n",
                     (unsigned long)tick, (unsigned)state_);
        }
#endif
      }
      break;
  }
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
  // Dump leading 8 bytes of the first panel block (offset = 4-byte prefix).
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

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

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
