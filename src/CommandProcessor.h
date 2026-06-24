#pragma once

#include "NetworkManager.h"
#include "SerialManager.h"
#include "SpiManager.h"
#include "SdManager.h"
#include "G6PanelProtocol.h"
#include "commands.h"

enum class ArenaState : uint8_t {
  ALL_OFF,
  ALL_ON,
  STREAMING_FRAME,  // Mode 5 — host streams raw frames
  OPEN_LOOP,        // Mode 2 — auto-advance frames from SD at frame_rate
  SHOW_FRAME,       // Mode 3 — host-commanded frame index
  CLOSED_LOOP,      // Mode 4 — AIN0 velocity integration
  PSRAM_PLAY,       // V2 — show/auto-advance panel-resident PSRAM frames (LAB-41/42)
  ERROR_DISPLAY,    // transient "CE / NN" diagnostic glyph
};

class CommandProcessor {
 public:
  CommandProcessor(NetworkManager &net, SerialManager &serial, SpiManager &spi,
                   SdManager &sd)
      : net_(net), serial_(serial), spi_(spi), sd_(sd) {}

  void begin();
  void processCommand();
  void serviceDisplay();

 private:
  NetworkManager &net_;
  SerialManager  &serial_;
  SpiManager     &spi_;
  SdManager      &sd_;

  // Set by processCommand() to point at whichever MessageSource (net_ or
  // serial_) originated the command being handled. Handlers send their
  // response back via this pointer so it lands on the right transport.
  MessageSource *current_source_ = nullptr;

  ArenaState state_ = ArenaState::ALL_OFF;

  // GS mode + refresh tracking.
  uint16_t block_byte_count_ = G6::block_byte_count_gs16;
  uint32_t refresh_rate_hz_  = AC::constants::refresh_rate_gs16_default;
  bool     refresh_rate_explicit_ = false;  // host set via SET_REFRESH_RATE

  // Frame buffer holds the current frame (streamed, SD-loaded, all-on, or
  // error glyph) including the 4-byte prefix the SPI dispatcher skips.
  uint8_t frame_buf_[AC::constants::frame_buf_byte_count_max];
  uint16_t frame_byte_count_ = 0;

  // Pattern playback state (Modes 2/3/4).
  uint16_t pattern_id_      = 0;   // 1-based; 0 = none open
  uint16_t frame_count_     = 0;   // frames in the open pattern
  uint16_t cur_frame_index_ = 0;   // 0-based
  uint16_t frame_rate_hz_   = 0;   // frame-advance rate (Mode 2 base)
  int8_t   gain_            = 0;   // Mode 4 velocity scaling (10x fps/V)
  uint32_t last_advance_us_ = 0;   // Mode 2 frame-advance clock
  uint32_t last_sample_us_  = 0;   // Mode 4 AIN sample clock
  float    frame_accum_     = 0.0f;// Mode 4 fractional-frame accumulator

  // Analog output — last commanded level (mV). 0 = DAC code 0 (power-up default).
  uint16_t ao_mv_ = 0;

  // Error display.
  uint32_t error_until_ms_ = 0;

  // V2 PSRAM playback (LAB-41/42). The arena streams a tiny V2 "display PSRAM
  // index" command per frame; the panel renders its locally-stored frame.
  // count==1 => static single index; count>1 + frame_rate_hz_>0 => auto-advance
  // [start, start+count) (reuses frame_rate_hz_ / last_advance_us_).
  uint16_t psram_start_index_ = 0;
  uint16_t psram_play_count_  = 1;
  uint16_t psram_play_offset_ = 0;   // 0..count-1
  uint8_t  psram_cmd_id_      = G6::cmd_disp_psram_persist;

  // Handlers.
  void handleBinaryCommand(const ParsedCommand &cmd);
  void handleStreamCommand(const ParsedCommand &cmd);
  void handleTrialParams(const ParsedCommand &cmd);
  void handleSetFramePosition(const ParsedCommand &cmd);
  void handleGetControllerInfo();
  void handleDisplayPsramIndex(const ParsedCommand &cmd);
  void handlePsramPlay(const ParsedCommand &cmd);
  void handleBulkWriteCommand(const ParsedCommand &cmd);
  void handleGetSdArchive();
  void drainBulkData(uint32_t remaining_bytes);

  // State transitions.
  void enterAllOff();
  void enterAllOn();
  void enterStreamingFrame(uint16_t block_byte_count);
  bool enterPatternMode(ArenaState mode, uint16_t pattern_id,
                        uint16_t frame_rate_hz, int8_t gain,
                        uint16_t init_frame);
  void showError(uint8_t code);

  // Per-mode service helpers.
  void transmitOnRefresh();
  void serviceOpenLoop();
  void serviceClosedLoop();
  void servicePsramPlay();               // V2 auto-advance (LAB-41/42)
  bool loadFrame(uint16_t frame_index);  // false on SD/CRC error (shows glyph)

  // Helpers.
  void fillFrameBufferAllOn(uint16_t block_byte_count);
  void buildPsramFrame(uint16_t index);  // fill frame_buf_ with V2 index blocks
  uint32_t defaultRefreshFor(uint16_t block_byte_count) const;
};
