#pragma once

#include "NetworkManager.h"
#include "SerialManager.h"
#include "SpiManager.h"
#include "SdManager.h"
#include "IspController.h"
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

  // SPI in-system-programming driver for g6-program-panel (0xC8).
  IspController   isp_{spi_};

  // Set by processCommand() to point at whichever MessageSource (net_ or
  // serial_) originated the command being handled. Handlers send their
  // response back via this pointer so it lands on the right transport.
  MessageSource *current_source_ = nullptr;

  ArenaState state_ = ArenaState::ALL_OFF;

  // GS mode + refresh tracking.
  uint16_t block_byte_count_ = G6::block_byte_count_gs16;
  uint32_t refresh_rate_hz_  = AC::constants::refresh_rate_gs16_default;
  bool     refresh_rate_explicit_ = false;  // host set via SET_REFRESH_RATE

  // Panel display mode: 0=oneshot, 1=persist (default), 2=triggered, 3=gated.
  // Governs the DISP_* opcode stamped into every panel block the controller
  // synthesises or patches (SD frames, streamed frames, ALL_ON).
  // Error-glyph frames are exempt — they always use the opcode from buildErrorFrame.
  uint8_t panel_disp_mode_ = 1;

  // Frame buffer holds the current frame (streamed, SD-loaded, all-on, or
  // error glyph) including the 4-byte prefix the SPI dispatcher skips.
  uint8_t frame_buf_[AC::constants::frame_buf_byte_count_max];
  uint16_t frame_byte_count_ = 0;

  // Pattern playback state (Modes 2/3/4).
  uint16_t pattern_id_      = 0;   // 1-based; 0 = none open
  uint16_t frame_count_     = 0;   // frames in the open pattern
  uint16_t cur_frame_index_ = 0;   // 0-based
  int16_t  frame_rate_hz_   = 0;   // frame-advance rate (Mode 2); negative = reverse
  int16_t  gain_            = 0;   // Mode 4 velocity scaling (10x fps/V)
  uint32_t last_advance_us_ = 0;   // Mode 2 frame-advance clock
  uint32_t last_sample_us_  = 0;   // Mode 4 AIN sample clock
  float    frame_accum_     = 0.0f;// Mode 4 fractional-frame accumulator
  uint32_t trial_end_ms_    = 0;   // trial_params (0x08) Duration auto-stop deadline; 0 = not armed

  // Digital IO roles (#135, SET_DIO_ROLE 0xAC). Ports are 1-based on the wire
  // (== the board's "Digital IO 1/2 (5V)" BNC silkscreen == 0xAA channel);
  // dio_role_[0] is port 1. Boot defaults (begin()): port 1 = out_programmable
  // driving LOW (historic bench behavior), port 2 = in_trigger — the external
  // trigger route to the panels' EINT net (U3 B→A + J30 shunt), which also
  // fixes the old boot contention where D35 stayed an output driving the EINT
  // net. SET_DIGITAL_OUT auto-promotes an `off` port to out_programmable but
  // REFUSES ports explicitly configured as in_trigger / out_debug_framescan.
  enum class DioRole : uint8_t {
    kOff = 0,             // translator B→A, our data pin hi-Z; BNC ignored
    kInTrigger = 1,       // pin config as kOff; on port 2 the BNC feeds EINT
    kOutProgrammable = 2, // translator A→B, host drives the BNC via 0xAA
    kOutFramescan = 3     // as output, gated by SpiManager per frame transfer
  };
  DioRole dio_role_[2] = { DioRole::kOff, DioRole::kOff };  // set in begin()

  // Analog output — last commanded level (mV). 0 = DAC code 0 (power-up default).
  uint16_t ao_mv_ = 0;

  // AO mode (#135, SET_AO_MODE 0xA3). 0 = programmable (0xA0 levels + 0xA2
  // LUT — today's behavior); 1 = frame_number: the DAC tracks the SD-pattern
  // frame index, normalized 0 V = frame 0 .. 5 V = last frame (updated in
  // loadFrame, Modes 2/3/4). 0xA0/0xA2 are refused while in frame_number.
  uint8_t ao_mode_ = 0;

  // AO LUT playback (LAB-82). mode 0 = frame-locked (advances with cur_frame_index_);
  // mode 1 = time-based (steps at ao_lut_step_hz_, max 1000 Hz).
  // ao_lut_len_ == 0 means the LUT is inactive. Send SET_AO_VOLTAGE (0xA0) to stop.
  static constexpr uint16_t kAoLutMaxLen = 4096;
  uint16_t ao_lut_[kAoLutMaxLen];
  uint16_t ao_lut_len_     = 0;
  uint8_t  ao_lut_mode_    = 0;
  uint16_t ao_lut_step_hz_ = 0;
  uint16_t ao_lut_idx_     = 0;
  uint32_t ao_lut_last_us_ = 0;

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
  void handleSetFirmwareFile(const ParsedCommand &cmd);  // set-firmware-file (0xE0)
  void handleGetFirmwareInfo();                          // get-firmware-info (0xE3)
  void handleProgramPanel(const ParsedCommand &cmd);     // g6-program-panel (0xC8) — SPI ISP
  void handleVerifyPanel(const ParsedCommand &cmd);      // g6-verify-panel (0xC9) — CRC running app flash
  void drainBulkData(uint32_t remaining_bytes);

  // State transitions.
  void enterAllOff();
  void enterAllOn();
  void enterStreamingFrame(uint16_t block_byte_count);
  bool enterPatternMode(ArenaState mode, uint16_t pattern_id,
                        int16_t frame_rate_hz, int16_t gain,
                        uint16_t init_frame, uint16_t duration_ticks);
  void showError(uint8_t code);

  // Per-mode service helpers.
  void transmitOnRefresh();
  void serviceTrialTimer();  // trial_params (0x08) Duration auto-stop
  void serviceOpenLoop();
  void serviceClosedLoop();
  void servicePsramPlay();               // V2 auto-advance (LAB-41/42)
  bool loadFrame(uint16_t frame_index);  // false on SD/CRC error (shows glyph)

  // Helpers.
  void fillFrameBufferAllOn(uint16_t block_byte_count);
  void fillFrameBufferDark();             // all-pixels-off Persistent frame (for all-off/stop)
  void buildPsramFrame(uint16_t index);  // fill frame_buf_ with V2 index blocks
  uint32_t defaultRefreshFor(uint16_t block_byte_count) const;
  bool writeDacMv(uint16_t mv);           // MCP4725 write, 0-5000 mV; false = I²C error
  bool applyAoLut(uint16_t idx);          // write ao_lut_[idx % ao_lut_len_] to DAC; false = I²C error
  void applyDioRole(uint8_t port, DioRole role);  // 1-based port; safe pin-transition ordering
  const char *dioRoleName(DioRole role) const;    // for error payloads / debug prints
  uint8_t dispOpcodeFor(bool gs16) const; // pick DISP_* opcode for panel_disp_mode_ × gs level
  void    patchDispMode();                // rewrite block[1]+parity in every panel block in frame_buf_
};
