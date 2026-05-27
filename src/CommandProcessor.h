#pragma once

#include "NetworkManager.h"
#include "SpiManager.h"
#include "G6PanelProtocol.h"
#include "commands.h"

enum class ArenaState : uint8_t {
  ALL_OFF,
  ALL_ON,
  STREAMING_FRAME,
};

class CommandProcessor {
 public:
  CommandProcessor(NetworkManager &net, SpiManager &spi)
      : net_(net), spi_(spi) {}

  void begin();
  void processCommand();
  void serviceDisplay();

 private:
  NetworkManager &net_;
  SpiManager     &spi_;

  ArenaState state_ = ArenaState::ALL_OFF;

  // GS mode + refresh tracking.
  uint16_t block_byte_count_ = G6::block_byte_count_gs16;
  uint32_t refresh_rate_hz_  = AC::constants::refresh_rate_gs16_default;
  bool     refresh_rate_explicit_ = false;  // host set via SET_REFRESH_RATE

  // Frame buffer holds the streamed payload between transmissions, including
  // the 4-byte prefix the SPI dispatcher skips.
  uint8_t frame_buf_[AC::constants::frame_buf_byte_count_max];
  uint16_t frame_byte_count_ = 0;

  // Handlers.
  void handleBinaryCommand(const ParsedCommand &cmd);
  void handleStreamCommand(const ParsedCommand &cmd);

  // State transitions.
  void enterAllOff();
  void enterAllOn();
  void enterStreamingFrame(uint16_t block_byte_count);

  // Helpers.
  void applyDefaultRefreshRate();
  void fillFrameBufferAllOn(uint16_t block_byte_count);
};
