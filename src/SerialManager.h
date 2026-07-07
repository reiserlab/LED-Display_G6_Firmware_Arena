#pragma once

#include <Arduino.h>
#include "constants.h"
#include "MessageSource.h"
#include "NetworkManager.h"  // for ParsedCommand

// USB-CDC command source. Mirrors NetworkManager's interface but uses the
// Teensy's USB Serial port (`Serial`) instead of TCP. Accepts the same G4-
// compatible binary framing — `[length, cmd, params...]` for short commands
// and `[0x32, len_lo, len_hi, ...]` for stream frames — so a host can drive
// the controller via USB serial (e.g. browser Web Serial, pio device monitor,
// raw `cat` / `printf`).
//
// In DEBUG_SERIAL builds, the same USB CDC link is shared with DBG_PRINTF
// diagnostic output. The Teensy USB CDC pipe is bidirectional and packet-
// based, so binary command bytes from the host and text printf bytes from
// the firmware do not interfere on the wire — but a human-readable terminal
// monitor will show interleaved text and binary, which can look noisy.
class SerialManager : public MessageSource {
 public:
  void begin();
  void serviceUsb();
  void flushResponses();

  bool hasCommand() const { return cmd_ready_; }
  const ParsedCommand &command() const { return parsed_cmd_; }
  void commandConsumed() override { cmd_ready_ = false; }

  using MessageSource::sendResponse;  // keep the char* convenience overload
  void sendResponse(uint8_t cmd_echo, uint8_t status,
                    const uint8_t *payload, size_t payload_len) override;
  size_t readBulkBytes(uint8_t* buf, size_t max_len) override;
  size_t sendRaw(const uint8_t* buf, size_t len) override;

 private:
  static constexpr size_t RX_BUF_SIZE
      = AC::constants::stream_header_byte_count
        + AC::constants::frame_buf_byte_count_max + 16;
  uint8_t rx_buf_[RX_BUF_SIZE];
  size_t  rx_len_ = 0;

  ParsedCommand parsed_cmd_;
  bool cmd_ready_ = false;

  static constexpr size_t RESP_BUF_SIZE = AC::constants::byte_count_per_response_max;
  uint8_t resp_buf_[RESP_BUF_SIZE];
  size_t  resp_len_ = 0;

  void parseIncoming();
};
