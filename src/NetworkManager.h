#pragma once

#include <Arduino.h>
#include <QNEthernet.h>
#include "constants.h"

using namespace qindesign::network;

// Parsed command from the G4-compatible binary protocol.
struct ParsedCommand {
  uint8_t  cmd;
  uint8_t  data[AC::constants::stream_header_byte_count
                + AC::constants::frame_buf_byte_count_max + 16];
  uint16_t data_len;   // total bytes received (including length / stream header)
  bool     is_stream;
};

class NetworkManager {
 public:
  void begin();

  void serviceTcp();
  void flushResponses();

  bool hasCommand() const { return cmd_ready_; }
  const ParsedCommand &command() const { return parsed_cmd_; }
  void commandConsumed() { cmd_ready_ = false; }

  // Build and queue a binary response: [len, status, echo_cmd, ...message].
  void sendResponse(uint8_t cmd_echo, uint8_t status, const char *message);

  const char *ipAddress() const { return ip_str_; }
  const char *macAddress() const { return mac_str_; }

 private:
  EthernetServer server_{AC::constants::ethernet_server_port};
  EthernetClient client_;

  // Receive buffer sized for the largest GS16 stream frame plus headroom.
  static constexpr size_t RX_BUF_SIZE
      = AC::constants::stream_header_byte_count
        + AC::constants::frame_buf_byte_count_max + 16;
  uint8_t rx_buf_[RX_BUF_SIZE];
  size_t  rx_len_ = 0;

  ParsedCommand parsed_cmd_;
  bool cmd_ready_ = false;

  // Response buffer.
  static constexpr size_t RESP_BUF_SIZE = AC::constants::byte_count_per_response_max;
  uint8_t resp_buf_[RESP_BUF_SIZE];
  size_t  resp_len_ = 0;

  char ip_str_[32] = "";
  char mac_str_[18] = "";  // "XX:XX:XX:XX:XX:XX"

  void parseIncoming();
};
