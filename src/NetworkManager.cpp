#include "NetworkManager.h"
#include "commands.h"

void NetworkManager::begin() {
  Ethernet.begin();  // DHCP
  server_.begin();
}

void NetworkManager::serviceTcp() {
  if (!client_ || !client_.connected()) {
    rx_len_ = 0;
    cmd_ready_ = false;
    EthernetClient newClient = server_.accept();
    if (newClient) {
      client_ = newClient;
      client_.setNoDelay(true);
      IPAddress peer = client_.remoteIP();
      DBG_PRINTF("[net] client connected from %u.%u.%u.%u\n",
                 peer[0], peer[1], peer[2], peer[3]);
    }
  }

  // Cache IP and MAC once available.
  if (ip_str_[0] == '\0') {
    IPAddress ip = Ethernet.localIP();
    if (ip != IPAddress{0, 0, 0, 0}) {
      snprintf(ip_str_, sizeof(ip_str_), "%u.%u.%u.%u",
               ip[0], ip[1], ip[2], ip[3]);

      uint8_t mac[6];
      Ethernet.macAddress(mac);
      snprintf(mac_str_, sizeof(mac_str_), "%02X:%02X:%02X:%02X:%02X:%02X",
               mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
  }

  parseIncoming();
}

void NetworkManager::parseIncoming() {
  if (!client_ || !client_.connected()) return;
  if (cmd_ready_) return;  // previous command not consumed yet

  // Bulk-read available bytes into rx_buf_.
  int avail = client_.available();
  if (avail > 0) {
    size_t to_read = min((size_t)avail, RX_BUF_SIZE - rx_len_);
    int n = client_.read(rx_buf_ + rx_len_, to_read);
    if (n > 0) rx_len_ += n;
  }

  if (rx_len_ == 0) return;

  // G4-style protocol with G6 stream framing:
  //   Binary form: [len, cmd, params...]      — len = remaining bytes
  //   Stream form: [0x32, len_lo, len_hi, ...] — 3-byte stream header, no
  //                                              analog_x/analog_y (G6 drop)
  uint8_t first_byte = rx_buf_[0];

  if (first_byte == AC::STREAM_FRAME_CMD) {
    if (rx_len_ < AC::constants::stream_header_byte_count) return;

    uint16_t claimed_len;
    memcpy(&claimed_len, rx_buf_ + 1, sizeof(claimed_len));
    uint32_t total_needed
        = (uint32_t)AC::constants::stream_header_byte_count + claimed_len;
    if (total_needed > RX_BUF_SIZE) {
      // Oversized — drop the buffer and resync.
      rx_len_ = 0;
      return;
    }
    if (rx_len_ < total_needed) return;  // wait for more data

    parsed_cmd_.cmd       = AC::STREAM_FRAME_CMD;
    parsed_cmd_.is_stream = true;
    parsed_cmd_.data_len  = (uint16_t)total_needed;
    memcpy(parsed_cmd_.data, rx_buf_, total_needed);
    cmd_ready_ = true;
    DBG_PRINTF("[net] parsed stream data_len=%lu frame=%u\n",
               (unsigned long)total_needed, (unsigned)claimed_len);

    size_t consumed = total_needed;
    if (consumed < rx_len_) {
      memmove(rx_buf_, rx_buf_ + consumed, rx_len_ - consumed);
    }
    rx_len_ -= consumed;
  } else if (first_byte <= AC::constants::first_command_byte_max_value_binary) {
    uint8_t remaining_len = first_byte;
    uint16_t total_needed = 1 + remaining_len;
    if (rx_len_ < total_needed) return;

    parsed_cmd_.is_stream = false;
    parsed_cmd_.data_len  = total_needed;
    memcpy(parsed_cmd_.data, rx_buf_, total_needed);
    parsed_cmd_.cmd = rx_buf_[1];
    cmd_ready_ = true;
    DBG_PRINTF("[net] parsed binary cmd=0x%02X len=%u\n",
               parsed_cmd_.cmd, (unsigned)total_needed);

    if (total_needed < rx_len_) {
      memmove(rx_buf_, rx_buf_ + total_needed, rx_len_ - total_needed);
    }
    rx_len_ -= total_needed;
  } else {
    // Unknown leading byte — discard buffered bytes to resync.
    rx_len_ = 0;
  }
}

void NetworkManager::flushResponses() {
  if (!client_ || !client_.connected()) return;
  if (resp_len_ == 0) return;

  client_.write(resp_buf_, resp_len_);
  client_.flush();
  resp_len_ = 0;
}

void NetworkManager::sendResponse(uint8_t cmd_echo, uint8_t status,
                                  const uint8_t *payload, size_t payload_len) {
  uint8_t response_byte_count = 0;
  resp_buf_[response_byte_count++] = 2;  // placeholder length
  resp_buf_[response_byte_count++] = status;
  resp_buf_[response_byte_count++] = cmd_echo;
  if (payload != nullptr && payload_len > 0 &&
      (response_byte_count + payload_len) < RESP_BUF_SIZE) {
    memcpy(resp_buf_ + response_byte_count, payload, payload_len);
    response_byte_count += payload_len;
  }
  resp_buf_[0] = response_byte_count - 1;  // length excluding length byte
  resp_len_ = response_byte_count;
}
