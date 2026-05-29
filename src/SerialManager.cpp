#include "SerialManager.h"
#include "commands.h"

void SerialManager::begin() {
  // Teensy 4's USB CDC port. The baud rate is informational on USB CDC
  // (real transfers happen at USB bulk rates regardless), but the framework
  // requires a `Serial.begin()` call before reads/writes are honored.
  // Calling this in DEBUG_SERIAL builds where main.cpp also calls
  // Serial.begin() is harmless — Teensy core treats it as idempotent.
  Serial.begin(115200);
}

void SerialManager::serviceUsb() {
  if (cmd_ready_) return;  // previous command not consumed yet

  int avail = Serial.available();
  if (avail > 0) {
    size_t to_read = min((size_t)avail, RX_BUF_SIZE - rx_len_);
    if (to_read > 0) {
      int n = Serial.readBytes(reinterpret_cast<char *>(rx_buf_ + rx_len_), to_read);
      if (n > 0) rx_len_ += n;
    }
  }

  if (rx_len_ == 0) return;
  parseIncoming();
}

void SerialManager::parseIncoming() {
  // Same framing as NetworkManager::parseIncoming():
  //   binary: [length, cmd, params...]      — length = remaining bytes
  //   stream: [0x32, len_lo, len_hi, ...]   — 3-byte stream header (G6)
  uint8_t first_byte = rx_buf_[0];

  if (first_byte == AC::STREAM_FRAME_CMD) {
    if (rx_len_ < AC::constants::stream_header_byte_count) return;

    uint16_t claimed_len;
    memcpy(&claimed_len, rx_buf_ + 1, sizeof(claimed_len));
    uint32_t total_needed
        = (uint32_t)AC::constants::stream_header_byte_count + claimed_len;
    if (total_needed > RX_BUF_SIZE) {
      rx_len_ = 0;  // oversized — resync
      return;
    }
    if (rx_len_ < total_needed) return;

    parsed_cmd_.cmd       = AC::STREAM_FRAME_CMD;
    parsed_cmd_.is_stream = true;
    parsed_cmd_.data_len  = (uint16_t)total_needed;
    memcpy(parsed_cmd_.data, rx_buf_, total_needed);
    cmd_ready_ = true;

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

    if (total_needed < rx_len_) {
      memmove(rx_buf_, rx_buf_ + total_needed, rx_len_ - total_needed);
    }
    rx_len_ -= total_needed;
  } else {
    // Unknown leading byte — discard rx buffer to resync.
    rx_len_ = 0;
  }
}

void SerialManager::flushResponses() {
  if (resp_len_ == 0) return;

  // Avoid blocking if the host isn't reading: only write when the USB CDC
  // TX buffer has room. If it doesn't, leave the response queued — the next
  // flushResponses() call will retry. (If the host never reads, the
  // response simply stays buffered. Harmless.)
  if (Serial.availableForWrite() < (int)resp_len_) return;

  Serial.write(resp_buf_, resp_len_);
  Serial.flush();
  resp_len_ = 0;
}

void SerialManager::sendResponse(uint8_t cmd_echo, uint8_t status,
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
  resp_buf_[0] = response_byte_count - 1;
  resp_len_ = response_byte_count;
}
