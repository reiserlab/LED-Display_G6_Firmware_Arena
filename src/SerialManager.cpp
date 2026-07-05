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

    parsed_cmd_.cmd              = AC::STREAM_FRAME_CMD;
    parsed_cmd_.is_stream        = true;
    parsed_cmd_.is_bulk          = false;
    parsed_cmd_.bulk_payload_len = 0;
    parsed_cmd_.data_len         = (uint16_t)total_needed;
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

    parsed_cmd_.is_stream        = false;
    parsed_cmd_.is_bulk          = false;
    parsed_cmd_.bulk_payload_len = 0;
    parsed_cmd_.data_len         = total_needed;
    memcpy(parsed_cmd_.data, rx_buf_, total_needed);
    parsed_cmd_.cmd = rx_buf_[1];
    cmd_ready_ = true;

    if (total_needed < rx_len_) {
      memmove(rx_buf_, rx_buf_ + total_needed, rx_len_ - total_needed);
    }
    rx_len_ -= total_needed;
  } else if (first_byte == AC::SET_PATTERN_FILE_CMD) {
    // [0x85, idx_lo, idx_hi, len_b0..b7, file_data...]
    // Need the 11-byte header before signaling cmd_ready; file data
    // stays in rx_buf_ for readBulkBytes() to drain.
    static constexpr size_t BULK_HDR = 11;
    if (rx_len_ < BULK_HDR) return;

    uint16_t idx;
    memcpy(&idx, rx_buf_ + 1, sizeof(idx));
    uint32_t len_lo;
    memcpy(&len_lo, rx_buf_ + 3, sizeof(len_lo));  // lower 32 bits; upper 32 ignored

    parsed_cmd_.cmd              = AC::SET_PATTERN_FILE_CMD;
    parsed_cmd_.is_stream        = false;
    parsed_cmd_.is_bulk          = true;
    parsed_cmd_.bulk_payload_len = len_lo;
    parsed_cmd_.data_len         = (uint16_t)BULK_HDR;
    memcpy(parsed_cmd_.data, rx_buf_, BULK_HDR);

    memmove(rx_buf_, rx_buf_ + BULK_HDR, rx_len_ - BULK_HDR);
    rx_len_ -= BULK_HDR;
    cmd_ready_ = true;
  } else if (first_byte == AC::SET_FIRMWARE_FILE_CMD) {
    // [0xE0, len_b0..b7, file_data...] — single firmware image, no index.
    // Need the 9-byte header before signaling cmd_ready; file data stays in
    // rx_buf_ for readBulkBytes() to drain.
    static constexpr size_t FW_BULK_HDR = 9;
    if (rx_len_ < FW_BULK_HDR) return;

    uint32_t len_lo;
    memcpy(&len_lo, rx_buf_ + 1, sizeof(len_lo));  // lower 32 bits; upper 32 ignored

    parsed_cmd_.cmd              = AC::SET_FIRMWARE_FILE_CMD;
    parsed_cmd_.is_stream        = false;
    parsed_cmd_.is_bulk          = true;
    parsed_cmd_.bulk_payload_len = len_lo;
    parsed_cmd_.data_len         = (uint16_t)FW_BULK_HDR;
    memcpy(parsed_cmd_.data, rx_buf_, FW_BULK_HDR);

    memmove(rx_buf_, rx_buf_ + FW_BULK_HDR, rx_len_ - FW_BULK_HDR);
    rx_len_ -= FW_BULK_HDR;
    cmd_ready_ = true;
  } else if (first_byte == AC::SET_PATTERN_FILENAME_CMD) {
    // Opcode-first framing: [0x83, idx_lo, idx_hi, name_len, chars…]
    // The standard binary framing can't carry this command for filenames > 45 chars
    // because the length byte would equal or exceed STREAM_FRAME_CMD (0x32).
    static constexpr uint8_t LONG_HDR = 4;  // opcode + 2 idx + name_len
    if (rx_len_ < LONG_HDR) return;

    uint8_t name_len = rx_buf_[3];
    if (name_len == 0 || name_len >= AC::constants::pattern_name_byte_count) {
      rx_len_ = 0;  // bad name_len — resync
      return;
    }
    uint16_t total_needed = LONG_HDR + name_len;
    if (rx_len_ < total_needed) return;

    // Re-pack into binary-frame layout so handleBinaryCommand works unchanged:
    // data[0] = total_needed (the binary length byte), then the opcode-first bytes.
    parsed_cmd_.cmd              = AC::SET_PATTERN_FILENAME_CMD;
    parsed_cmd_.is_stream        = false;
    parsed_cmd_.is_bulk          = false;
    parsed_cmd_.bulk_payload_len = 0;
    parsed_cmd_.data_len         = (uint16_t)(total_needed + 1);
    parsed_cmd_.data[0]          = (uint8_t)total_needed;
    memcpy(parsed_cmd_.data + 1, rx_buf_, total_needed);
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

size_t SerialManager::readBulkBytes(uint8_t* buf, size_t max_len) {
  // Top up rx_buf_ from USB CDC.
  int avail = Serial.available();
  if (avail > 0) {
    size_t space = RX_BUF_SIZE - rx_len_;
    size_t to_read = min((size_t)avail, space);
    if (to_read > 0) {
      int n = Serial.readBytes(reinterpret_cast<char *>(rx_buf_ + rx_len_), to_read);
      if (n > 0) rx_len_ += (size_t)n;
    }
  }
  size_t n = min(rx_len_, max_len);
  if (n > 0) {
    memcpy(buf, rx_buf_, n);
    if (n < rx_len_) memmove(rx_buf_, rx_buf_ + n, rx_len_ - n);
    rx_len_ -= n;
  }
  return n;
}

size_t SerialManager::sendRaw(const uint8_t* buf, size_t len) {
  // Flush any queued response frame (e.g. the 0x84 size header) first.
  // Spin-wait up to 5 s in case the TX buffer is momentarily full.
  uint32_t deadline = millis() + 5000UL;
  while (resp_len_ > 0 && millis() < deadline) flushResponses();
  if (!buf || len == 0) return 0;
  // Pace the bulk body to the USB-CDC TX FIFO. A single Serial.write() larger than
  // availableForWrite() blocks inside loop() until the host drains, which starves the
  // USB service and can trip the CDC transmit watchdog on big files (the host then sees
  // a "Break"). Chunk to the free FIFO space and yield() between writes, mirroring the
  // framed flushResponses() discipline.
  //
  // The room<=0 spin below has its own short stall deadline, reset on every
  // successful partial write. If the host stops draining entirely (a stalled
  // reader, not just a slow one), this returns short instead of spinning
  // forever — issue #16: the old unbounded spin here is what let a stalled
  // host wedge sendRaw (and therefore all of loop()) indefinitely, which
  // macOS's CDC-ACM driver would eventually report to the host as a "Break".
  //
  // 2000 ms, not the notes' original ~1-2 s lower end: on-hardware testing
  // (Linux host, native Teensy 4.1 USB) showed a genuinely slow-but-still-
  // draining host can leave availableForWrite() at 0 for a bit over 1.5 s at
  // a stretch even while the host reads every ~20 ms — likely USB/host-driver
  // buffer-release latency, not an application-level gap. 1500 ms produced
  // false aborts on a real, continuously-progressing transfer; 2000-5000 ms
  // all worked reliably, so 2000 ms was chosen for the smallest margin that
  // stopped false-triggering in repeated on-hardware runs.
  static constexpr uint32_t kStallTimeoutMs = 2000UL;
  size_t off = 0;
  uint32_t stall_deadline = millis() + kStallTimeoutMs;
  while (off < len) {
    int room = Serial.availableForWrite();
    if (room <= 0) {
      if (millis() > stall_deadline) break;
      yield();
      continue;
    }
    size_t n = ((size_t)room < (len - off)) ? (size_t)room : (len - off);
    Serial.write(buf + off, n);
    off += n;
    stall_deadline = millis() + kStallTimeoutMs;
  }
  return off;
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
