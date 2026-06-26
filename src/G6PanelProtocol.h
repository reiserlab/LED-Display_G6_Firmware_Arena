#pragma once
#include <stdint.h>
#include <stddef.h>

// G6 panel protocol v1 wire constants and helpers.
// See: public/docs/development/g6_01-panel-protocol.md
namespace G6 {

// Header byte: bits 0..6 = protocol version, bit 7 = even parity over the
// entire message (version bits + cmd + payload, parity bit excluded).
constexpr uint8_t header_version_v1                  = 0x01;  // parity 0
constexpr uint8_t header_version_v1_with_parity_bit  = 0x81;  // parity 1

// V1 display opcodes — all four panel display modes at both grayscale levels.
// The active mode is selected by SET_PANEL_DISPLAY_MODE (0x1B); default = persist.
constexpr uint8_t cmd_disp_2lvl_oneshot    = 0x10;
constexpr uint8_t cmd_disp_2lvl_persist    = 0x11;
constexpr uint8_t cmd_disp_2lvl_triggered  = 0x12;
constexpr uint8_t cmd_disp_2lvl_gated      = 0x13;
constexpr uint8_t cmd_disp_16lvl_oneshot   = 0x30;
constexpr uint8_t cmd_disp_16lvl_persist   = 0x31;
constexpr uint8_t cmd_disp_16lvl_triggered = 0x32;
constexpr uint8_t cmd_disp_16lvl_gated     = 0x33;

// Block sizes (header + cmd + pixel data + duty_cycle).
constexpr uint16_t block_byte_count_gs2  = 53;
constexpr uint16_t block_byte_count_gs16 = 203;

// ---- V2 (header 0x02/0x82) — PSRAM-backed display (LAB-41/42) ----
// The controller sends a 16-bit LE PSRAM frame index; the panel renders its
// locally-stored frame at that index. Low nibble = display mode (0 Oneshot /
// 1 Persistent / 2 Triggered / 3 Gated), same encoding as v1. The 0x6x set
// appends an explicit duty_cycle byte; 0x5x uses the duty stored with the frame.
constexpr uint8_t header_version_v2                 = 0x02;  // parity 0
constexpr uint8_t header_version_v2_with_parity_bit = 0x82;  // parity 1

constexpr uint8_t cmd_disp_psram_oneshot   = 0x50;
constexpr uint8_t cmd_disp_psram_persist   = 0x51;
constexpr uint8_t cmd_disp_psram_triggered = 0x52;
constexpr uint8_t cmd_disp_psram_gated     = 0x53;
constexpr uint8_t cmd_disp_psram_duty_oneshot = 0x60;  // +duty; 0x61..0x63 follow modes

// V2 block sizes: header + cmd + 16-bit LE index [+ 1 duty byte].
constexpr uint16_t block_byte_count_psram      = 4;
constexpr uint16_t block_byte_count_psram_duty = 5;
// build_psram_index_block() is defined below, after stamp_header_parity().

// Compute the parity bit (0 or 1) over a candidate message. The header byte
// is passed *without* its own parity bit (i.e. version bits only); the parity
// is set so the popcount across {version_bits, cmd, payload} is even.
inline uint8_t compute_parity_bit(uint8_t version_byte,
                                  uint8_t cmd,
                                  const uint8_t *payload,
                                  size_t payload_len) {
  uint8_t version_bits = version_byte & 0x7F;
  int ones = __builtin_popcount(version_bits) + __builtin_popcount(cmd);
  for (size_t i = 0; i < payload_len; ++i) {
    ones += __builtin_popcount(payload[i]);
  }
  return ones & 1;
}

// Validate the parity bit on a wire-format message (header byte first).
// Returns true if parity is correct.
inline bool validate_parity(const uint8_t *msg, size_t msg_len) {
  if (msg_len < 2) return false;
  uint8_t header = msg[0];
  uint8_t expected = compute_parity_bit(header, msg[1], msg + 2, msg_len - 2);
  uint8_t actual   = (header >> 7) & 0x01;
  return expected == actual;
}

// Stamp a parity-correct header byte into msg[0] for a buffer that already
// has {version_only_header, cmd, payload...} laid out.
inline void stamp_header_parity(uint8_t *msg, size_t msg_len) {
  if (msg_len < 2) return;
  uint8_t version = msg[0] & 0x7F;
  uint8_t p = compute_parity_bit(version, msg[1], msg + 2, msg_len - 2);
  msg[0] = version | (p << 7);
}

// Lay out a V2 "display PSRAM index" block into `block` and stamp parity.
// `cmd_id` selects mode + duty shape; `duty` is used only for the 0x6x
// explicit-duty opcodes. Returns the block byte count (4 or 5).
inline uint16_t build_psram_index_block(uint8_t *block, uint16_t index,
                                        uint8_t cmd_id, uint8_t duty) {
  bool explicit_duty = (cmd_id & 0xF0) == 0x60;
  block[0] = header_version_v2;          // parity stamped below
  block[1] = cmd_id;
  block[2] = (uint8_t)(index & 0xFF);
  block[3] = (uint8_t)((index >> 8) & 0xFF);
  uint16_t len = block_byte_count_psram;
  if (explicit_duty) { block[4] = duty; len = block_byte_count_psram_duty; }
  stamp_header_parity(block, len);
  return len;
}

} // namespace G6
