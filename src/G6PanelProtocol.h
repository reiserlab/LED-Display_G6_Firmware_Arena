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

// Display opcodes used by this build. Other v1 opcodes (Persistent / Triggered /
// Gated, COMM_CHECK) are recognized passively when present in pre-formatted
// host blocks; the controller does not synthesize them.
constexpr uint8_t cmd_disp_2lvl_oneshot   = 0x10;
constexpr uint8_t cmd_disp_2lvl_persist   = 0x11;
constexpr uint8_t cmd_disp_16lvl_oneshot  = 0x30;
constexpr uint8_t cmd_disp_16lvl_persist  = 0x31;

// Block sizes (header + cmd + pixel data + duty_cycle).
constexpr uint16_t block_byte_count_gs2  = 53;
constexpr uint16_t block_byte_count_gs16 = 203;

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

} // namespace G6
