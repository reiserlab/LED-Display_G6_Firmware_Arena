#pragma once
#include <stdint.h>
#include <stddef.h>

// CRC primitives used by the G6 pattern-file reader.
//   - CRC-8/AUTOSAR : pattern-file header byte 17 (over header bytes 0-16)
//   - CRC-16/CCITT  : per-frame trailer (over {FR magic, frame index, blocks})
// Parameters and test vectors per g6_01-panel-protocol.md § CRC-8 algorithm
// and g6_04-pattern-file-format.md § Per-frame CRC-16.
namespace G6 {

// CRC-8/AUTOSAR: poly 0x2F, init 0xFF, refin=false, refout=false, xorout 0xFF.
// Universal check value 0xDF over the ASCII string "123456789".
inline uint8_t crc8_autosar(const uint8_t *data, size_t len) {
  uint8_t crc = 0xFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x2F) : (uint8_t)(crc << 1);
    }
  }
  return crc ^ 0xFF;
}

// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, refin=false, refout=false,
// xorout 0x0000. Universal check value 0x29B1 over the ASCII string
// "123456789".
inline uint16_t crc16_ccitt_false(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

// CRC-32/ISO-HDLC (PKZIP): poly 0xEDB88320 (reflected), init 0xFFFFFFFF,
// xorout 0xFFFFFFFF. 4-bit nibble table (~64 B).
// Usage: crc = crc32_update(0xFFFFFFFFu, data, len) ^ 0xFFFFFFFFu
// Universal check value 0xCBF43926 over "123456789".
inline uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len) {
  static const uint32_t T[16] = {
    0x00000000u, 0x1db71064u, 0x3b6e20c8u, 0x26d930acu,
    0x76dc4190u, 0x6b6b51f4u, 0x4db26158u, 0x5005713cu,
    0xedb88320u, 0xf00f9344u, 0xd6d6a3e8u, 0xcb61b38cu,
    0x9b64c2b0u, 0x86d3d2d4u, 0xa00ae278u, 0xbdbdf21cu,
  };
  for (size_t i = 0; i < len; ++i) {
    crc = T[(crc ^ data[i]) & 0xfu] ^ (crc >> 4);
    crc = T[(crc ^ (data[i] >> 4)) & 0xfu] ^ (crc >> 4);
  }
  return crc;
}

} // namespace G6
