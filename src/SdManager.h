#pragma once

#include <Arduino.h>
#include <SD.h>
#include "constants.h"

// SD-card pattern backend for Modes 2/3/4.
//
// Patterns live in `/patterns/*.pat` on the Teensy 4.1's built-in SD slot
// (SDIO, BUILTIN_SDCARD). Files use the v2 G6PT format documented in
// g6_04-pattern-file-format.md:
//
//   Header (18 B): "G6PT" | [V|A_hi] | [A_lo|Obs6] | FrameCount(LE16)
//                  | RowCount | ColCount | gs_val | PanelMask(6 B)
//                  | CRC-8/AUTOSAR
//   Frame:         "FR" | FrameIndex(LE16) | PanelBlock[0..N-1] | CRC-16/CCITT
//
// Patterns are sorted alphabetically and addressed by a 1-based pattern ID
// (matches the G4.1 baseline convention). The reader validates the header
// CRC-8 on open and the per-frame CRC-16 on each frame read; on mismatch it
// returns a ControllerError code so the caller can surface an error display.
class SdManager {
 public:
  struct PatternInfo {
    uint16_t frame_count = 0;
    uint8_t  row_count   = 0;
    uint8_t  col_count   = 0;
    uint8_t  gs_val      = 0;   // 1 = GS2, 2 = GS16
    uint16_t block_size  = 0;   // 53 (GS2) or 203 (GS16)
    uint16_t num_panels  = 0;   // row_count * col_count
    uint32_t frame_size  = 0;   // prefix + blocks + CRC-16 trailer
  };

  // Mount the SD card. Returns true on success; safe to call when no card is
  // present (returns false, leaves available() == false).
  bool begin();
  bool available() const { return mounted_; }

  // Number of *.pat files discovered in /patterns (after begin()).
  uint16_t patternCount() const { return pattern_count_; }

  // Return the filename for the given 1-based pattern ID (same convention as
  // openPattern). Returns nullptr if pattern_id is 0 or > patternCount().
  const char* patternName(uint16_t pattern_id) const;

  // Open and validate the pattern with the given 1-based ID. Returns
  // AC::constants::CE_NONE on success, or a ControllerError code on failure.
  // On success info() describes the open pattern.
  uint8_t openPattern(uint16_t pattern_id);
  const PatternInfo &info() const { return info_; }
  bool patternOpen() const { return file_open_; }

  // Read the given 0-based frame of the open pattern into `dest`, laid out as
  // [4-byte "FR"+index prefix][panel blocks] — exactly what
  // SpiManager::transferFrame expects. Validates FR magic and the per-frame
  // CRC-16. Returns CE_NONE on success or a ControllerError code.
  uint8_t readFrame(uint16_t frame_index, uint8_t *dest, size_t dest_cap);

 private:
  bool     mounted_       = false;
  uint16_t pattern_count_ = 0;

  // Sorted basenames (without directory). Re-open by prepending pattern_dir.
  char names_[AC::constants::pattern_max_count]
             [AC::constants::pattern_name_byte_count];

  File     file_;
  bool     file_open_ = false;
  uint16_t open_id_   = 0;
  PatternInfo info_;

  void scanPatterns();
  uint8_t validateHeader(const uint8_t *hdr);
};
