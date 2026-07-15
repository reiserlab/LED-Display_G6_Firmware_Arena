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

  // Lightweight header metadata for GET_PATTERN_INFO (0x88) — filled by
  // readPatternInfo() without opening the display's file_ handle.
  struct PatternMeta {
    uint16_t frame_count = 0;
    uint8_t  gs_val      = 0;   // 1 = GS2, 2 = GS16
    uint8_t  rows        = 0;   // RowCount
    uint8_t  cols        = 0;   // ColCount
    uint8_t  arena_id    = 0;   // 6-bit V2 Arena ID
    uint8_t  observer_id = 0;   // 6-bit V2 Observer ID (host metadata only)
    uint32_t file_size   = 0;   // file.size()
    uint8_t  duty_cycle  = 0;   // frame 0, panel 0: last byte of the panel block
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

  // Return the origin name for the given 1-based pattern ID. Origin is set to
  // the first name assigned when pattern.temp was renamed (0x83 idx=0); preserved
  // across subsequent renames. For files present at boot, origin == name.
  const char* patternOrigin(uint16_t pattern_id) const;

  // Re-scan /patterns and rebuild the sorted name list. Call after any file
  // rename, addition, or deletion. Origins are reset to filenames (no history).
  void rescan();

  // Delete the pattern at the given 1-based pattern_id (0 = pattern.temp).
  // Returns CE_NONE on success, CE_BAD_PARAM if idx out of range,
  // CE_SD_FILE_ERROR on delete failure, CE_MANIFEST_WRITE_ERROR if manifest
  // write fails after a successful delete.
  uint8_t deletePattern(uint16_t pattern_id);

  // Format the SD card (purge-memory, 0x8F): wipes the entire filesystem,
  // not just /patterns -- also destroys /firmware/panel.bin and the
  // manifests. Rewrites a fresh empty manifest afterward; /patterns and
  // /firmware are NOT recreated here (SET_PATTERN_FILE_CMD/SET_FIRMWARE_FILE_CMD
  // create them lazily on next write).
  // Returns CE_NONE, CE_SD_FORMAT_ERROR, or CE_MANIFEST_WRITE_ERROR.
  uint8_t purgeMemory();

  // Rename the pattern at the given 1-based pattern_id (0 = pattern.temp) to
  // new_name (bare filename, no directory prefix).
  // On success, sets *new_idx_out to the new 1-based sorted index and returns
  // CE_NONE. Returns CE_BAD_PARAM, CE_SD_FILE_ERROR, or CE_MANIFEST_WRITE_ERROR.
  uint8_t renamePattern(uint16_t pattern_id, const char* new_name, uint16_t* new_idx_out);

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

  // Read header metadata for the given 1-based pattern ID into `out`, without
  // touching the display's open file_/info_ state (uses a separate File handle,
  // so it is safe to call while a pattern is playing). Validates the header
  // (magic/version/CRC-8, non-zero frame count, gs — see validateHeaderBytes)
  // and reads the first frame's panel-0 duty_cycle byte.
  // Returns CE_NONE on success or a ControllerError code. Backs GET_PATTERN_INFO
  // (0x88) — a cheap preview that avoids the full 0x84 bulk download.
  uint8_t readPatternInfo(uint16_t pattern_id, PatternMeta &out);

 private:
  bool     mounted_       = false;
  uint16_t pattern_count_ = 0;

  // Sorted basenames (without directory). Re-open by prepending pattern_dir.
  char names_[AC::constants::pattern_max_count]
             [AC::constants::pattern_name_byte_count];

  // Origin name parallel to names_[]. Set to the first name assigned when
  // pattern.temp was renamed (0x83 idx=0); preserved across subsequent renames.
  // For files present at boot, origin equals the filename.
  char origin_names_[AC::constants::pattern_max_count]
                    [AC::constants::pattern_name_byte_count];

  File     file_;
  bool     file_open_ = false;
  uint16_t open_id_   = 0;
  PatternInfo info_;

  void scanPatterns();
  // Field-level header checks shared by validateHeader() and
  // readPatternInfo(): magic, format version, CRC-8, frame_count != 0,
  // gs ∈ {1,2}. Pure — touches no member state and enforces no arena match.
  static uint8_t validateHeaderBytes(const uint8_t *hdr);
  uint8_t validateHeader(const uint8_t *hdr);
  bool writeManifest();
  void insertSorted(const char *name, const char *origin);
  void removeAt(uint16_t pos);
};
