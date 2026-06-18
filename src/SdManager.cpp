#include "SdManager.h"
#include "Crc.h"

using namespace AC;
using namespace AC::constants;

namespace {

// Case-insensitive check for a trailing ".pat" extension.
bool hasPatExtension(const char *name) {
  size_t n = strlen(name);
  if (n < 4) return false;
  const char *ext = name + (n - 4);
  return ext[0] == '.' &&
         (ext[1] == 'p' || ext[1] == 'P') &&
         (ext[2] == 'a' || ext[2] == 'A') &&
         (ext[3] == 't' || ext[3] == 'T');
}

uint16_t le16(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }

}  // namespace

bool SdManager::begin() {
  mounted_ = SD.begin(BUILTIN_SDCARD);
  pattern_count_ = 0;
  if (mounted_) {
    scanPatterns();
    DBG_PRINTF("[sd] mounted, %u pattern(s) in %s\n",
               (unsigned)pattern_count_, pattern_dir);
  } else {
    DBG_PRINTF("[sd] no card / mount failed\n");
  }
  return mounted_;
}

void SdManager::scanPatterns() {
  File dir = SD.open(pattern_dir);
  if (!dir || !dir.isDirectory()) {
    if (dir) dir.close();
    return;
  }

  for (;;) {
    File entry = dir.openNextFile();
    if (!entry) break;
    if (!entry.isDirectory()) {
      const char *name = entry.name();
      if (name && name[0] != '.' && hasPatExtension(name) &&
          strlen(name) < pattern_name_byte_count &&
          pattern_count_ < pattern_max_count) {
        // Insertion sort into names_ to keep the list alphabetical so the
        // 1-based pattern ID is stable across reboots.
        uint16_t pos = pattern_count_;
        while (pos > 0 && strcmp(names_[pos - 1], name) > 0) {
          strcpy(names_[pos], names_[pos - 1]);
          --pos;
        }
        strcpy(names_[pos], name);
        ++pattern_count_;
      }
    }
    entry.close();
  }
  dir.close();
}

uint8_t SdManager::validateHeader(const uint8_t *hdr) {
  // Magic "G6PT".
  if (hdr[0] != 'G' || hdr[1] != '6' || hdr[2] != 'P' || hdr[3] != 'T') {
    return CE_HEADER_CRC;
  }
  // Format version = high nibble of byte 4.
  if (((hdr[4] >> 4) & 0x0F) != pattern_format_version) return CE_HEADER_CRC;
  // Header CRC-8/AUTOSAR over bytes 0-16.
  if (G6::crc8_autosar(hdr, pattern_header_byte_count - 1) !=
      hdr[pattern_header_byte_count - 1]) {
    return CE_HEADER_CRC;
  }

  uint16_t frame_count = le16(hdr + 6);
  uint8_t  row = hdr[8];
  uint8_t  col = hdr[9];
  uint8_t  gs  = hdr[10];
  if (frame_count == 0) return CE_HEADER_CRC;       // 0 frames is invalid
  if (gs != 1 && gs != 2) return CE_HEADER_CRC;

  uint16_t num_panels = (uint16_t)row * col;
  if (num_panels == 0 || num_panels > arena_panel_count_max) {
    return CE_ARENA_MISMATCH;
  }
  // Panel-mask popcount must not exceed the declared grid size.
  uint16_t mask_bits = 0;
  for (uint8_t i = 0; i < 6; ++i) mask_bits += __builtin_popcount(hdr[11 + i]);
  if (mask_bits > num_panels) return CE_ARENA_MISMATCH;

  // This controller is hard-wired for the G6_2x10 arena. Reject any pattern
  // whose geometry would misroute against the compiled-in panel-set table.
  if (row != panel_count_per_frame_row || col != panel_count_per_frame_col) {
    return CE_ARENA_MISMATCH;
  }

  info_.frame_count = frame_count;
  info_.row_count   = row;
  info_.col_count   = col;
  info_.gs_val      = gs;
  info_.block_size  = (gs == 1) ? panel_block_byte_count_gs2
                                : panel_block_byte_count_gs16;
  info_.num_panels  = num_panels;
  info_.frame_size  = (uint32_t)pattern_frame_prefix_byte_count
                      + (uint32_t)num_panels * info_.block_size
                      + pattern_frame_crc_byte_count;
  return CE_NONE;
}

const char* SdManager::patternName(uint16_t pattern_id) const {
  if (pattern_id == 0 || pattern_id > pattern_count_) return nullptr;
  return names_[pattern_id - 1];
}

void SdManager::rescan() {
  pattern_count_ = 0;
  memset(names_, 0, sizeof(names_));
  if (mounted_) scanPatterns();
}

bool SdManager::deletePattern(uint16_t pattern_id) {
  if (!mounted_) return false;

  char path[sizeof(pattern_dir) + pattern_name_byte_count + 1];
  if (pattern_id == 0) {
    snprintf(path, sizeof(path), "%s/pattern.temp", pattern_dir);
    if (!SD.exists(path)) return false;
  } else {
    if (pattern_id > pattern_count_) return false;
    if (file_open_ && open_id_ == pattern_id) {
      file_.close();
      file_open_ = false;
    }
    snprintf(path, sizeof(path), "%s/%s", pattern_dir, names_[pattern_id - 1]);
  }

  if (!SD.remove(path)) return false;
  rescan();
  return true;
}

void SdManager::deleteAllPatterns() {
  if (!mounted_) return;

  if (file_open_) {
    file_.close();
    file_open_ = false;
  }

  File dir = SD.open(pattern_dir);
  if (!dir || !dir.isDirectory()) {
    if (dir) dir.close();
    rescan();
    return;
  }

  // Collect filenames first (can't remove while iterating on all SD libs).
  char to_delete[pattern_max_count + 1][pattern_name_byte_count];
  uint16_t count = 0;
  for (;;) {
    File entry = dir.openNextFile();
    if (!entry) break;
    if (!entry.isDirectory() && count <= pattern_max_count) {
      strncpy(to_delete[count], entry.name(), pattern_name_byte_count - 1);
      to_delete[count][pattern_name_byte_count - 1] = '\0';
      ++count;
    }
    entry.close();
  }
  dir.close();

  char path[sizeof(pattern_dir) + pattern_name_byte_count + 1];
  for (uint16_t i = 0; i < count; ++i) {
    snprintf(path, sizeof(path), "%s/%s", pattern_dir, to_delete[i]);
    SD.remove(path);
  }

  rescan();
}

bool SdManager::renamePattern(uint16_t pattern_id, const char* new_name,
                              uint16_t* new_idx_out) {
  if (!mounted_) return false;

  char old_path[sizeof(pattern_dir) + pattern_name_byte_count + 1];
  if (pattern_id == 0) {
    snprintf(old_path, sizeof(old_path), "%s/pattern.temp", pattern_dir);
  } else {
    if (pattern_id > pattern_count_) return false;
    // Close if it's currently open for reading.
    if (file_open_ && open_id_ == pattern_id) {
      file_.close();
      file_open_ = false;
    }
    snprintf(old_path, sizeof(old_path), "%s/%s", pattern_dir,
             names_[pattern_id - 1]);
  }

  // Resolve a unique target name: if new_name already exists, prepend an
  // increasing run of '0' chars followed by '_' until the name is free.
  // e.g. "foo.pat" → "0_foo.pat" → "00_foo.pat" → …
  char candidate[pattern_name_byte_count];
  strncpy(candidate, new_name, sizeof(candidate) - 1);
  candidate[sizeof(candidate) - 1] = '\0';

  char new_path[sizeof(pattern_dir) + pattern_name_byte_count + 1];
  snprintf(new_path, sizeof(new_path), "%s/%s", pattern_dir, candidate);

  for (uint16_t n = 1; SD.exists(new_path) && n <= 999; ++n) {
    snprintf(candidate, sizeof(candidate), "X%03u_%s", (unsigned)n, new_name);
    snprintf(new_path, sizeof(new_path), "%s/%s", pattern_dir, candidate);
  }

  if (!SD.rename(old_path, new_path)) return false;

  rescan();

  for (uint16_t i = 0; i < pattern_count_; ++i) {
    if (strncmp(names_[i], candidate, pattern_name_byte_count) == 0) {
      if (new_idx_out) *new_idx_out = i + 1;
      return true;
    }
  }
  return false;
}

uint8_t SdManager::openPattern(uint16_t pattern_id) {
  if (!mounted_) return CE_SD_NOT_PRESENT;
  if (pattern_id == 0 || pattern_id > pattern_count_) return CE_BAD_PARAM;

  // Already open and validated.
  if (file_open_ && open_id_ == pattern_id) return CE_NONE;

  if (file_open_) {
    file_.close();
    file_open_ = false;
  }

  char path[sizeof(pattern_dir) + pattern_name_byte_count + 1];
  snprintf(path, sizeof(path), "%s/%s", pattern_dir, names_[pattern_id - 1]);

  file_ = SD.open(path, FILE_READ);
  if (!file_) {
    DBG_PRINTF("[sd] open failed: %s\n", path);
    return CE_SD_FILE_ERROR;
  }

  uint8_t hdr[pattern_header_byte_count];
  if (file_.read(hdr, pattern_header_byte_count) != pattern_header_byte_count) {
    file_.close();
    return CE_SD_FILE_ERROR;
  }

  uint8_t err = validateHeader(hdr);
  if (err != CE_NONE) {
    file_.close();
    DBG_PRINTF("[sd] header invalid (err=%u): %s\n", (unsigned)err, path);
    return err;
  }

  file_open_ = true;
  open_id_   = pattern_id;
  DBG_PRINTF("[sd] opened id=%u %s frames=%u gs=%u block=%u\n",
             (unsigned)pattern_id, path, (unsigned)info_.frame_count,
             (unsigned)info_.gs_val, (unsigned)info_.block_size);
  return CE_NONE;
}

uint8_t SdManager::readFrame(uint16_t frame_index, uint8_t *dest,
                             size_t dest_cap) {
  if (!mounted_) return CE_SD_NOT_PRESENT;
  if (!file_open_) return CE_SD_FILE_ERROR;
  if (frame_index >= info_.frame_count) return CE_BAD_PARAM;

  // Frame body = prefix + panel blocks (everything except the CRC-16 trailer).
  size_t body_len = pattern_frame_prefix_byte_count
                    + (size_t)info_.num_panels * info_.block_size;
  if (dest_cap < body_len) return CE_BAD_PARAM;

  uint32_t offset = pattern_header_byte_count
                    + (uint32_t)frame_index * info_.frame_size;
  if (!file_.seek(offset)) return CE_SD_FILE_ERROR;

  if ((size_t)file_.read(dest, body_len) != body_len) return CE_SD_FILE_ERROR;

  uint8_t crc_bytes[pattern_frame_crc_byte_count];
  if ((size_t)file_.read(crc_bytes, pattern_frame_crc_byte_count) !=
      pattern_frame_crc_byte_count) {
    return CE_SD_FILE_ERROR;
  }

  // Frame magic "FR".
  if (dest[0] != 'F' || dest[1] != 'R') return CE_FRAME_CRC;

  // Per-frame CRC-16/CCITT over {FR magic, frame index, panel blocks}.
  uint16_t want = (uint16_t)crc_bytes[0] | ((uint16_t)crc_bytes[1] << 8);
  if (G6::crc16_ccitt_false(dest, body_len) != want) return CE_FRAME_CRC;

  return CE_NONE;
}
