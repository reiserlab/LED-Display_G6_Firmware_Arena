#pragma once
#include <stdint.h>

// ---------------------------------------------------------------------------
// Firmware build identity (GET_FIRMWARE_VERSION 0xCB + DEBUG_SERIAL boot banner).
//
// The four FW_* macros are injected per build by scripts/build_version.py (a
// PlatformIO `pre:` extra script listed in platformio.ini) from the git
// checkout being compiled: short SHA, branch, tracked-file dirty flag, and the
// UTC build date. The #ifndef fallbacks below exist so a build that bypasses
// that script (no git, source tarball, hand-invoked compiler) still compiles
// and reports "unknown" rather than failing — 0xCB must always answer.
//
// Why it exists: before this, no controller build was identifiable. 0xC2 is a
// protocol version + capability bitmap (the version byte is always 1), 0xE3 is
// the PANEL image footer, and run logs carried `firmware: null`. Issue #50 (a
// Mode-3 wedge on three rigs) could not be pinned to a build. The Studio reads
// 0xCB at connect and writes it into every run log as `run_metadata.firmware`.
//
// Any translation unit that reports these values must #include this header:
// the macros only reach code that includes it (CommandProcessor.cpp for the
// 0xCB handler, main.cpp for the boot banner).
// ---------------------------------------------------------------------------

#ifndef FW_GIT_SHA
#define FW_GIT_SHA "unknown"
#endif
#ifndef FW_GIT_BRANCH
#define FW_GIT_BRANCH "unknown"
#endif
#ifndef FW_BUILD_DATE
#define FW_BUILD_DATE "unknown"
#endif
#ifndef FW_GIT_DIRTY
#define FW_GIT_DIRTY 0
#endif

namespace AC {
namespace version {

// Raw strings as injected (NUL-terminated, unpadded). The 0xCB handler pads /
// truncates them into fixed-width ASCII fields; see the layout constants below.
constexpr char fw_git_sha[]    = FW_GIT_SHA;     // e.g. "6a0f3c9e"; "unknown" without git
constexpr char fw_git_branch[] = FW_GIT_BRANCH;  // e.g. "main"; "detached" for detached HEAD
constexpr char fw_build_date[] = FW_BUILD_DATE;  // UTC "YYYY-MM-DD"
constexpr bool fw_git_dirty    = (FW_GIT_DIRTY) != 0;  // tracked files modified at build time
#ifdef DEBUG_SERIAL
constexpr bool fw_debug_build  = true;
#else
constexpr bool fw_debug_build  = false;
#endif

// GET_FIRMWARE_VERSION (0xCB) payload layout, 46 bytes:
//   off  0  u8   ver    = fw_version_payload_version
//   off  1  u8   rows   = constants::panel_count_per_frame_row
//   off  2  u8   cols   = constants::panel_count_per_frame_col
//   off  3  u8   flags  bit0 dirty working tree at build, bit1 DEBUG_SERIAL build
//   off  4  char sha[8]      short git SHA, lowercase hex, right-padded with spaces
//   off 12  char date[10]    build date UTC "YYYY-MM-DD"
//   off 22  char branch[24]  git branch, right-padded with spaces, truncated to 24
constexpr uint8_t fw_version_payload_version = 1;
constexpr uint8_t fw_sha_field_len           = 8;
constexpr uint8_t fw_date_field_len          = 10;
constexpr uint8_t fw_branch_field_len        = 24;
constexpr uint8_t fw_version_payload_len
    = 4 + fw_sha_field_len + fw_date_field_len + fw_branch_field_len;  // 46
constexpr uint8_t fw_flag_dirty = 0x01;
constexpr uint8_t fw_flag_debug = 0x02;

}  // namespace version
}  // namespace AC
