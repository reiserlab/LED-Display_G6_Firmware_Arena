#pragma once
#include <stdint.h>

namespace AC {

// G4-compatible host command opcodes. Not every opcode is supported by this
// G6 build — see CommandProcessor for the dispatcher. G6-dropped commands
// (DISPLAY_RESET, SWITCH_GRAYSCALE) are kept here so the wire protocol can
// still recognize and explicitly reject them.
enum ArenaCommands : uint8_t {
  ALL_OFF_CMD                 = 0x00,
  SYSTEM_RESET_CMD            = 0x01,  // software system reset (SCB_AIRCR SYSRESETREQ)
  SWITCH_GRAYSCALE_CMD        = 0x06,  // dropped for G6
  TRIAL_PARAMS_CMD            = 0x08,  // "combined command": selects mode + pattern (Modes 2/3/4)
  SET_REFRESH_RATE_CMD        = 0x16,
  GET_REFRESH_RATE_CMD        = 0x17,  // returns current refresh rate as uint16 LE Hz
  STOP_DISPLAY_CMD            = 0x30,
  STREAM_FRAME_CMD            = 0x32,
  GET_FRAMES_SENT_CMD         = 0x33,  // returns uint32 LE frames pushed to panels since boot/reset
  RESET_FRAMES_SENT_CMD       = 0x34,  // zeroes the frames-sent counter
  GET_FILE_COUNT_CMD          = 0x80,  // returns pattern file count on SD as uint16 LE
  GET_PATTERN_FILENAME_CMD    = 0x82,  // [03 82 idx_lo idx_hi] 1-based; returns 1-byte-len + filename
  GET_PATTERN_FILE_CMD        = 0x84,  // [03 84 idx_lo idx_hi] 1-based; response: uint64 LE size, then raw bytes
  SET_PATTERN_FILENAME_CMD    = 0x83,  // [0x83, idx_lo, idx_hi, len, chars…] rename; returns new uint16 LE index
  SET_PATTERN_FILE_CMD        = 0x85,  // [0x85, idx_lo, idx_hi, len_b0..b7, data…] upload file (bulk stream)
  DELETE_PATTERN_FILE_CMD     = 0x86,  // [03 86 idx_lo idx_hi] delete pattern file; idx=0 deletes pattern.temp
  DELETE_ALL_PATTERNS_CMD     = 0x8F,  // [01 8F] delete all files in /patterns
  GET_SD_ARCHIVE_CMD          = 0x8A,  // [01 8A] stream full SD as a ZIP; only in ALL_OFF state
  SET_AO_VOLTAGE_CMD          = 0xA0,  // [03 A0 mv_lo mv_hi] set BNC J27 (MCP4725 DAC) 0-5000 mV
  GET_AO_VOLTAGE_CMD          = 0xA1,  // [01 A1] returns last commanded AO level as uint16 LE mV
  GET_DIGITAL_OUT_CMD         = 0xAA,  // [01 AA] returns current state of DO1 and DO2 as two bytes
  SET_DIGITAL_OUT_CMD         = 0xAB,  // [03 AB channel state] set DO1 (ch=1, J3) or DO2 (ch=2, J4) HIGH/LOW
  SET_ETHERNET_IP_ADDRESS_CMD = 0xC0,  // reserved — not yet implemented
  GET_ETHERNET_IP_ADDRESS_CMD = 0xC1,
  GET_CONTROLLER_INFO_CMD     = 0xC2,  // returns {version, capability_bitmap}
  SET_DIAG_OUTPUT_CMD         = 0xC3,  // [len=2,0xC3,on] mute/unmute DEBUG_SERIAL diagnostics
  GET_DIAG_OUTPUT_CMD         = 0xC4,  // returns current g_dbg_on state (0/1)
  SET_SPI_CLOCK_CMD           = 0xC5,  // [len=3,0xC5,lo,hi] uint16 LE MHz; echoes applied MHz
  GET_SPI_CLOCK_CMD           = 0xC6,  // returns current SPI clock as uint16 LE MHz
  SET_FRAME_POSITION_CMD      = 0x70,  // Mode 3: host-commanded frame index
  ALL_ON_CMD                  = 0xFF,
};

} // namespace AC
