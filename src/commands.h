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
  SET_PATTERN_ID_CMD          = 0x03,  // [03 03 id_lo id_hi] load 1-based pattern into Mode 3 at frame 0
  SWITCH_GRAYSCALE_CMD        = 0x06,  // dropped for G6
  TRIAL_PARAMS_CMD            = 0x08,  // "combined command": selects mode + pattern (Modes 2/3/4)
  SET_REFRESH_RATE_CMD        = 0x16,
  GET_REFRESH_RATE_CMD        = 0x17,  // returns current refresh rate as uint16 LE Hz
  SET_PANEL_DISPLAY_MODE_CMD  = 0x1B,  // [02 1B mode] 0=oneshot 1=persist 2=triggered 3=gated; sticky, default=persist
  GET_PANEL_DISPLAY_MODE_CMD  = 0x1C,  // [01 1C] returns current panel display mode as single byte
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
  GET_AO_VOLTAGE_CMD          = 0xA1,  // [01 A1] returns hardware DAC readback as uint16 LE mV
  SET_AO_LUT_CMD              = 0xA2,  // [len A2 mode step_hz_lo step_hz_hi count_lo count_hi mv...] upload+start AO LUT
  SET_DIGITAL_OUT_CMD         = 0xAA,  // [03 AA channel state] set DO1 (ch=1, J3) or DO2 (ch=2, J4) HIGH/LOW
  GET_DIGITAL_OUT_CMD         = 0xAB,  // [01 AB] returns current state of DO1 and DO2 as two bytes
  SET_ETHERNET_IP_ADDRESS_CMD = 0xC0,  // reserved — not yet implemented
  GET_ETHERNET_IP_ADDRESS_CMD = 0xC1,
  GET_CONTROLLER_INFO_CMD     = 0xC2,  // returns {version, capability_bitmap}
  SET_DIAG_OUTPUT_CMD         = 0xC3,  // [len=2,0xC3,on] mute/unmute DEBUG_SERIAL diagnostics
  GET_DIAG_OUTPUT_CMD         = 0xC4,  // returns current g_dbg_on state (0/1)
  SET_SPI_CLOCK_CMD           = 0xC5,  // [len=3,0xC5,lo,hi] uint16 LE MHz; echoes applied MHz
  GET_SPI_CLOCK_CMD           = 0xC6,  // returns current SPI clock as uint16 LE MHz
  SET_FRAME_POSITION_CMD      = 0x70,  // Mode 3: host-commanded frame index
  // V2 PSRAM display (LAB-41/42): drive panels to render their locally-stored
  // PSRAM frame(s) via the V2 panel-protocol command (header 0x02).
  DISPLAY_PSRAM_INDEX_CMD     = 0x71,  // payload: uint16 LE index — show one PSRAM frame
  PSRAM_PLAY_CMD              = 0x72,  // payload: start(2) count(2) fps(2) LE — auto-advance
  ALL_ON_CMD                  = 0xFF,
};

} // namespace AC
