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
  // PSRAM display (LAB-41/42): drive panels to render their locally-stored
  // PSRAM frame(s) via the V2 panel-protocol command (header 0x02). Grouped
  // here with the other display run-control / streaming commands.
  DISPLAY_PSRAM_INDEX_CMD     = 0x3A,  // payload: uint16 LE index — show one PSRAM frame
  PSRAM_PLAY_CMD              = 0x3B,  // payload: start(2) count(2) fps(2) LE — auto-advance
  GET_FILE_COUNT_CMD          = 0x80,  // returns pattern file count on SD as uint16 LE
  GET_PATTERN_FILENAME_CMD    = 0x82,  // [03 82 idx_lo idx_hi] 1-based; returns 1-byte-len + filename
  GET_PATTERN_INFO_CMD        = 0x88,  // [03 88 idx_lo idx_hi] 1-based; framed reply: header metadata + frame-0 duty_cycle (cheap preview, no bulk download)
  GET_PATTERN_FILE_CMD        = 0x84,  // [03 84 idx_lo idx_hi] 1-based; response: uint64 LE size, then raw bytes
  SET_PATTERN_FILENAME_CMD    = 0x83,  // [0x83, idx_lo, idx_hi, len, chars…] rename; returns new uint16 LE index
  SET_PATTERN_FILE_CMD        = 0x85,  // [0x85, idx_lo, idx_hi, len_b0..b7, data…] upload file (bulk stream)
  DELETE_PATTERN_FILE_CMD     = 0x86,  // [03 86 idx_lo idx_hi] delete pattern file; idx=0 deletes pattern.temp
  DELETE_ALL_PATTERNS_CMD     = 0x8F,  // [01 8F] delete all files in /patterns
  GET_SD_ARCHIVE_CMD          = 0x8A,  // [01 8A] stream full SD as a ZIP; only in ALL_OFF state
  SET_AO_VOLTAGE_CMD          = 0xA0,  // [03 A0 mv_lo mv_hi] set BNC J27 (MCP4725 DAC) 0-5000 mV
  GET_AO_VOLTAGE_CMD          = 0xA1,  // [01 A1] returns hardware DAC readback as uint16 LE mV
  SET_AO_LUT_CMD              = 0xA2,  // [len A2 mode step_hz_lo step_hz_hi count_lo count_hi mv...] upload+start AO LUT
  SET_AO_MODE_CMD             = 0xA3,  // [02 A3 mode] 0=programmable (0xA0/0xA2) | 1=frame_number (DAC tracks frame index, 0-5V normalized)
  GET_ANALOG_IN_CMD           = 0xA4,  // [01 A4] returns Analog In 1 + Analog In 2 as two int16 LE mV (±10V front-end, calibration TBD)
  SET_DIGITAL_OUT_CMD         = 0xAA,  // [03 AA channel state] drive "Digital IO 1/2 (5V)" BNC HIGH/LOW (requires role out_programmable; off auto-promotes)
  GET_DIGITAL_OUT_CMD         = 0xAB,  // [01 AB] returns current state of Digital IO 1 and 2 data pins as two bytes
  SET_DIO_ROLE_CMD            = 0xAC,  // [03 AC port role] port 1|2; role 0=off 1=in_trigger 2=out_programmable 3=out_debug_framescan
  GET_DIO_ROLE_CMD            = 0xAD,  // [01 AD] returns [role1, level1, role2, level2] (level = live pin read, BNC level in input roles)
  SET_ETHERNET_IP_ADDRESS_CMD = 0xC0,  // reserved — not yet implemented
  GET_ETHERNET_IP_ADDRESS_CMD = 0xC1,
  GET_CONTROLLER_INFO_CMD     = 0xC2,  // returns {version, capability_bitmap, mac[6]}
  SET_DIAG_OUTPUT_CMD         = 0xC3,  // [len=2,0xC3,on] mute/unmute DEBUG_SERIAL diagnostics
  GET_DIAG_OUTPUT_CMD         = 0xC4,  // returns current g_dbg_on state (0/1)
  SET_SPI_CLOCK_CMD           = 0xC5,  // [len=3,0xC5,lo,hi] uint16 LE MHz; echoes applied MHz
  GET_SPI_CLOCK_CMD           = 0xC6,  // returns current SPI clock as uint16 LE MHz
  G6_PANEL_STORAGE_MODE_CMD   = 0xC7,  // reserved — not yet implemented; [02 C7 mode] 0=SD 1=local storage
  G6_PROGRAM_PANEL_CMD        = 0xC8,  // [02 C8 panel_number] reflash one panel from /firmware/panel.bin via SPI ISP (panel_number 1-based, matches panel-map)
  G6_VERIFY_PANEL_CMD         = 0xC9,  // [02 C9 panel_number] CRC the panel's RUNNING app flash vs /firmware/panel.bin footer (panel_number 1-based)
  // Panel firmware image transfer to the controller SD (g6_03 § Panel firmware update).
  SET_FIRMWARE_FILE_CMD       = 0xE0,  // [0xE0, len_b0..b7, data…] upload image → /firmware/panel.bin; reply u32 LE CRC-32
  GET_FIRMWARE_INFO_CMD       = 0xE3,  // [01 E3] reply: 32-byte footer {magic[8], version[16], crc32 LE, size LE}
  // Frame-position block (G4-compatible X axis at 0x70; Y axis reserved for a
  // future G6 version, mirroring G4 setPositionX/setPositionY at 0x70/0x71).
  SET_FRAME_POSITION_CMD      = 0x70,  // Mode 3: host-commanded frame index (set-position-x; G4 setPositionX)
  SET_POSITION_Y_CMD          = 0x71,  // reserved — not yet implemented (future set-position-y; G4 setPositionY)
  GET_FRAME_POSITION_CMD      = 0x72,  // [01 72] returns cur_frame_index + frame_count (get-position-x), both uint16 LE
  GET_POSITION_Y_CMD          = 0x73,  // reserved — not yet implemented (future get-position-y)
  ALL_ON_CMD                  = 0xFF,
};

} // namespace AC
