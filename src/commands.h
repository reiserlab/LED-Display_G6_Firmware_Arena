#pragma once
#include <stdint.h>

namespace AC {

// G4-compatible host command opcodes. Not every opcode is supported by this
// G6 build — see CommandProcessor for the dispatcher. G6-dropped commands
// (DISPLAY_RESET, SWITCH_GRAYSCALE) are kept here so the wire protocol can
// still recognize and explicitly reject them.
enum ArenaCommands : uint8_t {
  ALL_OFF_CMD                 = 0x00,
  DISPLAY_RESET_CMD           = 0x01,  // dropped for G6
  SWITCH_GRAYSCALE_CMD        = 0x06,  // dropped for G6
  TRIAL_PARAMS_CMD            = 0x08,  // "combined command": selects mode + pattern (Modes 2/3/4)
  SET_REFRESH_RATE_CMD        = 0x16,
  GET_REFRESH_RATE_CMD        = 0x17,  // returns current refresh rate as uint16 LE Hz
  STOP_DISPLAY_CMD            = 0x30,
  STREAM_FRAME_CMD            = 0x32,
  GET_FRAMES_SENT_CMD         = 0x33,  // returns uint32 LE frames pushed to panels since boot/reset
  RESET_FRAMES_SENT_CMD       = 0x34,  // zeroes the frames-sent counter
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
