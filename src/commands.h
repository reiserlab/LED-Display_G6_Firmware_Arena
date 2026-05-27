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
  TRIAL_PARAMS_CMD            = 0x08,  // unsupported in this build (needs Modes 2/3/4)
  SET_REFRESH_RATE_CMD        = 0x16,
  STOP_DISPLAY_CMD            = 0x30,
  STREAM_FRAME_CMD            = 0x32,
  GET_ETHERNET_IP_ADDRESS_CMD = 0x66,
  SET_FRAME_POSITION_CMD      = 0x70,  // unsupported in this build (needs Modes 2/3)
  ALL_ON_CMD                  = 0xFF,
};

} // namespace AC
