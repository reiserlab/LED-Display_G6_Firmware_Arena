#pragma once
#include <stdint.h>

// Board-specific CS/panel map, selected at build time by an -DARENA_HW_*
// flag (see platformio.ini). One firmware image targets exactly one
// hardware variant on purpose: there is no runtime fallback or
// autodetection, so a host/firmware hardware mismatch fails at compile
// time (missing flag -> #error below) instead of showing up later as a
// silently wrong or frozen frame on the panels.

namespace AC {

struct PanelSet {
  uint8_t cs_pin;       // Teensy GPIO driving CS for both bus-paired panels
  uint8_t panel_b0;     // index into the streamed frame for the B0 panel
  uint8_t panel_b1;     // index into the streamed frame for the B1 panel
};

#if defined(ARENA_HW_10_10)
  #include "hw/ArenaConfig_10_10.h"
#elif defined(ARENA_HW_12_18)
  #include "hw/ArenaConfig_12_18.h"
#elif defined(ARENA_HW_2_10)
  #include "hw/ArenaConfig_2_10.h"
#else
  #error "Define ARENA_HW_10_10, ARENA_HW_12_18 or ARENA_HW_2_10 in build_flags (see platformio.ini)"
#endif

// MISO/CIPO OE-decode tie-high list. A board whose per-column OE decode ANDs
// more CS lines than it has populated panel rows must list the unused decode
// inputs so SpiManager::begin() can hold them HIGH (see hw/ArenaConfig_2_10.h,
// which defines ARENA_HW_CS_DECODE_TIE_HIGH along with the list). Boards that
// drive every decode input as a real row chip-select (10-10, 12-18) get this
// empty placeholder, and the tie-high loop in SpiManager::begin() is a no-op.
#ifndef ARENA_HW_CS_DECODE_TIE_HIGH
constexpr uint8_t cs_decode_tie_high_pins[1] = {};  // unused: count is 0
constexpr uint8_t cs_decode_tie_high_count = 0;
#endif

} // namespace AC
