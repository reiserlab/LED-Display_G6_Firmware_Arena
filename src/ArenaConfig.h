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
#else
  #error "Define ARENA_HW_10_10 or ARENA_HW_12_18 in build_flags (see platformio.ini)"
#endif

} // namespace AC
