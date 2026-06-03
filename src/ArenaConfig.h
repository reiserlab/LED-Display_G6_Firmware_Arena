#pragma once
#include <stdint.h>
#include "constants.h"

// Hardcoded panel map for the G6_2x10 arena (arena_10-10 v1 production).
//
// Source: docs/development/g6_arena_configs.h + g6_07-arena-firmware-interface.md.
//
// The 10 panel columns are split across two SPI buses:
//   B0 (Teensy SPI):  columns 0..4 (silk P1..P5)
//   B1 (Teensy SPI1): columns 5..9 (silk P6..P10)
//
// Each Teensy CS pin gates one column on B0 *and* the corresponding column on
// B1 simultaneously, so a single CS assertion lets us drive a pair of panels
// (one per bus) in parallel. With 5 within-bus columns x 2 panel rows we
// produce 10 panel sets, each carrying 2 panels.
//
// Panel index in the streamed frame is row-major: panel_index = row * 10 + col.

namespace AC {

struct PanelSet {
  uint8_t cs_pin;       // Teensy GPIO driving CS for both bus-paired panels
  uint8_t panel_b0;     // index into the streamed frame for the B0 panel (cols 0..4)
  uint8_t panel_b1;     // index into the streamed frame for the B1 panel (cols 5..9)
};

constexpr uint8_t panel_set_count = 10;

// Order is the SPI transmission order. Iterating bus-column then row keeps the
// pair of panels gated by each CS pin adjacent in time.
constexpr PanelSet panel_sets[panel_set_count] = {
    // bus_col 0 -> cols 0 & 5
    { 0,   0 + 0,  0 + 5 },   // row 0
    { 2,  10 + 0, 10 + 5 },   // row 1
    // bus_col 1 -> cols 1 & 6
    { 5,   0 + 1,  0 + 6 },
    { 6,  10 + 1, 10 + 6 },
    // bus_col 2 -> cols 2 & 7
    { 9,   0 + 2,  0 + 7 },
    { 10, 10 + 2, 10 + 7 },
    // bus_col 3 -> cols 3 & 8
    { 28,  0 + 3,  0 + 8 },
    { 29, 10 + 3, 10 + 8 },
    // bus_col 4 -> cols 4 & 9
    { 32,  0 + 4,  0 + 9 },
    { 23, 10 + 4, 10 + 9 },
};

// All distinct Teensy GPIOs used as CS lines. SpiManager pulls these HIGH on
// boot and drives them per panel-set during transfers.
constexpr uint8_t panel_set_cs_pin_count = panel_set_count;

// MISO/CIPO return-path OE decode (arena_10-10 v1).
//
// Each column's MISO tri-state buffer is gated by OE̅ = CS0 & CS1 & CS2 & CS3
// (SN74HCS08 AND -> active-low 74LVC1G125). Only two of those four CS lines per
// column are the row chip-selects driven above (panel_sets[].cs_pin). The other
// two are these GPIOs — they MUST be held HIGH or the AND never reaches all-HIGH,
// the buffer can never go Hi-Z, and all columns fight on the shared wired-OR MISO
// bus (Teensy reads CIPO as 00). Tying them HIGH collapses the decode to
// OE̅ = CS_row0 & CS_row1, so exactly one buffer per bus drives at a time.
// See Generation 6/arena-hardware-bug.md.
//   P1: 3,4   P2: 7,8   P3: 24,25   P4: 30,31   P5: 22,21
constexpr uint8_t cs_decode_tie_high_pins[] = { 3, 4, 7, 8, 24, 25, 30, 31, 22, 21 };
constexpr uint8_t cs_decode_tie_high_count =
    sizeof(cs_decode_tie_high_pins) / sizeof(cs_decode_tie_high_pins[0]);

} // namespace AC
