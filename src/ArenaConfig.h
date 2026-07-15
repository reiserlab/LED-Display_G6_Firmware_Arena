#pragma once
#include <stdint.h>
#include "constants.h"

// Hardcoded panel map for the G6_4x10 arena (arena_10-10 v1.1.7 production).
//
// Source: docs/development/g6_06-arena-firmware-interface.md (Pin assignments,
// CS0..CS3 per column) + maDisplayTools/configs/arena_hardware/arena_10-10_v1p1r7.yaml.
//
// The 10 panel columns are split across two SPI buses:
//   B0 (Teensy SPI):  columns 0..4 (silk P1..P5)
//   B1 (Teensy SPI1): columns 5..9 (silk P6..P10)
//
// Each Teensy CS pin gates one column on B0 *and* the corresponding column on
// B1 simultaneously, so a single CS assertion lets us drive a pair of panels
// (one per bus) in parallel. With 5 within-bus columns x 4 panel rows we
// produce 20 panel sets, each carrying 2 panels.
//
// Panel index in the streamed frame is row-major: panel_index = row * 10 + col.

namespace AC {

struct PanelSet {
  uint8_t cs_pin;       // Teensy GPIO driving CS for both bus-paired panels
  uint8_t panel_b0;     // index into the streamed frame for the B0 panel (cols 0..4)
  uint8_t panel_b1;     // index into the streamed frame for the B1 panel (cols 5..9)
};

constexpr uint8_t panel_set_count = 20;

// Order is the SPI transmission order. Iterating bus-column then row keeps the
// pairs of panels gated by each CS pin adjacent in time.
constexpr PanelSet panel_sets[panel_set_count] = {
    // bus_col 0 -> cols 0 & 5
    { 0,   0,  5 },   // row 0
    { 2,  10, 15 },   // row 1
    { 3,  20, 25 },   // row 2
    { 4,  30, 35 },   // row 3
    // bus_col 1 -> cols 1 & 6
    { 5,   1,  6 },   // row 0
    { 6,  11, 16 },   // row 1
    { 7,  21, 26 },   // row 2
    { 8,  31, 36 },   // row 3
    // bus_col 2 -> cols 2 & 7
    { 9,   2,  7 },   // row 0
    { 10, 12, 17 },   // row 1
    { 24, 22, 27 },   // row 2
    { 25, 32, 37 },   // row 3
    // bus_col 3 -> cols 3 & 8
    { 28,  3,  8 },   // row 0
    { 29, 13, 18 },   // row 1
    { 30, 23, 28 },   // row 2
    { 31, 33, 38 },   // row 3
    // bus_col 4 -> cols 4 & 9
    { 32,  4,  9 },   // row 0
    { 23, 14, 19 },   // row 1
    { 22, 24, 29 },   // row 2
    { 21, 34, 39 },   // row 3
};

// All distinct Teensy GPIOs used as CS lines. SpiManager pulls these HIGH on
// boot and drives them per panel-set during transfers.
constexpr uint8_t panel_set_cs_pin_count = panel_set_count;

// MISO/CIPO return-path OE decode (arena_10-10 v1.1.7).
//
// Each column's MISO tri-state buffer is gated by OE̅ = CS0 & CS1 & CS2 & CS3
// (SN74HCS08 AND -> active-low 74LVC1G125). All four CS lines per column are
// now real per-row chip-selects driven above (panel_sets[].cs_pin, rows 0..3),
// so the AND only reaches all-HIGH (buffer Hi-Z) when none of the four rows in
// that column is selected, and exactly one buffer per bus drives at a time.

} // namespace AC
