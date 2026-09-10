#pragma once

// Hardcoded panel map for the G6_4x12 arena (arena_12-18 v1.0 production).
// Included only from ../ArenaConfig.h when ARENA_HW_12_18 is defined; relies
// on that file's enclosing `namespace AC { ... }` and its `PanelSet`
// definition — do not include this file directly.
//
// Source: net-traced from Generation 6/Arena/arena_12-18/{teensy,fan_out,
// column_buffer}.kicad_sch (SHA of the "export v1.0" commit). CS_00..CS_19
// and the SPI bus B0/B1 SCK/COPI/CIPO pins are pin-for-pin identical to the
// arena_10-10 board (verified by tracing both schematics); CS_20..CS_23
// (Teensy D38..D41) are new GPIOs, unused on arena_10-10, added for the 6th
// column pair. This mapping is derived, not bench-verified — confirm on
// real hardware before trusting it for an unattended run.
//
// The 12 panel columns are split across two SPI buses:
//   B0 (Teensy SPI):  columns 0..5 (silk P1..P6)
//   B1 (Teensy SPI1): columns 6..11 (silk P7..P12)
//
// Each Teensy CS pin gates one column on B0 *and* the corresponding column on
// B1 simultaneously, so a single CS assertion lets us drive a pair of panels
// (one per bus) in parallel. With 6 within-bus columns x 4 panel rows we
// produce 24 panel sets, each carrying 2 panels.
//
// Panel index in the streamed frame is row-major: panel_index = row * 12 + col.

constexpr uint8_t panel_set_count = 24;

// Order is the SPI transmission order. Iterating bus-column then row keeps the
// pairs of panels gated by each CS pin adjacent in time.
constexpr PanelSet panel_sets[panel_set_count] = {
    // bus_col 0 -> cols 0 & 6 (P1/P7), CS_00..CS_03
    { 0,   0,  6 },   // row 0
    { 2,  12, 18 },   // row 1
    { 3,  24, 30 },   // row 2
    { 4,  36, 42 },   // row 3
    // bus_col 1 -> cols 1 & 7 (P2/P8), CS_04..CS_07
    { 5,   1,  7 },   // row 0
    { 6,  13, 19 },   // row 1
    { 7,  25, 31 },   // row 2
    { 8,  37, 43 },   // row 3
    // bus_col 2 -> cols 2 & 8 (P3/P9), CS_08..CS_11
    { 9,   2,  8 },   // row 0
    { 10, 14, 20 },   // row 1
    { 24, 26, 32 },   // row 2
    { 25, 38, 44 },   // row 3
    // bus_col 3 -> cols 3 & 9 (P4/P10), CS_12..CS_15
    { 28,  3,  9 },   // row 0
    { 29, 15, 21 },   // row 1
    { 30, 27, 33 },   // row 2
    { 31, 39, 45 },   // row 3
    // bus_col 4 -> cols 4 & 10 (P5/P11), CS_16..CS_19
    { 32,  4, 10 },   // row 0
    { 23, 16, 22 },   // row 1
    { 22, 28, 34 },   // row 2
    { 21, 40, 46 },   // row 3
    // bus_col 5 -> cols 5 & 11 (P6/P12), CS_20..CS_23 (new, unused on arena_10-10)
    { 41,  5, 11 },   // row 0
    { 40, 17, 23 },   // row 1
    { 39, 29, 35 },   // row 2
    { 38, 41, 47 },   // row 3
};

// All distinct Teensy GPIOs used as CS lines. SpiManager pulls these HIGH on
// boot and drives them per panel-set during transfers.
constexpr uint8_t panel_set_cs_pin_count = panel_set_count;

// CIPO return-path OE decode (arena_12-18 v1.0).
//
// Each column's CIPO tri-state buffer is gated by OE_bar = CS0 & CS1 & CS2 &
// CS3 (SN74HCS08 AND -> active-low 74LVC1G125), same topology as
// arena_10-10's miso_enable.kicad_sch (renamed cipo_enable.kicad_sch here,
// electrically identical). All four CS lines per column are real per-row
// chip-selects driven above (panel_sets[].cs_pin, rows 0..3), so the AND
// only reaches all-HIGH (buffer Hi-Z) when none of the four rows in that
// column is selected, and exactly one buffer per bus drives at a time.
