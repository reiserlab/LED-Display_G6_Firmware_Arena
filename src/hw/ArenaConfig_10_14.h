#pragma once

// Hardcoded panel map for the G6_4x10 arena (arena_10-14).
// Included only from ../ArenaConfig.h when ARENA_HW_10_14 is defined; relies
// on that file's enclosing `namespace AC { ... }` and its `PanelSet`
// definition: do not include this file directly.
//
// Source: net-traced from LED-Display_G6_Hardware_Arena/arena_10-14/
// {teensy,fan_out,panels,panel_column,cipo_enable}.kicad_sch at commit
// 64ec5575f0d5c3c51702700987b32667c43bc8f0. The Teensy pin for CS_00..CS_19
// and the B0/B1 SCK/COPI/CIPO pins resolve to the same GPIOs as arena_10-10,
// and each CS_xx fans out to the same column pair, so the table below is
// identical to ArenaConfig_10_10.h. It is kept as its own variant so the
// build flag, pixi tasks and docs name the real board. This mapping is
// derived from the schematics, not bench-verified: confirm with the visual
// tests on real hardware before trusting it for an unattended run.
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

constexpr uint8_t panel_set_count = 20;

// Order is the SPI transmission order. Iterating bus-column then row keeps the
// pairs of panels gated by each CS pin adjacent in time.
constexpr PanelSet panel_sets[panel_set_count] = {
    // bus_col 0 -> cols 0 & 5 (P1/P6), CS_00..CS_03
    { 0,   0,  5 },   // row 0
    { 2,  10, 15 },   // row 1
    { 3,  20, 25 },   // row 2
    { 4,  30, 35 },   // row 3
    // bus_col 1 -> cols 1 & 6 (P2/P7), CS_04..CS_07
    { 5,   1,  6 },   // row 0
    { 6,  11, 16 },   // row 1
    { 7,  21, 26 },   // row 2
    { 8,  31, 36 },   // row 3
    // bus_col 2 -> cols 2 & 7 (P3/P8), CS_08..CS_11
    { 9,   2,  7 },   // row 0
    { 10, 12, 17 },   // row 1
    { 24, 22, 27 },   // row 2
    { 25, 32, 37 },   // row 3
    // bus_col 3 -> cols 3 & 8 (P4/P9), CS_12..CS_15
    { 28,  3,  8 },   // row 0
    { 29, 13, 18 },   // row 1
    { 30, 23, 28 },   // row 2
    { 31, 33, 38 },   // row 3
    // bus_col 4 -> cols 4 & 9 (P5/P10), CS_16..CS_19
    { 32,  4,  9 },   // row 0
    { 23, 14, 19 },   // row 1
    { 22, 24, 29 },   // row 2
    { 21, 34, 39 },   // row 3
};

// All distinct Teensy GPIOs used as CS lines. SpiManager pulls these HIGH on
// boot and drives them per panel-set during transfers.
constexpr uint8_t panel_set_cs_pin_count = panel_set_count;

// MISO/CIPO return-path OE decode (arena_10-14, cipo_enable.kicad_sch).
//
// Each column's MISO tri-state buffer is gated by OE_bar = CS0 & CS1 & CS2 &
// CS3 (SN74HCS08 AND -> active-low 74LVC1G125). All four CS lines per column
// are real per-row chip-selects driven above (panel_sets[].cs_pin, rows
// 0..3), so the AND only reaches all-HIGH (buffer Hi-Z) when none of the
// four rows in that column is selected, and exactly one buffer per bus
// drives at a time.
