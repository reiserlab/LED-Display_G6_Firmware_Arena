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

// ---------------------------------------------------------------------------
// Idle (undriven) chip-select lines — bench diagnostic for the CIPO/MISO
// contention investigation (see debug/CIPO_investigation.md).
//
// The arena_10-10 hardware routes 20 CS nets (CS_00..CS_19) to the Teensy. Each
// column's MISO-confirmation buffer (74LVC1G125) is enabled by the AND of its
// FOUR group CS lines, so the buffer turns on if ANY of those four is asserted
// (low). The 2x10 build only uses 2 of the 4 per column (the two populated
// rows), so the other 2 per column are wired to a Teensy GPIO that the firmware
// never drives -> they float into the enable decode and can spuriously enable
// extra buffers, which then contend on the shared MISO_Bx bus and collapse the
// readback to 0. Authoritative map from arena_10_of_10_v1r1.kicad_pcb (U1):
//
//   group P1/P6 : CS_00=pin0*  CS_01=pin2*  CS_02=pin3   CS_03=pin4
//   group P2/P7 : CS_04=pin5*  CS_05=pin6*  CS_06=pin7   CS_07=pin8
//   group P3/P8 : CS_08=pin9*  CS_09=pin10* CS_10=pin24  CS_11=pin25
//   group P4/P9 : CS_12=pin28* CS_13=pin29* CS_14=pin30  CS_15=pin31
//   group P5/P10: CS_16=pin32* CS_17=pin23* CS_18=pin22  CS_19=pin21
//     (* = present in panel_sets above and driven by firmware; the rest float)
//
// Holding these HIGH (deasserted) makes the per-column enables strictly one-hot
// (only the one firmware-driven CS goes low at a time). Built only when
// CS_IDLE_HIGH is defined (env teensy41-csidle); see SpiManager::begin().
//
// KEEP-OUT — never add to this list (confirmed functional on U1):
//   SPI B0 11/12/13, SPI B1 26/27/1, analog 14/15, I2C 18/19,
//   level translators 35/37, and EINT on pin 33 (a panel->Teensy input).
constexpr uint8_t idle_cs_pins[] = { 3, 4, 7, 8, 21, 22, 24, 25, 30, 31 };
constexpr uint8_t idle_cs_pin_count = sizeof(idle_cs_pins) / sizeof(idle_cs_pins[0]);

} // namespace AC
