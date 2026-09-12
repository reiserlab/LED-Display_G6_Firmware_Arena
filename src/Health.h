#pragma once

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Controller health telemetry + reset-surviving breadcrumb (GET_HEALTH 0xCA).
//
// Issue #50: during Mode-3 host streaming (SET_FRAME_POSITION 0x70 at
// 100-286 Hz over USB CDC) the controller sometimes degrades from ~2 ms to
// ~100-500 ms per command and never recovers without a power cycle. Nothing
// in the firmware surfaced loop timing, SD errors, or what the controller was
// doing when it wedged. This module is the read-only instrumentation a soak
// harness polls (~1 Hz, and after a fault) to localize that:
//
//   * cumulative counters + max-durations (loop, SD readFrame, 0x70 count),
//     never cleared by a read — a poller diffs successive samples;
//   * a BREADCRUMB in uninitialized OCRAM that survives SYSTEM_RESET (0x01,
//     SCB_AIRCR SYSRESETREQ) but not power-on. Each potentially blocking call
//     site writes its op code + micros() just before the call and resets to
//     idle after it; the record also tracks the single slowest op seen this
//     boot. At the next boot the previous boot's record is harvested into
//     prev_* fields (and reported by 0xCA), then the live record is reset.
//
// Hot-path cost per mark()/clear(): three byte stores, one micros(), one
// checksum XOR chain, and a one-cache-line dcache flush (OCRAM is write-back
// cached on Teensy 4 — without the flush the record would still be sitting in
// the cache when SYSRESETREQ fires, exactly as PJRC's CrashReport handles it).
// ---------------------------------------------------------------------------

namespace Health {

// Breadcrumb op codes (payload byte `prev_breadcrumb` / `slow_op`).
enum LastOp : uint8_t {
  OP_IDLE      = 0,  // no blocking call in progress
  OP_SD_READ   = 1,  // SdManager::readFrame (via CommandProcessor::loadFrame)
  OP_SPI_FRAME = 2,  // SpiManager::transferFrame (panel frame push)
  OP_USB_WRITE = 3,  // SerialManager::flushResponses -> Serial.write/flush
  OP_CMD       = 4,  // CommandProcessor::handleBinaryCommand dispatch; arg = opcode
  OP_SD_OPEN   = 5,  // SdManager::openPattern (SD open + header read/validate)
};

// The reset-surviving record. Exactly one Cortex-M7 cache line (32 B) so a
// single arm_dcache_flush() commits it. Lives at a FIXED OCRAM address (see
// Health.cpp) rather than in a linker section: Teensy 4's linker script has
// no .noinit, and .dmabuffers (DMAMEM) starts at the bottom of OCRAM where
// the boot ROM may scribble; PJRC's own CrashReport keeps its reset-surviving
// data at the TOP of OCRAM (0x2027FF80..) for the same reason, so this sits
// in the cache line just below it.
struct Breadcrumb {
  uint32_t magic;       // kMagic when written by this firmware
  uint8_t  last_op;     // LastOp in progress (OP_IDLE between calls)
  uint8_t  op_arg;      // opcode when last_op == OP_CMD, else 0
  uint8_t  slow_op;     // op with the longest single duration this boot
  uint8_t  pad_;
  uint32_t stamp_us;    // micros() when last_op was set
  uint32_t slow_us;     // duration of the slowest op this boot
  uint32_t reserved_[3];    // pads the record to exactly 32 B
  uint32_t check;       // seal() over the fields above; rejects power-on garbage
};
static_assert(sizeof(Breadcrumb) == 32, "Breadcrumb must be one cache line");

// Live counters (ordinary .bss — reset every boot). All cumulative since boot
// except loop_max_1s_us, which is a firmware-maintained rolling window.
struct Stats {
  uint32_t loop_count      = 0;  // loop() iterations since boot
  uint32_t loop_max_us     = 0;  // longest gap between successive loop() entries
  uint32_t loop_max_1s_us  = 0;  // same, over the most recent COMPLETED 1 s window
  uint32_t sd_reads        = 0;  // readFrame() calls (loadFrame) since boot
  uint32_t sd_read_max_us  = 0;  // longest readFrame() since boot
  uint32_t cmd70_count     = 0;  // SET_FRAME_POSITION (0x70) commands received
  uint32_t reset_cause     = 0;  // SRC_SRSR captured once at boot, then cleared

  // Previous boot's breadcrumb, harvested in begin(). prev_valid == false
  // after a power-on (magic/check mismatch) or a first flash.
  bool     prev_valid      = false;
  uint8_t  prev_last_op    = 0;
  uint8_t  prev_op_arg     = 0;
  uint8_t  prev_slow_op    = 0;
  uint32_t prev_stamp_us   = 0;
  uint32_t prev_slow_us    = 0;

  // loop-timing accumulators (internal to loopTick()).
  uint32_t loop_last_entry_us = 0;
  uint32_t loop_win_max_us    = 0;
  uint32_t loop_win_start_us  = 0;
};

extern Stats stats;

// Call FIRST in setup(), before anything that could reset. Captures and
// clears SRC_SRSR, harvests the previous boot's breadcrumb, resets the live one.
void begin();

// Call at the top of every loop() iteration.
void loopTick();

// Breadcrumb: bracket a potentially blocking call. Innermost mark wins —
// a nested mark() overwrites the outer one and the outer clear() is a no-op
// (it only measures/records when an op is still marked).
void mark(uint8_t op, uint8_t arg = 0);
void clear();

// Read-only view of the live record's slowest-op fields (for GET_HEALTH).
uint8_t  slowOp();
uint32_t slowUs();

}  // namespace Health
