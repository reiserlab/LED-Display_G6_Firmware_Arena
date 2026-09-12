#pragma once

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Controller health telemetry + reset-surviving breadcrumb (GET_HEALTH 0xCA),
// hardware watchdog (RTWDOG / WDOG3), and CrashReport passthrough (0xCC).
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
//     SCB_AIRCR SYSRESETREQ), a lockup reset and the watchdog reset, but not
//     power-on. Each potentially blocking call site writes its op code +
//     micros() just before the call and resets to idle after it; the record
//     also tracks the single slowest op seen this boot. At the next boot the
//     previous boot's record is harvested into prev_* fields (and reported
//     by 0xCA), then the live record is reset.
//   * an ISR BREADCRUMB in the same record (isr_last / isr_count, written
//     from the refresh-timer and SPI-DMA ISRs, not checksummed) so a dump can
//     say "main loop at op X while ISR Y had been entered and never exited".
//   * a HARDWARE WATCHDOG (RTWDOG, 2 s, kicked from loop() only — never from
//     an ISR) that turns a hang into a reset WITH the breadcrumb + telemetry
//     ring intact. Its pre-reset interrupt captures the stacked PC/LR of the
//     context it preempted (the hung main loop, or the ISR that was
//     spinning) into the breadcrumb.
//
// Hot-path cost per mark()/clear(): three byte stores, one micros(), one
// checksum XOR chain, and a one-cache-line dcache flush (OCRAM is write-back
// cached on Teensy 4 — without the flush the record would still be sitting in
// the cache when SYSRESETREQ fires, exactly as PJRC's CrashReport handles it).
// ---------------------------------------------------------------------------

// Compile-time watchdog switch (default ON in this branch). Runtime control:
// SET_TELEMETRY (0xA8) flags bit6 = watchdog OFF, bit5 = starve (bench test).
#ifndef HEALTH_WATCHDOG
#define HEALTH_WATCHDOG 1
#endif

namespace Health {

// Breadcrumb op codes (payload byte `prev_breadcrumb` / `slow_op`).
enum LastOp : uint8_t {
  OP_IDLE        = 0,  // no blocking call in progress
  OP_SD_READ     = 1,  // SdManager::readFrame (via CommandProcessor::loadFrame)
  OP_SPI_FRAME   = 2,  // SpiManager::transferFrame (panel frame push)
  OP_USB_WRITE   = 3,  // SerialManager::flushResponses -> Serial.write/flush
  OP_CMD         = 4,  // CommandProcessor::handleBinaryCommand dispatch; arg = opcode
  OP_SD_OPEN     = 5,  // SdManager::openPattern (SD open + header read/validate)
  // Sub-ops inside the SET_FRAME_POSITION (0x70) handler — the window the
  // 2026-09-12 15:44 wedge sat in (breadcrumb OP_CMD/0x70, 3.3 ms after the
  // last FRAME). arg = opcode. clear() after a nested op leaves OP_IDLE, so
  // each step re-marks.
  OP_CMD_DISARM  = 6,  // spi_.disarmRefreshTimer() (IntervalTimer::end)
  OP_CMD_PRELOAD = 7,  // between disarm and loadFrame (index store, patternOpen check)
  OP_CMD_ARM     = 8,  // spi_.armRefreshTimer() (IntervalTimer::begin -> PIT + NVIC)
  OP_CMD_RESPOND = 9,  // current_source_->sendResponse()
};

// ISR breadcrumb ids (Breadcrumb::isr_last: id while inside, 0 after exit).
enum IsrId : uint8_t {
  ISR_NONE    = 0,
  ISR_REFRESH = 1,  // SpiManager::refreshISR (PIT / IntervalTimer)
  ISR_DMA     = 2,  // SpiManager::dmaISR (EventResponder, SPI DMA completion)
  ISR_WDOG    = 3,  // RTWDOG pre-reset interrupt (wdog_pc/wdog_lr captured)
};

// The reset-surviving record. Exactly one Cortex-M7 cache line (32 B) so a
// single arm_dcache_flush() commits it. Lives at a FIXED OCRAM address (see
// Health.cpp) rather than in a linker section: Teensy 4's linker script has
// no .noinit, and .dmabuffers (DMAMEM) starts at the bottom of OCRAM where
// the boot ROM may scribble; PJRC's own CrashReport keeps its reset-surviving
// data at the TOP of OCRAM (0x2027FF80..) for the same reason, so this sits
// in the cache line just below it.
//
// Field offsets are unchanged from v1 (a record left by the previous firmware
// still validates). isr_* and wdog_* are written from interrupt context and
// are deliberately NOT part of `check`: an ISR landing between a main-loop
// field write and its seal() would otherwise leave a stale checksum behind.
struct Breadcrumb {
  uint32_t magic;       // kMagic when written by this firmware
  uint8_t  last_op;     // LastOp in progress (OP_IDLE between calls)
  uint8_t  op_arg;      // opcode when last_op is OP_CMD*/sub-op, else 0
  uint8_t  slow_op;     // op with the longest single duration this boot
  uint8_t  isr_last;    // IsrId currently inside (0 = none)          [ISR-written]
  uint32_t stamp_us;    // micros() when last_op was set
  uint32_t slow_us;     // duration of the slowest op this boot
  uint32_t isr_count;   // ISR entries this boot                       [ISR-written]
  uint32_t wdog_pc;     // stacked PC when the watchdog pre-reset IRQ fired [ISR-written]
  uint32_t wdog_lr;     // stacked LR at that moment                   [ISR-written]
  uint32_t check;       // seal() over magic/last_op/op_arg/slow_op/stamp_us/slow_us
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
  uint32_t last_sd_read_us = 0;  // most recent readFrame() duration (telemetry FRAME.sd_load_us)
  uint32_t cmd70_count     = 0;  // SET_FRAME_POSITION (0x70) commands received
  uint32_t reset_cause     = 0;  // SRC_SRSR captured once at boot, then cleared
  uint32_t wdog_kicks      = 0;  // watchdog refreshes this boot (== loop iterations while armed)

  // Previous boot's breadcrumb, harvested in begin(). prev_valid == false
  // after a power-on (magic/check mismatch) or a first flash.
  bool     prev_valid      = false;
  uint8_t  prev_last_op    = 0;
  uint8_t  prev_op_arg     = 0;
  uint8_t  prev_slow_op    = 0;
  uint32_t prev_stamp_us   = 0;
  uint32_t prev_slow_us    = 0;
  uint8_t  prev_isr_last   = 0;  // ISR the previous boot was inside when it died (0 = none)
  uint32_t prev_isr_count  = 0;
  uint32_t prev_wdog_pc    = 0;  // valid when prev_isr_last == ISR_WDOG
  uint32_t prev_wdog_lr    = 0;

  // loop-timing accumulators (internal to loopTick()).
  uint32_t loop_last_entry_us = 0;
  uint32_t loop_win_max_us    = 0;
  uint32_t loop_win_start_us  = 0;
};

extern Stats stats;

// Call FIRST in setup(), before anything that could reset. Captures and
// clears SRC_SRSR, harvests the previous boot's breadcrumb, resets the live one.
void begin();

// Call at the top of every loop() iteration. Also kicks the watchdog.
void loopTick();

// Breadcrumb: bracket a potentially blocking call. Innermost mark wins —
// a nested mark() overwrites the outer one and the outer clear() is a no-op
// (it only measures/records when an op is still marked).
void mark(uint8_t op, uint8_t arg = 0);
void clear();

// ISR breadcrumb: call at entry/exit of an ISR body. One byte store, one
// increment, one cache-line flush; safe to interleave with mark()/clear().
void isrEnter(uint8_t id);
void isrExit();

// Read-only view of the live record's fields (for GET_HEALTH).
uint8_t  slowOp();
uint32_t slowUs();
uint8_t  isrLast();
uint32_t isrCount();

// ---- Hardware watchdog (RTWDOG = WDOG3, LPO 32 kHz / 256 -> 125 Hz ticks) ----
constexpr uint32_t watchdog_timeout_ms = 2000;

// Arm at the END of setup() (SD mount, blink, USB settle run unguarded). Also
// installs the pre-reset interrupt that captures the stacked PC/LR.
void watchdogBegin();
// Refresh. Called by loopTick(); ALSO call from any bounded spin that runs in
// main-loop context for longer than the timeout (SerialManager::sendRaw).
// Never call from an ISR — an ISR that keeps kicking would defeat the point.
void watchdogKick();
// Runtime enable/disable (SET_TELEMETRY bit6 = off). Re-programs the RTWDOG
// (UPDATE=1 keeps it reconfigurable). No-op when not compiled in.
void watchdogSetEnabled(bool on);
// Nesting-safe pause around synchronous operations that legitimately run
// longer than the timeout (SD format, panel ISP, firmware image upload,
// archive entry collection).
void watchdogSuspend();
void watchdogResume();
// Bench test only (SET_TELEMETRY bit5): stop kicking so the reset path can be
// validated end to end (boot_count++, prev_* and wdog_pc readable afterwards).
void watchdogStarve(bool on);
// GET_HEALTH `wdog_flags`: bit0 armed now, bit1 previous reset was the
// watchdog (SRSR wdog3_rst_b), bit2 previous boot captured wdog_pc/lr,
// bit3 compiled in, bit4 suspended, bit5 starving (test), bit6 RTWDOG
// configuration failed (unlock/RCS timed out — watchdog NOT running).
uint8_t watchdogFlags();
bool    watchdogArmed();

// PJRC CrashReport region: the top 128 B of OCRAM (arm_fault_info_struct at
// 0x2027FF80, 44 B; PJRC's own breadcrumbs at 0x2027FFC0). Raw passthrough
// for GET_CRASHREPORT (0xCC); never cleared here.
constexpr uint32_t crashreport_addr = 0x2027FF80UL;
constexpr uint8_t  crashreport_len  = 128;

}  // namespace Health
