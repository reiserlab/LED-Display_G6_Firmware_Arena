#pragma once

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Controller health telemetry + reset-surviving breadcrumbs (GET_HEALTH 0xCA),
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
//   * a main-loop BREADCRUMB in uninitialized OCRAM that survives
//     SYSTEM_RESET (0x01, SCB_AIRCR SYSRESETREQ), a lockup reset and the
//     watchdog reset, but not power-on. Each potentially blocking call site
//     writes its op code + micros() just before the call and resets to idle
//     after it; the record also tracks the single slowest op seen this boot.
//   * a SEPARATE ISR / WATCHDOG record (own cache line, own magic+checksum,
//     sealed only from interrupt context): which ISR was entered last and
//     never exited, and the stacked PC/LR captured by the watchdog's
//     pre-reset interrupt. Separate so that a main loop caught mid-mark()
//     (its checksummed fields dirty) cannot invalidate the PC capture, and
//     vice versa. Both are harvested independently at boot.
//   * a HARDWARE WATCHDOG (RTWDOG, 2 s, kicked from loop() only — never from
//     an ISR) that turns a hang into a reset WITH the breadcrumbs + telemetry
//     ring intact. Long synchronous operations widen it to a finite 30 s
//     window instead of disabling it — recovery is never fully off.
//
// Hot-path cost per mark()/clear(): three byte stores, one micros(), one
// checksum XOR chain, and a one-cache-line dcache flush (OCRAM is write-back
// cached on Teensy 4 — without the flush the record would still be sitting in
// the cache when SYSRESETREQ fires, exactly as PJRC's CrashReport handles it).
// ---------------------------------------------------------------------------

// Compile-time watchdog switch (default ON in this branch). Runtime control:
// SET_TELEMETRY (0xA8) flags bit4 = "watchdog bits present", then bit6 =
// watchdog OFF, bit5 = starve (bench test). Without bit4 the watchdog policy
// is untouched by a plain logging enable/disable.
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
  // 2026-09-12 wedges (#2 15:44, #3 16:27) sat in (breadcrumb OP_CMD/0x70,
  // ~3 ms after the last FRAME). arg = opcode. clear() after a nested op
  // leaves OP_IDLE, so each step re-marks.
  OP_CMD_DISARM  = 6,  // spi_.disarmRefreshTimer() (IntervalTimer::end)
  OP_CMD_PRELOAD = 7,  // between disarm and loadFrame (index store, patternOpen check)
  OP_CMD_ARM     = 8,  // spi_.armRefreshTimer() (IntervalTimer::begin -> PIT + NVIC)
  OP_CMD_RESPOND = 9,  // current_source_->sendResponse()
};

// ISR record ids (IsrRecord::isr_last: id while inside, 0 after exit).
enum IsrId : uint8_t {
  ISR_NONE    = 0,
  ISR_REFRESH = 1,  // SpiManager::refreshISR (PIT / IntervalTimer)
  ISR_DMA     = 2,  // SpiManager::dmaISR (EventResponder, SPI DMA completion)
  ISR_WDOG    = 3,  // RTWDOG pre-reset interrupt (wdog_pc/wdog_lr captured)
};

// The main-loop reset-surviving record. Exactly one Cortex-M7 cache line
// (32 B) so a single arm_dcache_flush() commits it. Lives at a FIXED OCRAM
// address (see Health.cpp) rather than in a linker section: Teensy 4's linker
// script has no .noinit, and .dmabuffers (DMAMEM) starts at the bottom of
// OCRAM where the boot ROM may scribble; PJRC's own CrashReport keeps its
// reset-surviving data at the TOP of OCRAM (0x2027FF80..) for the same
// reason, so this sits in the cache line just below it. Layout unchanged
// since v1 — a record left by the previous firmware still validates.
struct Breadcrumb {
  uint32_t magic;       // kMagic when written by this firmware
  uint8_t  last_op;     // LastOp in progress (OP_IDLE between calls)
  uint8_t  op_arg;      // opcode when last_op is OP_CMD or a sub-op, else 0
  uint8_t  slow_op;     // op with the longest single duration this boot
  uint8_t  pad_;
  uint32_t stamp_us;    // micros() when last_op was set
  uint32_t slow_us;     // duration of the slowest op this boot
  uint32_t reserved_[3];    // pads the record to exactly 32 B
  uint32_t check;       // seal() over the fields above; rejects power-on garbage
};
static_assert(sizeof(Breadcrumb) == 32, "Breadcrumb must be one cache line");

// The ISR / watchdog record: the cache line below the breadcrumb
// (0x2027FF20). Written ONLY from interrupt context (isrEnter/isrExit and the
// RTWDOG pre-reset ISR), each write sealed with its own checksum under a
// brief IRQ mask so nested ISRs cannot leave a stale checksum. Independent
// of the breadcrumb's validity by design (see header comment).
struct IsrRecord {
  uint32_t magic;         // kIsrMagic
  uint8_t  isr_last;      // IsrId currently inside (0 = none)
  uint8_t  wdog_fired;    // 1 once the RTWDOG pre-reset ISR ran (wdog_* valid)
  uint16_t pad_;
  uint32_t isr_count;     // ISR entries this boot
  uint32_t wdog_pc;       // stacked PC of the context the watchdog IRQ preempted
  uint32_t wdog_lr;       // stacked LR at that moment
  uint32_t wdog_stamp_us; // micros() in the watchdog ISR
  uint32_t reserved_;
  uint32_t check;
};
static_assert(sizeof(IsrRecord) == 32, "IsrRecord must be one cache line");

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
  uint32_t wdog_kicks      = 0;  // watchdog refreshes this boot

  // Previous boot's breadcrumb, harvested in begin(). prev_valid == false
  // after a power-on (magic/check mismatch) or a first flash.
  bool     prev_valid      = false;
  uint8_t  prev_last_op    = 0;
  uint8_t  prev_op_arg     = 0;
  uint8_t  prev_slow_op    = 0;
  uint32_t prev_stamp_us   = 0;
  uint32_t prev_slow_us    = 0;

  // Previous boot's ISR/watchdog record, harvested independently.
  bool     prev_isr_valid  = false;
  uint8_t  prev_isr_last   = 0;  // ISR the previous boot was inside when it died (0 = none)
  uint32_t prev_isr_count  = 0;
  bool     prev_wdog_fired = false;  // wdog_pc/lr below are a real capture
  uint32_t prev_wdog_pc    = 0;
  uint32_t prev_wdog_lr    = 0;

  // loop-timing accumulators (internal to loopTick()).
  uint32_t loop_last_entry_us = 0;
  uint32_t loop_win_max_us    = 0;
  uint32_t loop_win_start_us  = 0;
};

extern Stats stats;

// Call FIRST in setup(), before anything that could reset. Captures and
// clears SRC_SRSR, harvests both previous-boot records, resets the live ones.
void begin();

// Call at the top of every loop() iteration. Also kicks the watchdog.
void loopTick();

// Breadcrumb: bracket a potentially blocking call. Innermost mark wins —
// a nested mark() overwrites the outer one and the outer clear() is a no-op
// (it only measures/records when an op is still marked).
void mark(uint8_t op, uint8_t arg = 0);
void clear();

// ISR record: call at entry/exit of an ISR body. Byte store + increment +
// sealed flush under a brief IRQ mask.
void isrEnter(uint8_t id);
void isrExit();

// Read-only views of the live records (for GET_HEALTH).
uint8_t  slowOp();
uint32_t slowUs();
uint8_t  isrLast();
uint32_t isrCount();

// ---- Hardware watchdog (RTWDOG = WDOG3, LPO 32 kHz / 256 -> 125 Hz ticks) ----
constexpr uint32_t watchdog_timeout_ms = 2000;   // normal: every loop() must kick within this (TOVAL from the measured tick rate)
constexpr uint32_t watchdog_longop_ms  = 30000;  // finite window for SD format / ISP / image upload

// begin() arms the RTWDOG early with a long provisional timeout (boot is
// protected, never clipped); call this at the END of setup() to MEASURE the
// counter tick rate and program the real 2 s timeout from it.
void watchdogBegin();
// Refresh. Called by loopTick(); ALSO call from any bounded spin that runs in
// main-loop context for longer than the timeout (sendRaw, drainBulkData, the
// synchronous 0xE0 upload loop). Honours the starve test flag itself. Never
// call from an ISR — an ISR that keeps kicking would defeat the point.
void watchdogKick();
// Runtime enable/disable (SET_TELEMETRY bit4+bit6). Returns false if the
// RTWDOG refused the reprogramming (unlock/RCS timeout) — then the hardware
// state is unknown and is reported as still armed.
bool watchdogSetEnabled(bool on);
// Nesting-safe long-operation window: widens the timeout to
// watchdog_longop_ms for synchronous operations that legitimately exceed
// 2 s; Resume() restores 2 s when the outermost window closes. The watchdog
// is never disabled by this. Returns false if the RTWDOG refused.
bool watchdogSuspend();
bool watchdogResume();
// Bench test only (SET_TELEMETRY bit4+bit5): stop kicking so the reset path
// can be validated end to end (boot_count++, prev_* and wdog_pc readable).
void watchdogStarve(bool on);
// GET_HEALTH `wdog_flags`: bit0 armed now, bit1 previous reset was the
// watchdog (SRSR wdog3_rst_b), bit2 previous boot captured wdog_pc/lr,
// bit3 compiled in, bit4 long-op window (30 s) active, bit5 starving (test),
// bit6 the last RTWDOG reprogramming FAILED (state unknown).
uint8_t watchdogFlags();
bool    watchdogArmed();
// Raw WDOG3_CS: as found before the first programming (this silicon's reset
// default — expected 0x2520: UPDATE, CLK=LPO, RCS, CMD32EN) and a live read.
uint32_t watchdogCsAtBoot();
uint32_t watchdogCsNow();
// Measured counter tick rate (Hz; 0 until watchdogBegin ran), the live TOVAL
// readback, and the last verification bits: bit0 RCS timeout (advisory),
// bit1 EN mismatch, bit2 TOVAL mismatch, bit3 tick rate fell back to the
// default, bit4 the other unlock key width had to be retried.
uint32_t watchdogTickHz();
uint32_t watchdogTovalNow();
uint8_t  watchdogVerify();

// PJRC CrashReport region: the top 128 B of OCRAM (arm_fault_info_struct at
// 0x2027FF80, 44 B, len field = 11 words; PJRC's own breadcrumbs at
// 0x2027FFC0). Raw passthrough for GET_CRASHREPORT (0xCC); never cleared here.
constexpr uint32_t crashreport_addr = 0x2027FF80UL;
constexpr uint8_t  crashreport_len  = 128;

}  // namespace Health
