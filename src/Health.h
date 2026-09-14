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
  OP_CMD_DISARM  = 6,  // spi_.disarmRefreshTimer() (IntervalTimer::end) — since the free-running
                       // refresh (2026-09-13) only on the STOP/ALL_OFF path (arg 0), never on 0x70
  OP_CMD_PRELOAD = 7,  // between disarm and loadFrame (index store, patternOpen check)
  OP_CMD_ARM     = 8,  // spi_.armRefreshTimer() (IntervalTimer::begin -> PIT + NVIC); on 0x70 only at SHOW_FRAME entry / rate change
  OP_CMD_RESPOND = 9,  // current_source_->sendResponse()
};

// ISR record ids (IsrRecord::isr_last: id while inside, 0 after exit).
enum IsrId : uint8_t {
  ISR_NONE    = 0,
  ISR_REFRESH = 1,  // SpiManager::refreshISR (PIT / IntervalTimer)
  ISR_DMA     = 2,  // SpiManager::dmaISR (EventResponder, SPI DMA completion)
  ISR_WDOG    = 3,  // RTWDOG pre-reset interrupt (wdog_pc/wdog_lr captured)
  // Core/driver vectors wrapped by thin trampolines at the end of setup()
  // (main.cpp wrapVector): entry marker + count, exit restores the previous
  // id (nesting-safe), no flush (isrEnterLite). Only wrapped if the vector is
  // not the core's unused_interrupt_vector.
  ISR_USB     = 4,  // usb_isr (IRQ_USB1) — USB-CDC
  ISR_SDHC    = 5,  // USDHC1 (IRQ_SDHC1) — SdFat SDIO
  ISR_LPSPI   = 6,  // LPSPI3/LPSPI4 (only if something attached them; the SPI DMA path uses DMA channel ISRs)
  ISR_PIT     = 7,  // the core's pit_isr (IRQ_PIT = 122) — wrapped from SpiManager::armRefreshTimer via
                    // Health::wrapPitVector() after EVERY IntervalTimer::begin (begin re-attaches pit_isr)
  ISR_ID_COUNT = 8,
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

// The ISR / watchdog record: three cache lines just below the breadcrumb
// (0x2027FEC0..0x2027FF20). Written ONLY from interrupt context (isrEnter/
// isrExit seal it; the *Lite hooks update it with an incremental XOR checksum
// and no flush; the RTWDOG pre-reset ISR seals it), each seal under a brief
// IRQ mask so nested ISRs cannot leave a stale checksum. Independent of the
// breadcrumb's validity by design (see header comment). Grown from 32 B on
// 2026-09-13 for per-ISR counts + the watchdog context capture.
// Line 0 (@0..31) holds the CONTEXT plus its own checksum `check0`, so the
// watchdog ISR can seal and flush that one line first inside its ~128-bus-
// clock pre-reset window; even a partial flush yields pc/lr/xpsr/excret.
// Lines 1-2 hold the counts, covered by `check_all` (over every word except
// the two checksums, so the lite hooks can update both incrementally).
struct IsrRecord {
  uint32_t magic;             // kIsrMagic                                   @0
  uint8_t  isr_last;          // IsrId currently inside (0 = none)            @4
  uint8_t  wdog_fired;        // 1 once the RTWDOG pre-reset ISR ran (wdog_* valid)  @5
  uint8_t  wdog_prev_isr;     // isr_last as it was when the watchdog IRQ fired (before it wrote ISR_WDOG)  @6
  uint8_t  pad_;              //                                              @7
  uint32_t wdog_pc;           // stacked PC of the context the watchdog IRQ preempted  @8
  uint32_t wdog_lr;           // stacked LR at that moment                    @12
  uint32_t wdog_xpsr;         // stacked xPSR: IPSR bits 0-8 = exception number of the
                              // interrupted context (0 = thread; PIT = 16+122 = 138)  @16
  uint32_t wdog_excret;       // EXC_RETURN (LR at handler entry): bit 3 set = thread mode
                              // preempted (0xF9/0xE9/0xFD/0xED), clear = another handler (0xF1/0xE1)  @20
  uint32_t wdog_stamp_us;     // micros() in the watchdog ISR                 @24
  uint32_t check0;            // ~XOR of words 0..6 (line 0 context)          @28
  uint32_t cnt[ISR_ID_COUNT]; // per-id entries this boot (index = IsrId)      @32..63  (line 1)
  uint32_t isr_count;         // ISR entries this boot, all ids                @64      (line 2)
  uint32_t reserved_[6];      //                                              @68..91
  uint32_t check_all;         // ~XOR of every word except check0 / check_all @92
};
static_assert(sizeof(IsrRecord) == 96, "IsrRecord must be three cache lines");

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
  uint32_t cmd70_same_index = 0; // 0x70s answered without an SD read (index already in frame_buf_; SD fast path B). RAM only, not in GET_HEALTH
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
  uint32_t prev_isr_cnt[ISR_ID_COUNT] = {0};  // per-id entries in the previous boot
  bool     prev_wdog_fired = false;  // wdog_pc/lr/xpsr/excret below are a real capture
  uint32_t prev_wdog_pc    = 0;
  uint32_t prev_wdog_lr    = 0;
  uint32_t prev_wdog_xpsr  = 0;
  uint32_t prev_wdog_excret = 0;
  uint8_t  prev_wdog_prev_isr = 0;  // the ISR that was active when the watchdog fired (0 = none/main loop)

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

// ISR record, FULL hooks (refresh / DMA callbacks): the whole field update +
// seal + flush run under a PRIMASK critical section; returns the previous id,
// which isrExit restores (nesting-safe — a callback inside the wrapped
// pit_isr must not clobber the PIT trampoline's marker).
uint8_t isrEnter(uint8_t id);
void    isrExit(uint8_t prev);
// Cheap variant for high-rate wrapped vectors (USB, SDHC): byte store + count +
// an incremental XOR update of the record checksum, NO cache flush — the
// memory copy is refreshed by the next sealing hook (refresh/dma ISR) or by the
// watchdog ISR. Returns the previous id; pass it back to isrExitLite.
uint8_t isrEnterLite(uint8_t id);
void    isrExitLite(uint8_t prev);
// Wrap the core's PIT vector (IRQ_PIT) with an ISR_PIT trampoline. Idempotent;
// MUST be called right after every IntervalTimer::begin() because begin()
// re-attaches pit_isr and thereby removes the wrapper (SpiManager does this).
void wrapPitVector();

// Read-only views of the live records (for GET_HEALTH).
uint8_t  slowOp();
uint32_t slowUs();
uint8_t  isrLast();
uint32_t isrCount();

// ---- Hardware watchdog (RTWDOG = WDOG3, LPO 32 kHz / 256 -> 125 Hz ticks) ----
constexpr uint32_t watchdog_timeout_ms = 2000;   // normal: every loop() must kick within this
// EMPIRICAL RTWDOG timing on this silicon (CLK=LPO, PRES=/256), from the
// bench starve tests of 2026-09-12: TOVAL 254 expired 0.52 s after the last
// kick, TOVAL 1000 after 6.37 s. Two points => the comparator runs at the
// 127.5 Hz the readable counter (WDOG3_CNT) also shows (32.768 kHz / 256), AND
// there is a constant deficit of ~190 ticks (~1.5 s): expiry = (TOVAL - 190)
// ticks after the last kick, as if the counter already held ~190 at kick-stop.
// The mechanism is not yet identified (refresh not zeroing CNT? refreshes only
// honoured intermittently?) — the kick path now records CNT before/after each
// refresh (GET_HEALTH ver 5 tail) to name it. Until then TOVAL is calibrated:
//   TOVAL = tick_hz * seconds + watchdog_offset_ticks
// with tick_hz MEASURED at boot from WDOG3_CNT (fallback 127).
constexpr uint32_t watchdog_offset_ticks  = 190;
constexpr uint32_t watchdog_tick_hz_fallback = 127;
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
// Measured WDOG3_CNT tick rate (Hz; ~127 on this silicon; 0 = measurement
// failed, fallback used), the live TOVAL readback, and the last verification bits: bit0 RCS timeout (advisory),
// bit1 EN mismatch, bit2 TOVAL mismatch, bit3 tick rate fell back to the
// default, bit4 the other unlock key width had to be retried.
uint32_t watchdogTickHz();
uint32_t watchdogTovalNow();
uint8_t  watchdogVerify();
// Kick-path diagnostics (ver 5 tail): WDOG3_CNT read immediately BEFORE the
// refresh (max since boot = longest gap between kicks, in ticks) and
// immediately AFTER it (min/max since boot — a refresh that zeroes the counter
// reads 0..1 here; anything else names the ~190-tick anomaly), plus live CNT.
uint16_t watchdogCntBeforeMax();
uint16_t watchdogCntAfterMin();
uint16_t watchdogCntAfterMax();
uint16_t watchdogCntNow();

// PJRC CrashReport region: the top 128 B of OCRAM (arm_fault_info_struct at
// 0x2027FF80, 44 B, len field = 11 words; PJRC's own breadcrumbs at
// 0x2027FFC0). Raw passthrough for GET_CRASHREPORT (0xCC); never cleared here.
constexpr uint32_t crashreport_addr = 0x2027FF80UL;
constexpr uint8_t  crashreport_len  = 128;

}  // namespace Health
