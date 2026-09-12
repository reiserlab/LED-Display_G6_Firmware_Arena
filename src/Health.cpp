#include "Health.h"

namespace Health {

Stats stats;

namespace {

constexpr uint32_t kMagic    = 0x48364C54;  // "H6LT" — main-loop breadcrumb
constexpr uint32_t kIsrMagic = 0x48364952;  // "H6IR" — ISR / watchdog record

// Two cache lines below PJRC's CrashReport fault record (0x2027FF80) at the
// top of OCRAM: breadcrumb at 0x2027FF40, ISR/watchdog record at 0x2027FF20.
// Not in any linker section, so neither startup.c's .bss clear nor the .data
// copy touches them; the telemetry ring ends at 0x2027F000 below them.
volatile Breadcrumb *const crumb =
    reinterpret_cast<volatile Breadcrumb *>(0x2027FF40);
volatile IsrRecord *const isr =
    reinterpret_cast<volatile IsrRecord *>(0x2027FF20);

inline uint32_t computeCheck(uint32_t magic, uint8_t last_op, uint8_t op_arg,
                             uint8_t slow_op, uint32_t stamp_us, uint32_t slow_us) {
  return ~(magic ^ ((uint32_t)last_op | ((uint32_t)op_arg << 8) |
                    ((uint32_t)slow_op << 16)) ^ stamp_us ^ slow_us);
}

inline void seal() {
  crumb->check = computeCheck(crumb->magic, crumb->last_op, crumb->op_arg,
                              crumb->slow_op, crumb->stamp_us, crumb->slow_us);
  // OCRAM is write-back cached: push the line to memory so it survives a
  // reset that fires before any natural eviction (see Health.h header).
  arm_dcache_flush(const_cast<Breadcrumb *>(crumb), sizeof(Breadcrumb));
}

inline uint32_t computeIsrCheck(uint32_t magic, uint8_t isr_last, uint8_t fired,
                                uint32_t count, uint32_t pc, uint32_t lr, uint32_t stamp) {
  return ~(magic ^ ((uint32_t)isr_last | ((uint32_t)fired << 8)) ^ count ^ pc ^ lr ^ stamp);
}

inline uint32_t readPrimask() {
  uint32_t pm;
  asm volatile("mrs %0, primask" : "=r"(pm));  // the Teensy core has no __get_primask()
  return pm;
}

// Seal the ISR record under a brief IRQ mask: a higher-priority ISR landing
// between the checksum computation and its store would otherwise leave the
// record inconsistent. ~20 cycles masked; the RTWDOG IRQ simply pends.
inline void sealIsr() {
  uint32_t pm = readPrimask();
  __disable_irq();
  isr->check = computeIsrCheck(isr->magic, isr->isr_last, isr->wdog_fired, isr->isr_count,
                               isr->wdog_pc, isr->wdog_lr, isr->wdog_stamp_us);
  arm_dcache_flush(const_cast<IsrRecord *>(isr), sizeof(IsrRecord));
  if (!pm) __enable_irq();
}

// ---- RTWDOG (WDOG3) ----------------------------------------------------------
// Bench facts (2026-09-12, 86eeb4a): CS reset default 0x2520 (UPDATE, CLK=LPO,
// RCS, CMD32EN); after our programming CS reads 0x35E0 (+PRES, +EN, +INT);
// after a WATCHDOG reset the block keeps its registers (CS 0x31E0 = still EN,
// RCS clear) — the RTWDOG is not reset by the SRC reset it causes. And the
// tick with PRES (/256) is ~500 Hz, i.e. the "LPO" feeding it is 128 kHz on
// this silicon, not the 32 kHz the first build assumed (250 ticks expired in
// ~0.5 s instead of 2 s). So: the tick rate is MEASURED at boot from WDOG3_CNT
// and TOVAL derived from it; the bus clock gate (CCM_CCGR5_WDOG3, never opened
// by the Teensy core) is enabled before any access; and success is judged by
// the EN / TOVAL readback after a settle, not by RCS (already 1 at reset).
constexpr uint16_t kTovalProvisional = 0xFFFF;  // while measuring / during boot: >= 2 min at 500 Hz
constexpr uint32_t kTickHzFallback   = 500;     // bench-measured; used only if the measurement fails
constexpr uint32_t kSpinMax          = 100000;  // bounded: a mis-programmed RTWDOG must never brick boot
constexpr uint32_t kSettleUs         = 300;     // > 2 LPO clocks at 32 kHz; reconfiguration latency

bool     wd_compiled_ = (HEALTH_WATCHDOG) != 0;
bool     wd_wanted_   = (HEALTH_WATCHDOG) != 0;  // runtime desire (SET_TELEMETRY bit4+bit6 clears)
bool     wd_armed_    = false;                   // hardware enabled (or unknown after a failed disable)
bool     wd_starve_   = false;
bool     wd_failed_   = false;                   // the LAST reprogramming failed verification
bool     wd_irq_installed_ = false;
uint8_t  wd_suspend_  = 0;
bool     wd_cmd32_    = true;   // refresh-key width in effect (from the CS readback)
uint32_t wd_cs_boot_  = 0;      // WDOG3_CS as found before the first programming (reset default / retained)
uint32_t wd_cs_last_  = 0;      // WDOG3_CS read back after the last programming attempt
bool     wd_cs_boot_captured_ = false;
uint32_t wd_tick_hz_  = 0;      // measured counter tick rate (0 = not measured yet)
uint16_t wd_toval_normal_ = 0;  // ticks for watchdog_timeout_ms
uint16_t wd_toval_longop_ = 0;  // ticks for watchdog_longop_ms (capped at 0xFFFF)
uint8_t  wd_verify_   = 0;      // last verification: bit0 RCS timeout, bit1 EN mismatch, bit2 TOVAL mismatch,
                                // bit3 tick rate fell back to default, bit4 retried with the other key width

inline uint32_t readPrimaskWd() { uint32_t pm; asm volatile("mrs %0, primask" : "=r"(pm)); return pm; }

inline void rtwdogGateOn() {
  CCM_CCGR5 |= CCM_CCGR5_WDOG3(CCM_CCGR_ON);
  asm volatile("dsb");
  if (!wd_cs_boot_captured_) {
    wd_cs_boot_ = WDOG3_CS;
    wd_cmd32_   = (wd_cs_boot_ & WDOG_CS_CMD32EN) != 0;
    wd_cs_boot_captured_ = true;
  }
}

// Refresh key width must match the CMD32EN mode in effect.
inline void rtwdogRefresh() {
  if (wd_cmd32_) {
    WDOG3_CNT = 0xB480A602UL;
  } else {
    WDOG3_CNT = 0xA602; WDOG3_CNT = 0xB480;
  }
}

inline bool rtwdogSpin(uint32_t bit) {
  for (uint32_t i = 0; i < kSpinMax; ++i) {
    if (WDOG3_CS & bit) return true;
  }
  return false;
}

inline void spinUs(uint32_t us) {
  uint32_t t0 = micros();
  while ((uint32_t)(micros() - t0) < us) {}
}

// One programming attempt with the given unlock key width. NXP order: unlock
// key(s) -> TOVAL, WIN, CS IMMEDIATELY (the 128-bus-clock reconfiguration
// window opens at the unlock; nothing polled in between) -> then settle and
// verify from the readback.
bool rtwdogAttempt(bool enable, uint16_t toval, bool key32) {
  uint32_t primask = readPrimaskWd();
  __disable_irq();
  if (key32) {
    WDOG3_CNT = 0xD928C520UL;                 // 32-bit unlock
  } else {
    WDOG3_CNT = 0xC520; WDOG3_CNT = 0xD928;   // 16-bit unlock sequence (within 16 bus clocks)
  }
  WDOG3_TOVAL = toval;
  WDOG3_WIN   = 0;
  WDOG3_CS    = WDOG_CS_CMD32EN | WDOG_CS_CLK(1) | WDOG_CS_PRES | WDOG_CS_UPDATE
              | WDOG_CS_INT | (enable ? WDOG_CS_EN : 0);
  bool rcs = rtwdogSpin(WDOG_CS_RCS);   // informational: RCS may already read 1 (reset default)
  if (!primask) __enable_irq();
  spinUs(kSettleUs);                    // let the reconfiguration latch (<= 2 LPO clocks)
  wd_cs_last_ = WDOG3_CS;
  wd_cmd32_   = (wd_cs_last_ & WDOG_CS_CMD32EN) != 0;
  uint32_t toval_rb = WDOG3_TOVAL & 0xFFFF;
  uint8_t v = 0;
  if (!rcs)                                            v |= 0x01;
  if (((wd_cs_last_ & WDOG_CS_EN) != 0) != enable)     v |= 0x02;
  if (toval_rb != toval)                               v |= 0x04;
  wd_verify_ = (wd_verify_ & 0x18) | v;   // keep the tick-fallback / retried bits
  bool ok = (v & 0x06) == 0;              // EN + TOVAL as programmed = success; RCS is advisory
  if (ok) rtwdogRefresh();                // the new period starts now
  return ok;
}

// Program the RTWDOG (enable/disable, timeout). Retries once with the other
// unlock key width if the first attempt does not verify.
bool rtwdogProgram(bool enable, uint16_t toval) {
  rtwdogGateOn();
  wd_verify_ &= 0x08;  // clear everything but the tick-fallback bit
  bool key32 = wd_cmd32_;
  if (rtwdogAttempt(enable, toval, key32)) return true;
  wd_verify_ |= 0x10;
  return rtwdogAttempt(enable, toval, !key32);
}

void installWdogIrq();

// Arm (or re-arm) with the given timeout; on failure the hardware state is
// unknown, so `armed` is only cleared when a DISABLE is confirmed.
bool rtwdogApply(bool enable, uint16_t toval) {
  if (enable) installWdogIrq();
  bool ok = rtwdogProgram(enable, toval);
  wd_failed_ = !ok;
  if (ok) wd_armed_ = enable;
  else if (enable) wd_armed_ = true;   // assume the worst: it may be running
  return ok;
}

// Measure the counter tick rate: sync to a CNT edge, then time >= 64 ticks
// (bounded 400 ms). Needs the watchdog enabled and counting (provisional
// TOVAL 0xFFFF, so no expiry during the measurement).
uint32_t rtwdogMeasureTickHz() {
  uint32_t t_start = micros();
  uint32_t c0 = WDOG3_CNT & 0xFFFF;
  while ((WDOG3_CNT & 0xFFFF) == c0) {
    if ((uint32_t)(micros() - t_start) > 400000UL) return 0;
  }
  uint32_t t0 = micros();
  uint32_t start = WDOG3_CNT & 0xFFFF;
  uint32_t ticks = 0, dt = 0;
  for (;;) {
    ticks = ((WDOG3_CNT & 0xFFFF) - start) & 0xFFFF;
    dt = micros() - t0;
    if (ticks >= 64 || dt > 400000UL) break;
  }
  if (ticks < 4 || dt == 0) return 0;
  return (uint32_t)(((uint64_t)ticks * 1000000ULL + dt / 2) / dt);
}

void computeTovals(uint32_t tick_hz) {
  uint64_t n = ((uint64_t)tick_hz * watchdog_timeout_ms + 500) / 1000;
  uint64_t l = ((uint64_t)tick_hz * watchdog_longop_ms  + 500) / 1000;
  wd_toval_normal_ = (uint16_t)(n > 0xFFFF ? 0xFFFF : (n < 2 ? 2 : n));
  wd_toval_longop_ = (uint16_t)(l > 0xFFFF ? 0xFFFF : l);
}

// Early, provisional arming (called from begin(), first thing in setup()):
// opens the clock gate, captures the CS reset/retained default, kicks a
// watchdog that is still running from before a watchdog reset (the block
// keeps its registers), and programs a long provisional timeout so the boot
// sequence (blink, SD mount, USB settle) is protected but never clipped.
void rtwdogEarlyArm() {
  rtwdogGateOn();
  if (wd_cs_boot_ & WDOG_CS_EN) rtwdogRefresh();  // retained from the previous incarnation
  rtwdogApply(true, kTovalProvisional);
}

}  // namespace

// Pre-reset interrupt. Naked so the exception frame is still exactly at MSP
// (Teensy runs everything on MSP): frame[5] = LR, frame[6] = PC of the
// context that was executing when the watchdog expired — the hung main loop,
// or the ISR that was spinning (an equal-or-higher priority ISR, e.g. the
// LPSPI IRQs at priority 0, cannot be preempted: then only the reset
// happens and isr_last says which ISR was entered last). The reset follows
// within 128 bus clocks; spin so a stray return can never re-enter. Writes
// ONLY the ISR record — the breadcrumb may be mid-mark() and stays untouched.
extern "C" void health_rtwdog_isr_c(uint32_t *frame) {
  isr->wdog_pc       = frame[6];
  isr->wdog_lr       = frame[5];
  isr->wdog_stamp_us = micros();
  isr->wdog_fired    = 1;
  isr->isr_last      = ISR_WDOG;
  sealIsr();
  for (;;) {}
}

namespace {
__attribute__((naked)) void rtwdogISR() {
  asm volatile("mrs r0, msp\n\tb health_rtwdog_isr_c");
}
void installWdogIrq() {
  if (wd_irq_installed_) return;
  attachInterruptVector(IRQ_RTWDOG, rtwdogISR);
  NVIC_SET_PRIORITY(IRQ_RTWDOG, 0);  // preempt everything preemptible
  NVIC_ENABLE_IRQ(IRQ_RTWDOG);
  wd_irq_installed_ = true;
}
}  // namespace

void begin() {
  // Reset cause — capture once, then clear (w1c) so the NEXT boot reports only
  // its own cause instead of every sticky flag since power-on.
  stats.reset_cause = SRC_SRSR;
  SRC_SRSR = stats.reset_cause;

  // Harvest the previous boot's breadcrumb. Copy out first: the validity check
  // reads several fields and the volatile pointer would re-read each one.
  Breadcrumb prev;
  prev.magic    = crumb->magic;
  prev.last_op  = crumb->last_op;
  prev.op_arg   = crumb->op_arg;
  prev.slow_op  = crumb->slow_op;
  prev.stamp_us = crumb->stamp_us;
  prev.slow_us  = crumb->slow_us;
  prev.check    = crumb->check;
  if (prev.magic == kMagic &&
      prev.check == computeCheck(prev.magic, prev.last_op, prev.op_arg,
                                 prev.slow_op, prev.stamp_us, prev.slow_us)) {
    stats.prev_valid    = true;
    stats.prev_last_op  = prev.last_op;
    stats.prev_op_arg   = prev.op_arg;
    stats.prev_slow_op  = prev.slow_op;
    stats.prev_stamp_us = prev.stamp_us;
    stats.prev_slow_us  = prev.slow_us;
  }

  // Harvest the previous boot's ISR / watchdog record — independently.
  IsrRecord pi;
  pi.magic         = isr->magic;
  pi.isr_last      = isr->isr_last;
  pi.wdog_fired    = isr->wdog_fired;
  pi.isr_count     = isr->isr_count;
  pi.wdog_pc       = isr->wdog_pc;
  pi.wdog_lr       = isr->wdog_lr;
  pi.wdog_stamp_us = isr->wdog_stamp_us;
  pi.check         = isr->check;
  if (pi.magic == kIsrMagic &&
      pi.check == computeIsrCheck(pi.magic, pi.isr_last, pi.wdog_fired, pi.isr_count,
                                  pi.wdog_pc, pi.wdog_lr, pi.wdog_stamp_us)) {
    stats.prev_isr_valid  = true;
    stats.prev_isr_last   = pi.isr_last;
    stats.prev_isr_count  = pi.isr_count;
    stats.prev_wdog_fired = pi.wdog_fired != 0;
    stats.prev_wdog_pc    = pi.wdog_pc;
    stats.prev_wdog_lr    = pi.wdog_lr;
  }

  // Fresh live records for this boot.
  crumb->magic        = kMagic;
  crumb->last_op      = OP_IDLE;
  crumb->op_arg       = 0;
  crumb->slow_op      = OP_IDLE;
  crumb->pad_         = 0;
  crumb->stamp_us     = 0;
  crumb->slow_us      = 0;
  crumb->reserved_[0] = 0;
  crumb->reserved_[1] = 0;
  crumb->reserved_[2] = 0;
  seal();

  isr->magic         = kIsrMagic;
  isr->isr_last      = ISR_NONE;
  isr->wdog_fired    = 0;
  isr->pad_          = 0;
  isr->isr_count     = 0;
  isr->wdog_pc       = 0;
  isr->wdog_lr       = 0;
  isr->wdog_stamp_us = 0;
  isr->reserved_     = 0;
  sealIsr();

  stats.loop_win_start_us = micros();

  if (wd_compiled_ && wd_wanted_) rtwdogEarlyArm();
}

void loopTick() {
  uint32_t now = micros();
  if (stats.loop_count != 0) {  // first entry only sets the baseline (skips setup())
    uint32_t dt = now - stats.loop_last_entry_us;
    if (dt > stats.loop_max_us)     stats.loop_max_us     = dt;
    if (dt > stats.loop_win_max_us) stats.loop_win_max_us = dt;
  }
  stats.loop_last_entry_us = now;
  ++stats.loop_count;

  // Roll the 1 s window: publish the completed window's max, start a new one.
  if (now - stats.loop_win_start_us >= 1000000UL) {
    stats.loop_max_1s_us    = stats.loop_win_max_us;
    stats.loop_win_max_us   = 0;
    stats.loop_win_start_us = now;
  }

  watchdogKick();
}

void mark(uint8_t op, uint8_t arg) {
  crumb->last_op  = op;
  crumb->op_arg   = arg;
  crumb->stamp_us = micros();
  seal();
}

void clear() {
  uint8_t op = crumb->last_op;
  if (op != OP_IDLE) {  // outer clear() of a nested mark: nothing to measure
    uint32_t dt = micros() - crumb->stamp_us;
    if (dt > crumb->slow_us) {
      crumb->slow_us = dt;
      crumb->slow_op = op;
    }
    crumb->last_op = OP_IDLE;
    crumb->op_arg  = 0;
    seal();
  }
}

void isrEnter(uint8_t id) {
  isr->isr_last  = id;
  isr->isr_count = isr->isr_count + 1;
  sealIsr();
}

void isrExit() {
  isr->isr_last = ISR_NONE;
  sealIsr();
}

uint8_t  slowOp()   { return crumb->slow_op; }
uint32_t slowUs()   { return crumb->slow_us; }
uint8_t  isrLast()  { return isr->isr_last; }
uint32_t isrCount() { return isr->isr_count; }

// ---- watchdog API ---------------------------------------------------------------

void watchdogBegin() {
  if (!wd_compiled_ || !wd_wanted_) return;
  if (!wd_armed_) rtwdogEarlyArm();          // begin() was skipped or failed: try again
  uint32_t hz = rtwdogMeasureTickHz();
  if (hz == 0) {
    hz = kTickHzFallback;
    wd_verify_ |= 0x08;
  } else {
    wd_verify_ &= (uint8_t)~0x08;
  }
  wd_tick_hz_ = hz;
  computeTovals(hz);
  rtwdogApply(true, wd_toval_normal_);
}

void watchdogKick() {
  if (!wd_armed_ || wd_starve_) return;  // starve test lives HERE so every kick path honours it
  rtwdogRefresh();
  ++stats.wdog_kicks;
}

bool watchdogSetEnabled(bool on) {
  if (!wd_compiled_) return true;
  wd_wanted_ = on;
  if (wd_suspend_ > 0) return true;  // applied when the long-op window closes
  if (on == wd_armed_ && !wd_failed_) return true;
  return rtwdogApply(on, wd_toval_normal_);
}

bool watchdogSuspend() {
  if (wd_suspend_++ > 0) return true;  // nested: outer window already open
  if (!wd_armed_) return true;
  return rtwdogApply(true, wd_toval_longop_);
}

bool watchdogResume() {
  if (wd_suspend_ == 0) return true;
  if (--wd_suspend_ > 0) return true;
  if (!wd_compiled_) return true;
  if (!wd_wanted_) return wd_armed_ ? rtwdogApply(false, wd_toval_normal_) : true;
  return rtwdogApply(true, wd_toval_normal_);
}

void watchdogStarve(bool on) { wd_starve_ = on; }

uint8_t watchdogFlags() {
  uint8_t f = 0;
  if (wd_armed_)                                    f |= 0x01;
  if (stats.reset_cause & SRC_SRSR_WDOG3_RST_B)     f |= 0x02;
  if (stats.prev_isr_valid && stats.prev_wdog_fired) f |= 0x04;
  if (wd_compiled_)                                 f |= 0x08;
  if (wd_suspend_ > 0)                              f |= 0x10;
  if (wd_starve_)                                   f |= 0x20;
  if (wd_failed_)                                   f |= 0x40;
  return f;
}

bool watchdogArmed() { return wd_armed_; }
uint32_t watchdogCsAtBoot() { return wd_cs_boot_; }
uint32_t watchdogCsNow()    { return WDOG3_CS; }
uint32_t watchdogTickHz()   { return wd_tick_hz_; }
uint32_t watchdogTovalNow() { return WDOG3_TOVAL & 0xFFFF; }
uint8_t  watchdogVerify()   { return wd_verify_; }

}  // namespace Health
