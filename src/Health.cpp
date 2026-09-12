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
// Reset value of WDOG3_CS on i.MX RT1060 is 0x2520: disabled, UPDATE=1
// (reconfigurable after unlock), CLK=LPO (32 kHz), CMD32EN=1. PJRC's
// startup.c does not touch it. With PRES (/256) the counter ticks at 125 Hz,
// so TOVAL = timeout_ms / 8. INT=1 makes the timeout raise IRQ_RTWDOG first
// and assert the SRC reset 128 bus clocks later — long enough for the ISR
// below to stash the stacked PC/LR and flush one cache line. UPDATE=1 is what
// lets us re-program TOVAL live (long-op window) and disable at runtime.
constexpr uint16_t kTovalNormal = (uint16_t)(watchdog_timeout_ms / 8);  // 250  -> 2.0 s
constexpr uint16_t kTovalLongOp = (uint16_t)(watchdog_longop_ms  / 8);  // 3750 -> 30.0 s
constexpr uint32_t kSpinMax = 100000;  // bounded: a mis-programmed RTWDOG must never brick boot

bool     wd_compiled_ = (HEALTH_WATCHDOG) != 0;
bool     wd_wanted_   = (HEALTH_WATCHDOG) != 0;  // runtime desire (SET_TELEMETRY bit4+bit6 clears)
bool     wd_armed_    = false;                   // hardware enabled (or unknown after a failed disable)
bool     wd_starve_   = false;
bool     wd_failed_   = false;                   // the LAST reprogramming failed (state unknown)
bool     wd_irq_installed_ = false;
uint8_t  wd_suspend_  = 0;

inline bool rtwdogSpin(uint32_t bit) {
  for (uint32_t i = 0; i < kSpinMax; ++i) {
    if (WDOG3_CS & bit) return true;
  }
  return false;
}

// Program the RTWDOG (enable/disable, timeout) inside the unlock window, IRQs
// off. The counter is refreshed right after, so the new period starts now.
bool rtwdogProgram(bool enable, uint16_t toval) {
  uint32_t primask = readPrimask();
  __disable_irq();
  if (WDOG3_CS & WDOG_CS_CMD32EN) {
    WDOG3_CNT = 0xD928C520UL;                 // 32-bit unlock
  } else {
    WDOG3_CNT = 0xC520; WDOG3_CNT = 0xD928;   // 16-bit unlock sequence
  }
  bool ok = rtwdogSpin(WDOG_CS_ULK);
  if (ok) {
    WDOG3_TOVAL = toval;
    WDOG3_WIN   = 0;
    WDOG3_CS    = WDOG_CS_CMD32EN | WDOG_CS_CLK(1) | WDOG_CS_PRES | WDOG_CS_UPDATE
                | WDOG_CS_INT | (enable ? WDOG_CS_EN : 0);
    ok = rtwdogSpin(WDOG_CS_RCS);   // reconfiguration success
  }
  if (!primask) __enable_irq();
  if (ok) WDOG3_CNT = 0xB480A602UL;  // refresh
  return ok;
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
  rtwdogApply(true, kTovalNormal);
}

void watchdogKick() {
  if (!wd_armed_ || wd_starve_) return;  // starve test lives HERE so every kick path honours it
  WDOG3_CNT = 0xB480A602UL;
  ++stats.wdog_kicks;
}

bool watchdogSetEnabled(bool on) {
  if (!wd_compiled_) return true;
  wd_wanted_ = on;
  if (wd_suspend_ > 0) return true;  // applied when the long-op window closes
  if (on == wd_armed_ && !wd_failed_) return true;
  return rtwdogApply(on, kTovalNormal);
}

bool watchdogSuspend() {
  if (wd_suspend_++ > 0) return true;  // nested: outer window already open
  if (!wd_armed_) return true;
  return rtwdogApply(true, kTovalLongOp);
}

bool watchdogResume() {
  if (wd_suspend_ == 0) return true;
  if (--wd_suspend_ > 0) return true;
  if (!wd_compiled_) return true;
  if (!wd_wanted_) return wd_armed_ ? rtwdogApply(false, kTovalNormal) : true;
  return rtwdogApply(true, kTovalNormal);
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

}  // namespace Health
