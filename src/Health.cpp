#include "Health.h"

namespace Health {

Stats stats;

namespace {

constexpr uint32_t kMagic = 0x48364C54;  // "H6LT"

// One cache line below PJRC's CrashReport fault record (0x2027FF80) at the
// top of OCRAM. Not in any linker section, so neither startup.c's .bss clear
// nor the .data copy touches it; heap grows up from _heap_start and would
// need ~500 KB of malloc to get here (same assumption CrashReport makes).
volatile Breadcrumb *const crumb =
    reinterpret_cast<volatile Breadcrumb *>(0x2027FF40);

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

}  // namespace

void begin() {
  // Reset cause — capture once, then clear (w1c) so the NEXT boot reports only
  // its own cause instead of every sticky flag since power-on.
  stats.reset_cause = SRC_SRSR;
  SRC_SRSR = stats.reset_cause;

  // Harvest the previous boot's record. Copy out first: the validity check
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

  // Fresh live record for this boot.
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

uint8_t  slowOp() { return crumb->slow_op; }
uint32_t slowUs() { return crumb->slow_us; }

}  // namespace Health
