# pit-race-repro — Teensy 4.x `IntervalTimer::end()` / `pit_isr()` race

Minimal stand-alone reproducer (no arena code) for the controller wedge of
reiserlab/LED-Display_G6_Firmware_Arena#50. Core: `framework-arduinoteensy` 1.160.0,
`cores/teensy4/IntervalTimer.cpp` (upstream master has the same ordering).

**The race.** `IntervalTimer::end()` does `funct_table[i] = nullptr;` **before** `channel->TCTRL = 0;
channel->TFLG = 1;`, and `pit_isr()` acknowledges a channel's `TFLG` **only when its callback is non-null**
(`if (funct_table[0] != nullptr && channel->TFLG) { channel->TFLG = 1; funct_table[0](); }`). A PIT
expiry taken between the first two stores runs the ISR with a null callback, never clears the flag, and
re-enters forever at the PIT's NVIC priority: thread mode never runs again. Higher-priority ISRs and
equal-priority ISRs with a lower IRQ number (USB!) keep running, so the board stays enumerated.

**Bench result (2026-09-13 09:01 ET, Teensy 4.1 on the CSHL 2×10 controller, 600 MHz):**

| mode | sequence | result |
|---|---|---|
| `u` (stock) | `timer.end(); timer.begin(cb, 100 µs)` with a cycle-accurate wait that sweeps `end()` across the expiry | **main loop dead within 0.5 s** (no heartbeat, `?` unanswered, USB still enumerated, 134-baud reboot works) |
| `g` (guarded) | same, with `NVIC_DISABLE_IRQ(IRQ_PIT); dsb; isb; end(); dsb; NVIC_ENABLE_IRQ(IRQ_PIT)` | **5.92 M cycles / 2.72 M expiries in 10 min, no stall** |

The wait is essential: a tight `end(); begin();` loop restarts the period every iteration and the timer
never expires (0 storms in 53 M cycles) — the same starvation that capped the arena's displayed frame rate.

**Fix (either line closes it; both is belt and braces):**
1. `pit_isr()`: acknowledge the flag regardless of the callback —
   `if (channel->TFLG) { channel->TFLG = 1; if (funct_table[i]) funct_table[i](); }`
2. `end()`: disable first, null the callback last — `channel->TCTRL = 0; channel->TFLG = 1; funct_table[i] = nullptr;`

Build: `pio run -d tools/pit-race-repro -e teensy41`. Drive: open the CDC port, send `u`/`g`/`s`/`?`;
a heartbeat line prints every 500 ms. Side observation: `pit_irq` ≈ 2 × `ticks` — the ISR is entered a
second time per expiry because the posted `TFLG` write has not reached the PIT when the ISR returns.
