# Codex (gpt-6-astra) diff review — watchdog / sub-op breadcrumb build (`c47ee68..fb11681`) — reconciliation

**Run:** `.codex-review/codex-diff-review-20260912-162501-19350/` (standard + adversarial; a first run stalled at
launch waiting on stdin and produced nothing). Claude's independent analysis (written first):
`.codex-review/claude-analysis-20260912-wdog.md`. Fix commits: `4d1b61c` (review items) and `86eeb4a`
(RTWDOG never ran — clock gate; found on the bench, not by either review).

Context: wedges #2, #3 and #4 (15:44, 16:27, 16:36 ET) all captured with the breadcrumb `OP_CMD`/`0x70`
and the SD-read marker never set; the watchdog build exists to (a) self-reset a hung controller with the
ring intact and (b) capture the preempted PC. Bench: one controller, diagnostic build only.

| # | Finding | Verdict | Action |
|---|---|---|---|
| W1 | **Rejected pattern upload resets the controller**: `drainBulkData()` waits up to 15 s without kicking. | **VERIFIED+FIXED** (`4d1b61c`) | `drainBulkData` and the synchronous 0xE0 upload loop kick per iteration; other `while` loops audited (bounded by frame counts); the unbounded `while (!dmaComplete_)` in `transferPanelSet` is deliberately left un-kicked — if the DMA completion never arrives the watchdog is the exit and `wdog_pc` names it. |
| W2 | `tests/test_health.py` cannot import (duplicate namedtuple field); CrashReport `len` is 11 words not 44 bytes; sub-ops carry `arg = 0x70`. | **VERIFIED+FIXED** | Renamed field; `len ∈ {0, 11}`; arg assertion excludes ops 4/6–9. Host decoder fixed the same way (`present = len ∈ {11, 44} && ipsr ≠ 0`). |
| W3 | **Torn-record evidence loss**: the watchdog ISR flushed the shared breadcrumb line while main context could be mid-`mark()`; a checksum failure at boot discards everything including the captured PC. | **VERIFIED+FIXED** | Separate 32 B record at `0x2027FF20` (`'H6IR'`, own checksum, sealed only from ISR context under a short IRQ mask), harvested independently; breadcrumb back to its v1 layout. |
| W4 | Gating 0xCC on the ring flag sends an unknown opcode to `c47ee68` after rollback. | **VERIFIED+FIXED** | 0xCB flags **bit 3** = `crashreport` (0xCC + HEALTH ver ≥ 2); host gates 0xCC on it (`f49bd4c`). |
| W5 | Starve flag bypassed by `sendRaw`'s direct kicks. | **VERIFIED+FIXED** | Check moved into `watchdogKick()`. |
| W6 | Failed reprogramming recorded as success (`wd_armed_` cleared unconditionally). | **VERIFIED+FIXED** | Armed state kept as unknown/armed, `wdog_flags` bit 6, `Suspend/Resume/SetEnabled` return `bool`, callers append `STATE(telemetry, 0xEE, opcode)`. This is exactly the flag that exposed W10 on the bench. |
| W7 | Long ops disable recovery entirely. | **VERIFIED+FIXED** | 0x8F/0xC8/0xC9/0x8A/0xE0 reprogram `TOVAL` to 30 s and back (UPDATE=1 makes live reprogramming legal); never off. |
| W8 | Every SET_TELEMETRY reasserts watchdog policy. | **VERIFIED+FIXED** | Bits 5/6 apply only when bit 4 (watchdog control) is set; `0x01`/`0x00` never touch it. |
| W9 | `scripts/soak_mode3.py` decodes only the 66 B prefix; stale bit comments. | **VERIFIED+FIXED** | Decodes 97/89/66/55 B; comments updated. |
| W10 | (Bench, not review) `wdog_flags = 0x49` after flashing: **RTWDOG never ran** — WDOG3's bus clock (`CCM_CCGR5` bits 4–5) is gated at boot on the Teensy core; every register write was dropped. | **FOUND ON BENCH + FIXED** (`86eeb4a`) | Clock gate opened first; NXP order (unlock → TOVAL/WIN/CS immediately → wait RCS); key width follows the CMD32EN readback; success requires hardware readback of EN and TOVAL. HEALTH **ver 3, 97 B** adds `wdog_cs_boot`/`wdog_cs_now` so the bench sees the hardware's own state. Lesson for the ship-to-course gate: never trust a firmware flag for a peripheral without reading the peripheral back. |
| W11 | The 128-bus-clock pre-reset window may not suffice for the ISR's stores + flush under bus contention. | **ACCEPTED** (measure) | Starve test on the bench checks `wdog_flags` bit 2 + non-zero PC; under DMA load to be re-checked at the first real watchdog reset. Reset still happens if the capture is lost. |
| W12 | `ISR_NONE` does not exclude an ISR hang (hooks bracket callbacks, not driver code). | **ACCEPTED** (documented) | Interpretation rule recorded in README; driver-level hooks deferred. |
| W13 | Instrumented ISRs add cache flushes on every DMA callback; a telemetry-off comparison does not isolate them. | **ACCEPTED** | Diagnostic build; the stock `arena-2x10-local` control night is the isolation. |
| W14 | Default-enabled watchdog on both envs; combine diagnostics and recovery policy. | **REJECTED** for this build | One bench controller, campaign scope; the runtime disable (bit 6 with bit 4) and compile switch exist. Revisit at fw #54. |
| W15 | Pattern replacement deletes-then-writes; a reset mid-write leaves a partial file. | **DEFERRED** (fw #54) | Temp-file replacement + restart cleanup. Pre-existing design; the soak does not write patterns. |
| W16 | Single-copy ring header + resets during active execution. | **DEFERRED** (fw #54) | Same as the ring review; the starve test + first real watchdog reset will show whether the header survives a mid-append reset. |
| W17 | Versioned crash envelope; Teensy platform version not pinned; DEBUG banner clears the report. | **DEFERRED** / **ACCEPTED** | Performance build preserves the record; envelope deferred to fw #54. |

## Bench validation required before the night relies on it

1. After flashing `86eeb4a`: 0xCB clean SHA, HEALTH ver 3, `wdog_flags = 0x09`, `wdog_cs_boot = 0x2520`, `wdog_cs_now` bit 7 set, kicks ≈ loop count.
2. Starve test (`SET_TELEMETRY 0x31`): reset within 2 s; afterwards `wdog_flags` bits 1+2, non-zero `prev_wdog_pc`, ring `boot_count` +1 with the pre-reset records intact, breadcrumb valid.
3. The Studio's self-reset path (`studio-postmortem.js`) reconnects, drains, and the soak continues.
