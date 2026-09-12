# Codex (gpt-6-astra) diff review — `feat/telemetry-ring-2x10` (22b756d..ac4c08f) — reconciliation

**Run:** `.codex-review/codex-diff-review-20260912-121400-94382/` (standard + adversarial, both rc 0).
Claude's independent analysis (written first): `claude-analysis-20260912-ring.md`.
Context: diagnostic build for the fw #50 soak (T4). Flash decision = tonight, one bench controller.

Verdict legend: **VERIFIED+FIXING** (agent commit in progress, bounded list sent 12:20 ET), **ACCEPTED**
(documented / plan), **DEFERRED** (correct; not before tonight, with reason), **REJECTED**.

| # | Finding | Verdict | Action |
|---|---|---|---|
| F1 | Reference drainer `scripts/telemetry_drain.py` gates on the `health` capability and sends 0xA8 to health-only firmware (CE 01 + glyph). **Blocking.** | **VERIFIED+FIXING** | 0xC2 → 0xCB → require flags bit 2 before 0xA8/0xA9 (also `--disable`/`--no-enable`); fake health-only controller check. The Studio host already gates on bit 2 (PR #198). |
| F2 | FRAME identity uses `cur_frame_index_`/`pattern_id_`; PSRAM/streamed frames are mislabelled or deduplicated away. | **ACCEPTED** (documented limitation) | Tonight's soak is SD playback only. README states FRAME = SD-playback frames; PSRAM/streaming tracking = follow-up issue. |
| F3 | HIL test bugs: `drain(ack=False)` cannot finish after >1 block; global timestamp monotonicity contradicts the recording contract (CMD carries dispatch-entry time, appended after the handler's STATE); overrun marker evicted by sustained overflow. | **VERIFIED+FIXING** | Tests corrected as Codex proposed. |
| F4 | Stale comments say health bit 7 gates telemetry. | **VERIFIED+FIXING** | Comments → "GET_FIRMWARE_VERSION flags bit 2". |
| F5 | Peek mode (`--no-ack`) loops on `more` without progress; `--max-bytes` smaller than a record never progresses. | **VERIFIED+FIXING** | One request per poll in peek mode; reject `--max-bytes < 32`. |
| F6 | `--raw-out` acks before the file is flushed; cleanup acks before close. | **VERIFIED+FIXING** | Flush before ack; close before the final ack. Same rule the Studio host already follows (ack means stored). |
| F7 | **Reset/ACK race:** an old high ack after a ring re-initialisation (power cycle) frees unseen new records; `dropped` does not count them. | **VERIFIED+FIXED both sides** | Firmware: ack ≥ `next_seq` or moving `read_off` backwards is ignored; seq 0 and 0xFFFFFFFF never emitted. Host (`js/arena-telemetry.js` `rearm()`): the first request after any (re)connect and before the post-reboot dump carries NO_ACK; a changed `boot_count` without `survived_reboot` resets the cursor; same incarnation drops already-stored records as duplicates (`stats.duplicates`, `stats.incarnations`). Tested. |
| F8 | Heap guard not run in `ack()`/`peek()`/`configure()`. | **VERIFIED+FIXING** | Check added to those paths. |
| F9 | Boot repair validates coarse len/type only; a malformed record can survive and block every drain (Python decoder rejects it forever). | **VERIFIED+FIXING** | Exact type-specific size validation at repair; truncate at the first failure. |
| F10 | Sequence rollover (u32) vs ordinary comparison; 0xFFFFFFFF doubles as NO_ACK. | **PARTIAL** | Never emit 0 / 0xFFFFFFFF (F7). Modular comparison on the host (`seqAfter`). 87 days at 572 rec/s is outside the diagnostic build's life; 64-bit cursor deferred. |
| F11 | Single-copy header: a reset during a header write loses the whole history; dual headers with generations proposed. | **DEFERRED** (documented) | The wedge is a hang; our bootloader reboot comes minutes later with no append in flight, so the header is consistent when it matters. A spontaneous reset mid-append could lose the ring — accepted for the diagnostic build; dual headers before anything ships to course controllers. The interrupted-write bench test (pre-T4 gate) measures the real exposure. |
| F12 | Linker reservation instead of runtime `__brkval` monitoring. | **DEFERRED** | Same position as the status review: runtime guard + flag for the diagnostic build, linker reservation before course deployment. |
| F13 | Timestamp semantics: CMD t_us = dispatch entry (excludes USB queueing), FRAME t_us = SPI transfer start (not display); seq order ≠ t order. | **ACCEPTED** (documented) | README states both; host analysis orders by seq and treats t_us wrap by seq order, never by monotonic t. |
| F14 | Retention at higher rates (~0.65 s at 100 KB/s); overrun marker cannot be promised. | **ACCEPTED** (documented) | Use `dropped` + seq gaps as the loss measure; marker is informational. |
| F15 | Instrumentation shares the workload (cache maintenance per record, drain round-trips on the same link); no worst-case jitter measurement. | **ACCEPTED** (plan) | That is what the telemetry-off control arm and the T2 proxy (`spi_us`, host RTT p99 on vs off) are for; T2 proper waits for the AD3. |
| F16 | `lastResponseStatus()` couples telemetry to the reply buffer. | **DEFERRED** | Design nit; fine while every handler replies synchronously. |
| F17 | No block-level schema version; raw sidecar has no file header/firmware identity. | **PARTIAL** | The Studio log carries `stream_schema` + `run_metadata.firmware` (0xCB) per file. The pyserial sidecar gets a header line in a follow-up; the 0xA9 header `ver` is deferred. |
| F18 | Two transports (USB + TCP) would share one destructive cursor. | **ACCEPTED** (documented) | TCP servicing is disabled; single drain owner stated in README. |
| F19 | No ISR/multi-writer race found. | n/a | Matches my analysis (single producer in `loop()`). |

## Go / no-go for flashing tonight

**Go once the agent's commit lands and builds**, because every blocking item is in the reference
script or the HIL tests, not in the recording path; the one runtime change that protects tonight's
evidence (F7 ack guard) is small and mirrored on the host. Remaining risk accepted for the
diagnostic build: single-copy header (F11), runtime heap guard (F12), SD-only FRAME contract (F2).

## Deferred list (carry to the ring proposal / LAB-149)

Tracked as [#54](https://github.com/reiserlab/LED-Display_G6_Firmware_Arena/issues/54).

Dual committed headers with generations; linker-reserved region; 64-bit or epoch-tagged cursor;
frame generation identity for PSRAM/streaming; block schema version + sidecar header; T2 jitter
measurement with instruments.
