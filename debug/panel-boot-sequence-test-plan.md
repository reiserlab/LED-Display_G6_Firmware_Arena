# Panel boot-sequence gentler-startup — test plan

Covers the phase-1 change: `panel_power_settle_ms` delay in `SpiManager::begin()`
plus the `CommandProcessor::logPanelBootScan()` fleet presence sweep (built on
`IspController::checkPanelPresent`, which wraps the existing `pollPanelAlive`
COMM_CHECK probe). No panel-firmware change, no wire-protocol change — this is
entirely bench/hardware verification of the arena side.

Build for bench testing with the non-`-performance` task so the boot report is
compiled in: `pixi run deploy-10-10` (or `deploy-12-18`), then
`pixi run monitor-10-10` to watch the boot text.

## 1. Regression — nothing else moved

- [ ] `pixi run build-10-10` and `pixi run build-12-18` both compile clean
      (this session couldn't invoke PlatformIO — first thing to confirm on
      the bench before anything else below).
- [ ] `pixi run deploy-10-10-performance`: board boots and displays/streams
      normally. Confirms `logPanelBootScan()` compiles to a true no-op outside
      `DEBUG_SERIAL` and adds no behavior change to the production build path.
- [ ] `pixi run test-serial` (full existing HIL suite) passes unchanged against
      a `deploy-10-10` (debug) build. The suite connects to an already-running
      board and never triggers a fresh boot, so this mainly confirms the new
      code didn't regress anything reachable from normal command processing
      (`isp_` still works for `G6_PROGRAM_PANEL_CMD`/`G6_VERIFY_PANEL_CMD`,
      SPI clock restore doesn't leave the bus in a bad state, etc).
- [ ] `g6-program-panel` (0xC8) and `g6-verify-panel` (0xC9) against one panel
      still work end-to-end — confirms `checkPanelPresent`'s save/restore of
      the SPI clock via `getSpiClockMhz()`/`setSpiClockMhz()` doesn't leave a
      stale clock behind for the ISP driver's other entry points.

## 2. The delay itself (electrical)

- [ ] Scope the target CS pin (any panel_sets[i].cs_pin) and SCK relative to
      the panel's own 3.3 V/5 V rail on power-up. Confirm CS/SCK stay at
      power-on-reset (floating/undriven) for the full `panel_power_settle_ms`
      window and only start toggling after it — i.e. the delay is actually
      landing before the first bus activity, not just before the log line.
- [ ] With the delay in place, power-cycle the controller with the previously
      problematic wiring restored (i.e. *without* the power-trace cut that
      was the field workaround) and confirm the panel(s) no longer glitch
      into the bad state. This is the actual pass/fail bar for the feature —
      everything else here is supporting verification.
- [ ] If it still glitches: capture the scope trace and increase
      `panel_power_settle_ms` (constants.h) — 500 ms was picked as a
      reasonable starting point, not measured against this board's actual
      panel power-on time. Re-test at the new value.
- [ ] Confirm no regression to overall boot-to-first-frame latency budget
      (whatever the lab's tolerance is) — the settle delay plus the presence
      sweep (below) are both extra, purely additive boot time versus today.

## 3. Presence sweep — populated arena (happy path)

- [ ] Full 40-panel arena_10-10 (or 48-panel arena_12-18), all panels present
      and powered. `pixi run monitor-10-10` across a power cycle shows:
      `[boot] panel scan: 40/40 panels responded` (or 48/48).
- [ ] Time the sweep (timestamps are already in every `DBG_PRINTF`/sentinel
      line via `[millis]`-style prefixing where used, or bracket it manually
      with a scope on the frame-scan gate pin / a stopwatch on the monitor
      log) — expect on the order of a few hundred ms for 40 populated panels
      at `panel_boot_scan_poll_ms=5`. If it's meaningfully slower, that's a
      signal `checkPanelPresent`'s per-panel round-trip is costing more than
      assumed and the constants need retuning.

## 4. Presence sweep — missing/absent panels

- [ ] Physically unplug one panel (mid-chain, not just the last one) before
      power-up. Confirm:
  - The controller does **not** hang waiting on it — total boot time stays
    bounded (roughly `panel_boot_scan_timeout_ms` extra for that one index,
    not a multi-second stall).
  - The log line correctly reports `N-1/N` and names the missing panel's
    1-based index (matches the physical position pulled).
  - Every OTHER panel still shows as responded — one absent panel doesn't
    cascade into false negatives on its bus-mate (each `panel_sets[]` entry
    gates two panels on independent SPI buses via one CS, see
    `ArenaConfig.h` — worth specifically confirming the bus-mate of the
    pulled panel still reports alive, since that's the one case where a
    wiring/addressing mistake would be easy to miss).
- [ ] Repeat with **all** panels absent (nothing plugged into the panel bus).
      Confirm the sweep completes in roughly
      `panel_count_per_frame * panel_boot_scan_timeout_ms` (≈2 s for 40
      panels at the current constants) rather than hanging, and reports
      `0/40 panels responded; no reply from: 1,2,3,...`.
- [ ] Repeat with two non-adjacent panels absent (e.g. panel 7 and panel 32)
      to confirm the missing-list formatting handles multiple entries
      correctly (comma-separated, no off-by-one on the 1-based numbering).

## 5. Interaction with existing behavior

- [ ] Confirm `blinkStartupPattern()`'s LED_BUILTIN/D13 blink still completes
      cleanly before `spi.begin()` — the new delay is additive time inside
      `spi.begin()`, after the blink, so this should be unaffected, but worth
      a visual check that the "OK" morse pattern still looks right (no
      unexpected interaction if someone later moves the delay).
- [ ] Confirm `SET_DIAG_OUTPUT` (0xC3) muting behavior is unaffected — the
      boot scan prints unconditionally on a `DEBUG_SERIAL` build (by design,
      since `g_dbg_on` isn't set yet at boot) but every *later* `DBG_PRINTF`
      line should still honor the runtime mute/unmute as before.
- [ ] Confirm a host (webDisplayTools Arena Console, or a plain serial
      terminal) that connects mid-sweep or right after boot doesn't
      misinterpret the sentinel-prefixed boot lines as a command response —
      this is the same demux convention already used for the SRC_SRSR/
      CrashReport dump, so this is mainly a "didn't regress the existing
      convention" check, not new risk.

## Known gaps / explicitly out of scope for this change

- No per-panel firmware-version/hash readback yet (the `ISP_ENTER` `appcrc`
  field discussed as the likely mechanism) — pending confirmation that
  entering/exiting ISP mode on every panel at every boot has no side effect
  worth paying for on every power-cycle (`ISP_EXIT_REBOOT`'s name suggests it
  reboots the panel, which would be an unacceptable per-boot cost across a
  full fleet if literal). Needs a look at the panel-firmware repo's
  `isp.cpp` before that phase is designed, not just this repo.
- No wire-protocol/host-visible surface for the scan result — it's a local
  serial log only, `DEBUG_SERIAL` builds only. Deferred until this is folded
  into the planned bidirectional-SPI/error-logging work so the wire format
  is designed once, not twice.
