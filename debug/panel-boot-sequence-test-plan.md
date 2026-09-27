# Panel boot sequence + panel inventory — test plan

Covers, on branch `claude/panel-boot-sequence-n6rbg1`:

- **Power settle:** `panel_power_settle_ms` delay at the top of `SpiManager::begin()`.
- **Presence scan:** `COMM_CHECK` per panel at every boot (`PanelInventory::scanPresence`).
- **Fingerprint sweep:** `ISP_ENTER` + `ISP_VERIFY_CRC` per responding panel, run from
  `loop()` one panel per pass while the display is `ALL_OFF`. Auto-starts only on a cold
  power-on (`SRC_SRSR` = power-on reset and nothing else).
- **`GET_PANEL_INVENTORY` (0xCF):** host-readable result, gated on 0xC2 capability bit 6.
  Wire format: `src/PanelInventory.h`.

No panel-firmware change. Build with the non-`-performance` task to get the boot log:
`pixi run deploy-10-10` (or `deploy-12-18`), then `pixi run monitor-10-10`.

## What was verified off-hardware (this session)

PlatformIO and pixi downloads are blocked in the authoring sandbox, so nothing here has
been compiled for Teensy. Instead:

- `g++ -fsyntax-only -Wall -Wextra -Wformat=2` against stub Arduino/Teensy headers for every
  changed translation unit, in all four build configs (10-10 / 12-18 × debug / performance):
  no errors, no warnings from changed code.
- Host unit test of `PanelInventory` against a scripted fake `IspController`, both panel
  counts: paging (32 + 8 and 32 + 16), byte layout, flags, sweep state machine (reference /
  no reference / match / differ / ISP failure / absent), rescan cancelling a sweep,
  warm-boot flag, zero-panel case, and the boot-log text.
- `tests/test_panel_inventory.py`'s parser fed with pages dumped from the C++
  `buildPage()`: agrees byte-for-byte.

## 1. Build + regression

- [ ] `pixi run build-10-10` and `pixi run build-12-18` compile clean. **First real compile.**
- [ ] `pixi run deploy-10-10-performance`: boots, displays, streams normally.
- [ ] `pixi run test-serial` passes — the existing suite plus the 10 new tests in
      `tests/test_panel_inventory.py` (need at least one responding panel; the
      reference/0xC9 cross-checks skip without `/firmware/panel.bin` on SD).
- [ ] `g6-verify-panel` (0xC9) and `g6-program-panel` (0xC8) on one panel still work
      end-to-end. 0xC9 now runs through the shared `fingerprintPanel()`; its reply text
      should be unchanged.

## 2. Power settle (electrical)

- [ ] Scope a CS pin and SCK against the panel supply rail at power-up: both stay
      undriven for the full `panel_power_settle_ms`, then start.
- [ ] Power-cycle with the original (uncut) wiring: panels no longer enter the bad state.
      **This is the pass/fail bar for the delay.** If it still happens, capture the trace and
      raise `panel_power_settle_ms`.
- [ ] **Scenario B — Teensy on USB power, arena supply off** (the case a VUSB/VIN trace
      cut would address, if that's the trace in question): the Teensy boots, waits 500 ms,
      then drives CS into unpowered panels regardless. A fixed delay cannot cover this; it
      would need a panel-power sense before driving CS. Confirm whether this scenario is the
      original failure, so we know whether the delay alone is sufficient.

## 3. Presence scan

- [ ] Full arena: `[boot] panel scan: 40/40 panels responded` (48/48 on 12-18).
- [ ] Unplug one mid-chain panel: boot is not stalled, the log names that 1-based panel,
      and its bus-mate on the same CS line still reports present.
- [ ] Unplug several; then all (≈ panel_count × 50 ms, e.g. ~2 s for 40). Log reports
      `0/40 … no reply from: 1,2,3,4,5,6,7,8,...` and boot continues.

## 4. Fingerprint sweep

- [ ] **Cold power-on, homogeneous fleet, `panel.bin` on SD:** within a few seconds of boot
      the log shows `[boot] panel fw: all 40 panels identical, crc 0x… over N B (= SD panel.bin)`.
      Record the sweep duration (poll 0xCF for `fp_valid`, or stopwatch) — expected tens of
      ms per panel. If it's much longer, VERIFY_CRC's XIP CRC is slower than assumed.
- [ ] **Warm reset** (Teensy reflash, `SYSTEM_RESET` 0x01, reset button): the log shows
      `not a cold power-on, fingerprint sweep skipped` and 0xCF flags have bit 5. This checks
      the `SRC_SRSR` classification. If a real power-cycle is ever classified warm, the
      sweep just doesn't auto-run (safe); if a warm reset is ever classified cold, that is a
      bug to report.
- [ ] **Record the reset bits on a true cold boot.** The existing `=== SRC_SRSR (reset
      cause) = 0x… ===` boot line lists them. The rule in `main.cpp` assumes a power-up sets
      only `IPP_RESET_B`. If the Teensy's bootloader chip also asserts another bit at
      power-up (e.g. `IPP_USER_RESET_B`), every boot reads as warm and the sweep never
      auto-runs — drop that bit from the exclusion mask, after confirming a reflash still
      sets something that stays excluded.
- [ ] **One panel on different firmware** (flash one panel with an older image): it appears
      as its own `crc … (!= SD panel.bin): panels N` group, status 4 in 0xCF.
- [ ] **No `/firmware/panel.bin` on SD:** fingerprints report `(prefix; no SD panel.bin)`,
      `fp_len` = 65536, flag bit 4, per-panel status 5.
- [ ] **Panel on pre-ISP firmware** (if one exists): status 6, `no ISP reply` group; costs
      ~0.4 s of sweep time (the ENTER retry), doesn't stall anything else.
- [ ] 0xCF CRC for a panel == the CRC in that panel's 0xC9 reply
      (`test_fingerprint_agrees_with_verify_panel` automates this).

## 5. Side effects I could not rule out from this repo (panel firmware not accessible)

These are the reasons the sweep only auto-runs on a cold boot. Each needs a bench answer.

- [ ] **Display after the sweep:** after a cold-boot sweep completes, ALL_ON, streaming and
      SD pattern modes (2/3/4) look correct on every panel. The sweep leaves each panel after
      `ISP_VERIFY_CRC` without an exit command — exactly what 0xC9 already does in the
      field — but confirm no panel is stuck in an ISP state.
- [ ] **PSRAM-resident frames:** `ISP_ENTER` can fail with a "PSRAM alloc" status, i.e. it
      allocates a staging buffer in panel PSRAM. Load PSRAM frames (0x3A / 0x3B), run 0xCF
      action 2, then replay the PSRAM frames. If they're corrupted, host-triggered sweeps must
      be documented as destructive to PSRAM contents (or refused while frames are loaded).
- [ ] **Post-flash smiley:** flash a panel (0xC8), then power-cycle. Note whether the boot
      sweep retires the post-flash boot indicator (COMM_CHECK is exempt; ISP_ENTER may not
      be). Cosmetic, but changes what the bench sees after a flash.

## 6. Interaction with the running system

- [ ] Start ALL_ON mid-sweep (right after a cold boot): the display works, 0xCF shows the
      sweep paused (`fp_in_progress` still set). ALL_OFF: the sweep resumes and completes.
- [ ] Commands stay responsive during the sweep: 0xC2 round-trip stays within roughly one
      panel's fingerprint time (~0.1 s).
- [ ] 0xCF action 1/2 while the display runs → status 10 (`CE_DISPLAY_ACTIVE`); action 0
      still answers.
- [ ] SET_DIAG_OUTPUT muting still works for everything after boot; host-triggered sweeps
      print nothing (only the boot sweep writes its summary).

## Open items outside this repo

- `g6_03-controller.md` § Command Registry (docs repo `reiserlab/Modular-LED-Display`)
  needs the 0xCF entry and capability bit 6. Not accessible from the authoring session.
- Host decoder (webDisplayTools `js/arena-wire-g6.js`): add `GET_PANEL_INVENTORY: 0xcf`,
  `[6, 'panel_inventory']` to `CAPABILITY_BITS`, and a page decoder. Left to the Studio
  preflight work so the two efforts don't collide in that file.
- Branch merge note: the controller-health branches advertise capability 0xA3 (bit 7);
  merged with this branch it becomes 0xE3. That uses the last free bit in the byte.
