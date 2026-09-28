# Panel boot sequence + panel inventory — test plan

Covers, on branch `claude/panel-boot-sequence-n6rbg1`:

- **Power settle:** `panel_power_settle_ms` delay at the top of `SpiManager::begin()`.
- **Presence scan:** `COMM_CHECK` per panel at every boot (`PanelInventory::scanPresence`).
- **Fingerprint sweep:** `ISP_ENTER` + `ISP_VERIFY_CRC` per responding panel, run from
  `loop()` one panel per pass while the display is `ALL_OFF`. Starts at every boot.
- **`PANEL_INVENTORY_SCAN` (0xD0) / `GET_PANEL_INVENTORY` (0xD1):** host rescan / read, gated on
  0xC2 feature bit 0 (`panel_inventory`). Wire format: `src/PanelInventory.h`.

No panel-firmware change. Build with the non-`-performance` task to get the boot log:
`pixi run deploy-10-10` (or `deploy-12-18`, `deploy-2-10`), then `pixi run monitor-<variant>`.

## 1. Build + regression

- [x] `pixi run build-2-10`, `build-10-10` and `build-12-18` compile clean on `main` + #56.
- [ ] `pixi run deploy-<variant>-performance`: boots, displays, streams normally.
- [ ] `pixi run test-serial` passes: the existing suite plus `tests/test_panel_inventory.py`
      (needs at least one responding panel; the reference/0xC9 cross-checks skip without
      `/firmware/panel.bin` on SD). Tests that need Ethernet, a debug build or a 4×10 arena
      fail for those reasons on other rigs; compare against `main` on the same rig.
- [ ] `g6-verify-panel` (0xC9) and `g6-program-panel` (0xC8) on one panel still work
      end-to-end. 0xC9 now runs through the shared `fingerprintPanel()`; its reply text
      should be unchanged.
- [ ] **Watchdog (#56):** run `0xD0` action 1, then unplug a high-numbered panel before the
      sweep reaches it (it passed presence, so its ENTER/VERIFY_CRC step times out, ~3.4 s).
      No controller reset: `GET_HEALTH` still reports the original reset cause, and that
      panel ends as status 6.

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
      Record the sweep duration (poll 0xD1 for `fp_valid`, or stopwatch) — expected tens of
      ms per panel. If it's much longer, VERIFY_CRC's XIP CRC is slower than assumed.
- [ ] **Warm reset** (Teensy reflash, `SYSTEM_RESET` 0x01): the sweep runs again and the
      panels keep displaying normally afterwards (they stayed powered through the reset).
- [ ] **One panel on different firmware** (flash one panel with an older image): it appears
      as its own `crc … (!= SD panel.bin): panels N` group, status 4 in 0xD1.
- [ ] **No `/firmware/panel.bin` on SD:** fingerprints report `(prefix; no SD panel.bin)`,
      `fp_len` = 65536, flag bit 4, per-panel status 5.
- [ ] **Panel on pre-ISP firmware** (if one exists): status 6, `no ISP reply` group; costs
      ~0.4 s of sweep time (the ENTER retry), doesn't stall anything else.
- [ ] 0xD1 CRC for a panel == the CRC in that panel's 0xC9 reply
      (`test_fingerprint_agrees_with_verify_panel` automates this).

## 5. Panel-side behavior (answered from the panel firmware source)

Resolved by reading `LED-Display_G6_Firmware_Panel/panel/src/isp.cpp` and `isp_logic.h`;
the bench items confirm rather than discover.

- **ISP_ENTER has no side effects beyond arming a session.** The 2 MiB PSRAM staging buffer
  is reserved once in `Isp::init()` at panel boot, separately from the PSRAM frame store
  (`psram_store`, demo frames generated at boot); ENTER does not allocate. No display change.
- **The post-flash smiley survives.** `retires_boot_indicator()` exempts COMM_CHECK and every
  ISP opcode 0xE4-0xE9.
- **`appcrc` in the ENTER reply is hard-coded 0** (`TODO(bench)`), so it is not used.
- **VERIFY_CRC** CRCs XIP flash with a 4-bit table CRC-32; no bounds check on `len`
  (the controller only asks for <= image size or 64 KB).
- [ ] Confirm on the bench: after a sweep, ALL_ON / streaming / SD modes look correct on
      every panel; flash a panel (0xC8), power-cycle, and the smiley is still shown.
- [ ] Re-time one `g6-program-panel` flash: IspController now runs from flash (FLASHMEM).
      The stream should stay ~0.6 s; a large increase means the page loop is not staying
      in the I-cache.

## 6. Interaction with the running system

- [ ] Start ALL_ON mid-sweep (right after boot): the display works, 0xD1 shows the
      sweep paused (`fp_in_progress` still set). ALL_OFF: the sweep resumes and completes.
- [ ] Commands stay responsive during the sweep: 0xC2 round-trip stays within roughly one
      panel's fingerprint time (~0.1 s).
- [ ] 0xD0 (either action) while the display runs → status 10 (`CE_DISPLAY_ACTIVE`); 0xD1
      still answers.
- [ ] SET_DIAG_OUTPUT muting still works for everything after boot; host-triggered sweeps
      print nothing (only the boot sweep writes its summary).

## Open items outside this repo

- Host decoder (webDisplayTools `js/arena-wire-g6.js`): the 0xC2 feature bitmap, 0xD0/0xD1
  encoders and a page decoder. Coordinated with the Studio preflight work so the file isn't
  edited twice.
