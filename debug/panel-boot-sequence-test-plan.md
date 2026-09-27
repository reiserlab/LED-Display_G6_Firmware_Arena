# Panel boot sequence + panel inventory — test plan

Covers, on branch `claude/panel-boot-sequence-n6rbg1`:

- **Power settle:** `panel_power_settle_ms` delay at the top of `SpiManager::begin()`.
- **Presence scan:** `COMM_CHECK` per panel at every boot (`PanelInventory::scanPresence`).
- **Fingerprint sweep:** `ISP_ENTER` + `ISP_VERIFY_CRC` per responding panel, run from
  `loop()` one panel per pass while the display is `ALL_OFF`. Starts at every boot.
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
- [ ] **Warm reset** (Teensy reflash, `SYSTEM_RESET` 0x01): the sweep runs again and the
      panels keep displaying normally afterwards (they stayed powered through the reset).
- [ ] **One panel on different firmware** (flash one panel with an older image): it appears
      as its own `crc … (!= SD panel.bin): panels N` group, status 4 in 0xCF.
- [ ] **No `/firmware/panel.bin` on SD:** fingerprints report `(prefix; no SD panel.bin)`,
      `fp_len` = 65536, flag bit 4, per-panel status 5.
- [ ] **Panel on pre-ISP firmware** (if one exists): status 6, `no ISP reply` group; costs
      ~0.4 s of sweep time (the ENTER retry), doesn't stall anything else.
- [ ] 0xCF CRC for a panel == the CRC in that panel's 0xC9 reply
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

- [ ] Start ALL_ON mid-sweep (right after boot): the display works, 0xCF shows the
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
