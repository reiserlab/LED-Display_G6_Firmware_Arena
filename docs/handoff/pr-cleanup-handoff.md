# Handoff: open-PR merge / cleanup + panel inventory follow-through

Working doc for a local follow-up session. It records the decisions already made, the state
of every open PR, and the exact next steps. Delete this directory (`docs/handoff/`) once the
steps are done — it is scaffolding, not permanent documentation.

## Repos involved

| Repo | Role | Notes |
|---|---|---|
| `reiserlab/LED-Display_G6_Firmware_Arena` | Teensy 4.1 controller firmware — all PRs below | this file lives here |
| `reiserlab/Modular-LED-Display` | spec docs (`docs/development/g6_03-controller.md` = command registry) | spec change is a **patch in this directory**, not yet pushed |
| `reiserlab/LED-Display_G6_Firmware_Panel` | panel (RP2354) firmware | read-only reference for ISP behavior |
| `reiserlab/webDisplayTools` | Arena Studio host (`js/arena-wire-g6.js` decoders) | host changes follow the firmware |

Branch tips when this was written:

| Branch | SHA | PR |
|---|---|---|
| arena `main` | `d6358d2` | — |
| `feat/mode3-reliability` | `781efe2` | #56 |
| `follow-up/48-fix-nits` | `bf6da5c` | #57 (floesche) |
| `docs/windows-flash-notes` | `50ec2c4` | #49 |
| `claude/qwiic-i2c-validation-cac1ce` | `c05eb05` | #58 (stacked on #56) |
| `feat/ai-12bit-g3-gain` | `aae3768` | #46 |
| `feat/ai-calibration` | `9f37bcd` | #47 (stacked on #46) |
| `claude/panel-boot-sequence-n6rbg1` | `f92a0e9` | #59 (draft) |
| Modular-LED-Display `docs/qwiic-i2c-registry` | `0e2512f` | spec base for the patch |
| webDisplayTools `claude/controller-settings-block` | `47606b5` | Studio run preflight (another session) |

## Decisions already made

1. **Feature bitmap (approved).** The 0xC2 capability byte is full (bit 6 `ai_cal` from #47,
   bit 7 `health` from #56). New command families are advertised in a bitmap appended to the
   0xC2 reply after the MAC: payload byte 8 = count N (firmware sends 4), then N bytes, bit k
   = feature k. Presence is signalled by payload length, like the MAC — no capability bit.
   Initial bits: 0 `panel_inventory`, 1 `qwiic_i2c`, 2 `ai_stream` (reserved). Bits are never
   reused; nothing already shipped moves. The 0xCB `flags` byte keeps gating 0xCC–0xCE and
   takes no new assignments. Host decoders already tolerate longer 0xC2 replies
   (`decodeControllerInfo` checks `>= 2` / `>= 8`).
2. **Opcode allocation (approved).** Allocate by functional block; a set takes the even code
   and its get the next odd code (get-only → odd, set-only/action → even); never use a
   G4-reserved code; every new family ships with its feature bit. Legacy irregular pairs are
   grandfathered. New **0xD_ panel-fleet block**: 0xD0 `panel-inventory-scan`, 0xD1
   `get-panel-inventory`; 0xD2–0xDF reserved for panel diagnostics (e.g. bidirectional-SPI
   error counters/logs). 0xCF stays spare. Analog-in block stream → 0xAE/0xAF if two codes
   suffice.
3. **#56 merges first** — "plenty of lab testing"; lab-test evidence to be attached in the
   PR when merging.
4. **#49 is superseded by #57** — close it (see step 3).

The full spec text for 1 and 2 is `g6_03-feature-bitmap-0xD0.patch` in this directory
(one commit, `bb0d641`, on top of Modular-LED-Display `docs/qwiic-i2c-registry` @ `0e2512f`).
It could not be pushed from the cloud session (repo not in its authorized set).

## Step-by-step plan

### Step 0 — push the spec patch (Modular-LED-Display)

```
cd Modular-LED-Display
git fetch origin
git switch -c claude/panel-inventory-feature-bitmap origin/docs/qwiic-i2c-registry
git am ../LED-Display_G6_Firmware_Arena/docs/handoff/g6_03-feature-bitmap-0xD0.patch
git push -u origin claude/panel-inventory-feature-bitmap
```
Open a draft PR against `docs/qwiic-i2c-registry` (that branch stacks on
`docs/g6_03-analog-io`). The patch only touches `docs/development/g6_03-controller.md`. It
follows that repo's `CLAUDE.md` (current truth, no dated notes): it *replaces* the
"cap_ext vs health bit-7 conflict" note rather than annotating it.

### Step 1 — merge #56 (Mode-3 reliability)

- Attach the lab-test evidence to the PR; its body says "merge after the lab day is clean".
- floesche is the requested reviewer: get the review or note the waiver.
- `mergeable_state` was clean against `main`. Nothing else has to land first.

### Step 2 — rebase and merge #57 (tooling, floesche)

Conflicts with post-#56 `main`: 3 hunks, all mechanical.
- `pixi.toml`: #56 adds `monitor-2-10` as the old bash+tee one-liner → rewrite as
  `monitor-2-10 = "python scripts/monitor.py -e teensy41-2-10"` like #57's other monitor tasks.
- `platformio.ini`: #57 deletes `scripts/find_teensy.py` and its `extra_scripts = pre:…`
  hook in `env:teensy41-base`; keep the deletion (the 2-10 envs extend base).
- `src/SpiManager.cpp`: comment only ("all 20 CS lines" → `panel_set_count`).
Check: `pixi run build-2-10`, `build-10-10`, `build-12-18`; `pixi run monitor-2-10` finds the port.

### Step 3 — close #49 (Windows flash docs)

Superseded by #57. #57's CLAUDE.md states `--upload-port` "is ignored by the teensy-cli protocol
on every OS" (contradicting #49's main advice) and already covers "first attempt fails with
`error writing to Teensy`, re-run". Before closing, confirm #57 keeps #49's two remaining
facts — the loader waits until the arena is powered; a held COM port blocks the soft reboot —
and add them to #57's gotcha if missing.

### Step 4 — #58 Qwiic I2C bridge

Retarget from `feat/mode3-reliability` to `main` after step 1 (3 commits, additive). Fixes
before merge:
- **Watchdog:** `GET_I2C_SCAN` (0xB0) / `I2C_TRANSFER` (0xB1) run Wire1 synchronously with no
  application-level timeout. On a stuck bus (no on-board pull-ups; long cables planned) a
  112-address scan can exceed #56's 2 s watchdog → controller reset. Either set a Wire1
  timeout and bound the scan well under 2 s, or add 0xB0/0xB1 to #56's `long_op` list in
  `handleBinaryCommand` (the `watchdogSuspend()`/`watchdogResume()` window). Bench: short SDA
  to GND during a scan.
- **Feature bit 1 (`qwiic_i2c`)** in the new 0xC2 feature bitmap, replacing "hosts probe
  0xB0" (probing flashes a CE 01 glyph on firmware without the bridge). Whichever of #58/#59
  lands first implements the bitmap itself (see step 6).

### Step 5 — #46 → #47 analog-in

Rebase onto `main` after step 1. #46 × #58 = 1 hunk; #47 × #56 = 4 hunks
(`CommandProcessor.cpp`, `commands.h`, `constants.h` capability line, `tests/commands.py`).
- #47 **keeps** 0xC2 bit 6 `ai_cal`. Merged capability value with #56: `0xE3`
  (bits 0,1,5,6,7).
- The future analog block stream can no longer use 0xA8/0xA9 (#56 telemetry). Per the spec
  patch: 0xAE/0xAF, feature bit 2.
- `SET_ANALOG_CAL` (0xA6) refuses while the display runs but not during an in-flight SD
  transfer, yet it writes `/config/analog_cal.json`. Add the `dl_active_ || ul_active_ ||
  ar_active_` guard that 0xC8/0xC9 use.
- Neither PR has run on hardware; its bench items (T1–T3, C1–C3, CL1/CL2) gate the merge.

### Step 6 — #59 panel boot + inventory (draft)

Bench-test **now, on its current branch** (independent of the merge order), using
`debug/panel-boot-sequence-test-plan.md`. The code has **never been compiled for Teensy** —
`pixi run build-10-10` / `build-12-18` first; record the RAM1 "code" / "padding" figures that
`teensy_size` prints.

After step 1, rebase onto `main` and make these changes:
1. **Renumber** 0xCF → 0xD0 `PANEL_INVENTORY_SCAN` `[02 D0 action]` (0 = presence, 1 =
   presence + fingerprints; replies with page 0 of 0xD1) and 0xD1 `GET_PANEL_INVENTORY`
   `[01 D1]` / `[02 D1 first]`. Today's 0xCF actions map as 1 → D0/0, 2 → D0/1, 0 → D1.
   Files: `src/commands.h`, `CommandProcessor.cpp` (dispatch + `handleGetPanelInventory`),
   `src/PanelInventory.h` wire-format comment, `tests/commands.py`,
   `tests/test_panel_inventory.py`.
2. **Capability:** revert `controller_capability_bitmap` to `main`'s value (drop bit 6 —
   it is `ai_cal`). Implement the feature bitmap in `handleGetControllerInfo()`: append
   `[4, f0, f1, f2, f3]` after the MAC with bit 0 set (response grows 8 → 13 payload bytes,
   frame length 0x0F). Update the pytest capability test to read feature bit 0.
3. **Watchdog (#56):** add 0xD0 to the `long_op` list (presence rescan up to ~2.4 s with
   panels absent), and bracket each `fingerprintStep()` in `CommandProcessor::serviceInventory()`
   with `Health::watchdogSuspend()` / `watchdogResume()` (a step on an unresponsive panel can
   reach ~3.4 s: 400 ms ENTER retry + 3 s VERIFY_CRC poll). Consider a `Health::mark()` op code
   for the breadcrumb.
4. Conflict with #56 was 5 hunks (`CommandProcessor.cpp`, `commands.h`, `constants.h`,
   `main.cpp` ×1, `tests/commands.py`). `main.cpp` no longer touches `SRC_SRSR` (the sweep runs
   every boot), so there is no ordering issue with `Health::begin()` clearing that register.
5. Re-run the host checks (below), then the HIL suite, then mark ready for review.

### Step 7 — host side (webDisplayTools)

After steps 0 and 6: decode the 0xC2 feature bitmap (`FEATURE_BITS` table next to
`CAPABILITY_BITS`), add encoders/decoders for 0xD0/0xD1 (export + golden test per that repo's
CLAUDE.md "Wire module exports" rule), gate 0xB0/0xB1 on `qwiic_i2c`. The Studio preflight
session (`claude/controller-settings-block`) wants per-panel firmware from 0xD1 to flag a
mismatched panel — coordinate there so `arena-wire-g6.js` isn't edited twice.

## Findings to carry forward

**Panel firmware (verified from source, `panel/src/isp.cpp`, `isp_logic.h`):**
- `ISP_ENTER` only arms a session (nonce, `armed_`); the 2 MiB PSRAM staging buffer is
  `pmalloc`'d once in `Isp::init()` at panel boot, separate from the ~20 KB `psram_store`
  frame store (demo frames generated at boot; no upload path). No display change.
- ISP opcodes 0xE4–0xE9 and COMM_CHECK are exempt from `retires_boot_indicator()` — the
  post-flash smiley survives the sweep.
- ENTER's `appcrc` field is hard-coded 0 (`TODO(bench)`) — do not use it.
- `ISP_VERIFY_CRC` = 4-bit-table CRC-32 over XIP flash `[start, start+len)`, no bounds check;
  requires the ENTER nonce. `ISP_EXIT_REBOOT` really reboots — never sent by the arena.

**Memory (from webDisplayTools design docs; confirm with a real build):** RAM1 (ITCM code +
DTCM data/stack) had ~132 KB free before #56 and ~97 KB after. On Teensy 4 all code not marked
`FLASHMEM` runs from ITCM, which is carved from RAM1 in 32 KB banks — crossing a bank boundary
costs 32 KB of DTCM at once. RAM2 has ~390 KB unused. #59 marks `IspController` +
`PanelInventory` `FLASHMEM` (net RAM1 saving vs `main`). Worth adopting for other cold paths
(SD archive, calibration, I2C bridge).

**Pairwise conflict matrix at the SHAs above** (hunks; "stacked" = one contains the other):

| | #47 | #49 | #56 | #57 | #58 | #59 |
|---|---|---|---|---|---|---|
| **#46** | stacked | 0 | 0 | 0 | 1 | 1 |
| **#47** | | 0 | 4 | 0 | 5 | 3 |
| **#49** | | | 0 | 1 | 0 | 0 |
| **#56** | | | | 3 | stacked | 5 |
| **#57** | | | | | 3 | 0 |
| **#58** | | | | | | 5 |

## Verification status of #59

Done off-hardware (cloud session, no Teensy toolchain — PlatformIO/pixi downloads blocked):
`g++ -fsyntax-only` with stub Arduino headers for every changed file × {10-10, 12-18} ×
{debug, performance} × {C++14, C++17}; a host unit test of `PanelInventory` against a fake
`IspController`; the pytest parser checked byte-for-byte against `buildPage()` output. Not
done: any real compile, any hardware run. The stubs/unit test were scratch files and are not
in the repo; a local session with pixi should rely on `pixi run build-*` and `pixi run test`.

## Open questions for Michael

- Which power trace was cut? If it is the Teensy's VUSB–VIN link, the original failure was a
  USB-powered Teensy driving CS into an unpowered arena, which a fixed 500 ms delay cannot
  cover (test plan "Scenario B").
- Post review comments on other authors' PRs (#57 floesche; #58/#47 fixes), or hand the notes
  over directly?
