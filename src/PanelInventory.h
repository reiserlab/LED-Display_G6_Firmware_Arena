#pragma once

#include <Arduino.h>
#include "IspController.h"
#include "constants.h"

// PanelInventory — per-panel presence + firmware fingerprint for the fleet,
// filled in the late boot step (CommandProcessor::serviceLateBootBlank) and
// served by GET_PANEL_INVENTORY (0xD1). A host that starts a display before
// that step cancels the boot scan and owns the inventory from then on
// (PANEL_INVENTORY_SCAN 0xD0 when idle); 0xD1 reports presence_valid clear
// until it does.
//
// Presence: COMM_CHECK liveness probe per panel (IspController::
// checkPanelPresent). Fast (~7 ms per responding panel, ~50 ms per absent
// one), no ISP mode, no display-state change.
//
// Fingerprint: ISP_ENTER + ISP_VERIFY_CRC per responding panel
// (IspController::fingerprintPanel) — the same exchange g6-verify-panel
// (0xC9) already uses in the field; no ISP_EXIT_REBOOT, no reboot. Covers
// [0, image_size) of the panel's running app flash when /firmware/panel.bin
// is on SD (so the CRC is directly comparable to that image), else a
// panel_fingerprint_prefix_bytes prefix. Tens of ms per panel, so it runs one
// panel per loop() pass (CommandProcessor::serviceInventory), only while the
// display is ALL_OFF and no SD transfer is in flight; a running display just
// pauses it. Safe on a panel that stayed powered through a
// controller-only reset: ENTER only arms a session (the PSRAM staging buffer
// is reserved once at panel boot, Isp::init, apart from the PSRAM frame
// store), and ISP opcodes don't retire the post-flash smiley
// (retires_boot_indicator, panel isp_logic.h).
//
// WIRE FORMAT (g6_03 § 0xD0 / 0xD1). Gate both on 0xC2 feature bit 0.
//   PANEL_INVENTORY_SCAN (0xD0)  [02 D0 action]
//     action 0 = rescan presence (blocking), then reply. Panels still present
//                keep their fingerprint; panels that went absent lose it; a
//                panel that (re)appears reads status 2 until the next action 1.
//            1 = rescan presence, restart the fingerprint sweep (all
//                fingerprints discarded), reply at once (poll 0xD1 until
//                flags.fp_valid)
//     Requires ALL_OFF (else status CE_DISPLAY_ACTIVE = 10) and no SD transfer
//     (else status 1). Reply: echo 0xD0, the page payload below for first = 0.
//   GET_PANEL_INVENTORY (0xD1)  [01 D1] | [02 D1 first]
//     Read the stored inventory; no panel traffic, always allowed.
//     first = 0-based index of the page's first panel (default 0). Entry k
//             of the page is panel NUMBER first+k+1 — the 1-based numbering
//             0xC8/0xC9 and the panel map use.
//   Page payload (status 0), all multi-byte fields little-endian:
//     [0]      version = 1
//     [1]      panel_count          panels this build drives
//     [2]      flags                see Flags below
//     [3]      first                echo of the requested page start
//     [4]      n                    entries in this page (<= 32; 0 past the end)
//     [5..8]   ref_crc32  u32       /firmware/panel.bin CRC (0 if none)
//     [9..12]  fp_len     u32       app-flash bytes each fingerprint covers
//     [13..16] age_ms     u32       ms since the presence scan finished
//     [17]     scan_id    u8        changes on every presence scan (boot or
//                                   0xD0, either action); wraps. Pages of one
//                                   read must agree, else re-read.
//     [18..]   n x { status u8 (PanelStatus), crc32 u32 (0 unless fingerprinted) }
//   Max 18 + 32*5 = 178 B, under byte_count_per_response_max; 40- and
//   48-panel builds read in two pages (first = 0, then 32).
class PanelInventory {
 public:
  enum PanelStatus : uint8_t {
    kUnknown       = 0,  // not scanned
    kAbsent        = 1,  // no COMM_CHECK reply (missing, unpowered, miswired)
    kPresent       = 2,  // replied; not fingerprinted (yet, or sweep skipped)
    kFwMatch       = 3,  // fingerprint == /firmware/panel.bin
    kFwDiffers     = 4,  // fingerprint != /firmware/panel.bin
    kFwNoReference = 5,  // fingerprinted; no SD reference to compare against
    kFwFailed      = 6,  // replied to COMM_CHECK but not to ISP (e.g. pre-ISP firmware)
  };

  enum Flags : uint8_t {
    kFlagPresenceValid  = 0x01,  // a presence scan has completed
    kFlagFpValid        = 0x02,  // the last fingerprint sweep completed (panels that appeared
                                 // in a later presence-only rescan still read kPresent)
    kFlagFpInProgress   = 0x04,  // sweep running, or paused while the display runs
    kFlagRefPresent     = 0x08,  // sweep compared against /firmware/panel.bin
    kFlagFpPrefix       = 0x10,  // no reference: CRCs cover a fixed prefix only
    // 0x20 reserved
  };

  static constexpr uint8_t kVersion      = 1;
  static constexpr uint8_t kPageEntryMax = 32;
  static constexpr uint8_t kHeaderBytes  = 18;
  static constexpr uint8_t kEntryBytes   = 5;
  static constexpr size_t  kPageBytesMax = kHeaderBytes + kPageEntryMax * kEntryBytes;

  explicit PanelInventory(IspController &isp) : isp_(isp) {}

  // Probe every panel. Panels still present keep their fingerprint result;
  // panels that went absent lose it. A running sweep continues and also
  // covers panels that (re)appeared.
  void scanPresence();

  // (Re)arm the fingerprint sweep over the panels the last presence scan
  // found, discarding every previous fingerprint. The SD reference is read on
  // the first fingerprintStep(), so this is safe to call before the SD card
  // is mounted.
  void startFingerprints(bool log_when_done);

  bool fingerprintActive() const { return fp_active_; }
  bool presenceValid() const { return flags_ & kFlagPresenceValid; }
  bool anyAbsent() const {
    for (uint8_t i = 0; i < kPanels; ++i) if (status_[i] == kAbsent) return true;
    return false;
  }
  bool logWhenDone() const { return log_when_done_; }

  // One unit of sweep work (reference read, or one panel). Returns true on
  // the call that completes the sweep.
  bool fingerprintStep();

  // Fill `out` (>= kPageBytesMax bytes) with the 0xD0/0xD1 page payload for the
  // page starting at `first`; returns the payload length.
  size_t buildPage(uint8_t first, uint8_t *out) const;

  void printPresence(Print &out) const;
  void printFingerprints(Print &out) const;

 private:
  static constexpr uint8_t kPanels = AC::constants::panel_count_per_frame;

  IspController &isp_;
  uint8_t  status_[kPanels] = {};
  uint32_t crc_[kPanels]    = {};
  uint8_t  flags_           = 0;
  uint8_t  scan_id_         = 0;
  uint8_t  present_count_   = 0;
  uint32_t presence_ms_     = 0;
  uint32_t ref_crc_         = 0;
  uint32_t fp_len_          = 0;
  uint8_t  fp_next_         = 0;
  bool     fp_active_       = false;
  bool     fp_ref_pending_  = false;
  bool     log_when_done_   = false;

  static bool isFingerprinted(uint8_t s) {
    return s == kFwMatch || s == kFwDiffers || s == kFwNoReference;
  }
  // Comma-separated 1-based numbers of the panels matching `member`, capped.
  template <typename Pred> void printPanelList(Print &out, Pred member) const;
};
