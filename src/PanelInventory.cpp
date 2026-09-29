#include "PanelInventory.h"

// sendResponse() silently drops a payload that doesn't fit (3 framing bytes +
// payload must stay under the response buffer), so guard the page size here.
static_assert(PanelInventory::kPageBytesMax + 3 < AC::constants::byte_count_per_response_max,
              "a GET_PANEL_INVENTORY page must fit one response frame");

static void putU32(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

FLASHMEM void PanelInventory::scanPresence() {
  present_count_ = 0;
  for (uint8_t i = 0; i < kPanels; ++i) {
    if (!isp_.checkPanelPresent(i)) {
      status_[i] = kAbsent;
      crc_[i] = 0;
      continue;
    }
    ++present_count_;
    if (status_[i] == kAbsent || status_[i] == kUnknown) status_[i] = kPresent;
  }
  // A panel that reappeared behind the sweep cursor would otherwise stay
  // kPresent until the next action 1; fingerprinted panels are skipped anyway.
  if (fp_active_) fp_next_ = 0;
  presence_ms_ = millis();
  ++scan_id_;
  flags_ |= kFlagPresenceValid;
}

FLASHMEM void PanelInventory::startFingerprints(bool log_when_done) {
  for (uint8_t i = 0; i < kPanels; ++i) {
    if (status_[i] != kAbsent && status_[i] != kUnknown) status_[i] = kPresent;
    crc_[i] = 0;
  }
  flags_ = (uint8_t)((flags_ & ~(kFlagFpValid | kFlagRefPresent | kFlagFpPrefix)) |
                    kFlagFpInProgress);
  ref_crc_        = 0;
  fp_len_         = 0;
  fp_next_        = 0;
  fp_ref_pending_ = true;
  fp_active_      = true;
  log_when_done_  = log_when_done;
}

FLASHMEM bool PanelInventory::fingerprintStep() {
  if (!fp_active_) return false;

  if (fp_ref_pending_) {
    fp_ref_pending_ = false;
    const char *err = "";
    if (isp_.readReferenceFooter(&ref_crc_, &fp_len_, &err)) {
      flags_ |= kFlagRefPresent;
    } else {
      ref_crc_ = 0;
      fp_len_  = AC::constants::panel_fingerprint_prefix_bytes;
      flags_  |= kFlagFpPrefix;
    }
    return false;
  }

  while (fp_next_ < kPanels && status_[fp_next_] != kPresent) ++fp_next_;
  if (fp_next_ < kPanels) {
    const uint8_t i = fp_next_++;
    uint32_t crc = 0;
    bool match = false;
    const char *err = "";
    if (isp_.fingerprintPanel(i, fp_len_, ref_crc_, &crc, &match, &err)) {
      crc_[i] = crc;
      status_[i] = (flags_ & kFlagRefPresent) ? (match ? kFwMatch : kFwDiffers)
                                               : kFwNoReference;
    } else {
      status_[i] = kFwFailed;
      DBG_PRINTF("[inv] panel %u fingerprint failed: %s\n", (unsigned)(i + 1), err);
    }
    return false;
  }

  fp_active_ = false;
  flags_ = (uint8_t)((flags_ & ~kFlagFpInProgress) | kFlagFpValid);
  return true;
}

FLASHMEM size_t PanelInventory::buildPage(uint8_t first, uint8_t *out) const {
  uint8_t n = 0;
  if (first < kPanels) {
    n = (uint8_t)(kPanels - first);
    if (n > kPageEntryMax) n = kPageEntryMax;
  }
  const uint32_t age_ms =
      (flags_ & kFlagPresenceValid) ? (uint32_t)(millis() - presence_ms_) : 0;

  size_t o = 0;
  out[o++] = kVersion;
  out[o++] = kPanels;
  out[o++] = flags_;
  out[o++] = first;
  out[o++] = n;
  putU32(out + o, ref_crc_); o += 4;
  putU32(out + o, fp_len_);  o += 4;
  putU32(out + o, age_ms);   o += 4;
  out[o++] = scan_id_;
  for (uint8_t k = 0; k < n; ++k) {
    out[o++] = status_[first + k];
    putU32(out + o, crc_[first + k]);
    o += 4;
  }
  return o;
}

template <typename Pred>
FLASHMEM void PanelInventory::printPanelList(Print &out, Pred member) const {
  static constexpr uint8_t kListMax = 8;
  uint8_t listed = 0;
  for (uint8_t i = 0; i < kPanels; ++i) {
    if (!member(i)) continue;
    if (listed == kListMax) { out.print(",..."); return; }
    out.printf(listed ? ",%u" : "%u", (unsigned)(i + 1));
    ++listed;
  }
}

FLASHMEM void PanelInventory::printPresence(Print &out) const {
  out.printf("[boot] panel scan: %u/%u panels responded",
             (unsigned)present_count_, (unsigned)kPanels);
  if (present_count_ < kPanels) {
    out.print("; no reply from: ");
    printPanelList(out, [this](uint8_t i) { return status_[i] == kAbsent; });
  }
  out.println();
}

FLASHMEM void PanelInventory::printFingerprints(Print &out) const {
  uint8_t fingerprinted = 0, failed = 0, distinct = 0;
  uint32_t first_crc = 0;
  for (uint8_t i = 0; i < kPanels; ++i) {
    if (status_[i] == kFwFailed) ++failed;
    if (!isFingerprinted(status_[i])) continue;
    ++fingerprinted;
    bool seen = false;
    for (uint8_t j = 0; j < i && !seen; ++j) {
      seen = isFingerprinted(status_[j]) && crc_[j] == crc_[i];
    }
    if (!seen && distinct++ == 0) first_crc = crc_[i];
  }

  if (fingerprinted == 0 && failed == 0) {
    out.println("[boot] panel fw: no responding panels to fingerprint");
    return;
  }

  const bool ref = (flags_ & kFlagRefPresent) != 0;
  auto refNote = [&](uint32_t crc) {
    return ref ? (crc == ref_crc_ ? " (= SD panel.bin)" : " (!= SD panel.bin)")
               : " (prefix; no SD panel.bin)";
  };

  if (distinct == 1 && failed == 0) {
    out.printf("[boot] panel fw: all %u panels identical, crc 0x%08lX over %lu B%s\n",
               (unsigned)fingerprinted, (unsigned long)first_crc,
               (unsigned long)fp_len_, refNote(first_crc));
    return;
  }

  out.printf("[boot] panel fw: %u distinct fingerprint(s) over %lu B, %u ISP failure(s)\n",
             (unsigned)distinct, (unsigned long)fp_len_, (unsigned)failed);
  for (uint8_t i = 0; i < kPanels; ++i) {
    if (!isFingerprinted(status_[i])) continue;
    bool seen = false;
    uint8_t count = 0;
    for (uint8_t j = 0; j < kPanels; ++j) {
      if (!isFingerprinted(status_[j]) || crc_[j] != crc_[i]) continue;
      if (j < i) { seen = true; break; }
      ++count;
    }
    if (seen) continue;
    const uint32_t crc = crc_[i];
    out.printf("[boot]   crc 0x%08lX x%u%s: panels ", (unsigned long)crc, (unsigned)count,
               refNote(crc));
    printPanelList(out, [this, crc](uint8_t k) {
      return isFingerprinted(status_[k]) && crc_[k] == crc;
    });
    out.println();
  }
  if (failed) {
    out.printf("[boot]   no ISP reply x%u: panels ", (unsigned)failed);
    printPanelList(out, [this](uint8_t k) { return status_[k] == kFwFailed; });
    out.println();
  }
}
