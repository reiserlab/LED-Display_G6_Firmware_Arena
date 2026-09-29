#pragma once
#include <Arduino.h>
#include "constants.h"

// SentinelPrint — boot-time text diagnostics that share the single USB-CDC
// pipe with binary command responses. Sentinel-prefixes every line so a host
// can demux it from [length, status, ...] response frames, the same
// convention DBG_PRINTF (constants.h) uses. Unlike DBG_PRINTF, this can't gate
// on g_dbg_on: that flag is only ever set by a host command (SET_DIAG_OUTPUT,
// 0xC3), so it's still at its power-on default during boot, before any host
// has had a chance to connect and set it. Diagnostics that must fire during
// boot itself (the SRC_SRSR/CrashReport dump, the panel presence sweep)
// therefore print unconditionally whenever DEBUG_SERIAL is compiled in,
// through this class, instead.
//
// Never blocks the boot sequence: no Serial.flush(), and each byte checks
// availableForWrite() first and silently drops instead of waiting on a
// full/absent host.
#ifdef DEBUG_SERIAL
class SentinelPrint : public Print {
 public:
  size_t write(uint8_t c) override {
    if (Serial.availableForWrite() < (at_line_start_ ? 2 : 1)) return 0;
    if (at_line_start_) {
      Serial.write(AC::constants::diag_line_sentinel);
      at_line_start_ = false;
    }
    Serial.write(c);
    if (c == '\n') at_line_start_ = true;
    return 1;
  }
  size_t write(const uint8_t *buffer, size_t size) override {
    size_t n = 0;
    for (size_t i = 0; i < size; ++i) n += write(buffer[i]);
    return n;
  }

 private:
  bool at_line_start_ = true;
};
#endif
