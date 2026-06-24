#include "SpiManager.h"
#include "G6PanelProtocol.h"

using namespace AC;
using namespace AC::constants;

SpiManager   *SpiManager::instance_     = nullptr;
volatile bool SpiManager::dmaComplete_  = false;

void SpiManager::dmaISR(EventResponderRef) {
  dmaComplete_ = true;
}

void SpiManager::refreshISR() {
  if (instance_) {
    instance_->refreshFlag = true;
    instance_->isr_count_++;
  }
}

void SpiManager::begin() {
  instance_ = this;

  dmaEvent_.attachImmediate(dmaISR);

  // Bring up both SPI buses. Explicitly route each region's CIPO pin to the
  // peripheral's SDI (MISO) input *before* begin() — otherwise the LPSPI
  // samples a different/unconnected SDI and reads the panel's confirmation as
  // all-zeros, even though the panel drives CIPO on this pin (confirmed on a
  // logic analyzer). setMISO() must precede begin().
  for (uint8_t r = 0; r < region_count_per_frame; ++r) {
    region_spi_[r]->setMISO(region_cipo_pins[r]);
    region_spi_[r]->begin();
  }

  // Drive every CS line HIGH (deselected) before any transaction can run.
  for (uint8_t i = 0; i < panel_set_count; ++i) {
    pinMode(panel_sets[i].cs_pin, OUTPUT);
    digitalWriteFast(panel_sets[i].cs_pin, HIGH);
  }

  // Hold the 2nd pair of per-column MISO OE-decode inputs HIGH. Without this
  // they float (≈low), the OE̅ = CS0&CS1&CS2&CS3 AND can never reach all-HIGH,
  // and every column's buffer stays enabled — shorting the wired-OR MISO bus so
  // CIPO reads 00. Tied HIGH, OE̅ = CS_row0 & CS_row1, so one buffer per bus
  // drives at a time. See ArenaConfig.h / arena-hardware-bug.md.
  for (uint8_t i = 0; i < cs_decode_tie_high_count; ++i) {
    pinMode(cs_decode_tie_high_pins[i], OUTPUT);
    digitalWriteFast(cs_decode_tie_high_pins[i], HIGH);
  }
}

void SpiManager::armRefreshTimer(uint32_t frequency_hz) {
  if (frequency_hz == 0) return;
  uint32_t period_us = microseconds_per_second / frequency_hz;
  refreshTimer_.begin(refreshISR, period_us);
}

void SpiManager::disarmRefreshTimer() {
  refreshTimer_.end();
  refreshFlag = false;
}

void SpiManager::setSpiClockMhz(uint16_t mhz) {
  if (mhz < 1)  mhz = 1;
  if (mhz > 30) mhz = 30;
  spi_clock_hz_      = (uint32_t)mhz * 1'000'000UL;
  cs_setup_delay_ns_ = (uint32_t)((1'000'000'000ULL * cs_setup_sck_periods) / spi_clock_hz_);
  cs_hold_delay_ns_  = (uint32_t)((1'000'000'000ULL * cs_hold_sck_periods)  / spi_clock_hz_);
#ifdef DEBUG_SERIAL
  cipo_realign_bits_ = (spi_clock_hz_ >= 20'000'000UL) ? 1 : 0;
#endif
}

void SpiManager::beginPanelSetTransaction() {
  SPISettings settings(spi_clock_hz_, spi_bit_order, spi_data_mode);
  for (uint8_t r = 0; r < region_count_per_frame; ++r) {
    region_spi_[r]->beginTransaction(settings);
  }
}

void SpiManager::endPanelSetTransaction() {
  for (uint8_t r = 0; r < region_count_per_frame; ++r) {
    region_spi_[r]->endTransaction();
  }
}

void SpiManager::transferPanelSet(const uint8_t *block_b0,
                                  const uint8_t *block_b1,
                                  uint16_t block_byte_count,
                                  uint8_t *miso_b0,
                                  uint8_t *miso_b1) {
#ifdef DEBUG_SERIAL
  // Debug CIPO capture: when a MISO buffer is requested, use blocking
  // full-duplex transfers so the received bytes are read into the buffers by
  // the CPU. The async-DMA path (below) does NOT reliably populate a regular
  // RAM buffer on Teensy 4 — an eDMA write into a cached/TCM buffer is not
  // coherent with the CPU read, so the readback returns the stale memset
  // zeros. Sequential transfers inside the shared CS window are fine for a
  // 1-in-300-frames diagnostic. Production passes nullptr and never hits this.
  if (miso_b0 != nullptr || miso_b1 != nullptr) {
    region_spi_[0]->transfer(const_cast<uint8_t *>(block_b0),
                             miso_b0, block_byte_count);
    region_spi_[1]->transfer(const_cast<uint8_t *>(block_b1),
                             miso_b1, block_byte_count);
    return;
  }
#endif

  // Async DMA on region 0 (SPI) — non-const cast is safe; SPI driver only
  // reads the buffer when retbuf is nullptr.
  dmaComplete_ = false;
  region_spi_[0]->transfer(const_cast<uint8_t *>(block_b0),
                           miso_b0, block_byte_count, dmaEvent_);

  // Blocking transfer on region 1 (SPI1) runs simultaneously with the DMA.
  region_spi_[1]->transfer(const_cast<uint8_t *>(block_b1),
                           miso_b1, block_byte_count);

  // Wait for region 0 DMA to drain before releasing CS.
  while (!dmaComplete_) { /* spin */ }
}

#ifdef DEBUG_SERIAL
// Recover the CIPO confirmation when the buffered return path delays MISO by a
// whole bit at high SCK (see constants::cipo_realign_bits_). Left-shifts the
// captured byte stream by `bits` (0..7), pulling in the MSBs of the following
// byte; `raw` must hold at least n+1 valid bytes. bits==0 is a plain copy.
static inline void realignCipo(const uint8_t *raw, uint8_t *out,
                               uint8_t n, uint8_t bits) {
  if (bits == 0) {
    for (uint8_t k = 0; k < n; ++k) out[k] = raw[k];
    return;
  }
  for (uint8_t k = 0; k < n; ++k) {
    out[k] = (uint8_t)((raw[k] << bits) | (raw[k + 1] >> (8 - bits)));
  }
}
#endif

void SpiManager::transferFrame(const uint8_t *frame_buf,
                               uint16_t block_byte_count) {
  if (frame_buf == nullptr) return;
  if (block_byte_count != G6::block_byte_count_gs2 &&
      block_byte_count != G6::block_byte_count_gs16 &&
      block_byte_count != G6::block_byte_count_psram &&
      block_byte_count != G6::block_byte_count_psram_duty) {
    return;
  }
  ++frames_sent_;

#ifdef DEBUG_SERIAL
  uint32_t t0 = micros();
#endif

  const uint8_t *blocks_base = frame_buf + stream_frame_prefix_byte_count;

#ifdef DEBUG_SERIAL
  // Capture every panel set's CIPO on the frames we print (every 300th); other
  // frames use the fast async path. Capturing ALL sets — not just set 0 — is
  // what makes an edge/reworked port visible: e.g. silk P3 is panel set 4
  // (CS pin 9), not set 0 (P1, CS pin 0).
  // Gate the blocking full-duplex capture on the runtime diag flag: when
  // diagnostics are muted, every frame takes the fast async-DMA path, so a
  // DEBUG_SERIAL build idles at the same timing as a production build.
  static uint32_t tf_count = 0;
  ++tf_count;
  const bool capture = g_dbg_on && (tf_count % 300) == 0;
#endif

  for (uint8_t i = 0; i < panel_set_count; ++i) {
    const PanelSet &ps = panel_sets[i];
    const uint8_t *block_b0 = blocks_base + (uint32_t)ps.panel_b0 * block_byte_count;
    const uint8_t *block_b1 = blocks_base + (uint32_t)ps.panel_b1 * block_byte_count;

    beginPanelSetTransaction();
    digitalWriteFast(ps.cs_pin, LOW);
    // CS-to-SCK setup time. Sized as ~cs_setup_sck_periods of SCK at the
    // currently configured spi_clock_speed (see constants.h). Lets the
    // PL022 peripheral's TX shift register load bit 0 before the first edge,
    // preventing the off-by-one-bit corruption we hit early in bring-up.
    delayNanoseconds(cs_setup_delay_ns_);
#ifdef DEBUG_SERIAL
    if (capture) {
      transferPanelSet(block_b0, block_b1, block_byte_count,
                       miso_scratch_b0_, miso_scratch_b1_);
      // Recover the 3-byte confirmation from the (possibly bit-delayed) return
      // path. Reads scratch[0..3] — valid for both GS2 (53 B) and GS16 (203 B).
      realignCipo(miso_scratch_b0_, cipo_b0_[i], 3, cipo_realign_bits_);
      realignCipo(miso_scratch_b1_, cipo_b1_[i], 3, cipo_realign_bits_);
    } else {
      transferPanelSet(block_b0, block_b1, block_byte_count);
    }
#else
    transferPanelSet(block_b0, block_b1, block_byte_count);
#endif
    // SCK-to-CS hold time. Sized as ~cs_hold_sck_periods of SCK at the
    // currently configured spi_clock_speed (see constants.h). Covers both
    // the PL022 shift-register-to-FIFO transfer for the final byte AND a
    // few polling-loop iterations on the peripheral to actually drain the FIFO
    // before the peripheral sees cs_pin go HIGH and exits its read loop.
    delayNanoseconds(cs_hold_delay_ns_);
    digitalWriteFast(ps.cs_pin, HIGH);
    endPanelSetTransaction();
  }

#ifdef DEBUG_SERIAL
  if (capture) {
    DBG_PRINTF("[spi] transferFrame count=%lu us=%lu refresh_ticks=%lu cipo_realign=%u\n",
               (unsigned long)tf_count,
               (unsigned long)(micros() - t0),
               (unsigned long)isr_count_,
               (unsigned)cipo_realign_bits_);
    for (uint8_t i = 0; i < panel_set_count; ++i) {
      DBG_PRINTF("[spi] CIPO set%-2u cs=%-2u B0=%02X %02X %02X  B1=%02X %02X %02X\n",
                 (unsigned)i, (unsigned)panel_sets[i].cs_pin,
                 cipo_b0_[i][0], cipo_b0_[i][1], cipo_b0_[i][2],
                 cipo_b1_[i][0], cipo_b1_[i][1], cipo_b1_[i][2]);
    }
  }
#endif
}
