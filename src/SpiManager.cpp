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

  // Bring up both SPI buses. CIPO pin is informational; SPI.begin() owns its
  // configured pins regardless.
  for (uint8_t r = 0; r < region_count_per_frame; ++r) {
    pinMode(region_cipo_pins[r], INPUT);
    region_spi_[r]->begin();
  }

  // Drive every CS line HIGH (deselected) before any transaction can run.
  for (uint8_t i = 0; i < panel_set_count; ++i) {
    pinMode(panel_sets[i].cs_pin, OUTPUT);
    digitalWriteFast(panel_sets[i].cs_pin, HIGH);
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

void SpiManager::beginPanelSetTransaction() {
  SPISettings settings(spi_clock_speed, spi_bit_order, spi_data_mode);
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

void SpiManager::transferFrame(const uint8_t *frame_buf,
                               uint16_t block_byte_count) {
  if (frame_buf == nullptr) return;
  if (block_byte_count != G6::block_byte_count_gs2 &&
      block_byte_count != G6::block_byte_count_gs16 &&
      block_byte_count != G6::block_byte_count_psram &&
      block_byte_count != G6::block_byte_count_psram_duty) {
    return;
  }

#ifdef DEBUG_SERIAL
  uint32_t t0 = micros();
#endif

  const uint8_t *blocks_base = frame_buf + stream_frame_prefix_byte_count;

#ifdef DEBUG_SERIAL
  // On debug builds, capture MISO for panel set 0 so we can inspect the
  // panel's CIPO confirmation slot (first 3 bytes). All other panel sets
  // discard MISO as before.
  uint8_t *miso_b0_first = miso_scratch_b0_;
  uint8_t *miso_b1_first = miso_scratch_b1_;
  memset(miso_b0_first, 0, block_byte_count);
  memset(miso_b1_first, 0, block_byte_count);
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
    delayNanoseconds(cs_setup_delay_ns);
#ifdef DEBUG_SERIAL
    if (i == 0) {
      transferPanelSet(block_b0, block_b1, block_byte_count,
                       miso_b0_first, miso_b1_first);
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
    delayNanoseconds(cs_hold_delay_ns);
    digitalWriteFast(ps.cs_pin, HIGH);
    endPanelSetTransaction();
  }

#ifdef DEBUG_SERIAL
  static uint32_t tf_count = 0;
  ++tf_count;
  if ((tf_count % 300) == 0) {
    DBG_PRINTF("[spi] transferFrame count=%lu us=%lu refresh_ticks=%lu\n",
               (unsigned long)tf_count,
               (unsigned long)(micros() - t0),
               (unsigned long)isr_count_);
    DBG_PRINTF("[spi] CIPO set0 B0=%02X %02X %02X  B1=%02X %02X %02X\n",
               miso_b0_first[0], miso_b0_first[1], miso_b0_first[2],
               miso_b1_first[0], miso_b1_first[1], miso_b1_first[2]);
  }
#endif
}
