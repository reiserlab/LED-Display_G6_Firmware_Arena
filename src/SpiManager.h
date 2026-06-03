#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <EventResponder.h>
#include "constants.h"
#include "ArenaConfig.h"
#include "G6PanelProtocol.h"

class SpiManager {
 public:
  void begin();

  // Transmit a full frame buffer once to the panel chain. The frame buffer
  // layout is identical to the streamed payload: [4-byte prefix][20 panel
  // blocks in row-major panel order], with each block already including the
  // panel-protocol header byte (parity-correct), cmd byte, pixel data, and
  // duty_cycle. block_byte_count selects GS2 (53) vs GS16 (203).
  void transferFrame(const uint8_t *frame_buf, uint16_t block_byte_count);

  // Display refresh timer — fires refreshFlag from ISR; main loop drains.
  void armRefreshTimer(uint32_t frequency_hz);
  void disarmRefreshTimer();

  volatile bool refreshFlag = false;
  volatile uint32_t isr_count_ = 0;  // refresh ISRs since boot (debug counter)

 private:
  SPIClass *region_spi_[AC::constants::region_count_per_frame] = { &SPI, &SPI1 };

  IntervalTimer refreshTimer_;

  static SpiManager *instance_;
  static void refreshISR();

  // DMA completion for parallel SPI transfers.
  EventResponder dmaEvent_;
  static volatile bool dmaComplete_;
  static void dmaISR(EventResponderRef event);

  void beginPanelSetTransaction();
  void endPanelSetTransaction();
  void transferPanelSet(const uint8_t *block_b0,
                        const uint8_t *block_b1,
                        uint16_t block_byte_count,
                        uint8_t *miso_b0 = nullptr,
                        uint8_t *miso_b1 = nullptr);

#ifdef DEBUG_SERIAL
  // MISO scratch — captures one panel set's CIPO confirmation slot per transfer.
  uint8_t miso_scratch_b0_[G6::block_byte_count_gs16];
  uint8_t miso_scratch_b1_[G6::block_byte_count_gs16];
  // First 3 CIPO bytes for every panel set, retained for the periodic dump so
  // edge ports (e.g. P3 = panel set 4) are visible, not just set 0.
  uint8_t cipo_b0_[AC::panel_set_count][3];
  uint8_t cipo_b1_[AC::panel_set_count][3];
#endif
};
