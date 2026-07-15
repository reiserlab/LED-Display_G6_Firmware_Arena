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
  // layout is identical to the streamed payload: [4-byte prefix][40 panel
  // blocks in row-major panel order], with each block already including the
  // panel-protocol header byte (parity-correct), cmd byte, pixel data, and
  // duty_cycle. block_byte_count selects GS2 (53) vs GS16 (203).
  void transferFrame(const uint8_t *frame_buf, uint16_t block_byte_count);

  // Single-panel ISP transaction (g6-program-panel / 0xC8). Asserts ONLY the
  // target panel's CS, clocks `len` bytes full-duplex on that panel's bus, and
  // captures the CIPO return into `cipo` (may be nullptr to ignore the reply).
  // Uses a blocking full-duplex transfer — NOT the async-DMA path — because the
  // DMA write into a RAM buffer isn't CPU-coherent on Teensy 4 (see
  // transferPanelSet). Returns false if `panel_index` isn't in the arena map.
  // Run ISP at a conservative clock (≤18 MHz) so the CIPO return path doesn't
  // need the high-clock 1-bit realign.
  bool transferSinglePanel(uint8_t panel_index, const uint8_t *copi,
                           uint8_t *cipo, size_t len);

  // Display refresh timer — fires refreshFlag from ISR; main loop drains.
  void armRefreshTimer(uint32_t frequency_hz);
  void disarmRefreshTimer();

  // SPI clock — runtime-adjustable in whole MHz (1..30). CS setup/hold delays
  // and the CIPO realign shift are recomputed proportionally on each change.
  void     setSpiClockMhz(uint16_t mhz);
  uint16_t getSpiClockMhz() const { return (uint16_t)(spi_clock_hz_ / 1'000'000UL); }

  // Frames-sent counter — incremented by transferFrame(); zeroed by host command.
  uint32_t framesSent() const { return frames_sent_; }
  void     resetFramesSent()  { frames_sent_ = 0; }

  // Frame-scan debug gate (DIO role out_debug_framescan, #135): gate pins are
  // driven HIGH when transferFrame() starts clocking panel data and LOW after
  // the last panel set completes — a scope envelope spanning ALL SPI comms for
  // one frame, LOW between frames. -1 = disabled. Set by
  // CommandProcessor::applyDioRole (pins already configured as outputs there);
  // one slot per DIO port so both ports may gate simultaneously.
  void setFramescanGatePins(int16_t pin_a, int16_t pin_b) {
    framescan_pin_a_ = pin_a;
    framescan_pin_b_ = pin_b;
  }

  volatile bool refreshFlag = false;
  volatile uint32_t isr_count_ = 0;  // refresh ISRs since boot (debug counter)

 private:
  SPIClass *region_spi_[AC::constants::region_count_per_frame] = { &SPI, &SPI1 };

  // Runtime SPI timing — all derived from spi_clock_hz_; updated together by
  // setSpiClockMhz() so the invariant (delays == f(clock)) always holds.
  uint32_t spi_clock_hz_      = AC::constants::spi_clock_speed;
  uint32_t cs_setup_delay_ns_ = AC::constants::cs_setup_delay_ns;
  uint32_t cs_hold_delay_ns_  = AC::constants::cs_hold_delay_ns;
  uint32_t frames_sent_       = 0;
  int16_t  framescan_pin_a_   = -1;  // frame-scan gate pins (setFramescanGatePins)
  int16_t  framescan_pin_b_   = -1;
#ifdef DEBUG_SERIAL
  uint8_t  cipo_realign_bits_ = AC::constants::cipo_realign_left_bits;
#endif

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
