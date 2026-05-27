#pragma once
#include <Arduino.h>
#include <SPI.h>

// Define DEBUG_SERIAL to enable Serial.printf diagnostics.
// When undefined, all debug prints and their formatting are compiled out.
#ifdef DEBUG_SERIAL
  #define DBG_PRINTF(...) do { Serial.printf("[%lu] ", millis()); Serial.printf(__VA_ARGS__); } while(0)
#else
  #define DBG_PRINTF(...) ((void)0)
#endif

namespace AC {
namespace constants {

// -----------------------------------------------------------------------------
// G6 panel geometry — 20x20 pixels per panel; G6_2x10 arena = 2 rows x 10 cols.
// -----------------------------------------------------------------------------

constexpr uint8_t panel_pixel_count_per_row = 20;
constexpr uint8_t panel_pixel_count_per_col = 20;
constexpr uint16_t panel_pixel_count
    = (uint16_t)panel_pixel_count_per_row * panel_pixel_count_per_col;  // 400

constexpr uint8_t panel_count_per_frame_row = 2;
constexpr uint8_t panel_count_per_frame_col = 10;
constexpr uint8_t panel_count_per_frame
    = panel_count_per_frame_row * panel_count_per_frame_col;            // 20

// G6 v1 panel-block sizes (header + cmd + pixel data + duty_cycle).
constexpr uint16_t panel_block_byte_count_gs2  = 53;
constexpr uint16_t panel_block_byte_count_gs16 = 203;

// Stream frame layout: 4-byte frame prefix ("FR" + 16-bit LE frame index)
// followed by 20 panel blocks in row-major panel order.
constexpr uint16_t stream_frame_prefix_byte_count = 4;
constexpr uint16_t stream_frame_byte_count_gs2
    = stream_frame_prefix_byte_count + panel_count_per_frame * panel_block_byte_count_gs2;   // 1064
constexpr uint16_t stream_frame_byte_count_gs16
    = stream_frame_prefix_byte_count + panel_count_per_frame * panel_block_byte_count_gs16;  // 4064

// Largest frame buffer we hold for SPI dispatch (= GS16 payload).
constexpr uint16_t frame_buf_byte_count_max = stream_frame_byte_count_gs16;

// -----------------------------------------------------------------------------
// SPI configuration — G6 v1 panel protocol mandates SPI MODE3 (CPOL=1, CPHA=1).
// Clock per panel firmware default upper bound (g6_01-panel-protocol.md §SPI framing).
// -----------------------------------------------------------------------------

constexpr uint32_t spi_clock_speed = 30'000'000;
constexpr uint8_t  spi_bit_order   = MSBFIRST;
constexpr uint8_t  spi_data_mode   = SPI_MODE3;

// Two SPI buses (B0 = Teensy SPI, B1 = Teensy SPI1).
constexpr uint8_t region_count_per_frame = 2;
constexpr uint8_t region_cipo_pins[region_count_per_frame] = { 12, 1 };

// -----------------------------------------------------------------------------
// Display refresh defaults — G6 has no SWITCH_GRAYSCALE command; the mode is
// inferred from the streamed payload size (1064 = GS2, 4064 = GS16). Refresh
// rates are inherited from the G4.1-ArenaSlim baseline, host-overridable via
// SET_REFRESH_RATE.
// -----------------------------------------------------------------------------

constexpr uint32_t refresh_rate_gs16_default = 300;
constexpr uint32_t refresh_rate_gs2_default  = 1000;

// -----------------------------------------------------------------------------
// Ethernet / TCP framing.
// -----------------------------------------------------------------------------

constexpr uint16_t ethernet_server_port = 62222;

// G6 stream header = [0x32, len_lo, len_hi]; no analog_x / analog_y bytes
// (per g6_03-controller.md § Stream-Frame for G6).
constexpr uint8_t stream_header_byte_count = 3;

// First byte of a binary command is the "remaining length"; the max legal value
// for a length byte is 0x31 (50 trailing bytes — comfortably above any G6
// non-stream command). 0x32 is the stream-command discriminator.
constexpr uint8_t first_command_byte_max_value_binary = 0x32;

constexpr uint16_t byte_count_per_response_max = 200;

// -----------------------------------------------------------------------------
// Misc timing.
// -----------------------------------------------------------------------------

constexpr uint32_t microseconds_per_second = 1'000'000;
constexpr uint32_t milliseconds_per_second = 1'000;

} // namespace constants
} // namespace AC
