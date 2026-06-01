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

constexpr uint32_t spi_clock_speed = 25'000'000;
constexpr uint8_t  spi_bit_order   = MSBFIRST;
constexpr uint8_t  spi_data_mode   = SPI_MODE3;

// CS-to-SCK setup and SCK-to-CS hold delays, expressed as SCK-period counts
// and converted to nanoseconds against the current spi_clock_speed. Both
// scale automatically when spi_clock_speed changes, preserving the bit-edge
// margin we want at the panel's SPI peripheral. Rationale:
//   - cs_setup: PL022 peripheral needs CS asserted ~1-2 SCK periods before the
//     first clock edge so its TX shift register can load bit 0.
//   - cs_hold:  panel firmware's polling loop breaks on cs_pin HIGH without
//     draining the RX FIFO; ~25 SCK periods gives plenty of time for the
//     PL022 to shift the final byte into the FIFO and for the peripheral loop
//     to read it before CS rises.
constexpr uint32_t cs_setup_sck_periods = 2;
constexpr uint32_t cs_hold_sck_periods  = 25;
constexpr uint32_t cs_setup_delay_ns
    = (uint32_t)((1'000'000'000ULL * cs_setup_sck_periods) / spi_clock_speed);
constexpr uint32_t cs_hold_delay_ns
    = (uint32_t)((1'000'000'000ULL * cs_hold_sck_periods)  / spi_clock_speed);

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
// Controller identity — get-controller-info (0x67).
// Response payload is {version_byte, capability_bitmap} (g6_03 § 5).
// -----------------------------------------------------------------------------

constexpr uint8_t controller_info_version = 1;  // G6 controller protocol v1
// Capability bitmap: bit0 g6_mode (always 1 for any G6 controller),
// bit1 v2_local_storage, bit2 mode_1_tsi, bit3 v3_triggered, bit4 v3_gated.
// Advertises g6_mode + v2_local_storage (V2 display-from-PSRAM, LAB-41/42).
constexpr uint8_t controller_capability_bitmap = 0x03;

// -----------------------------------------------------------------------------
// SD pattern backend — Modes 2/3/4 load .pat files from the built-in SD slot.
// File format per g6_04-pattern-file-format.md (v2 18-byte G6PT header).
// -----------------------------------------------------------------------------

constexpr char     pattern_dir[]                    = "/patterns";
constexpr uint16_t pattern_max_count                = 256;   // listing capacity
constexpr uint8_t  pattern_name_byte_count          = 64;    // incl. NUL
constexpr uint8_t  pattern_header_byte_count        = 18;
constexpr uint8_t  pattern_frame_prefix_byte_count  = 4;     // "FR" + index16
constexpr uint8_t  pattern_frame_crc_byte_count     = 2;     // CRC-16 trailer
constexpr uint8_t  pattern_format_version           = 2;     // v2 header
constexpr uint8_t  arena_panel_count_max            = 48;    // 6-byte panel mask

// -----------------------------------------------------------------------------
// Display modes (trial-params "mode" byte, G4-compatible).
//   2 = Open Loop      (auto-advance frames at frame_rate)
//   3 = Show Frame     (host-commanded frame index via set-frame-position)
//   4 = Closed Loop    (AIN0 velocity integration)
//   5 = Streaming      (host streams raw frames; the 0x32 path)
// -----------------------------------------------------------------------------

constexpr uint8_t display_mode_open_loop   = 2;
constexpr uint8_t display_mode_show_frame   = 3;
constexpr uint8_t display_mode_closed_loop = 4;
constexpr uint8_t display_mode_streaming   = 5;

// -----------------------------------------------------------------------------
// Mode 4 closed-loop velocity — samples AIN0 (Teensy D14, BNC J28) at 500 Hz.
// fps = V_ain * 100 * gain / 10, where gain is the signed trial-params byte
// carrying 10x the actual fps/V scaling factor (g6_03 § 6 Mode 4).
// -----------------------------------------------------------------------------

constexpr uint8_t  mode4_ain_pin        = 14;     // AIN0 / Teensy D14
constexpr uint32_t mode4_sample_rate_hz = 500;
constexpr uint16_t adc_full_scale_counts = 1023;  // 10-bit analogRead default
constexpr float    adc_ref_volts        = 3.3f;
// Bipolar BNC input range that the OPA2277 front-end maps onto the ADC span
// (midscale = 0 V). Hardware calibration value — flagged TBD in g6_03 § Mode 4.
constexpr float    mode4_ain_input_range_volts = 10.0f;

// -----------------------------------------------------------------------------
// Controller error display (g6_03 § 6) — "CE / NN" glyph held >= this long.
// -----------------------------------------------------------------------------

constexpr uint32_t error_display_hold_ms = 750;  // spec floor is 500 ms

enum ControllerError : uint8_t {
  CE_NONE            = 0,
  CE_UNKNOWN_CMD     = 1,   // unrecognized opcode
  CE_BAD_PAYLOAD_LEN = 2,   // framing length mismatch
  CE_BAD_PARAM       = 3,   // out-of-range pattern / frame index
  CE_SD_NOT_PRESENT  = 4,   // no SD card mounted
  CE_SD_FILE_ERROR   = 5,   // pattern file missing / unreadable
  CE_HEADER_CRC      = 6,   // pattern header CRC-8 mismatch / bad magic
  CE_FRAME_CRC       = 7,   // per-frame CRC-16 mismatch / bad FR magic
  CE_ARENA_MISMATCH  = 8,   // pattern geometry != this controller's arena
};

// -----------------------------------------------------------------------------
// Misc timing.
// -----------------------------------------------------------------------------

constexpr uint32_t microseconds_per_second = 1'000'000;
constexpr uint32_t milliseconds_per_second = 1'000;

} // namespace constants
} // namespace AC
