#pragma once
#include <Arduino.h>
#include <SPI.h>

// Runtime gate for DEBUG_SERIAL diagnostics (defined in main.cpp). A host flips
// it with the SET_DIAG_OUTPUT (0xC3) command: web-serial mutes diagnostics on
// connect for a clean command/response channel; the CIPO capture scripts turn
// them back on. Declared in all builds so the command handler is uniform; only
// read in DEBUG_SERIAL builds.
extern volatile bool g_dbg_on;

// Define DEBUG_SERIAL to enable Serial.printf diagnostics. When undefined, all
// debug prints and their formatting are compiled out.
//
// Diagnostics share the single USB-CDC pipe with command responses, so each
// line is prefixed with AC::constants::diag_line_sentinel (a byte larger than
// any response length byte) — letting a host demux diagnostics from [length,
// status, ...] response frames and never desync. The line is emitted only when
// g_dbg_on is set AND the USB TX buffer has room for a whole line, so a slow or
// absent reader can never block the control loop (drops the line instead).
#ifdef DEBUG_SERIAL
  #define DBG_PRINTF(...) do { \
      if (g_dbg_on && \
          Serial.availableForWrite() >= AC::constants::diag_line_byte_count_max) { \
        Serial.write(AC::constants::diag_line_sentinel); \
        Serial.printf("[%lu] ", millis()); \
        Serial.printf(__VA_ARGS__); \
      } } while (0)
#else
  #define DBG_PRINTF(...) ((void)0)
#endif

namespace AC {
namespace constants {

// -----------------------------------------------------------------------------
// G6 panel geometry — 20x20 pixels per panel; G6_4x10 arena = 4 rows x 10 cols.
// -----------------------------------------------------------------------------

constexpr uint8_t panel_pixel_count_per_row = 20;
constexpr uint8_t panel_pixel_count_per_col = 20;
constexpr uint16_t panel_pixel_count
    = (uint16_t)panel_pixel_count_per_row * panel_pixel_count_per_col;  // 400

constexpr uint8_t panel_count_per_frame_row = 4;
constexpr uint8_t panel_count_per_frame_col = 10;
constexpr uint8_t panel_count_per_frame
    = panel_count_per_frame_row * panel_count_per_frame_col;            // 40

// G6 v1 panel-block sizes (header + cmd + pixel data + duty_cycle).
constexpr uint16_t panel_block_byte_count_gs2  = 53;
constexpr uint16_t panel_block_byte_count_gs16 = 203;

// Stream frame layout: 4-byte frame prefix ("FR" + 16-bit LE frame index)
// followed by 40 panel blocks in row-major panel order.
constexpr uint16_t stream_frame_prefix_byte_count = 4;
constexpr uint16_t stream_frame_byte_count_gs2
    = stream_frame_prefix_byte_count + panel_count_per_frame * panel_block_byte_count_gs2;   // 2124
constexpr uint16_t stream_frame_byte_count_gs16
    = stream_frame_prefix_byte_count + panel_count_per_frame * panel_block_byte_count_gs16;  // 8124

// Largest frame buffer we hold for SPI dispatch (= GS16 payload).
constexpr uint16_t frame_buf_byte_count_max = stream_frame_byte_count_gs16;

// -----------------------------------------------------------------------------
// SPI configuration — G6 v1 panel protocol mandates SPI MODE3 (CPOL=1, CPHA=1).
// Clock per panel firmware default upper bound (g6_01-panel-protocol.md §SPI framing).
// -----------------------------------------------------------------------------

constexpr uint32_t spi_clock_speed = 25'000'000;
constexpr uint8_t  spi_bit_order   = MSBFIRST;
constexpr uint8_t  spi_data_mode   = SPI_MODE3;

// CIPO readback realignment (DEBUG_SERIAL diagnostic only).
// The buffered, wired-OR MISO return path has a fixed round-trip delay. Above
// ~20 MHz that delay exceeds one bit, so the captured confirmation comes back
// right-shifted by exactly one bit (e.g. 01 30 62 reads as 00 98 31). Shifting
// the captured stream left by this many bits recovers it. The slip is 0 at low
// clocks and metastable (uncorrectable) around 10 MHz, so this is gated to the
// high-speed path. See Generation 6/arena-hardware-bug.md.
constexpr uint8_t cipo_realign_left_bits = (spi_clock_speed >= 20'000'000) ? 1 : 0;

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
// inferred from the streamed payload size (2124 = GS2, 8124 = GS16). Refresh
// rates are host-overridable via SET_REFRESH_RATE.
// -----------------------------------------------------------------------------

constexpr uint32_t refresh_rate_gs16_default = 400;
constexpr uint32_t refresh_rate_gs2_default  = 1200;

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

// DEBUG_SERIAL diagnostic line framing (firmware->host on the USB-CDC pipe).
// Each diagnostic line is prefixed with diag_line_sentinel; because it exceeds
// the largest possible response length byte (byte_count_per_response_max), a
// host can split sentinel-prefixed diagnostic text from [length, status, ...]
// response frames unambiguously. diag_line_byte_count_max bounds one line
// (sentinel + "[millis] " + body + newline); DBG_PRINTF only writes when the
// USB TX buffer has at least this much room.
constexpr uint8_t diag_line_sentinel        = 0xFF;
constexpr int     diag_line_byte_count_max  = 110;

// -----------------------------------------------------------------------------
// Controller identity — get-controller-info (0xC2).
// Response payload is {version_byte, capability_bitmap} (g6_03 § 5).
// -----------------------------------------------------------------------------

constexpr uint8_t controller_info_version = 1;  // G6 controller protocol v1
// Capability bitmap: bit0 g6_mode (always 1 for any G6 controller),
// bit1 v2_local_storage, bit2 mode_1_tsi, bit3 v3_triggered, bit4 v3_gated,
// bit5 io_ext (extended I/O command set: SET_DIO_ROLE 0xAC / GET_DIO_ROLE
// 0xAD / SET_AO_MODE 0xA3 / GET_ANALOG_IN 0xA4 — lets hosts detect the
// #135 rig-I/O roles by capability instead of firmware-version guessing).
// Advertises g6_mode + v2_local_storage + io_ext.
constexpr uint8_t controller_capability_bitmap = 0x23;

// -----------------------------------------------------------------------------
// SD pattern backend — Modes 2/3/4 load .pat files from the built-in SD slot.
// File format per g6_04-pattern-file-format.md (v2 18-byte G6PT header).
// -----------------------------------------------------------------------------

constexpr char     pattern_dir[]                    = "/patterns";
constexpr char     pattern_temp_name[]              = "pattern.temp";  // staging file in /patterns
constexpr char     manifest_bin_path[]              = "/MANIFEST.bin";
constexpr char     manifest_txt_path[]              = "/MANIFEST.txt";

// Panel firmware image for SPI in-system programming (g6_03 § Panel firmware
// update). A single image is held at a time; the trailing 32-byte footer is
// {magic[8], version[16], image_crc32(u32 LE), image_size(u32 LE)}.
constexpr char     firmware_dir[]                   = "/firmware";
constexpr char     firmware_path[]                  = "/firmware/panel.bin";
constexpr uint8_t  firmware_footer_byte_count       = 32;
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

// Digital outputs — bidirectional 5 V via SN74LVC1T45 level translators.
// DIR pin HIGH = A→B (Teensy drives BNC); LOW = B→A (BNC drives Teensy).
// Firmware initialises both channels as outputs driving LOW.
constexpr uint8_t do1_data_pin = 37;  // DO1 BNC J3 data  (D37, Teensy pad 29, via U2)
constexpr uint8_t do1_dir_pin  = 36;  // DO1 U2 direction (D36, Teensy pad 28)
constexpr uint8_t do2_data_pin = 35;  // DO2 BNC J4 data  (D35, Teensy pad 27, via U3)
constexpr uint8_t do2_dir_pin  = 34;  // DO2 U3 direction (D34, Teensy pad 26)

constexpr uint8_t  mode4_ain_pin        = 14;     // AIN0 / Teensy D14 — BNC "Analog In 1 (±10V)" (J28)
constexpr uint8_t  ain2_pin             = 15;     // AIN1 / Teensy D15 — BNC "Analog In 2 (±10V)" (J29, g6_03: experimenter-available)
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
  CE_ARENA_MISMATCH       = 8,   // pattern geometry != this controller's arena
  CE_MANIFEST_WRITE_ERROR = 9,   // MANIFEST.bin or MANIFEST.txt write failed
  CE_DISPLAY_ACTIVE       = 10,  // command refused: display is running; stop first
  CE_SD_FORMAT_ERROR      = 11,  // SD.format() failed (purge-memory, 0x8F)
};

// -----------------------------------------------------------------------------
// Misc timing.
// -----------------------------------------------------------------------------

constexpr uint32_t microseconds_per_second = 1'000'000;
constexpr uint32_t milliseconds_per_second = 1'000;
constexpr uint32_t duration_tick_ms = 10;  // trial_params (0x08) Duration field unit

} // namespace constants
} // namespace AC
