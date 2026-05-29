#include "ErrorGlyph.h"
#include <string.h>
#include "constants.h"
#include "G6PanelProtocol.h"

namespace G6Error {

namespace {

// 5x7 bitmap font. Each glyph is 7 rows; each row uses the low 5 bits, where
// bit 4 (0x10) is the leftmost column and bit 0 (0x01) the rightmost.
// Only the glyphs the error display needs are defined: digits 0-9, 'C', 'E'.
constexpr uint8_t kFontDigits[10][7] = {
    {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E},  // 0
    {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E},  // 1
    {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F},  // 2
    {0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E},  // 3
    {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02},  // 4
    {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E},  // 5
    {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E},  // 6
    {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08},  // 7
    {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E},  // 8
    {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C},  // 9
};
constexpr uint8_t kFontC[7] = {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E};
constexpr uint8_t kFontE[7] = {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F};

constexpr uint8_t kPanelDim = AC::constants::panel_pixel_count_per_row;  // 20
constexpr uint8_t kGlyphW = 5;
constexpr uint8_t kGlyphH = 7;

// Stamp one 5x7 glyph into the 20x20 image (origin row 0 = top) at the given
// top-left corner. On-pixels are set to GS16 max (0x0F).
void stampGlyph(uint8_t img[kPanelDim][kPanelDim], const uint8_t glyph[7],
                uint8_t top, uint8_t left) {
  for (uint8_t gr = 0; gr < kGlyphH; ++gr) {
    uint8_t bits = glyph[gr];
    for (uint8_t gc = 0; gc < kGlyphW; ++gc) {
      if (bits & (0x10 >> gc)) {
        uint8_t r = top + gr;
        uint8_t c = left + gc;
        if (r < kPanelDim && c < kPanelDim) img[r][c] = 0x0F;
      }
    }
  }
}

}  // namespace

uint16_t buildErrorFrame(uint8_t *frame_buf, uint8_t code, uint8_t panel_count) {
  using namespace AC::constants;

  // 1. Render the glyph into a 20x20 intensity image (row 0 = top).
  uint8_t img[kPanelDim][kPanelDim];
  memset(img, 0, sizeof(img));

  // Top row: "C" "E"; bottom row: tens, ones. Two 5-wide glyphs per row with
  // a gap, vertically centered in each half.
  const uint8_t left_col = 4;
  const uint8_t right_col = 11;
  const uint8_t top_row = 2;
  const uint8_t bottom_row = 11;
  if (code > 99) code = 99;
  uint8_t tens = code / 10;
  uint8_t ones = code % 10;

  stampGlyph(img, kFontC, top_row, left_col);
  stampGlyph(img, kFontE, top_row, right_col);
  stampGlyph(img, kFontDigits[tens], bottom_row, left_col);
  stampGlyph(img, kFontDigits[ones], bottom_row, right_col);

  // 2. Pack into GS16 panel pixel bytes (200 bytes). Pixel layout per
  //    g6_04 § Pixel Data Layout: origin bottom-left, row-major,
  //    pixel_num = row_from_bottom*20 + col; even pixel = high nibble.
  uint8_t pixels[panel_pixel_count / 2];  // 200 bytes for 400 pixels
  memset(pixels, 0, sizeof(pixels));
  for (uint8_t r = 0; r < kPanelDim; ++r) {
    uint8_t row_from_bottom = (kPanelDim - 1) - r;
    for (uint8_t c = 0; c < kPanelDim; ++c) {
      uint8_t v = img[r][c] & 0x0F;
      if (!v) continue;
      uint16_t pixel_num = (uint16_t)row_from_bottom * kPanelDim + c;
      uint16_t byte_index = pixel_num / 2;
      if (pixel_num & 1) {
        pixels[byte_index] = (pixels[byte_index] & 0xF0) | v;       // odd: low
      } else {
        pixels[byte_index] = (pixels[byte_index] & 0x0F) | (v << 4); // even: high
      }
    }
  }

  // 3. Build the frame buffer: "FR" + index 0, then `panel_count` identical
  //    GS16 oneshot blocks with parity recomputed.
  frame_buf[0] = 'F';
  frame_buf[1] = 'R';
  frame_buf[2] = 0;
  frame_buf[3] = 0;

  const uint16_t block_size = panel_block_byte_count_gs16;
  const uint8_t duty_cycle = 0x80;  // mid brightness for a steady diagnostic
  for (uint8_t p = 0; p < panel_count; ++p) {
    uint8_t *block = frame_buf + stream_frame_prefix_byte_count
                     + (uint32_t)p * block_size;
    block[0] = G6::header_version_v1;       // parity stamped below
    block[1] = G6::cmd_disp_16lvl_oneshot;  // 0x30
    memcpy(block + 2, pixels, sizeof(pixels));
    block[2 + sizeof(pixels)] = duty_cycle;
    G6::stamp_header_parity(block, block_size);
  }

  return (uint16_t)(stream_frame_prefix_byte_count
                    + (uint32_t)panel_count * block_size);
}

}  // namespace G6Error
