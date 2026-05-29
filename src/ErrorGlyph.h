#pragma once
#include <stdint.h>

// Controller error display (g6_03-controller.md § 6). Composes a 20x20
// diagnostic glyph — "CE" on the top half, a two-digit error code on the
// bottom — as a full G6 frame of identical GS16 oneshot panel blocks, ready
// for SpiManager::transferFrame(). The block parity is recomputed here since
// the controller synthesizes these blocks.
namespace G6Error {

// Fill `frame_buf` with a 4-byte "FR" prefix followed by `panel_count`
// GS16 oneshot blocks (203 bytes each), every panel showing the same
// "CE / NN" glyph for the given error `code` (0-99). Returns the total byte
// count written (prefix + panel_count * 203).
uint16_t buildErrorFrame(uint8_t *frame_buf, uint8_t code, uint8_t panel_count);

}  // namespace G6Error
