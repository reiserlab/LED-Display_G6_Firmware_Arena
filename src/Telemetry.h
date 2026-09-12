#pragma once

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Controller telemetry ring buffer (SET_TELEMETRY 0xA8 / GET_TELEMETRY_BLOCK 0xA9).
//
// Issue #50 follow-on to GET_HEALTH (0xCA): the breadcrumb says what the
// controller was doing at the instant it wedged; this ring says what led up
// to it. Every dispatched command (controller receive time + reply status),
// every displayed frame change (SD load time + SPI push time), and every state
// transition / error glyph / slow SD read is appended as a small binary record
// to a 64 KiB byte ring in OCRAM. A host drains it live over the single USB-CDC
// link with framed, chunked GET_TELEMETRY_BLOCK replies and an ACK CURSOR:
// records are only freed once the host acknowledges their sequence number, so
// a lost or timed-out reply is harmless (re-ask with the same ack, get the
// same bytes). Nothing here touches the SD card.
//
// CRASH-DUMP SEMANTICS. The ring lives at a FIXED OCRAM address, in no linker
// section (same reasoning as the Health breadcrumb): startup.c's .bss clear
// and .data copy never touch it, so it survives SYSTEM_RESET (0x01), a CPU
// lockup reset, and the bootloader's program-button reboot (RAM is kept) —
// but not a power-on. At boot, if the header's magic + checksum validate and
// the cursors are in range, the contents are KEPT (boot_count++), the record
// chain is walked and truncated at the first inconsistent record (a partially
// written trailing record is ignored, never a reason to wipe), and a
// STATE(boot) record is appended; the next drain hands the host the last
// seconds before the reset. Otherwise (power-on garbage) the ring is
// initialised.
//
// FULL RING = OVERWRITE OLDEST. The ring is first a crash recorder: the
// lead-up to a hang matters more than unread old history. When a record does
// not fit, read_off is advanced past the oldest record(s) until it does and
// `dropped` counts every evicted record; the host sees the seq gap plus the
// counter, and one STATE(ring_overrun) marks each eviction episode.
//
// COMMIT ORDER (reset safety). appendRaw writes the record bytes and flushes
// them FIRST, then updates write_off / next_seq / checksum and flushes the
// header. A reset between the two leaves the header pointing at the last
// complete record; the orphan bytes beyond write_off are simply never seen.
//
// COST + CACHE. Recording is a few dozen byte stores plus two dcache flushes
// (record lines + the header line). The flushes are required: startup.c maps
// OCRAM as MEM_CACHE_WBWA (write-back, write-allocate), so without them the
// most recent records — exactly the ones a crash dump wants — would still be
// dirty in the D-cache when a reset invalidates it. Health.cpp makes the same
// choice for the breadcrumb. ~0.2 µs per record at 600 MHz; at 286 Hz Mode-3
// streaming (CMD + FRAME per step) that is < 0.02 % CPU. Making the region
// non-cacheable via an MPU region was rejected: the base is not 64 KiB-aligned
// (MPU regions must be size-aligned), and a cached region reads faster for
// the drain.
//
// MEMORY MAP (OCRAM / "RAM2", 0x20200000..0x20280000):
//   .bss.dma (DMAMEM)      0x20200000 .. ~0x2020D9A0   (55.7 KB in this build)
//   heap                   grows up from _heap_start = end of .bss.dma, and
//                          the linker's _heap_end is the TOP of OCRAM — so
//                          malloc CAN in principle reach this region
//   [~390 KiB unused]
//   telemetry ring         0x2026F000 .. 0x2027F000    (this module)
//   [3872 B gap]
//   Health ISR/wdog record 0x2027FF20 .. 0x2027FF40
//   Health breadcrumb      0x2027FF40 .. 0x2027FF60
//   PJRC CrashReport       0x2027FF80 .. 0x20280000
// HEAP GUARD. The heap would need ~390 KiB of malloc to reach the ring and
// this firmware's heap use is small and static (SdFat, Wire) — but nothing
// enforces that, so begin() AND every append compare the live heap break
// (__brkval, startup.c) against kRingBase - kHeapGuardBytes. If the heap gets
// that close the ring is disabled for the rest of this boot (events off, no
// ring access at all — including the header, which the heap would clobber
// first), a STATE(ring_overrun, code 0xFF) is appended if the ring is still
// intact, and the 0xA9 header reports flags bit2 "disabled: heap collision".
//
// SYNTHETIC PRODUCER (bench test T1, drain throughput). SET_TELEMETRY flags
// bit7 turns on a dummy producer: service() (called from loop()) emits a
// CMD-type record (cmd 0xFE, status 0, payload = u32 counter) at `rate`
// records/s, paced by micros() accumulation (no timer). Off at boot.
//
// LAYOUT (all little-endian). Header, 32 B at kRingBase:
//   off  0  u32 magic      = 0x47365452 ('G6TR')
//   off  4  u8  ver        = 1
//   off  5  u8  flags      bit0 events_enabled
//   off  6  u16 reserved
//   off  8  u32 write_off  next byte to write, offset into the record area
//   off 12  u32 read_off   host ack cursor (oldest unacked byte)
//   off 16  u32 next_seq   seq of the next record appended (starts at 1)
//   off 20  u32 dropped    records evicted (overwrite-oldest) since init
//   off 24  u32 boot_count boots that KEPT this ring's contents (0 = fresh)
//   off 28  u32 checksum   sum of the seven u32 words above
// Record area: kRingBase+32 .. kRingBase+0x10000 (65,504 B), a byte ring.
// A record never straddles the end: if the tail cannot hold the next record
// a PAD record (len = tail bytes, type 0) fills it and writing wraps to 0.
// Record:
//   off  0  u8  len        total record length including this byte
//   off  1  u8  type       0 PAD, 1 CMD, 2 FRAME, 3 STATE
//   off  2  u32 seq        monotonic record counter
//   off  6  u32 t_us       micros() (raw; the host unwraps the 71.6 min wrap)
//   off 10      payload
//   CMD   (type 1, 13..21 B): cmd u8 @10, status u8 @11, plen u8 @12,
//         payload[plen ≤ 8] @13 — the first ≤ 8 request bytes after
//         [len, cmd]; t_us = receipt time before dispatch; status = the
//         reply's status byte (0xFF if none was queued). The synthetic
//         producer uses cmd 0xFE, status 0, plen 4, payload = counter u32.
//   FRAME (type 2, 20 B): idx u16 @10, pattern u16 @12, sd_load_us u32 @14,
//         spi_us u16 @18 — appended from transmitOnRefresh when the displayed
//         frame index or pattern differs from the last FRAME record; t_us =
//         start of the SPI push; sd_load_us = the most recent readFrame
//         duration (u32: 129 ms SD reads have been observed, u16 would clip)
//   STATE (type 3, 14 B): kind u8 @10, code u8 @11, arg u16 @12
//         kind 1 boot          code = reset_cause & 0xFF, arg = prev breadcrumb op << 8 | prev_valid
//         kind 2 state_change  code = new ArenaState, arg = pattern_id
//         kind 3 error_glyph   code = CE code, arg = 0
//         kind 4 sd_slow       code = 0, arg = readFrame µs / 100 (when > 20 ms)
//         kind 5 ring_overrun  code = 0: arg = records evicted since the last marker
//                              code = 0xFF: ring disabled, heap collision (arg 0)
//         kind 6 telemetry     code = SET_TELEMETRY flags, arg = rate (records/s)
//         kind 7 sd_open       code = openPattern CE result, arg = pattern_id
// ---------------------------------------------------------------------------

namespace Telemetry {

constexpr uint32_t kRingBase   = 0x2026F000UL;
constexpr uint32_t kRingSize   = 0x10000UL;               // 64 KiB
constexpr uint32_t kRingEnd    = kRingBase + kRingSize;    // 0x2027F000
constexpr uint32_t kHeaderSize = 32;
constexpr uint32_t kDataSize   = kRingSize - kHeaderSize;  // 65,504 B of records
constexpr uint32_t kMagic      = 0x47365452UL;             // 'G6TR'
constexpr uint8_t  kVersion    = 1;
constexpr uint32_t kHeapGuardBytes = 4096;                 // disable when __brkval >= base - this

// SET_TELEMETRY request flags (also the header flags bit0).
constexpr uint8_t kFlagEventsEnabled = 0x01;  // bit0 record events
constexpr uint8_t kFlagSynthetic     = 0x80;  // bit7 synthetic producer (T1)

// GET_TELEMETRY_BLOCK reply flags byte.
constexpr uint8_t kBlockFlagEvents        = 0x01;
constexpr uint8_t kBlockFlagSurvivedBoot  = 0x02;
constexpr uint8_t kBlockFlagHeapCollision = 0x04;
constexpr uint8_t kBlockFlagSynthetic     = 0x08;

enum RecordType : uint8_t {
  REC_PAD   = 0,
  REC_CMD   = 1,
  REC_FRAME = 2,
  REC_STATE = 3,
};

enum StateKind : uint8_t {
  ST_BOOT         = 1,
  ST_STATE_CHANGE = 2,
  ST_ERROR_GLYPH  = 3,
  ST_SD_SLOW      = 4,
  ST_RING_OVERRUN = 5,
  ST_TELEMETRY    = 6,
  ST_SD_OPEN      = 7,
};
constexpr uint8_t kOverrunCodeEvicted       = 0x00;
constexpr uint8_t kOverrunCodeHeapCollision = 0xFF;
constexpr uint8_t kSyntheticCmd             = 0xFE;

constexpr uint8_t  kRecordHeaderLen = 10;  // len, type, seq, t_us
constexpr uint8_t  kCmdPayloadMax   = 8;
constexpr uint8_t  kCmdRecordLenMax = kRecordHeaderLen + 3 + kCmdPayloadMax;  // 21
constexpr uint8_t  kFrameRecordLen  = kRecordHeaderLen + 10;                  // 20
constexpr uint8_t  kStateRecordLen  = kRecordHeaderLen + 4;                   // 14
constexpr uint8_t  kRecordLenMax    = kCmdRecordLenMax;
constexpr uint32_t kSdSlowThresholdUs = 20000;  // readFrame slower than this → STATE(sd_slow)

struct RingHeader {
  uint32_t magic;
  uint8_t  ver;
  uint8_t  flags;
  uint16_t reserved;
  uint32_t write_off;
  uint32_t read_off;
  uint32_t next_seq;
  uint32_t dropped;
  uint32_t boot_count;
  uint32_t checksum;
};
static_assert(sizeof(RingHeader) == kHeaderSize, "RingHeader must be exactly 32 B");
static_assert(kRingBase % 32 == 0, "ring base must be cache-line aligned");
static_assert(kRingEnd <= 0x2027FF20UL, "ring must end below the Health ISR/watchdog record + breadcrumb");
static_assert(kRingEnd <= 0x2027FF80UL, "ring must end below PJRC CrashReport");

// Call in setup() AFTER Health::begin() (the boot record carries the reset
// cause + previous breadcrumb) and before anything that could record.
// Keeps-or-initialises the ring, then appends STATE(boot).
void begin();

// Call once per loop() iteration: heap guard + synthetic producer. One
// compare when nothing is on.
void service();

// SET_TELEMETRY: flags bit0 events on/off (default ON at every boot), bit7
// synthetic producer on/off at `rate_hz` records/s (off at boot); rate_hz < 0
// leaves the stored rate unchanged (the 2-byte request form). Records
// STATE(telemetry, flags, rate). No effect once the heap guard tripped.
void configure(uint8_t flags, int32_t rate_hz);
bool enabled();
bool syntheticOn();
bool ringOk();          // false once disabled (heap guard), or never usable
bool heapCollision();   // the guard tripped this boot

// Producers. All are no-ops while events are disabled or the ring is unusable.
void cmd(uint32_t t_rx_us, uint8_t cmd_byte, uint8_t status,
         const uint8_t *req, uint8_t req_len);
void frame(uint32_t t_us, uint16_t idx, uint16_t pattern,
           uint32_t sd_load_us, uint32_t spi_us);
void state(uint8_t kind, uint8_t code, uint16_t arg);

// Drain (GET_TELEMETRY_BLOCK). ack() frees every record with seq <= ack_seq
// (0xFFFFFFFF = no ack). peek() copies as many WHOLE records as fit in
// max_bytes from the read cursor WITHOUT advancing it (PAD records are
// skipped, never returned) and reports the first seq, count, and whether
// unread records remain.
struct BlockInfo {
  uint32_t first_seq;   // 0 if no record returned
  uint16_t n_records;
  bool     more;
};
void   ack(uint32_t ack_seq);
size_t peek(uint8_t *dst, size_t max_bytes, BlockInfo &info);
uint8_t blockFlags();   // the 0xA9 header flags byte

// Read-only header views (for the block header + DEBUG_SERIAL banner).
uint32_t dropped();
uint32_t bootCount();
uint32_t nextSeq();
uint32_t pendingBytes();     // unacked bytes in the ring (incl. PADs)
bool     keptAcrossReset();  // this boot found a valid ring and kept it

}  // namespace Telemetry
