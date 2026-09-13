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
//   [3776 B gap]
//   Health ISR/wdog record 0x2027FEC0 .. 0x2027FF20 (3 lines)
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
//   FRAME (type 2, 26 B): idx u16 @10, pattern u16 @12, sd_load_us u32 @14,
//         spi_us u16 @18, req_age_us u32 @20, superseded u8 @24, flags u8 @25 —
//         appended from transmitOnRefresh when the displayed frame index or
//         pattern differs from the last FRAME record; t_us = start of the SPI
//         push; sd_load_us = the readFrame that produced this frame (u32: 129 ms
//         SD reads have been observed, u16 would clip); req_age_us = dispatch of
//         the SET_FRAME_POSITION that requested this frame (loadFrame entry for
//         Mode 2/4) → SPI start, u32 so a 30–90 ms card stall is representable
//         (dispatch→SPI latency: excludes USB/host queueing); superseded = frames
//         loaded into frame_buf_ but replaced before any transfer since the last
//         FRAME record (0 = every load was shown); flags bit0 = frame came from
//         an SD read, bit1 = the pattern file is contiguous (O(1) seek path).
//         Ring layout version 2 (kVersion): a ring written by a 20 B-FRAME build
//         is re-initialised at boot instead of being truncated by repairChain.
//   STATE (type 3, 14 B): kind u8 @10, code u8 @11, arg u16 @12
//         kind 1 boot          code = reset_cause & 0xFF, arg = prev breadcrumb op << 8 | prev_valid
//         kind 2 state_change  code = new ArenaState, arg = pattern_id
//         kind 3 error_glyph   code = CE code, arg = 0
//         kind 4 sd_slow       code = 0, arg = readFrame µs / 100 (when > 20 ms)
//         kind 5 ring_overrun  code = 0: arg = records evicted since the last marker
//                              code = 0xFF: ring disabled, heap collision (arg 0)
//         kind 6 telemetry     code = SET_TELEMETRY flags, arg = rate (records/s)
//         kind 7 sd_open       code = openPattern CE result, arg = pattern_id
//         kind 8 wdog_context  (boot after a watchdog reset) code = EXC_RETURN & 0xFF,
//                              arg = xPSR IPSR (bits 0-8) | prior isr_last << 9 (bits 9-15)
//         kind 9 prev_isr_count (boot after a watchdog reset) code = ISR id, arg = min(65535, entries >> 12)
//         kind 10 timer_fail   code = 0, arg = requested refresh rate (Hz); the timer stayed un-armed
//         kind 11 sd_layout    after every sd_open: code bit0 = pattern file contiguous (FILE_FLAG_CONTIGUOUS
//                              set by contiguousRange → O(1) seeks), bit1 = exFAT volume, bit2 = legacy seek forced
//                              (SET_SD_DIAG 0xCE bit0: contiguousRange skipped at this open), bit3 = same-index
//                              skip disabled (0xCE bit1); arg = sectors per cluster
//         kind 12 sd_slow_ctx  follows every sd_slow: code = SdFat card errorCode() (sticky: 0 = the driver never
//                              saw an error this boot), arg = errorData() >> 16 = USDHC IRQSTAT bits 16-31 saved at
//                              the LAST driver error (command/data timeout, CRC, end-bit, auto-CMD12, DMA error
//                              bits; may predate this read) — did the driver see an error/retry, or did the card
//                              just hold the bus?
//         kind 13 sd_reads     at pattern close / re-open: reads = arg << (code & 0x7F) (binary shift so a
//                              360k-read trial fits) — readFrame calls while that pattern was open; the host
//                              cannot derive it once same-index SET_FRAME_POSITIONs skip the SD read.
//                              code bit 7 = cumulative CHECKPOINT (every 30 000 reads, same open), not a close
//         sd_slow (kind 4) code byte, since ring v2: bits 0-1 = slowest phase of the read (1 seek,
//                              2 body read, 3 CRC-trailer read), bit7 = the read returned an error
// ---------------------------------------------------------------------------

namespace Telemetry {

constexpr uint32_t kRingBase   = 0x2026F000UL;
constexpr uint32_t kRingSize   = 0x10000UL;               // 64 KiB
constexpr uint32_t kRingEnd    = kRingBase + kRingSize;    // 0x2027F000
constexpr uint32_t kHeaderSize = 32;
constexpr uint32_t kDataSize   = kRingSize - kHeaderSize;  // 65,504 B of records
constexpr uint32_t kMagic      = 0x47365452UL;             // 'G6TR'
constexpr uint8_t  kVersion    = 2;  // 2: FRAME 26 B (req_age_us/superseded/flags), STATE kinds 11-13
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
  ST_WDOG_CONTEXT = 8,  // after a watchdog reset: code = EXC_RETURN low byte (0xF9 thread / 0xF1 handler preempted),
                        // arg = stacked xPSR IPSR (bits 0-8; 0 = thread, else exception number, PIT = 138)
                        //     | (isr_last as it was when the watchdog fired) << 9   (7 bits; 0 = main loop)
  ST_PREV_ISR_COUNT = 9,  // after a watchdog reset, one per ISR id with a non-zero count: code = ISR id,
                          // arg = min(65535, count >> 12) (units of 4096 entries)
  ST_TIMER_FAIL   = 10, // IntervalTimer::begin() failed (no free PIT channel): code = 0, arg = requested refresh Hz
  ST_SD_LAYOUT    = 11, // after sd_open: code bit0 contiguous, bit1 exFAT, bit2 legacy seek (diag), bit3 no same-index skip (diag); arg = sectors/cluster
  ST_SD_SLOW_CTX  = 12, // follows sd_slow: code = card errorCode(), arg = errorData() >> 16 (USDHC error bits)
  ST_SD_READS     = 13, // at pattern close/re-open: reads = arg << (code & 0x7F); code bit 7 = periodic checkpoint
};
// sd_slow (kind 4) code byte: which phase of readFrame was slowest, + error flag.
constexpr uint8_t kSdSlowPhaseSeek  = 1;
constexpr uint8_t kSdSlowPhaseBody  = 2;
constexpr uint8_t kSdSlowPhaseTail  = 3;
constexpr uint8_t kSdSlowErrorFlag  = 0x80;
// FRAME flags byte.
constexpr uint8_t kFrameFlagSdRead     = 0x01;  // frame_buf_ was filled by readFrame
constexpr uint8_t kFrameFlagContiguous = 0x02;  // the open pattern file is contiguous (O(1) seek path)
constexpr uint8_t kOverrunCodeEvicted       = 0x00;
constexpr uint8_t kOverrunCodeHeapCollision = 0xFF;
constexpr uint8_t kSyntheticCmd             = 0xFE;

constexpr uint8_t  kRecordHeaderLen = 10;  // len, type, seq, t_us
constexpr uint8_t  kCmdPayloadMax   = 8;
constexpr uint8_t  kCmdRecordLenMax = kRecordHeaderLen + 3 + kCmdPayloadMax;  // 21
constexpr uint8_t  kFrameRecordLen  = kRecordHeaderLen + 16;                  // 26 (ring v2; 20 in v1)
constexpr uint8_t  kStateRecordLen  = kRecordHeaderLen + 4;                   // 14
constexpr uint8_t  kRecordLenMax    = kFrameRecordLen > kCmdRecordLenMax ? kFrameRecordLen : kCmdRecordLenMax;
constexpr uint32_t kSdSlowThresholdUs = 10000;  // readFrame slower than this → STATE(sd_slow); 10 ms = the worst-case
                                                 // acceptable display freeze (Michael, 2026-09-13; 5 ms target); was 20 ms in ring v1

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
static_assert(kRingEnd <= 0x2027FEC0UL, "ring must end below the Health ISR/watchdog record (0x2027FEC0) + breadcrumb");
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
           uint32_t sd_load_us, uint32_t spi_us,
           uint32_t req_age_us, uint8_t superseded, uint8_t flags);
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
