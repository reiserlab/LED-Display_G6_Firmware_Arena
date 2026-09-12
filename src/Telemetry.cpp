#include "Telemetry.h"
#include "Health.h"

// Live heap break from the Teensy core (startup.c: `char *__brkval`, advanced by
// _sbrk). The linker's _heap_end is the TOP of OCRAM, so malloc can in principle
// grow into the fixed ring region — begin() and every append check this.
extern "C" char *__brkval;

namespace Telemetry {

namespace {

RingHeader *const hdr  = reinterpret_cast<RingHeader *>(kRingBase);
uint8_t    *const data = reinterpret_cast<uint8_t *>(kRingBase + kHeaderSize);

// DTCM mirrors of the hot-path gates (one load + branch per producer call;
// the OCRAM header is the persistent copy).
bool     ring_ok_        = false;  // ring usable this boot (address range clear)
bool     enabled_        = false;  // hdr->flags & kFlagEventsEnabled
bool     kept_           = false;  // this boot kept a valid ring
bool     heap_collision_ = false;  // the heap guard tripped this boot

// Overwrite-oldest bookkeeping: one STATE(ring_overrun) per eviction episode.
uint32_t dropped_reported_ = 0;   // hdr->dropped at the last marker
uint32_t last_evict_us_    = 0;
bool     evicted_ever_     = false;
constexpr uint32_t kEvictEpisodeGapUs = 1000000UL;

// Synthetic producer (T1).
bool     synth_on_        = false;
uint16_t synth_rate_      = 0;
uint32_t synth_period_us_ = 0;
uint32_t synth_last_us_   = 0;
uint32_t synth_counter_   = 0;
constexpr uint16_t kSynthCatchUpMax = 256;  // records per service() after a long loop stall

inline uint32_t computeCheck(const RingHeader &h) {
  return h.magic
       + ((uint32_t)h.ver | ((uint32_t)h.flags << 8) | ((uint32_t)h.reserved << 16))
       + h.write_off + h.read_off + h.next_seq + h.dropped + h.boot_count;
}

// Commit the header to memory. OCRAM is write-back cached (startup.c MPU
// region MEM_CACHE_WBWA): without the flush the header could sit dirty in the
// D-cache when a reset invalidates it and the kept ring would fail the boot
// walk. One 32 B line.
inline void sealHeader() {
  hdr->checksum = computeCheck(*hdr);
  arm_dcache_flush(hdr, sizeof(RingHeader));
}

inline uint32_t usedBytes() {
  uint32_t w = hdr->write_off, r = hdr->read_off;
  return (w >= r) ? (w - r) : (kDataSize - r + w);
}
inline uint32_t freeBytes() { return kDataSize - 1 - usedBytes(); }  // one byte kept free: full != empty

inline bool isPadAt(uint32_t off) {
  uint8_t len = data[off];
  return len < kRecordHeaderLen || data[off + 1] == REC_PAD;
}

inline uint32_t seqAt(uint32_t off) {
  uint32_t seq;
  memcpy(&seq, data + off + 2, sizeof(seq));
  return seq;
}

// Exact per-type record shape — the same rules tests/telemetry_codec.py
// enforces, so a record that survives the boot repair or reaches peek() can
// always be decoded by the host. PAD: len < kRecordHeaderLen with type 0 (a
// 1-byte PAD has no type byte), or any len with type 0.
inline bool wellFormedAt(uint32_t off, uint32_t w) {
  uint8_t  len = data[off];
  if (len == 0) return false;
  uint32_t end = off + len;
  if (end > kDataSize) return false;                 // straddles the ring end
  if (off < w && end > w) return false;              // straddles the write cursor
  if (len < kRecordHeaderLen) return len == 1 || data[off + 1] == REC_PAD;
  switch (data[off + 1]) {
    case REC_PAD:   return true;
    case REC_CMD:   return len >= kRecordHeaderLen + 3 && len <= kCmdRecordLenMax
                        && data[off + 12] == (uint8_t)(len - (kRecordHeaderLen + 3));
    case REC_FRAME: return len == kFrameRecordLen;
    case REC_STATE: return len == kStateRecordLen;
    default:        return false;
  }
}

inline void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
inline void put32(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

inline bool heapTooClose() { return (uint32_t)__brkval >= kRingBase - kHeapGuardBytes; }

// Overwrite-oldest: advance read_off past whole records until `need` bytes
// are free. Returns the number of RECORDS evicted (PADs are not records).
uint32_t makeRoom(uint32_t need) {
  uint32_t evicted = 0;
  while (freeBytes() < need) {
    uint32_t r = hdr->read_off;
    if (r == hdr->write_off) break;  // empty yet no room: impossible (need << kDataSize)
    uint8_t len = data[r];
    if (len == 0 || r + len > kDataSize) {  // corrupt chain (should never happen): discard it
      hdr->read_off = hdr->write_off;
      break;
    }
    if (!isPadAt(r)) ++evicted;
    uint32_t end = r + len;
    hdr->read_off = (end == kDataSize) ? 0 : end;
  }
  return evicted;
}

// Append one fully-built record (rec[0] = len, rec[1] = type, rec[6..9] = t_us;
// seq is stamped here). Evicts the oldest records if needed. Returns the
// number of records evicted. Reset-safe ordering at every step:
//   1. eviction published (header with the advanced read_off flushed) BEFORE
//      any byte of the old records is overwritten;
//   2. PAD + record bytes written and flushed beyond write_off (invisible);
//   3. header (write_off, next_seq, checksum) flushed — the record is live.
uint32_t appendRaw(uint8_t *rec, uint8_t len) {
  uint32_t tail = kDataSize - hdr->write_off;
  uint32_t need = (tail < len) ? tail + len : len;  // PAD the tail, then the record at 0
  uint32_t evicted = makeRoom(need);
  if (evicted) {
    hdr->dropped += evicted;
    sealHeader();
  }

  uint32_t w = hdr->write_off;
  if (tail < len) {
    data[w] = (uint8_t)tail;             // PAD: len = remaining tail bytes
    if (tail >= 2) data[w + 1] = REC_PAD;
    arm_dcache_flush(data + w, tail);
    w = 0;
  }

  uint32_t seq = hdr->next_seq;
  if (seq == 0 || seq == 0xFFFFFFFFUL) seq = 1;  // 0 = "none" in block headers, 0xFFFFFFFF = "no ack"
  put32(rec + 2, seq);
  memcpy(data + w, rec, len);
  arm_dcache_flush(data + w, len);  // record lines first ...

  uint32_t nw = w + len;
  hdr->write_off = (nw == kDataSize) ? 0 : nw;
  hdr->next_seq  = seq + 1;
  sealHeader();                     // ... then publish it via the header
  return evicted;
}

void buildState(uint8_t *rec, uint32_t t_us, uint8_t kind, uint8_t code, uint16_t arg) {
  rec[0] = kStateRecordLen;
  rec[1] = REC_STATE;
  put32(rec + 6, t_us);
  rec[10] = kind;
  rec[11] = code;
  put16(rec + 12, arg);
}

// Heap guard tripped: leave a marker for the post-reboot reader if the ring
// is still intact (the heap has not reached the header yet), then stop
// touching the ring for the rest of this boot.
void tripHeapGuard() {
  heap_collision_ = true;
  if ((uint32_t)__brkval < kRingBase) {
    uint8_t ov[kStateRecordLen];
    buildState(ov, micros(), ST_RING_OVERRUN, kOverrunCodeHeapCollision, 0);
    appendRaw(ov, kStateRecordLen);
    hdr->flags &= (uint8_t)~kFlagEventsEnabled;
    sealHeader();
  }
  enabled_  = false;
  synth_on_ = false;
  ring_ok_  = false;
}

// Shared tail of every producer: heap guard, append, eviction marker.
void append(uint8_t *rec, uint8_t len) {
  if (heapTooClose()) {
    tripHeapGuard();
    return;
  }
  uint32_t evicted = appendRaw(rec, len);
  if (evicted) {
    uint32_t now = micros();
    if (!evicted_ever_ || (now - last_evict_us_) >= kEvictEpisodeGapUs) {
      // New eviction episode: one marker carrying the count since the last one.
      uint32_t since = hdr->dropped - dropped_reported_;
      uint8_t ov[kStateRecordLen];
      buildState(ov, now, ST_RING_OVERRUN, kOverrunCodeEvicted,
                 since > 0xFFFF ? 0xFFFF : (uint16_t)since);
      appendRaw(ov, kStateRecordLen);  // may evict more; counted toward the next marker
      dropped_reported_ = hdr->dropped;
    }
    evicted_ever_  = true;
    last_evict_us_ = now;
  }
}

// Boot-time header validation: magic/version/checksum and cursors in range.
bool validHeader() {
  if (hdr->magic != kMagic || hdr->ver != kVersion) return false;
  if (hdr->checksum != computeCheck(*hdr)) return false;
  if (hdr->write_off >= kDataSize || hdr->read_off >= kDataSize) return false;
  if (hdr->next_seq == 0) return false;
  return true;
}

// Walk the kept chain read_off → write_off and truncate at the first
// inconsistent record (zero len, runs past the ring end or past write_off,
// unknown type). With the commit order above this never fires; it is the
// safety net for "tolerate a partially written trailing record, never wipe".
void repairChain() {
  uint32_t off = hdr->read_off, w = hdr->write_off, steps = 0;
  while (off != w) {
    if (!wellFormedAt(off, w) || ++steps > kDataSize) {
      hdr->write_off = off;  // everything from here on is not a decodable record
      return;
    }
    uint32_t end = off + data[off];
    off = (end == kDataSize) ? 0 : end;
  }
}

void initRing() {
  hdr->magic      = kMagic;
  hdr->ver        = kVersion;
  hdr->flags      = kFlagEventsEnabled;
  hdr->reserved   = 0;
  hdr->write_off  = 0;
  hdr->read_off   = 0;
  hdr->next_seq   = 1;   // first_seq == 0 in a block header means "no records"
  hdr->dropped    = 0;
  hdr->boot_count = 0;
  sealHeader();
}

}  // namespace

void begin() {
  // Heap guard at boot: if the heap is already within kHeapGuardBytes of the
  // ring (it would take ~390 KiB of malloc before setup() in this build),
  // never touch the region — a valid ring in RAM is left exactly as it is.
  if (heapTooClose()) {
    heap_collision_ = true;
    ring_ok_        = false;
    return;
  }
  ring_ok_ = true;

  if (validHeader()) {
    kept_ = true;
    repairChain();
    ++hdr->boot_count;
    hdr->flags |= kFlagEventsEnabled;  // recording is on by default at every boot
    sealHeader();
  } else {
    kept_ = false;
    initRing();
  }
  enabled_          = true;
  synth_on_         = false;  // synthetic producer is always off at boot
  synth_rate_       = 0;
  dropped_reported_ = hdr->dropped;
  evicted_ever_     = false;

  // Boot marker: reset cause + what the previous boot was doing (Health::begin()
  // must already have run so both are harvested).
  uint16_t arg = ((uint16_t)Health::stats.prev_last_op << 8)
               | (Health::stats.prev_valid ? 1 : 0)
               | ((Health::stats.prev_valid && Health::stats.prev_isr_last == Health::ISR_WDOG) ? 2 : 0);
  state(ST_BOOT, (uint8_t)(Health::stats.reset_cause & 0xFF), arg);
}

void service() {
  if (!ring_ok_) return;
  if (heapTooClose()) {  // also caught per append; this catches it while idle
    tripHeapGuard();
    return;
  }
  if (!synth_on_) return;
  uint32_t now = micros();
  uint16_t steps = 0;
  while ((uint32_t)(now - synth_last_us_) >= synth_period_us_) {
    synth_last_us_ += synth_period_us_;
    uint8_t rec[kRecordHeaderLen + 3 + 4];
    rec[0] = sizeof(rec);
    rec[1] = REC_CMD;
    put32(rec + 6, synth_last_us_);   // scheduled time, so the stream's period is exact
    rec[10] = kSyntheticCmd;
    rec[11] = 0;
    rec[12] = 4;
    put32(rec + 13, synth_counter_++);
    append(rec, sizeof(rec));          // not gated by events_enabled: a pure synthetic stream is a valid T1 setup
    if (!ring_ok_) return;             // heap guard tripped mid-burst
    if (++steps >= kSynthCatchUpMax) { synth_last_us_ = now; break; }
  }
}

void configure(uint8_t flags, int32_t rate_hz) {
  if (!ring_ok_) return;  // heap guard tripped: stays disabled until reboot
  if (heapTooClose()) { tripHeapGuard(); return; }
  bool on    = (flags & kFlagEventsEnabled) != 0;
  bool synth = (flags & kFlagSynthetic) != 0;
  if (on && !enabled_) {
    enabled_ = true;
    hdr->flags |= kFlagEventsEnabled;
    sealHeader();
  }
  if (rate_hz >= 0) synth_rate_ = (uint16_t)(rate_hz > 0xFFFF ? 0xFFFF : rate_hz);
  synth_on_        = synth && synth_rate_ > 0;
  synth_period_us_ = synth_on_ ? 1000000UL / synth_rate_ : 0;
  synth_last_us_   = micros();
  synth_counter_   = 0;
  // Recorded while still (or newly) enabled, so a disable leaves a last record
  // saying so and an enable starts with one.
  state(ST_TELEMETRY, flags, synth_rate_);
  if (!on && enabled_) {
    enabled_ = false;
    hdr->flags &= (uint8_t)~kFlagEventsEnabled;
    sealHeader();
  }
}

bool enabled()       { return enabled_; }
bool syntheticOn()   { return synth_on_; }
bool ringOk()        { return ring_ok_; }
bool heapCollision() { return heap_collision_; }

void cmd(uint32_t t_rx_us, uint8_t cmd_byte, uint8_t status,
         const uint8_t *req, uint8_t req_len) {
  if (!enabled_) return;
  if (req_len > kCmdPayloadMax) req_len = kCmdPayloadMax;
  uint8_t rec[kCmdRecordLenMax];
  uint8_t len = (uint8_t)(kRecordHeaderLen + 3 + req_len);
  rec[0] = len;
  rec[1] = REC_CMD;
  put32(rec + 6, t_rx_us);
  rec[10] = cmd_byte;
  rec[11] = status;
  rec[12] = req_len;
  for (uint8_t i = 0; i < req_len; ++i) rec[13 + i] = req[i];
  append(rec, len);
}

void frame(uint32_t t_us, uint16_t idx, uint16_t pattern,
           uint32_t sd_load_us, uint32_t spi_us) {
  if (!enabled_) return;
  uint8_t rec[kFrameRecordLen];
  rec[0] = kFrameRecordLen;
  rec[1] = REC_FRAME;
  put32(rec + 6, t_us);
  put16(rec + 10, idx);
  put16(rec + 12, pattern);
  put32(rec + 14, sd_load_us);
  put16(rec + 18, spi_us > 0xFFFF ? 0xFFFF : (uint16_t)spi_us);
  append(rec, kFrameRecordLen);
}

void state(uint8_t kind, uint8_t code, uint16_t arg) {
  if (!enabled_) return;
  uint8_t rec[kStateRecordLen];
  buildState(rec, micros(), kind, code, arg);
  append(rec, kStateRecordLen);
}

void ack(uint32_t ack_seq) {
  if (!ring_ok_ || ack_seq == 0xFFFFFFFFUL) return;
  if (heapTooClose()) { tripHeapGuard(); return; }
  // A seq this incarnation has not generated yet (e.g. a host still holding an
  // ack from before a power cycle re-initialised the ring) is ignored — it
  // would otherwise free records the host has never seen. The walk below only
  // ever advances read_off forward, so it can never move backwards.
  if (ack_seq >= hdr->next_seq) return;
  uint32_t off = hdr->read_off, w = hdr->write_off;
  bool moved = false;
  while (off != w) {
    if (!wellFormedAt(off, w)) break;   // corrupt chain: stop, never run past it
    uint8_t len = data[off];
    if (!isPadAt(off) && seqAt(off) > ack_seq) break;
    uint32_t end = off + len;
    off = (end == kDataSize) ? 0 : end;
    moved = true;
  }
  if (moved) {
    hdr->read_off = off;
    sealHeader();
  }
}

size_t peek(uint8_t *dst, size_t max_bytes, BlockInfo &info) {
  info.first_seq = 0;
  info.n_records = 0;
  info.more      = false;
  if (!ring_ok_) return 0;
  if (heapTooClose()) { tripHeapGuard(); return 0; }
  uint32_t off = hdr->read_off, w = hdr->write_off;
  size_t n = 0;
  while (off != w) {
    if (!wellFormedAt(off, w)) break;   // never hand the host a record it cannot decode
    uint8_t len = data[off];
    if (!isPadAt(off)) {
      if (n + len > max_bytes) break;   // whole records only; the rest waits
      memcpy(dst + n, data + off, len);
      if (info.n_records == 0) info.first_seq = seqAt(off);
      n += len;
      ++info.n_records;
    }
    uint32_t end = off + len;
    off = (end == kDataSize) ? 0 : end;
  }
  info.more = (off != w);
  return n;
}

uint8_t blockFlags() {
  uint8_t f = 0;
  if (enabled_)                      f |= kBlockFlagEvents;
  if (ring_ok_ && hdr->boot_count)   f |= kBlockFlagSurvivedBoot;
  if (heap_collision_)               f |= kBlockFlagHeapCollision;
  if (synth_on_)                     f |= kBlockFlagSynthetic;
  return f;
}

uint32_t dropped()         { return ring_ok_ ? hdr->dropped    : 0; }
uint32_t bootCount()       { return ring_ok_ ? hdr->boot_count : 0; }
uint32_t nextSeq()         { return ring_ok_ ? hdr->next_seq   : 0; }
uint32_t pendingBytes()    { return ring_ok_ ? usedBytes()     : 0; }
bool     keptAcrossReset() { return kept_; }

}  // namespace Telemetry
