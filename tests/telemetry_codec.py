"""Wire codec for the controller telemetry ring (0xA8 / 0xA9) — mirrors src/Telemetry.h.

Shared by tests/test_telemetry.py (HIL) and scripts/telemetry_drain.py (bench
drainer). Pure functions, no transport dependency.

SET_TELEMETRY (0xA8) request:        [04 A8 flags, rate u16 LE]  or  [02 A8 flags]
    flags bit0 = record events (default ON at boot), bit7 = synthetic producer
    (dummy CMD records, cmd 0xFE, at `rate` records/s; off at boot)
GET_TELEMETRY_BLOCK (0xA9) request:  [08 A9 ack_seq u32 LE, max_bytes u16 LE, flags u8]
GET_TELEMETRY_BLOCK reply payload:   18-byte header, then records verbatim
    off  0  u32 t_now_us     controller micros() at reply time
    off  4  u32 first_seq    seq of the first returned record; 0 if none
    off  8  u16 n_records
    off 10  u32 dropped      records evicted (overwrite-oldest) since the ring was initialised
    off 14  u8  more         1 if unread records remain after this block
    off 15  u8  flags        bit0 events_enabled, bit1 survived a reboot,
                             bit2 ring disabled (heap collision), bit3 synthetic producer on
    off 16  u16 boot_count

Record: len u8, type u8, seq u32, t_us u32, payload
    type 1 CMD   : cmd u8, status u8, plen u8, payload[plen <= 8]         (13..21 B)
    type 2 FRAME : idx u16, pattern u16, sd_load_us u32, spi_us u16        (20 B)
    type 3 STATE : kind u8, code u8, arg u16                               (14 B)
PAD records (type 0) are internal to the ring and never returned.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import Any

NO_ACK = 0xFFFFFFFF
BLOCK_HEADER_LEN = 18
BLOCK_HEADER_FMT = "<IIHIBBH"
RECORD_HEADER_LEN = 10
RECORD_BYTES_MAX = 178  # firmware cap: 196-byte max framed payload minus the 18-byte header

FRAME_RECORD_LEN = 20
STATE_RECORD_LEN = 14

REC_PAD, REC_CMD, REC_FRAME, REC_STATE = 0, 1, 2, 3
REC_NAMES = {REC_CMD: "cmd", REC_FRAME: "frame", REC_STATE: "state"}

ST_BOOT, ST_STATE_CHANGE, ST_ERROR_GLYPH, ST_SD_SLOW, ST_RING_OVERRUN, ST_TELEMETRY, ST_SD_OPEN = range(1, 8)
STATE_KIND_NAMES = {
    ST_BOOT: "boot",
    ST_STATE_CHANGE: "state_change",
    ST_ERROR_GLYPH: "error_glyph",
    ST_SD_SLOW: "sd_slow",
    ST_RING_OVERRUN: "ring_overrun",
    ST_TELEMETRY: "telemetry",
    ST_SD_OPEN: "sd_open",
}
OVERRUN_CODE_EVICTED = 0x00
OVERRUN_CODE_HEAP_COLLISION = 0xFF
SYNTHETIC_CMD = 0xFE

ARENA_STATE_NAMES = ["ALL_OFF", "ALL_ON", "STREAMING_FRAME", "OPEN_LOOP", "SHOW_FRAME",
                     "CLOSED_LOOP", "PSRAM_PLAY", "ERROR_DISPLAY"]

# SET_TELEMETRY request flags
SET_FLAG_EVENTS = 0x01
SET_FLAG_SYNTHETIC = 0x80

# GET_TELEMETRY_BLOCK header flags
FLAG_EVENTS_ENABLED = 0x01
FLAG_SURVIVED_REBOOT = 0x02
FLAG_HEAP_COLLISION = 0x04
FLAG_SYNTHETIC_ON = 0x08


def build_set_telemetry(flags: int, rate_hz: int | None = 0) -> bytes:
    """Params for SET_TELEMETRY (0xA8).

    rate_hz=None → the 2-byte form [02 A8 flags] (rate unchanged on the
    controller); otherwise the 4-byte form [04 A8 flags rate_lo rate_hi].
    """
    if rate_hz is None:
        return bytes([flags & 0xFF])
    return struct.pack("<BH", flags & 0xFF, rate_hz & 0xFFFF)


def build_get_block(ack_seq: int = NO_ACK, max_bytes: int = RECORD_BYTES_MAX, flags: int = 0) -> bytes:
    """Params for GET_TELEMETRY_BLOCK (0xA9): [ack_seq u32, max_bytes u16, flags u8]."""
    return struct.pack("<IHB", ack_seq & 0xFFFFFFFF, max_bytes & 0xFFFF, flags & 0xFF)


@dataclass
class BlockHeader:
    t_now_us: int
    first_seq: int
    n_records: int
    dropped: int
    more: int
    flags: int
    boot_count: int


@dataclass
class Record:
    type: int
    seq: int
    t_us: int
    raw: bytes                       # the whole record, verbatim
    fields: dict[str, Any] = field(default_factory=dict)

    @property
    def name(self) -> str:
        return REC_NAMES.get(self.type, f"type{self.type}")

    def as_dict(self) -> dict[str, Any]:
        d = {"rec": self.name, "seq": self.seq, "t_us": self.t_us}
        d.update(self.fields)
        return d


def parse_block_header(payload: bytes) -> BlockHeader:
    if len(payload) < BLOCK_HEADER_LEN:
        raise ValueError(f"block payload too short: {len(payload)} < {BLOCK_HEADER_LEN}")
    return BlockHeader(*struct.unpack_from(BLOCK_HEADER_FMT, payload, 0))


def _decode_payload(rtype: int, body: bytes) -> dict[str, Any]:
    if rtype == REC_CMD:
        if len(body) < 3:
            raise ValueError("CMD record payload too short")
        cmd, status, plen = body[0], body[1], body[2]
        if len(body) != 3 + plen:
            raise ValueError(f"CMD record plen={plen} but {len(body) - 3} payload bytes present")
        d = {"cmd": cmd, "status": status, "params": body[3:3 + plen].hex()}
        if cmd == SYNTHETIC_CMD and plen == 4:
            d["counter"] = struct.unpack("<I", body[3:7])[0]
        return d
    if rtype == REC_FRAME:
        if len(body) != FRAME_RECORD_LEN - RECORD_HEADER_LEN:
            raise ValueError(f"FRAME record payload is {len(body)} B, expected 10")
        idx, pattern, sd_load_us, spi_us = struct.unpack("<HHIH", body)
        return {"idx": idx, "pattern": pattern, "sd_load_us": sd_load_us, "spi_us": spi_us}
    if rtype == REC_STATE:
        if len(body) != STATE_RECORD_LEN - RECORD_HEADER_LEN:
            raise ValueError(f"STATE record payload is {len(body)} B, expected 4")
        kind, code, arg = struct.unpack("<BBH", body)
        d = {"kind": kind, "kind_name": STATE_KIND_NAMES.get(kind, f"kind{kind}"), "code": code, "arg": arg}
        if kind == ST_STATE_CHANGE and code < len(ARENA_STATE_NAMES):
            d["state_name"] = ARENA_STATE_NAMES[code]
        if kind == ST_BOOT:
            d["prev_breadcrumb_op"] = arg >> 8
            d["prev_breadcrumb_valid"] = arg & 1
        if kind == ST_RING_OVERRUN:
            d["heap_collision"] = code == OVERRUN_CODE_HEAP_COLLISION
        if kind == ST_TELEMETRY:
            d["events"] = bool(code & SET_FLAG_EVENTS)
            d["synthetic"] = bool(code & SET_FLAG_SYNTHETIC)
            d["rate_hz"] = arg
        return d
    raise ValueError(f"unknown record type {rtype}")


def parse_records(buf: bytes) -> list[Record]:
    """Split a block's record bytes (after the 18-byte header) into Records.

    Strict: raises ValueError on a zero length, a record running past the end
    of the buffer (i.e. a split record — the firmware must never emit one), or
    a PAD (the firmware must never return one).
    """
    out: list[Record] = []
    i = 0
    while i < len(buf):
        rlen = buf[i]
        if rlen < RECORD_HEADER_LEN:
            raise ValueError(f"record at {i} has len {rlen} < {RECORD_HEADER_LEN} (PAD or corrupt)")
        if i + rlen > len(buf):
            raise ValueError(f"record at {i} (len {rlen}) runs past block end ({len(buf)}) — split record")
        raw = bytes(buf[i:i + rlen])
        rtype = raw[1]
        if rtype == REC_PAD:
            raise ValueError(f"PAD record returned at {i}")
        seq, t_us = struct.unpack_from("<II", raw, 2)
        out.append(Record(rtype, seq, t_us, raw, _decode_payload(rtype, raw[RECORD_HEADER_LEN:])))
        i += rlen
    return out


def parse_block(payload: bytes) -> tuple[BlockHeader, list[Record]]:
    hdr = parse_block_header(payload)
    recs = parse_records(payload[BLOCK_HEADER_LEN:])
    if len(recs) != hdr.n_records:
        raise ValueError(f"header says {hdr.n_records} records, decoded {len(recs)}")
    if recs and recs[0].seq != hdr.first_seq:
        raise ValueError(f"header first_seq={hdr.first_seq} but first record seq={recs[0].seq}")
    if not recs and hdr.first_seq != 0:
        raise ValueError(f"empty block but first_seq={hdr.first_seq}")
    return hdr, recs


class SeqTracker:
    """Tracks record sequence continuity across blocks and reboots."""

    def __init__(self):
        self.last_seq: int | None = None
        self.gaps: list[tuple[int, int]] = []   # (expected, got)
        self.records = 0
        self.by_type = {"cmd": 0, "frame": 0, "state": 0}

    def feed(self, recs: list[Record]) -> None:
        for r in recs:
            if self.last_seq is not None and r.seq != self.last_seq + 1:
                self.gaps.append((self.last_seq + 1, r.seq))
            self.last_seq = r.seq
            self.records += 1
            self.by_type[r.name] = self.by_type.get(r.name, 0) + 1
