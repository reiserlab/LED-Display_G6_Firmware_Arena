#!/usr/bin/env python3
"""Standalone drainer for the G6 controller telemetry ring (0xA8 / 0xA9).

Polls GET_TELEMETRY_BLOCK at --hz, looping on `more` inside each poll so the
ring is emptied every cycle, acknowledging each block with the NEXT request
(lossless: a lost or timed-out reply is simply re-served). Decodes every record
(src/Telemetry.h layout, tests/telemetry_codec.py) and prints one JSON object
per line to stdout, tracks seq gaps, and prints a summary on Ctrl-C.

HANDSHAKE (the reference behaviour for any host): GET_CONTROLLER_INFO (0xC2)
must advertise capability bit 7 (health) before GET_FIRMWARE_VERSION (0xCB) is
sent, and 0xCB `flags` must have bit 2 (telemetry ring compiled in) before ANY
0xA8 / 0xA9 is sent — including --disable and --no-enable. Health-only
firmware (22b756d) answers an unknown 0xA8 with a CE 01 error glyph on the
arena. `--self-test` runs this handshake against an in-process fake
health-only controller and asserts neither opcode was sent.

Bench experiment T1 without a browser (webDisplayTools docs/development/
controller-telemetry-ring-buffer-proposal.md § 5). The link is single-flight and
there is ONE USB-CDC port, so this runs INSTEAD of a driver: use --synthetic RATE
to have the controller generate a dummy record stream at a known rate (drain
throughput / loss), or drain whatever the controller is doing on its own (a
Mode-2 playback started earlier, an ALL_ON, an error glyph, the boot record of
the previous crash). It is also the reference decoder for the Studio poller.

Output line shapes (all fields little-endian integers unless noted):
  {"rec":"block", ...}                            bookkeeping (first block, dropped changes)
  {"rec":"gap", "expected":n, "got":m, ...}       a seq gap (eviction or loss)
  {"rec":"cmd",   "seq":n, "t_us":u, "rx_ms":m, "cmd":0x70, "status":0, "params":"0a00"}
  {"rec":"frame", "seq":n, "t_us":u, "rx_ms":m, "idx":10, "pattern":1, "sd_load_us":1234, "spi_us":800}
  {"rec":"state", "seq":n, "t_us":u, "rx_ms":m, "kind":2, "kind_name":"state_change", "code":4, "arg":1, ...}
`rx_ms` is the host's time.time()*1000 when the block carrying the record was
received; `t_us` is the controller's raw micros() (wraps every 71.6 min — pair
with the block's t_now_us / rx_ms to fit a clock, see proposal § 8). seq is the
controller's APPEND order; t_us may be non-monotonic within one dispatch (a CMD
carries its dispatch-entry time but is appended after its handler's STATEs).

Usage:
    python3 scripts/telemetry_drain.py --port /dev/cu.usbmodem123456
    python3 scripts/telemetry_drain.py --port COM7 --hz 20 --raw-out run.ctl.bin
    python3 scripts/telemetry_drain.py --port /dev/ttyACM0 --no-ack   # peek only: ONE request per poll, frees nothing
    python3 scripts/telemetry_drain.py --port ... --hz 20 --synthetic 5000 --quiet   # T1: 5000 rec/s dummy stream
    python3 scripts/telemetry_drain.py --port ... --disable            # turn recording off and exit
    python3 scripts/telemetry_drain.py --self-test                     # no hardware: handshake unit check

Requires pyserial (the repo's pixi / PlatformIO env has it). Exit codes:
0 normal (Ctrl-C), 1 could not open the port / controller not answering /
no telemetry ring in this firmware, 2 bad arguments.
"""

from __future__ import annotations

import argparse
import json
import os
import struct
import sys
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)

from tests.commands import (  # noqa: E402
    GET_CONTROLLER_INFO_CMD,
    GET_FIRMWARE_VERSION_CMD,
    GET_TELEMETRY_BLOCK_CMD,
    SET_TELEMETRY_CMD,
)
from tests.telemetry_codec import (  # noqa: E402
    BLOCK_HEADER_LEN,
    NO_ACK,
    RECORD_BYTES_MAX,
    SET_FLAG_EVENTS,
    SET_FLAG_SYNTHETIC,
    SeqTracker,
    build_get_block,
    build_set_telemetry,
    parse_block,
)

CAP_HEALTH = 0x80          # 0xC2 capability bit 7: 0xCA/0xCB exist
FW_FLAG_TELEMETRY = 0x04   # 0xCB flags bit 2: telemetry ring compiled in (0xA8/0xA9 exist)
MAX_BYTES_MIN = 32         # below this a block can carry at most one CMD record: no useful progress


class NoTelemetryRing(RuntimeError):
    """The connected firmware does not carry the telemetry ring (or cannot be identified)."""


def handshake(t, timeout: float = 1.0) -> dict:
    """0xC2 → (health bit) → 0xCB → (flags bit 2). Raises NoTelemetryRing BEFORE
    any 0xA8/0xA9 could be sent. Returns {"capabilities":…, "fw_flags":…, "fw_label":…}."""
    st, echo, info, _ = t.command(GET_CONTROLLER_INFO_CMD, timeout=timeout)
    if st != 0 or echo != GET_CONTROLLER_INFO_CMD or len(info) < 2:
        raise NoTelemetryRing("GET_CONTROLLER_INFO failed or too short")
    caps = info[1]
    if not (caps & CAP_HEALTH):
        raise NoTelemetryRing("capability bit 7 (health) not set: firmware predates 0xCB, no telemetry ring")
    st, echo, ver, _ = t.command(GET_FIRMWARE_VERSION_CMD, timeout=timeout)
    if st != 0 or echo != GET_FIRMWARE_VERSION_CMD or len(ver) < 4:
        raise NoTelemetryRing("GET_FIRMWARE_VERSION failed or too short")
    flags = ver[3]
    label = ""
    if len(ver) >= 46:
        label = f"{bytes(ver[4:12]).decode('ascii', 'replace').strip()} {bytes(ver[22:46]).decode('ascii', 'replace').strip()}"
    if not (flags & FW_FLAG_TELEMETRY):
        raise NoTelemetryRing(f"firmware {label or '?'} has no telemetry ring (0xCB flags={flags:#04x} lacks bit 2)")
    return {"capabilities": caps, "fw_flags": flags, "fw_label": label}


def emit(obj: dict, quiet: bool = False) -> None:
    if not quiet:
        sys.stdout.write(json.dumps(obj, separators=(",", ":")) + "\n")
        sys.stdout.flush()


class _FakeHealthOnlyController:
    """Answers 0xC2 with health set and 0xCB with flags bit 2 CLEAR (build 22b756d);
    records every opcode it is sent so the self-test can prove 0xA8/0xA9 never went out."""

    def __init__(self):
        self.sent: list[int] = []

    def command(self, cmd, params=b"", timeout=1.0):
        self.sent.append(cmd)
        if cmd == GET_CONTROLLER_INFO_CMD:
            return 0, cmd, bytes([1, 0xA3]) + bytes(6), []
        if cmd == GET_FIRMWARE_VERSION_CMD:
            return 0, cmd, bytes([1, 2, 10, 0x00]) + b"22b756dd" + b"2026-09-11" + b"feat/controller-health-2".ljust(24), []
        # Real health-only firmware: status 1 "Unknown command" + a CE 01 glyph on the arena.
        return 1, cmd, b"Unknown command", []


def self_test() -> int:
    fake = _FakeHealthOnlyController()
    try:
        handshake(fake)
    except NoTelemetryRing as e:
        print(f"self-test: handshake refused as expected: {e}", file=sys.stderr)
    else:
        print("self-test FAILED: handshake accepted a health-only controller", file=sys.stderr)
        return 1
    forbidden = {SET_TELEMETRY_CMD, GET_TELEMETRY_BLOCK_CMD}
    leaked = [c for c in fake.sent if c in forbidden]
    if leaked:
        print(f"self-test FAILED: sent {[hex(c) for c in leaked]} to a health-only controller", file=sys.stderr)
        return 1
    assert fake.sent == [GET_CONTROLLER_INFO_CMD, GET_FIRMWARE_VERSION_CMD], fake.sent
    print("self-test OK: 0xC2, 0xCB only; no 0xA8/0xA9 sent to a health-only controller", file=sys.stderr)
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="USB-CDC device node (e.g. /dev/cu.usbmodem123456, COM7)")
    ap.add_argument("--hz", type=float, default=10.0, help="poll cadence between drain bursts (default 10)")
    ap.add_argument("--max-bytes", type=int, default=RECORD_BYTES_MAX,
                    help=f"record bytes requested per block ({MAX_BYTES_MIN}..{RECORD_BYTES_MAX}; firmware clamps to {RECORD_BYTES_MAX})")
    ap.add_argument("--no-ack", action="store_true",
                    help="peek only: never acknowledge, ONE request per poll (no `more` loop), ring is not freed")
    ap.add_argument("--no-enable", action="store_true", help="do not send SET_TELEMETRY(1) at start")
    ap.add_argument("--synthetic", type=int, default=0, metavar="RATE",
                    help="T1: also turn on the synthetic producer (flags bit7) at RATE records/s; turned off on exit")
    ap.add_argument("--disable", action="store_true", help="send SET_TELEMETRY(0) and exit")
    ap.add_argument("--raw-out", default=None,
                    help="also append every raw block payload to this file as {rx_ms u64, n u16} + bytes")
    ap.add_argument("--records-only", action="store_true", help="suppress block/bookkeeping lines")
    ap.add_argument("--quiet", action="store_true", help="print only the final summary")
    ap.add_argument("--timeout", type=float, default=1.0, help="per-command reply timeout, s")
    ap.add_argument("--self-test", action="store_true", help="no hardware: run the handshake against a fake health-only controller")
    args = ap.parse_args()

    if args.self_test:
        return self_test()
    if not args.port:
        ap.error("--port is required (or use --self-test)")
    if not (MAX_BYTES_MIN <= args.max_bytes <= 0xFFFF):
        print(f"error: --max-bytes must be >= {MAX_BYTES_MIN} (a block smaller than that cannot make progress "
              f"past a 21-byte CMD record); got {args.max_bytes}", file=sys.stderr)
        return 2

    from tests.transport import SerialTransport  # pyserial only needed for real hardware
    t = SerialTransport(args.port)
    try:
        t.open()
        ident = handshake(t, args.timeout)
    except NoTelemetryRing as e:
        print(f"error: {e} — not sending 0xA8/0xA9", file=sys.stderr)
        try:
            t.close()
        except Exception:  # noqa: BLE001
            pass
        return 1
    except Exception as e:  # noqa: BLE001
        print(f"error: cannot talk to controller on {args.port}: {e}", file=sys.stderr)
        return 1
    print(f"controller: fw {ident['fw_label']} caps={ident['capabilities']:#04x} "
          f"fw_flags={ident['fw_flags']:#04x} (telemetry ring present)", file=sys.stderr)

    if args.disable:
        st, _, _, _ = t.command(SET_TELEMETRY_CMD, build_set_telemetry(0), timeout=args.timeout)
        print(f"SET_TELEMETRY(0) status={st}", file=sys.stderr)
        t.close()
        return 0 if st == 0 else 1
    if not args.no_enable or args.synthetic:
        flags = SET_FLAG_EVENTS | (SET_FLAG_SYNTHETIC if args.synthetic > 0 else 0)
        st, _, _, _ = t.command(SET_TELEMETRY_CMD, build_set_telemetry(flags, args.synthetic), timeout=args.timeout)
        if st != 0:
            print(f"warning: SET_TELEMETRY(0x{flags:02X}, rate={args.synthetic}) status={st}", file=sys.stderr)

    raw_f = open(args.raw_out, "ab") if args.raw_out else None
    tracker = SeqTracker()
    blocks = bursts = errors = 0
    bytes_total = 0
    dropped_last = None
    ack_seq = NO_ACK
    period = 1.0 / args.hz if args.hz > 0 else 0.0
    t_start = time.monotonic()
    first = True

    try:
        while True:
            burst_start = time.monotonic()
            bursts += 1
            while True:
                try:
                    st, echo, payload, _ = t.command(GET_TELEMETRY_BLOCK_CMD,
                                                     build_get_block(ack_seq, args.max_bytes),
                                                     timeout=args.timeout)
                except RuntimeError as e:  # timeout: re-ask with the same ack next time
                    errors += 1
                    emit({"rec": "error", "err": str(e), "ack_seq": ack_seq}, args.quiet)
                    break
                rx_ms = time.time() * 1000.0
                if st != 0:
                    errors += 1
                    emit({"rec": "error", "status": st, "payload": bytes(payload).hex()}, args.quiet)
                    break
                payload = bytes(payload)
                try:
                    hdr, recs = parse_block(payload)
                except ValueError as e:
                    errors += 1
                    emit({"rec": "error", "err": f"decode: {e}", "payload": payload.hex()}, args.quiet)
                    break
                blocks += 1
                bytes_total += len(payload) - BLOCK_HEADER_LEN
                if raw_f:
                    # Durable BEFORE the ack: once ack_seq advances the controller may
                    # free these records, so they must already be on disk.
                    raw_f.write(struct.pack("<QH", int(rx_ms), len(payload)) + payload)
                    raw_f.flush()
                if first or dropped_last != hdr.dropped:
                    emit({"rec": "block", "rx_ms": rx_ms, "t_now_us": hdr.t_now_us, "first_seq": hdr.first_seq,
                          "n": hdr.n_records, "dropped": hdr.dropped, "more": hdr.more, "flags": hdr.flags,
                          "boot_count": hdr.boot_count}, args.quiet or args.records_only)
                    first = False
                dropped_last = hdr.dropped
                gaps_before = len(tracker.gaps)
                tracker.feed(recs)
                for g in tracker.gaps[gaps_before:]:
                    emit({"rec": "gap", "expected": g[0], "got": g[1], "rx_ms": rx_ms},
                         args.quiet or args.records_only)
                for r in recs:
                    d = r.as_dict()
                    d["rx_ms"] = round(rx_ms, 3)
                    d["t_now_us"] = hdr.t_now_us
                    emit(d, args.quiet)
                if args.no_ack:
                    break  # peek mode: without an ack `more` never clears — one request per poll
                if recs:
                    ack_seq = recs[-1].seq
                if not hdr.more:
                    break
            if period > 0:
                sleep_for = period - (time.monotonic() - burst_start)
                if sleep_for > 0:
                    time.sleep(sleep_for)
    except KeyboardInterrupt:
        pass
    finally:
        if raw_f:
            raw_f.close()  # everything delivered is on disk before the final ack frees it
        if args.synthetic:
            try:  # leave the controller quiet: events on, synthetic off
                t.command(SET_TELEMETRY_CMD, build_set_telemetry(SET_FLAG_EVENTS, 0), timeout=args.timeout)
            except Exception:  # noqa: BLE001
                pass
        if not args.no_ack and ack_seq != NO_ACK:
            try:  # free the last delivered block so a later drainer starts clean
                t.command(GET_TELEMETRY_BLOCK_CMD, build_get_block(ack_seq, 0), timeout=args.timeout)
            except Exception:  # noqa: BLE001
                pass
        t.close()
        elapsed = time.monotonic() - t_start
        summary = {
            "rec": "summary", "elapsed_s": round(elapsed, 3), "bursts": bursts, "blocks": blocks,
            "records": tracker.records, "by_type": tracker.by_type, "record_bytes": bytes_total,
            "bytes_per_s": round(bytes_total / elapsed, 1) if elapsed > 0 else None,
            "seq_gaps": len(tracker.gaps), "last_seq": tracker.last_seq,
            "controller_dropped": dropped_last, "errors": errors,
        }
        print(json.dumps(summary, separators=(",", ":")), file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
