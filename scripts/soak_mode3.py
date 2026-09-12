#!/usr/bin/env python3
"""Browser-free soak / repro driver for the G6 controller Mode-3 wedge (fw issue #50).

Reproduces what Arena Studio + the FicTrac bridge do on the wire, without a
browser: open a pattern with TRIAL_PARAMS (0x08, mode 3), then stream
SET_FRAME_POSITION (0x70) single-flight at a fixed rate with a FicTrac-like
index walk, while polling controller health once a second. When the controller
stops answering (the browser's rule: >= 3 failures in the last 10 commands) the
driver runs a fixed post-mortem lifecycle -- quiet period, confirmation probe,
slow-timeout probe window -- and then either halts (leaving the controller
wedged for a human) or sends SYSTEM_RESET, reconnects, reads the reset-surviving
breadcrumb from GET_HEALTH, and resumes the soak.

The log is behavior_v2 NDJSON, the same format fictrac-bridge/bridge.py writes,
so the existing run-log readers / analyzers can consume it:

    {"type":"frame_schema","level":"behavior_v2","cols":[...],
     "arena_cols":["t_off","dt","hex","status","rx_off"],"t0":<epoch ms>}
    {"type":"log","event":"run_metadata", ...,"ms":<epoch ms>}
    ["a", t_off_ms, dt_ms, "<request hex>", status|null, rx_off_ms(, "error")]
    {"type":"log","event":"health"|"fault"|"probe"|..., ...,"ms":<epoch ms>}

Requirements: Python 3.10+, pyserial (the firmware repo's pixi / PlatformIO
env has it). Standard library otherwise.

Usage examples:

    # Mode-3 soak at 100 Hz, random walk over SD pattern 1, 10 hours, halt on fault
    python3 soak_mode3.py --port /dev/cu.usbmodem123456 --pattern 1

    # Fast arm (286 Hz) with a wider walk, halt on fault
    python3 soak_mode3.py --port /dev/cu.usbmodem123456 --hz 286 --walk-sigma 3

    # Reset + reconnect + keep soaking after a fault, up to 3 resets
    python3 soak_mode3.py --port /dev/cu.usbmodem123456 --on-fault reset-continue

    # Mode-2 control arm: controller-timed playback at 50 fps, health tick only
    python3 soak_mode3.py --port /dev/cu.usbmodem123456 --mode 2 --fps 50

    # Sequential / fixed / jumpy index sequences
    python3 soak_mode3.py --port COM7 --index-walk seq
    python3 soak_mode3.py --port COM7 --index-walk fixed --fixed-index 7
    python3 soak_mode3.py --port COM7 --index-walk jump --jump-frames 90 --jump-every 50

    # No hardware: in-process fake controller that wedges after 150 commands
    python3 soak_mode3.py --dry-run --dry-run-wedge-after 150 --max-commands 400

Exit codes: 0 normal end (hours elapsed, --max-commands, Ctrl-C), 1 startup
failure (controller never answered / pattern could not be opened), 2 fault with
--on-fault halt (or --max-resets exhausted), 3 reconnect failed after a reset.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import json
import math
import os
import random
import statistics
import struct
import sys
import time
from collections import deque
from typing import Any, Optional

try:
    import serial  # pyserial
except ImportError:  # --dry-run works without it
    serial = None

# --------------------------------------------------------------------------
# Wire protocol (mirrors src/commands.h / tests/commands.py in the firmware repo)
# --------------------------------------------------------------------------

SYSTEM_RESET_CMD = 0x01
TRIAL_PARAMS_CMD = 0x08
STOP_DISPLAY_CMD = 0x30
GET_FRAMES_SENT_CMD = 0x33
SET_FRAME_POSITION_CMD = 0x70
GET_FRAME_POSITION_CMD = 0x72
GET_PATTERN_INFO_CMD = 0x88
GET_CONTROLLER_INFO_CMD = 0xC2
GET_HEALTH_CMD = 0xCA
GET_FIRMWARE_INFO_CMD = 0xE3

CAP_HEALTH_BIT = 0x80  # GET_CONTROLLER_INFO capability bitmap bit 7
DIAG_SENTINEL = 0xFF  # DEBUG_SERIAL diagnostic line: 0xFF <ascii> \n

# GET_HEALTH payload, schema ver 1, 66 bytes LE -- CommandProcessor::handleGetHealth.
# (webDisplayTools' decodeHealth reads only the first 55 bytes; both are accepted.)
HEALTH_FMT = "<BBIIIIIIBIIIIBHIBIBBIBI"
HEALTH_LEN = struct.calcsize(HEALTH_FMT)  # 66
HEALTH_FIELDS = (
    "ver", "flags", "uptime_ms", "loop_count", "loop_max_us", "loop_max_1s_us",
    "sd_reads", "sd_read_max_us", "sd_err", "sd_err_data",
    "frames_sent", "isr_count", "cmd70_count", "state", "cur_frame", "reset_cause",
    "prev_breadcrumb", "prev_breadcrumb_us", "prev_breadcrumb_arg",
    "prev_slow_op", "prev_slow_us", "slow_op", "slow_us",
)
# ver 2 (fw feat/telemetry-ring-2x10 fb11681+): 23-byte watchdog / ISR tail appended at offset 66.
HEALTH_FMT_V2 = HEALTH_FMT + "BIIIBBII"
HEALTH_LEN_V2 = struct.calcsize(HEALTH_FMT_V2)  # 89
HEALTH_FMT_V3 = HEALTH_FMT_V2 + "II"  # + raw WDOG3_CS at boot / now
HEALTH_LEN_V3 = struct.calcsize(HEALTH_FMT_V3)  # 97
HEALTH_FIELDS_V2 = HEALTH_FIELDS + (
    "prev_isr_last", "prev_isr_count", "prev_wdog_pc", "prev_wdog_lr",
    "wdog_flags", "breadcrumb_isr_last", "breadcrumb_isr_count", "wdog_kicks",
)
HEALTH_FIELDS_V3 = HEALTH_FIELDS_V2 + ("wdog_cs_boot", "wdog_cs_now")
HEALTH_ISRS = ("none", "refresh", "dma", "wdog")
HEALTH_FMT_55 = "<BBIIIIIIBIIIIBHIBI"  # the 55-byte prefix (fields up to prev_breadcrumb_us)
HEALTH_LEN_55 = struct.calcsize(HEALTH_FMT_55)
HEALTH_OPS = ("idle", "sd_read", "spi_transfer", "usb_write", "command", "sd_open",
              "cmd_disarm", "cmd_preload", "cmd_arm", "cmd_respond")  # 6-9: 0x70 sub-ops (v2 fw)
ARENA_STATES = ("ALL_OFF", "ALL_ON", "STREAMING_FRAME", "OPEN_LOOP", "SHOW_FRAME",
                "CLOSED_LOOP", "PSRAM_PLAY", "ERROR_DISPLAY")


def build_frame(cmd: int, params: bytes = b"") -> bytes:
    """[length, cmd, params...] -- length excludes itself."""
    body = bytes([cmd]) + params
    return bytes([len(body)]) + body


def encode_trial_params(mode: int, pattern_id: int, frame_rate: int = 0,
                        init_pos: int = 0, gain: int = 0, duration_ticks: int = 0,
                        duty: int = 0) -> bytes:
    """TRIAL_PARAMS (0x08) frame. Wire order verified against the firmware
    (handleTrialParams), the browser (encodeTrialParams) and tests/test_health.py:
        mode u8, pattern_id u16, frame_rate i16, init_pos u16, gain i16,
        duration u16 (10 ms ticks), duty u8 (0 = pattern's stored duty).
    -> [0D 08 mode pat_lo pat_hi fr_lo fr_hi init_lo init_hi gain_lo gain_hi dur_lo dur_hi duty]
    """
    params = struct.pack("<BHhHhHB", mode, pattern_id, frame_rate, init_pos, gain,
                         duration_ticks, duty)
    return build_frame(TRIAL_PARAMS_CMD, params)


def encode_set_frame_position(index: int) -> bytes:
    return build_frame(SET_FRAME_POSITION_CMD, struct.pack("<H", index))  # 03 70 lo hi


def encode_u16_cmd(cmd: int, value: int) -> bytes:
    return build_frame(cmd, struct.pack("<H", value))


def decode_controller_info(payload: bytes) -> dict:
    d: dict[str, Any] = {"version": None, "capability": None, "mac": None, "health_capable": False}
    if len(payload) >= 2:
        d["version"] = payload[0]
        d["capability"] = payload[1]
        d["health_capable"] = bool(payload[1] & CAP_HEALTH_BIT)
    if len(payload) >= 8:
        d["mac"] = ":".join(f"{b:02X}" for b in payload[2:8])
    return d


def decode_pattern_info(payload: bytes) -> dict:
    """GET_PATTERN_INFO (0x88) 12-byte payload: frame_count u16, gs u8, rows u8,
    cols u8, arena u8, observer u8, file_size u32, duty u8."""
    if len(payload) < 2:
        return {}
    d = {"frame_count": struct.unpack_from("<H", payload)[0]}
    if len(payload) >= 12:
        gs, rows, cols, arena, observer, size, duty = struct.unpack_from("<BBBBBIB", payload, 2)
        d.update(gs=gs, rows=rows, cols=cols, arena_id=arena, observer_id=observer,
                 file_size=size, duty=duty)
    return d


def decode_frame_position(payload: bytes) -> dict:
    if len(payload) < 4:
        return {}
    cur, n = struct.unpack_from("<HH", payload)
    return {"cur_frame": cur, "frame_count": n}


def decode_frames_sent(payload: bytes) -> Optional[int]:
    return struct.unpack_from("<I", payload)[0] if len(payload) >= 4 else None


def decode_health(payload: bytes) -> Optional[dict]:
    """GET_HEALTH (0xCA) -> dict. 97-byte ver-3, 89-byte ver-2, 66-byte ver-1, or the 55-byte prefix."""
    if len(payload) >= HEALTH_LEN_V2:
        if len(payload) >= HEALTH_LEN_V3:
            vals = struct.unpack_from(HEALTH_FMT_V3, payload)
            h = dict(zip(HEALTH_FIELDS_V3, vals))
            h["wdog_cs_boot_hex"] = f"0x{h['wdog_cs_boot']:04X}"
            h["wdog_cs_now_hex"] = f"0x{h['wdog_cs_now']:04X}"
            h["wdog_cs_en"] = bool(h["wdog_cs_now"] & 0x80)
        else:
            vals = struct.unpack_from(HEALTH_FMT_V2, payload)
            h = dict(zip(HEALTH_FIELDS_V2, vals))
        wf = h["wdog_flags"]
        h["wdog_armed"] = bool(wf & 0x01)
        h["prev_reset_was_wdog"] = bool(wf & 0x02)
        h["prev_wdog_pc_valid"] = bool(wf & 0x04)
        h["wdog_longop_window"] = bool(wf & 0x10)
        h["wdog_starving"] = bool(wf & 0x20)
        h["wdog_reprogram_failed"] = bool(wf & 0x40)
        h["prev_isr_last_name"] = _name(HEALTH_ISRS, h["prev_isr_last"], "isr")
        h["prev_wdog_pc_hex"] = f"0x{h['prev_wdog_pc']:08X}"
        h["prev_wdog_lr_hex"] = f"0x{h['prev_wdog_lr']:08X}"
    elif len(payload) >= HEALTH_LEN:
        vals = struct.unpack_from(HEALTH_FMT, payload)
        h = dict(zip(HEALTH_FIELDS, vals))
    elif len(payload) >= HEALTH_LEN_55:
        vals = struct.unpack_from(HEALTH_FMT_55, payload)
        h = dict(zip(HEALTH_FIELDS[: len(vals)], vals))
    else:
        return None
    flags = h["flags"]
    h["sd_mounted"] = bool(flags & 0x01)
    h["pattern_open"] = bool(flags & 0x02)
    h["display_active"] = bool(flags & 0x04)
    h["breadcrumb_valid"] = bool(flags & 0x08)
    h["state_name"] = _name(ARENA_STATES, h["state"], "state")
    h["prev_breadcrumb_op"] = _name(HEALTH_OPS, h["prev_breadcrumb"], "op")
    if "prev_breadcrumb_arg" in h:
        # arg = opcode when the previous boot's last op was a command dispatch (op 4)
        h["prev_breadcrumb_arg_hex"] = f"0x{h['prev_breadcrumb_arg']:02X}"
    if "prev_slow_op" in h:
        h["prev_slow_op_name"] = _name(HEALTH_OPS, h["prev_slow_op"], "op")
    if "slow_op" in h:
        h["slow_op_name"] = _name(HEALTH_OPS, h["slow_op"], "op")
    h["payload_len"] = len(payload)
    return h


def _name(table, code, prefix):
    return table[code] if 0 <= code < len(table) else f"{prefix}_{code}"


def hexstr(b: bytes) -> str:
    """Lowercase, unspaced hex -- what bridge.py's expand_arena_command() accepts."""
    return b.hex()


# --------------------------------------------------------------------------
# Transports
# --------------------------------------------------------------------------

class ReplyTimeout(Exception):
    """No matching reply within the timeout."""


class Reply:
    __slots__ = ("status", "echo", "payload", "dt_ms", "stale", "diag")

    def __init__(self, status, echo, payload, dt_ms, stale=0, diag=None):
        self.status = status
        self.echo = echo
        self.payload = payload
        self.dt_ms = dt_ms
        self.stale = stale  # frames with a foreign echo skipped while waiting
        self.diag = diag or []  # 0xFF diagnostic lines seen while waiting


class SerialLink:
    """USB-CDC link. Single-flight: `command()` blocks until the reply whose echo
    matches the opcode arrives, or raises ReplyTimeout. Framing mirrors
    tests/transport.py in the firmware repo (length-prefixed, 0xFF diag lines
    stripped). A late reply to an EARLIER command (echo mismatch) is skipped and
    counted (`Reply.stale`) rather than being mistaken for this command's reply.
    """

    def __init__(self, port: str, baud: int = 115200, settle: float = 0.3):
        if serial is None:
            raise SystemExit("pyserial is required for --port (pip install pyserial, or use the firmware pixi env)")
        self.port = port
        self.baud = baud
        self.settle = settle
        self._ser = None
        self._buf = bytearray()
        self.stale_total = 0

    def open(self):
        self._ser = serial.Serial(self.port, baudrate=self.baud, timeout=0.005)
        time.sleep(self.settle)
        self._ser.reset_input_buffer()
        self._buf.clear()

    def close(self):
        ser, self._ser = self._ser, None
        if not ser:
            return
        try:
            # Drain whatever the Teensy is still pushing so its USB TX spin-wait can
            # exit before the port closes (see SerialTransport.close in the fw tests).
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                waiting = ser.in_waiting
                if waiting:
                    ser.read(waiting)
                else:
                    time.sleep(0.05)
                    if not ser.in_waiting:
                        break
        except Exception:
            pass
        try:
            ser.close()
        except Exception:
            pass

    @property
    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def flush_input(self):
        if self._ser:
            try:
                self._ser.reset_input_buffer()
            except Exception:
                pass
        self._buf.clear()

    def command(self, cmd: int, params: bytes = b"", timeout: float = 1.0) -> Reply:
        ser = self._ser
        if ser is None:
            raise OSError("port not open")
        t0 = time.monotonic()
        ser.write(build_frame(cmd, params))
        ser.flush()
        deadline = t0 + timeout
        stale = 0
        diag: list[str] = []
        while True:
            frame = self._pop_frame(diag)
            if frame is not None:
                status, echo, payload = frame
                if echo == cmd:
                    return Reply(status, echo, payload, (time.monotonic() - t0) * 1000.0, stale, diag)
                stale += 1
                self.stale_total += 1
                continue
            now = time.monotonic()
            if now >= deadline:
                raise ReplyTimeout(f"response timeout after {int(timeout * 1000)} ms (cmd 0x{cmd:02X})")
            chunk = ser.read(max(1, ser.in_waiting))
            if chunk:
                self._buf.extend(chunk)

    def _pop_frame(self, diag: list):
        """Consume one complete response frame from the buffer -> (status, echo, payload),
        or None if incomplete. Leading 0xFF diagnostic lines are consumed into `diag`."""
        buf = self._buf
        while buf:
            if buf[0] == DIAG_SENTINEL:
                nl = buf.find(b"\n")
                if nl == -1:
                    return None
                diag.append(bytes(buf[1:nl]).decode("ascii", "replace").strip())
                del buf[: nl + 1]
                continue
            length = buf[0]
            if length == 0:  # never valid (status+echo always present); resync
                del buf[0]
                continue
            if len(buf) < 1 + length:
                return None
            frame = bytes(buf[1: 1 + length])
            del buf[: 1 + length]
            status = frame[0]
            echo = frame[1] if len(frame) > 1 else -1
            return status, echo, frame[2:]
        return None


class FakeLink:
    """In-process controller model for --dry-run. Answers instantly (2 ms) with
    plausible payloads; after `wedge_after` total commands it "degrades": every
    reply takes `wedge_latency` seconds (so the soak's 0.5 s timeouts fail but the
    5 s probes succeed slowly -- the #50 signature). SYSTEM_RESET reboots it:
    counters reset, breadcrumb/reset_cause populated, wedge cleared (wedges once).
    """

    def __init__(self, wedge_after: int = 0, wedge_latency: float = 1.2, frame_count: int = 200,
                 ok_latency: float = 0.002):
        self.wedge_after = wedge_after
        self.wedge_latency = wedge_latency
        self.ok_latency = ok_latency
        self.frame_count = frame_count
        self.stale_total = 0
        self._open = False
        self._boot(first=True)

    def _boot(self, first: bool):
        self.n_cmds = 0
        self.wedged = False
        self.wedge_armed = first  # only the first boot wedges
        self.boot_t = time.monotonic()
        self.loop_count = 0
        self.sd_reads = 0
        self.frames_sent = 0
        self.cmd70 = 0
        self.state = 0
        self.cur = 0
        self.pattern_open = False
        self.reset_cause = 0x00000001 if first else 0x00000040  # POR vs SW (SRC_SRSR-ish)
        self.prev = (0, 0, 0, 0, 0) if first else (4, 123456, 0x70, 1, 987)  # last op = command 0x70

    # -- link lifecycle --
    def open(self):
        self._open = True

    def close(self):
        self._open = False

    @property
    def is_open(self):
        return self._open

    def flush_input(self):
        pass

    # -- the model --
    def command(self, cmd: int, params: bytes = b"", timeout: float = 1.0) -> Reply:
        if not self._open:
            raise OSError("port not open")
        t0 = time.monotonic()
        self.n_cmds += 1
        self.loop_count += 37
        if self.wedge_armed and self.wedge_after and self.n_cmds >= self.wedge_after:
            self.wedged = True
        latency = self.wedge_latency if self.wedged else self.ok_latency
        if latency > timeout:
            time.sleep(timeout)
            raise ReplyTimeout(f"response timeout after {int(timeout * 1000)} ms (cmd 0x{cmd:02X})")
        time.sleep(latency)
        status, payload = self._handle(cmd, params)
        return Reply(status, cmd, payload, (time.monotonic() - t0) * 1000.0)

    def _handle(self, cmd, params):
        if cmd == GET_CONTROLLER_INFO_CMD:
            return 0, bytes([1, 0xA3, 0x04, 0xE9, 0xE5, 0x12, 0x91, 0xC0])
        if cmd == GET_PATTERN_INFO_CMD:
            (idx,) = struct.unpack("<H", params)
            if idx < 1 or idx > 5:
                return 1, b"pattern info read failed"
            return 0, struct.pack("<HBBBBBIB", self.frame_count, 2, 2, 10, 0, 0, 216_000, 128)
        if cmd == TRIAL_PARAMS_CMD:
            mode = params[0]
            self.state = {2: 3, 3: 4, 4: 5}.get(mode, 0)
            self.pattern_open = True
            self.cur = struct.unpack_from("<H", params, 5)[0]
            self.sd_reads += 1
            return 0, b""
        if cmd == SET_FRAME_POSITION_CMD:
            self.cmd70 += 1
            if not self.pattern_open:
                return 1, b"SET_FRAME_POSITION: no pattern selected (send trial-params first)"
            (idx,) = struct.unpack("<H", params)
            if idx >= self.frame_count:
                return 1, b"SET_FRAME_POSITION: index out of range"
            self.cur = idx
            self.sd_reads += 1
            self.frames_sent += 1
            return 0, b""
        if cmd == GET_FRAME_POSITION_CMD:
            return 0, struct.pack("<HH", self.cur, self.frame_count if self.pattern_open else 0)
        if cmd == GET_FRAMES_SENT_CMD:
            if self.state == 3:  # open loop: controller-timed frames accrue
                self.frames_sent += 1
            return 0, struct.pack("<I", self.frames_sent)
        if cmd == GET_HEALTH_CMD:
            flags = 0x01 | (0x02 if self.pattern_open else 0) | (0x04 if self.state else 0)
            if self.prev[1]:
                flags |= 0x08
            loop_max = 900_000 if self.wedged else 1200
            pb, pb_us, pb_arg, pslow, pslow_us = self.prev
            payload = struct.pack(
                HEALTH_FMT, 1, flags, int((time.monotonic() - self.boot_t) * 1000), self.loop_count,
                loop_max, loop_max, self.sd_reads, 1_500_000 if self.wedged else 2100, 0, 0,
                self.frames_sent, self.frames_sent * 2, self.cmd70, self.state, self.cur,
                self.reset_cause, pb, pb_us, pb_arg, pslow, pslow_us,
                1 if self.wedged else 0, 1_500_000 if self.wedged else 2100)
            return 0, payload
        if cmd == GET_FIRMWARE_INFO_CMD:
            return 1, b"No firmware image present"
        if cmd == STOP_DISPLAY_CMD:
            self.state = 0
            self.pattern_open = False
            return 0, b""
        if cmd == SYSTEM_RESET_CMD:
            self._boot(first=False)
            return 0, b"rebooting"
        return 1, b"unknown command"


# --------------------------------------------------------------------------
# Index sequences
# --------------------------------------------------------------------------

class IndexWalker:
    """Frame-index generator. `kind`: walk (FicTrac-like Gaussian random walk,
    wraps mod frame_count), random (uniform), seq (i+1), fixed, jump (walk plus a
    wide +-jump_frames every jump_every samples)."""

    def __init__(self, kind: str, frame_count: int, seed: Optional[int], sigma: float = 1.6,
                 fixed_index: int = 0, jump_frames: int = 50, jump_every: int = 100):
        self.kind = kind
        self.n = max(1, frame_count)
        self.rng = random.Random(seed)
        self.sigma = sigma
        self.fixed = min(max(0, fixed_index), self.n - 1)
        self.jump_frames = jump_frames
        self.jump_every = max(1, jump_every)
        self.pos = 0.0
        self.i = 0

    def next(self) -> int:
        self.i += 1
        k = self.kind
        if k == "fixed":
            return self.fixed
        if k == "seq":
            return self.i % self.n
        if k == "random":
            return self.rng.randrange(self.n)
        # walk / jump
        self.pos += self.rng.gauss(0.0, self.sigma)
        if k == "jump" and self.i % self.jump_every == 0:
            self.pos += self.rng.choice((-1, 1)) * self.jump_frames
        self.pos %= self.n
        return int(round(self.pos)) % self.n


# --------------------------------------------------------------------------
# NDJSON log (behavior_v2 layout, see fictrac-bridge/bridge.py)
# --------------------------------------------------------------------------

class SoakLog:
    def __init__(self, path: str, flush_every: float = 1.0):
        self.path = path
        self.t0_epoch_ms = int(time.time() * 1000)
        self._mono0 = time.monotonic()
        self._f = open(path, "w", encoding="utf-8")
        self._flush_every = flush_every
        self._last_flush = time.monotonic()
        self.rows = 0
        self.events = 0
        self._write(
            {"type": "frame_schema", "level": "behavior_v2",
             "cols": ["ms", "fc", "idx", "ft", "x", "y", "hd"],
             "arena_cols": ["t_off", "dt", "hex", "status", "rx_off"],
             "t0": self.t0_epoch_ms})

    # -- time helpers --
    def now_off_ms(self) -> int:
        return int(round((time.monotonic() - self._mono0) * 1000.0))

    def epoch_ms(self) -> int:
        return self.t0_epoch_ms + self.now_off_ms()

    # -- writers --
    def _write(self, obj):
        self._f.write(json.dumps(obj, separators=(",", ":")) + "\n")
        now = time.monotonic()
        if now - self._last_flush >= self._flush_every:
            self.flush()

    def flush(self):
        self._f.flush()
        try:
            os.fsync(self._f.fileno())
        except OSError:
            pass
        self._last_flush = time.monotonic()

    def arena(self, t_off: int, dt_ms: float, request: bytes, status, rx_off: int,
              error: Optional[str] = None):
        row = ["a", t_off, round(dt_ms, 2), hexstr(request), status, rx_off]
        if error is not None:
            row.append(error)
        self.rows += 1
        self._write(row)

    def event(self, event_name: str, **fields):
        obj = {"type": "log", "event": event_name}
        obj.update(fields)
        obj["ms"] = self.epoch_ms()
        self.events += 1
        self._write(obj)

    def close(self):
        try:
            self.flush()
            self._f.close()
        except Exception:
            pass


# --------------------------------------------------------------------------
# The soak
# --------------------------------------------------------------------------

SPIN_MARGIN_S = 0.0012  # time.sleep() overshoots by ~0.5-1 ms on macOS/Linux


def _sleep_until(t: float):
    """Hybrid wait: coarse sleep, then spin the last ~1 ms so the 0x70 schedule
    holds at 286 Hz (plain sleep() alone loses slots to kernel timer slack)."""
    remaining = t - time.monotonic()
    if remaining > SPIN_MARGIN_S:
        time.sleep(remaining - SPIN_MARGIN_S)
    while time.monotonic() < t:
        pass

class SoakEnd(Exception):
    def __init__(self, reason: str, exit_code: int):
        super().__init__(reason)
        self.reason = reason
        self.exit_code = exit_code


class Soak:
    FAULT_THRESHOLD = 3
    FAULT_WINDOW = 10

    def __init__(self, args, link, log: SoakLog):
        self.args = args
        self.link = link
        self.log = log
        self.mode = args.mode
        self.frame_count = args.frames or 0
        self.controller: dict = {}
        self.health_capable = False
        # rolling state (bounded)
        self.outcomes: deque[int] = deque(maxlen=self.FAULT_WINDOW)
        self.rtts: deque[tuple[float, float]] = deque(maxlen=max(2000, int(args.hz * 90)))
        self.diag_logged = 0
        self.stale_logged = 0
        # counters
        self.sent70 = self.ok70 = self.timeouts = self.bad_status = self.xport_errors = 0
        self.skipped_slots = 0
        self.resets = 0
        self.faults = 0
        self.total_cmds = 0
        self._win = {"t": time.monotonic(), "sent": 0, "ok": 0, "timeouts": 0, "bad": 0}
        self.start_mono = time.monotonic()
        self.next_health = self.start_mono
        self.next_progress = self.start_mono + args.progress_every

    # -- one command, logged as an ["a", ...] row -------------------------------
    def send(self, cmd: int, params: bytes = b"", timeout: float = None, track: bool = False):
        """Send one framed command, log the row, return (reply|None, error|None).
        `track=True` feeds the fault window (steady-state commands only)."""
        timeout = self.args.timeout if timeout is None else timeout
        request = build_frame(cmd, params)
        t_off = self.log.now_off_ms()
        t0 = time.monotonic()
        self.total_cmds += 1
        reply = err = None
        try:
            reply = self.link.command(cmd, params, timeout)
        except ReplyTimeout as e:
            err = str(e)
            if track:
                self.timeouts += 1
                self._win["timeouts"] += 1
        except Exception as e:  # SerialException, OSError, ...
            err = f"transport error: {e.__class__.__name__}: {e}"
            if track:
                self.xport_errors += 1
        dt_ms = (time.monotonic() - t0) * 1000.0
        rx_off = t_off + int(round(dt_ms))
        if reply is not None:
            if reply.status != 0:
                err = f"status {reply.status}: {reply.payload.decode('ascii', 'replace')}"
                if track:
                    self.bad_status += 1
                    self._win["bad"] += 1
            self.log.arena(t_off, dt_ms, request, reply.status, rx_off, err)
            if reply.stale:
                self._note_stale(cmd, reply.stale)
            if reply.diag:
                self._note_diag(reply.diag)
        else:
            self.log.arena(t_off, dt_ms, request, None, rx_off, err)
        if track:
            self.rtts.append((time.monotonic(), dt_ms))
            self.outcomes.append(1 if err else 0)
        return reply, err

    def _note_stale(self, cmd, n):
        if self.stale_logged < 500:
            self.stale_logged += 1
            self.log.event("stale_reply", cmd=f"0x{cmd:02X}", skipped=n, total=self.link.stale_total)

    def _note_diag(self, lines):
        for line in lines:
            if self.diag_logged >= 2000:
                return
            self.diag_logged += 1
            self.log.event("diag", text=line)

    # -- startup -----------------------------------------------------------------
    def startup(self):
        a = self.args
        info = None
        for _ in range(3):
            reply, err = self.send(GET_CONTROLLER_INFO_CMD, timeout=2.0)
            if reply is not None and reply.status == 0:
                info = decode_controller_info(reply.payload)
                break
        if info is None:
            self.log.event("startup_failed", stage="controller_info")
            raise SoakEnd("startup-failed", 1)
        self.controller = info
        self.health_capable = info["health_capable"]
        self.log.event("run_metadata", driver="soak_mode3.py", args=vars(a),
                       controller=info, python=sys.version.split()[0],
                       fault_rule={"threshold": self.FAULT_THRESHOLD, "window": self.FAULT_WINDOW},
                       log=self.log.path)
        if self.mode == 3 and not self.frame_count:
            reply, err = self.send(GET_PATTERN_INFO_CMD, struct.pack("<H", a.pattern), timeout=5.0)
            pinfo = decode_pattern_info(reply.payload) if reply is not None and reply.status == 0 else {}
            if not pinfo.get("frame_count"):
                self.log.event("startup_failed", stage="pattern_info", error=err)
                raise SoakEnd("startup-failed", 1)
            self.frame_count = pinfo["frame_count"]
            self.log.event("pattern_info", pattern=a.pattern, **pinfo)
        self.walker = IndexWalker(a.index_walk, self.frame_count or 1, a.seed, a.walk_sigma,
                                  a.fixed_index, a.jump_frames, a.jump_every)
        if not self.open_pattern():
            raise SoakEnd("startup-failed", 1)

    def open_pattern(self) -> bool:
        a = self.args
        if self.mode == 3:
            frame = encode_trial_params(3, a.pattern, 0, 0, 0, 0, 0)
        else:
            frame = encode_trial_params(2, a.pattern, a.fps, 0, 0, 0, 0)
        reply, err = self.send(TRIAL_PARAMS_CMD, frame[2:], timeout=10.0)
        ok = reply is not None and reply.status == 0
        self.log.event("trial_params", mode=self.mode, pattern=a.pattern,
                       fps=(a.fps if self.mode == 2 else 0), frame_count=self.frame_count,
                       hex=hexstr(frame), ok=ok, error=err)
        return ok

    # -- steady state --------------------------------------------------------------
    def run(self) -> int:
        a = self.args
        try:
            self.startup()
            if self.mode == 3:
                self.loop_mode3()
            else:
                self.loop_mode2()
        except SoakEnd as e:
            self.finish(e.reason)
            return e.exit_code
        except KeyboardInterrupt:
            self.log.event("interrupt")
            self.best_effort_stop()
            self.finish("keyboard-interrupt")
            return 0
        return 0

    def deadline_reached(self) -> Optional[str]:
        a = self.args
        if a.hours > 0 and time.monotonic() - self.start_mono >= a.hours * 3600.0:
            return "hours-elapsed"
        if a.max_commands > 0 and self.sent70 >= a.max_commands:
            return "max-commands"
        return None

    def loop_mode3(self):
        a = self.args
        period = 1.0 / a.hz
        next_t = time.monotonic()
        while True:
            reason = self.deadline_reached()
            if reason:
                self.best_effort_stop()
                raise SoakEnd(reason, 0)
            now = time.monotonic()
            # Health tick between 0x70s, never concurrently. Deferred while the
            # most recent 0x70 failed: four more 0.5 s timeouts per tick would
            # stretch fault detection ~5x past the browser's; the post-mortem
            # probe window reads the same counters with long timeouts instead.
            if now >= self.next_health and not (self.outcomes and self.outcomes[-1]):
                self.health_tick()
                now = time.monotonic()
                next_t = self._reschedule(next_t, now, period)
            if now >= self.next_progress:
                self.progress()
            if now < next_t:
                _sleep_until(min(next_t, now + 0.05))
                continue
            idx = self.walker.next()
            self.sent70 += 1
            self._win["sent"] += 1
            reply, err = self.send(SET_FRAME_POSITION_CMD, struct.pack("<H", idx), track=True)
            if not err:
                self.ok70 += 1
                self._win["ok"] += 1
            self.check_fault(idx, err)
            next_t = self._reschedule(next_t + period, time.monotonic(), period)

    def loop_mode2(self):
        while True:
            reason = self.deadline_reached()
            if reason:
                self.best_effort_stop()
                raise SoakEnd(reason, 0)
            now = time.monotonic()
            if now >= self.next_health:
                self.health_tick(track=True)
                self.check_fault(None, self._last_health_err)
            if now >= self.next_progress:
                self.progress()
            time.sleep(min(0.05, max(0.0, self.next_health - time.monotonic())))

    def _reschedule(self, next_t: float, now: float, period: float) -> float:
        """Monotonic schedule: if we are late by more than a period, skip the
        missed slots (count them) instead of bursting to catch up."""
        if now - next_t > period:
            missed = int((now - next_t) // period)
            self.skipped_slots += missed
            next_t = now
        return next_t

    # -- health --------------------------------------------------------------------
    _last_health_err: Optional[str] = None

    def health_tick(self, track: bool = False):
        self.next_health = time.monotonic() + self.args.health_every
        dts: dict[str, Optional[float]] = {}
        errs = []
        frames_sent = cur = fcount = None
        health = None

        reply, err = self.send(GET_FRAMES_SENT_CMD, track=track)
        dts["33"] = round(reply.dt_ms, 2) if reply else None
        if reply and reply.status == 0:
            frames_sent = decode_frames_sent(reply.payload)
        elif err:
            errs.append(err)

        reply, err = self.send(GET_CONTROLLER_INFO_CMD, track=track)
        dts["c2"] = round(reply.dt_ms, 2) if reply else None
        if reply and reply.status == 0:
            self.health_capable = decode_controller_info(reply.payload)["health_capable"]
        elif err:
            errs.append(err)

        reply, err = self.send(GET_FRAME_POSITION_CMD, track=track)
        dts["72"] = round(reply.dt_ms, 2) if reply else None
        if reply and reply.status == 0:
            fp = decode_frame_position(reply.payload)
            cur, fcount = fp.get("cur_frame"), fp.get("frame_count")
        elif err:
            errs.append(err)

        if self.health_capable:
            reply, err = self.send(GET_HEALTH_CMD, track=track)
            dts["ca"] = round(reply.dt_ms, 2) if reply else None
            if reply and reply.status == 0:
                health = decode_health(reply.payload)
            elif err:
                errs.append(err)

        self._last_health_err = errs[-1] if errs else None
        self.log.event("health", frames_sent=frames_sent, cur_frame=cur, frame_count=fcount,
                       health=health, dt_ms=dts, errors=errs or None, sent70=self.sent70)

    # -- progress ------------------------------------------------------------------
    def progress(self, final: bool = False):
        now = time.monotonic()
        w = self._win
        span = max(1e-6, now - w["t"])
        recent = [dt for (t, dt) in self.rtts if t >= w["t"]]
        med = p99 = None
        if recent:
            recent.sort()
            med = round(statistics.median(recent), 2)
            p99 = round(recent[min(len(recent) - 1, int(math.ceil(len(recent) * 0.99)) - 1)], 2)
        rec = {
            "elapsed_s": round(now - self.start_mono, 1), "sent": self.sent70, "ok": self.ok70,
            "timeouts": self.timeouts, "bad_status": self.bad_status,
            "transport_errors": self.xport_errors, "skipped_slots": self.skipped_slots,
            "stale_replies": self.link.stale_total, "faults": self.faults, "resets": self.resets,
            "window_s": round(span, 1), "window_sent": w["sent"], "window_ok": w["ok"],
            "applied_hz": round(w["ok"] / span, 1), "rtt_med_ms": med, "rtt_p99_ms": p99,
            "log_rows": self.log.rows,
        }
        self.log.event("soak_progress", **rec)
        self.log.flush()
        print(f"[soak] t={rec['elapsed_s']:.0f}s sent={rec['sent']} ok={rec['ok']} "
              f"timeouts={rec['timeouts']} bad={rec['bad_status']} skipped={rec['skipped_slots']} "
              f"applied={rec['applied_hz']}Hz rtt med/p99={med}/{p99}ms faults={self.faults} "
              f"resets={self.resets}", file=sys.stderr, flush=True)
        self._win = {"t": now, "sent": 0, "ok": 0, "timeouts": 0, "bad": 0}
        self.next_progress = now + self.args.progress_every

    # -- fault lifecycle ---------------------------------------------------------------
    def check_fault(self, last_index, last_err):
        if not last_err:
            return
        n = sum(self.outcomes)
        if n < self.FAULT_THRESHOLD:
            return
        self.faults += 1
        self.log.event("fault", kind="controller_unresponsive", failures=n, window=len(self.outcomes),
                       threshold=self.FAULT_THRESHOLD, last_index=last_index, last_error=last_err,
                       sent70=self.sent70, ok70=self.ok70, elapsed_s=round(time.monotonic() - self.start_mono, 3),
                       fault_number=self.faults)
        self.progress()
        self.post_mortem()
        a = self.args
        if a.on_fault == "halt" or self.resets >= a.max_resets:
            reason = "fault-halt" if a.on_fault == "halt" else "max-resets"
            self.log.event("fault_policy", policy=a.on_fault, action="halt", resets=self.resets)
            raise SoakEnd(reason, 2)
        self.log.event("fault_policy", policy=a.on_fault, action="reset", resets=self.resets)
        self.reset_and_resume()

    def post_mortem(self):
        a = self.args
        # 1. quiet period: no sends, drop anything the controller is still emitting
        self.log.event("quiet_start", seconds=a.quiet_seconds)
        time.sleep(a.quiet_seconds)
        self.link.flush_input()
        # 2. confirmation probe
        reply, err = self.send(GET_CONTROLLER_INFO_CMD, timeout=2.0)
        self.log.event("confirm_probe", name="controller_info", answered=reply is not None,
                       dt_ms=round(reply.dt_ms, 2) if reply else None, status=reply.status if reply else None,
                       error=err)
        # 3. probe window with long timeouts, one round every probe_interval seconds
        window_end = time.monotonic() + a.probe_seconds
        rounds = 0
        self.log.event("probe_window_start", seconds=a.probe_seconds, interval=a.probe_interval,
                       timeout=a.probe_timeout)
        while time.monotonic() < window_end:
            round_start = time.monotonic()
            rounds += 1
            self.probe_round(rounds)
            self.log.flush()
            wait = a.probe_interval - (time.monotonic() - round_start)
            if wait > 0:
                time.sleep(min(wait, max(0.0, window_end - time.monotonic())))
        self.log.event("probe_window_end", rounds=rounds)

    def probe_round(self, round_no: int):
        a = self.args
        probes = [
            ("controller_info", GET_CONTROLLER_INFO_CMD, b"", decode_controller_info),
            ("frames_sent", GET_FRAMES_SENT_CMD, b"", lambda p: {"frames_sent": decode_frames_sent(p)}),
            ("frame_position", GET_FRAME_POSITION_CMD, b"", decode_frame_position),
        ]
        if self.health_capable:
            probes.append(("health", GET_HEALTH_CMD, b"", decode_health))
        probes.append(("pattern_info_1", GET_PATTERN_INFO_CMD, struct.pack("<H", 1), decode_pattern_info))
        probes.append(("firmware_info", GET_FIRMWARE_INFO_CMD, b"",
                       lambda p: {"footer_hex": hexstr(p)} if len(p) >= 32 else {"text": p.decode("ascii", "replace")}))
        for name, cmd, params, decoder in probes:
            reply, err = self.send(cmd, params, timeout=a.probe_timeout)
            decoded = None
            if reply is not None:
                try:
                    decoded = decoder(reply.payload) if reply.status == 0 else {"text": reply.payload.decode("ascii", "replace")}
                except Exception as e:  # never let a decoder kill the probe window
                    decoded = {"decode_error": str(e)}
            self.log.event("probe", round=round_no, name=name, cmd=f"0x{cmd:02X}",
                           dt_ms=round(reply.dt_ms, 2) if reply else None,
                           status=reply.status if reply else None,
                           payload_hex=hexstr(reply.payload) if reply else None,
                           decoded=decoded, error=err)

    def reset_and_resume(self):
        a = self.args
        reply, err = self.send(SYSTEM_RESET_CMD, timeout=3.0)
        self.log.event("system_reset", answered=reply is not None,
                       status=reply.status if reply else None,
                       text=reply.payload.decode("ascii", "replace") if reply else None, error=err)
        self.resets += 1
        self.link.close()
        self.log.event("port_closed", wait_s=a.reset_wait)
        self.log.flush()
        time.sleep(a.reset_wait)
        deadline = time.monotonic() + a.reconnect_seconds
        attempts = 0
        last_exc = None
        while True:
            attempts += 1
            try:
                self.link.open()
                break
            except Exception as e:
                last_exc = f"{e.__class__.__name__}: {e}"
                if time.monotonic() >= deadline:
                    self.log.event("reconnect_failed", attempts=attempts, error=last_exc)
                    raise SoakEnd("reconnect-failed", 3)
                time.sleep(0.5)
        self.log.event("port_reopened", attempts=attempts, last_error=last_exc)
        # post-reset probe: 0xC2 + 0xCA (the breadcrumb / reset_cause are the payload)
        reply, err = self.send(GET_CONTROLLER_INFO_CMD, timeout=5.0)
        info = decode_controller_info(reply.payload) if reply and reply.status == 0 else None
        if info:
            self.health_capable = info["health_capable"]
        health = None
        herr = None
        if self.health_capable:
            hreply, herr = self.send(GET_HEALTH_CMD, timeout=5.0)
            health = decode_health(hreply.payload) if hreply and hreply.status == 0 else None
        # After a host-commanded 0x01 the plain prev_breadcrumb always reads
        # command/0x01; the previous boot's SLOWEST op is the real payload of the
        # reset experiment, so surface it at the top level too.
        summary = None
        if health:
            summary = {
                "reset_cause": health.get("reset_cause"),
                "breadcrumb_valid": health.get("breadcrumb_valid"),
                "prev_last_op": health.get("prev_breadcrumb_op"),
                "prev_last_op_arg": health.get("prev_breadcrumb_arg_hex"),
                "prev_last_op_us": health.get("prev_breadcrumb_us"),
                "prev_slow_op": health.get("prev_slow_op_name"),
                "prev_slow_us": health.get("prev_slow_us"),
            }
        self.log.event("post_reset_probe", reset_number=self.resets, prev_boot=summary,
                       controller=info, controller_error=err, health=health, health_error=herr)
        print(f"[soak] post-reset breadcrumb: {summary}", file=sys.stderr, flush=True)
        self.log.flush()
        # resume the soak: re-open the pattern, clear the fault window
        for attempt in range(1, 4):
            if self.open_pattern():
                break
            time.sleep(1.0)
        else:
            self.log.event("reopen_failed")
            raise SoakEnd("reopen-failed", 2)
        self.outcomes.clear()
        self.next_health = time.monotonic()
        self.log.event("soak_resumed", reset_number=self.resets)

    # -- end -----------------------------------------------------------------------
    def best_effort_stop(self):
        try:
            self.send(STOP_DISPLAY_CMD, timeout=1.0)
        except Exception:
            pass

    def finish(self, reason: str):
        self.progress(final=True)
        self.log.event("soak_end", reason=reason, sent70=self.sent70, ok70=self.ok70,
                       timeouts=self.timeouts, bad_status=self.bad_status,
                       transport_errors=self.xport_errors, faults=self.faults, resets=self.resets,
                       total_commands=self.total_cmds,
                       elapsed_s=round(time.monotonic() - self.start_mono, 3))
        self.log.close()
        print(f"[soak] end: {reason} -> {self.log.path}", file=sys.stderr, flush=True)


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Mode-3 soak / repro driver for the G6 controller wedge (fw #50).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("Usage examples:", 1)[1])
    g = p.add_argument_group("link")
    g.add_argument("--port", help="serial device, e.g. /dev/cu.usbmodem123456 or COM7 (required unless --dry-run)")
    g.add_argument("--baud", type=int, default=115200, help="CDC baud (ignored by the Teensy; default 115200)")
    g.add_argument("--timeout", type=float, default=0.5, help="reply timeout for soak commands, s (browser: 0.5)")

    g = p.add_argument_group("display")
    g.add_argument("--mode", type=int, choices=(2, 3), default=3, help="3 = host-stepped 0x70 soak (default); 2 = controller-timed control arm")
    g.add_argument("--pattern", type=int, default=1, help="1-based SD pattern index (default 1)")
    g.add_argument("--frames", type=int, default=0, help="override frame count (default: read via GET_PATTERN_INFO 0x88)")
    g.add_argument("--fps", type=int, default=10, help="Mode-2 frame_rate, Hz (signed; default 10)")

    g = p.add_argument_group("mode-3 stepping")
    g.add_argument("--hz", type=float, default=100.0, help="0x70 rate (default 100; 286 for the fast arm)")
    g.add_argument("--index-walk", choices=("walk", "random", "seq", "fixed", "jump"), default="walk")
    g.add_argument("--walk-sigma", type=float, default=1.6, help="walk/jump: per-sample Gaussian step, frames (default 1.6)")
    g.add_argument("--fixed-index", type=int, default=0, help="fixed: the constant index")
    g.add_argument("--jump-frames", type=int, default=50, help="jump: width of the periodic jump, frames")
    g.add_argument("--jump-every", type=int, default=100, help="jump: one jump every N samples")
    g.add_argument("--seed", type=int, default=None, help="RNG seed for the index sequence")

    g = p.add_argument_group("duration / cadence")
    g.add_argument("--hours", type=float, default=10.0, help="run length, hours (0 = forever; default 10)")
    g.add_argument("--max-commands", type=int, default=0, help="stop after N 0x70 commands (0 = no limit; testing aid)")
    g.add_argument("--health-every", type=float, default=1.0, help="health tick period, s (default 1.0)")
    g.add_argument("--progress-every", type=float, default=60.0, help="soak_progress period, s (default 60)")

    g = p.add_argument_group("fault lifecycle")
    g.add_argument("--on-fault", choices=("halt", "reset-continue"), default="halt")
    g.add_argument("--quiet-seconds", type=float, default=1.0, help="no-send quiet period after a fault (default 1.0)")
    g.add_argument("--probe-seconds", type=float, default=60.0, help="length of the slow-timeout probe window (default 60)")
    g.add_argument("--probe-interval", type=float, default=2.0, help="start a probe round every N s (default 2)")
    g.add_argument("--probe-timeout", type=float, default=5.0, help="reply timeout for probe commands (default 5)")
    g.add_argument("--max-resets", type=int, default=3, help="reset-continue: halt after this many resets (default 3)")
    g.add_argument("--reset-wait", type=float, default=3.0, help="reset-continue: seconds to wait before re-opening the port")
    g.add_argument("--reconnect-seconds", type=float, default=20.0, help="reset-continue: keep retrying open() this long")

    g = p.add_argument_group("logging")
    g.add_argument("--log-dir", default="./soak-logs")

    g = p.add_argument_group("dry run (no hardware)")
    g.add_argument("--dry-run", action="store_true", help="use an in-process fake controller")
    g.add_argument("--dry-run-wedge-after", type=int, default=0, help="fake wedges after N commands (0 = never)")
    g.add_argument("--dry-run-wedge-latency", type=float, default=1.2, help="fake's reply latency once wedged, s")
    g.add_argument("--dry-run-frames", type=int, default=200, help="fake pattern frame count")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if not args.dry_run and not args.port:
        print("error: --port is required (or --dry-run)", file=sys.stderr)
        return 1
    if args.hz <= 0:
        print("error: --hz must be > 0", file=sys.stderr)
        return 1
    if args.mode == 3 and args.fps != 10:
        print("note: --fps is only used with --mode 2", file=sys.stderr)

    os.makedirs(args.log_dir, exist_ok=True)
    stamp = _dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    rate = int(round(args.hz)) if args.mode == 3 else args.fps
    path = os.path.join(args.log_dir, f"soak-{stamp}-mode{args.mode}-{rate}hz.jsonl")

    if args.dry_run:
        link = FakeLink(args.dry_run_wedge_after, args.dry_run_wedge_latency, args.dry_run_frames)
    else:
        link = SerialLink(args.port, args.baud)

    log = SoakLog(path)
    print(f"[soak] log: {path}", file=sys.stderr, flush=True)
    try:
        link.open()
    except Exception as e:
        log.event("startup_failed", stage="open", error=f"{e.__class__.__name__}: {e}")
        log.event("soak_end", reason="startup-failed")
        log.close()
        print(f"error: cannot open {args.port}: {e}", file=sys.stderr)
        return 1

    soak = Soak(args, link, log)
    try:
        return soak.run()
    finally:
        try:
            link.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
