"""Qwiic / STEMMA QT sensor helpers over the controller's I2C bridge
(GET_I2C_SCAN 0xA6 / I2C_TRANSFER 0xA7 -> Wire1 = jack J2 on arena_12-18).

Shared by tests/test_qwiic_i2c.py and scripts/qwiic_probe.py. The register
maps here are the minimum needed to prove the I2C path end to end — identify
the part, round-trip a config register, take one raw light reading — not full
drivers. The LAB-211 calibration work should build on vendor drivers.

Parts (Linear LAB-211):
  0x10        VEML7700  lux baseline           (Adafruit 4162)
  0x29        TSL2591   full/IR intensity      (Adafruit 1980)
  0x39        AS7343    14-ch spectral/color   (Adafruit 6477)
  0x70..0x77  PCA9548   8-channel I2C mux      (Adafruit 5626, default 0x70)
"""

from contextlib import contextmanager
from dataclasses import dataclass
import struct
import time

from .commands import GET_I2C_SCAN_CMD, I2C_TRANSFER_CMD

I2C_STATUS_TEXT = {
    1: "bad framing",
    2: "address NACK",
    3: "data NACK",
    4: "bus error/timeout",
    5: "short read",
}
_NO_QWIIC_TEXT = b"No Qwiic jack"


class NoQwiicJack(RuntimeError):
    """The flashed firmware variant has no Qwiic jack (arena_10-10 build)."""


class I2CError(RuntimeError):
    def __init__(self, status, addr, msg):
        super().__init__(
            f"I2C 0x{addr:02X}: {I2C_STATUS_TEXT.get(status, f'status {status}')} ({msg})")
        self.status = status
        self.addr = addr


def _check_no_jack(status, payload):
    if status != 0 and _NO_QWIIC_TEXT in bytes(payload):
        raise NoQwiicJack(bytes(payload).decode("ascii", errors="replace"))


def i2c_scan(transport) -> list[int]:
    st, echo, payload, _ = transport.command(GET_I2C_SCAN_CMD)
    _check_no_jack(st, payload)
    if st != 0:
        raise RuntimeError(f"GET_I2C_SCAN failed: status={st} {bytes(payload)!r}")
    payload = bytes(payload)
    count = payload[0]
    addrs = list(payload[1:1 + count])
    if len(addrs) != count:
        raise RuntimeError(f"GET_I2C_SCAN: count {count} but {len(addrs)} addresses")
    return addrs


def i2c_xfer(transport, addr, wbytes=b"", rlen=0, timeout=1.0) -> bytes:
    """Write `wbytes` then read `rlen` bytes (repeated start). Raises I2CError."""
    wbytes = bytes(wbytes)
    params = bytes([addr, len(wbytes)]) + wbytes + bytes([rlen])
    st, echo, payload, _ = transport.command(I2C_TRANSFER_CMD, params, timeout=timeout)
    _check_no_jack(st, payload)
    if st != 0:
        raise I2CError(st, addr, bytes(payload).decode("ascii", errors="replace"))
    return bytes(payload)


def i2c_ack(transport, addr) -> bool:
    try:
        i2c_xfer(transport, addr)
        return True
    except I2CError as e:
        if e.status == 2:
            return False
        raise


def read_reg(transport, addr, reg, n=1) -> bytes:
    return i2c_xfer(transport, addr, bytes([reg]), n)


def write_reg(transport, addr, reg, data) -> None:
    i2c_xfer(transport, addr, bytes([reg]) + bytes(data), 0)


# ── Address book ─────────────────────────────────────────────────────────────

MCP4725_ADDR = 0x60  # the arena's AO DAC — lives on Wire (D18/D19), never on Wire1

_KNOWN = {
    0x10: "VEML7700 lux",
    0x29: "TSL2591 light",
    0x39: "AS7343 spectral",
    MCP4725_ADDR: "MCP4725 DAC (arena AO — must NOT be visible on the Qwiic bus)",
}


def is_mux(addr) -> bool:
    return 0x70 <= addr <= 0x77


def describe(addr) -> str:
    if is_mux(addr):
        return f"PCA9548 8-ch mux (A2..A0 = {addr - 0x70:03b})"
    return _KNOWN.get(addr, "unknown")


# ── PCA9548 mux ──────────────────────────────────────────────────────────────

class PCA9548:
    """One control register: bit k enables downstream channel k."""

    def __init__(self, transport, addr=0x70):
        self.t = transport
        self.addr = addr

    def read_mask(self) -> int:
        return i2c_xfer(self.t, self.addr, b"", 1)[0]

    def select(self, mask: int) -> None:
        i2c_xfer(self.t, self.addr, bytes([mask & 0xFF]), 0)

    @contextmanager
    def channel(self, ch: int):
        previous = self.read_mask()
        self.select(1 << ch)
        try:
            yield
        finally:
            self.select(previous)

    def channel_map(self) -> dict[int, list[int]]:
        """{channel: [downstream addresses]} — excludes the mux itself and
        anything that is on the root bus. Restores the mask it found."""
        previous = self.read_mask()
        try:
            self.select(0)
            root = set(i2c_scan(self.t))
            result = {}
            for ch in range(8):
                self.select(1 << ch)
                result[ch] = sorted(set(i2c_scan(self.t)) - root)
            return result
        finally:
            self.select(previous)


@dataclass(frozen=True)
class Located:
    addr: int
    mux: int | None = None      # mux address, or None when on the root bus
    channel: int | None = None

    def where(self) -> str:
        if self.mux is None:
            return "root"
        return f"mux 0x{self.mux:02X} ch{self.channel}"


def locate_devices(transport) -> list[Located]:
    """Everything reachable: root-bus devices plus each mux channel's devices."""
    root = i2c_scan(transport)
    found = [Located(a) for a in root]
    for mux_addr in (a for a in root if is_mux(a)):
        for ch, addrs in PCA9548(transport, mux_addr).channel_map().items():
            found.extend(Located(a, mux_addr, ch) for a in addrs)
    return found


@contextmanager
def reach(transport, dev: Located):
    """Select the mux channel for `dev` (no-op for root devices)."""
    if dev.mux is None:
        yield
        return
    with PCA9548(transport, dev.mux).channel(dev.channel):
        yield


# ── VEML7700 (0x10) ──────────────────────────────────────────────────────────

class VEML7700:
    ADDR = 0x10
    REG_CONF = 0x00    # ALS_CONF_0: ALS_GAIN[12:11], ALS_IT[9:6], ALS_SD[0]
    REG_ALS = 0x04
    REG_WHITE = 0x05
    REG_ID = 0x07      # low byte = device id 0x81 (datasheet rev >= 1.7; absent on early silicon)
    CONF_GAIN1_IT100 = 0x0000        # gain x1, 100 ms, power on
    CONF_GAIN1_IT200 = 0x0040        # gain x1, 200 ms (ALS_IT = 0001)
    LUX_PER_COUNT_GAIN1_IT100 = 0.0576
    DEVICE_ID_EXPECTED = 0x81

    def __init__(self, transport, addr=ADDR):
        self.t = transport
        self.addr = addr

    def read16(self, reg) -> int:
        return struct.unpack("<H", read_reg(self.t, self.addr, reg, 2))[0]

    def write16(self, reg, value) -> None:
        write_reg(self.t, self.addr, reg, struct.pack("<H", value))

    def identify(self) -> dict:
        v = self.read16(self.REG_ID)
        return {"id_raw": v, "device_id": v & 0xFF,
                "device_id_ok": (v & 0xFF) == self.DEVICE_ID_EXPECTED}

    def configure(self, conf=CONF_GAIN1_IT100) -> None:
        self.write16(self.REG_CONF, conf)
        time.sleep(0.25)  # > 2 integration windows at 100 ms

    def read(self) -> dict:
        als = self.read16(self.REG_ALS)
        white = self.read16(self.REG_WHITE)
        return {"als": als, "white": white,
                "lux_approx": als * self.LUX_PER_COUNT_GAIN1_IT100}


# ── TSL2591 (0x29) ───────────────────────────────────────────────────────────

class TSL2591:
    ADDR = 0x29
    CMD = 0xA0         # CMD bit + "normal" transaction, OR'd with the register
    REG_ENABLE = 0x00
    REG_CONTROL = 0x01
    REG_ID = 0x12
    REG_STATUS = 0x13
    REG_C0DATAL = 0x14  # C0 (full) lo/hi, then C1 (IR) lo/hi
    ID_EXPECTED = 0x50
    ENABLE_PON = 0x01
    ENABLE_AEN = 0x02
    STATUS_AVALID = 0x01
    GAIN = {"low": (0x00, 1.0), "med": (0x10, 25.0), "high": (0x20, 428.0), "max": (0x30, 9876.0)}
    ATIME = {100: 0x00, 200: 0x01, 300: 0x02, 400: 0x03, 500: 0x04, 600: 0x05}

    def __init__(self, transport, addr=ADDR):
        self.t = transport
        self.addr = addr
        self.gain = "med"
        self.atime_ms = 100

    def read_reg(self, reg, n=1) -> bytes:
        return read_reg(self.t, self.addr, self.CMD | reg, n)

    def write_reg(self, reg, value) -> None:
        write_reg(self.t, self.addr, self.CMD | reg, [value])

    def identify(self) -> dict:
        v = self.read_reg(self.REG_ID)[0]
        return {"id": v, "id_ok": v == self.ID_EXPECTED}

    def configure(self, gain="med", atime_ms=100) -> None:
        self.gain, self.atime_ms = gain, atime_ms
        self.write_reg(self.REG_CONTROL, self.GAIN[gain][0] | self.ATIME[atime_ms])
        self.write_reg(self.REG_ENABLE, self.ENABLE_PON | self.ENABLE_AEN)
        time.sleep(atime_ms / 1000 * 1.2 + 0.05)

    def read(self) -> dict:
        status = self.read_reg(self.REG_STATUS)[0]
        full, ir = struct.unpack("<HH", self.read_reg(self.REG_C0DATAL, 4))
        out = {"full": full, "ir": ir, "visible": full - ir,
               "valid": bool(status & self.STATUS_AVALID), "lux_approx": None}
        if full > 0 and full < 0xFFFF:
            cpl = (self.atime_ms * self.GAIN[self.gain][1]) / 408.0
            out["lux_approx"] = ((full - ir) * (1.0 - ir / full)) / cpl
        return out

    def power_off(self) -> None:
        self.write_reg(self.REG_ENABLE, 0x00)


# ── AS7343 (0x39) ────────────────────────────────────────────────────────────

class AS7343:
    """14-channel spectral sensor. Register map per the AS7343 datasheet as
    used by Adafruit_AS7343: registers below 0x80 need CFG0.REG_BANK = 1,
    everything else bank 0. Raw counts only — no vendor calibration applied."""
    ADDR = 0x39
    REG_AUXID = 0x58
    REG_REVID = 0x59
    REG_ID = 0x5A
    REG_ENABLE = 0x80
    REG_ATIME = 0x81
    REG_STATUS2 = 0x90
    REG_STATUS = 0x93
    REG_ASTATUS = 0x94      # reading it latches DATA_0..17 that follow it
    REG_DATA_0_L = 0x95
    REG_CFG0 = 0xBF
    REG_CFG1 = 0xC6         # AGAIN[4:0]
    REG_ASTEP_L = 0xD4
    REG_CFG20 = 0xD6        # auto_SMUX[6:5]: 0 = 6 ch, 2 = 12 ch, 3 = 18 ch
    CFG0_REG_BANK = 0x10
    ENABLE_PON = 0x01
    ENABLE_SP_EN = 0x02
    STATUS2_AVALID = 0x40
    STATUS2_ASAT = 0x18     # ASAT_DIGITAL | ASAT_ANALOG
    CFG20_AUTO_SMUX_18 = 3 << 5
    ID_EXPECTED = 0x81
    GAIN_CODE = {0.5: 0, 1: 1, 2: 2, 4: 3, 8: 4, 16: 5, 32: 6, 64: 7,
                 128: 8, 256: 9, 512: 10, 1024: 11, 2048: 12}
    STEP_US = 2.78
    # 18-channel auto-SMUX order (3 cycles x 6 ADCs). Slots 4/5 of every cycle
    # are the clear channel (VIS) and the flicker-detect channel (FD); FD is
    # not a light measurement in this mode and sits at full scale, so it is
    # kept out of `sat`. (Adafruit's driver calls these VIS_TL / VIS_BR.)
    CHANNELS = ("FZ_450", "FY_555", "FXL_600", "NIR_855", "VIS0", "FD0",
                "F2_425", "F3_475", "F4_515", "F6_640", "VIS1", "FD1",
                "F1_405", "F7_690", "F8_745", "F5_550", "VIS2", "FD2")
    SPECTRAL = ("F1_405", "F2_425", "FZ_450", "F3_475", "F4_515", "F5_550", "FY_555",
                "FXL_600", "F6_640", "F7_690", "F8_745", "NIR_855")

    def __init__(self, transport, addr=ADDR):
        self.t = transport
        self.addr = addr
        self.full_scale = 65535
        self.tint_ms = 50.0

    def identify(self) -> dict:
        write_reg(self.t, self.addr, self.REG_CFG0, [self.CFG0_REG_BANK])
        try:
            auxid, revid, ident = read_reg(self.t, self.addr, self.REG_AUXID, 3)
        finally:
            write_reg(self.t, self.addr, self.REG_CFG0, [0x00])
        return {"id": ident, "revid": revid, "auxid": auxid,
                "id_ok": ident == self.ID_EXPECTED}

    def enable_roundtrip(self) -> bool:
        """PON on, read back, PON off — proves a register write lands."""
        write_reg(self.t, self.addr, self.REG_ENABLE, [self.ENABLE_PON])
        try:
            return bool(read_reg(self.t, self.addr, self.REG_ENABLE)[0] & self.ENABLE_PON)
        finally:
            write_reg(self.t, self.addr, self.REG_ENABLE, [0x00])

    def configure(self, gain=256, atime=29, astep=599) -> None:
        """Adafruit defaults: 30 x 600 x 2.78 us = 50 ms per cycle, x3 cycles."""
        write_reg(self.t, self.addr, self.REG_CFG0, [0x00])
        write_reg(self.t, self.addr, self.REG_ENABLE, [self.ENABLE_PON])
        write_reg(self.t, self.addr, self.REG_ATIME, [atime])
        write_reg(self.t, self.addr, self.REG_ASTEP_L, struct.pack("<H", astep))
        write_reg(self.t, self.addr, self.REG_CFG1, [self.GAIN_CODE[gain]])
        write_reg(self.t, self.addr, self.REG_CFG20, [self.CFG20_AUTO_SMUX_18])
        self.full_scale = min(65535, (atime + 1) * (astep + 1))
        self.tint_ms = (atime + 1) * (astep + 1) * self.STEP_US / 1000.0

    @property
    def frame_ms(self) -> float:
        """Time for one 18-channel measurement (three SMUX cycles)."""
        return 3 * self.tint_ms

    def start(self) -> None:
        """Kick off one measurement; the sensor integrates on its own until
        collect(). Lets a host read other sensors meanwhile."""
        write_reg(self.t, self.addr, self.REG_ENABLE, [self.ENABLE_PON])
        status = read_reg(self.t, self.addr, self.REG_STATUS)[0]
        write_reg(self.t, self.addr, self.REG_STATUS, [status])   # clear sticky flags
        write_reg(self.t, self.addr, self.REG_ENABLE, [self.ENABLE_PON | self.ENABLE_SP_EN])
        self._started = time.monotonic()

    def collect(self, timeout=2.0) -> dict:
        """Wait for AVALID (sleeping out the remaining frame time first), then
        latch and read all 18 channels and stop the measurement."""
        remaining = self._started + self.frame_ms / 1000.0 - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)
        deadline = time.monotonic() + timeout
        try:
            while True:
                status2 = read_reg(self.t, self.addr, self.REG_STATUS2)[0]
                if status2 & self.STATUS2_AVALID:
                    break
                if time.monotonic() > deadline:
                    raise TimeoutError("AS7343: AVALID never set")
                time.sleep(0.002)
            block = read_reg(self.t, self.addr, self.REG_ASTATUS, 1 + 2 * len(self.CHANNELS))
        finally:
            write_reg(self.t, self.addr, self.REG_ENABLE, [self.ENABLE_PON])
        return self._decode(block, status2)

    def read(self, timeout=2.0) -> dict:
        """One 18-channel measurement: start, wait for AVALID, latch + read."""
        self.start()
        return self.collect(timeout)

    def _decode(self, block, status2) -> dict:
        astatus = block[0]
        values = struct.unpack("<18H", block[1:])
        channels = dict(zip(self.CHANNELS, values))
        spectral = {k: channels[k] for k in self.SPECTRAL}
        vis = [v for k, v in channels.items() if k.startswith("VIS")]
        fd = [v for k, v in channels.items() if k.startswith("FD")]
        return {
            "channels": channels,
            "spectral": spectral,
            "vis": sum(vis) / len(vis),
            "fd": sum(fd) / len(fd),
            "sat": max(spectral.values()) >= self.full_scale,
            "sat_vis": max(vis) >= self.full_scale,
            "asat_flags": (status2 & self.STATUS2_ASAT) | (astatus & 0x80),
            "full_scale": self.full_scale,
            "tint_ms": self.tint_ms,
        }

    def power_off(self) -> None:
        write_reg(self.t, self.addr, self.REG_ENABLE, [0x00])


SENSOR_CLASSES = {VEML7700.ADDR: VEML7700, TSL2591.ADDR: TSL2591, AS7343.ADDR: AS7343}
SENSOR_NAMES = {addr: cls.__name__ for addr, cls in SENSOR_CLASSES.items()}


class Sampler:
    """Every supported sensor on the bus, read together in pipelined rounds:
    the AS7343 frame is started first, the continuously-integrating TSL2591 /
    VEML7700 are read while it runs, then the AS7343 is collected. A round
    therefore takes the slowest integration, not the sum."""

    def __init__(self, transport, tsl_gain="med", tsl_atime=100, as_gain=64):
        self.t = transport
        self.devices = locate_devices(transport)
        self.sensors: list[tuple[Located, object]] = []
        self.period_s = 0.0
        for dev in self.devices:
            cls = SENSOR_CLASSES.get(dev.addr)
            if cls is None:
                continue
            s = cls(transport)
            with reach(transport, dev):
                if isinstance(s, TSL2591):
                    s.configure(gain=tsl_gain, atime_ms=tsl_atime)
                    self.period_s = max(self.period_s, tsl_atime / 1000.0)
                elif isinstance(s, VEML7700):
                    s.configure()
                    self.period_s = max(self.period_s, 0.1)
                else:
                    s.configure(gain=as_gain)
                    self.period_s = max(self.period_s, s.frame_ms / 1000.0)
            self.sensors.append((dev, s))
        self.settings = {"tsl_gain": tsl_gain, "tsl_atime": tsl_atime, "as_gain": as_gain}

    def missing(self, expected=SENSOR_CLASSES) -> list[str]:
        present = {dev.addr for dev, _ in self.sensors}
        return [f"{SENSOR_NAMES[a]} (0x{a:02X})" for a in expected if a not in present]

    def round(self) -> list[tuple[Located, object, dict]]:
        spectral = [(d, s) for d, s in self.sensors if isinstance(s, AS7343)]
        others = [(d, s) for d, s in self.sensors if not isinstance(s, AS7343)]
        for d, s in spectral:
            with reach(self.t, d):
                s.start()
        out = {}
        for d, s in others:
            with reach(self.t, d):
                out[id(s)] = s.read()
        for d, s in spectral:
            with reach(self.t, d):
                out[id(s)] = s.collect()
        return [(d, s, out[id(s)]) for d, s in self.sensors]

    def close(self) -> None:
        for d, s in self.sensors:
            if isinstance(s, (TSL2591, AS7343)):
                with reach(self.t, d):
                    s.power_off()


def flatten(sensor, r: dict) -> dict:
    """Reading dict -> ordered {metric: value} for tables; None = unusable."""
    if isinstance(sensor, TSL2591):
        return {"full": r["full"], "ir": r["ir"],
                "lux~": None if r["lux_approx"] is None else r["lux_approx"], "sat": int(r["full"] >= 0xFFFF or r["lux_approx"] is None)}
    if isinstance(sensor, VEML7700):
        return {"als": r["als"], "white": r["white"], "lux~": r["lux_approx"], "sat": int(r["als"] >= 0xFFFF)}
    out = {k: v for k, v in r["spectral"].items()}
    out["VIS"] = r["vis"]
    out["sat"] = int(r["sat"])
    return out
