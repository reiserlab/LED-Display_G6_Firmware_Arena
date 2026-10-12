"""LAB-214 peripherals on the Qwiic / STEMMA QT jack, over the controller's
I2C bridge (GET_I2C_SCAN 0xB0 / I2C_TRANSFER 0xB1 -> Wire1 = J2 on arena_12-18).

Companion to tests/qwiic_sensors.py (the LAB-211 light sensors), same
philosophy: the minimum register map to identify the part, round-trip a
config register and take one reading -- not full drivers. Nothing here is
in firmware; every part is driven from the host, which is the LAB-214 rule
(firmware-native only when the controller must act or log on its own
timeline).

Parts:
  0x44        SHT4x      temperature / humidity  (Adafruit 5776 SHT41, 5665 SHT45)
  0x6A/0x6B   LSM6DSOX   6-axis IMU: tilt + latched "was it bumped" flag
                         (Adafruit 4438; WHO_AM_I 0x6C -- the LSM6DSO32 answers
                         the same but has a different g-range table)
  0x40..0x7F  PCA9685    16-channel PWM / servo driver (SparkFun Servo pHAT
                         DEV-15316, Adafruit 815). NOTE: it also ACKs the
                         "all call" address 0x70 (reads too) until MODE1.ALLCALL
                         is cleared, which is the PCA9548 mux's default address;
                         the pHAT's logic needs its own USB-C power.
"""

import math
import struct
import time

from .qwiic_sensors import AS7343, TSL2591, VEML7700, I2CError, i2c_xfer, is_mux, read_reg, reach, write_reg


# ── Sensirion CRC-8 (poly 0x31, init 0xFF, no reflection, no final XOR) ─────

def crc8_sensirion(data: bytes) -> int:
    crc = 0xFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x31) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def _unpack_sensirion_words(frame: bytes) -> tuple[list[int], bool]:
    """[msb lsb crc] x n -> (words, every_crc_ok)."""
    words, ok = [], True
    for i in range(0, len(frame), 3):
        words.append((frame[i] << 8) | frame[i + 1])
        ok &= crc8_sensirion(frame[i:i + 2]) == frame[i + 2]
    return words, ok


# ── SHT4x (0x44) ─────────────────────────────────────────────────────────────

class SHT4x:
    """Command/response part: write one command byte, wait, read 6 bytes
    (T msb, T lsb, crc, RH msb, RH lsb, crc). It NACKs a read while measuring,
    so the two legs are separate bridge transfers, not a repeated-start pair."""
    ADDR = 0x44
    CMD_MEASURE_HIGH = 0xFD    # 8.3 ms max, +-0.2 C / +-1.8 %RH (SHT41)
    CMD_MEASURE_MED = 0xF6     # 4.5 ms
    CMD_MEASURE_LOW = 0xE0     # 1.7 ms
    CMD_SERIAL = 0x89
    CMD_SOFT_RESET = 0x94
    MEASURE_WAIT_S = {CMD_MEASURE_HIGH: 0.010, CMD_MEASURE_MED: 0.006, CMD_MEASURE_LOW: 0.003}

    def __init__(self, transport, addr=ADDR):
        self.t = transport
        self.addr = addr

    def _command(self, cmd: int, wait_s: float, rlen: int = 6) -> bytes:
        i2c_xfer(self.t, self.addr, bytes([cmd]), 0)
        time.sleep(wait_s)
        return i2c_xfer(self.t, self.addr, b"", rlen)

    def identify(self) -> dict:
        words, crc_ok = _unpack_sensirion_words(self._command(self.CMD_SERIAL, 0.002))
        return {"serial": (words[0] << 16) | words[1], "crc_ok": crc_ok}

    def soft_reset(self) -> None:
        i2c_xfer(self.t, self.addr, bytes([self.CMD_SOFT_RESET]), 0)
        time.sleep(0.002)

    @staticmethod
    def convert(t_raw: int, rh_raw: int) -> tuple[float, float]:
        """Datasheet 4.6: T = -45 + 175 * raw / 65535, RH = -6 + 125 * raw / 65535,
        RH clipped to 0..100 (the formula overshoots slightly at the ends)."""
        t_c = -45.0 + 175.0 * t_raw / 65535.0
        rh = min(100.0, max(0.0, -6.0 + 125.0 * rh_raw / 65535.0))
        return t_c, rh

    def read(self, cmd=CMD_MEASURE_HIGH) -> dict:
        (t_raw, rh_raw), crc_ok = _unpack_sensirion_words(self._command(cmd, self.MEASURE_WAIT_S[cmd]))
        t_c, rh = self.convert(t_raw, rh_raw)
        return {"temp_c": t_c, "rh_pct": rh, "t_raw": t_raw, "rh_raw": rh_raw, "crc_ok": crc_ok}


# ── LSM6DSOX (0x6A, SA0 high -> 0x6B) ────────────────────────────────────────

class LSM6DSOX:
    """Accelerometer for arena inclination plus the embedded wake-up detector
    as a latched "bumped since last read" flag. Register names per the
    LSM6DSOX datasheet / AN5272; the IF_INC auto-increment default (CTRL3_C
    bit 2) is relied on for the 6-byte block reads."""
    ADDR = 0x6A
    ADDR_ALT = 0x6B
    WHO_AM_I_EXPECTED = 0x6C   # LSM6DSOX and LSM6DSO32 both answer 0x6C
    REG_WHO_AM_I = 0x0F
    REG_CTRL1_XL = 0x10
    REG_CTRL2_G = 0x11
    REG_CTRL3_C = 0x12
    REG_WAKE_UP_SRC = 0x1B
    REG_STATUS = 0x1E
    REG_OUT_TEMP_L = 0x20
    REG_OUTX_L_G = 0x22
    REG_OUTX_L_A = 0x28
    REG_TAP_CFG0 = 0x56
    REG_TAP_CFG2 = 0x58
    REG_WAKE_UP_THS = 0x5B
    REG_WAKE_UP_DUR = 0x5C
    REG_MD1_CFG = 0x5E
    CTRL3_BDU = 0x40
    CTRL3_IF_INC = 0x04
    CTRL3_SW_RESET = 0x01
    STATUS_XLDA = 0x01
    ODR_CODE = {0: 0x0, 12.5: 0x1, 26: 0x2, 52: 0x3, 104: 0x4, 208: 0x5, 416: 0x6}
    FS_XL_CODE = {2: 0x0, 16: 0x1, 4: 0x2, 8: 0x3}      # g
    FS_G_CODE = {250: 0x0, 500: 0x1, 1000: 0x2, 2000: 0x3}  # dps
    TAP_CFG0_LIR = 0x01
    TAP_CFG0_INT_CLR_ON_READ = 0x40
    TAP_CFG2_INTERRUPTS_ENABLE = 0x80
    MD1_CFG_INT1_WU = 0x20
    WAKE_UP_SRC_WU_IA = 0x08
    WAKE_UP_SRC_AXES = 0x07       # X_WU bit2, Y_WU bit1, Z_WU bit0

    def __init__(self, transport, addr=ADDR):
        self.t = transport
        self.addr = addr
        self.fs_g = 2
        self.fs_dps = 250

    def identify(self) -> dict:
        who = read_reg(self.t, self.addr, self.REG_WHO_AM_I)[0]
        return {"who_am_i": who, "id_ok": who == self.WHO_AM_I_EXPECTED}

    def configure(self, odr_hz=104, fs_g=2, gyro_odr_hz=104, fs_dps=250) -> None:
        """Accel + gyro on at `odr_hz`, block data update on, auto-increment on.
        A slow ODR (12.5 Hz) is plenty for inclination; the wake-up detector
        wants >= 26 Hz to catch a tap-like bump."""
        self.fs_g, self.fs_dps = fs_g, fs_dps
        write_reg(self.t, self.addr, self.REG_CTRL3_C, [self.CTRL3_BDU | self.CTRL3_IF_INC])
        write_reg(self.t, self.addr, self.REG_CTRL1_XL,
                  [(self.ODR_CODE[odr_hz] << 4) | (self.FS_XL_CODE[fs_g] << 2)])
        write_reg(self.t, self.addr, self.REG_CTRL2_G,
                  [(self.ODR_CODE[gyro_odr_hz] << 4) | (self.FS_G_CODE[fs_dps] << 2)])
        time.sleep(2.5 / max(odr_hz, 12.5) + 0.01)   # first samples after the ODR change

    def ctrl_roundtrip(self) -> bool:
        """Proves a register write lands: CTRL1_XL reads back what configure() wrote."""
        want = read_reg(self.t, self.addr, self.REG_CTRL1_XL)[0]
        write_reg(self.t, self.addr, self.REG_CTRL1_XL, [want ^ 0x10])
        try:
            return read_reg(self.t, self.addr, self.REG_CTRL1_XL)[0] == (want ^ 0x10)
        finally:
            write_reg(self.t, self.addr, self.REG_CTRL1_XL, [want])

    @property
    def accel_g_per_lsb(self) -> float:
        return {2: 0.061e-3, 4: 0.122e-3, 8: 0.244e-3, 16: 0.488e-3}[self.fs_g]

    @property
    def gyro_dps_per_lsb(self) -> float:
        return {250: 8.75e-3, 500: 17.5e-3, 1000: 35e-3, 2000: 70e-3}[self.fs_dps]

    def read(self) -> dict:
        """One block read of temp + gyro + accel (OUT_TEMP_L .. OUTZ_H_A, 14 B)."""
        block = read_reg(self.t, self.addr, self.REG_OUT_TEMP_L, 14)
        temp_raw, gx, gy, gz, ax, ay, az = struct.unpack("<7h", block)
        a = [v * self.accel_g_per_lsb for v in (ax, ay, az)]
        g = [v * self.gyro_dps_per_lsb for v in (gx, gy, gz)]
        return {"accel_g": tuple(a), "gyro_dps": tuple(g),
                "temp_c": 25.0 + temp_raw / 256.0, "accel_raw": (ax, ay, az),
                **self.tilt(a)}

    @staticmethod
    def tilt(a) -> dict:
        """Inclination from the gravity vector: pitch about Y (X tipping),
        roll about X (Y tipping), and the angle between the board's Z axis
        and vertical -- all degrees. |a| should be ~1 g when still."""
        ax, ay, az = a
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm == 0:
            return {"pitch_deg": float("nan"), "roll_deg": float("nan"),
                    "from_vertical_deg": float("nan"), "accel_norm_g": 0.0}
        return {
            "pitch_deg": math.degrees(math.atan2(ax, math.sqrt(ay * ay + az * az))),
            "roll_deg": math.degrees(math.atan2(ay, az)),
            "from_vertical_deg": math.degrees(math.acos(max(-1.0, min(1.0, abs(az) / norm)))),
            "accel_norm_g": norm,
        }

    # Latched bump detection (AN5272 "wake-up interrupt"): the slope of the
    # acceleration (static gravity filtered out) exceeding WK_THS on any axis
    # sets WAKE_UP_SRC.WU_IA, and the host asks "was it bumped since I last
    # looked" once per trial. Bench-verified 2026-10-12 on the bridge: the flag
    # only latches when the wake-up is ALSO routed to INT1 (MD1_CFG.INT1_WU --
    # the pin need not be wired), and it only clears on the WAKE_UP_SRC read
    # when INT_CLR_ON_READ is set together with LIR. LIR alone shows the live
    # status (clears as soon as the motion stops); LIR + route without
    # INT_CLR_ON_READ latches forever.
    def arm_bump(self, threshold_g=0.25) -> None:
        """Threshold resolution is FS_XL / 64 per LSB (31.25 mg at +-2 g), 1..63."""
        lsb_g = self.fs_g / 64.0
        ths = max(0, min(63, round(threshold_g / lsb_g)))
        write_reg(self.t, self.addr, self.REG_WAKE_UP_DUR, [0x00])   # WAKE_DUR 0: 1 ODR sample, FS/64 steps
        write_reg(self.t, self.addr, self.REG_WAKE_UP_THS, [ths])
        write_reg(self.t, self.addr, self.REG_TAP_CFG0, [self.TAP_CFG0_LIR | self.TAP_CFG0_INT_CLR_ON_READ])
        write_reg(self.t, self.addr, self.REG_MD1_CFG, [self.MD1_CFG_INT1_WU])
        write_reg(self.t, self.addr, self.REG_TAP_CFG2, [self.TAP_CFG2_INTERRUPTS_ENABLE])
        self.bump_flag()   # clear anything latched while arming

    def disarm_bump(self) -> None:
        write_reg(self.t, self.addr, self.REG_TAP_CFG2, [0x00])
        write_reg(self.t, self.addr, self.REG_MD1_CFG, [0x00])
        write_reg(self.t, self.addr, self.REG_TAP_CFG0, [0x00])

    def bump_flag(self) -> dict:
        """Read-and-clear. axes = which of X/Y/Z crossed the threshold."""
        src = read_reg(self.t, self.addr, self.REG_WAKE_UP_SRC)[0]
        axes = "".join(n for n, bit in (("X", 4), ("Y", 2), ("Z", 1)) if src & bit)
        return {"bumped": bool(src & self.WAKE_UP_SRC_WU_IA), "axes": axes, "raw": src}

    def power_off(self) -> None:
        self.disarm_bump()
        write_reg(self.t, self.addr, self.REG_CTRL1_XL, [0x00])
        write_reg(self.t, self.addr, self.REG_CTRL2_G, [0x00])


# ── PCA9685 (0x40 default; A5..A0 straps -> 0x40..0x7F) ──────────────────────

class PCA9685:
    """16-channel 12-bit PWM for hobby servos. Host-side only: "go to position"
    is a 4-byte register write (~1 ms over the bridge); frame-locked motion
    would need firmware (LAB-214 says: not before it is actually needed).
    Servo power never comes from the Qwiic 3.3 V -- the driver board's V+
    rail takes its own 5-6 V supply; only the logic side is on the bus."""
    ADDR = 0x40
    ALLCALL_ADDR = 0x70   # answers here too until disable_allcall(); collides with a PCA9548 at 0x70
    REG_MODE1 = 0x00
    REG_MODE2 = 0x01
    REG_LED0_ON_L = 0x06
    REG_ALL_LED_ON_L = 0xFA
    REG_PRESCALE = 0xFE
    MODE1_RESTART = 0x80
    MODE1_AI = 0x20          # register auto-increment
    MODE1_SLEEP = 0x10
    MODE1_ALLCALL = 0x01
    MODE2_OUTDRV = 0x04      # totem pole (the default)
    OSC_HZ = 25_000_000
    CHANNELS = 16
    COUNTS = 4096

    def __init__(self, transport, addr=ADDR):
        self.t = transport
        self.addr = addr
        self.freq_hz = 200.0  # POR prescale 0x1E

    def identify(self) -> dict:
        """No ID register: read MODE1 + PRESCALE and check that the reserved
        bits read as the part defines them (MODE1 bit 4..0 = 1 0 0 0 1 at POR)."""
        mode1 = read_reg(self.t, self.addr, self.REG_MODE1)[0]
        prescale = read_reg(self.t, self.addr, self.REG_PRESCALE)[0]
        return {"mode1": mode1, "prescale": prescale,
                "freq_hz": self.freq_for_prescale(prescale),
                "sleeping": bool(mode1 & self.MODE1_SLEEP),
                "allcall": bool(mode1 & self.MODE1_ALLCALL)}

    @classmethod
    def prescale_for(cls, freq_hz: float) -> int:
        """Datasheet 7.3.5: prescale = round(25 MHz / (4096 * f)) - 1, 3..255."""
        return max(3, min(255, round(cls.OSC_HZ / (cls.COUNTS * freq_hz)) - 1))

    @classmethod
    def freq_for_prescale(cls, prescale: int) -> float:
        return cls.OSC_HZ / (cls.COUNTS * (prescale + 1))

    def configure(self, freq_hz=50.0) -> None:
        """Set the PWM frequency (prescale is writable only in SLEEP), wake,
        enable auto-increment, and turn ALLCALL off so 0x70 is free for a mux."""
        mode1 = read_reg(self.t, self.addr, self.REG_MODE1)[0] & ~self.MODE1_RESTART & 0xFF
        write_reg(self.t, self.addr, self.REG_MODE1, [(mode1 | self.MODE1_SLEEP) & ~self.MODE1_ALLCALL & 0xFF])
        write_reg(self.t, self.addr, self.REG_PRESCALE, [self.prescale_for(freq_hz)])
        write_reg(self.t, self.addr, self.REG_MODE1, [(mode1 | self.MODE1_AI) & ~self.MODE1_SLEEP & ~self.MODE1_ALLCALL & 0xFF])
        time.sleep(0.001)                                   # oscillator start-up >= 500 us
        write_reg(self.t, self.addr, self.REG_MODE1,
                  [(mode1 | self.MODE1_AI | self.MODE1_RESTART) & ~self.MODE1_SLEEP & ~self.MODE1_ALLCALL & 0xFF])
        self.freq_hz = self.freq_for_prescale(self.prescale_for(freq_hz))

    def disable_allcall(self) -> None:
        mode1 = read_reg(self.t, self.addr, self.REG_MODE1)[0]
        write_reg(self.t, self.addr, self.REG_MODE1, [mode1 & ~self.MODE1_ALLCALL & ~self.MODE1_RESTART & 0xFF])

    def set_counts(self, channel: int, on: int, off: int) -> None:
        if not 0 <= channel < self.CHANNELS:
            raise ValueError(f"channel {channel} out of 0..15")
        write_reg(self.t, self.addr, self.REG_LED0_ON_L + 4 * channel, struct.pack("<HH", on & 0x1FFF, off & 0x1FFF))

    def read_counts(self, channel: int) -> tuple[int, int]:
        return struct.unpack("<HH", read_reg(self.t, self.addr, self.REG_LED0_ON_L + 4 * channel, 4))

    def pulse_to_counts(self, pulse_us: float) -> int:
        return max(0, min(self.COUNTS - 1, round(pulse_us * 1e-6 * self.freq_hz * self.COUNTS)))

    def set_pulse_us(self, channel: int, pulse_us: float) -> int:
        """Hobby servo: ~1000-2000 us within a 20 ms (50 Hz) frame."""
        off = self.pulse_to_counts(pulse_us)
        self.set_counts(channel, 0, off)
        return off

    def set_off(self, channel: int) -> None:
        """Full OFF (LEDn_OFF_H bit 4) -- the servo goes limp, no holding torque."""
        self.set_counts(channel, 0, 0x1000)

    def all_off(self) -> None:
        write_reg(self.t, self.addr, self.REG_ALL_LED_ON_L, struct.pack("<HH", 0, 0x1000))

    def sleep(self) -> None:
        mode1 = read_reg(self.t, self.addr, self.REG_MODE1)[0]
        write_reg(self.t, self.addr, self.REG_MODE1, [(mode1 | self.MODE1_SLEEP) & ~self.MODE1_RESTART & 0xFF])


# ── Address book for the probe ───────────────────────────────────────────────

PERIPHERAL_CLASSES = {
    SHT4x.ADDR: SHT4x,
    LSM6DSOX.ADDR: LSM6DSOX,
    LSM6DSOX.ADDR_ALT: LSM6DSOX,
    PCA9685.ADDR: PCA9685,
}

KNOWN_PERIPHERALS = {
    SHT4x.ADDR: "SHT4x temperature/humidity",
    LSM6DSOX.ADDR: "LSM6DSOX 6-axis IMU (tilt/bump)",
    LSM6DSOX.ADDR_ALT: "LSM6DSOX 6-axis IMU, SA0 high",
    PCA9685.ADDR: "PCA9685 16-ch PWM / servo driver",
    0x18: "LIS3DH accelerometer (LAB-214 alt part, no driver yet)",
    0x19: "LIS3DH accelerometer, SA0 high (no driver yet)",
}


def pca9685_allcall_shadow(transport, scan: list[int]) -> int | None:
    """The address a PCA9685 on the bus is shadowing with its all-call, or None.
    A PCA9685 ACKs its all-call address (0x70 at power-on, the PCA9548 mux's
    default) for as long as MODE1.ALLCALL is set -- reads included, it answers
    with MODE1 (bench 2026-10-12), so a read probe cannot tell it from a mux.
    `locate_devices()` must not map mux channels through it. Decided from MODE1."""
    if PCA9685.ADDR not in scan or PCA9685.ALLCALL_ADDR not in scan:
        return None
    try:
        mode1 = read_reg(transport, PCA9685.ADDR, PCA9685.REG_MODE1)[0]
    except I2CError:
        return None
    return PCA9685.ALLCALL_ADDR if mode1 & PCA9685.MODE1_ALLCALL else None


# ── Read-only transfer per device, for the bus soak (scripts/qwiic_soak.py, HIL test) ──

# address -> (write bytes, read length) of a harmless ID / status register
SOAK_READ = {
    VEML7700.ADDR: (bytes([VEML7700.REG_ID]), 2),
    TSL2591.ADDR: (bytes([TSL2591.CMD | TSL2591.REG_ID]), 1),
    AS7343.ADDR: (bytes([AS7343.REG_STATUS]), 1),      # bank-0 register, no bank switch needed
    LSM6DSOX.ADDR: (bytes([LSM6DSOX.REG_WHO_AM_I]), 1),
    LSM6DSOX.ADDR_ALT: (bytes([LSM6DSOX.REG_WHO_AM_I]), 1),
    PCA9685.ADDR: (bytes([PCA9685.REG_MODE1]), 1),
}


def soak_transfer(transport, dev, rlen=0) -> float:
    """One read-only transaction against `dev` (a Located), disturbing no sensor
    configuration; returns the host round-trip in seconds. SHT4x gets its
    serial-number command (two legs); a mux its control register; anything
    else a register read of at least `rlen` bytes; an unknown part gets a plain
    1-byte read so nothing is ever written to it."""
    t0 = time.perf_counter()
    with reach(transport, dev):
        if dev.addr == SHT4x.ADDR:
            i2c_xfer(transport, dev.addr, bytes([SHT4x.CMD_SERIAL]), 0)
            time.sleep(0.001)
            i2c_xfer(transport, dev.addr, b"", 6)
        elif is_mux(dev.addr):
            i2c_xfer(transport, dev.addr, b"", 1)
        elif dev.addr in SOAK_READ:
            wbytes, n = SOAK_READ[dev.addr]
            i2c_xfer(transport, dev.addr, wbytes, max(n, rlen) if rlen else n)
        else:
            i2c_xfer(transport, dev.addr, b"", 1)   # unknown part: plain read, never a pointer write
    return time.perf_counter() - t0
