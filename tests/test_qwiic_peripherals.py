"""LAB-214 peripherals on the Qwiic jack (tests/qwiic_peripherals.py): SHT4x,
LSM6DSOX, PCA9685 over the 0xB0/0xB1 bridge, plus a bus-limit check with
whatever is plugged in. Hardware-adaptive like test_qwiic_i2c.py: each part's
tests skip unless it answers; the module skips on a build without the jack.

Run:
    pixi run test-serial -- tests/test_qwiic_peripherals.py -s
"""

import time

import pytest

from .qwiic_peripherals import LSM6DSOX, PCA9685, SHT4x, soak_transfer
from .qwiic_sensors import NoQwiicJack, I2CError, i2c_scan, i2c_xfer, locate_devices, reach, write_reg
from .test_health import read_health


@pytest.fixture(scope="module")
def scan(transport):
    try:
        return i2c_scan(transport)
    except NoQwiicJack as e:
        pytest.skip(str(e))


@pytest.fixture(scope="module")
def devices(transport, scan):
    return locate_devices(transport)


def _find(devices, *addrs):
    hits = [d for d in devices if d.addr in addrs]
    if not hits:
        pytest.skip("no device at " + "/".join(f"0x{a:02X}" for a in addrs) + " on the Qwiic bus")
    return hits


# ── SHT4x ────────────────────────────────────────────────────────────────────

def test_sht4x_serial_and_reading(transport, devices):
    for dev in _find(devices, SHT4x.ADDR):
        with reach(transport, dev):
            s = SHT4x(transport)
            ident = s.identify()
            assert ident["crc_ok"], f"SHT4x @ {dev.where()}: serial-number CRC failed"
            assert ident["serial"] not in (0, 0xFFFFFFFF)
            r = s.read()
            print(f"\nSHT4x @ {dev.where()}: serial 0x{ident['serial']:08X} {r}")
            assert r["crc_ok"], "measurement CRC failed"
            assert 0.0 < r["temp_c"] < 50.0, "implausible lab temperature"
            assert 0.0 <= r["rh_pct"] <= 100.0


def test_sht4x_read_while_measuring_is_a_nack_not_a_hang(transport, devices):
    # The bridge must report the busy NACK (status 2) immediately -- the host
    # library relies on two separate legs with a wait, so a repeated-start
    # read straight after the command must fail cleanly.
    for dev in _find(devices, SHT4x.ADDR):
        with reach(transport, dev):
            t0 = time.monotonic()
            with pytest.raises(I2CError) as e:
                i2c_xfer(transport, SHT4x.ADDR, bytes([SHT4x.CMD_MEASURE_HIGH]), 6)
            assert e.value.status in (2, 4), e.value
            assert time.monotonic() - t0 < 0.5
            time.sleep(0.012)
            i2c_xfer(transport, SHT4x.ADDR, b"", 6)   # drain the finished measurement


# ── LSM6DSOX ─────────────────────────────────────────────────────────────────

def test_lsm6dsox_identify_configure_and_gravity(transport, devices):
    for dev in _find(devices, LSM6DSOX.ADDR, LSM6DSOX.ADDR_ALT):
        with reach(transport, dev):
            s = LSM6DSOX(transport, dev.addr)
            ident = s.identify()
            assert ident["id_ok"], f"WHO_AM_I 0x{ident['who_am_i']:02X} != 0x6C"
            s.configure(odr_hz=104)
            try:
                assert s.ctrl_roundtrip(), "CTRL1_XL did not read back"
                r = s.read()
                print(f"\nLSM6DSOX @ {dev.where()}: {r}")
                # Still on the bench: the accelerometer sees 1 g of gravity.
                assert 0.85 < r["accel_norm_g"] < 1.15, f"|a| = {r['accel_norm_g']:.3f} g, expected ~1 g at rest"
                assert 0.0 <= r["from_vertical_deg"] <= 90.0
                assert -10.0 < r["temp_c"] < 60.0
            finally:
                s.power_off()


def test_lsm6dsox_bump_flag_latches_and_clears_on_read(transport, devices):
    # No physical bump needed: with the threshold at 0 the slope detector fires
    # on noise; raising it back to the maximum silences it. A latched flag then
    # survives the quiet period, reads True exactly once, and is False after.
    for dev in _find(devices, LSM6DSOX.ADDR, LSM6DSOX.ADDR_ALT):
        with reach(transport, dev):
            s = LSM6DSOX(transport, dev.addr)
            s.configure(odr_hz=104)
            try:
                s.arm_bump(threshold_g=2.0)                      # WK_THS = 63 (max): quiet
                time.sleep(0.1)
                assert not s.bump_flag()["bumped"], "flag set while still and at the maximum threshold"
                s.arm_bump(threshold_g=0.0)                      # fire on noise
                time.sleep(0.2)
                write_reg(transport, dev.addr, s.REG_WAKE_UP_THS, [63])   # silence WITHOUT reading the flag
                time.sleep(0.3)
                first, second = s.bump_flag(), s.bump_flag()
                print(f"\nLSM6DSOX @ {dev.where()}: latched {first}, after read {second}")
                assert first["bumped"], "wake-up event was not latched across the quiet period"
                assert first["axes"], "latched event without an axis"
                assert not second["bumped"], "flag did not clear on read"
            finally:
                s.power_off()


# ── PCA9685 ──────────────────────────────────────────────────────────────────

def test_pca9685_allcall_shadow_is_not_mistaken_for_a_mux(transport, scan):
    """Runs BEFORE the round-trip test (file order): configure() clears ALLCALL.
    While MODE1.ALLCALL is set the PCA9685 also ACKs 0x70 (the PCA9548
    default). locate_devices() must keep that address a root device and never
    map mux channels through it, whatever 0x70 does with a read."""
    from .qwiic_peripherals import pca9685_allcall_shadow
    from .qwiic_sensors import mux_responds
    if PCA9685.ADDR not in scan:
        pytest.skip("no PCA9685 on the bus")
    pwm = PCA9685(transport)
    mode1 = pwm.identify()["mode1"]
    if not mode1 & PCA9685.MODE1_ALLCALL:
        pytest.skip("PCA9685 all-call already off (configure() ran); power-cycle the board to retest")
    assert PCA9685.ALLCALL_ADDR in scan, "ALLCALL set but 0x70 did not ACK the scan"
    print(f"\n0x70 answers a 1-byte read: {mux_responds(transport, PCA9685.ALLCALL_ADDR)}")
    assert pca9685_allcall_shadow(transport, scan) == PCA9685.ALLCALL_ADDR
    devices = locate_devices(transport)
    assert not any(d.mux == PCA9685.ALLCALL_ADDR for d in devices), "all-call address was mapped as a mux"
    assert any(d.addr == PCA9685.ALLCALL_ADDR and d.mux is None for d in devices)
    pwm.disable_allcall()
    try:
        assert PCA9685.ALLCALL_ADDR not in i2c_scan(transport), "0x70 still ACKs with ALLCALL cleared"
    finally:
        write_reg(transport, PCA9685.ADDR, PCA9685.REG_MODE1, [mode1 & ~PCA9685.MODE1_RESTART & 0xFF])  # restore


def test_pca9685_mode1_and_prescale_roundtrip(transport, devices):
    for dev in _find(devices, PCA9685.ADDR):
        with reach(transport, dev):
            pwm = PCA9685(transport)
            before = pwm.identify()
            print(f"\nPCA9685 @ {dev.where()}: {before}")
            pwm.configure(50.0)
            try:
                after = pwm.identify()
                assert after["prescale"] == PCA9685.prescale_for(50.0) == 121
                assert not after["sleeping"] and not after["allcall"]
                off = pwm.set_pulse_us(15, 1500)
                assert pwm.read_counts(15) == (0, off)
                pwm.set_off(15)
                assert pwm.read_counts(15)[1] & 0x1000, "full-OFF bit not set"
            finally:
                pwm.all_off()
                write_reg(transport, PCA9685.ADDR, PCA9685.REG_MODE1,
                          [(before["mode1"] | PCA9685.MODE1_SLEEP) & ~PCA9685.MODE1_RESTART & 0xFF])  # back to how we found it


# ── Bus limits with whatever is attached ─────────────────────────────────────

def test_bus_soak_no_errors_and_bounded_loop_cost(transport, devices):
    """200 rounds of read-only transfers over every device; zero I2C errors,
    and the controller's worst 1 s loop gap stays under 5 ms (one transfer
    blocks the loop for its wire time only -- the full-scan 12 ms is the
    ceiling the display path must never see from sensor traffic)."""
    if not devices:
        pytest.skip("nothing on the Qwiic bus")
    time.sleep(2.2)   # let the ~12 ms GET_I2C_SCAN from the fixtures leave the 1 s health window
    errors = 0
    t_end = time.monotonic() + 2.5
    rounds = 0
    while rounds < 200 or time.monotonic() < t_end:
        for d in devices:
            try:
                soak_transfer(transport, d, 0)
            except I2CError:
                errors += 1
        rounds += 1
    h = read_health(transport)
    print(f"\nsoak: {rounds} rounds x {len(devices)} devices, {errors} errors, loop_max_1s_us={h.loop_max_1s_us}")
    assert errors == 0
    assert h.loop_max_1s_us < 5000, f"sensor traffic stalled the loop {h.loop_max_1s_us} us"
