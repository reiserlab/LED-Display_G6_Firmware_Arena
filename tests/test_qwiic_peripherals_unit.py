"""Hardware-free checks of tests/qwiic_peripherals.py: Sensirion CRC, SHT4x
conversion, LSM6DSOX tilt math, PCA9685 prescale / pulse math."""

import math

from .qwiic_peripherals import LSM6DSOX, PCA9685, SHT4x, crc8_sensirion, _unpack_sensirion_words


def test_sensirion_crc_datasheet_vector():
    assert crc8_sensirion(bytes([0xBE, 0xEF])) == 0x92   # SHT4x datasheet 4.4 example
    assert crc8_sensirion(b"\x00\x00") == 0x81


def test_unpack_words_flags_a_bad_crc():
    good = bytes([0xBE, 0xEF, 0x92, 0x00, 0x00, 0x81])
    assert _unpack_sensirion_words(good) == ([0xBEEF, 0x0000], True)
    bad = bytes([0xBE, 0xEF, 0x93, 0x00, 0x00, 0x81])
    assert _unpack_sensirion_words(bad) == ([0xBEEF, 0x0000], False)


def test_sht4x_conversion_endpoints():
    assert SHT4x.convert(0, 0) == (-45.0, 0.0)            # RH formula gives -6 %, clipped
    t, rh = SHT4x.convert(0xFFFF, 0xFFFF)
    assert math.isclose(t, 130.0) and rh == 100.0          # 119 % clipped
    t, rh = SHT4x.convert(0x62AC, 0x8E85)                   # bench frame 2026-10-12
    assert math.isclose(t, 22.45, abs_tol=0.01) and math.isclose(rh, 63.6, abs_tol=0.1)


def test_tilt_from_gravity_vector():
    flat = LSM6DSOX.tilt((0.0, 0.0, 1.0))
    assert math.isclose(flat["from_vertical_deg"], 0.0) and math.isclose(flat["pitch_deg"], 0.0)
    assert math.isclose(flat["roll_deg"], 0.0) and math.isclose(flat["accel_norm_g"], 1.0)
    on_edge = LSM6DSOX.tilt((1.0, 0.0, 0.0))
    assert math.isclose(on_edge["from_vertical_deg"], 90.0) and math.isclose(on_edge["pitch_deg"], 90.0)
    upside_down = LSM6DSOX.tilt((0.0, 0.0, -1.0))
    assert math.isclose(upside_down["from_vertical_deg"], 0.0)   # |az| -- inclination, not orientation
    ten_deg = LSM6DSOX.tilt((math.sin(math.radians(10)), 0.0, math.cos(math.radians(10))))
    assert math.isclose(ten_deg["from_vertical_deg"], 10.0, abs_tol=1e-9)
    assert math.isnan(LSM6DSOX.tilt((0.0, 0.0, 0.0))["pitch_deg"])


def test_lsm6dsox_scale_factors():
    s = LSM6DSOX.__new__(LSM6DSOX)
    s.fs_g, s.fs_dps = 2, 250
    assert math.isclose(16393 * s.accel_g_per_lsb, 1.0, rel_tol=0.001)   # 1 g at +-2 g
    assert math.isclose(s.gyro_dps_per_lsb, 8.75e-3)


def test_pca9685_prescale_and_pulse_math():
    assert PCA9685.prescale_for(50.0) == 121                        # datasheet 7.3.5 worked example
    assert math.isclose(PCA9685.freq_for_prescale(121), 50.03, abs_tol=0.01)
    assert PCA9685.prescale_for(10000.0) == 3 and PCA9685.prescale_for(1.0) == 255
    pwm = PCA9685.__new__(PCA9685)
    pwm.freq_hz = PCA9685.freq_for_prescale(121)
    assert pwm.pulse_to_counts(1500) == 307                         # 1.5 ms of a 20 ms frame
    assert pwm.pulse_to_counts(0) == 0 and pwm.pulse_to_counts(1e6) == 4095
