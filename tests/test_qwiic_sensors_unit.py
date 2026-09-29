"""Hardware-free checks of the Qwiic sensor helpers (tests/qwiic_sensors.py): the TSL2591
saturation ceiling, flatten()'s saturation flags, and the arena-port candidate list. Does not
use the `transport` fixture, so it runs without a controller attached."""

import struct

from scripts import arena_port
from .qwiic_sensors import TSL2591, VEML7700, flatten


class _FakeTsl(TSL2591):
    """TSL2591 whose registers come from the constructor instead of the bus."""

    def __init__(self, full, ir, atime_ms=100, gain="med"):
        self.gain, self.atime_ms, self._d = gain, atime_ms, (full, ir)

    def read_reg(self, reg, n=1) -> bytes:
        return bytes([1]) if reg == self.REG_STATUS else struct.pack("<HH", *self._d)


def test_tsl2591_ceiling_follows_integration_time():
    assert TSL2591.ceiling_for(100) == 36863
    for atime in (200, 300, 400, 500, 600):
        assert TSL2591.ceiling_for(atime) == 65535
    assert _FakeTsl(0, 0, atime_ms=100).ceiling == 36863
    assert _FakeTsl(0, 0, atime_ms=300).ceiling == 65535


def test_tsl2591_saturation_uses_the_ceiling_not_0xffff():
    at_ceiling = _FakeTsl(36863, 10).read()
    assert at_ceiling["sat"] and at_ceiling["lux_approx"] is None
    below = _FakeTsl(36862, 10).read()
    assert not below["sat"] and below["lux_approx"] is not None
    ir_only = _FakeTsl(5000, 36863).read()
    assert ir_only["sat"] and ir_only["lux_approx"] is None
    long_atime = _FakeTsl(40000, 10, atime_ms=200).read()
    assert not long_atime["sat"] and long_atime["lux_approx"] is not None


def test_tsl2591_dark_is_not_saturated():
    dark = _FakeTsl(0, 0).read()
    assert not dark["sat"] and dark["lux_approx"] is None
    flat = flatten(_FakeTsl(0, 0), dark)
    assert flat["sat"] == 0 and flat["lux~"] is None


def test_flatten_tsl_sat_comes_from_read():
    s = _FakeTsl(36863, 0)
    assert flatten(s, s.read()) == {"full": 36863, "ir": 0, "lux~": None, "sat": 1}


def test_flatten_veml_saturates_on_either_channel():
    s = VEML7700.__new__(VEML7700)
    r = {"als": 100, "white": 0xFFFF, "lux_approx": 5.0}
    assert flatten(s, r)["sat"] == 1
    r = {"als": 100, "white": 200, "lux_approx": 5.0}
    assert flatten(s, r)["sat"] == 0


def test_list_arena_ports_prefers_named_and_reports_ambiguity(monkeypatch):
    class P:
        def __init__(self, device, product=None, vid=None, pid=None):
            self.device, self.product, self.vid, self.pid = device, product, vid, pid

    fake = [
        P("/dev/cu.other", product="Foo", vid=0x1234, pid=1),
        P("/dev/cu.teensyB", product="USB Serial", vid=0x16C0, pid=0x0483),
        P("/dev/cu.arenaA", product="G6_Arena", vid=0x16C0, pid=0x0483),
    ]
    monkeypatch.setattr(arena_port.list_ports, "comports", lambda: fake)
    assert arena_port.list_arena_ports() == ["/dev/cu.arenaA", "/dev/cu.teensyB"]
    assert arena_port.find_arena_port() == "/dev/cu.arenaA"
    monkeypatch.setattr(arena_port.list_ports, "comports", lambda: fake[:1])
    assert arena_port.list_arena_ports() == [] and arena_port.find_arena_port() is None
