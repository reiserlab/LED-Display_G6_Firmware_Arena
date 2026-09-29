"""scripts/arena_port.py: candidate ordering and the fail-closed rule (issue #61). No hardware —
`serial.tools.list_ports.comports` is replaced by a fake, and the `transport` fixture is not used."""

import pytest

from scripts import arena_port


class _Port:
    def __init__(self, device, product=None, vid=None, pid=None, serial_number=None):
        self.device, self.product, self.vid, self.pid = device, product, vid, pid
        self.serial_number = serial_number


def _ports(monkeypatch, *ports):
    monkeypatch.setattr(arena_port.list_ports, "comports", lambda: list(ports))


ARENA = _Port("/dev/cu.usbmodem1", product="G6_Arena", vid=0x16C0, pid=0x0483, serial_number="12169940")
TEENSY = _Port("/dev/cu.usbmodem2", product="USB Serial", vid=0x16C0, pid=0x0483)
OTHER = _Port("/dev/cu.other", product="Foo", vid=0x1234, pid=0x0001)
TEENSY_HID = _Port("/dev/cu.usbmodem3", product="Teensy", vid=0x16C0, pid=0x0486)  # not CDC serial


def test_no_candidates(monkeypatch):
    _ports(monkeypatch, OTHER, TEENSY_HID)
    assert arena_port.list_arena_ports() == []
    assert arena_port.find_arena_port() is None


def test_single_named_port(monkeypatch):
    _ports(monkeypatch, OTHER, ARENA)
    assert arena_port.find_arena_port() == ARENA.device


def test_single_unnamed_teensy_still_found(monkeypatch):
    # Windows' inbox CDC driver hides the product string: fall back to VID/PID.
    _ports(monkeypatch, TEENSY)
    assert arena_port.find_arena_port() == TEENSY.device


def test_named_listed_before_unnamed(monkeypatch):
    _ports(monkeypatch, TEENSY, ARENA)
    assert arena_port.list_arena_ports() == [ARENA.device, TEENSY.device]


def test_two_candidates_fail_closed(monkeypatch):
    _ports(monkeypatch, ARENA, TEENSY)
    with pytest.raises(arena_port.AmbiguousArenaPort) as ei:
        arena_port.find_arena_port()
    assert ei.value.candidates == [ARENA.device, TEENSY.device]
    assert "pass --port" in str(ei.value)
    assert "G6_Arena" in str(ei.value) and "sn 12169940" in str(ei.value), "name each candidate"


def test_two_named_arenas_fail_closed(monkeypatch):
    second = _Port("/dev/cu.usbmodem9", product="G6_Arena", vid=0x16C0, pid=0x0483)
    _ports(monkeypatch, ARENA, second)
    with pytest.raises(arena_port.AmbiguousArenaPort):
        arena_port.find_arena_port()


def test_scripts_conftest_resolves_port_lazily():
    """scripts/conftest.py must not run port detection while pytest registers options —
    an ambiguous bench would crash startup before an explicit --port could be parsed."""
    import pathlib
    src = pathlib.Path(__file__).resolve().parents[1].joinpath("scripts", "conftest.py").read_text()
    assert "default=None" in src and "default=find_arena_port()" not in src
    assert "AmbiguousArenaPort" in src
