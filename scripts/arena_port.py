"""Locate the G6 arena controller's USB-CDC serial port on Linux, macOS and Windows.

Matches a port whose USB product string contains "G6_Arena" (set by
scripts/patch_usb_strings.py; Windows' inbox CDC driver does not expose it),
then falls back to any Teensy USB-serial device (VID 16C0, PID 0483).

With more than one candidate the choice is ambiguous and `find_arena_port()`
refuses to guess (`AmbiguousArenaPort`): a second Teensy on the bench — another
controller, say — would otherwise be monitored or tested silently. Callers tell
the user to pass `--port`. This guards the monitor and the HIL suite only:
`pixi run deploy-*` goes through `teensy-cli`, which picks whichever Teensy
enters its bootloader and ignores port selection (see CLAUDE.md).
"""

from serial.tools import list_ports

TEENSY_VID = 0x16C0
TEENSY_SERIAL_PID = 0x0483
USB_PRODUCT = "G6_Arena"  # keep in sync with scripts/patch_usb_strings.py


class AmbiguousArenaPort(RuntimeError):
    """More than one arena/Teensy serial port is attached; pass --port."""

    def __init__(self, candidates):
        # candidates: the ListPortInfo objects, so the message can name each one
        self.candidates = [p.device for p in candidates]
        detail = ", ".join(
            f"{p.device} ({p.product or 'no product string'}"
            f"{', sn ' + p.serial_number if p.serial_number else ''})"
            for p in candidates
        )
        super().__init__("several arena/Teensy serial ports: " + detail + " — pass --port")


def _candidates():
    ports = list(list_ports.comports())
    named = [p for p in ports if p.product and USB_PRODUCT in p.product]
    teensy = [
        p
        for p in ports
        if p.vid == TEENSY_VID and p.pid == TEENSY_SERIAL_PID and p not in named
    ]
    return named + teensy


def list_arena_ports() -> list[str]:
    """Every candidate device: ports with the G6_Arena product string first, then any
    other Teensy USB-serial device. More than one entry means the choice is ambiguous."""
    return [p.device for p in _candidates()]


def find_arena_port() -> str | None:
    """The one arena port; None when there is none; raises AmbiguousArenaPort for
    several. A named G6_Arena port does not win over an unnamed Teensy: two
    controllers on one machine must be told apart explicitly."""
    ports = _candidates()
    if len(ports) > 1:
        raise AmbiguousArenaPort(ports)
    return ports[0].device if ports else None


def describe_usb_ports() -> str:
    seen = ", ".join(
        f"{p.device} ({p.vid:04X}:{p.pid:04X})" for p in list_ports.comports() if p.vid
    )
    return seen or "none"
