"""Locate the G6 arena controller's USB-CDC serial port on Linux, macOS and Windows.

Matches a port whose USB product string contains "G6_Arena" (set by
scripts/patch_usb_strings.py; Windows' inbox CDC driver does not expose it),
then falls back to any Teensy USB-serial device (VID 16C0, PID 0483).
"""

from serial.tools import list_ports

TEENSY_VID = 0x16C0
TEENSY_SERIAL_PID = 0x0483
USB_PRODUCT = "G6_Arena"  # keep in sync with scripts/patch_usb_strings.py


def find_arena_port() -> str | None:
    ports = list(list_ports.comports())
    for p in ports:
        if p.product and USB_PRODUCT in p.product:
            return p.device
    for p in ports:
        if p.vid == TEENSY_VID and p.pid == TEENSY_SERIAL_PID:
            return p.device
    return None


def describe_usb_ports() -> str:
    seen = ", ".join(
        f"{p.device} ({p.vid:04X}:{p.pid:04X})" for p in list_ports.comports() if p.vid
    )
    return seen or "none"
