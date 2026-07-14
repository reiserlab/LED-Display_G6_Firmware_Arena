"""AD3 helpers for the PR #15 stuck-row repro tests
(``test_pr15_stuck_row_timeout.py`` — see
``Panel-Firmware/next-steps-pr-15.md`` finding #1).

Wiring (default pin assignment; a fresh dwfpy ``Device()`` session per call,
mirroring ``debug/spi_capture.py``'s style):

    AD3 DIO0  -> flywire probe on the debug-flashed panel's row-0 GPIO test
                 point (GP20, bottom row). Used by both the Oneshot and
                 Triggered tests (see test_pr15_stuck_row_timeout.py's
                 module docstring's IMPORTANT note for why it's row 0 for
                 both, not some other row).
    AD3 DIO1  -> arena external EINT input (jumper = direct-to-EINT), with a
                 ~330 ohm series resistor for protection — see
                 ``Panel-Firmware/panel/tools/ad3_trigger.py``.
    AD3 GND   -> panel GND (any panel ground pad).

Row polarity: rows are active-LOW = ON (``twopio_fail_dark()`` in
``display_scan_twopio.cpp`` drives rows HIGH for OFF). A healthy row reads
HIGH once its Oneshot/Triggered frame is done; the stuck-row bug leaves it
LOW.

``dwfpy`` is imported lazily inside each function so importing this module
(and collecting the pytest file that imports it) never requires the
WaveForms runtime / dwfpy package outside the ``debugad3`` pixi environment.
"""

from __future__ import annotations


def sample_row_pin(dio_in: int = 0, settle_s: float = 0.05) -> int:
    """Return the current digital level (0/1) of AD3 DIO<dio_in>."""
    import time

    import dwfpy as dwf

    with dwf.Device() as device:
        io = device.digital_io
        io[dio_in].enabled = False  # input (no output drive on this pin)
        io.configure()
        time.sleep(settle_s)
        io.read_status()
        return int(io[dio_in].input_state)


def sample_row_pin_debounced(dio_in: int = 0, samples: int = 7,
                              interval_s: float = 0.01) -> int:
    """Majority-vote digital level of AD3 DIO<dio_in> over `samples` reads.

    The row's per-panel command is continuously re-delivered by the arena's
    refresh timer (every ~3.3 ms at the default GS16 refresh rate — see
    CommandProcessor::transmitOnRefresh()), so on FIXED firmware a single
    sample can land during one row's legitimate ~45-50 us ON slice within an
    otherwise-healthy scan (a real, brief, expected pulse — not the stuck-row
    bug). Voting over several reads spread across multiple scans makes the
    stuck-vs-healthy read reliable without needing to stop that retransmit.
    """
    import time

    import dwfpy as dwf

    with dwf.Device() as device:
        io = device.digital_io
        io[dio_in].enabled = False
        io.configure()
        highs = 0
        for i in range(samples):
            if i:
                time.sleep(interval_s)
            io.read_status()
            highs += int(io[dio_in].input_state)
        return 1 if highs * 2 > samples else 0


def drive_eint_burst(edges: int, freq: float = 1000.0, dio_out: int = 1) -> None:
    """Drive exactly `edges` rising edges on AD3 DIO<dio_out>, then idle LOW.

    Mirrors the `burst` mode in Panel-Firmware's `panel/tools/ad3_trigger.py`
    (ctypes), reimplemented with dwfpy's high-level digital_output API.
    """
    import time

    import dwfpy as dwf

    period = 1.0 / freq
    with dwf.Device() as device:
        out = device.digital_output
        channel = out[dio_out]
        channel.setup_pulse(
            low=period / 2,
            high=period / 2,
            repetition=edges,
            initial_state="low",
            idle_state="low",
            configure=True,
            start=True,
        )
        deadline = time.time() + edges * period + 0.5
        while time.time() < deadline:
            if out.read_status() == dwf.Status.DONE:
                break
            time.sleep(0.001)
        out.reset()
