"""Hex-dump the decoded SPI bytes from the most-recent AD3 capture.

Loads the newest `*.npz` under `debug/` (written by `spi_capture.py`) and
writes both directions of the captured SPI transaction as a hex dump to a
timestamped text file. MOSI is the controller -> panel command stream;
MISO is the panel's CIPO confirmation slot + idle bytes.

    pixi run -e debugad3 dump-spi
"""

from __future__ import annotations

import datetime as dt
import sys
from pathlib import Path

import numpy as np

DEBUG_DIR = Path(__file__).resolve().parent
LOG_DIR = DEBUG_DIR.parent / "log"


def find_latest_capture() -> Path:
    candidates = sorted(DEBUG_DIR.glob("spi_capture*.npz"),
                        key=lambda p: p.stat().st_mtime, reverse=True)
    if not candidates:
        sys.exit(f"No spi_capture*.npz under {DEBUG_DIR} — run "
                 "`pixi run -e debugad3 spi-capture` first.")
    return candidates[0]


def format_hexdump(label: str, payload: np.ndarray, width: int = 16) -> str:
    """Format `payload` as `00000000: XX XX ...  | ascii` lines under a header."""
    lines = [f"[{label}] {payload.size} bytes"]
    if payload.size == 0:
        lines.append("  (empty)")
        return "\n".join(lines)
    for offset in range(0, payload.size, width):
        chunk = payload[offset:offset + width]
        hex_part = " ".join(f"{b:02X}" for b in chunk)
        hex_part = f"{hex_part:<{width * 3 - 1}}"
        ascii_part = "".join(chr(b) if 0x20 <= b < 0x7F else "." for b in chunk)
        lines.append(f"  {offset:08X}: {hex_part}  |{ascii_part}|")
    return "\n".join(lines)


def main() -> int:
    src = find_latest_capture()
    data = np.load(src)
    mosi = np.asarray(data["mosi"] if "mosi" in data.files else [], dtype=np.uint8)
    miso = np.asarray(data["miso"] if "miso" in data.files else [], dtype=np.uint8)
    sample_rate = float(data["sample_rate"]) if "sample_rate" in data.files else float("nan")
    samples = data["samples"] if "samples" in data.files else None

    timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    out_path = LOG_DIR / f"spi_dump_{timestamp}.txt"

    header = [
        "# G6 SPI capture dump",
        f"# Generated:   {dt.datetime.now().isoformat(timespec='seconds')}",
        f"# Source:      {src.name}",
        f"# Sample rate: {sample_rate / 1e6:.3f} MS/s",
        f"# Window:      {samples.size if samples is not None else 0} samples"
        + (f" ({samples.size / sample_rate * 1e6:.2f} us)"
           if samples is not None and sample_rate else ""),
        "",
    ]
    body = [
        format_hexdump("MOSI (controller -> panel)", mosi),
        "",
        format_hexdump("MISO (panel -> controller)", miso),
        "",
    ]
    out_path.write_text("\n".join(header + body))

    print(f"Wrote {out_path}")
    print(f"  MOSI: {mosi.size} bytes, MISO: {miso.size} bytes")
    return 0


if __name__ == "__main__":
    sys.exit(main())
