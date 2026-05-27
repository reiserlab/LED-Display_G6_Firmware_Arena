"""Plot the most-recent AD3 SPI capture as a 4-trace digital waveform.

Loads the newest `*.npz` under `debug/` (matches what `spi_capture.py` writes),
draws SCK / MOSI / MISO / CS as stacked step traces with the CS-low region
shaded, and annotates decoded byte boundaries when bytes were captured.

    pixi run -e debugad3 plot-recent
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

DEBUG_DIR = Path(__file__).resolve().parent

# Channel order in the stacked plot (top -> bottom).
CHANNEL_ORDER = [
    ("SCK",  "bit_sck",  0,    "tab:blue"),
    ("MOSI", "bit_mosi", 1,    "tab:orange"),
    ("MISO", "bit_miso", 2,    "tab:green"),
    ("CS",   "bit_cs",   3,    "tab:red"),
]


def find_latest_capture() -> Path:
    candidates = sorted(DEBUG_DIR.glob("*.npz"),
                        key=lambda p: p.stat().st_mtime, reverse=True)
    if not candidates:
        sys.exit(f"No .npz files in {DEBUG_DIR} — run `pixi run -e debugad3 "
                 "spi-capture` first.")
    return candidates[0]


def _bit(samples: np.ndarray, position: int) -> np.ndarray:
    return ((samples >> position) & 1).astype(np.uint8)


def main() -> int:
    path = find_latest_capture()
    print(f"Plotting {path}")

    data = np.load(path)
    samples = np.asarray(data["samples"], dtype=np.uint16)
    mosi_bytes = data["mosi"] if "mosi" in data.files else np.zeros(0, np.uint8)
    miso_bytes = data["miso"] if "miso" in data.files else np.zeros(0, np.uint8)
    sample_rate = float(data["sample_rate"]) if "sample_rate" in data.files else 125e6

    # Resolve per-channel bit positions from the npz when present; otherwise
    # fall back to the spi_capture.py defaults.
    bit_positions = {
        key: int(data[key]) if key in data.files else default_bit
        for _, key, default_bit, _ in CHANNEL_ORDER
    }

    t_us = np.arange(samples.size) / sample_rate * 1e6
    fig, ax = plt.subplots(figsize=(12, 5))

    # Stack the 4 traces with vertical offsets so they don't overlap.
    spacing = 1.5
    for row, (name, key, _, color) in enumerate(CHANNEL_ORDER):
        offset = (len(CHANNEL_ORDER) - 1 - row) * spacing
        signal = _bit(samples, bit_positions[key]).astype(float) + offset
        ax.step(t_us, signal, where="post", linewidth=1.0, color=color, label=name)
        ax.text(t_us[0] - 1.5, offset + 0.5, name,
                ha="right", va="center", fontsize=10, color=color)

    # Shade the CS-low intervals across the full plot height.
    cs = _bit(samples, bit_positions["bit_cs"])
    low = cs == 0
    transitions = np.flatnonzero(np.diff(low.astype(np.int8)))
    edges = [0] + (transitions + 1).tolist() + [len(cs)]
    for start, end in zip(edges[:-1], edges[1:]):
        if low[start]:
            ax.axvspan(t_us[start], t_us[min(end, len(t_us) - 1)],
                       color="tab:red", alpha=0.06, lw=0)

    # Annotate decoded byte boundaries (MOSI hex on top, MISO underneath) when
    # the decoder found bytes. We can't easily recover the per-byte time offsets
    # without re-running the decoder, so just print them in the title strip.
    if mosi_bytes.size:
        head = min(mosi_bytes.size, 12)
        title_mosi = " ".join(f"{b:02X}" for b in mosi_bytes[:head])
        title_miso = " ".join(f"{b:02X}" for b in miso_bytes[:head])
        title = (f"{path.name}    "
                 f"{mosi_bytes.size} SPI bytes captured\n"
                 f"MOSI[:{head}] = {title_mosi}\n"
                 f"MISO[:{head}] = {title_miso}")
    else:
        title = f"{path.name}    (no SPI bytes decoded — see per-channel stats)"
    ax.set_title(title, fontsize=10, family="monospace", loc="left")

    ax.set_xlim(t_us[0], t_us[-1])
    ax.set_ylim(-0.5, len(CHANNEL_ORDER) * spacing)
    ax.set_yticks([])
    ax.set_xlabel("time (µs)")
    ax.grid(axis="x", alpha=0.3)
    fig.tight_layout()

    plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
