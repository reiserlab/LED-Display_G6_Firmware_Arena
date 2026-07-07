"""Static regression test for wrap-unsafe millis()/micros() comparisons,
PR #27 review point 9.

Source: reiserlab/LED-Display_G6_Firmware_Arena#27, review comment
pullrequestreview-4632788440. Finding and the fix implemented here:
debug/pr27-response.md, point 9 ("Wrap-unsafe millis() comparisons").

`millis() < deadline` / `millis() > stall_deadline` (SerialManager.cpp:194
and :223, before this fix) misfire at the ~49.7-day millis() wrap: once
millis() wraps around to a small value while `deadline` still holds a
value from before the wrap, a raw comparison of the two absolute
timestamps gives the wrong answer. The fix rewrites both as the wrap-safe
signed-difference idiom already used elsewhere in this codebase, e.g.
`(int32_t)(millis() - deadline) >= 0`, which stays correct across the wrap
because it's the SUBTRACTION, evaluated in unsigned arithmetic, that
naturally wraps correctly, not a comparison of the two raw operands.

This can't be exercised as a HIL behavior test: the wrap itself is 49.7
days away on real hardware, nothing a test can wait for. Instead, this
statically scans the firmware source for the banned pattern. A direct
millis()/micros() call immediately followed by a comparison operator
(rather than a subtraction) is exactly the unsafe shape, so its absence is
the regression signal.

Run (works without a controller attached; doesn't use the `transport`
fixture, so it isn't gated on serial/tcp at all):
    python -m pytest tests/test_pr27_wrap_safe_millis.py -v
    pixi run test-serial -- -k pr27_wrap
"""

import re
from pathlib import Path

SRC_DIR = Path(__file__).parent.parent / "src"

# millis()/micros() directly followed (only whitespace between) by a
# comparison operator is the unsafe shape: it compares two absolute
# timestamps directly. The safe idiom subtracts first, e.g.
# `(millis() - deadline) >= 0`, so millis() is followed by ` - ...`, not a
# comparison operator, and doesn't match this pattern.
_UNSAFE_PATTERN = re.compile(r"\b(?:millis|micros)\(\)\s*[<>]")


def _code_only(line: str) -> str:
    """Strip a trailing `//` line comment (this codebase's only comment
    style), so a match inside an explanatory comment isn't mistaken for
    the real thing."""
    idx = line.find("//")
    return line if idx == -1 else line[:idx]


def test_no_wrap_unsafe_millis_comparisons_in_firmware_source():
    """No .cpp/.h file under src/ may compare millis()/micros() directly
    against a stored deadline; every such check must subtract first."""
    offenders = []
    for path in sorted(SRC_DIR.glob("*.cpp")) + sorted(SRC_DIR.glob("*.h")):
        text = path.read_text(encoding="utf-8", errors="replace")
        for lineno, line in enumerate(text.splitlines(), start=1):
            if _UNSAFE_PATTERN.search(_code_only(line)):
                offenders.append(f"{path.name}:{lineno}: {line.strip()}")

    assert not offenders, (
        "found wrap-unsafe millis()/micros() comparison(s); use the "
        "signed-difference idiom instead, e.g. "
        "(int32_t)(millis() - deadline) >= 0:\n" + "\n".join(offenders)
    )
