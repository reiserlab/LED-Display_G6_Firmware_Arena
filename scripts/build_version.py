"""PlatformIO `pre:` extra script — embed the build's git identity as -D macros.

Every controller build gets its short git SHA, branch, working-tree dirty
flag, and UTC build date compiled in (src/Version.h), so a controller can be
pinned to a build after the fact: GET_FIRMWARE_VERSION (0xCB) reports them,
the DEBUG_SERIAL boot banner prints them, and the Studio writes them into
every run log (`run_metadata.firmware`). Motivated by firmware issue #50 (a
wedge on three rigs that could not be tied to a build).

Never fails the build: when git is missing, or the source tree is not a git
checkout (e.g. a source tarball), every git-derived value falls back to
"unknown" and the date is still stamped. The same fallbacks exist as #ifndef
defaults in src/Version.h for builds that bypass this script entirely.

PlatformIO re-runs pre: scripts on every `pio run`, so a build after a commit
picks up the new SHA automatically. Because the values are CPPDEFINES, a
change in any of them (new commit, new day, dirty<->clean) changes the compile
command line and SCons rebuilds every object in the env — expect one full
rebuild after each commit, not an incremental one.
"""

Import("env")

import subprocess
from datetime import datetime, timezone

UNKNOWN = "unknown"
GIT_TIMEOUT_S = 10


def _git(*args):
    """Run a git command in the project dir; return stripped stdout or None."""
    try:
        proc = subprocess.run(
            ["git", *args],
            cwd=env.subst("$PROJECT_DIR"),
            capture_output=True,
            text=True,
            timeout=GIT_TIMEOUT_S,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    if proc.returncode != 0:
        return None
    return proc.stdout.strip()


def _ascii_safe(s, max_len):
    """Keep the macro a plain ASCII string literal: no quotes, backslashes,
    control chars, or whitespace (git ref names never contain the latter, but
    the value ends up on a shell command line, so be strict)."""
    out = []
    for ch in s:
        if ch in '"\\' or not (0x21 <= ord(ch) <= 0x7E):
            ch = "_"
        out.append(ch)
    return "".join(out)[:max_len] or UNKNOWN


def _sha():
    sha = _git("rev-parse", "--short=8", "HEAD")
    if not sha:
        return UNKNOWN
    sha = sha.lower()
    if len(sha) < 7 or any(c not in "0123456789abcdef" for c in sha):
        return UNKNOWN
    return sha[:8]  # --short=8 may return more when 8 is ambiguous


def _branch():
    ref = _git("rev-parse", "--abbrev-ref", "HEAD")
    if not ref:
        return UNKNOWN
    if ref == "HEAD":
        return "detached"
    return _ascii_safe(ref, 64)  # firmware truncates to its 24-char field


def _dirty():
    # Tracked modifications only; untracked files (stray logs, captures) do
    # not make a build "dirty". Unknown status (no git) -> treat as clean.
    status = _git("status", "--porcelain", "--untracked-files=no")
    return 1 if status else 0


def _date():
    return datetime.now(timezone.utc).strftime("%Y-%m-%d")


sha, branch, dirty, date = _sha(), _branch(), _dirty(), _date()

env.Append(
    CPPDEFINES=[
        ("FW_GIT_SHA", env.StringifyMacro(sha)),
        ("FW_GIT_BRANCH", env.StringifyMacro(branch)),
        ("FW_BUILD_DATE", env.StringifyMacro(date)),
        ("FW_GIT_DIRTY", dirty),
    ]
)

print(
    "build_version: FW_GIT_SHA=%s FW_GIT_BRANCH=%s FW_BUILD_DATE=%s FW_GIT_DIRTY=%d"
    % (sha, branch, date, dirty)
)
