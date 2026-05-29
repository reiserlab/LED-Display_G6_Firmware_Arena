#!/usr/bin/env bash
# Serve the Web Serial UI and open a Chromium-based browser in parallel.
#
# Web Serial requires (a) a Chromium-based browser — Chrome / Edge / Opera /
# Brave; Firefox and Safari do not implement navigator.serial — and (b) a
# secure context. http://localhost counts as a secure context in Chromium, so
# plain HTTP on localhost is sufficient; no HTTPS/cert needed.
#
# The browser launch is backgrounded so it runs alongside the (foreground)
# HTTP server; Ctrl+C stops the server. Override the browser with
#   BROWSER=/path/to/chrome pixi run webserial
# and the port with PORT=9001.
set -euo pipefail

PORT="${PORT:-8000}"
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
URL="http://localhost:${PORT}/"

# Pick a browser: explicit $BROWSER wins, else the first Chromium-based one
# found on PATH.
browser=""
if [ -z "$browser" ]; then
  for b in chromium chromium-browser google-chrome google-chrome-stable \
           brave-browser microsoft-edge; do
    if command -v "$b" >/dev/null 2>&1; then
      browser="$(command -v "$b")"
      break
    fi
  done
fi

if [ -n "$browser" ]; then
  echo "Opening $URL in: $browser"
  # Subshell + sleep: give http.server a moment to bind before the first
  # request, then detach so it doesn't tie up this shell.
  ( sleep 1; "$browser" "$URL" >/dev/null 2>&1 & ) || true
else
  echo "No Chromium-based browser found on PATH."
  echo "Open $URL manually in Chrome / Edge / Opera (Web Serial only)."
fi

echo "Serving $DIR on $URL  (Ctrl+C to stop)"

# Serve with caching fully disabled. The stock `python -m http.server` answers
# conditional requests with 304 Not Modified, so browsers keep showing an old
# cached index.html / main.js after you edit them. This handler strips the
# conditional-request headers (so every GET re-reads the file) and sends
# Cache-Control: no-store, guaranteeing the freshest page each load.
exec python -c '
import sys, functools
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler

port = int(sys.argv[1])
directory = sys.argv[2]

class NoCache(SimpleHTTPRequestHandler):
    def send_head(self):
        for h in ("If-Modified-Since", "If-None-Match"):
            while h in self.headers:
                del self.headers[h]
        return super().send_head()

    def end_headers(self):
        self.send_header("Cache-Control", "no-store, max-age=0")
        super().end_headers()

handler = functools.partial(NoCache, directory=directory)
ThreadingHTTPServer(("", port), handler).serve_forever()
' "$PORT" "$DIR"
