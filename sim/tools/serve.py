"""Dev server for browser iteration.

`python -m http.server` sends no Cache-Control, so browsers heuristically cache
the ES modules and a reload happily re-runs yesterday's main.js while serving a
fresh index.html. That failure is silent and genuinely nasty to debug -- the
page looks updated because the HTML is, but the behaviour is stale.

This is the same static server with caching turned off.

    python tools/serve.py [port]
"""

import os
import sys
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 5273


class NoCacheHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=ROOT, **kwargs)

    def end_headers(self):
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        super().end_headers()

    def log_message(self, fmt, *args):
        # Quiet: one line per asset per reload is noise, not information.
        if "404" in (fmt % args):
            super().log_message(fmt, *args)


if __name__ == "__main__":
    print(f"SDM26 Driver-in-Loop  ->  http://localhost:{PORT}   (no-cache, serving {ROOT})")
    ThreadingHTTPServer(("127.0.0.1", PORT), NoCacheHandler).serve_forever()
