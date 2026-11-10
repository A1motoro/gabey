"""Small loopback-only HTTP host; no web framework or online assets."""

import argparse
import json
import logging
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

from pbs.demo.service import get_config, simulate

STATIC = Path(__file__).with_name("static")
ASSETS = {
    "/": ("index.html", "text/html"),
    "/app.js": ("app.js", "text/javascript"),
    "/style.css": ("style.css", "text/css"),
    "/presentation": ("presentation.html", "text/html"),
    "/presentation/": ("presentation.html", "text/html"),
    "/presentation.css": ("presentation.css", "text/css"),
    "/presentation.js": ("presentation.js", "text/javascript"),
    "/presentation-events.js": ("presentation-events.js", "text/javascript"),
}


class DemoHandler(BaseHTTPRequestHandler):
    def send_data(self, code, body, content_type):
        self.send_response(code)
        self.send_header("Content-Type", content_type + "; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(body)

    def json_response(self, code, value):
        self.send_data(code, json.dumps(value, allow_nan=False).encode(), "application/json")

    def do_GET(self):
        path = urlsplit(self.path).path
        if path == "/api/config":
            self.json_response(200, get_config())
        elif path in ASSETS:
            filename, mime = ASSETS[path]
            self.send_data(200, (STATIC / filename).read_bytes(), mime)
        else:
            self.json_response(404, {"error": "Not found."})

    def do_POST(self):
        if self.path != "/api/simulate":
            self.json_response(404, {"error": "Not found."})
            return
        # No cross-origin requests that could consume local simulation resources.
        origin = self.headers.get("Origin")
        if origin and origin != f"http://{self.headers.get('Host')}":
            self.json_response(403, {"error": "Use the local demo page to run a simulation."})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 4096:
                raise ValueError("Send a JSON parameter object of at most 4096 bytes.")
            config = json.loads(self.rfile.read(length))
            result = simulate(config)
        except (ValueError, TypeError, UnicodeError) as exc:
            self.json_response(400, {"error": str(exc)})
            return
        except Exception:
            logging.exception("Demo simulation failed")
            self.json_response(500, {"error": "Simulation failed. Check the terminal and retry."})
            return
        self.json_response(200, result)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Local 2.5D cut-in comparison demo")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args(argv)
    if not 0 <= args.port <= 65535:
        parser.error("Port must be between 0 and 65535 (0 selects a free port).")
    try:
        server = ThreadingHTTPServer(("127.0.0.1", args.port), DemoHandler)
    except OSError as exc:
        parser.exit(1, f"Cannot start demo: {exc}. Try --port {args.port + 1}.\n")
    url = f"http://127.0.0.1:{server.server_port}"
    print(f"Cut-in demo: {url}\nPress Ctrl+C to stop.", flush=True)
    if not args.no_browser:
        threading.Timer(0.5, webbrowser.open, args=(url,)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
