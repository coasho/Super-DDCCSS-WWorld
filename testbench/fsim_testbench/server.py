"""The UI's server: the page and its files, and a small JSON API over the runner. Standard library only; it listens on
127.0.0.1 alone."""
import http.server
import json
import math
import mimetypes
import os
import threading
import time

from . import registry
from .runner import CLOCKS, SKIES


def runner_clocks():
    return CLOCKS.keys()

UI = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ui")


def _clean(x):
    """JSON has no NaN or infinity: None in their place."""
    if isinstance(x, float):
        return x if math.isfinite(x) else None
    if isinstance(x, dict):
        return {k: _clean(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_clean(v) for v in x]
    return x


class Server(http.server.ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, runner, port, app):
        super().__init__(("127.0.0.1", port), Handler)
        self.runner = runner
        self.app = app
        self.last_poll = None  # when the page last asked for the status (the window closed: it stops asking)


class Handler(http.server.BaseHTTPRequestHandler):
    server_version = "fsim-testbench"

    def log_message(self, fmt, *args):  # quiet: the console is the user's
        pass

    def _json(self, value, code=200):
        body = json.dumps(_clean(value), allow_nan=False, default=str).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/api/catalog":
            c = registry.catalog()
            c["speeds"] = list(self.server.app.speeds)
            c["clocks"] = list(runner_clocks())
            c["skies"] = list(SKIES.keys())
            c["viewer"] = self.server.app.viewer_available()
            return self._json(c)
        if path == "/api/status":
            self.server.last_poll = time.monotonic()
            with self.server.runner.lock:
                snap = dict(self.server.runner.snapshot)
            snap["viewer_running"] = self.server.app.viewer_running()
            return self._json(snap)
        name = "index.html" if path in ("/", "/index.html") else path.lstrip("/")
        full = os.path.normpath(os.path.join(UI, name))
        if not full.startswith(UI) or not os.path.isfile(full):
            return self._json({"error": "not found"}, 404)
        with open(full, "rb") as f:
            body = f.read()
        self.send_response(200)
        self.send_header("Content-Type", (mimetypes.guess_type(full)[0] or "application/octet-stream") + ("; charset=utf-8" if full.endswith((".html", ".js", ".css")) else ""))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        n = int(self.headers.get("Content-Length") or 0)
        try:
            body = json.loads(self.rfile.read(n) or b"{}")
        except ValueError:
            return self._json({"error": "not JSON"}, 400)
        path = self.path.split("?", 1)[0]
        r = self.server.runner
        if path == "/api/run":  # {id, options, add}: a new session, or (add) one more run in this one
            if body.get("id") not in registry.REGISTRY:
                return self._json({"error": "unknown scenario"}, 400)
            r.submit("run", body["id"], body.get("options") or {}, bool(body.get("add")))
        elif path == "/api/stop":  # the session: every run, every aircraft
            r.submit("stop")
        elif path == "/api/stop_run":  # {run}: one run and its aircraft
            r.submit("stop_run", int(body.get("run", -1)))
        elif path == "/api/focus":  # {name}: the viewer turns to that aircraft
            r.submit("focus", str(body.get("name", "")))
        elif path == "/api/pause":
            r.submit("pause", bool(body.get("paused")))
        elif path == "/api/clock":  # {clock}: the hour the session's world is lit for (runner.CLOCKS)
            r.submit("clock", str(body.get("clock", "midday")))
        elif path == "/api/sky":  # {sky}: the session world's cloud (runner.SKIES)
            r.submit("sky", str(body.get("sky", "scattered")))
        elif path == "/api/speed":
            r.submit("speed", float(body.get("speed", 1.0)))
        elif path == "/api/action":  # {run, index}
            r.submit("action", int(body.get("run", -1)), int(body.get("index", -1)))
        elif path == "/api/viewer":
            self.server.app.start_viewer()
        else:
            return self._json({"error": "not found"}, 404)
        return self._json({"ok": True})


def serve(runner, app, port=0):
    s = Server(runner, port, app)
    threading.Thread(target=s.serve_forever, name="testbench-http", daemon=True).start()
    return s
