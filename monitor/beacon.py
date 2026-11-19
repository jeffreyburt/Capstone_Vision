"""Status heartbeats from the laptop programs (vision.main, control.main) to the system GUI.

Each program sends one small JSON packet about 5 times a second to 127.0.0.1:<system.status_port> (5007).
Nothing has to be listening: when the system GUI isn't running the packets are simply dropped.

    {"source": "vision", "t": <laptop time.time()>, "pid": 1234, "started": ..., ...program fields}

The packets come from a background thread, so a program that is blocked (e.g. vision.main waiting for the
first camera frame) still reports that it is alive.
"""

import json
import os
import socket
import threading
import time

DEFAULT_PORT = 5007


def status_port(cfg):
    return int((cfg.get("system") or {}).get("status_port", DEFAULT_PORT))


def _jsonable(o):
    """json.dumps fallback for numpy scalars and arrays."""
    if hasattr(o, "tolist"):
        return o.tolist()
    raise TypeError(f"{type(o).__name__} is not JSON serializable")


class StatusBeacon:
    """Sends {"source": name, **fields, **extra()} every period_s until close()."""

    def __init__(self, source, port=DEFAULT_PORT, host="127.0.0.1", period_s=0.2, extra=None):
        self.source, self.addr, self.period_s = source, (host, int(port)), period_s
        self.extra = extra          # optional callable returning live fields, evaluated at every send
        self.fields = {}
        self.started = time.time()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def update(self, fields):
        """Replace the reported fields (pass a new dict each time; it isn't copied)."""
        self.fields = fields

    def _packet(self):
        fields = dict(self.fields)
        if self.extra is not None:
            try:
                fields.update(self.extra())
            except Exception as e:  # noqa: BLE001 (a status report must never crash the program)
                fields["extra_error"] = repr(e)
        return {**fields, "source": self.source, "t": time.time(), "pid": os.getpid(), "started": self.started}

    def send_now(self):
        try:
            self.sock.sendto(json.dumps(self._packet(), default=_jsonable).encode(), self.addr)
        except (OSError, TypeError, ValueError):
            pass  # nobody listening, or a field that isn't JSON: skip this beat

    def _loop(self):
        while not self._stop.wait(self.period_s):
            self.send_now()

    def close(self):
        """Stop beating and tell the GUI this program has exited."""
        self._stop.set()
        self._thread.join(timeout=1)
        self.fields = {**self.fields, "state": "exited"}
        self.extra = None
        self.send_now()
        self.sock.close()


class StatusListener:
    """System GUI side: keeps the newest heartbeat from each source."""

    def __init__(self, port=DEFAULT_PORT, host="127.0.0.1"):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, int(port)))
        self.port = self.sock.getsockname()[1]
        self.sock.settimeout(0.5)
        self.latest = {}  # source -> (message, laptop time it arrived)
        self._lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while self._running:
            try:
                data, _ = self.sock.recvfrom(65536)
            except (socket.timeout, ConnectionResetError):
                continue
            except OSError:
                break  # socket closed
            try:
                msg = json.loads(data)
                source = str(msg["source"])
            except (ValueError, KeyError, TypeError):
                continue
            with self._lock:
                self.latest[source] = (msg, time.time())

    def get(self, source):
        """(message, seconds since it arrived), or (None, None) if nothing has been heard."""
        with self._lock:
            got = self.latest.get(source)
        if got is None:
            return None, None
        return got[0], time.time() - got[1]

    def close(self):
        self._running = False
        self.sock.close()
        self._thread.join(timeout=1)
