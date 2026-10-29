"""Laptop side: subscribe to camera streams and always hand out the newest frame.

    from vision.config import load_config
    from camstream.receiver import FrameReceiver

    rx = FrameReceiver(load_config(), ["robot1", "robot2"])
    f = rx.get_latest("robot1")          # None until the first frame arrives
    if f is not None:
        f.image                          # BGR numpy array (decoded on first access)
        f.seq, f.t_capture, f.meta       # e.g. f.meta.get("pan_deg")
        f.age                            # seconds since capture (needs synced clocks, see docs/NETWORK.md)
    rx.stats("robot1")                   # fps, Mbps, dropped frames, latency
    rx.close()

Frames that arrive while nobody is reading are simply replaced by newer ones, so a slow consumer never
falls behind.
"""

import os
import threading
import time
from collections import deque

import cv2
import numpy as np
import zmq

from camstream.protocol import unpack


class Frame:
    def __init__(self, name, header, payload, t_received):
        self.name, self.header, self.payload, self.t_received = name, header, payload, t_received
        self._image = None

    seq = property(lambda self: self.header["seq"])
    t_capture = property(lambda self: self.header["t_capture"])
    meta = property(lambda self: self.header.get("meta", {}))

    @property
    def image(self):
        if self._image is None:
            self._image = cv2.imdecode(np.frombuffer(self.payload, np.uint8), cv2.IMREAD_COLOR)
        return self._image

    @property
    def age(self):
        return time.time() - self.t_capture


class FrameReceiver:
    def __init__(self, cfg, names=None, host=None):
        """names: streams to subscribe to (default: all in config). host: override every stream's host
        (e.g. "127.0.0.1" when testing senders on the same machine). The CAMSTREAM_HOST environment
        variable does the same for every program (vision.main, viewer, tools)."""
        host = host or os.environ.get("CAMSTREAM_HOST")
        all_streams = cfg.get("streams", {})
        names = list(names or all_streams)
        unknown = [n for n in names if n not in all_streams]
        if unknown:
            raise ValueError(f"unknown stream(s) {unknown}; config has {list(all_streams)}")
        self.streams = {n: dict(all_streams[n], **({"host": host} if host else {})) for n in names}

        self.ctx = zmq.Context.instance()
        self.poller = zmq.Poller()
        self.sockets = {}
        for n, s in self.streams.items():
            sock = self.ctx.socket(zmq.SUB)
            sock.setsockopt(zmq.RCVHWM, 2)
            sock.setsockopt(zmq.CONFLATE, 1)
            sock.setsockopt(zmq.LINGER, 0)
            sock.setsockopt(zmq.SUBSCRIBE, b"")
            sock.connect(f"tcp://{s['host']}:{s['port']}")
            self.poller.register(sock, zmq.POLLIN)
            self.sockets[sock] = n

        self._cond = threading.Condition()
        self._latest = {n: None for n in names}
        self._history = {n: deque(maxlen=60) for n in names}  # (t_received, bytes, seq, t_capture)
        self._dropped = {n: 0 for n in names}
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while self._running:
            for sock, _ in self.poller.poll(200):
                name = self.sockets[sock]
                try:
                    msg = sock.recv(zmq.NOBLOCK)
                    header, payload = unpack(msg)
                except (zmq.Again, ValueError):
                    continue
                now = time.time()
                with self._cond:
                    prev = self._latest[name]
                    if prev is not None and header["seq"] > prev.seq + 1:
                        self._dropped[name] += header["seq"] - prev.seq - 1
                    self._latest[name] = Frame(name, header, payload, now)
                    self._history[name].append((now, len(msg), header["seq"], header["t_capture"]))
                    self._cond.notify_all()

    def get_latest(self, name):
        """Newest frame of a stream, or None if nothing has arrived yet."""
        with self._cond:
            return self._latest[name]

    def wait_new(self, name, last_seq=None, timeout=1.0):
        """Block until a frame newer than last_seq arrives (or timeout -> None).

        If the sender restarted (its seq went back to 0), the new frames still count as new."""
        def is_new():
            f = self._latest[name]
            return f is not None and (last_seq is None or f.seq != last_seq)

        with self._cond:
            if not self._cond.wait_for(is_new, timeout):
                return None
            return self._latest[name]

    def stats(self, name):
        """fps / Mbps over the last ~60 frames, dropped frames, and latency (valid only with synced clocks)."""
        with self._cond:
            h = list(self._history[name])
            dropped = self._dropped[name]
            latest = self._latest[name]
        out = {"fps": 0.0, "mbps": 0.0, "kb_per_frame": 0.0, "dropped": dropped,
               "latency_ms": None, "since_last_s": None}
        if latest is not None:
            out["since_last_s"] = time.time() - latest.t_received
            out["latency_ms"] = (latest.t_received - latest.t_capture) * 1000
        if len(h) >= 2:
            span = h[-1][0] - h[0][0]
            if span > 0:
                out["fps"] = (len(h) - 1) / span
                out["mbps"] = sum(b for _, b, _, _ in h[1:]) * 8 / span / 1e6
            out["kb_per_frame"] = sum(b for _, b, _, _ in h) / len(h) / 1000
        return out

    def close(self):
        self._running = False
        self._thread.join(timeout=2)
        for sock in self.sockets:
            sock.close()
