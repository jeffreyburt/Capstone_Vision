"""Camera sender: runs on a Pi, publishes JPEG frames + metadata for the laptop to subscribe to.

    python -m camstream.sender birdseye          # master Pi (settings from config.yaml streams.birdseye)
    python -m camstream.sender robot1            # robot 1 Pi
    python -m camstream.sender robot1 --source test   # no camera: synthetic test pattern

Use from other Python code (e.g. the robot's control program attaching pan/tilt angles to each frame):

    from camstream.sender import CameraSender
    sender = CameraSender.from_config(load_config(), "robot1")
    sender.start()                              # runs in a background thread
    sender.update_meta(pan_deg=12.5, tilt_deg=-3.0)   # attached to every frame from now on
"""

import argparse
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2
import zmq

from camstream.protocol import VERSION, pack
from vision.camera import open_source
from vision.config import load_config


def stream_camera_config(cfg, name):
    """Config copy whose `camera` section is the base camera settings + this stream's overrides."""
    stream = cfg["streams"][name]
    camera = {**cfg.get("camera", {}), **stream.get("camera", {})}
    return {**cfg, "camera": camera}


class CameraSender:
    def __init__(self, name, port, source, quality=80, max_fps=None, bind="*"):
        self.name, self.source = name, source
        self.quality = int(quality)
        self.min_period = 1.0 / max_fps if max_fps else 0.0
        self.ctx = zmq.Context.instance()
        self.sock = self.ctx.socket(zmq.PUB)
        self.sock.setsockopt(zmq.SNDHWM, 2)
        self.sock.setsockopt(zmq.CONFLATE, 1)  # keep only the newest frame per subscriber: no backlog
        self.sock.setsockopt(zmq.LINGER, 0)
        self.sock.bind(f"tcp://{bind}:{port}")
        self.port = port
        self._meta, self._meta_lock = {}, threading.Lock()
        self._running = False
        self._thread = None
        self.stats = {"frames": 0, "bytes": 0, "encode_s": 0.0}

    @classmethod
    def from_config(cls, cfg, name, source=None):
        s = cfg["streams"][name]
        cam_cfg = stream_camera_config(cfg, name)
        src = open_source(cam_cfg, source if source is not None else s.get("source", "picam"))
        if hasattr(src, "label"):
            src.label = name
        if hasattr(src, "realtime"):  # video files: play like a live camera, looping
            src.realtime, src.loop = True, True
        return cls(name, s["port"], src, s.get("quality", 80), s.get("max_fps"))

    def update_meta(self, **kwargs):
        """Set metadata fields attached to every following frame (thread-safe)."""
        with self._meta_lock:
            self._meta.update(kwargs)

    def send_frame(self, frame, t_capture, seq):
        t0 = time.perf_counter()
        ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        self.stats["encode_s"] += time.perf_counter() - t0
        if not ok:
            return
        with self._meta_lock:
            meta = dict(self._meta)
        header = {"v": VERSION, "camera": self.name, "seq": seq, "t_capture": t_capture, "t_sent": time.time(),
                  "width": frame.shape[1], "height": frame.shape[0], "encoding": "jpeg", "meta": meta}
        msg = pack(header, jpg.tobytes())
        try:
            self.sock.send(msg, zmq.NOBLOCK)
        except zmq.Again:
            pass  # nobody connected / not ready: drop the frame
        self.stats["frames"] += 1
        self.stats["bytes"] += len(msg)

    def run(self, print_every=5.0):
        """Capture and send until stop() (blocking)."""
        self._running = True
        seq, last_sent, last_print = 0, 0.0, time.time()
        base = dict(self.stats)
        while self._running:
            got = self.source.read()
            if got is None:
                break
            frame, t_capture = got
            # Skip frames only when the camera runs clearly faster than max_fps. The 0.8 tolerance stops
            # normal timing jitter from dropping every other frame when the camera rate == max_fps.
            if t_capture - last_sent < 0.8 * self.min_period:
                continue
            last_sent = t_capture
            self.send_frame(frame, t_capture, seq)
            seq += 1
            now = time.time()
            if print_every and now - last_print >= print_every:
                dt = now - last_print
                n = self.stats["frames"] - base["frames"]
                mbps = (self.stats["bytes"] - base["bytes"]) * 8 / dt / 1e6
                enc = (self.stats["encode_s"] - base["encode_s"]) / max(n, 1) * 1000
                print(f"[{self.name}] {n / dt:.1f} fps, {mbps:.2f} Mbps, "
                      f"{(self.stats['bytes'] - base['bytes']) / max(n, 1) / 1000:.0f} kB/frame, encode {enc:.1f} ms")
                last_print, base = now, dict(self.stats)

    def start(self):
        self._thread = threading.Thread(target=self.run, daemon=True)
        self._thread.start()
        return self

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3)
        self.source.close()
        self.sock.close()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("name", help="stream name from config.yaml `streams:` (birdseye, robot1, robot2)")
    ap.add_argument("--config", help="path to config.yaml")
    ap.add_argument("--source", help="override the camera source (picam | test | webcam | video file)")
    args = ap.parse_args(argv)

    cfg = load_config(args.config)
    if args.name not in cfg.get("streams", {}):
        ap.error(f"unknown stream {args.name!r}; choose from {list(cfg.get('streams', {}))}")
    sender = CameraSender.from_config(cfg, args.name, args.source)
    print(f"Streaming '{args.name}' on tcp port {sender.port}. Ctrl+C to stop.")
    try:
        sender.run()
    except KeyboardInterrupt:
        pass
    finally:
        sender.stop()


if __name__ == "__main__":
    main()
