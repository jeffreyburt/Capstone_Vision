"""Watch camera streams on the laptop and measure the network (fps, bandwidth, dropped frames, latency).

    python -m camstream.viewer                    # all streams in config.yaml, in a window
    python -m camstream.viewer robot1 robot2      # just these
    python -m camstream.viewer --stats            # no window: print numbers every 2 s (bandwidth test)
    python -m camstream.viewer --host 127.0.0.1   # senders running on this same machine (testing)

Latency is only meaningful when the sender's and laptop's clocks are synced (docs/NETWORK.md section 6).
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2
import numpy as np

from camstream.receiver import FrameReceiver
from vision.config import load_config


def stats_line(name, s):
    lat = f"{s['latency_ms']:.0f} ms" if s["latency_ms"] is not None else "-"
    last = f"{s['since_last_s']:.1f}s ago" if s["since_last_s"] is not None else "no frames yet"
    return (f"{name:9s} {s['fps']:5.1f} fps  {s['mbps']:6.2f} Mbps  {s['kb_per_frame']:6.1f} kB/frame  "
            f"dropped {s['dropped']:4d}  latency {lat:>7s}  last {last}")


def tile(frames, names, tile_w=640):
    tiles = []
    for n in names:
        f = frames.get(n)
        if f is None or f.image is None:
            img = np.zeros((tile_w * 9 // 16, tile_w, 3), np.uint8)
            cv2.putText(img, f"{n}: waiting...", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (200, 200, 200), 2)
        else:
            img = f.image
            img = cv2.resize(img, (tile_w, int(img.shape[0] * tile_w / img.shape[1])))
            meta = " ".join(f"{k}={v}" for k, v in f.meta.items())
            cv2.putText(img, f"{n} #{f.seq} {meta}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        tiles.append(img)
    h = max(t.shape[0] for t in tiles)
    tiles = [np.vstack([t, np.zeros((h - t.shape[0], t.shape[1], 3), np.uint8)]) for t in tiles]
    return np.hstack(tiles)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("names", nargs="*", help="streams to show (default: all)")
    ap.add_argument("--config")
    ap.add_argument("--host", help="override every stream's host")
    ap.add_argument("--stats", action="store_true", help="print statistics only, no window")
    ap.add_argument("--seconds", type=float, help="stop after this long")
    args = ap.parse_args(argv)

    cfg = load_config(args.config)
    rx = FrameReceiver(cfg, args.names or None, host=args.host)
    names = list(rx.streams)
    for n in names:
        print(f"Subscribing to {n} at {rx.streams[n]['host']}:{rx.streams[n]['port']}")
    start = last_print = time.time()
    try:
        while not args.seconds or time.time() - start < args.seconds:
            now = time.time()
            if now - last_print >= 2.0:
                for n in names:
                    print(stats_line(n, rx.stats(n)))
                print()
                last_print = now
            if args.stats:
                time.sleep(0.1)
                continue
            cv2.imshow("camstream", tile({n: rx.get_latest(n) for n in names}, names))
            if cv2.waitKey(30) & 0xFF in (ord("q"), 27):
                break
    except KeyboardInterrupt:
        pass
    finally:
        rx.close()
        if not args.stats:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
