"""Run the central vision system.

Examples:
    python -m vision.main                          # Pi camera, send UDP to the host in config.yaml
    python -m vision.main --stream                 # ...plus a debug view at http://<pi-ip>:8080/
    python -m vision.main --source webcam --show   # laptop webcam with a debug window
    python -m vision.main --source clip.mp4 --show # replay a recording
    python -m vision.main --source tcp://<pi-ip>:8888 --show   # process a video stream from the Pi on a laptop
"""

import argparse
import time

import cv2

from .camera import open_source
from .config import load_config
from .output import UdpPublisher
from .pipeline import VisionPipeline


def main(argv=None):
    ap = argparse.ArgumentParser(description="Central camera vision system")
    ap.add_argument("--config", help="path to config.yaml (default: repo config.yaml)")
    ap.add_argument("--source", help="picam | webcam | camera index | video file | stream URL (overrides config)")
    ap.add_argument("--show", action="store_true", help="show a debug window (needs a display)")
    ap.add_argument("--stream", nargs="?", const=-1, type=int, metavar="PORT",
                    help="serve the debug view over HTTP (default port from config)")
    ap.add_argument("--host", help="UDP destination host (overrides config)")
    ap.add_argument("--port", type=int, help="UDP destination port (overrides config)")
    ap.add_argument("--loop", action="store_true", help="loop video files")
    ap.add_argument("--max-frames", type=int, help="stop after this many frames")
    ap.add_argument("--print", dest="print_msgs", action="store_true", help="print every published message")
    args = ap.parse_args(argv)

    cfg = load_config(args.config)
    out_cfg = cfg.get("output", {})
    dbg_cfg = cfg.get("debug", {})
    publisher = UdpPublisher(args.host or out_cfg.get("host", "127.0.0.1"), args.port or out_cfg.get("port", 5005),
                             out_cfg.get("broadcast", False))
    publish_period = 1.0 / float(out_cfg.get("rate_hz", 10))
    lost_after = cfg.get("tracking", {}).get("lost_after_s", 1.0)
    robot_size = 2 * max([r.get("center_offset_m", 0.15) for r in cfg.get("robots", [])] or [0.15])

    pipeline = VisionPipeline(cfg)
    source = open_source(cfg, args.source, loop=args.loop)

    server = None
    if args.stream is not None:
        from .debug_view import MjpegServer
        server = MjpegServer(dbg_cfg.get("stream_port", 8080) if args.stream == -1 else args.stream)
    floor_map = None

    print(f"Sending UDP to {publisher.addr[0]}:{publisher.addr[1]} at {1 / publish_period:.0f} Hz. Ctrl+C to stop.")
    last_publish = last_status = last_stream = last_snapshot = 0.0
    frames_since_status = 0
    fps = None
    try:
        while True:
            got = source.read()
            if got is None:
                print("End of stream.")
                break
            frame, ts = got
            result = pipeline.process(frame, ts)
            frames_since_status += 1
            now = time.time()

            if now - last_publish >= publish_period:
                publisher.send(result.message)
                last_publish = now
                if args.print_msgs:
                    print(result.message)

            if now - last_status >= 1.0:
                fps = frames_since_status / (now - last_status) if last_status else None
                robots = ", ".join(f"R{r['id']} ({r['x']:.2f}, {r['y']:.2f}) {r['heading'] or 0:.0f}deg age {r['age']:.1f}s"
                                   for r in result.message["robots"]) or "no robots"
                print(f"[{fps or 0:5.1f} fps] markers {[m.id for m in result.markers]}  {robots}")
                last_status, frames_since_status = now, 0

            want_stream = server is not None and now - last_stream >= 0.1  # ~10 fps is plenty for viewing
            if args.show or want_stream:
                from .debug_view import FloorMap, compose, draw_overlay
                if floor_map is None:
                    floor_map = FloorMap(dbg_cfg.get("map_x", (-3, 3)), dbg_cfg.get("map_y", (-0.5, 7)),
                                         dbg_cfg.get("map_px_per_m", 70), pipeline.geometry,
                                         (frame.shape[1], frame.shape[0]))
                view = compose(draw_overlay(frame, result, pipeline.geometry, fps),
                               floor_map.draw(result.message, robot_size, lost_after))
                if want_stream:
                    snapshot = frame if now - last_snapshot >= 2.0 else None
                    server.update(view, snapshot)
                    last_stream = now
                    if snapshot is not None:
                        last_snapshot = now
                if args.show:
                    cv2.imshow("vision", view)
                    if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                        break

            if args.max_frames and pipeline.frame_index >= args.max_frames:
                break
    except KeyboardInterrupt:
        pass
    finally:
        source.close()
        publisher.close()
        if server:
            server.close()
        if args.show:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
