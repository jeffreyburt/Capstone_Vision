"""Lens (intrinsic) calibration with a printed checkerboard. Run once per camera at the running resolution.

1. Print the board:      python tools/calibrate_camera.py --make-board board.pdf
   Tape it to something FLAT and rigid, and measure one square (default 25 mm).
2. Capture ~20 views, filling the frame and tilting the board in different directions:
     with a display:     python tools/calibrate_camera.py --show       (SPACE = capture, C = calibrate, Q = quit)
     headless Pi:        python tools/calibrate_camera.py --auto --stream   (watch http://<pi-ip>:8080/)
     from saved images:  python tools/calibrate_camera.py --images "calib/images/*.jpg"
3. The result goes to the calibration.intrinsics path in config.yaml. RMS error under ~1 px is good.
"""

import argparse
import glob
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2
import numpy as np

from vision.camera import open_source
from vision.config import load_config, resolve_path
from vision.geometry import CameraModel


def make_board(path, cols, rows, square_mm, dpi=300):
    """Checkerboard with (cols x rows) INNER corners on a landscape letter page."""
    pw, ph = int(11 * dpi), int(8.5 * dpi)
    sq = int(round(square_mm / 25.4 * dpi))
    w, h = (cols + 1) * sq, (rows + 1) * sq
    if w > pw or h > ph:
        raise SystemExit("Board too big for the page; use smaller --square-mm")
    img = np.full((ph, pw), 255, np.uint8)
    x0, y0 = (pw - w) // 2, (ph - h) // 2
    for r in range(rows + 1):
        for c in range(cols + 1):
            if (r + c) % 2 == 0:
                img[y0 + r * sq:y0 + (r + 1) * sq, x0 + c * sq:x0 + (c + 1) * sq] = 0
    cv2.putText(img, f"{cols}x{rows} inner corners, {square_mm} mm squares (print at 100%)",
                (x0, y0 - dpi // 8), cv2.FONT_HERSHEY_SIMPLEX, dpi / 250, 0, 2)
    path = Path(path)
    if path.suffix.lower() == ".pdf":
        from PIL import Image
        Image.fromarray(img).save(path, resolution=dpi)
    else:
        cv2.imwrite(str(path), img)
    print(f"Wrote {path}")


def find_corners(gray, board):
    ok, corners = cv2.findChessboardCornersSB(gray, board, cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE)
    return corners if ok else None


def calibrate(views, board, square_m, size, out_path):
    objp = np.zeros((board[0] * board[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board[0], 0:board[1]].T.reshape(-1, 2) * square_m
    rms, K, dist, _, _ = cv2.calibrateCamera([objp] * len(views), views, size, None, None)
    hfov = 2 * np.degrees(np.arctan(size[0] / (2 * K[0, 0])))
    print(f"\nCalibrated from {len(views)} views at {size[0]}x{size[1]}: RMS error {rms:.3f} px, "
          f"fx={K[0, 0]:.1f} fy={K[1, 1]:.1f}, horizontal FOV {hfov:.1f} deg")
    if rms > 1.0:
        print("  RMS > 1 px: check the board is flat, in focus, and views are varied; consider recapturing.")
    CameraModel(K, dist, size).save(out_path, rms_px=float(rms), views=len(views))
    print(f"Saved {out_path}")


def main():
    cfg = load_config()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--board", default="9x6", help="INNER corners, cols x rows (default 9x6)")
    ap.add_argument("--square-mm", type=float, default=25.0)
    ap.add_argument("--make-board", metavar="FILE", help="write a printable board (.pdf or .png) and exit")
    ap.add_argument("--images", help="glob of saved images to calibrate from")
    ap.add_argument("--source", help="camera source (default from config)")
    ap.add_argument("--show", action="store_true", help="interactive window")
    ap.add_argument("--auto", action="store_true", help="capture automatically every --interval seconds")
    ap.add_argument("--interval", type=float, default=2.0)
    ap.add_argument("--count", type=int, default=20, help="views to collect in --auto mode")
    ap.add_argument("--stream", nargs="?", const=8080, type=int, metavar="PORT", help="preview over HTTP")
    ap.add_argument("--save-dir", default=str(resolve_path(cfg, "calib/images")))
    ap.add_argument("--out", default=str(resolve_path(cfg, cfg["calibration"]["intrinsics"])))
    args = ap.parse_args()

    board = tuple(int(v) for v in args.board.lower().split("x"))
    if args.make_board:
        make_board(args.make_board, *board, args.square_mm)
        return
    square_m = args.square_mm / 1000

    views, size = [], None
    if args.images:
        for f in sorted(glob.glob(args.images)):
            gray = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
            size = (gray.shape[1], gray.shape[0])
            c = find_corners(gray, board)
            print(f"{f}: {'ok' if c is not None else 'board not found'}")
            if c is not None:
                views.append(c)
        if len(views) < 8:
            raise SystemExit(f"Only {len(views)} usable images; need at least 8 (20 recommended)")
        calibrate(views, board, square_m, size, args.out)
        return

    if not (args.show or args.auto):
        raise SystemExit("Choose --show (interactive), --auto (hands-free) or --images")
    server = None
    if args.stream:
        from vision.debug_view import MjpegServer
        server = MjpegServer(args.stream)
    save_dir = Path(args.save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)
    source = open_source(cfg, args.source)
    last_try, message = 0.0, ""
    try:
        while True:
            got = source.read()
            if got is None:
                break
            frame = got[0]
            size = (frame.shape[1], frame.shape[0])
            key = -1
            capture = False
            if args.auto and time.time() - last_try >= args.interval:
                capture, last_try = True, time.time()

            preview = frame.copy()
            cv2.putText(preview, f"views: {len(views)}  {message}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
            if args.show:
                cv2.imshow("calibrate", cv2.resize(preview, (size[0] * 720 // size[1], 720)))
                key = cv2.waitKey(1) & 0xFF
                capture = capture or key == ord(" ")
            if server:
                server.update(cv2.resize(preview, (size[0] * 540 // size[1], 540)))

            if capture:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                c = find_corners(gray, board)
                if c is None:
                    message = "board NOT found"
                else:
                    views.append(c)
                    cv2.imwrite(str(save_dir / f"calib_{len(views):02d}.jpg"), frame)
                    message = "captured - move the board"
                print(f"views: {len(views)}  {message}")
            if key in (ord("q"), 27):
                return
            if key == ord("c") or (args.auto and len(views) >= args.count):
                break
    finally:
        source.close()
        if server:
            server.close()
        if args.show:
            cv2.destroyAllWindows()
    if len(views) < 8:
        raise SystemExit(f"Only {len(views)} views; need at least 8 (20 recommended)")
    calibrate(views, board, square_m, size, args.out)


if __name__ == "__main__":
    main()
