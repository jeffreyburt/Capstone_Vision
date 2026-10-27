"""Generate print-ready marker pages (one marker per page) as a PDF plus individual PNGs.

    python tools/generate_markers.py                  # all robot markers from config.yaml
    python tools/generate_markers.py --ids 0-7 --size-cm 20
    python tools/generate_markers.py --ids 40-45 --out markers/floor

Print the PDF at 100% / "actual size" (NOT "fit to page"), then MEASURE the black square and put
that value in config.yaml as markers.size_m. Mount on stiff, matte board, keeping the white border.
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2
import numpy as np

from vision.config import load_config
from vision.markers import get_dictionary

PAGES = {"letter": (8.5, 11.0), "a4": (8.27, 11.69)}  # inches


def parse_ids(text):
    ids = []
    for part in text.split(","):
        if "-" in part:
            a, b = part.split("-")
            ids.extend(range(int(a), int(b) + 1))
        else:
            ids.append(int(part))
    return ids


def describe(cfg, mid):
    for r in cfg.get("robots", []):
        faces = {int(k): v for k, v in r["faces"].items()}
        if mid in faces:
            name = {0: "front", 90: "left", 180: "back", 270: "right"}.get(int(faces[mid]) % 360, f"{faces[mid]}deg")
            return f"robot {r['id']} - {name}"
    return ""


def make_page(dictionary, mid, size_cm, dpi, page, label):
    pw, ph = (int(v * dpi) for v in PAGES[page])
    page_img = np.full((ph, pw), 255, np.uint8)
    side = int(round(size_cm / 2.54 * dpi))
    if side > pw - 2:
        raise SystemExit(f"{size_cm} cm marker does not fit on a {page} page")
    marker = cv2.aruco.generateImageMarker(dictionary, mid, side)
    x0, y0 = (pw - side) // 2, int(1.2 * dpi)
    page_img[y0:y0 + side, x0:x0 + side] = marker

    font, s = cv2.FONT_HERSHEY_SIMPLEX, dpi / 150
    ty = y0 + side + int(0.6 * dpi)
    cv2.putText(page_img, f"ID {mid}   {label}", (x0, ty), font, s, 0, max(1, int(s * 2)))
    cv2.putText(page_img, f"black square = {size_cm:.1f} cm  (print at 100%, measure, update config)",
                (x0, ty + int(0.35 * dpi)), font, s * 0.6, 0, max(1, int(s)))
    # 10 cm ruler to check the printer scale.
    ry, ten_cm = ty + int(0.8 * dpi), int(round(10 / 2.54 * dpi))
    cv2.line(page_img, (x0, ry), (x0 + ten_cm, ry), 0, max(2, dpi // 100))
    for x in (x0, x0 + ten_cm):
        cv2.line(page_img, (x, ry - dpi // 10), (x, ry + dpi // 10), 0, max(2, dpi // 100))
    cv2.putText(page_img, "10 cm", (x0 + ten_cm + dpi // 10, ry + dpi // 20), font, s * 0.6, 0, max(1, int(s)))
    cells = dictionary.markerSize + 2
    if (pw - side) / 2 < side / cells:
        print(f"  note: side margins are narrower than one marker cell; keep a white border when mounting")
    return page_img


def main():
    cfg = load_config()
    m = cfg.get("markers", {})
    default_ids = ",".join(str(i) for r in cfg.get("robots", []) for i in r["faces"])
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ids", default=default_ids, help="e.g. 0-7 or 0,1,5 (default: all robot faces in config)")
    ap.add_argument("--size-cm", type=float, default=m.get("size_m", 0.15) * 100)
    ap.add_argument("--dictionary", default=m.get("dictionary", "DICT_4X4_50"))
    ap.add_argument("--page", choices=PAGES, default="letter")
    ap.add_argument("--dpi", type=int, default=300)
    ap.add_argument("--out", default="markers")
    args = ap.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    d = get_dictionary(args.dictionary)
    pages = []
    for mid in parse_ids(args.ids):
        img = make_page(d, mid, args.size_cm, args.dpi, args.page, describe(cfg, mid))
        cv2.imwrite(str(out / f"marker_{mid:02d}.png"), img)
        pages.append(img)
        print(f"marker {mid:2d}  {describe(cfg, mid)}")

    try:
        from PIL import Image
        imgs = [Image.fromarray(p) for p in pages]
        pdf = out / f"markers_{args.dictionary}_{args.size_cm:.0f}cm.pdf"
        imgs[0].save(pdf, save_all=True, append_images=imgs[1:], resolution=args.dpi)
        print(f"Wrote {pdf} (print at 100% scale)")
    except ImportError:
        print("Pillow not installed: only PNGs written (pip install pillow for a PDF)")


if __name__ == "__main__":
    main()
