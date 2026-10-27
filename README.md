# Capstone Central Vision

The central camera (Raspberry Pi 4 + Camera Module 3) finds the speaker robots and reports their
**floor position (m)** and **heading (deg)** to the controller laptop over UDP.

- **Robots:** ArUco markers (`DICT_4X4_50`) on all 4 sides of each robot, detected with OpenCV.
- **Positions:** every marker corner is projected onto its known height using a calibrated camera.
  This gives the face position and direction, and from those the robot's center and heading.
- **Person detection:** planned for a later phase. The output already has a `"user": null` field for it.

See [docs/PLAN.md](docs/PLAN.md) for the design and the reasoning behind it.

---

## 1. Try it on a laptop (no hardware needed)

```bash
py -3.12 -m venv .venv                 # Windows   (Linux/Mac: python3 -m venv .venv)
.venv\Scripts\activate                 # Windows   (Linux/Mac: source .venv/bin/activate)
pip install -r requirements.txt
python -m pytest                       # run the tests
python tools/make_sim_video.py         # render a synthetic video of 2 moving robots
python -m vision.main --config sim/config.yaml --show
```

The debug window shows detected markers on the left and a top-down map on the right. Press `q` to quit.

## 2. Raspberry Pi setup

**Step-by-step install and test guide: [docs/PI_GUIDE.md](docs/PI_GUIDE.md)**. Short version: run
`bash scripts/setup_pi.sh`, then `python tools/pi_check.py`. The manual equivalent:

Use 64-bit Raspberry Pi OS. Picamera2 comes from apt, so the virtualenv needs `--system-site-packages`:

```bash
sudo apt update && sudo apt install -y python3-picamera2 python3-venv git
git clone <this repo> && cd Capstone_Vision
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r requirements.txt
rpicam-hello -t 2000                   # check the camera works (older OS: libcamera-hello)
python -c "import picamera2, cv2; print('ok', cv2.__version__)"
```

If `import picamera2` fails with a numpy error after the pip install, the pip numpy doesn't match the
apt-built Picamera2. Run `pip uninstall numpy` inside the venv so the system numpy is used. If OpenCV then
complains, install an OpenCV built for that numpy, e.g. `pip install "opencv-python<4.12"` for numpy 1.x.

## 3. Setup workflow (do these in order)

| Step | Command | Notes |
|---|---|---|
| **Print markers** | `python tools/generate_markers.py` | Writes `markers/*.pdf`. Print at **100% / actual size**. **Measure the black square** and set `markers.size_m`. |
| **Mount markers** | — | One marker per face, **upright as printed** (label text below the marker), centered on the face, on matte, rigid board. Set `markers.center_height_m` (floor to marker center) and `center_offset_m` (marker surface to robot center). Robot 1 uses IDs 0–3 and robot 2 uses 4–7 (front, left, back, right). Change `robots:` if your faces differ. |
| **Range test** | `python tools/range_test.py --stream` | Do this early. Put a marker at 5/10/15/20 ft and check the detection rate and px/cell (aim for ≥ 4–5). Results go to `results/range_test.csv`. |
| **Lens calibration** | `python tools/calibrate_camera.py --make-board board.pdf`, then `--show` or `--auto --stream` | About 20 varied views of the flat board at the **running resolution**. RMS under 1 px is good. Writes `calib/camera_intrinsics.yaml`. |
| **Mount camera** | — | About 2 m high, tilted down so the far end of the area is near the top of the frame. **Note:** a camera 2 m high, aimed at the middle of the room, can't see anything closer than about 2 m. |
| **Floor calibration** | `python tools/record.py --snapshot calib/floor.jpg`, then `python tools/calibrate_floor.py --image calib/floor.jpg` | Tape 6–8 marks on the floor, put their measured (x, y) in `calib/floor_points.yaml`, and click them in order. The 1 m grid drawn afterwards should line up with the real floor. **Redo if the camera moves.** |
| **Run** | `python -m vision.main --stream --host <laptop-ip>` | Debug view at `http://<pi-ip>:8080/`. |

Floor clicking needs a display, so copy the snapshot to a laptop and run `calibrate_floor.py` there.
Then copy `calib/floor_pose.yaml` back to the Pi.

## 4. Output format (UDP JSON, 10 Hz, default port 5005)

```json
{"t": 1728170000.123, "frame": 4512,
 "robots": [{"id": 1, "x": 1.42, "y": 3.10, "heading": 87.5, "vx": 0.0, "vy": 0.12, "age": 0.05}],
 "user": null}
```

- `x`, `y`: robot center on the floor in meters, in the frame defined by your floor points
  (recommended: origin below the camera, x to the right, y away from the camera).
- `heading`: direction of the robot's **front face** in degrees, counter-clockwise from +x.
  Range (-180, 180]. For example, 90 means facing +y, away from the camera.
- `vx`, `vy`: smoothed velocity estimate (m/s).
- `age`: seconds since the robot was last seen. Robots unseen for `tracking.forget_after_s` are left out.

For the controls team:
- `python tools/receiver_example.py` is a dependency-free receiver to start from.
- `python tools/mock_publisher.py --host <laptop-ip>` sends realistic fake data, including a fake user,
  so the controls team can work before the vision system is running.

## 5. Useful options

```bash
python -m vision.main --source webcam --show        # laptop webcam (use printed markers)
python -m vision.main --source clips/a.mp4 --show   # replay a recording (tools/record.py)
python -m vision.main --print                       # print every message sent
```

**If the Pi 4 is too slow**, process on the laptop instead, with no code changes:
on the Pi run `rpicam-vid -t 0 --width 1920 --height 1080 --inline --listen -o tcp://0.0.0.0:8888`,
then on the laptop run `python -m vision.main --source tcp://<pi-ip>:8888 --show`.
Copy the calibration files to the laptop as well.

**Tuning:** see the comments in `config.yaml`. The most likely knobs to change:
- `camera.width/height`: 1280×720 is faster but has shorter range.
- `camera.focus_dioptres`: 1 / distance in meters to focus at.
- `tracking.alpha`: lower is smoother but laggier.
- `markers.dictionary`: switch to `DICT_APRILTAG_36h11` if you see false detections.

## 6. Layout

```
vision/            the package (camera, markers, robots, geometry, tracking, output, pipeline, debug_view, sim, main)
tools/             generate_markers, calibrate_camera, calibrate_floor, record, range_test,
                   mock_publisher, receiver_example, make_sim_video
calib/             calibration files (floor_points.yaml is an example to edit)
tests/             pytest suite (geometry, robot pose, rendered end-to-end, tracking, UDP)
```
