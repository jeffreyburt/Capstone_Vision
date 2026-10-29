# Central Vision System — Project Plan

**Status:** Robot detection, person detection and camera streaming are implemented. §10 describes the current architecture: processing on the laptop, the Pis stream. §9 has the original Q&A decisions.
**Scope:** The central controller camera (Raspberry Pi 4 + Pi Camera). It finds where every robot and the user are on the floor and sends those positions to the control system.
**Guiding principle:** Keep it simple and easy to develop. Pure Python, well-known libraries, and something working end to end early.

---

## 1. What the vision system does

**Inputs:** frames from the Pi Camera.

**Outputs (about 10 Hz):** floor-plane positions in meters, in one shared room coordinate frame:

```json
{
  "t": 1728170000.123,
  "frame": 4512,
  "robots": [
    {"id": 1, "x": 1.42, "y": 3.10, "heading": 87.5, "vx": 0.0, "vy": 0.1, "age": 0.05},
    {"id": 2, "x": -1.38, "y": 3.05, "heading": 92.0, "vx": 0.0, "vy": 0.0, "age": 0.05}
  ],
  "user": null
}
```

`age` is the number of seconds since that object was last seen. The controller can then tell the difference between "the robot is here" and "we last saw the robot here 2 s ago".

**Out of scope (unless you tell me otherwise):** working out the optimal speaker positions, robot path planning, and the robots' onboard cameras. Those belong to the controller and robot code. The vision system only reports where things are.

---

## 2. Technology choices

| Concern | Choice | Why |
|---|---|---|
| Language | Python 3 | Your preference. Every library below has first-class Python support. |
| Camera API | **Picamera2** on the Pi, **OpenCV `VideoCapture`** on a laptop | Picamera2 is the official, supported way to use the Pi Camera on current Raspberry Pi OS. A small `Camera` wrapper lets the whole pipeline run on your Windows laptop with a webcam or a recorded video, so most development doesn't need the Pi. |
| Core vision | **OpenCV** (`opencv-python`) | One library covers marker detection, camera calibration, geometry and drawing debug overlays. |
| Robot markers | **ArUco `DICT_4X4_50`** via `cv2.aruco` (built into OpenCV, no extra install) | See §3. |
| Person detection | **Ultralytics YOLO11n** (nano), exported to **NCNN** for the Pi's ARM CPU | About 3 lines of Python to use. Detects full bodies reliably at 20 ft. Has an official Raspberry Pi export path. See §4. |
| Pixel → floor mapping | Camera calibration (OpenCV checkerboard) + a one-time floor calibration | See §5. |
| Smoothing / tracking | A small hand-written filter (EMA / alpha-beta) in plain Python | No extra dependency, and easy to understand and tune. |
| Output | **JSON over UDP** to a configurable IP/port | Uses only the standard library. Any language on any machine can receive it. Losing a packet doesn't matter because a new one arrives 100 ms later. |
| Config | One `config.yaml` | All tunable values in one place: resolution, marker size, IDs, ports. |
| Tests | `pytest` | Geometry and marker tests run on the laptop with synthetic images. |

**Fallbacks if the Pi 4 is too slow:** lower the YOLO input size; run person detection less often; or (later) a Pi 5, optionally with the Raspberry Pi AI HAT. The code structure stays the same.

---

## 3. Robot markers

### Recommendation: ArUco 4x4 (`DICT_4X4_50`), printed 15–20 cm (6–8 in) black-square size, mounted **vertically** on the robot

**Why 4x4 instead of AprilTag 36h11?** Detection range depends on how many pixels each marker "cell" covers. A 4x4 marker is 6 cells across (including the black border). A 36h11 AprilTag is 8 cells across. At the same printed size, 4x4 cells are about 33% bigger, which gives noticeably more range. The drawback of 4x4 is a higher false-positive rate. We avoid that by **only accepting the few IDs assigned to our robots** and by tracking across frames. If false positives still show up, switching to AprilTag (`DICT_APRILTAG_36h11`, also in `cv2.aruco`) is a one-line config change.

**Range estimate at 20 ft (6.1 m)**, for Camera Module 3 (66° horizontal FOV), with the marker facing the camera:

| Marker size | Resolution | Marker width in image | px/cell (4x4) | px/cell (36h11) |
|---|---|---|---|---|
| 15 cm | 1280×720 | ~24 px | ~4.0 | ~3.0 |
| 15 cm | 1920×1080 | ~36 px | ~6.0 | ~4.5 |
| 20 cm | 1280×720 | ~32 px | ~5.4 | ~4.0 |
| 20 cm | 1920×1080 | ~48 px | ~8.1 | ~6.1 |

A common rule of thumb is that detection becomes reliable at about **4–5 px per cell** (motion blur and focus also matter). So **4x4 at 15–20 cm and 1280×720 should be just good enough, with 1920×1080 as the safety margin.** Step 2 of the plan is a real range test to check these numbers before we build anything else on top of them.

**Why vertical and not on top of the robot?** A camera at about 1.5–2 m height looks at the floor 6 m away at a shallow angle. A marker lying flat on top of a robot would be squashed to about a third of its height in the image, and range would collapse. Vertical markers that face the camera avoid this. The robots will turn, though, so the likely design is **a marker on each side of the robot** (a box or a short "mast" with 4 faces). Each face gets its own ID, mapped in config, for example:

```yaml
robots:
  1: {marker_ids: [0, 1, 2, 3]}   # front, left, back, right
  2: {marker_ids: [4, 5, 6, 7]}
```

Seeing a particular face also gives a rough heading for free. This depends on your robot design (see Questions).

**Printing:** I'll include `tools/generate_markers.py`, which outputs print-ready PDFs/PNGs at the exact physical size, with the required white border. Mount markers on rigid, matte backing (foam board), since glossy prints cause glare.

---

## 4. Person detection

**Body detection, not face detection.** At 20 ft a face is only about 20–25 px wide, and detection fails as soon as the user turns away. A full body is about 250+ px tall at that distance, which is easy for a detector.

**Model:** YOLO11n (Ultralytics), class "person" only, exported to NCNN (Ultralytics' recommended format for Raspberry Pi CPUs). Expected speed on a Pi 4 is a few frames per second at a 320–416 px input size. That is enough for a person, since the speakers move much more slowly than that. We'll measure it in step 5.

**User position on the floor:** the bottom-center of the person's bounding box (their feet) projected onto the floor (§5). If their feet are often hidden (sitting on a couch, or blocked by a robot), we can switch to a "known height" method based on the box center or top. The config will support both.

**Choosing "the user" when several people are visible:** the default rule is to lock onto the first or largest person and then keep following whoever is closest to the user's last known position. Alternatives depend on your answer to the multi-person question below.

**Fallback detector:** if Ultralytics or PyTorch turns out to be a problem to install on the Pi, OpenCV's DNN module with MobileNet-SSD needs no new dependencies. It is less accurate but workable.

---

## 5. Getting real-world positions: pixel → floor

Everything (robots and user) is reported in one **floor coordinate frame in meters**. For example, the origin is the center of the listening area, +y points away from the camera, and +x points to the right. We get there in two one-time calibration steps:

1. **Lens (intrinsic) calibration**: wave a printed checkerboard in front of the camera. `tools/calibrate_camera.py` saves `camera_intrinsics.yaml`. This is done once per camera and lens.
2. **Floor (extrinsic) calibration**: put 4 or more tape marks on the floor at measured positions. `tools/calibrate_floor.py` shows the camera image, you click each mark and enter (or load) its coordinates, and OpenCV `solvePnP` computes exactly where the camera sits and how it is tilted relative to the floor. This is redone whenever the camera is moved.

After that, one function does all the mapping:

```python
pixel_to_floor(u, v, height=0.0) -> (x, y)
```

It casts a ray from the camera through pixel (u, v) and finds where the ray reaches the given height.
- **User:** feet pixel, `height = 0`.
- **Robot:** marker-center pixel, `height =` the marker's mounting height (from config).

This is **much more accurate at range than estimating distance from the marker's apparent size**. At 6 m, a 1-pixel error in marker size changes the size-based distance estimate by about 25 cm, while the floor-ray method moves only a few cm per pixel.

**Camera placement matters.** I recommend mounting the camera **raised (about 1.5–2.5 m) and tilted slightly down** over the area. The robots are then less likely to hide the user, and floor depth estimates are more accurate.

---

## 6. Software architecture

```
Capstone_Vision/
├── config.yaml                 # all settings
├── requirements.txt, pytest.ini, README.md
├── docs/PLAN.md
├── vision/                     # the package
│   ├── config.py               # load config.yaml
│   ├── camera.py               # PiCameraSource, OpenCVSource (webcam/stream), VideoFileSource
│   ├── markers.py              # ArUco detection -> MarkerDetection(id, corners)
│   ├── geometry.py             # CameraModel, FloorPose, pixels_to_floor(), solve_floor_pose()
│   ├── robots.py               # markers -> robot center + heading
│   ├── tracking.py             # alpha-beta smoothing, outlier rejection, timeouts
│   ├── output.py               # message format + UDP publisher
│   ├── pipeline.py             # per-frame processing
│   ├── debug_view.py           # overlay, top-down map, MJPEG stream
│   ├── sim.py                  # synthetic renderer for tests / laptop development
│   └── main.py                 # entry point
├── tools/                      # generate_markers, calibrate_camera, calibrate_floor, record, range_test,
│                               # mock_publisher, receiver_example, make_sim_video
├── calib/                      # calibration files (floor_points.yaml example)
└── tests/                      # pytest suite
```

**Runtime loop (threads kept to a minimum):**

```
[capture thread] ── latest frame ──► [main loop: markers every frame] ──► tracking ──► UDP publish (10 Hz)
                                 └─► [person thread: YOLO on latest frame, as fast as it can] ─┘
```

Marker detection is cheap and runs on every frame. YOLO is slow, so it runs in its own thread on the newest available frame. That keeps it from slowing the robot updates. Each detection carries its own timestamp.

**Run modes:**
`python -m vision.main --source picam`, `--source webcam`, or `--source clip.mp4`, plus `--debug` for the overlay window and map.

---

## 7. Development steps

Each step ends with something you can demo or measure.

| # | Step | Deliverable / "done when" |
|---|---|---|
| 0 | **Environment setup** | Repo skeleton, `requirements.txt`, README. Pi: 64-bit Raspberry Pi OS, Picamera2 (apt), venv with `--system-site-packages`. Laptop: same venv recipe minus Picamera2. |
| 1 | **Camera capture + recording** | `camera.py` works with picam, webcam and video file; `record.py` saves clips. Live preview shows FPS. |
| 2 | **Markers + range test** ⚠️ *biggest risk, done early* | `generate_markers.py`, `markers.py`, `range_test.py`. Print 15 and 20 cm markers, test at 5/10/15/20 ft at 1280×720 and 1920×1080. **Result: we lock in marker size and resolution.** |
| 3 | **Lens calibration** | `calibrate_camera.py` → `camera_intrinsics.yaml`; reprojection error reported. |
| 4 | **Floor calibration + pixel→floor** | `geometry.py`, `calibrate_floor.py`. Place a marker at taped spots and check the reported (x, y) against a tape measure. Target: ±10–15 cm at 20 ft. |
| 5 | **Person detection** | `people.py` with YOLO11n NCNN. Measure FPS on the Pi 4 and the detection rate at 20 ft. User (x, y) on the top-down map. |
| 6 | **Tracking + robustness** | Smoothing, "lost" timeouts, user selection with multiple people, ignoring unknown marker IDs. |
| 7 | **Output interface** | UDP JSON publisher, `receiver_example.py`, and `mock_publisher.py`. *(The mock can be delivered right after step 0 so your teammates aren't blocked.)* |
| 8 | **Integration + performance tuning** | Full pipeline on the Pi at the target rates. Tuning: resolution, YOLO input size, threading. Optional: start on boot (systemd). |
| 9 | **Validation + docs** | Accuracy and latency measurements for your report, a lighting/occlusion test, and a final README. |

**Testing strategy:** most logic can be tested on the laptop without hardware.
- Geometry: project known 3D points with a fake camera, then confirm `pixel_to_floor` recovers them.
- Markers: render markers synthetically at shrinking sizes and with blur.
- Tracking: feed scripted detections.
- Recorded clips from the real room give repeatable end-to-end tests.

---

## 8. Risks and mitigations

| Risk | Mitigation |
|---|---|
| Markers not detected reliably at 20 ft | Measured in step 2 before anything depends on it. Options: larger markers, higher resolution, Camera Module 3's autofocus / a fixed focus at about 4 m, matte prints, good lighting. |
| Pi 4 too slow for YOLO | Smaller input size, lower person-detection rate (the user moves slowly), or the MobileNet-SSD fallback. Upgrade path: Pi 5 / AI HAT. |
| Robot blocks the user's feet | Elevated camera; "known height" fallback for user position; hold the last position briefly. |
| Several people in the room | User selection by continuity (see Questions for alternatives). |
| Lighting changes, window glare | Matte markers, fixed exposure settings in Picamera2, test early in the real room. |
| Pi Python packaging (Picamera2 is apt-only) | Documented venv recipe with `--system-site-packages`; pinned versions. |

---

## 9. Decisions from Q&A (2026-10-05)

| Question | Answer | Effect on the design |
|---|---|---|
| Camera | Camera Module 3 (66° HFOV) | Force the full-FOV 2304×1296 binned sensor mode, so smaller output sizes don't crop the view. Fix focus at ~4 m (0.25 dioptre) so autofocus doesn't hunt. |
| Mounting | ~2 m high, tilted slightly down, open room | Matches the floor-ray geometry in §5. |
| Robots | 2 robots, ~30×30 cm, omnidirectional, markers on all sides | 4 markers per robot (IDs 0–3 and 4–7), one per face. Robot center = face center pushed 15 cm inward. |
| Heading | Needed, rough is OK | Heading = which face is visible + the face's yaw, measured from where the marker's corners land at their known heights. If two faces are visible, their results are averaged. |
| People | One standing person | User = feet point (bottom of box) on the floor. If the feet are cut off at the image edge, fall back to the head at a known height. |
| Consumer | Controller laptop over Ethernet/WiFi | UDP JSON sent to the laptop's IP (or broadcast). |
| Rates | TBD | Robots are updated on every frame (~10–20 Hz expected). Person detection runs as fast as the Pi allows (~3–6 Hz expected). Output is sent at 10 Hz. |
| Compute | Stuck with Pi 4 | Offload fallback without code changes: the Pi streams video (`rpicam-vid ... --listen -o tcp://0.0.0.0:8888`) and the laptop runs `python -m vision.main --source tcp://<pi-ip>:8888`. |

**Output conventions:** distances in meters; `heading` in degrees, counter-clockwise from the world +x axis (0° means the robot's "front" face points along +x). The world frame comes from the floor calibration points. Recommended layout: x to the right as seen from the camera, y pointing away from the camera, z up.

## 10. Architecture update (2026-10-29)

The team changed the system layout:

- **Controller laptop:** does location planning **and all vision processing**. That covers birds-eye
  robots, plus person detection with YOLO. Running YOLO on the laptop removes the Pi 4 speed problem
  from §4.
- **Master Pi 4:** connected to the laptop over Ethernet. It runs the birds-eye camera *sender* and is the
  WiFi access point and bridge for the robot Pis. The network setup is a separate stack; the contract it
  must meet is in [NETWORK.md](NETWORK.md).
- **Robot (slave) Pis:** each streams its own camera to the laptop. Robot motion and pan/tilt control are
  out of scope for vision, and so is processing the robot camera images (pan/tilt check). The vision
  scope there is the **transport**.
- **No ROS** for now.

**Camera transport (`camstream/`):**
- Each Pi publishes JPEG frames with a JSON header (camera, seq, capture time, size, free-form `meta` such
  as pan/tilt angles) on a ZeroMQ PUB socket.
- The laptop subscribes with `FrameReceiver`, which keeps only the newest frame per camera. ZeroMQ
  "conflate" on both ends means no backlog builds up when bandwidth or the consumer is slow.
- The cameras *bind* and the laptop *connects*, so the Pis never need the laptop's address and several
  laptop programs can watch the same camera.
- JPEG was chosen over H.264 for simplicity, per-frame metadata and dropping stale frames. Measured cost:
  about 2.8 Mbps per 768×432 robot stream at 10 fps (test pattern), well within the Pi 4 access point's
  capacity.

**Person → floor:**
- The foot point (bottom-center of the box) is projected to z = 0.
- Height = where the head pixel's ray passes over that floor point.
- If the feet are cut off at the image edge, the head is projected at an assumed height instead.
- With several people in view, the user is the one nearest the current user track (otherwise the most
  confident detection).
- In simulation (a real person photo composited into rendered scenes): 3.3 cm mean floor error, 95% of
  frames within 9.5 cm.
