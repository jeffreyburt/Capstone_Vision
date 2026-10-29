# Capstone Vision

Vision for the speaker-robot system. A birds-eye camera finds both robots and the listener. All
processing happens on the controller laptop, which turns that into floor positions (meters) and
headings for the location planner.

```
Master Pi 4 + Camera Module 3  ──Ethernet──►  Controller laptop                      ──UDP 5005──► location planner
  camstream.sender birdseye                     vision.main: markers → robots          (JSON, 10 Hz)
                                                YOLO → user
Robot 1 / 2 Pi + camera  ~~WiFi via master Pi~~►  camstream.receiver.FrameReceiver ──► robot-camera processing
  camstream.sender robot1/robot2                                                       (pan/tilt check, not in this repo)
```

- **Robots:** ArUco markers (`DICT_4X4_50`) on all 4 sides. Every marker corner is projected onto its
  known height with a calibrated camera, which gives the robot's center and heading.
- **User:** YOLO person detection (Ultralytics, on the laptop). The feet go onto the floor, and the
  height is estimated from the top of the box.
- **Camera streaming (`camstream/`):** every Pi publishes JPEG frames plus metadata (timestamp, frame
  number, optional pan/tilt angles) over ZeroMQ. The laptop always gets the newest frame and never builds
  a backlog.

Docs:
- [docs/PI_GUIDE.md](docs/PI_GUIDE.md): step-by-step install and test.
- [docs/NETWORK.md](docs/NETWORK.md): the network contract (addresses, ports, bandwidth, time).
- [docs/PLAN.md](docs/PLAN.md): design and rationale.

---

## Quick start on a laptop (no hardware)

```bash
py -3.12 -m venv .venv
```
```bash
.venv\Scripts\activate
```
```bash
pip install -r requirements-laptop.txt
```
```bash
python -m pytest
```
```bash
python tools/make_sim_video.py --person
```
```bash
python -m vision.main --config sim/config.yaml --show
```

The window shows detected markers and people on the left and a top-down map on the right. Press `q` to quit.

## Running the real system

| Where | Command |
|---|---|
| Master Pi | `python -m camstream.sender birdseye` (or install as a service: `bash scripts/install_sender_service.sh birdseye`) |
| Robot Pis | `python -m camstream.sender robot1` / `robot2` |
| Laptop | `python -m vision.main --show` (birds-eye processing + UDP output) |
| Laptop | `python -m camstream.viewer --stats` (stream health / bandwidth) |

Pis install with `bash scripts/setup_pi.sh` (`requirements.txt`). The laptop uses `requirements-laptop.txt`,
which adds YOLO.

## Output to the location planner (UDP JSON, 10 Hz, `127.0.0.1:5005` by default)

```json
{"t": 1728170000.123, "frame": 4512,
 "robots": [{"id": 1, "x": 1.42, "y": 3.10, "heading": 87.5, "vx": 0.0, "vy": 0.12, "age": 0.05}],
 "user": {"x": 0.10, "y": 4.80, "height": 1.72, "vx": 0.0, "vy": 0.0, "age": 0.12, "confidence": 0.88}}
```

- `x`, `y`: floor position in meters, in the frame set by the floor calibration (recommended: origin
  below the camera, x to the right, y away from the camera).
- `heading`: the direction of the robot's **front face**, in degrees counter-clockwise from +x, in the
  range (-180, 180].
- `vx`, `vy`: smoothed velocity in m/s.
- `age`: seconds since that object was last actually seen. Use it to stop trusting stale positions.
- Robots and the user that haven't been seen for 5 s are left out (`user` becomes `null`).
- `t` is the birds-eye frame's capture time (master Pi clock).

Two helpers:
- `python tools/receiver_example.py` is a dependency-free receiver to start from.
- `python tools/mock_publisher.py` sends realistic fake data, so the planner can be developed without
  any hardware.

## Robot camera frames (for the pan/tilt processing)

```python
from vision.config import load_config
from camstream.receiver import FrameReceiver

rx = FrameReceiver(load_config(), ["robot1", "robot2"])
f = rx.get_latest("robot1")     # None until the first frame arrives
f.image, f.seq, f.t_capture, f.meta, f.age
rx.stats("robot1")              # fps, Mbps, dropped frames, latency
```

To attach pan/tilt angles to frames, run the sender inside the robot's control program:

```python
from camstream.sender import CameraSender
sender = CameraSender.from_config(cfg, "robot1").start()
sender.update_meta(pan_deg=..., tilt_deg=...)
```

## Useful options

```bash
python -m vision.main --source webcam --show        # laptop webcam (with printed markers)
python -m vision.main --source clips/a.mp4 --show   # replay a recording (tools/record.py)
python -m vision.main --stream                      # debug view in a browser: http://localhost:8080/
python -m camstream.sender robot1 --source test     # synthetic camera, for network testing
```

To test senders and receivers on one machine, set `CAMSTREAM_HOST=127.0.0.1`. That points every stream at
localhost.

## Layout

```
vision/      birds-eye processing: camera sources, markers, robots, people, geometry, tracking, output,
             pipeline, debug view, simulator, main
camstream/   camera streaming: protocol, sender (Pis), receiver + viewer (laptop)
tools/       generate_markers, calibrate_camera, calibrate_floor, record, range_test, pi_check,
             mock_publisher, receiver_example, make_sim_video
scripts/     setup_pi.sh, install_sender_service.sh
calib/       calibration files (floor_points.yaml is an example to edit)
tests/       pytest suite
```
