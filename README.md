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
- **[docs/STATUS.md](docs/STATUS.md): start here.** Current state of every machine and robot, verified
  results, known problems, how-to snippets and rules for working on the hardware.
- [docs/PI_GUIDE.md](docs/PI_GUIDE.md): step-by-step install and test.
- [docs/NETWORK.md](docs/NETWORK.md): the network contract (addresses, ports, bandwidth, time).
- [docs/PLAN.md](docs/PLAN.md): design and rationale.
- [docs/MVP_RUNBOOK.md](docs/MVP_RUNBOOK.md): robot control MVP (robots follow the user): setup, bring-up, tuning.

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
| Robot Pis | `.venv/bin/python ~/drive_server.py --robot-id 1 --hardware` (from `~/capstone-base`; robot control MVP) |
| Laptop | `python -m control.main --arm` (robots follow the user; see [docs/MVP_RUNBOOK.md](docs/MVP_RUNBOOK.md)) |
| Laptop | `python -m monitor` (system GUI, http://localhost:8090/: status of every Pi, robot and laptop program) |

Pis install with `bash scripts/setup_pi.sh` (`requirements.txt`). The laptop uses `requirements-laptop.txt`,
which adds YOLO.

### Marker visibility

The current robots use `DICT_4X4_50`: robot 1 IDs 0–3 (rear 2), robot 2 IDs 4–7 (rear 6).
Keep the whole black square and white margin clear of wires, connectors, brackets and glare.
`markers.adaptive_thresh_constant` controls OpenCV's threshold bias; 3 is verified on the current rear
markers, which were missed at the library default of 7. This does not recover covered marker cells or
change the allowed IDs. Robot 1's rear tag still needs its overlapping connector/cable cleared.

### System GUI

The top panel shows the **main camera with live detection overlays**: red person boxes, green ArUco
marker IDs, yellow robot centers, and a cyan floor estimate for the selected listener. The panel lists
current detections and frame age, and hides the video and targets when frames or vision heartbeats go
stale. It relays the existing annotated vision stream through the monitor; viewing never starts control
or sends wheel/servo commands.

Start the main camera publisher on the main Pi (`python -m camstream.sender birdseye` in its configured
virtual environment), then run `python -m vision.main --gui` and `python -m monitor` on the laptop.
Open `http://localhost:8090/`. The main feed uses Ethernet `192.168.1.20:5600`; robot monitoring still
uses the SSH gateway. Robot centers and the listener floor estimate require floor calibration; image
boxes and marker outlines can be shown without it. No marker detections means no robot locations are
claimed. Person detection identifies people, not identities, and may choose a different listener in a crowd.


`python -m monitor` runs on its own (start it first; it keeps working while the other programs start, stop
or crash) and shows, for every machine, green / amber / red rows plus a network diagram and a top-down map
with the robots, the user and the control targets. It only reads; it never starts, stops or commands
anything. Nothing needs installing on the Pis:

| Machine | What it shows | Where it comes from |
|---|---|---|
| Laptop | vision fps, markers seen, floor calibration, user; control mode and per-robot command | heartbeats from `vision.main` and `control.main` (UDP 127.0.0.1:5007) |
| Master Pi | reachable on Ethernet and through the robot route, camera sender, frames reaching vision, robot WiFi access point and its clients | TCP connect, SSH health script |
| Robots | reachable, WiFi signal, drive server (hardware/simulation, state, motors), head tracker (servos, torque, tracking), camera, pose from vision, control command and target | TCP, UDP status query to the drive server, the head tracker's `/api/state`, heartbeats |
| Every Pi | temperature, under-voltage / throttling, load, memory, disk, uptime, clock offset, which of our programs run | SSH with the laptop's key every 10 s (`BatchMode`, read-only commands) |

Settings (SSH users, intervals, port) are in `config.yaml` `system:`. The current configuration uses
SSH through the main Pi for monitoring and camera forwarding, so the Mac can keep its Internet WiFi
and monitor the robots without a direct subnet route. See [dashboard SSH setup](docs/NETWORK.md#dashboard-access-without-a-laptop-route-mac-or-windows).
This does not establish the network route required for driving. `--no-ssh` skips the SSH checks. The
robot cameras are only streamed while a card's camera is shown. Older drive servers still answer the status
query; copy the new `robot/drive_server.py` to the robots to also see hardware/simulation and command details.

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
control/     robot control MVP on the laptop: planner (listening triangle), controller, UDP links, main,
             drive_test
robot/       drive_server.py: runs on each robot Pi, UDP velocity commands -> wheels, with a watchdog;
             config/ measured per-robot wheel maps; join_robot_wifi.sh, wifi_client_test.sh; card/ SD-card prep
monitor/     system GUI on the laptop: probes (TCP, SSH health, drive server, head tracker), status rows,
             heartbeats from vision/control, web page
tools/       generate_markers, calibrate_camera, calibrate_floor, record, range_test, pi_check,
             mock_publisher, receiver_example, make_sim_video, floor_from_markers, rough_floor_pose,
             drive_once (one short robot move), one_lease_dhcp (find/address a Pi on a direct cable)
scripts/     setup_pi.sh, install_sender_service.sh, setup_robot_ap.sh, set_robot_ap_channel.sh
calib/       calibration files (floor_points.yaml is an example to edit)
tests/       pytest suite
```
