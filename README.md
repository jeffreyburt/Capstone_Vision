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

### Current 2-meter following setup

`control.formation.distance_m` is 2.0 m. Only robot 2 is selected in the current **unarmed** controller
preview (`python -m control.main --robots 2`); it sends no wheel commands. The main camera was moved to
a measured 0.395 m lens height, so its previous 1.006 m floor pose is no longer used. While the configured
replacement `calib/floor_pose_36cm.yaml` is missing, the enabled `calibration.approximate` fallback provides
**display-only estimated positions** using the confirmed 10 cm marker size, a provisional 10° downward
tilt and zero roll. Tilt was inferred from visible side-marker positions assuming their configured 0.30 m
center height; it is not a measured floor calibration. A measured floor pose takes precedence when saved.

In approximate mode robot X/Y is the weighted location of visible marker centers obtained with PnP;
top and tilted tags can contribute, but head/chassis offsets remain uncorrected and chassis heading is
left unknown. Person X/Y uses detected feet and the approximate floor plane; no floor position is claimed
when the feet are cropped out. The main dashboard shows X/Y, robot-to-person distances when both are fresh,
and explicit approximate/no-driving labels. Errors grow near the horizon. Compare estimates with a tape
measure. The UDP output carries `position_quality: approximate` and `driving_allowed: false`; the controller
rejects this data for driving even if started with `--arm`. This fallback does not provide the calibrated
3D top-marker model or moving-head compensation needed for later autonomous chassis navigation.

For floor calibration, generate two sheets with
`python tools/generate_markers.py --ids 40-41 --size-cm 15 --out markers/floor-36cm`, print at 100%,
measure the 15 cm black squares, place both flat and separated on the visible floor, then run
`python tools/floor_from_markers.py --ids 40 41 --forward-id 40 --size-cm 15`. Check the reported distances
with a tape measure and restart vision. Camera height alone does not establish a floor pose. For head
tracking while following, also measure the moving-marker geometry described in
[docs/HEAD_TRACKER.md](docs/HEAD_TRACKER.md), or mount the tracking markers rigidly on the chassis.

### Grid targets for both robots

On the system dashboard, select **Robot 1** or **Robot 2** below the floor map, then click the grid to
set that robot's destination. Each robot has its own colored dashed target. **Cancel all targets**
clears both. Click coordinates use the exact map scale, including responsive resizing. Targets outside
`area` are rejected. The target service lives only in memory and a monitor restart clears it.

The current setup is **preview only**: camera positions are approximate, chassis headings are unknown,
and neither robot's wheel server is responding. Both robots are selected in the unarmed waypoint
controller (`python -m control.main --robots 1,2 --waypoints`). It consumes dashboard targets without
following the person, shows previews, and sends no wheel commands. Preview targets stay non-executable
when calibration later improves; a new target must be selected after navigation is ready.

Physical navigation additionally requires a measured floor pose, fresh compensated chassis position
and heading (see [HEAD_TRACKER.md](docs/HEAD_TRACKER.md)), an accessible hardware drive server that
reports `require_compensated_pose: true`, and the direct routed drive network in
[NETWORK.md](docs/NETWORK.md). Only after verification can the separate controller be started with
`--waypoints --robots 1,2 --arm`. Approximate, stale, missing or uncompensated feedback and loss of the
waypoint service produce zero velocity. This is a direct point controller with the existing keep-out
constraints, not an obstacle-mapping path planner. Keep the test area clear and follow the hardware
bring-up procedure before arming. There is no auto-arm action in the dashboard.

### Marker visibility

The current robots use `DICT_4X4_50`: robot 1 IDs 0–3 (rear 2), robot 2 IDs 4–7 (rear 6).
Keep the whole black square and white margin clear of wires, connectors, brackets and glare.
`markers.adaptive_thresh_constant` controls OpenCV's threshold bias; 3 is verified on the current rear
markers, which were missed at the library default of 7. This does not recover covered marker cells or
change the allowed IDs. Robot 1's rear tag still needs its overlapping connector/cable cleared.

### System GUI

All three camera feeds are grouped at the top of the system dashboard: the annotated main camera,
then Robot 1 and Robot 2 side by side (stacked on narrow screens). Robot feeds are visible by default,
relay through the main Pi, and retain the head dashboards' 180-degree display rotation. Each can be
hidden individually; **Show all cameras** restores both. Disconnected streams retry automatically and
unavailable cameras show their reported status instead of silently disappearing. Viewing feeds does
not start head tracking, engage torque, or send wheel commands.


The top panel shows the **main camera with live detection overlays**: red person boxes, green ArUco
marker IDs, yellow robot centers, and a cyan floor estimate for the selected listener. The panel lists
current detections and frame age, and hides the video and targets when frames or vision heartbeats go
stale. It relays the existing annotated vision stream through the monitor; viewing never starts control
or sends wheel/servo commands.

Enable the main camera publisher on the main Pi once with
`bash scripts/install_sender_service.sh birdseye` from its checkout. It starts immediately and on boot;
check it with `systemctl status camstream-birdseye.service`. Then run `python -m vision.main --gui` and
`python -m monitor` on the laptop. If the laptop's vision receiver stays stalled after a long disconnect,
restart only `vision.main --gui`.
Open `http://localhost:8090/`. The main feed uses Ethernet `192.168.1.20:5600`; robot monitoring still
uses the SSH gateway. Robot centers and the listener floor estimate require floor calibration; image
boxes and marker outlines can be shown without it. No marker detections means no robot locations are
claimed. Person detection identifies people, not identities, and may choose a different listener in a crowd.


`python -m monitor` runs on its own (start it first; it keeps working while the other programs start, stop
or crash) and shows, for every machine, green / amber / red rows plus a network diagram and a top-down map
with the robots, the user and the control targets. Its status probes are read-only. Grid clicks set targets for a separate controller; the dashboard never
arms or starts wheel hardware. Nothing needs installing on the Pis:

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
