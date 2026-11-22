# Capstone head tracking and vision-guided driving

## Current installed profile (2026-11-20)

The saved, active configuration and experiment recordings are in [calib/README.md](../calib/README.md).
Robot 1 uses its mechanically secured rear marker as `fixed`; it does not need servo replies. Robot 2
uses camera-fitted pan/tilt geometry and measured HTTP joint feedback through the main Pi. It is limited
to the tested pan 331–451 / tilt 710–775 range. Both wheel services run standalone compensated-pose mode;
do not start the neutral-only UART wrapper alongside the dashboards or wheel services.

Open the main dashboard, enable click driving, select a robot, then click a fresh grid destination.
Driving starts disabled after monitor restart. The currently running controller has `--arm`, but the
separate dashboard enable switch and fresh calibrated-pose checks gate every command. Cancelling goals,
disabling driving, loss of vision/joints or leaving the calibrated range stops motion. Mac Wi-Fi stays
unchanged; command UDP is relayed over authenticated SSH through the main Pi.

Marker calibration additionally supports `rotation_vector_rad` (a Rodrigues rotation of neutral corner
offsets about the marker center) and `head_pose.valid_units` (two calibrated encoder ranges). Fixed-only
heads need no live joint source; placeholder neutral/scale/pivots are unused for their fixed marker.
The image-space pose fit avoids unstable ray/height intersections at this low camera vantage point.
See the calibration inventory for reuse, validation accuracy, clock mapping and startup commands.

## Head tracking from the system dashboard (2026-11-21)

The system dashboard (`python -m monitor`, http://localhost:8090/) now shows head tracking for both robots, under
each robot's live camera: servo positions, torque, camera and tracking status, face boxes drawn on the camera
image, and **Engage torque / Start tracking / Stop tracking / Disengage torque** buttons.

- **How it works:** the robot's own dashboard (`head_tracker/dashboard.py`, port 8080) still owns the servo
  UART, the camera and the tracking loop. `monitor/heads.py` relays its `/api/state`, `/api/camera/frame` and
  control requests through the existing SSH camera tunnel. The system dashboard tab runs the same bundled
  MediaPipe face detector (served from `head_tracker/assets/` at `/head-assets/`) for every enabled robot and posts
  observations, so the robot pages at :8080 no longer need to be open. No robot-side code changed.
- **Start tracking** uses the settings saved on the robot (`tracking-settings.json`: taught tilt range,
  directions, speed, prediction). Teach limits on the robot's own page if none are saved.
- **Per-robot switch:** `system.robots.<id>.head_tracking: false` (with optional `head_tracking_reason`) in
  `config.yaml`. A disabled robot is still shown read-only; the relay refuses engage, start, observations,
  disengage and camera frames for it, but always forwards **Stop**.
- **Current setting:** Robot 1 is **disabled** (its head servos are not connected; its dashboard reports
  "No response from servo"). Robot 2 is enabled. To turn Robot 1 on later, connect the servos, teach its limits
  on http://192.168.50.105:8080/, remove `head_tracking: false`, and restart the monitor.
- **Keep the dashboard tab open and visible while tracking.** Detection pauses in a hidden tab; the robot then
  holds position (it stops following after 0.6 s without observations). Closing the tab does not disengage torque.
- Tracking moves Robot 2's markers. Click (waypoint) driving needs its head inside the calibrated pose range
  above; keyboard driving does not use the camera pose.
- 2026-11-21: Robot 2 torque engaged and tracking started from the system dashboard (saved settings: speed 60,
  signs pan -1 / tilt +1, tilt 433–812, prediction 0.8 s). Observations were accepted; frame fetch plus detection
  took about 40 ms. Tests: `tests/test_heads.py`.

## Finding the user when tracking starts (2026-11-22)

- **Main camera first:** whenever the main camera sees both the user and the robot, the dashboard aims the head at
  the user from that (also when the user is outside the head camera's view: the robot now accepts aim points up to
  half a frame beyond the image edges; before, those were rejected as invalid and the head just held).
- **Otherwise scan:** before any face has been seen, and again after 4 s without one, the head sweeps side to side
  (+-150 units) on two rows looking up (`search_up_units` 100, then 40), at 150 units/s so a face is caught mid-sweep,
  centred on the head's calibrated forward/level pose (`head_pose.neutral_units`, sent as `search_center`).
  Face detection takes over as soon as a face is seen.
- **Lagging servo:** if a joint falls more than max(45, 0.3 x speed) units behind (e.g. tilting far up under load),
  the target re-syncs to where it is; tracking only stops if that persists for ~2 s.
- **Calibration page robot step:** the head sweep now tilts in each robot's own "up" direction (Robot 1 +units,
  Robot 2 -units, from `head_tracking_signs`). Before, Robot 1's tilt sweep hit its lower limit and barely moved:
  run **Calibrate robots** again to fit Robot 1's tilt properly.
- **Main camera receiver:** if no frame arrives for 5 s (main Pi rebooted), it reopens the stream itself.
- 2026-11-22: Robot 1's servos read 3.95 V with all fault flags set and would not hold torque: its 7.4 V servo
  supply was off or flat. Check it before running Robot 1's head.

## Aiming the speaker, not the camera (2026-11-22)

Each speaker driver sits ~4 in (0.1016 m) below its camera centre. To put the speaker's axis on the face the head
looks up by atan(offset / distance), so the face is held that far **below** the image centre instead of on it.
Distance comes from the face box: the face (assumed `face_width_m` 0.15 m wide) and the offset subtend angles in the
same ratio at any distance, so the drop in servo units is the face box width in units x offset / face width. No
extra calibration is needed beyond the measured image scale.

- Config per robot: `system.robots.<id>.speaker_offset_m: 0.1016`, `face_width_m: 0.15` (both robots); sent with
  Start as `aim_offset_m` / `face_width_m`. Observations now carry the face box width (`size`, fraction of the image
  width); without it (main-camera aim) no image offset is applied, because that aim is computed from the speaker's
  own height instead (`aim_from_main_camera(..., speaker_offset_m)`).
- At 1 m the face is held at 0.71 of the image height (24 units, ~5.8 deg up); at 2 m ~0.60; at 0.5 m ~0.91.
- The dashboard draws a blue **speaker aim** cross where the face sits when the speaker is on it.
- Unit test checks the offset against the exact angle at 0.5-3 m (within 3%). Deployed to both robots
  (backups `bak-2026-11-22-speaker/`).

## Robot 1 tilt direction, measured image scale, steadier prediction (2026-11-22)

- **Robot 1 can look up.** Its camera is mounted upside down like Robot 2's, and **+tilt units look up** on Robot 1
  (down on Robot 2). The earlier test that said "down only" misread the image-shift sign; with tilt direction +1 the
  head tilted away from the face until it lost it. Now `head_tracking_signs: [-1, -1]` for Robot 1 (`[-1, 1]` for
  Robot 2) and its tracking range tilt 692-980 runs from level up to ~70 deg up. Its vision head model has tilt
  `degrees_per_unit` -0.24.
- **Measured image scale.** Moving each joint +-40 units against a textured view (phase correlation 0.4-0.7):

  | | Pan: +units turn the camera | Units across the 320x240 width | Tilt: +units turn the camera | Units across the height |
  | --- | --- | --- | --- | --- |
  | Robot 1 | left | 168 | up | 116 |
  | Robot 2 | left | 171 | down | 117 |

  The tracker had assumed 216 x 171, so every correction and prediction aimed 30-50% too far: the "getting lost"
  overshoot. Per robot: `system.robots.<id>.head_tracking_units_per_frame`, sent with Start as `units_per_frame`;
  the main-camera aim uses it too. Default `UNITS_PER_FRAME = (170, 117)`.
- **Prediction no longer swings away:** face velocity is estimated over the last 0.3 s instead of two frames 33 ms
  apart (detection jitter looked like fast motion), capped at 150 units/s, and a lost face is followed for at most
  0.6 s before the head holds the predicted spot.
- Deployed to both robots (identical files; backups `bak-2026-11-22-scale/`); both tracking afterwards.

## No overshoot at high speed; both robots on the same tracker (2026-11-22)

- **Cause of the oscillation:** the loop chased the face's *image* offset, which only updates once per detected
  frame and describes where the head pointed when that frame was taken (30-150 ms earlier). Between frames the
  head kept accelerating on a stale error, passed the face, and did the same coming back.
- **Fix:** each detection is turned into a *servo-space goal* (the pan/tilt that centres the face, using where the
  head pointed at capture time) and `tracking_control.approach()` moves toward it at `APPROACH_GAIN` (10/s) x the
  remaining error, capped at the tracking speed, with the same dead zone, acceleration limit and hard limits. A
  stale frame no longer adds motion. Prediction already worked in servo space and feeds the same goal.
- Simulation (`tests/test_tracking_control.py`): face 150 units off, 30 fps camera, 150-250 ms delay, servo lag,
  speed 400: no overshoot at gains 5-16, settles in ~0.8 s at 10; the old image-error loop overshoots by >20 units.
- **Same code on both robots:** `dashboard.py`, `tracking_control.py`, `camera_feed.py`, `index.html`,
  `assets/head_detection.mjs` are byte-identical to the repo on both. Per-robot differences come from hardware and
  config only: Robot 1 finds its USB BusLinker (pan ID 1, tilt ID 2), Robot 2 its Pi UART `/dev/serial0`; limits and
  directions from `system.robots.<id>.head_tracking_*` (Robot 2 also has its own saved settings). Backups in
  `bak-2026-11-22-unified/` on each robot. Drive side was already identical: same `drive_server.py`; Robot 1's mixed
  Hiwonder + TB6612 drive comes from `robot1_four_wheels.json` / `motor_speed.json`, Robot 2's plain Hiwonder from
  `robot2_wheels.json`. `motor_diagnostic.py` added to Robot 2 (its dashboard motor tests still need the sudoers
  entry in MOTOR_DIAGNOSTICS.md).
- Both tracking from the dashboard afterwards: Robot 1 27 and Robot 2 28.5 detections/s at 320x240.

## Robot 1 head tracking (2026-11-22)

- Limits from the dashboard's range seen (pan 307-1062, tilt 684-1093), with margins and the 0-1000 command range:
  `system.robots.1.head_tracking_limits: [[327, 980], [692, 980]]`. **Tilt only moves down from level** on this
  mount (+tilt looks down, measured from Robot 1's camera: 70 units moved the image 221 px), so Robot 1 cannot look up
  at a standing face unless the stop near 684 is changed.
- Directions `head_tracking_signs: [-1, 1]` (pan matches Robot 2; the pan image test was weak against a plain wall).
  With no settings saved on the robot, Start uses these config limits/signs; Speed/Predict default to 300 / 1.5 s.
- Vision now treats Robot 1's head as moving: forward = units 772/686 (where the secured head sat for the fixed
  calibration), rear tag 2 on the tilt stage, side tag 3 on pan (centre only), valid range pan 307-1000 / tilt
  680-1000. **Pivots and degrees-per-unit are estimates** (LX-224 nominal 0.24 deg/unit, Robot 2's pivots) until the
  Calibration page's robot step is run with the robots back in front of the main camera; it fits them.
- Fixed in `head_tracker/dashboard.py`: error replies containing '–' crashed the robot's handler and dropped the
  connection. Deployed to Robot 1; Robot 2 still has the old file (it was offline) and should get it when back.
- Robot 1 tracking started from the dashboard; ~15 face-detection frames/s from its camera. Not yet tested with a face.

## Robot 1 head servos on a USB BusLinker; live positions for finding limits (2026-11-22)

Robot 1's pan (ID 1) and tilt (ID 2) LX servos are daisy-chained on a BusLinker V2 plugged into the Pi by USB
(CH340, `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`), powered at 7.4 V. `head_tracker/dashboard.py` now picks
its servo bus as: `CAPSTONE_SERVO_PORT` if set, else a USB BusLinker if plugged in, else `/dev/serial0` (Robot 2 is
unchanged), and reports it as `servoPort`. Deployed to Robot 1 (`~/capstone-head-tracker/`, backup in
`bak-2026-11-22-usb/`; camera code left as it was).

On the system dashboard each robot's head-tracking panel shows live pan/tilt (refreshed ~4x a second) and the
**range seen** since **Reset range**. To find travel limits: Disengage torque, move the head by hand to each end,
and read the range. Engage/Disengage torque and Stop now work even when a robot's head tracking is disabled;
tracking itself stays refused. Robot 1's head tracking stays off until its limits are measured.

## Faster tracking and head-coordinate prediction (2026-11-21)

| Change | Before | After | File |
| --- | --- | --- | --- |
| Robot camera frame rate | 15 fps | 30 fps (measured 30.0 on Robot 2) | `head_tracker/camera_feed.py` |
| Image error for full speed (beyond the settle zone) | 0.35 of the frame | 0.20 | `head_tracker/tracking_control.py` |
| Acceleration | 4 × speed per s | 8 × speed per s | `head_tracker/tracking_control.py` |
| Following-error stop | 45 units | max(45, 0.3 × speed): 90 at 300 units/s | `head_tracker/dashboard.py` |
| Robot 2 tracking speed / prediction | 60 units/s / 0.8 s | 300 units/s (about 72°/s) / 1.5 s | saved on the robot |
| System dashboard detection loop | 40 ms pause per frame | next new frame immediately (about 28 detections/s measured) | `monitor/page.html` |

- **Off-screen prediction is kept and now works in head (servo) coordinates.** Each observed face is converted to
  the servo positions that would centre it, using where the head was aimed when that frame was captured. When the
  face leaves the frame, the robot predicts the person's position from their recent velocity for the prediction time,
  turns there, and holds there once the prediction expires. Before, prediction was in image coordinates and ignored
  the head's own turning, so a lost face kept the head turning at full speed for the whole prediction time
  (up to about 100° at 300 units/s).
- The image-to-servo scale `UNITS_PER_FRAME = (216, 171)` assumes Camera Module 3 at 640×480 sees about 52°×41° and
  an LX-224 unit is 0.24°. It is an estimate: if a predicted turn consistently overshoots or falls short, adjust it.
- The system dashboard has **Speed** and **Predict off-screen** inputs under each robot's head-tracking buttons.
  They apply when tracking starts; taught limits and directions still come from the robot.
- Deployed to Robot 2 `~/capstone-dashboard/` (backup in `bak-2026-11-21-fast/`). This also brings that copy up to
  date with the repo (it was missing the optional `--pose-host` joint-feedback options). The dashboard service was
  restarted with torque held; tracking was restarted from the system dashboard.
- **Not yet tested with a person moving in front of the camera** at the new speed. If the head overshoots or
  oscillates, lower the speed from the dashboard (for example 200) before changing the gains.

## Aiming from the main camera (2026-11-21)

When Robot 2's own camera doesn't see a face, the system dashboard aims its head from the main camera instead:
`GET /api/head/robot2/aim` takes the user's floor position and height and Robot 2's position and heading from
vision, works out the pan/tilt that points the head camera at the face (head_pose neutral units, degrees per unit,
pan pivot, tilt pivot height; positive pan = chassis left, positive tilt = down; face 0.12 m below the top of the
head), clamps it to the tracking limits, and converts it to an image-space observation for the robot's existing
tracking loop. Face detection in the head camera still wins when it sees a face. The head-tracking line on the
dashboard shows which source is aiming. Untested with a person yet at the time of writing.

## Smaller robot camera frames (branch `feature/face-res-and-feedforward`, merged into main 2026-11-22)

**On Robot 1 (2026-11-22):** 320×240 keeps the same field of view as 640×480 (frames matched at scale 1.00,
correlation 0.99), so `UNITS_PER_FRAME` still holds; the system dashboard detected on 29.3 frames/s (was ~15 at
640×480 on Robot 1's Pi 3). The dashboard's head panel shows each robot's camera size and fps. Robot 2 still runs
the previous files (it was offline) and needs `dashboard.py`, `camera_feed.py`, `tracking_control.py`, `index.html`
and `assets/head_detection.mjs` copied when it is back.


- `head_tracker/dashboard.py --camera-size WxH` (default **320x240**, was fixed 640x480) and `--camera-fps N`
  (default 30, now allowed up to 60). `--camera-size 640x480` restores the previous behaviour.
- Why: BlazeFace (short range) shrinks every frame to 128×128, so 320×240 should not cost detection quality,
  while JPEG encoding on the Pi, WiFi transfer and browser decoding drop to about a quarter.
- Expected effect: **lower latency per detection, not many more detections per second by itself.** The
  dashboard loop was already measured at ~28 detections/s against the camera's 30 fps, so the camera rate is
  the ceiling. The smaller frames are what make `--camera-fps 60` worth trying.
- Both detector pages (`head_tracker/assets/head_detection.mjs` on the robot's :8080 page and `monitor/page.html`
  in the system dashboard) now detect at the frame's real size and scale boxes to the 640×480 overlay.
- **Check on the robot before keeping it:**
  1. Picamera2 picks the sensor mode from the requested size. If 320×240 selects a different crop than 640×480,
     the field of view changes and `UNITS_PER_FRAME` (prediction scale) is off: compare a frame at each size.
  2. Faces at the far end of the room still detected at 320×240.
  3. Detections/s on the system dashboard at 30 fps, then at `--camera-fps 60`.
- Deploy: copy `head_tracker/camera_feed.py`, `dashboard.py` and `assets/head_detection.mjs` to the robot's
  dashboard folder (Robot 2: `~/capstone-dashboard/`, back up first), add the flags to the service's `ExecStart`
  if not using the defaults, restart the dashboard service with torque held. Roll back by restoring the backup.

The sections below describe general workflows and legacy neutral-only setup; use the current installed
profile above for this robot pair.

The robot has two operating modes. In parked mode, the robot-mounted camera and browser face detector aim the speaker using the existing head tracker. In drive mode, the overhead vision camera tracks the ArUco markers and the laptop commands the chassis. The speaker stays level and faces forward during drive mode because the markers are attached to it.

## Layout and setup

- Copy the repository to each robot Pi. `head_tracker/dashboard.py` and its assets run the parked head tracker. `drive_with_head_lock.py` and `robot/drive_server.py` run the wheels and verify servo feedback in drive mode.
- On the laptop, run the team's Capstone_Vision checkout. Use `control/` and `config.yaml` to configure the vision UDP port (5005), the robot drive port (5700), robot IDs, addresses, and neutral marker geometry. The integration uses the existing UDP protocol. Follow the [MVP runbook](MVP_RUNBOOK.md) for the laptop setup, dry run, simulation, and wheel calibration.
- Use the [head-tracker README](../head_tracker/README.md) to set up the Pi camera and UART. The integrated drive command runs in the `capstone-base` virtual environment and needs both `capstone_base` and `pyserial` importable there. Both robot programs use `/dev/serial0`; verify that it points to the BusLinker UART on each Pi.

## Parked head tracking

1. Stop the laptop follow controller, then stop the drive server on each robot. Stop any other programs controlling the wheels.
2. From the repository root on the robot, run `/usr/bin/python3 head_tracker/dashboard.py`, or start the installed service for this copy. Open `http://ROBOT_IP:8080/`.
3. Engage torque and start head tracking as described in the tracker README. Keep the browser tab visible. In this mode, the moving markers should not be used for chassis navigation.

## Vision-guided driving with the head at neutral

For this procedure, leave `head_pose` unset in the laptop configuration. Use the simultaneous tracking procedure below for calibrated moving markers.

1. Stop tracking on all robots visible to the overhead camera. Put each speaker in its calibrated neutral pose, level and facing the chassis front, and engage both servos to hold it there. Use the dashboard's position capture to record that robot's neutral pan and tilt in servo units. Keep the mount supported if you disengage torque for manual positioning.
2. Stop the dashboard process or service, and close other servo and wheel controllers. With the wheels lifted for the first run, start the integrated drive server from the repository root. Replace the uppercase placeholders with measured or configured values.

   ```sh
   ~/capstone-base/.venv/bin/python drive_with_head_lock.py \
     --neutral PAN_UNITS TILT_UNITS -- \
     --robot-id ROBOT_ID --allow-from LAPTOP_IP
   ```

   The wrapper enables hardware and passes the arguments after `--` to the handoff drive server. If needed, pass the robot's `--hiwonder-config`, `--counts-per-meter`, and `--turn-radius` there as well. The wrapper requires valid in-tolerance positions and engaged torque from both servos before starting. It holds exclusive access to the UART and monitors feedback during operation. A missing reply, loss of torque, or out-of-tolerance position stops the wheels and exits the drive server. Recheck neutral before restarting.
3. Start or restart `vision.main` on the laptop so it reacquires the markers in the neutral pose. Verify robot IDs, reported positions and headings, and the user position. Run `python -m control.main` for a dry run before following the runbook's armed tests.
4. To return to parked tracking, stop the laptop controller and the robot drive server before restarting the dashboard. Repeat the neutral and vision checks before every drive session.

For UDP simulation, use `python3 -m robot.drive_server` with the handoff's normal options and a valid wheel-mapping file. Direct hardware use of the handoff server bypasses the neutral checks, so use the wrapper for integrated hardware runs. Leave other serial tools closed during drive mode.

## Hardware-free checks

```sh
python3 -m unittest discover -s tests -p 'test_tracking_control.py' -v
python3 -m unittest discover -s tests -p 'test_head_lock.py' -v
python3 -m pytest -o pythonpath=. tests/test_drive_server.py
```

## Face tracking while driving

Use this mode after measuring the marker and servo geometry of each robot. The dashboard owns the servo UART and sends encoder readings to laptop UDP port 5006. Vision uses the readings that match the overhead frame's capture time and projects each marker corner at its corrected height. The wheel server requires a compensated pose on each drive command.

### Measure and configure each robot

Use a level chassis with the speaker level and facing forward. Define the chassis origin on the floor directly under the point used as the robot's drive center. Positive X is forward, positive Y is left, and positive Z is up. Measure each marker's center and the two pivot locations from that origin. Update `faces` with each marker's neutral outward yaw. Mount each calibrated marker vertically and upright at neutral.

Add a `head_pose` block inside each corresponding `robots:` entry in the laptop's `config.yaml`. The numbers below illustrate the format. Replace every neutral reading, angle scale and dimension with the corresponding measurements. Use that robot's actual marker IDs.

```yaml
head_pose:
  neutral_units: [500, 500]          # pan, tilt encoder readings at neutral
  degrees_per_unit: [0.24, -0.24]   # signed angles per encoder unit, measured at the mount
  pan_pivot_m: [0.02, 0.0, 0.30]
  tilt_pivot_m: [0.04, 0.0, 0.34]
  markers:
    1: {mount: pan, center_m: [0.02, 0.15, 0.30]}
    3: {mount: pan, center_m: [0.02, -0.15, 0.30]}
    2: {mount: tilt, center_m: [-0.13, 0.0, 0.34]}
```

- Positive pan rotates the speaker from forward toward chassis left. Positive tilt rotates about neutral chassis +Y, taking the forward direction down. Compare actual angle changes with changes in encoder readings to establish each sign and scale.
- `pan` follows the pan joint only. `tilt` follows the tilt joint and its parent pan joint. If the rear tilt assembly is attached directly to the chassis and does not follow pan, use `tilt_only`. Use `fixed` for a calibrated chassis-mounted marker. Markers omitted from the block, such as an unmeasured front marker, are ignored for this robot.
- Compensation assumes a vertical pan axis, a lateral tilt axis, and a chassis that stays level. Check the setup with the chassis stationary and the head moving through both axes separately before testing combined motion.

### Start and verify

1. Synchronize the laptop, the master Pi and both robot Pis to a common time reference. The network runbook describes local chrony setup for the Pis. Check clock offsets before enabling this mode because overhead frame capture and encoder feedback come from different machines. Allow robot traffic to UDP 5006 on the laptop.
2. On each robot Pi, from the repo root, start the dashboard with its own ID. Stop any existing dashboard service first. Use the existing dashboard to engage torque and start tracking.

   ```sh
   /usr/bin/python3 head_tracker/dashboard.py --pose-host 192.168.1.10 --robot-id 1
   ```

3. On the laptop, restart `vision.main --gui` with the measured configuration. Keep the chassis stationary and pan and tilt while checking the GUI. The reported chassis center and heading should hold steady as different markers come into view. Run `python -m control.main --robots 1` and check that poses are usable. If it reports `compensated pose unavailable`, check clock offsets, source IPs, UDP 5006, servo feedback and marker calibration.
4. With wheels lifted, start the drive server in simulation using the command below without `--hardware` and verify that `control.main --robots 1 --arm` produces commands. Confirm that blocking feedback or hiding all calibrated markers stops the commands. Then repeat with `--hardware`, one robot at a time. Keep the wheel and servo controllers in separate processes.

   ```sh
   ~/capstone-base/.venv/bin/python -m robot.drive_server \
     --robot-id 1 --allow-from 192.168.1.10 --require-compensated-pose --hardware
   ```

   Use ID 2 for Robot 2. This mode uses the standalone wheel server with the compensation flag. The neutral-only wrapper reserves the UART for its own checks and is used for the neutral drive procedure above.
