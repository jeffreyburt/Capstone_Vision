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
