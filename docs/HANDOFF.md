# Agent handoff: current robot system

Updated 2026-11-22. Start here; dated sections of older runbooks may describe superseded hardware.
This repository is shared by multiple workers. Git files and live service state, not chat history,
are the source of truth. See also [four-wheel drive](FOUR_WHEEL_DRIVE.md),
[keyboard driving](KEYBOARD_DRIVING.md), [motor diagnostics](MOTOR_DIAGNOSTICS.md),
[network](NETWORK.md), [head tracking](HEAD_TRACKER.md), and [Pi guide](PI_GUIDE.md).

## Machines and entry points

| Machine | Address / login | Role |
| --- | --- | --- |
| Mac | Ethernet 192.168.1.10 | Dashboard, main vision processing, sole body-command controller |
| Main Pi | `vision@192.168.1.20`; AP 192.168.50.1 | Main camera sender and robot Wi-Fi gateway |
| Robot 1 / left | `robot-pi1@192.168.50.105` | Pi 3 B+, mixed Hiwonder + TB6612 four-wheel chassis |
| Robot 2 / right | `capstone-pi2@192.168.50.12` | Existing Hiwonder four-wheel chassis and working head servos |

The Mac uses Ethernet to the main Pi and retains its Internet connection. **Do not switch
its Wi-Fi to the robot AP.** Existing SSH config uses ProxyJump via `vision@192.168.1.20`
for both robot IPs. SSH keys are already installed. Do not commit passwords/private keys or
weaken authentication to work around a deployment failure. The main Pi recently required a
power cycle after SSH rejected new connections before authentication.

Dashboard: http://localhost:8090/ (`python -m monitor`). Main vision:
`python -m vision.main --gui`. Sole chassis controller:
`python -m control.main --robots 1,2 --waypoints --arm`. This process's `--arm` permits sends;
the dashboard's keyboard/waypoint arming separately decides whether nonzero motion is requested.
Default dashboard state after restart is disarmed, with no keyboard lease or waypoint targets.

## Current drive hardware

Robot 1 logical wheel routing is FL=M3, FR=M1, RL=M4, RR=M2, signs `[+,+,-,+]` in that order.
**Logical M1 now means TB6612 channel A; damaged physical Hiwonder M1 remains zero.**
The mixed profile has no disabled logical channels and caps all wheels to 70 counts/10ms
(calibration extended to full duty 2026-11-21, see FOUR_WHEEL_DRIVE.md). Robot 1 waypoint speed
stays capped at 0.038 m/s; WASD may use up to 0.53 m/s; strafing is enabled. Commands requiring a
nonzero FR speed below 2 units hold the whole chassis. Do not remove that guard or raise the cap
above 70: the Hiwonder motors saturate near 75-80 counts/10ms. Robot 2 also caps at 70.

Canonical files:

- `robot/config/robot1_four_wheels.json`: routing, driver parameters, BCM pins, TB direction.
- `calib/motor_speed.json`: measured positive/negative speed curves and validation data.
- `robot/mixed_motors.py`: continuous mixed-driver owner and fault/low-speed behavior.
- `robot/motor_speed.py`: shared `SpeedMatch` and fractional `PrecisionPWM` implementation.
- `robot/services/capstone-drive-1-fourwheel.conf`: systemd drop-in for Robot 1.
- `robot/config/robot1_three_wheels.json`: preserved rollback/diagnostic configuration.
- `robot/config/robot2_wheels.json`: Robot 2's unchanged mapping.

All drive files are deployed in `~/capstone-drive/`. Drive units are
`capstone-drive-1.service` and `capstone-drive-2.service`, UDP port 5700, accepting motion from
the main Pi at 192.168.50.1. The four-wheel Robot 1 drop-in uses `/usr/bin/python3` plus
`PYTHONPATH=/home/robot-pi1/capstone-base/src`; its original unit/venv remains available for rollback.

Both robots were explicitly confirmed **on the floor** for this integration. No physical
motion was performed for deployment verification. Installed FR direction (+1 in config),
loaded speed matching, and floor mecanum behavior still need a supervised physical check.
Prior lifted-wheel measurements are saved in `calib/recordings/robot1_motor_speed_*.json`.

## Dashboard and ownership

- **2026-11-22 afternoon:** merged Jeffrey's `feature/fov-soft-stop` (camera-view guard; see below). Robot 1 had
  stopped driving because its drive server had latched "TB6612 encoder pulses stopped" (most likely while its 7.4 V
  supply was off: no FR power, no encoder pulses); restarting `capstone-drive-1.service` clears it. Vision's head
  feedback now re-opens a dead SSH tunnel (Robot 2 had stayed "Fresh head feedback unavailable" after a main-Pi
  reboot). With no heading history and no full fit, a single tag's own orientation now gives the heading
  (`position_source: one tag`), but only while the head is within 35 deg of forward (`SINGLE_TAG_MAX_PAN_DEG`);
  further aside the robot waits for a heading rather than risk turning on a bad one.

- **Camera view guard (branch `feature/fov-soft-stop`, 2026-11-22; software and simulation only, not yet run on
  the robots):** robots were leaving the main camera's image. `vision.main` now adds `view.polygon` to every
  message: the floor area where a robot's tags (heights = each robot's marker height +- half a tag) appear inside
  the image, 2% of its width from the edges. It is computed from the existing lens + floor calibration, so there
  is **no extra calibration step**; it is re-derived whenever vision reloads its geometry. Because the main camera
  is only ~0.33 m high, almost level with the tags, the view is a ~±50° wedge (the calibrated lens model stops
  being valid at 51.5° off axis, `lens_fold_radius`, so that also bounds it). With the 2026-11-22 calibration the
  tag view spans x -0.63..+0.74 m at y = 0.6 m, -1.06..+1.25 m at 1.0 m and -1.61..+1.89 m at 1.5 m, so near the
  camera it is narrower than `x_limits_m` (±0.9). The floor map draws it as a dotted blue outline.
  `control/view_guard.py` post-processes every mode's commands in `control.main`:
  - *Soft stop (all modes, including WASD):* for a robot the camera sees, the velocity part pointing out of the
    view falls linearly from full speed at `slow_m` (0.50 m) inside the edge to zero at `stop_m` (0.20 m); motion
    along the edge or inward is untouched. No-strafe chassis scale their forward speed instead. Simulated on the
    real calibration with camera delay: 0.30 m/s with 0.3 s delay stopped at 0.20 m; 0.53 m/s with 0.4 s delay at
    0.18 m. Waypoints beyond the edge stop at it (then "no camera progress" after 6 s). Needs a heading; a robot
    without one is not limited.
  - *Return (Follow, armed waypoints, plain `control.main`; never keyboard, paused, or without live calibrated
    vision):* a robot the camera has lost whose dead-reckoned estimate (last camera pose + commands actually sent
    since; now kept in every mode) is within `stop_m` of the edge or outside it drives at 0.10 m/s, heading held,
    toward a point 0.15 m inside the stop line, still avoiding the user and the other robot. It stops when the
    camera sees it again or when the estimate says it is back inside ("waiting for the camera").
  - *Watchdog:* returning ends for good once the last camera pose is older than `return_timeout_s` (4 s) or the
    estimate has moved `return_max_m` (0.6 m); the robot then stays stopped with "lost outside the camera view ...
    bring it back into view by hand or keyboard" until it is seen again.
  - Settings: `control.view_guard` in `config.yaml`. **Deploy:** restart `python -m vision.main --gui` (publishes
    the view) and the sole `control.main` on the Mac; without a `view` in the vision message the guard does
    nothing. **Rollback:** `view_guard.enabled: false` and restart both, or revert the branch.
  - Tests: `tests/test_view_guard.py` (polygon vs direct projection, soft stop, return, both watchdogs).
    Unverified on hardware: whether 0.20 m keeps the head tags detectable at the edge (tag 2/6 sit 0.10 m behind
    the centre and swing with the pan), and how far dead reckoning drifts in 4 s on the floor. First check: drive
    one robot by WASD toward a side edge at low speed and confirm it holds with its tags still detected.

- **Line formation (2026-11-22):** the robots stay on a line `control.formation.line_y_m` (1.5 m) in front of the
  camera and the user is the third corner of an equilateral triangle (side = 2h/sqrt(3), h = user's distance beyond
  the line), so it opens as the user walks away and closes as they come nearer. If the robots would cross
  `x_limits_m`, the side is capped and they step forward toward the user (still equilateral), returning to the line
  when it fits; the side never drops below `min_side_m` (0.8 m). The line distance can be set on the Calibration
  page's step 3. Without `line_y_m` the old fixed-size triangle (`distance_m`) is used.
- **Heads look up first (2026-11-22):** until a face has been seen, head tracking tilts up by `search_up_units` (100,
  about 24 deg) from where it started ("up" is the opposite of each robot's tilt direction); the main camera's aim
  takes over whenever it sees the user and the robot. Turning on **Follow user** also starts head tracking on both
  robots (engaging torque first).
- **Calibration run from the dashboard 2026-11-22 11:43 / 11:47:** floor height fitted 0.326 m vs the 0.37 m entered
  (passed the 5 cm check; the advanced checklist flags > 3 cm), then robots and Robot 2's head sweep. Previous files
  are in `calib/history/20261122_1143*` and `..._1147*`.

- **Main camera timestamps after a main-Pi reboot (2026-11-22):** the receiver kept the smallest laptop-minus-Pi clock
  offset ever seen, so after the main Pi rebooted with a different clock every frame looked ~18 h old and vision
  stalled. `camstream/receiver.py` now re-learns the offset when the sender restarts or 5 frames in a row disagree by
  over 1 s.

- **Follow user (2026-11-21):** the dashboard's floor-map panel has a **Follow user** switch (off after every
  monitor restart). With it on, `control.main --waypoints` keeps both robots in an equilateral listening
  triangle with the user (`control.formation`: 1.0 m sides, facing -90° = toward the camera, down the hallway).
  Robot centres are clamped to the hallway, `x_limits_m: [-0.52, 0.55]` (walls at -0.87 / +0.90 m, 0.35 m
  clearance) and `y_limits_m: [0.6, 6.0]`. Turning on click driving or keyboard control turns Follow off.
  Robot sides were swapped to match the physical placement: Robot 1 is on the camera's left (`side: right`,
  the user's right when facing the camera). Follow speed is 0.15 m/s for both robots.
- **Wider head tracking (2026-11-21):** Robot 2's head range and tracking limits are pan 141-641 (~±68°) and tilt
  450-812 (inside the owner-taught 433-812). Rear tag 6 is limited to its measured range only for the full fit
  (`markers.6.valid_units`); outside it, it still gives a tag-centre position. Tag-size (PnP) distances are scaled
  by `markers.size_distance_scale: 1.09` (they read 8-10% short against the floor-anchored fit at 2304x1296).
  Tag 5's centre was re-measured over pans 430-620 with the chassis still: (-0.057, 0.166, 0.227) m. Held still at
  pans 200-600, Robot 2 was located in every frame: x within 5 cm, heading within 9°, distance within ~17 cm.
- **Speed and up/down head tracking (2026-11-21):** the main camera now streams 2304x1296 (2x2-binned full field,
  same lens calibration scaled) at up to 30 fps instead of 4608x2592 at 10: vision runs ~13 fps (was 3-5) and a frame
  reaches the dashboard in ~0.16 s. The master Pi reads its own `~/Capstone_Vision/config.yaml` (backup
  `config.yaml.bak-2026-11-21-fullres`). Pose tolerances halved with the resolution (Robot 1 5 px, Robot 2 8 px).
  Control runs at 20 Hz; follow speed 0.30 m/s, position gain 1.2, Robot 1 turn rate 0.5 rad/s, formation replans
  after 0.10 m, less position smoothing. Robot 2's tilt range is 560-800 (was 690-785) so it can look up at a standing
  face; its rear tag 6 is used only inside its measured tilt range 690-785 (`markers.6.valid_units`). Tag-centre
  positions now take their distance from the tag's apparent size (PnP) instead of intersecting a height plane, which
  at this camera height moved ~15 cm per 1 cm of tag height (~30 cm when the head tilted up).
- **Robot positions without a full tag fit (2026-11-21):** with two or more of a robot's head tags visible, each
  tag's centre ray (at its known height) gives a position and the direction between the tags gives the heading;
  with one tag, the full image-space fit is used if it passes; otherwise the tag-centre ray with the tracker's
  last heading. Extra tags: Robot 1 side tag 3 (nominal geometry) and Robot 2 side tag 5 (2026-11-20 fit), both
  `center_only`. The vision message reports `position_source` (fit / two tags / tag centre). When no tag is seen,
  follow mode dead-reckons each robot on its own sent commands at half speed for up to `control.dead_reckon_s`
  (1.5 s), then stops.
- **Turned robots lost their pose (2026-11-21):** facing the user turns each chassis ~30°, which made the rear-tag
  fits worse (Robot 1 8.2 px, Robot 2 14.7 px with its head panned) and Robot 2's pose went stale; a backwards
  mirror solution (-39° vs 114°) also scored almost as well. Now `formation.max_turn_deg: 20` limits how far a
  chassis turns toward the user (Robot 2's tracking head covers the rest), `head_pose.heading_window_deg:
  [90, 60]` rejects fits not facing roughly down the hallway, and tolerances are Robot 1 10 px / Robot 2 15 px
  (these supersede the 8 / 10 px below). After the change both robots were located in 58-60 of 60 frames.
- **Head wobble tolerance (2026-11-21):** pose fits accept up to `head_pose.max_rms_px` (Robot 1: 8 px,
  Robot 2: 10 px; default 3). Robot 2 now uses only its rear tag 6 (side tag 5 gave >100° heading errors) and
  its head range is pan 311-471 / tilt 690-785. Head tracking from the dashboard is limited to that range by
  `system.robots.2.head_tracking_limits`, so Robot 2 stays located while its head follows the user. Measured
  stationary: both robots located in 76 of 77 frames, under 1 mm / 0.2° jitter.

- **Main Pi restarts (2026-11-21):** `control.main`'s wheel commands travel over one SSH connection through
  the main Pi. It used to stay dead after the main Pi rebooted, so both robots showed "drive server silent"
  and keyboard driving stayed disabled until `control.main` was restarted. `control/ssh_gateway.py` now
  reopens it automatically (at most every 2 s). A new connection has no robot acknowledgements, so
  motion stays replaced by stops until each robot answers again. Verified by killing the live gateway SSH
  process: both links returned to "link ok" within about 5 s. The camera stream to vision also recovers
  by itself once the main Pi's camera sender is back (allow about a minute after a reboot).

- **Branch `feature/face-res-and-feedforward` (2026-11-21; merged into main 2026-11-22). Feed-forward now also runs in
  the dashboard's Follow mode, together with dead reckoning; not yet driven on hardware:**
  - *Follow-mode feed-forward* (`control.feedforward` in `config.yaml`, `user_feedforward` in `control/main.py`):
    while the user walks faster than 5 cm/s, each robot's command gets the user's measured velocity added
    (capped at 0.15 m/s; the robot's `max_speed_m_s` still caps the total), targets re-plan every tick from where
    the user will be 0.4 s ahead, and standing still keeps the `replan_m` hysteresis. In no-strafe mode only the
    part along the chassis is added, once aligned. Waypoint mode is unchanged. Simulated with 0.5 s camera delay
    at 4 fps (`tests/test_feedforward.py`): mean error behind a user walking 0.05–0.10 m/s fell from 24–31 cm to
    2.5–6.4 cm. Turn off with `enabled: false`. Untested with real user-velocity noise; if robots twitch while
    the user stands, raise `min_user_speed_m_s`.
  - *Smaller head-tracking camera frames*: see [head tracker](HEAD_TRACKER.md) "Smaller robot camera frames".
  - On Windows, `tests/test_calibration.py`, `test_heads.py`, `test_monitor.py`, `test_motor_diagnostic.py`,
    `test_waypoints.py` (import `fcntl`), `test_manual.py`, `test_motor_speed.py` and `test_ssh_gateway.py` fail on
    `main` as well; run the full suite on the Mac.
- WASD manual mode selects one robot; simultaneous keys give normalized diagonals. Release,
  Space, tab blur, lost connection, or Stop end movement. The 0.4 s server lease and robot
  0.5 s watchdog stop stale input. `control/manual.py` owns token/sequence/expiry rules.
- `control.main` is the sole laptop wheel sender; do not start a second controller on UDP 5005.
  `control/ssh_gateway.py` relays through the main Pi, with fresh-ACK gating.
- `manual` UDP commands explicitly bypass camera pose requirements. Autonomous `drive` still
  requires compensated pose. Both retain the same robot source-IP restriction/watchdog.
- Motor tests in `monitor/motors.py` stop the robot's drive service before claiming GPIO/I2C,
  serialize via `/tmp/capstone-motor-diagnostic.lock`, then restore it. Do not run standalone
  GPIO tools against an active mixed driver. A file lock alone does not pause that service.
- Main camera and robot feeds remain on the dashboard. Head tracking is independent of chassis
  driving. Robot 1's head servos remain unavailable; its head is mechanically secured.

## Vision/head calibration

Saved camera intrinsics, floor pose, and head geometry remain in `calib/`; do not overwrite
or recalibrate on reboot. The floor-pose filename includes historical height terminology:
`calib/floor_pose_36cm.yaml` is the saved active floor calibration. User lens height was 39.5 cm;
floor tags were 15 cm with about 1.03 m spacing; robot tags have 10 cm black squares.

Robot 1 uses its secured fixed-head rear marker. Robot 2 uses the fitted articulated head
model and timestamped servo feedback; do not treat its moving head marker as a fixed chassis
marker. `calib/robot1_head_pose.yaml` and `calib/robot2_head_pose.yaml` plus config loading
are authoritative. Use [head tracker documentation](HEAD_TRACKER.md) and current files for
actual ranges/transforms. This drive integration changes none of that geometry.

## Multi-agent work checklist

1. Read `AGENTS.md`, this handoff, and the subsystem document; inspect `git status`, recent
   commits, live unit definitions, and existing process arguments before changing them.
2. Agree on file ownership/task boundaries. Do not overwrite another worker's files or commit
   their unrelated modifications. Avoid concurrent motor/servo tests and deployments.
3. Preserve canonical calibration and raw evidence. Back up live robot files before deployment.
4. Disable dashboard keyboard/waypoint modes and stop the sole controller before drive deployment.
   Keep camera/head processes intact unless the task actually needs to change them.
5. Verify without motion first: full pytest suite, service status, backend telemetry, zero
   effective commands, physical Hiwonder M1 zero, TB duty zero, no fault. Record physical test
   conditions explicitly; passing software tests does not establish wheel direction or floor motion.
6. Update these documents and relevant calibration/config when behavior changes. Record unresolved
   checks, deployment/rollback steps, and tests in the commit/PR description.
7. Fetch/rebase before pushing shared `main`, preserving other agents' commits. All changes requested
   in this project should be committed and pushed. Leave the working tree understandable for the next agent.

## Useful verification commands

```sh
.venv/bin/python -m pytest -q
ssh robot-pi1@192.168.50.105 'systemctl status capstone-drive-1.service --no-pager'
ssh capstone-pi2@192.168.50.12 'systemctl status capstone-drive-2.service --no-pager'
```

For a UDP status query, run from the main Pi and send `{"type":"status"}` to robot UDP 5700;
this is read-only. `mixed_drive` in Robot 1 replies exposes backend, real Hiwonder commands,
TB target/actual rates, PWM, low-speed guard and latched fault. The dashboard APIs
`/api/navigation`, `/api/motors`, and `/state` are also read-only GETs. Do not confuse a target,
a stale ACK, or an HTTP success with a confirmed physical movement.
