# Robot MVP runbook: robots follow the user

> Current state of each robot, verified results and known problems: **[STATUS.md](STATUS.md)**.

**Goal:** the user walks around; the two speaker robots keep a listening triangle around them and turn to
face them.

```
Master Pi + camera ──Ethernet──► laptop: vision.main ──UDP 5005──► laptop: control.main
                                  (markers, YOLO)                   (targets + position control)
                                                                         │ UDP 5700, 10 Hz: body velocity
                                                                         ▼ ▲ status replies
                                                         robot Pi: robot/drive_server.py ──I2C──► wheels
```

**Decisions made for the MVP (2026-11-19), and how they differ from the whiteboard:**

- **The laptop closes the position loop.** It already sees every robot, so it computes each robot's velocity
  and sends it over UDP. The robot Pi only turns that into wheel speeds and stops by itself if commands stop
  for 0.5 s. This needs no encoder calibration and no clock sync between machines. Nate's robot-side
  position controller and dashboards are not used.
- **Pan/tilt stays at neutral; the whole robot turns to face the user.** The markers are on the speaker
  head, so if the pan/tilt moved, the camera would report the head's direction, not the chassis'.
  Face tracking is not part of the MVP.
- **The user's facing direction is assumed** (`control.formation.facing_deg`), because the camera only
  gives a position. The default (-90, toward the camera) keeps the robots between the user and the camera,
  where the markers are largest in the image.

## One-time setup

1. **Network** (`docs/NETWORK.md`). The master Pi runs the robot WiFi `capstone-robots`
   (`scripts/setup_robot_ap.sh`, done 2026-11-19). The robot Pis join it with static `192.168.50.11` / `.12`
   and gateway `192.168.50.1`; these match `config.yaml` `control.robots`. The laptop needs a route
   once (admin PowerShell): `route -p add 192.168.50.0 mask 255.255.255.0 192.168.1.20`. Check with
   `ping 192.168.50.11`.
   **New robot SD card** (Raspberry Pi OS Bookworm already on it; Pi 3 or Pi 4, e.g. robot 2's Pi 4): with the card in
   the laptop, `python robot/card/prepare_card.py --boot D:\ --robot 2 --bundle <dir>` (see its docstring for
   `<dir>`). It asks for the WiFi and login passwords. On first boot the Pi sets hostname `robot2-pi`, user
   `user` with the laptop's SSH key, the `capstone-robots` WiFi at `192.168.50.12`, I2C and the servo UART,
   installs Nate's code + the drive server under `~`, and reboots. A `robot-drive` service is installed but
   not enabled. Then skip step 2 for that robot.
2. **Copy the drive server to each robot Pi** (it reuses Nate's Hiwonder driver from `~/capstone-base`):
   ```bash
   scp robot/drive_server.py user@192.168.50.11:~/
   ```
3. **Marker geometry on the speaker head.** With pan/tilt at neutral, measure and set `robots:` in
   `config.yaml`:
   - `faces`: marker ID -> face direction, where **0 = the speaker's front** (90 left, 180 back, 270 right).
     The drive server's "forward" is this same face.
   - `center_offset_m`: marker surface to the pan axis (the point the camera reports as the robot).
   - `marker_center_height_m`: height of the marker centers above the floor.
4. **Remove the floor-calibration markers** (IDs 0 and 1 are also robot 1's IDs).

## Bring-up, in this order

Each step must look right before the next one. Keep the system GUI open the whole time
(`python -m monitor`, http://localhost:8090/): it shows every Pi, the drive servers, the head trackers,
vision and the control loop, and draws the control targets on its map.

1. **Vision** (laptop): `python -m vision.main --gui`, open http://localhost:8080/. Check that the robot dot
   is at the robot and its heading arrow points out of the speaker front.
2. **Drive server in simulation** (robot Pi; prints motor commands, touches nothing):
   ```bash
   cd ~/capstone-base && .venv/bin/python ~/drive_server.py --robot-id 1
   ```
3. **Dry run** (laptop; sends nothing): `python -m control.main --robots 1`. Once a second it prints the
   user, the robot pose, its target and the command it *would* send.
4. **Armed against the simulated drive server:** `python -m control.main --robots 1 --arm`. The status line
   should say `link ok`, and the robot Pi should print changing motor counts. Nothing moves yet.
5. **Hardware, wheels lifted.** First **map the wheels** on a new robot, one motor at a time (the GitHub
   default `teleop.json` was wrong for robot 2): on the robot,
   `.venv/bin/capstone-motor-test --motor-type 3 --channel N --speed 5 --duration 1` for N = 1..4, noting
   which wheel turns and which way. Write the result as a robot-specific file like
   [robot/config/robot2_wheels.json](../robot/config/robot2_wheels.json) (`wheel_channels` in FL, FR, RL, RR
   order; a sign of -1 where a positive command turns the wheel backward) and pass it with
   `--hiwonder-config`. Then start the drive server with `--hardware --allow-from 192.168.1.10` and, still
   lifted, check forward / strafe / rotate with `python tools/drive_once.py --robot N --vx 0.05 --seconds 20`
   (and `--vy`, `--wz 0.25`). Ctrl+C on either side stops it.
6. **Drive tests on the floor** (about 1 m clear around the robot). These check directions and calibrate
   speed; the tool prints what to change:
   ```bash
   python -m control.drive_test --robot 1 --vx 0.1 --seconds 3
   python -m control.drive_test --robot 1 --vy 0.1 --seconds 3
   python -m control.drive_test --robot 1 --wz 0.3 --seconds 4
   ```
   Apply the suggested factors to the drive server's `--counts-per-meter` (straight tests) and then
   `--turn-radius` (turn test), restart it and repeat until the ratios are near 1.0. A "PROBLEM" line means a
   wheel direction or the front face is wrong: fix the robot's wheel config or the `faces` in `config.yaml`.
   Notes from robot 2:
   - Put the robot 2–5 m from the camera; closer than ~1.5 m it is at the image edge and often not detected.
   - Use 0.10 m/s for the speed calibration: at 0.05 m/s the wheel counts round down (6.5 -> 6).
   - The suggested factor assumes the commanded counts; when start and end are seen on different marker
     faces, heading-based numbers (turns, "left"/"forward" split) can be off by ~10°.
   - **If the strafe test turns the robot instead of moving it sideways**, set `control.limits.allow_strafe:
     false` (no-strafe mode: turn, drive straight, turn to face the user) and check the rollers.
7. **Follow test, one robot:** stand **3–4 m out from the camera**, with nobody else in view if possible.
   Dry run first (`python -m control.main --robots 2`): check that the printed **user** position is you (the
   person tracker keeps whoever it locked onto first; restart `vision.main` to re-lock) and that the target
   is inside the camera's view. Then `python -m control.main --robots 2 --arm`. Keep a hand near the robot's
   power switch. **Space** pauses (robots stop), **q** or **Ctrl+C** quits (sends stop).
8. **Both robots:** start a drive server with `--robot-id 2` on robot 2, then `python -m control.main --arm`.

## What stops a robot

| Condition | Where |
|---|---|
| No command for 0.5 s (laptop quit or crashed, WiFi dropped) | drive server watchdog |
| Camera hasn't seen the robot for 0.7 s (`max_pose_age_s`) | control loop |
| User not seen for 2 s (`user_lost_s`), or no vision messages for 1 s | control loop |
| Within 0.05 m and 5 deg of its target (restarts at 2x that; 0.15 m in no-strafe mode) | control loop |
| Space (pause), q, Ctrl+C | control loop |

Speed is capped twice: `control.limits.max_speed_m_s` (0.15 m/s) on the laptop and the drive server's
`--max-counts` / 0.3 m/s. Within `keep_out_m` (0.8 m) of the user a robot never moves toward them; it can
still slide sideways around them. Nothing else is avoided (furniture, a second person).

## Tuning (`config.yaml` `control:`)

| Symptom | Try |
|---|---|
| Wiggles or overshoots around the target | lower `position_gain` / `yaw_gain`; or a faster birds-eye stream (full 12 MP runs at ~4 fps) |
| Too slow | raise `max_speed_m_s` (stay at or below 0.25) |
| Keeps stopping and restarting | its markers drop out: check the GUI; maybe raise `max_pose_age_s` a little |
| Robots end up too far away to detect | lower `distance_m`, or change `facing_deg` |
| Chases small movements of the user | raise `replan_m` |

## Known limits

- Pan/tilt must stay at neutral (see above). Pan aiming comes next. It needs the robot to report its pan
  angle so vision can subtract it.
- Straight-line moves only, no path planning. A robot whose path is blocked by the user slides along the
  keep-out circle or stops (in no-strafe mode it stops).
- No-strafe mode (`allow_strafe: false`) is a global setting for now: both robots turn-then-drive.
- Robot 1 always takes the user's left and robot 2 the right (`side` in `control.robots`).
- The planned targets are drawn in the system GUI (`python -m monitor`), not in the vision GUI.
