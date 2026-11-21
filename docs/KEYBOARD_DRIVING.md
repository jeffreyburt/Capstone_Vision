# Individual robot keyboard control

Open http://localhost:8090/ and use **Keyboard driving** at the top. Select Robot 1 or Robot 2,
choose a speed (default 0.23 m/s; up to 0.97 m/s), and press **Enable keyboard control**.

- W: chassis forward; S: backward; A: sideways left; D: sideways right.
- Hold two keys for diagonals: W+A, W+D, S+A, or S+D. Diagonal speed is normalized.
- Release keys to stop. Opposing keys cancel each other.
- Space clears movement; **Stop / disable** exits keyboard mode.
- The eight direction buttons also work: hold a button with mouse/touch to move, release to stop.

Only the selected robot is commanded to move. The other robot receives stops. Changing robots
requires enabling again. Direction is relative to the robot chassis, not the camera or room grid.
Speed is capped by each robot's existing limit (WASD: 0.53 m/s on both robots; see the 2026-11-21 sections below).
Typing in input fields does not move a robot. Leaving the tab, losing browser focus, or losing
the dashboard connection disables the keyboard session; it must be enabled again to resume.

The browser renews a short command lease every 100 ms. Missing renewals revoke the session after
0.4 s; the existing controller polls at 10 Hz and the robot's 0.5 s watchdog remains active.
Requests carry a session token and increasing sequence number; delayed packets cannot restart an
expired or replaced session. The SSH gateway also requires fresh robot acknowledgements.

Enabling clears waypoint destinations and disarms click driving. Waypoint changes are rejected
while keyboard mode is active. Motor diagnostics disable keyboard mode before taking ownership;
keyboard mode cannot be enabled during a motor test. Disabling keyboard mode never resumes old
waypoints. Camera pose is not required for this explicitly operator-controlled mode.

## Components and deployment

The existing `python -m control.main --robots 1,2 --waypoints --arm` process remains the sole
laptop wheel-command sender. It reads keyboard state from `/api/navigation`, prioritizes the
manual session, and sends a distinct `manual` UDP message through the existing SSH gateway.
Autonomous `drive` messages still require compensated camera pose. Robot drive servers accept
`manual` from their existing allowed main-Pi address, with the same speed clamp, acceleration,
and watchdog as other motion. The UI shows controller/link status rather than assuming an HTTP
acknowledgement means a motor moved.

Files: `control/manual.py`, `monitor/server.py`, `monitor/page.html`, `control/main.py`,
`control/link.py`, `control/ssh_gateway.py`, and `robot/drive_server.py`.
The updated `robot/drive_server.py` must be copied to `~/capstone-drive/drive_server.py` on each
robot and its `capstone-drive-<id>.service` restarted. Restart the local monitor/controller to load
updated Python code. Opening the dashboard or restarting these programs does not enable driving.

Robot 1 now uses [calibrated mixed four-wheel driving](FOUR_WHEEL_DRIVE.md). Its replacement
front-right motor follows the saved speed profile. All wheels are capped at 70 command units (was 5),
and a requested nonzero FR speed below 2 holds the whole chassis. Robot 2 is unchanged.

2026-11-21: deployed to both robot Pis after the main Pi was power cycled to restore SSH.
Both drive services reported keyboard capability and zero motor outputs. The local monitor and
sole armed controller were restarted with keyboard and waypoint driving disabled. Live verification
exercised a zero-key browser session and zero-velocity manual packets to both robots, then disabled
the session. No nonzero physical movement commands were sent while implementing this feature.
The Enable button waits for a connected, armed controller and a robot advertising keyboard support.

## 2026-11-21: higher keyboard speed limits

Change made to allow much faster WASD driving. **Not yet deployed to the robots or run on hardware;
unit tests could not be run in the authoring environment (missing numpy/cv2/pytest).**

| Setting | Before | After | File |
| --- | --- | --- | --- |
| Keyboard speed range | 0.02–0.12 m/s | 0.02–0.19 m/s | `control/manual.py`, `monitor/page.html` |
| Dashboard default speed | 0.05 m/s | 0.15 m/s | `control/manual.py`, `monitor/page.html` |
| Robot 2 `max_speed_m_s` | 0.12 | 0.19 (~0.192 m/s = 25 counts/10 ms at 13000 counts/m) | `config.yaml` |
| Robot 2 `max_yaw_rad_s` | 0.30 | 0.60 | `config.yaml` |
| Drive server `--accel` default | 40 counts/10 ms per s | 100 | `robot/drive_server.py` |

- **Robot 1 is unchanged (0.038 m/s).** Its replacement FR motor is only calibrated for command units 2–5, and
  all wheels are capped together at 5; going faster needs a new calibration at higher speeds first.
- Robot 2's real ceiling is the drive server wheel cap (`--max-counts`, default 25; its wheel config allows 30,
  about 0.23 m/s). To go faster, raise `--max-counts` on its service and `max_speed_m_s` together.
- The `--accel` default also applies to autonomous driving on both robots.
- Deploy: copy `robot/drive_server.py` to `~/capstone-drive/` on each robot, restart
  `capstone-drive-<id>.service`, and restart the local monitor/controller. Do the first test with the
  robot lifted or in open space.

Deployment, 2026-11-21 (all tests passed beforehand: 223 passed, 10 subtests):

- Live `~/capstone-drive/drive_server.py` backed up on both robots as
  `drive_server.py.bak-2026-11-21-prespeed`, then replaced with this commit's file.
- **Robot 1:** `capstone-drive-1.service` restarted; logs show cap 5 counts/10 ms and zero outputs.
- **Robot 2:** file copied, but **the service was NOT restarted**. `capstone-pi2` has no passwordless sudo
  rule for it (see MOTOR_DIAGNOSTICS.md), so it still runs the previous file until someone runs
  `sudo systemctl restart capstone-drive-2.service` on it. The old file already clamps at 0.3 m/s with
  the same 25-count cap, so 0.19 m/s works now; only the faster acceleration waits on the restart.
  Robot 2's previous live file predated the mixed-drive change; the new file's mixed path only runs
  when the config has a `tb6612` section, which `robot2_wheels.json` does not.
- Laptop `monitor` and `control.main --robots 1,2 --waypoints --arm` restarted from `.venv`; dashboard
  came back disarmed with keyboard mode off and both links OK. No motion commands were sent.
- Rollback: copy the `.bak-2026-11-21-prespeed` file back over `drive_server.py` and restart the service.

## 2026-11-21: keyboard speed raised to the maximum

Robot 2 now runs at its configured motor ceiling. **Not yet tested with the robot moving.**

| Setting | Before | After | File |
| --- | --- | --- | --- |
| Keyboard speed range | 0.02–0.19 m/s | 0.02–0.23 m/s | `control/manual.py`, `monitor/page.html` |
| Dashboard default speed | 0.15 m/s | 0.23 m/s | `control/manual.py`, `monitor/page.html` |
| Robot 2 `max_speed_m_s` | 0.19 | 0.23 | `config.yaml` |
| Drive server `--max-counts` default | 25 | 30 counts/10 ms | `robot/drive_server.py` |

- 0.23 m/s is the most Robot 2 can reach without changing the team's motor config. The drive server uses the
  smaller of `--max-counts` and `max_speed_counts` in `~/capstone-base/config/robot2_wheels.json` (30).
  30 counts/10 ms at 13000 counts/m (still an estimate) is about 0.231 m/s. Diagonals and turning are
  scaled down so no wheel exceeds 30.
- Going further would need raising `max_speed_counts` above 30 in Robot 2's config (the Hiwonder board accepts up
  to 127). The motors' real top speed under load has not been measured, so that needs a supervised test first.
- **Robot 1 is unchanged at 0.038 m/s.** The new `--max-counts` default does not affect it: its calibration
  still caps every wheel at 5.
- Robot 2 uses the new 30-count cap only after `capstone-drive-2.service` is restarted (needs sudo);
  until then its old file caps it at 25 counts, about 0.19 m/s.

## 2026-11-21: Robot 2 WASD cap raised to 0.97 m/s

User-requested. **Not yet tested with the robot moving; Robot 2's real top speed under load is unknown.**

| Setting | Before | After | File |
| --- | --- | --- | --- |
| Keyboard speed range | 0.02–0.23 m/s | 0.02–0.97 m/s (dashboard default stays 0.23) | `control/manual.py`, `monitor/page.html` |
| New `keyboard_max_speed_m_s` limit | none | Robot 2: 0.97; unset (0) means use `max_speed_m_s` | `control/controller.py`, `config.yaml` |
| Robot 2 `max_speed_m_s` (waypoints/autonomous) | 0.23 | 0.23, unchanged | `config.yaml` |
| Robot 2 `max_speed_counts` | 30 | 127 | `robot/config/robot2_wheels.json` and the live `~/capstone-base/config/robot2_wheels.json` |
| Drive server `--max-counts` default | 30 | 127 | `robot/drive_server.py` |
| Drive server body clamp | hard-coded 0.3 m/s | new `--max-speed`, default 1.0 m/s | `robot/drive_server.py` |

- **Why 0.97, not 1.0:** the Hiwonder board takes wheel speed as a signed byte, so 127 counts/10 ms is the most it
  accepts. At 13000 counts/m (an estimate) that is about 0.977 m/s.
- **Motor saturation:** the JGB37-520 motors may top out below 127 counts/10 ms. If they do, the wheels can't all
  reach their targets, and fast diagonals or strafes may drift. Forward/back is least affected.
- **Stopping distance:** the browser lease (0.4 s) plus the robot watchdog (0.5 s) means at full speed the robot
  can travel roughly 0.5–0.9 m after input is lost. Space/Stop/key release stop sooner.
- **Ramp:** at the 100 counts/10 ms per s acceleration it takes about 1.3 s to reach full speed.
- Waypoint and autonomous driving keep their old limits; only WASD uses the new cap. Robot 1 is unchanged
  (0.038 m/s; calibration caps it at 5 counts).
- The live `robot2_wheels.json` on Robot 2 was backed up as `robot2_wheels.json.bak-2026-11-21-prespeed`.
- Robot 2 needs `sudo systemctl restart capstone-drive-2.service` before any of the new robot-side limits take
  effect. Until then it runs the original file (25-count cap, about 0.19 m/s).
- **Deployed 2026-11-21:** Robot 2's drive service restarted on the new file (log shows cap 127 counts/10 ms,
  wheels at zero). `capstone-pi2` has no passwordless sudo, so the restart was done by waiting until keyboard
  mode was off and the wheels had been at zero for 10 s, then sending SIGKILL to the service's own-user process;
  `Restart=on-failure` brought it back 3 s later. Never do this while the wheels are moving: the motor board
  keeps its last speed with no watchdog until the service is back. The earlier notes saying Robot 2 still
  needs a sudo restart are superseded.

## 2026-11-21: both robots matched at 0.53 m/s

**This replaces the 0.97 m/s cap above.** A lifted top-speed sweep of Robot 1 (see
[FOUR_WHEEL_DRIVE.md](FOUR_WHEEL_DRIVE.md#high-speed-extension-2026-11-21)) showed the Hiwonder
JGB37-520 motors track commands only up to about 70 counts/10 ms and saturate near 75-80. Robot 2
uses the same motor type, so its 127 cap was asking for speed the wheels can't deliver, and fast
diagonals would drift because different wheels saturate at different points.

| Setting | Before | After |
| --- | --- | --- |
| WASD cap (`keyboard_max_speed_m_s`) | Robot 1 none (0.038), Robot 2 0.97 | both 0.53 m/s |
| Keyboard speed range | 0.02–0.97 m/s | 0.02–0.53 m/s (default 0.23) |
| Robot 1 wheel cap | 5 counts/10 ms | 70 (calibration extended to full duty) |
| Robot 2 `max_speed_counts` (repo and live) | 127 | 70 |
| Drive server `--max-counts` default | 127 | 70 |
| Waypoint caps | Robot 1 0.038, Robot 2 0.23 | unchanged |

Robot 2's top speed was inferred from Robot 1's identical Hiwonder motors, not measured on Robot 2.
Neither robot has been driven at top speed on the floor.
