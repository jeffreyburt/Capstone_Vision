# Individual robot keyboard control

Open http://localhost:8090/ and use **Keyboard driving** at the top. Select Robot 1 or Robot 2,
choose a speed (default 0.15 m/s), and press **Enable keyboard control**.

- W: chassis forward; S: backward; A: sideways left; D: sideways right.
- Hold two keys for diagonals: W+A, W+D, S+A, or S+D. Diagonal speed is normalized.
- Release keys to stop. Opposing keys cancel each other.
- Space clears movement; **Stop / disable** exits keyboard mode.
- The eight direction buttons also work: hold a button with mouse/touch to move, release to stop.

Only the selected robot is commanded to move. The other robot receives stops. Changing robots
requires enabling again. Direction is relative to the robot chassis, not the camera or room grid.
Speed is capped by each robot's existing limit (Robot 1: 0.038 m/s; Robot 2: 0.19 m/s).
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
front-right motor follows the saved speed profile. All wheels are capped at 5 command units,
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
