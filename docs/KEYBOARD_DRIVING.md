# Individual robot keyboard control

Open http://localhost:8090/ and use **Keyboard driving** at the top. Select Robot 1 or Robot 2,
choose a speed (default 0.05 m/s), and press **Enable keyboard control**.

- W: chassis forward; S: backward; A: sideways left; D: sideways right.
- Hold two keys for diagonals: W+A, W+D, S+A, or S+D. Diagonal speed is normalized.
- Release keys to stop. Opposing keys cancel each other.
- Space clears movement; **Stop / disable** exits keyboard mode.
- The eight direction buttons also work: hold a button with mouse/touch to move, release to stop.

Only the selected robot is commanded to move. The other robot receives stops. Changing robots
requires enabling again. Direction is relative to the robot chassis, not the camera or room grid.
Speed is capped by each robot's existing limit (Robot 1: 0.075 m/s; Robot 2: 0.12 m/s).
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

Robot 1 still uses its configured **three-wheel** chassis driver. The calibrated replacement
TB6612 motor remains available through individual motor tests and is not yet integrated into
four-wheel chassis navigation/manual driving. Sideways motion on that robot can consequently
be imperfect. Robot 2 retains its existing four-wheel mapping.

2026-11-21: deployed to both robot Pis after the main Pi was power cycled to restore SSH.
Both drive services reported keyboard capability and zero motor outputs. The local monitor and
sole armed controller were restarted with keyboard and waypoint driving disabled. Live verification
exercised a zero-key browser session and zero-velocity manual packets to both robots, then disabled
the session. No nonzero physical movement commands were sent while implementing this feature.
The Enable button waits for a connected, armed controller and a robot advertising keyboard support.
