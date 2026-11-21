# Dashboard motor diagnostics

Open http://localhost:8090/ and use **Individual motor tests** at the top.
Select a robot, controller channel, direction, speed, and duration, then press
**Run selected motor**. Lift chassis wheels first. This is a hardware test independent
of marker visibility and floor calibration. No test runs merely by opening the dashboard.

- Robot 1: Hiwonder M2–M4 and replacement TB6612 channel A. Hiwonder M1 remains disabled.
- Robot 2: Hiwonder M1–M4.
- Speed: 1–50%, signed by the direction selector. Hiwonder percentages scale its configured
  `max_speed_counts` (rounded to at least one count/10ms); TB6612 uses PWM duty directly.
- Duration: 0.2–5 seconds. **Stop test** ends the current run early.
- Hiwonder displays all four signed counters, change since start, and measured rates so
  unexpected cross-channel activity is visible. Counter wrap is handled.
- TB6612 displays rising/falling edge totals on C1/BCM22 and C2/BCM27, edge rates, and current
  logic levels. These unsigned edge totals are not directly comparable to Hiwonder counts.
- Live values update approximately ten times per second and the chart plots changes over time.
  A stale-data message means the displayed sample is no longer fresh. Zero pulses are reported
  explicitly, but do not alone establish that an encoder is damaged.

Starting a test clears waypoints and disables click driving. The runner stops the normal
robot drive service, holds a per-robot file lock for the entire diagnostic, commands only the
selected channel, stops outputs on completion/error, then restores the drive service only if
it was originally active. Navigation stays disarmed after the test. SSH stdin EOF or a Stop
message ends the run; the remote duration limit also bounds a disconnected test. No sudo
passwords are stored or passed by the dashboard.

## Deployment

Copy `robot/motor_diagnostic.py` to `~/capstone-drive/motor_diagnostic.py` on each robot.
System Python needs `smbus2`; Robot 1 also needs `gpiozero` and `lgpio`. The Hiwonder driver
is loaded from `~/capstone-base/src`. Existing controller configuration paths:

- Robot 1: `~/capstone-drive/robot1_three_wheels.json`
- Robot 2: `~/capstone-base/config/robot2_wheels.json`

The dashboard uses configured robot SSH users/addresses and this Mac's existing ProxyJump
configuration. No new listener or Pi network port is required.

An administrator must install this narrowly scoped sudoers entry on Robot 1 (validate with
`visudo -cf` before installing as root-owned mode 0440 in `/etc/sudoers.d/`):

```
robot-pi1 ALL=(root) NOPASSWD: /usr/bin/systemctl stop capstone-drive-1.service, /usr/bin/systemctl start capstone-drive-1.service
```

For Robot 2 use:

```
capstone-pi2 ALL=(root) NOPASSWD: /usr/bin/systemctl stop capstone-drive-2.service, /usr/bin/systemctl start capstone-drive-2.service
```

2026-11-21: Robot 1 runner and explicitly user-authorized sudoers entry installed. Robot 2 was
unreachable through the main Pi; its runner/permission deployment remains pending. Selecting
Robot 2 reports the SSH connection error. No physical motor runs were performed for this UI change.

Earlier diagnostics confirmed both Pi GPIO inputs respond to internal pulls and jumper-to-ground
tests. Powered motor tests still returned zero encoder pulses; encoder supply, ground, wiring,
and encoder electronics remain under investigation. See [TB6612 wiring](TB6612_TEST.md).

## TB6612 speed compensation

The dashboard now applies a saved TB6612 speed trim, initially **50%**. A requested 25% test
therefore applies **12.5% PWM duty**, in either direction. Hiwonder commands are unchanged.
Adjust **TB6612 speed trim** (10–100%) and press **Save trim** between tests; the dashboard
reports both the saved trim and actual applied PWM duty. Settings persist on the Mac in
`calib/motor_speed.json` and are passed to the Pi with each test. Direct runner calls default
to the same initial 50% unless `tb6612_scale` is supplied explicitly.

This is a provisional reduction, not a measured speed calibration or encoder speed regulator.
PWM duty and Hiwonder's closed-loop counts/10ms commands are different quantities; equal
percentages do not imply equal shaft speed. This adjustment affects motor diagnostics only:
Robot 1's navigation remains in its existing three-wheel mode. Do not interpret the trim as
integrating the replacement motor into chassis driving.

Updated wiring (2026-11-21): STBY=BCM25, PWMA=BCM18, AIN1=BCM23, AIN2=BCM24,
C1=BCM22, C2=BCM27. All four wheels are physically attached. This pin update does not
enable four-wheel navigation; TB6612 remains available through individual motor tests.
