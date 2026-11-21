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
  logic levels. Each phase is displayed separately; their sum is used as x4 count magnitude for matching, assuming identical encoder resolution and gearbox ratios.
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

The original fixed trim was provisional and is now superseded by the measured calibration below.
PWM duty and Hiwonder's closed-loop counts/10ms commands are different quantities; equal
uncalibrated percentages do not imply equal shaft speed. This adjustment affects motor diagnostics only:
Robot 1 now also uses the measured curves through its [mixed four-wheel driver](FOUR_WHEEL_DRIVE.md);
the diagnostic target trim is independent of chassis matching.

Updated wiring (2026-11-21): STBY=BCM25, PWMA=BCM18, AIN1=BCM23, AIN2=BCM24,
C1=BCM22, C2=BCM27. All four wheels are physically attached. This pin update does not
enable four-wheel navigation; TB6612 remains available through individual motor tests.

## Measured matching (2026-11-21)

The fixed 50% / manually tuned 23% duty trims have been replaced by direction-specific measured
speed curves plus encoder feedback. The saved trim is now **100% of the matched speed target**,
not 100% PWM. Dashboard speed settings use the same rounded counts/10ms command as the other
motors, converted using the measured reference median of **103.7185 counts/s per command unit**.
At a 25% request with the current cap of 10, all motors target command 2 (about 207 counts/s).
The TB6612 feed-forward lookup estimates duty and a bounded PI correction adjusts it using
both phases' edge rates over a moving 0.3 s window. A 0.2 s startup burst helps overcome breakaway
friction, then the running-speed curve and encoder feedback take over. PWM remains 1 kHz, with fractional percentages passed directly to lgpio; gpiozero's
LGPIO backend truncated these corrections to whole percentages and was inadequate here.

The matching assumes identical encoder resolution and gear ratio across all four installed
motors. It matches encoder count rates, not independently measured wheel RPM or floor travel.
The Hiwonder convention uses x4 encoder counts ([vendor chassis explanation](https://wiki.hiwonder.com/projects/Black-Mecanum-Wheel-Chassis/en/latest/docs/4_Raspberry%20Pi_formatted.html));
summing rising and falling edges on both GPIO phases gives the corresponding magnitude.
No robot wheel dimension or marker calibration was changed.

Reference motors M2/M3/M4 were tested at +10%, +30%, +50%, and -30%. The TB6612 was swept at
2, 3, 4, 5, 6, 8, and 10% raw PWM duty in each direction. Reverse measurements were less
consistent, so the fit pools non-monotonic observations and live feedback corrects the residual.
Both phases now track one another closely. Only the selected motor was commanded in each run.

Five-second independent compensated validation (rates measured after the first 0.75 s):

| Requested speed | Reference target (counts/s) | TB6612 measured (counts/s) | Difference |
| --- | ---: | ---: | ---: |
| -50% | 518.6 | 520.0 | +0.3% |
| -40% | 414.9 | 419.1 | +1.0% |
| -30% | 311.2 | 297.2 | -4.5% |
| -20% | 207.4 | 204.8 | -1.3% |
| +20% | 207.4 | 188.7 | -9.0% |
| +30% | 311.2 | 302.5 | -2.8% |
| +40% | 414.9 | 392.6 | -5.4% |
| +50% | 518.6 | 514.7 | -0.8% |

The lowest tested 10% setting repeatedly stalled, sometimes alternating between stationary and
moving periods. It is **not** marked calibrated. The runner rejects matched targets below
207.4 or above 518.6 counts/s (command magnitudes 2–5; tested at 20–50% with the current cap).
The greatest average error in accepted runs was 9%; this is approximate matching, not an
exact RPM guarantee. At the lower boundary, short-window deviations reached about 12%.

These results validate unloaded matching over the tested range, not loaded driving. The three
original motors remain unchanged. Four-wheel chassis integration is now enabled; see [the mixed driver document](FOUR_WHEEL_DRIVE.md).
Tests end with outputs off, normal drive service restored, and click driving disarmed.

Saved source and measurements:

- `calib/motor_speed.json`: active curves and trim; also copied to the Pi at
  `~/capstone-drive/motor_speed.json` as the direct runner's default.
- `robot/motor_speed.py`: reusable speed matching and fractional PWM output.
- `tools/measure_motor_speeds.py --wheels-lifted --output <recording.json>`: raw sweep;
  explicitly bypasses matching while collecting raw data, restores previous trim afterward.
- `tools/fit_motor_speeds.py <recording.json> --output <settings.json>`: monotonic curve fit.
- `calib/recordings/robot1_motor_speed_sweep_2026-11-21.json`: full raw sweep.
- `calib/recordings/robot1_motor_speed_validation_2026-11-21.json`: physical validation including excluded low-speed attempts.
- `calib/recordings/robot1_motor_speed_midrange_validation_2026-11-21.json`: additional accepted midrange tests.
- The initial fixed-duty baseline and initial validation are retained as diagnostic history.

Dashboard target trim changes preserve the measured curves. Increasing a target beyond the
measured range is rejected. The legacy `tb6612_encoder_test.py` remains a raw PWM test and does
not apply this matching controller.
