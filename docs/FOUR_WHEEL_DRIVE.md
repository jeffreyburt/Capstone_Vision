# Robot 1 calibrated four-wheel drive

Robot 1 now uses a mixed backend: the Hiwonder board drives three wheels and the TB6612FNG
drives the front-right replacement. The previously measured calibration is preserved in
[`calib/motor_speed.json`](../calib/motor_speed.json). Robot 2 is unchanged.

## Routing and wiring

All command arrays are logical M1–M4, in signed encoder counts per 10 ms. Wheel order in the
kinematics is FL, FR, RL, RR; the configured channel order is `[3, 1, 4, 2]` with signs
`[+1, +1, -1, +1]`.

| Wheel | Logical channel | Actual output |
| --- | --- | --- |
| Front left | M3 | Hiwonder M3 |
| Front right | M1 | TB6612 channel A |
| Rear left | M4 | Hiwonder M4 |
| Rear right | M2 | Hiwonder M2 |

**Physical Hiwonder M1 is always commanded zero.** It is not repaired or re-enabled. Only
logical M1 is restored to the wheel mixer. Keep the replacement motor connected to TB6612
AO1/AO2, not the damaged port.

The canonical wiring is [`robot/config/robot1_four_wheels.json`](../robot/config/robot1_four_wheels.json).
The diagnostic runner reads that deployed file too.

| TB6612 / encoder signal | BCM GPIO | Physical header pin |
| --- | ---: | ---: |
| PWMA | 18 | 12 |
| STBY | 25 | 22 |
| AIN1 | 23 | 16 |
| AIN2 | 24 | 18 |
| C1 | 22 | 15 |
| C2 | 27 | 13 |

Logic/encoder supply and common ground must retain the existing wiring. Motor power remains
external. `tb6612.direction` is +1 in the deployed config, preserving the logical forward
mapping and the existing AO1/AO2 polarity. **Installed wheel direction and floor motion have
not been physically verified in this change**, because the user confirmed both robots are
on the floor. If a supervised direction check proves FR is reversed, change only this sign
and redeploy/restart; do not swap encoder phases to try to reverse the motor.

## Calibration contract

- Same saved positive/negative measured curves, x4 count convention, and PI gains as diagnostics.
- Reference: 103.718477654 counts/second per command unit, assuming identical encoder resolution
  and gear ratios on all installed motors.
- Reliable replacement command magnitudes: **2–70** (about 207–7260 counts/s). The 1-unit region
  stalled during lifted-wheel calibration and remains excluded. Up to 2026-11-21 this was 2–5;
  see "High-speed extension" below.
- All four wheel commands are capped together at **70**, preserving wheel ratios when scaling.
  Waypoint driving keeps Robot 1 at **0.038 m/s**; WASD may use up to **0.53 m/s** (13000 counts/m
  is still an estimate).
- If logical M1 would be ±1 after rounding, the **whole chassis command is zero** for that tick.
  Other wheels are not allowed to continue with an uncalibrated FR speed. Ramping can therefore
  have a short initial deadband, and combined motions with a very small FR component can hold.
- M1=0 with other nonzero wheels is legitimate for some mecanum diagonals and is allowed.
- A 0.2 s breakaway burst, then the calibrated feed-forward duty and 0.3 s encoder-rate window,
  drive a bounded PI correction. Integral correction survives same-direction speed changes.
  Reversal first disables the bridge and waits 0.1 s before enabling the opposite direction.
- The PWM is 1 kHz using fractional duty via lgpio, not gpiozero's integer-percent PWM wrapper.
- The chassis backend uses the measured profile directly, not the diagnostic dashboard's manual
  target trim. Applying a trim to only FR during chassis driving would spoil wheel matching.

Calibration validation was **unloaded**, with roughly 1–9% average matching error in accepted
runs and about 12% short-window error near the low boundary. This is not proof of loaded floor
performance or accurate odometry. There is no claim that wheel slip, roller orientation, wheel
radius, or physical forward polarity has been recalibrated.

## Control ownership and stopping

`DriveServer` remains the sole robot-side owner. `MixedMotors` routes one vector to both
drivers, and the same 20 Hz tick advances the TB6612 feedback controller. No second drive
thread races the Hiwonder controller. Encoder callback threads only count edges.

The existing 0.5 s command watchdog, source-IP restriction, manual keyboard lease, and pose
requirement for autonomous commands are unchanged. Stop/zero commands disable PWM/STBY and
zero the Hiwonder outputs. Missing TB6612 encoder pulses for >0.75 s after startup latch a
mixed-backend fault and stop the whole vector. Normal commands do not clear that fault;
fix the wiring/power issue and restart the drive service explicitly. Gross phase-count disagreement
also faults: after startup and at least 40 edges, a phase below 70% of the other cannot support
valid summed x4 speed matching. Telemetry reports these faults.

Individual motor tests stop `capstone-drive-1.service`, acquire their diagnostic file lock,
then restore the service after releasing their GPIOs. With the mixed driver running, never
start the legacy standalone GPIO test directly: use dashboard diagnostics, or stop the drive
service first. This is especially important when several agents work on the same robot.

## Telemetry

Drive status and normal ACK packets include `mixed_drive`:

- `backend: hiwonder+tb6612`, `minimum_counts`, `maximum_counts`, `direction`;
- `hiwonder_commands`: actual Hiwonder array with M1 always zero;
- `tb6612.command`, `target_count_rate`, `measured_count_rate`, `edges`, `duty_percent`;
- `guard`: unsupported low-speed whole-chassis hold; `fault`: latched hardware failure.

`motors` reports the effective logical output vector, including zero when held by the guard.
The keyboard panel displays the FR target/measured rate, guard, or fault. Four-wheel support
applies to both manual keyboard driving and autonomous drive commands; autonomous motion still
needs compensated camera feedback and explicit waypoint arming.

## Deployment and rollback

The service uses system Python because gpiozero/lgpio/smbus2 are installed there; its PYTHONPATH
points to `~/capstone-base/src` for the existing I2C driver. The old virtualenv-only command
could not be assumed to provide GPIO dependencies.

Copy these repository files to `/home/robot-pi1/capstone-drive/`:

```
robot/drive_server.py
robot/mixed_motors.py
robot/motor_speed.py
robot/motor_diagnostic.py
robot/config/robot1_four_wheels.json
calib/motor_speed.json
robot/services/capstone-drive-1-fourwheel.conf
```

With keyboard and waypoint driving disabled and the laptop controller stopped, install the
service drop-in using an administrator account:

```sh
sudo install -d -m 0755 /etc/systemd/system/capstone-drive-1.service.d
sudo install -o root -g root -m 0644 ~/capstone-drive/capstone-drive-1-fourwheel.conf /etc/systemd/system/capstone-drive-1.service.d/fourwheel.conf
sudo systemctl daemon-reload
sudo systemctl restart capstone-drive-1.service
```

The original unit remains unchanged. Restart the local monitor and controller to load
`config.yaml`. They start with keyboard and waypoint driving disabled. Verify `status`
telemetry reports `hiwonder+tb6612`, no disabled logical channels, max_counts 5, no fault,
and all motor commands/duty zero **before enabling user control**.

To roll back, disable driving, stop the service, remove only the `fourwheel.conf` drop-in,
reload systemd, and restart. The original unit then uses the preserved
`robot1_three_wheels.json` and virtualenv. Restore the old Robot 1 speed/strafe config if
reverting the laptop profile. The current driver still supports the old config. Original
files are also backed up remotely as `drive_server.before-fourwheel.py`,
`motor_diagnostic.before-fourwheel.py`, and `motor_speed.before-fourwheel.json`.

## Validation state

Simulated tests cover routing, a permanently zero physical M1, whole-vector low-speed holds,
watchdog shutdown, fault latching, and reversal dead time. Deployment verification uses only
read-only status and zero commands. **No physical wheel movement or floor test is part of
this deployment.** The earlier speed sweep recordings remain the calibration evidence.

## High-speed extension (2026-11-21)

Measured with Robot 1 lifted, battery 12.6 V, using `robot/speed_sweep.py` (raw data in
`calib/recordings/robot1_top_speed_sweep_2026-11-21.jsonl`).

- **TB6612 FR motor, open loop:** near-linear, about 8,000 counts/s forward and 7,820 reverse at
  100% duty. The saved curves keep the earlier points below 10% duty and use the sweep from 10–100%.
  `SpeedMatch` now accepts curve duties up to 1.0 (was 0.5).
- **Hiwonder M2–M4, closed loop:** track their command exactly up to 70 counts/10 ms (about 7,250
  counts/s), then saturate near 7,800–8,000 counts/s; 80, 100 and 127 give the same speed.
- So the shared cap is **70 counts/10 ms** (`maximum_count_rate` = 70 × 103.718 counts/s). This is
  about 0.54 m/s straight ahead at 13000 counts/m.

Closed-loop validation through `MixedMotors` (`speed_sweep.py --validate`, all four logical wheels at
the same command, still lifted; `calib/recordings/robot1_top_speed_validation_2026-11-21.jsonl`):

| Command | FR error forward | FR error reverse | FR duty fwd / rev |
| ---: | ---: | ---: | ---: |
| 20 | +0.3% | -0.6% | 26% / 27% |
| 40 | +0.5% | -0.6% | 51% / 53% |
| 60 | +0.1% | -0.6% | 76% / 80% |
| 70 | -0.1% | -1.6% | 90% / 94% |

FR stayed within 1.6% of the Hiwonder wheels at every speed. At 70 in reverse it uses 94% duty,
leaving only about 6% for extra floor load. **Not yet validated on the floor:** under load FR may
fall short at the top speed, which would show as a slight curve. If it does, lower the cap (for example
to 65) rather than removing the guard.

Deployed files on Robot 1 were backed up to `~/capstone-drive/bak-2026-11-21-topspeed/` first.
Rollback: copy them back and `sudo systemctl stop/start capstone-drive-1.service`.

`speed_sweep.py` stops the drive service, holds the motor diagnostic lock, and stops every output on
error, Ctrl+C, SIGTERM/SIGHUP, or stdin EOF. An earlier run without the SIGTERM handler was cut off
by `timeout` mid-sweep; the wheels were found stopped afterwards, but that is why the handler exists.
