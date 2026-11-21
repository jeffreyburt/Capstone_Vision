# Agent handoff: current robot system

Updated 2026-11-21. Start here; dated sections of older runbooks may describe superseded hardware.
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
The mixed profile has no disabled logical channels and caps all wheels to 5 counts/10ms.
Robot 1 body speed is capped at 0.038 m/s; strafing is enabled. Commands requiring a nonzero
FR speed below 2 units hold the whole chassis. Do not remove that guard or raise the cap
without extending/revalidating the calibration.

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
