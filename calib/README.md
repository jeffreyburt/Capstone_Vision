# Persistent calibration and click-to-drive setup

The calibration is saved in Git and loaded by `config.yaml`. Rebooting the Pis or restarting the laptop
programs does **not** require repeating the measurements. Camera or mechanical changes can invalidate
the affected calibration; normal travel and power cycles do not.

## Dashboard checklist (no AI needed)

Open <http://localhost:8090/calibration> (link in the main dashboard header). It lists every setup step in
order with live status (done / needs attention / to do), shows every stored value read from these files, and
runs the tools: lens calibration (via the vision GUI), floor measurement, right-head sweeps and fit, and a
15-second stationary check. Fits only ever write candidates under `calib/candidates/`; Apply backs up the old
file to `calib/history/` and writes a consolidated copy to `calib/snapshots/`. Owner measurements, operator
confirmations (left head secured, wheel directions verified) and an event history are kept in
[calibration_record.json](calibration_record.json). A `robot<ID>_head_pose.yaml` whose status starts with
`active` is the runtime head geometry and overrides the copy embedded in `config.yaml`. Restart
`python -m vision.main --gui` after applying a floor or head calibration. The page never drives wheels.

## Floor recalibration — 2026-11-21

The main camera had moved (now 36.3 cm high and 12.6° down; it was 37.9 cm and 11.2°), so head poses failed with
29–46 px reprojection errors. New layout from the owner: 15 cm tags 2.0 m in front of the camera, ID 1 0.70 m to the
left and ID 0 0.70 m to the right, lens 0.37 m above the floor. `tools/floor_from_markers.py --forward-mid` (also
used by the calibration page via `floor_forward_mid: true`) points +y at the tags' midpoint, so **+y runs straight
down the hallway and +x to the camera's right**. Fit vs owner: height 0.363 vs 0.37 m; ID 0 (+0.708, 1.970) and
ID 1 (−0.708, 1.954) vs (±0.70, 2.00); spacing 1.415 vs 1.40 m; both tags within 0.2° of the floor plane. The previous
pose is in `history/20261121_144223_floor_pose_36cm.yaml`. With this pose the hallway walls project to
x ≈ −0.87 m (left) and x ≈ +0.90 m (right), parallel to +y. Head calibrations were not changed; robot pose fits
still need checking with the robots back in view (Robot 1's head must be re-secured level first).

## Current active calibration — 2026-11-20

| File | Information |
| --- | --- |
| [camera_intrinsics.yaml](camera_intrinsics.yaml) | Lens matrix/distortion at 4608 × 2592; 12 views, 0.874 px RMS. |
| [floor_pose_36cm.yaml](floor_pose_36cm.yaml) | Current floor transform; filename is historical. Owner: lens 39.5 cm, floor-tag spacing ~103 cm. Fit: 37.9 cm, 103.6 cm. |
| [robot1_head_pose.yaml](robot1_head_pose.yaml) | Secured fixed left head, rear tag 2; no working servos or feedback required. |
| [robot2_head_pose.yaml](robot2_head_pose.yaml) | Camera-fitted right pan/tilt geometry, tag mounting angles, tested range and live validation. |
| [head_neutral_pending.json](head_neutral_pending.json) | Historical filename; now records active calibration, owner measurements, original neutral observations and validation provenance. |
| [../robot/config/robot1_three_wheels.json](../robot/config/robot1_three_wheels.json) | Left wheel mapping/signs; M1 disabled; ten-count speed cap (previously tested on the floor). M1 motor must remain disconnected. |
| [../robot/config/robot2_wheels.json](../robot/config/robot2_wheels.json) | Existing right wheel mapping/board settings. |
| [floor_pose.yaml](floor_pose.yaml) | Historical ~1 m camera pose; not active. |

Active head geometry is embedded in `config.yaml`; the per-robot YAML files preserve its measured/fitted
source and validation. Update both together when changing geometry.

Chassis origin is the floor point beneath the midpoint of the four wheel centers; +X forward, +Y left,
+Z up. The owner supplied the centered rear-tag offset, recorded for both as X=-0.14 m, Y=0. Both heads
were level/facing chassis front at the reference capture. Robot black squares are 10 cm; borrowed floor
tags are 15 cm. Hidden top/front tags are not required: left uses tag 2; right uses rear 6 and side 5.
The left head must remain mechanically secured.

Right reference readings: pan 391, tilt 770. The model accounts for small tag mounting-angle offsets,
a pan-following side marker, and a rear marker following pan and tilt. It uses fitted joint scales and
pivot lines, not assumed manufacturer angle scales. Pan-axis Z and tilt-axis Y are unobservable along
their own axes; zero in those coordinates is a gauge choice, not a physical pivot-height measurement.

**Right calibrated envelope:** pan 331–451, tilt 710–775 encoder units (the upper five tilt units allow
feedback jitter around the 770 reference). Outside it, no compensated pose is published and navigation
stops. Wider movement needs additional validation; the model must not silently extrapolate. This initial
calibration is for the gentle movements tested, not a guarantee at arbitrary head speeds.

Independent offline validation: about 1.56 px corner-coordinate RMS, p95 position deviation ~3.2 cm,
p95 heading deviation ~4.6°. Separate **live** validation while the chassis stayed forward/stationary:
62 valid compensated frames, p95 position deviation ~4.0 cm, maximum ~5.8 cm, p95 heading deviation ~2.4°.
These describe relative chassis stability, not absolute survey accuracy. The navigation stopping tolerance
is 10 cm. Current speed limits are 0.075 m/s for the three-wheel left robot and 0.12 m/s for the
four-wheel right robot; turn-rate limits are 0.20/0.30 rad/s. Position gain is 0.6, retaining slowdown
near destinations and the existing wheel acceleration ramp. The higher-speed profile has not yet had a measured floor waypoint test; calibration validation itself
used no wheel movement.

## Dashboard operation

Open <http://localhost:8090/>. The floor map shows each robot's actual camera position, selected target,
X/Y and distance errors, tracking age, and distance-error history. To drive:

1. Confirm readiness and a clear path, with left M1 disconnected and its head secured.
2. Turn on **Enable click driving**, select a robot, then click a destination on the grid.
3. **Cancel all targets** stops both targets. Turning the enable switch off clears them too.

The movement switch always starts **off** when the dashboard restarts. Changing it clears old goals;
preview targets never automatically turn into movements when calibration/network conditions recover.
Vision loss, stale capture, missing joint samples, invalid head range, and six seconds of failed
translation progress stop commands. Robot-side watchdogs stop after 0.5 s without commands.

## Network and restart

The Mac keeps its Internet connection. Vision reads the main camera over Ethernet; measured right joints
are polled through the existing authenticated SSH gateway. A minimum-observed camera send/receive clock
offset and HTTP server-relative sample ages map both feeds onto the laptop clock without changing Pi
system clocks. Delayed frames retain their age. Slow HTTP round trips or stale samples are rejected.
The measured 300–450 ms stream/processing delay uses a 650 ms capture cutoff in low-speed navigation;
packet-arrival freshness and the robot watchdog remain separate checks.

`control.drive_via` sends UDP commands over authenticated SSH through the main Pi. Both robot services
allow commands from the main Pi's Wi-Fi address 192.168.50.1 and require compensated poses. No Mac route
or Wi-Fi change is needed. Services are `capstone-drive-1.service` and `capstone-drive-2.service`, enabled
at boot; service startup sends only zero speed. Robot 1 uses ~/capstone-drive/robot1_three_wheels.json;
Robot 2 retains ~/capstone-base/config/robot2_wheels.json.

Start the laptop programs from the repo in separate terminals if they are not already running:

```sh
.venv/bin/python -m monitor
.venv/bin/python -m vision.main --gui
.venv/bin/python -m control.main --robots 1,2 --waypoints --arm
```

`--arm` enables the transport, but the dashboard's separate movement switch starts off; only a new
explicit enabled grid click can produce motion. Do not run duplicate controllers or wheel tests beside
the services. This session verified status/stop-only gateway traffic and all-zero motor outputs.

## Saved recordings and reproducibility

- [right_head_motion_2026-11-20.json](right_head_motion_2026-11-20.json): 118 raw samples. The strict
  return check recorded `failed` at pan 384 vs 391; the data was retained, motion stopped, torque held.
- [right_head_validation_2026-11-20.json](right_head_validation_2026-11-20.json): 85 independent samples
  from smaller movements; not used to train the model.
- [right_head_offset_tilt_2026-11-20.json](right_head_offset_tilt_2026-11-20.json): 86 samples exposing the
  side/rear tags by moving the mechanism while keeping the chassis facing forward.
- [right_head_live_validation_2026-11-20.json](right_head_live_validation_2026-11-20.json): 83 samples
  of the running feedback pipeline, including the exact model and SHA-256 hash used at capture.
- [right_head_motion_fit_2026-11-20.json](right_head_motion_fit_2026-11-20.json): initial PnP diagnostics,
  superseded by the mounting-angle-aware fit; not active geometry.
- [right_head_pixel_model_fit_2026-11-20.json](right_head_pixel_model_fit_2026-11-20.json): refined fit,
  independent and live validation, candidate parameters and model provenance.

Run `python tools/fit_head_motion.py` in the project environment to reproduce offline fitting (SciPy is
in requirements-laptop.txt). It writes candidate diagnostics, never activates a new model or drives
hardware. A rerun does not change the saved active config or establish a new live validation by itself.
The experiment-specific `tools/capture_right_head_motion.py` requires `--execute` and a new `--out`
recording path before it can engage/move the right head. It never drives wheels, refuses to overwrite
recordings, and preserves saved tracking limits. It temporarily replaces dashboard A/B sweep parameters
(the original values are in each recording). Inspect final positions/torque after any return warning. After the completed experiment, the right
dashboard was restarted without servo movement to restore its original A/B sweep parameters; torque
remained engaged near neutral (~396/766), and head tracking/sweep remained off.

## When calibration needs revisiting

- Move the main camera: repeat floor calibration. Change lens/focus/resolution/crop: validate intrinsics.
- Move a tag, shift the secured left head, reindex a servo horn, or change a linkage: redo the affected
  head geometry. Changing a motor controller does not by itself invalidate camera geometry.
- IDs 0/1 are currently excluded from robot poses because floor copies are visible. Do not re-enable
  them as robot tags until duplicates are removed and their actual mount geometry is calibrated.
