# Control black box

`control.main` (the sole chassis controller) records every control step, so a test that went wrong can be examined
afterwards by a person or an agent. It is on by default (`blackbox:` in `config.yaml`); `--no-blackbox` skips one
run. Added 2026-11-22 on branch `feature/blackbox` (built on `feature/fov-soft-stop`, so it also records the
camera view guard).

## What is kept

One folder per `control.main` run in `logs/blackbox/<YYYYmmdd_HHMMSS>/` on the machine running it (the Mac).
`logs/` is not in Git: copy a session folder if it has to be shared.

| File | Contents |
| --- | --- |
| `meta.json` | start time, `control.main` arguments, robots driven, Git branch / commit / uncommitted changes |
| `config.yaml` | the exact config the run used |
| `events.jsonl` | timeline (read this first): `mode`, `robot_lost` (with the last pose) / `robot_seen`, `reason` (whenever a robot's reason changes in kind, numbers ignored), `vision_lost` / `vision_back`, `user_lost` / `user_seen`, `view_return` / `view_lost` (view guard), `view_changed` (camera view polygon), `mark`, `paused` / `resumed`, `stopped` |
| `ticks-NNN.jsonl.gz` | every control step (20 Hz): vision message (robots incl. `position_source`, `pose_compensated`, `pose_status`, head units; user), vision age, dashboard navigation (armed, follow, targets, WASD keys/speed), targets, commands, reasons, link reply age/status, dead-reckoning estimates (pose, time since last camera pose, distance), view-guard state |
| `snapshots/` | annotated main-camera JPEG (detections drawn by `vision.main --gui`) at `robot_lost`, `view_return`, `view_lost`, `vision_lost` and Marks; at most one per 2 s and 200 per session |

A session is about 2–5 MB per hour; folders beyond `max_total_mb` (500) are deleted oldest first when
`control.main` starts. Writing runs on a background thread, flushes every second (a crash loses at most ~1 s), and
drops records rather than delaying the control loop if the disk stalls.

## Marking a moment

When something goes wrong during a test, press **Mark black box** under the dashboard's floor map and type what
you saw (optional). The event and a camera snapshot are saved, and `--at mark` jumps to it. Marks reach the
controller through the dashboard's navigation API, so they are recorded when `control.main` runs with
`--waypoints` (the normal dashboard setup). The status next to the button shows the number of events and snapshots
and the session name, or "off" when the controller is not recording.

## Reading it

```
python tools/blackbox.py list                         # sessions, newest first
python tools/blackbox.py summary                      # newest session: modes, per-robot % located, losses,
                                                      # view-guard returns, closest approach to the view edge,
                                                      # time per reason, marks
python tools/blackbox.py events --kind robot_lost,view_return,mark
python tools/blackbox.py window --at mark --before 5 --after 2 --robot 1 --every 2
```

`SESSION` (after the command) is a folder name, a unique prefix or a path; default newest. `--at` takes seconds
since the start, a clock time (`15:31:07`), `mark` / `mark2`, or `event:N` (line N of `events`). In `window`, each
robot line shows the pose seen by vision (or `unseen`), target, command, dead-reckoning estimate and age/distance
when it is not seen, view-guard state, a link problem if any, and the reason. For anything else, the raw records
are JSON lines: `gzip.open(path, "rt")` in Python.

Suggested order for a failed run: `summary` → `events` around the problem → `window --at event:N` → the snapshot
with the matching clock time.

## Not covered

- Only `control.main` records. The dashboard (monitor), vision and the robots' drive servers keep their own logs.
  Camera frames are only the event snapshots, not video.
- Keyboard (WASD) commands are recorded as `control.main` sends them; they reach it from the dashboard.
- Tested with unit tests and a dry run against `tools/mock_publisher.py` on Windows; not yet run during a real test
  on the Mac. The dashboard Mark endpoint test (`tests/test_monitor.py`) cannot run on Windows (`fcntl`).
