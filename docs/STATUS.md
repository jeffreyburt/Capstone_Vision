# System status: read this first

**As of 2026-11-20.** Where the integrated speaker-robot system stands, what is verified on hardware, what is
known to be broken, and how to work on it. Update this file whenever a fact here changes; put dates on
results and say whether they came from hardware, simulation or reasoning.

**Repo:** `github.com/jeffreyburt/Capstone_Vision`, branch `main`. Several people *and* AI agents commit to
it, sometimes from the same checkout: **always `git fetch` and rebase before pushing, and never commit or
discard someone else's uncommitted files without asking.**

## 1. What the system does

The birds-eye camera on the master Pi streams to the laptop. `vision.main` finds the robots (ArUco markers
on the speaker heads) and the listener (YOLO). `control.main` plans a listening triangle around the listener
and drives each robot there by sending body velocities to `robot/drive_server.py` on the robot. **The laptop
closes the position loop**; the robots only execute velocities, with a 0.5 s watchdog.

```
master Pi camera ──Ethernet──► laptop: vision.main ──UDP 5005──► control.main ──UDP 5700──► robot drive_server ──I2C──► wheels
                               (GUI :8080)                       (dry run unless --arm)       (via master Pi's WiFi AP)
                               python -m monitor (:8090) watches everything, read-only
```

| Doc | For |
|---|---|
| [MVP_RUNBOOK.md](MVP_RUNBOOK.md) | Bring-up order, commands, what stops a robot, tuning |
| [NETWORK.md](NETWORK.md) | Addresses, ports, WiFi, known robot 1 WiFi issue |
| [HEAD_TRACKER.md](HEAD_TRACKER.md) | William's face tracking and pan/tilt-compensated driving |
| [PLAN.md](PLAN.md) | Vision design (§1–10) and the MVP architecture decision (§11) |
| [README](../README.md) | Vision output format, system GUI, repo layout |

## 2. Machines

**2026-11-20 main camera restored after reboot:** The main Pi was online but its manually launched
camera publisher had not restarted; TCP 5600 was closed and the laptop retained an old frame. Installed
and enabled the existing `camstream-birdseye.service` using
`bash ~/Capstone_Vision/scripts/install_sender_service.sh birdseye`. Verified the service is active and
enabled, publishing the IMX708 Wide feed at roughly 4 fps. The laptop vision receiver did not recover its
overnight connection, so only `vision.main --gui` was restarted; fresh changing annotated JPEGs and IDs
2/3/6 were then verified through dashboard `/video/birdseye`. The old floor pose remains disabled pending
calibration. No motor or servo controls were changed. After future Pi reboots the publisher starts
automatically; if an overnight laptop connection remains stalled, restart its passive vision process.


**2026-11-19 all-camera dashboard update:** The main camera publisher and laptop vision process were
already running; fresh annotated main-camera frames verified at about 3–4 fps. Both robot IMX708 feeds
also deliver changing valid JPEGs through the monitor's `/camera/robot1` and `/camera/robot2` routes.
The dashboard now groups both robot feeds directly below the main camera, visible by default, with
per-camera hide controls, a Show all cameras button, retry handling, and explicit unavailable status.
Head-camera display rotation is included in the camera metadata and matches the individual dashboards.
Verified all three visible image sources in a rendered browser, changing frames from every endpoint,
and all 24 monitor tests. Restarted only the laptop monitor; no robot controls, torque or wheel states
were changed.


**2026-11-19 servo capture fix:** Robot 2's capture buttons incorrectly treated fresh feedback as stale
because the browser used the Mac's clock (~3.4 hours ahead of the Pi). The integrated head dashboard now
returns `serverTime` and compares feedback against that same clock. Capture buttons also explain why
they are disabled (motion, torque-control operation, stale/missing readings, or disconnection). Applied
only the targeted backend change and updated HTML on robot 2, preserving unrelated code and backups;
restarted its already-stopped dashboard service with torque off. Verified `/api/capture` HTTP 200,
readings around pan 770 / tilt 657–658, both rendered capture buttons enabled, and JavaScript checks for
clock skew, stale data, missing readings and active motion. Existing A/B limits were not overwritten.
The full repository suite passes 112 tests plus 10 subtests. Tracking was already stopped by the owner
before this fix and was not restarted; torque remains off. No motion commands were sent.


**2026-11-19 follow-distance preparation (current camera position):** The owner moved the main camera
to approximately 0.36 m above the floor and requested robot 2 follow the listener at 2 m. Formation distance
is now 2.0 m. `calibration.floor_pose` points to `calib/floor_pose_36cm.yaml`, which does not exist yet;
the old `calib/floor_pose.yaml` assumes a 1.006 m lens height and is preserved only as a historical pose.
Vision therefore reports image markers/person boxes but no metric positions. Robot 2's controller runs
without `--arm` (DRY RUN); no wheel commands were sent. Do not arm it until the new floor pose is measured
and validated. Simultaneous head tracking and metric following also need the documented moving-marker
geometry and clock synchronization, or chassis-mounted fixed markers.

Robot 2 head camera and both servos respond; the requested stationary head-tracking test was started
with saved limits/signs, torque on and a reduced 60 units/s speed limit. It is holding while no face target
is confirmed. The dashboard is reachable on this Mac at `http://localhost:18082/` via SSH forwarding;
browser face detection requires that tab to remain visible. Printable 15 cm floor markers 40/41 can be
regenerated with `python tools/generate_markers.py --ids 40-41 --size-cm 15 --out markers/floor-36cm`.
Print at actual size, measure the black square, and place the sheets flat on the floor before running
`tools/floor_from_markers.py --ids 40 41 --forward-id 40 --size-cm 15`; validate its distance output with
a tape measure and restart vision after saving. Do not substitute the height alone for camera pose.


**2026-11-19 marker detection update (later live checks):** The main camera now consistently detects
robot 2 side ID 5 and rear ID 6 and reports robot 2's position. OpenCV's threshold bias of 7 missed rear
ID 6 beside its dark connectors; `markers.adaptive_thresh_constant: 3` recovers it without changing
allowed IDs, dictionary error correction or polygon tolerances. Robot 1 rear ID 2 is recovered in some
frames and produces a pose, but a cable/connector overlaps its top border and it remains intermittent.
Clear the entire black square and white margin physically; software does not reconstruct covered tags.
Regression coverage includes a cropped robot-only ID 6 image (no people or surrounding scene). The full
suite passes 112 tests plus 10 subtests. Only the laptop vision process was restarted; no motion or torque
commands were sent.


**2026-11-19 camera update (later live checks):** Both robot IMX708 head-tracking cameras now deliver
fresh changing JPEGs; robot 2 recovered after the owner reseated its ribbon and rebooted. Head tracking
remains off. Robot 1's servo communication remains unconfirmed while awaiting a replacement BusLinker;
USB enumerates, but read-only ID requests received no replies even with external power on.

The main `imx708_wide` camera publisher was started in `~/Capstone_Vision` on the main Pi (still manual,
not a boot service). The laptop vision process was reconnected to Ethernet `192.168.1.20:5600` and
verified at about 3–4 fps with changing annotated JPEGs and live person detections through the system
dashboard. No configured robot markers were detected in the observed scene. The dashboard now has a
main-camera panel showing person/marker overlays, the selected listener's estimated floor position,
counts and frame freshness; stale feeds and targets are hidden. `control.main` was not started, and no
motion or torque commands were sent. The SSH gateway probe program is now sent through stdin so the
health process listing cannot mistake its source text for running camera or motor programs.


**2026-11-19 update (Mac, SSH and read-only hardware checks):** Robot 1 has been replaced by the Pi 3
at `192.168.50.105`, user `robot-pi1`, hostname `raspberrypi`, WiFi MAC `b8:27:eb:35:63:17`. Both robot
Pis are reachable through the main Pi. Robot 2 remains `.12` / `capstone-pi2` / `raspberrypi`; the main
Pi is `.1.20` Ethernet / `.50.1` WiFi, user and hostname `vision`. Passwords remain outside the repo.
The Mac's public key was installed on the main Pi with explicit owner approval; existing robot keys
work through ProxyJump. The dashboard uses SSH gateway probes and camera forwarding, with no Mac
WiFi changes or motor commands. `.105` is DHCP: reserve the lease or update both config hosts if it changes.

Latest observed status: Robot 1 dashboard runs but neither servo replies and no camera is detected.
Robot 2 dashboard reports pan 408 / tilt 759, torque off, tracking off, and camera frames stopped.
Its existing wheel server reports stopped. The main camera sender and laptop vision/control are not
running. Clocks differ (main ~33 days behind, robot 1 ~7 minutes, robot 2 ~3.3 hours); Robot 1 has historical
undervoltage/throttling flags. These are displayed as real warnings, not fixed by monitoring. No motion
or service restarts on the Pis were performed. Older hardware observations below remain historical.


| Machine | Address | Login | State (2026-11-19) |
|---|---|---|---|
| Laptop (Windows / Mac) | Ethernet `192.168.1.10`; route `192.168.50.0/24 via 192.168.1.20` | — | Runs vision, control, the system GUI. Its only Ethernet port is shared: plug it into the **master Pi** for normal operation |
| Master Pi 4 | `192.168.1.20` (eth0), `192.168.50.1` (WiFi AP) | `vision`, laptop SSH key; sudo needs the password | Camera sender running (started by hand with `nohup`, **not a service**: restart after a reboot, §6). Runs WiFi AP `capstone-robots` (channel 1). **Clock ~33 days behind** (no RTC/internet) |
| Robot 2: Pi 4 | `192.168.50.12` (WiFi); wired DHCP | `capstone-pi2`, laptop SSH key; sudo needs the password | William's Debian 13 image. On the robot WiFi. `~/capstone-base` (+ `.venv`), `~/Capstone_Vision`, `~/drive_server.py`. William's head-tracker dashboard service on :8080. **Drives** (§3) |
| Retired Robot 1: Pi 3 (Nate's) | should be `192.168.50.11`; falls back to its bench hotspot `Capstone-Pi` `10.42.0.1` | `user`, laptop SSH key; sudo needs the password | **Cannot join `capstone-robots`** (§4). Code added, nothing driven. To be replaced by a backup Pi 3 |
| Current Robot 1: Pi 3 B+ | `192.168.50.105` DHCP (MAC `b8:27:eb:35:63:17`) | `robot-pi1`, laptop SSH key | Joined `capstone-robots`; dashboard active; see current hardware warnings above |

On Windows, SSH from the laptop: use **Windows OpenSSH** (`C:\Windows\System32\OpenSSH\ssh.exe`); Git Bash's ssh lacks
the host keys. In Git Bash set `MSYS_NO_PATHCONV=1` or remote paths like `/tmp/x` get rewritten.

## 3. Robot 2: verified on hardware (2026-11-19)

| Item | Result |
|---|---|
| Wheel board | Hiwonder at I2C `0x34`, 12.7 V supply |
| Wheel map | Measured one motor at a time: FL=M3, FR=M1, RL=M4, RR=M2; signs `[1, -1, -1, 1]`. **Differs from the GitHub default `teleop.json`.** Saved as [robot/config/robot2_wheels.json](../robot/config/robot2_wheels.json) and on the Pi as `~/capstone-base/config/robot2_wheels.json` |
| Lifted tests | Forward, strafe-left and rotate-CCW all turn every wheel the right way |
| Forward on the floor | Camera-measured **0.400 m for 0.400 m** commanded at 0.10 m/s, dead straight. `--counts-per-meter 13000` is within ~4% at 0.05–0.15 m/s (at 0.05 m/s the counts round down, so it goes ~8% short) |
| Turn in place | 0.3 rad/s for 4 s: ~85° measured vs 69° commanded, ~6 cm drift. Turn radius **~0.16 m (tentative**: start and end were measured on different marker faces) |
| **Strafe** | **Broken on the floor:** a 0.20 m strafe-left command rotated it **−71° (CW) with ~5 cm of motion**, although every wheel turns the right way. Not a mirrored wheel pair (the turn test would have slid ~20 cm). Most likely the front rollers don't roll sideways (stiff, or too little weight on the front). **Check the rollers by hand.** Until fixed: **`control.limits.allow_strafe: false`** (no-strafe mode, §5) |
| Follow test | Dry runs OK (correct targets, turn-first commands). **Not yet run armed** |

Run its drive server like this (hardware mode, robot 2's map, commands only from the laptop):

```bash
cd ~/capstone-base && .venv/bin/python ~/drive_server.py --robot-id 2 --hardware \
    --hiwonder-config config/robot2_wheels.json --allow-from 192.168.1.10
```

**Camera findings for robot 2:**
- **No front marker** on either robot. Robot 2 is seen by its back (6) and left (5) markers only, because
- **the right-side marker (7) is covered by the robot's own camera module** on its white bracket: it is never
  detected. Move the camera or the tag.
- The heading read from the **back marker is ~9–10° off** its true direction of travel. Recheck per face.
- Below ~1.5 m from the camera the robot sits at the image's bottom edge and is detected in ~1 frame in 4.
  Keep robots 2–5 m out.

## 4. Retired Robot 1 (Pi 3): historical WiFi problem

Its onboard WiFi (BCM43430/1, firmware 7.45.98) **joins an iPhone hotspot fine but never reaches the
master Pi's access point** (`ASSOC-REJECT status_code=16`; nothing in the AP's log), on channel 6 and 1.
Robot 2 (Pi 4), an iPhone and the backup Pi 3 join it fine, and robot 1's own hotspot has identical settings.
A reboot and `options brcmfmac feature_disable=0x82000` did not help. That option is still in
**`/etc/modprobe.d/brcmfmac-capstone.conf` on robot 1: delete it** (sudo) if the Pi is kept. Fixes: a USB
WiFi adapter, or the backup Pi 3.

Also on robot 1: Nate's `~/capstone-base` (a plain copy with his local `teleop.json`, `pan-tilt.local.json`,
`servo.local.json`) is untouched; `~/Capstone_Vision` and `~/drive_server.py` were added; the wheel board
answers at `0x34`. Its `capstone-robots` profile (static `.11`, priority 200 over the hotspot's 100) is saved.
**Its wheel map was never measured**: do the one-motor-at-a-time test (§6) before driving it.

## 5. Control software state

- `control/controller.py` **no-strafe mode** (`allow_strafe: false`, on now, applies to both robots): turn
  toward the target (or back up if that's the smaller turn), drive straight once within 25°, then turn to
  face the user; never sends sideways speed. Parks until the error exceeds 15 cm. Tested in simulation
  including a base whose strafe turns into rotation. Set `allow_strafe: true` once a robot strafes properly.
- The listener is assumed to **face the camera** (`facing_deg: -90`), so the robots go between the listener
  and the camera. **Stand about 3–4 m from the camera**, otherwise the targets land outside the camera's view.
- **Person tracking locks onto whoever it saw first** and keeps them while they stay visible. With other
  people in the lab it can follow the wrong person (it did, 6 m away). Check the user position in the dry
  run; restart `vision.main` to re-lock onto the most confident detection.
- Vision runs at **~2.5 fps** (12 MP frames + YOLO on the laptop): fine for slow following.

## 6. How-to snippets

| Task | How |
|---|---|
| Pi on a direct Ethernet cable gets no address | `python tools/one_lease_dhcp.py --listen`, then `python tools/one_lease_dhcp.py <mac> 192.168.1.50` |
| Put the laptop's SSH key on a Pi | the user types the password once: `ssh <user>@<ip> "umask 077; mkdir -p ~/.ssh; echo '<laptop .pub key>' >> ~/.ssh/authorized_keys"` |
| Restart the camera sender | on the master Pi: `cd ~/Capstone_Vision && nohup .venv/bin/python -u -m camstream.sender birdseye > ~/sender.log 2>&1 < /dev/null &` |
| Map a robot's wheels | wheels lifted, on the robot: `.venv/bin/capstone-motor-test --motor-type 3 --channel N --speed 5 --duration 1` for N=1..4; note which wheel turns which way |
| Watch wheels turn | `python tools/drive_once.py --robot 2 --vy 0.05 --seconds 20` (lifted first) |
| Camera-measured floor test | `python -m control.drive_test --robot 2 --vx 0.10 --seconds 4` (robot 2–3 m out, visible) |
| Join a robot to the robot WiFi | on the robot: `sudo bash join_robot_wifi.sh <1\|2>` (from `robot/`) |
| Can a Pi join WiFi as a client at all? | on the robot: `sudo bash wifi_client_test.sh iphone` (from `robot/`) |
| Change the AP channel | on the master Pi: `sudo bash scripts/set_robot_ap_channel.sh 1` |
| Prepare a fresh Bookworm SD card for a robot | `python robot/card/prepare_card.py --boot D:\ --robot N --bundle <dir>` (not used yet on a robot) |

## 7. Rules for working on the hardware

- **Ask the people at the robot before every move**, say what it will do, and start with wheels lifted.
- Never type passwords for people: they type `sudo`, WiFi and login passwords themselves in a terminal.
- Only one program may drive a robot's wheels (drive server *or* teleop *or* motor test). The head tracker
  owns the servo serial port; don't run other servo tools beside it.
- Robot drive servers started from a laptop session die with that session; the watchdog then stops the wheels.

## 8. Next steps, in order

1. Robot 2: check the front rollers (spin by hand; press down on the front while strafing). Uncover the
   right-side marker. Recheck the heading offset per face.
2. Robot 2: first armed follow test (`python -m control.main --robots 2 --arm`), listener 3–4 m out, nobody
   else in view.
3. Robot 1: swap in the backup Pi 3 (or a USB WiFi adapter), join `capstone-robots`, map its wheels, repeat §3.
4. Make the camera sender a service on the master Pi (`scripts/install_sender_service.sh birdseye`, needs sudo).
5. Time sync (chrony on the master Pi): needed for William's pan/tilt compensation, not for the MVP loop.
