# Robot setup handoff: drive server for the "robots follow the user" MVP

> **Superseded in part (2026-11-19 evening): read [docs/STATUS.md](../docs/STATUS.md) for the current state.**
> Since this was written: robot 2 turned out to be a **Pi 4** running William's image (user `capstone-pi2`),
> already set up and driving with its measured wheel map
> ([robot/config/robot2_wheels.json](../robot/config/robot2_wheels.json)); the GitHub default wheel map was
> wrong for it. The robot WiFi runs on **channel 1**. Robot 1's Pi 3 cannot join it (see STATUS §4). The
> procedures below (wheel checks, calibration, the command protocol) still apply.

You (an AI agent helping a teammate) are setting up the **robot side** of the first integrated test of
the capstone speaker-robot system. This file is self-contained; the other files in this zip are the code
and docs it refers to.

- **Source of truth:** GitHub `jeffreyburt/Capstone_Vision`, branch `main`, commit `1ef94f9` or later.
  The zip is a snapshot of the relevant files from that commit.
- **The robot's own code:** GitHub `NateMG4/NEU-capstone-robot` (Python package `capstone_base`),
  installed on the robot Pi at `~/capstone-base` with a venv at `~/capstone-base/.venv`.
  Its author is unavailable. The robots **have driven before**, but the code/config that did it
  **may not be pushed** to GitHub, so trust what is on the Pi over the GitHub version.

## 1. The system and what changed

Whiteboard design (`whiteboard.jpg`): a birds-eye camera on a "master" Pi streams to a laptop. The laptop
finds the robots (ArUco markers) and the person (YOLO), plans where the robots should go, and sends them
set points over WiFi. Each robot Pi drives a mecanum wheel base and a pan/tilt speaker mount.

**MVP decisions (made 2026-11-19; follow these):**

1. **The laptop closes the position loop.** The laptop computes each robot's body velocity from the camera
   pose and sends it over UDP at 10 Hz. The robot Pi only turns velocity into wheel speeds and stops by
   itself if commands stop. The robot's encoder odometry, navigation, dashboards and vision receiver in
   `capstone_base` are **not used**.
2. **Pan/tilt stays at neutral the whole time.** The ArUco markers are mounted on the pan/tilt speaker
   head, so if it moves the camera reports the head's direction instead of the chassis'. The whole robot
   turns to face the listener instead. No face tracking in the MVP.
3. Formation: the two robots form a listening triangle with the user (1.5 m from the user, 60 deg apart),
   each facing the user. Robot 1 takes the user's left, robot 2 the right. All of that is on the laptop.

```
master Pi camera ──Ethernet──► laptop: vision.main ──UDP 5005──► laptop: control.main
                                                                   │ UDP 5700, 10 Hz (body velocity)
                                                                   ▼ ▲ status reply
                                              robot Pi: drive_server.py ──I2C──► Hiwonder board ──► wheels
```

## 2. Your job (robot side)

Deliverable: on each robot Pi, `robot/drive_server.py` runs with `--hardware`, moves the robot correctly
when sent commands, and the measurements in section 6 are reported back to Jeffrey (who runs the laptop
side).

### 2.1 Facts about the robot Pi (from the robot repo's docs; verify)

- Raspberry Pi 3 (2.4 GHz WiFi only). Until it joins the robot network (section 2.6) it runs its own
  hotspot `Capstone-Pi`; SSH is `ssh user@10.42.0.1` from a machine on that hotspot. That hotspot's
  password is in the team's Obsidian notes (Capstone → Code Reference → Pi - WiFi).
- Wheel board: Hiwonder 4-channel encoder motor module on I2C bus 1, address 0x34 (52), motor_type 3.
- Wheel mapping used by keyboard teleop: `~/capstone-base/config/teleop.json`
  (`wheel_channels [1,3,2,4]` = FL,FR,RL,RR -> M1,M3,M2,M4; `motor_signs [1,-1,-1,1]`, `max_speed_counts 30`).
  M1 front-left, M2 rear-left, M3 front-right, M4 rear-right.
- Known-good manual drive (if it still works, the wheel config is right):
  `cd ~/capstone-base && .venv/bin/capstone-teleop --backend hiwonder --config config/teleop.json --speed 5`
  (W/S forward/back, A/D strafe, Q/E rotate, Space stop, C quit).
- Pan/tilt: two LX-224 servos on serial `/dev/ttyS0` (IDs 1 = pan, 2 = tilt), separate from the wheels.

### 2.2 First: find what actually drove the robot

1. `cd ~/capstone-base && git status && git log --oneline -5 && git diff --stat` — note any unpushed or
   uncommitted changes. Don't discard anything.
2. Find the wheel config that worked: `config/teleop.json`, or `config/robot.local.json` (gitignored),
   or ask the teammate. Confirm with a short lifted-wheel teleop run.
3. Confirm `from capstone_base.hiwonder_i2c import HiwonderConfig, HiwonderController` imports in
   `.venv/bin/python`, and that `HiwonderController` has `initialize()`, `set_motor_speed_counts(M1..M4)`
   and `stop()` (the drive server uses only these; they have been stable since the repo's first commit).

### 2.3 Install the drive server

```bash
scp drive_server.py user@10.42.0.1:~/      # from the zip's robot/ folder
ssh user@10.42.0.1
cd ~/capstone-base
.venv/bin/python ~/drive_server.py --robot-id 1                                    # simulation: prints, no I2C
.venv/bin/python ~/drive_server.py --robot-id 1 --hardware                         # real motors
.venv/bin/python ~/drive_server.py --robot-id 1 --hardware --hiwonder-config config/robot.local.json  # other config
```

Options: `--counts-per-meter` (default 13000, an estimate), `--turn-radius` (default 0.2 m),
`--max-counts` (25), `--accel` (40 counts/10ms per s), `--timeout` (0.5 s), `--port` (5700),
`--allow-from <laptop IP>`. `--hiwonder-config` accepts teleop.json style or a robot.local.json with a
nested `"hiwonder"` object. Only one program may own the motor board: stop teleop/dashboards first.

### 2.4 Test it on the Pi alone (no laptop needed)

Run the server in one SSH session and this in another. Start with the **wheels lifted**, someone at the
power switch:

```bash
python3 - <<'EOF'
import json, socket, time
VX, VY, WZ, SECONDS = 0.1, 0.0, 0.0, 2.0      # forward m/s, left m/s, CCW rad/s
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
end = time.time() + SECONDS
while time.time() < end:
    s.sendto(json.dumps({"type": "drive", "robot_id": 1, "seq": 0, "vx": VX, "vy": VY, "wz": WZ}).encode(),
             ("127.0.0.1", 5700))
    time.sleep(0.1)
s.sendto(json.dumps({"type": "stop", "robot_id": 1}).encode(), ("127.0.0.1", 5700))
EOF
```

Check, in order:
1. `vx=0.1`: every wheel rolls the robot **forward**. The simulation-mode log for this is
   `M1..M4 counts/10ms: [13, -13, -13, 13]` (Hiwonder's forward pattern).
2. `vy=0.1`: strafes **left**. `wz=0.5`: turns **counter-clockwise** seen from above.
3. After the sender ends, the server logs a stop, and if the sender is killed mid-run it logs
   `Command timeout: wheels stopped` within 0.5 s.
4. Then on the floor, short and slow.

**"Forward" must be the speaker's front**: the face whose marker the vision config calls face 0. If the
robot's mechanical front and the speaker's front differ, tell Jeffrey which marker is on the speaker front
rather than changing code.

If a direction is wrong, fix the wheel config (`motor_signs` / `wheel_channels`) and re-test with teleop
too. Don't change the mixing in `drive_server.py` unless the config can't express the fix.

### 2.5 Calibrate speed with a tape measure

On the floor, with the defaults:
- Send `vx=0.1` for `SECONDS=3` and measure the distance traveled D (meters). New
  `--counts-per-meter = current × 0.3 / D` (the speed ramp costs ~2 cm, so ignore small differences).
- After fixing that, send `wz=0.5` for `SECONDS=4` (commands 115 deg) and measure the angle turned A.
  New `--turn-radius = current × 115 / A`.

(Jeffrey can repeat this more precisely from the laptop with `python -m control.drive_test`, which measures
with the camera.)

### 2.6 Network: join the master Pi's access point

The master Pi now runs the robot WiFi (details in `docs/NETWORK.md`):

| Setting | Value |
|---|---|
| SSID | `capstone-robots`, WPA2-PSK, 2.4 GHz channel 1. Get the password from Jeffrey (not in any file). |
| Robot 1 Pi | static `192.168.50.11/24`, gateway `192.168.50.1` |
| Robot 2 Pi | static `192.168.50.12/24`, gateway `192.168.50.1` |
| Laptop | `192.168.1.10`, reaches `192.168.50.0/24` through the master Pi (`192.168.1.20`) |

The gateway matters: the laptop is on a different subnet, so drive-server replies only reach it through
`192.168.50.1`. Example with NetworkManager on robot 1 (do this from a session that will survive the WiFi
switch, e.g. on the Pi's own console, or SSH over the new network afterwards at `192.168.50.11`):

```bash
sudo nmcli con add type wifi ifname wlan0 con-name capstone-robots ssid capstone-robots \
  wifi-sec.key-mgmt wpa-psk wifi-sec.psk '<password>' \
  ipv4.method manual ipv4.addresses 192.168.50.11/24 ipv4.gateway 192.168.50.1 ipv6.method disabled \
  connection.autoconnect yes connection.autoconnect-priority 20
sudo nmcli con modify capstone-hotspot connection.autoconnect-priority 0   # keep the bench hotspot as fallback
sudo nmcli con up capstone-robots
```

The Pi 3 has one radio, so it can't run its `Capstone-Pi` bench hotspot and join `capstone-robots` at
the same time. With the priorities above it joins `capstone-robots` when it's in range and falls back to
the hotspot otherwise. Check the hotspot profile's real name with `nmcli con show` first. Then, from the
laptop, `ping 192.168.50.11` and `ssh user@192.168.50.11` should work.

### 2.7 Optional: start on boot

A systemd unit running `~/capstone-base/.venv/bin/python /home/user/drive_server.py --robot-id N --hardware
<calibrated flags>` with `WorkingDirectory=/home/user/capstone-base` and `Restart=on-failure` is fine,
**only after** the tests above pass. The server stops the wheels on SIGTERM and on any I2C error.

## 3. The command protocol (don't change it without telling Jeffrey)

UDP JSON, robot listens on port 5700, replies to the sender's address.

```json
laptop -> robot  {"type": "drive", "robot_id": 1, "seq": 17, "vx": 0.10, "vy": 0.0, "wz": 0.2}
                 {"type": "stop",  "robot_id": 1, "seq": 18}
robot -> laptop  {"type": "status", "robot_id": 1, "seq": 17, "state": "driving",
                  "motors": [5, -5, -5, 5], "error": null}
```

`vx` forward m/s, `vy` left m/s, `wz` counter-clockwise rad/s, in the robot's body frame. A command for
another `robot_id` is rejected. The server clamps speed to 0.3 m/s and 1 rad/s, ramps wheel speeds, and
stops immediately on a stop command or after `--timeout` without a drive command.

## 4. Pan/tilt for the MVP

Put the pan/tilt at neutral (speaker facing the robot's front, level) and keep it there: the servos hold
position while powered with torque on. Nate's `capstone-pan-tilt` dashboard (port 8082, see
`docs/pan-tilt-dashboard.md` in his repo) can do this; it uses the serial bus, so it can run alongside the
drive server. Don't add pan/tilt motion to the drive server.

## 5. Safety

Wheels lifted for first tests; a person at the power switch for the first floor runs; short, slow commands.
Run only one program on the motor board at a time.

## 6. Report back to Jeffrey

1. Robot ID ↔ physical robot, and each robot Pi's IP on the network the laptop will use.
2. Which wheel config file drove correctly, and any sign/channel changes made.
3. Calibrated `--counts-per-meter` and `--turn-radius` (and the exact command line used).
4. **Marker geometry on the speaker head, with pan/tilt at neutral** (the laptop's vision config needs it):
   - which marker ID is on which face of which robot (front = speaker front, left, back, right);
     the laptop config currently assumes robot 1 = IDs 0,1,2,3 and robot 2 = IDs 4,5,6,7 in
     front,left,back,right order;
   - height of the marker centers above the floor (config assumes 0.30 m);
   - horizontal distance from a marker's surface to the pan axis (config assumes 0.15 m);
   - whether the pan axis is above the chassis center, and if not, the offset;
   - printed marker size (black square edge; config assumes 0.10 m).
5. Anything unpushed/changed in `~/capstone-base` that the system depends on.

## 7. Files in this zip

| File | What |
|---|---|
| `ROBOT_SETUP_HANDOFF.md` | this file |
| `whiteboard.jpg` | the team's system diagram |
| `robot/drive_server.py` | **the program to install on each robot Pi** |
| `docs/MVP_RUNBOOK.md` | full bring-up order, including the laptop side |
| `docs/NETWORK.md` | planned addresses and ports |
| `config.yaml` | laptop config (`robots:` = marker geometry, `control:` = robot addresses, limits) |
| `control/` | laptop side, for reference: planner, controller, UDP link, main loop, drive_test |
| `tests/test_drive_server.py` | drive server tests (12, no hardware). From the zip folder: `python -m pytest -o pythonpath=. tests/test_drive_server.py` |
