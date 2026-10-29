# Install & Test Guide (Pis + controller laptop)

This takes you from blank SD cards to robot and user positions arriving on the controller laptop.
Each part ends with a ✅ check, so if something breaks you know which step caused it.

**Who does what:**

| Machine | Runs | Command |
|---|---|---|
| **Master Pi** | Birds-eye camera sender | `python -m camstream.sender birdseye` |
| **Robot 1 / 2 Pi** | Robot camera sender | `python -m camstream.sender robot1` (or `robot2`) |
| **Laptop** | All vision processing: robots, person detection, output | `python -m vision.main --show` |

The Pis only capture and stream. Calibration, testing and processing all happen on the laptop, where you
have a screen. The network (master Pi WiFi access point, addresses, time sync) is set up separately; see
[NETWORK.md](NETWORK.md). Until it's ready, part 6 explains how to test over a direct Ethernet cable.

| Part | What you get | Needs markers? | Time |
|---|---|---|---|
| 1–5 | A Pi set up, camera working, code installed and self-checked (repeat for each Pi) | no | 45 min / Pi |
| 6 | Laptop set up, simulated end-to-end test | no | 20 min |
| 7 | Master Pi streaming to the laptop | no | 10 min |
| 8 | Live marker detection | yes | 10 min |
| 9 | Range test at 5–20 ft | yes | 30 min |
| 10–11 | Lens + floor calibration, real positions in meters | yes | 45 min |
| 12 | Person detection check | no | 10 min |
| 13 | Robot camera streams over WiFi | no | 15 min |
| 14 | Start automatically on boot | — | 5 min |

---

## What you need

- Per Pi: Raspberry Pi 4 (2 GB+), official 5 V / 3 A USB-C power supply, microSD card (16 GB+),
  Camera Module 3 and its ribbon cable (the standard cable it ships with fits the Pi 4), heatsink or fan
- Controller laptop with an Ethernet port (or a USB Ethernet adapter)
- For later parts: printer, tape measure, masking tape, stiff board (foam board or cardboard),
  and a way to mount the birds-eye camera about 2 m high

---

## 1. Flash the SD card (laptop)

1. Install **Raspberry Pi Imager** from raspberrypi.com/software and open it.
2. Choose **Raspberry Pi 4** → **Raspberry Pi OS (other)** → **Raspberry Pi OS Lite (64-bit)**.
   It **must be 64-bit**. Lite has no desktop, which is fine because the Pis run headless.
3. When it asks about OS customisation, choose **Edit settings** and set:
   - hostname: `master-pi`, `robot1-pi` or `robot2-pi` (from [NETWORK.md](NETWORK.md))
   - a username and password (use the same on all Pis to keep things simple)
   - WiFi: for now, your home network, so the Pi can download packages. The network-setup work moves the
     robot Pis onto the master Pi's network later.
   - **Services → Enable SSH** (password authentication)
4. Write the card.

## 2. Connect the camera

**Unplug the Pi's power first.** On the Pi 4 the camera connector sits between the micro-HDMI ports and the
audio jack.
1. Gently lift the connector's plastic latch.
2. Insert the ribbon with the **silver contacts facing the HDMI ports**.
3. Push the latch back down. Plug the other end into the camera the same way.

Peel the protective film off the camera lens if it's still on.

## 3. First boot and camera test

Power the Pi and wait about 2 minutes. From the laptop:

```bash
ssh <username>@master-pi.local
```

If the name isn't found, look up the Pi's IP in your router's device list and use `ssh <username>@<ip>`.

Update the Pi, then reboot:

```bash
sudo apt update && sudo apt full-upgrade -y && sudo reboot
```

SSH back in, then test the camera:

```bash
rpicam-hello --list-cameras
```

✅ **Check:** the output lists **imx708** (Camera Module 3).

## 4. Copy the code to the Pi

**Option A: GitHub (recommended).** Push this repo to GitHub, then on the Pi:

```bash
git clone https://github.com/<you>/Capstone_Vision.git && cd Capstone_Vision
```

**Option B: copy from the laptop.** In PowerShell on the laptop:

```bash
tar --exclude=.venv --exclude=sim --exclude=results --exclude=*.pt -czf vision.tgz -C C:\Users\Jeffrey\Documents Capstone_Vision
```
```bash
scp vision.tgz <username>@master-pi.local:~
```

Then on the Pi:

```bash
tar xzf vision.tgz && cd Capstone_Vision
```

To update later: with A, run `git pull`. With B, repeat the copy.

## 5. Install and self-check

```bash
bash scripts/setup_pi.sh
```

This installs Picamera2 from apt, creates a virtualenv (`.venv`), installs the pip packages and checks the
imports. It takes about 5–15 minutes and should end with `Setup OK`.

Activate the virtualenv. **Do this every time you log in:**

```bash
source .venv/bin/activate
```

Then run the tests and the Pi self-check:

```bash
python -m pytest
```
```bash
python tools/pi_check.py
```

✅ **Check:**
- `pytest` passes, with 1 skipped. The skipped one is the YOLO test, which only runs on the laptop.
- `pi_check` ends with `ALL CHECKS PASSED`. Note the **capture fps** for your report.

**Repeat parts 1–5 for each robot Pi** (hostnames `robot1-pi`, `robot2-pi`).

---

## 6. Laptop setup and simulated test (no Pi needed)

On the laptop, in the `Capstone_Vision` folder:

```bash
py -3.12 -m venv .venv
```
```bash
.venv\Scripts\activate
```
```bash
pip install -r requirements-laptop.txt
```

This includes YOLO person detection, which pulls in PyTorch (a large download). If the laptop has an
NVIDIA GPU, installing the CUDA build of PyTorch makes detection much faster; it works on the CPU otherwise.

```bash
python -m pytest
```

✅ **Check:** all tests pass. The first run downloads the YOLO model (`yolo11n.pt`, about 6 MB).

Rehearse the whole system on one machine. Three senders play the role of the three Pis, and
`CAMSTREAM_HOST` tells the laptop to find all streams on itself. Each command goes in its own terminal:

```bash
python tools/make_sim_video.py --person
```
```bash
python -m camstream.sender birdseye --source sim/sim.mp4
```
```bash
python -m camstream.sender robot1 --source test
```
```bash
$env:CAMSTREAM_HOST="127.0.0.1"; python -m vision.main --config sim/config.yaml --source stream://birdseye --show
```

✅ **Check:** the window shows two simulated robots with green marker outlines and a walking person with a
red box. The top-down map shows both robots (with heading arrows) and the user circle.

**Direct cable, before the real network exists:** plug the master Pi straight into the laptop. Set the
laptop's Ethernet IPv4 to `192.168.50.10` / `255.255.255.0`. On the Pi:

```bash
sudo nmcli con mod "Wired connection 1" ipv4.method manual ipv4.addresses 192.168.50.1/24 && sudo nmcli con up "Wired connection 1"
```

That gives the Pi the address [NETWORK.md](NETWORK.md) expects. After that, `ssh <username>@192.168.50.1`
works, and so does everything below.

## 7. Master Pi → laptop stream

On the **master Pi**:

```bash
python -m camstream.sender birdseye
```

It prints fps, Mbps and kB/frame every 5 s. On the **laptop**:

```bash
python -m camstream.viewer birdseye
```

✅ **Check:**
- The live camera image appears in a window.
- The printed stats show about 15 fps, `dropped` staying near 0, and roughly 20–40 Mbps (this is on Ethernet, so plenty of headroom).

If the sender's `encode` time is above about 60 ms, the Pi can't keep up with 15 fps at 1920×1080. Lower
`streams.birdseye.quality` or `max_fps`.

## 8. Live marker detection (laptop)

1. Print the markers with `python tools/generate_markers.py`. Print `markers/*.pdf` at **100% / actual size**.
2. **Measure the black square.** If it isn't exactly 15.0 cm, set `markers.size_m` in `config.yaml`.
3. Tape one page to a box, keeping the white border.

With the sender from part 7 still running:

```bash
python -m vision.main --show
```

✅ **Check:** holding the marker 1–3 m from the camera draws a green outline with its ID. The overlay says
`(no floor calibration)` and no robots are reported yet; that's expected until part 11.

If the image is blurry up close, the focus is fixed at 4 m. That's correct for the real setup.

## 9. Range test (laptop)

Mount the camera roughly where it will really go: about 2 m high, tilted slightly down. Tape distances on
the floor: **5, 10, 15, 20 ft**. Then run:

```bash
python tools/range_test.py
```

At each distance, put the marker there facing the camera at about 30 cm high, type a label such as `15ft`,
and keep it still for 10 s. Repeat with the marker turned about 45°.

✅ **Check:**

| Distance | Detection rate | px/cell |
|---|---|---|
| 5–15 ft | ~100% | — |
| 20 ft | ≥ 95% | ≥ 4 |

Results go to `results/range_test.csv`.

If 20 ft is weak, try these in order:
1. Raise `streams.birdseye.quality` to 90–95. JPEG compression can blur small markers.
2. Use bigger markers: `python tools/generate_markers.py --size-cm 20`.
3. Improve the lighting.
4. Check the focus.

## 10. Lens calibration (laptop)

1. Print `python tools/calibrate_camera.py --make-board board.pdf` at 100%, tape it **flat** to stiff board,
   and check one square measures 25 mm (otherwise pass `--square-mm`).
2. Run:

```bash
python tools/calibrate_camera.py --show
```

3. Hold the board 1–2 m from the camera and press **SPACE** for each view. Take about 20 views: all corners
   of the frame, near and far, tilted in different directions.
4. Press **C** to calibrate.

✅ **Check:** `RMS error` below 1.0 px and a horizontal FOV of roughly 60–70°. The result is written to `calib/camera_intrinsics.yaml`.

## 11. Floor calibration (laptop)

Do this every time the birds-eye camera is moved.

1. Fix the camera in its final position.
2. Tape **6–8 marks** over the whole area, near and far. Write their measured (x, y) in meters, in order,
   into `calib/floor_points.yaml`. Recommended frame: origin on the floor directly below the camera,
   x to the right, y away from the camera.
3. Take a snapshot and click the marks:

```bash
python tools/record.py --snapshot calib/floor.jpg
```
```bash
python tools/calibrate_floor.py --image calib/floor.jpg
```

Click each mark in order (U = undo), then press **S**.

✅ **Check:**
- Reprojection error is a few px or less.
- The camera height comes out near 2.0 m.
- The 1 m grid lines up with the real floor.

**Accuracy check:** run `python -m vision.main --show`, put a marked box on each tape mark, and compare.
Aim for ±10–15 cm at 20 ft.

**Robots:** mount one marker per side, **upright as printed**:
- Robot 1 uses IDs 0/1/2/3 and robot 2 uses 4/5/6/7, in the order front, left, back, right.
- Set `markers.center_height_m` and `center_offset_m` in `config.yaml`.

## 12. Person detection (laptop)

With `python -m vision.main --show` running, walk around the area.

✅ **Check:**
- You get a red box with your foot point marked, and a red user circle on the map that follows you.
- Your height is shown roughly right (±10 cm).
- The status line prints `user (x, y)`.
- Standing on a tape mark gives roughly the right position.

If the robots block your feet, the position comes from your head at `people.head_height_m` instead; set
that to your real height for best results.

The controller gets everything over UDP. To see exactly what it receives:

```bash
python tools/receiver_example.py
```

## 13. Robot camera streams (robot Pis, over WiFi)

This needs the robot network from [NETWORK.md](NETWORK.md), or temporarily any shared WiFi with
`streams.robot1.host` set to the Pi's current IP.

On each **robot Pi**:

```bash
python -m camstream.sender robot1
```

(Use `robot2` on the other robot.) Then on the **laptop**:

```bash
python -m camstream.viewer robot1 robot2
```

For a bandwidth test without windows, run this while the robots drive around the room:

```bash
python -m camstream.viewer --stats
```

✅ **Check:**
- Each robot stream runs at about 10 fps and 2–4 Mbps.
- `dropped` stays low.
- `last` stays under about 0.2 s.

**Using the frames in code:** the robot-camera processing (pan/tilt check) belongs to someone else. They get
frames like this:

```python
from vision.config import load_config
from camstream.receiver import FrameReceiver
rx = FrameReceiver(load_config(), ["robot1", "robot2"])
f = rx.get_latest("robot1")   # f.image (BGR array), f.seq, f.t_capture, f.meta
```

To attach pan/tilt angles to each frame, the robot's control program runs the sender itself:

```python
from camstream.sender import CameraSender
sender = CameraSender.from_config(load_config(), "robot1").start()
sender.update_meta(pan_deg=..., tilt_deg=...)
```

See [camstream/sender.py](../camstream/sender.py) for details.

## 14. Start automatically on boot

On each Pi, run its own stream name:

```bash
bash scripts/install_sender_service.sh birdseye
```

Logs: `journalctl -u camstream-birdseye -f`.

To stop it, which you must do before running `pi_check.py` or any other tool that uses that Pi's camera:

```bash
sudo systemctl stop camstream-birdseye
```

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `rpicam-hello`: **No cameras available** | Power off and reseat the ribbon at both ends: contacts toward the HDMI ports, latch fully closed. Run `sudo apt full-upgrade` and reboot. |
| Sender: camera busy / "Device or resource busy" | Another program has the camera, often the boot service. Run `sudo systemctl stop camstream-<name>`. |
| `setup_pi.sh` fails with `$'\r': command not found` | Windows line endings: `sed -i 's/\r$//' scripts/*.sh` |
| `import picamera2` fails with a **numpy** error | Run `.venv/bin/pip install "numpy<2" "opencv-python==4.10.0.84"`, then rerun `pi_check.py`. |
| `ImportError: libGL.so.1` | `sudo apt install -y libgl1` |
| Laptop prints `Waiting for stream 'birdseye' from 192.168.50.1:5600` | The sender isn't running, or the address is wrong. Check `ping 192.168.50.1`, check the sender's terminal, and check `streams:` in `config.yaml`. |
| Stream works but is slow or `dropped` keeps rising | Not enough bandwidth (WiFi). Lower that stream's `max_fps`, `quality` or resolution in `config.yaml`. |
| Controller receives no UDP | Windows firewall: allow Python on Private networks. Check `output.host`/`port`. |
| Markers detected but no robots in the output | The floor calibration is missing, or the IDs aren't listed under `robots:`. |
| Robot about 30 cm off or heading 180° wrong | That marker is upside down, or `center_offset_m` / face angles are wrong. |
| No user detected | Check `people.enabled`, and that `ultralytics` is installed on the laptop (the startup log says if it's disabled). The whole body should be in view. |
| User position jumps when they walk behind a robot | Their feet are hidden. Set `people.head_height_m` to their height, or raise the camera. |
| Low fps on the Pi | Check `vcgencmd get_throttled` (`0x0` = OK). Add a heatsink or fan, and use the official power supply. |
