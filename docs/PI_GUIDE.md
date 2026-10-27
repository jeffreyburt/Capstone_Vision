# Raspberry Pi Install & Test Guide

This takes you from a blank SD card to robots showing up on the controller laptop. Each part ends with a
check, so if something breaks you know which step caused it. Run every command on the Pi from inside the
`Capstone_Vision` folder unless a step says **(laptop)**.

| Part | What you get | Needs markers? | Time |
|---|---|---|---|
| 1–3 | Pi running, camera working | no | 30 min |
| 4–5 | Code installed and self-checked | no | 20 min |
| 6 | Full pipeline running on the Pi (simulated video) | no | 10 min |
| 7 | Live camera detecting a printed marker | yes | 10 min |
| 8 | Range test at 5–20 ft | yes | 30 min |
| 9–10 | Lens + floor calibration, real positions in meters | yes | 45 min |
| 11 | Positions arriving on the controller laptop | yes | 10 min |
| 12 | Start automatically on boot (optional) | — | 5 min |

---

## What you need

- Raspberry Pi 4 (2 GB+), official 5 V / 3 A USB-C power supply, microSD card (16 GB+)
- Camera Module 3 and its ribbon cable (the standard cable it ships with fits the Pi 4)
- Laptop on the same network (WiFi or Ethernet)
- For later parts: printer, tape measure, masking tape, stiff board (foam board or cardboard),
  and a way to mount the camera about 2 m high (tripod, shelf, or tape to a wall)

---

## 1. Flash the SD card (laptop)

1. Install **Raspberry Pi Imager** from raspberrypi.com/software and open it.
2. Choose **Raspberry Pi 4** → **Raspberry Pi OS (other)** → **Raspberry Pi OS Lite (64-bit)**.
   - Lite has no desktop. That's fine, because the debug view streams to your laptop's browser.
     If you'd rather plug in a monitor, the normal 64-bit desktop version works too.
   - It **must be 64-bit**.
3. When it asks about OS customisation, choose **Edit settings** and set:
   - hostname: `vision-pi`
   - a username and password (remember them)
   - your WiFi network and country (skip this if you'll use Ethernet)
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

Insert the card, power the Pi, and wait about 2 minutes. Then, from your laptop (Windows PowerShell or
Terminal):

```bash
ssh <username>@vision-pi.local
```

If `vision-pi.local` isn't found, get the Pi's IP address from your router's device list and use
`ssh <username>@<ip>`.

Update the Pi, then reboot:

```bash
sudo apt update && sudo apt full-upgrade -y && sudo reboot
```

SSH back in, then test the camera:

```bash
rpicam-hello --list-cameras
```

✅ **Check:** the output lists **imx708** (that's Camera Module 3). If it says "No cameras available", see Troubleshooting.

Take a test photo:

```bash
rpicam-still -o test.jpg
```

Then, from a second terminal **(laptop)**, copy the photo over and look at it:

```bash
scp <username>@vision-pi.local:test.jpg .
```

✅ **Check:** the picture is sharp and correctly exposed.

## 4. Copy the code to the Pi

**Option A: GitHub (recommended for a team).** Push this repo to GitHub, then on the Pi:

```bash
git clone https://github.com/<you>/Capstone_Vision.git
cd Capstone_Vision
```

**Option B: copy straight from your laptop.** In PowerShell on the laptop:

```bash
tar --exclude=.venv --exclude=sim --exclude=results -czf vision.tgz -C C:\Users\Jeffrey\Documents Capstone_Vision
```
```bash
scp vision.tgz <username>@vision-pi.local:~
```

Then on the Pi:

```bash
tar xzf vision.tgz && cd Capstone_Vision
```

To update the Pi later: with A, run `git pull`. With B, repeat the copy. Your `calib/` files on the Pi are
overwritten by B, so back them up first.

## 5. Install and self-check

```bash
bash scripts/setup_pi.sh
```

This installs Picamera2 from apt, creates a Python virtualenv (`.venv`) that can see it, installs the pip
packages, and checks the imports. It takes about 5–15 minutes. It should end with `Setup OK`.

Activate the virtualenv. **Do this every time you log in:**

```bash
source .venv/bin/activate
```

Run the unit tests:

```bash
python -m pytest
```

✅ **Check:** `25 passed`. These test the geometry, robot pose math, tracking and UDP. They also detect
robots in rendered images at up to 20 ft. No camera is needed.

Run the Pi self-check:

```bash
python tools/pi_check.py
```

✅ **Check:** `ALL CHECKS PASSED`. Write these numbers down for your report:

- **processing speed (ms/frame):** detection time per frame on the Pi 4. Under 100 ms (10+ fps) is good.
- **capture fps:** how fast the camera delivers frames.
- **live pipeline rate:** the real end-to-end rate. This is what you'll actually get.

If the rate is too low, set `width: 1280` and `height: 720` in `config.yaml` and rerun. That is faster, but
gives shorter range; part 8 tells you whether that's acceptable.

## 6. Full pipeline on a simulated video (no markers needed)

This checks the whole system on the Pi, including the debug stream, before you print anything.

```bash
python tools/make_sim_video.py --seconds 10
```

Rendering takes a few minutes on the Pi. Then run the pipeline on the result:

```bash
python -m vision.main --config sim/config.yaml --stream --loop
```

On the laptop, open **http://vision-pi.local:8080/** in a browser.

✅ **Check:**
- Two simulated robots, with green outlines around their markers.
- The top-down map on the right shows both robots moving, with heading arrows.
- The SSH terminal prints a status line every second.

Press `Ctrl+C` to stop.

## 7. Live camera with a printed marker

1. Print markers **(laptop or Pi)** with `python tools/generate_markers.py`. This writes
   `markers/markers_DICT_4X4_50_15cm.pdf`. Print it at **100% / "Actual size"**, not "Fit to page".
2. **Measure the black square** with a ruler. If it isn't exactly 15.0 cm, put the real value in
   `config.yaml` → `markers.size_m` (for example `0.148`).
3. Tape one page to a box or board, keeping the white border around the black square.

Start the system:

```bash
python -m vision.main --stream
```

Open http://vision-pi.local:8080/ and hold the marker 1–3 m in front of the camera.

✅ **Check:** a green outline appears with the marker's ID and its size in pixels. The overlay says
`(no floor calibration)` and the robot list stays empty. **That's expected until part 10.**

If the picture is blurry, the focus is fixed at 4 m. For close-up testing, set
`camera.focus_dioptres: null` in `config.yaml` (autofocus), and set it back for the real setup.

## 8. Range test (the most important measurement)

This decides the marker size and camera resolution. Mount the camera roughly where it will really go
(about 2 m high, tilted slightly down). Then mark distances on the floor with tape: **5, 10, 15 and 20 ft**.

```bash
python tools/range_test.py --stream
```

For each distance:
1. Put the marker at that distance, facing the camera, at the real mounting height (about 30 cm).
2. Type a label such as `15ft` and press Enter.
3. Keep it still for 10 s. Then repeat with the marker turned about 45°.

✅ **Check (what good looks like):**

| Distance | Detection rate | px/cell |
|---|---|---|
| 5–15 ft | ~100% | — |
| 20 ft | ≥ 95% | ≥ 4 |

Results are appended to `results/range_test.csv`.

If 20 ft is weak, try these in order:
1. Raise `width`/`height` to 1920×1080 if you lowered them.
2. Use bigger markers: `python tools/generate_markers.py --size-cm 20`.
3. Improve the lighting.
4. Check the focus.

Run the test again after each change.

## 9. Lens calibration

Do this once per camera, at the resolution you'll actually run.

1. Make the checkerboard: `python tools/calibrate_camera.py --make-board board.pdf`.
2. Print it at 100% and tape it **perfectly flat** to stiff board. Measure one square; it should be 25 mm.
   If it isn't, pass `--square-mm <value>`.
3. Run the calibration and open http://vision-pi.local:8080/ to watch it:

```bash
python tools/calibrate_camera.py --auto --stream
```

4. Hold the board 1–2 m from the camera. Every 2 s the tool tries to grab a view, and the preview
   pauses briefly while it does.
5. Move the board between captures: all corners of the frame, near and far, tilted left, right, up and down.
6. After 20 good views it calibrates automatically.

✅ **Check:** `RMS error` **below 1.0 px** and a horizontal FOV of roughly 60–70°. The result is written to
`calib/camera_intrinsics.yaml`. The captured images are kept in `calib/images/`, so you can re-run offline
with `--images "calib/images/*.jpg"`.

## 10. Floor calibration

Do this every time the camera is moved or re-aimed.

1. **Fix the camera in its final position.** It must not move after this step.
2. Choose the coordinate frame. Recommended:
   - origin = the point on the floor directly below the camera;
   - x to the right as seen from the camera;
   - y straight out from the camera.
3. Put **6–8 tape marks** spread over the whole robot area, near and far, left and right. Measure each mark's
   (x, y) in meters and write them, **in order**, into `calib/floor_points.yaml`.
4. Take a snapshot. Stop `vision.main` first if it's running, because only one program can use the camera:

```bash
python tools/record.py --snapshot calib/floor.jpg
```

5. Clicking needs a screen, so do it on the laptop. First copy the files over **(laptop)**:

```bash
scp <username>@vision-pi.local:Capstone_Vision/calib/floor.jpg <username>@vision-pi.local:Capstone_Vision/calib/camera_intrinsics.yaml calib/
```

   Also make sure the laptop's `calib/floor_points.yaml` matches the Pi's.

6. Click the marks in order **(laptop)**:

```bash
python tools/calibrate_floor.py --image calib/floor.jpg
```

   Click each tape mark in the order listed in `floor_points.yaml` (U = undo), then press **S** to solve.

   ✅ **Check:**
   - `Reprojection error` is a few px or less.
   - `height` is close to the camera's real height (about 2.0 m).
   - The yellow and green 1 m grid drawn on the image lines up with the real floor.

7. Copy the result back to the Pi **(laptop)**:

```bash
scp calib/floor_pose.yaml <username>@vision-pi.local:Capstone_Vision/calib/
```

**Accuracy check:** run `python -m vision.main --stream` and put a marked box or robot on each tape mark.
The position on the map (and in the SSH status line) should match the mark. Record the error at each
mark; aim for ±10–15 cm at 20 ft. `range_test.py` also prints position jitter once the floor is calibrated.

Before testing on the real robots:
- Mount **one marker per side, upright as printed** (the ID text goes below the marker).
- Robot 1 uses IDs 0/1/2/3 and robot 2 uses 4/5/6/7, in the order front, left, back, right.
- Set `markers.center_height_m` (floor to marker center) and `center_offset_m` (marker surface to robot
  center) in `config.yaml`.

## 11. Send positions to the controller laptop

1. Find the laptop's IP address. On Windows, run `ipconfig` and look for "IPv4 Address".
2. Start the receiver **(laptop)**:

```bash
python tools/receiver_example.py
```

   Windows may show a firewall prompt; **allow Python on Private networks**.

3. Start the vision system on the Pi:

```bash
python -m vision.main --stream --host <laptop-ip>
```

✅ **Check:** the laptop prints lines such as `R1: (0.52, 3.10) m, 88.0 deg, age 0.03s` about 10 times a second.

To avoid typing the IP each time, set it in `config.yaml` → `output.host`. Or, if the laptop's IP keeps
changing, use `host: 255.255.255.255` with `broadcast: true`, which sends to everyone on the local network.

The controls team can develop without the Pi by using `python tools/mock_publisher.py`. It sends fake data
in the same format.

## 12. Start automatically on boot (optional)

```bash
sudo tee /etc/systemd/system/vision.service > /dev/null <<EOF
[Unit]
Description=Capstone central vision
After=network-online.target
Wants=network-online.target

[Service]
User=$USER
WorkingDirectory=$PWD
Environment=PYTHONUNBUFFERED=1
ExecStart=$PWD/.venv/bin/python -m vision.main --stream
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF
```

Then enable it and watch its log:

```bash
sudo systemctl daemon-reload && sudo systemctl enable --now vision
```
```bash
journalctl -u vision -f
```

To stop it, which you need to do before running any other tool that uses the camera, run
`sudo systemctl stop vision`. To keep it from starting on boot, run `sudo systemctl disable vision`.

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `rpicam-hello`: **No cameras available** | Power off and reseat the ribbon at both ends: contacts toward the HDMI ports, latch fully closed. Run `sudo apt full-upgrade` and reboot. |
| Camera works but `vision.main` says the camera is busy / "Device or resource busy" | Another program has the camera. Stop the other tool, or run `sudo systemctl stop vision`. |
| `setup_pi.sh` fails with `$'\r': command not found` | The file got Windows line endings. Run `sed -i 's/\r$//' scripts/setup_pi.sh` and rerun. |
| `import picamera2` fails with a **numpy** error ("dtype size changed", "compiled using NumPy 1.x") | The pip numpy doesn't match the apt-built Picamera2. Run `.venv/bin/pip install "numpy<2" "opencv-python==4.10.0.84"`, then `python tools/pi_check.py` again. |
| `ImportError: libGL.so.1` | Run `sudo apt install -y libgl1`. |
| Browser can't open `vision-pi.local:8080` | Use the Pi's IP instead (`hostname -I` on the Pi). Check that `vision.main` was started with `--stream`. |
| Laptop receives nothing | Check the Windows firewall allows Python (Private), that `--host` is the laptop's current IP, and that both machines are on the same network. University WiFi often blocks device-to-device traffic, so use a phone hotspot, a travel router or Ethernet. |
| Markers detected, but no robots in the output | Floor calibration is missing (the overlay says so), or the marker IDs aren't listed under `robots:` in `config.yaml`. |
| Robot position is consistently about 30 cm off, or heading is 180° wrong | That marker is mounted upside down, or `center_offset_m` / face angles are wrong. |
| Positions are good near the camera but bad far away | Redo the floor calibration with more tape marks in the far area. Check the camera hasn't moved. |
| Low fps | Lower the resolution to 1280×720. Close other programs. Check `vcgencmd get_throttled` (anything other than `0x0` means power or heat problems). A heatsink or fan helps. |
| Blurry images | Set `camera.focus_dioptres` to `1 / distance_m` for the middle of your area (0.25 = 4 m), or `null` for autofocus. |
