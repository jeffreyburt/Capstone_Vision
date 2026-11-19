# Capstone Head Tracker

A Raspberry Pi camera and two serial bus servos that keep a detected face near the center of the camera frame. Includes a local web dashboard, live video, manual limit capture, smooth sweeps, and short-term prediction when a face leaves view.

For use with the overhead vision and wheel-control system, follow the [integration runbook](../docs/HEAD_TRACKER.md).

## Where it runs

- The **Pi** captures camera frames, runs the web server, and controls the servos.
- A **connected browser** runs the bundled MediaPipe face detector and sends target coordinates to the Pi.
- **Internet access is not needed at runtime.** The browser and Pi must still be connected over a local network. Head tracking does **not** currently run autonomously with the browser disconnected. If observations stop arriving, movement stops.

## Hardware

Tested on Raspberry Pi 3 Model B and Raspberry Pi 4 Model B with Raspberry Pi OS/Debian 13, Camera Module 3 (IMX708), a Hiwonder BusLinker, and two Hiwonder LX-224 servos.

| Connection | Purpose |
| --- | --- |
| Pi physical pin 8 / GPIO14 TX | BusLinker RX |
| Pi physical pin 10 / GPIO15 RX | BusLinker TX |
| Pi GND | BusLinker GND |
| Camera connector | Camera Module 3 ribbon cable |
| External servo supply | BusLinker and servos; use the servo's specified voltage |

Set the BusLinker jumpers for **TTL/UART**, not USB. The servos use 115200 baud and must have distinct IDs: **1 = pan**, **2 = tilt**. Connect only one servo when assigning an ID. The program does not assign IDs automatically.

Turn the Pi off and disconnect power before changing ribbon or GPIO connections. Verify the camera ribbon is oriented correctly at both ends.

## Install

Clone this repository onto the Pi, then install system dependencies:

```sh
sudo apt update
sudo apt install python3-serial python3-picamera2 ffmpeg rpicam-apps i2c-tools
```

Picamera2 requires the Raspberry Pi camera stack. Use the system Python (`/usr/bin/python3`); a normal isolated virtual environment will not see the system camera libraries.

Enable the serial hardware and disable the serial login console:

```sh
sudo raspi-config
```

Select **Interface Options → Serial Port**, answer **No** to a login shell and **Yes** to serial hardware. Enable I2C if using additional I2C hardware; it is not required for the BusLinker servos themselves.

### Raspberry Pi 4 UART configuration used for this project

On the tested Pi 4, `/boot/firmware/config.txt` includes:

```ini
[all]
camera_auto_detect=1
enable_uart=1
dtoverlay=miniuart-bt
core_freq=500
core_freq_min=500
```

This routes the PL011 UART to GPIO14/15 and moves Bluetooth to the mini UART. The fixed 500 MHz core clock is the stock Pi 4 core frequency. A prior 250 MHz setting caused camera capture to fail with `failed to set up VPU clock` on this board. Do not apply Pi 4 clock settings blindly to other Pi models.

Reboot after editing boot settings, then check:

```sh
ls -l /dev/serial0
rpicam-hello --list-cameras
```

The tested Pi 4 maps `/dev/serial0` to `ttyAMA0`. Add your account to the device groups if required, then log out and back in:

```sh
sudo usermod -aG dialout,video,render "$USER"
```

## Run

From the repository directory:

```sh
python3 dashboard.py
```

Open **http://PI_IP_ADDRESS:8080/** from your browser. The server binds to all Pi network interfaces. Starting the server reads servo feedback but does not engage torque or start movement.

For startup after reboot:

```sh
sudo bash scripts/install-service.sh
```

The installer uses the repository's location and the account that invoked sudo. It enables `capstone-dashboard.service` and starts it immediately. Inspect it with:

```sh
systemctl status capstone-dashboard.service
journalctl -u capstone-dashboard.service -n 50 --no-pager
```

Run only one instance at a time; the server and standalone servo scripts must not compete for `/dev/serial0`.

## Dashboard workflow

Capture requires stopped tracking/sweeps and fresh valid readings from both servos. The message above
the capture buttons explains any block. Freshness uses the Pi's `serverTime` and `position_time`, so a
Mac/Pi clock difference does not disable capture or hide live positions. Deploy both `dashboard.py`
and `index.html` together, restart the stopped dashboard service, and refresh the browser after updating.

1. Support the mechanism and **Disengage torque**.
2. Move the camera by hand to one allowed endpoint; **Capture current position as A** records both servos.
3. Move to the other endpoint and capture B.
4. **Engage torque** to hold the current positions.
5. Start a smooth sweep, or **Start head tracking**.

**Head-tracking pan uses the full supported command range, 0–1000. Tilt uses the minimum and maximum of its A/B positions.** This is a software command range, not proof that every position is mechanically safe for a particular assembly. Sweeps use the exact A/B pairs entered in the form.

The camera image and detection input are rotated 180 degrees for the project's upside-down mounting. To change orientation, update both the image CSS in `index.html` and the input transform in `assets/head_detection.mjs`.

### Tracking controls

- Default maximum tracking speed: **180 units/sec**. Adjustable **5–400 units/sec**, approximately 1.2–96 degrees/sec for an LX-224. Actual speed depends on servo load and hardware limits.
- **Reverse pan/tilt** controls the direction mapping. Stop and reverse the appropriate axis if it moves away from the target.
- A center dead zone, hysteresis, acceleration limiting, and a 25 ms target filter reduce hunting while retaining responsiveness.
- **Predict offscreen** extrapolates the last detected image position using estimated velocity for **1.2 seconds** by default, adjustable **0–3 seconds**. It then decelerates to a hold. An orange indicator identifies a predicted target. Prediction is approximate and does not recover an invisible face indefinitely.
- The nearest face to the previous target is preferred when several faces are visible. This is geometric continuity, not identity recognition.
- Camera failure, stale browser observations, lost servo feedback, and excessive servo following error stop movement.
- **Stop motion** stops immediately and retains torque. **Disengage torque** releases both servos; the mechanism may fall or move under load.

Tracking directions, bounds, speed, and prediction settings are saved on the Pi in `tracking-settings.json`. Sweep setpoints are saved in browser local storage, separately for each dashboard origin. A change of Pi IP or browser can require teaching the limits again. No motion resumes automatically after reboot.

Keep the dashboard visible while tracking. Detection pauses in a hidden browser tab. Video is streamed live; this application does not record it.

## Network access

The dashboard has no authentication or TLS and exposes servo motion controls. Use it only on a trusted local network; do not forward port 8080 to the internet. SSH credentials and machine-specific settings are not included in this repository.

## Code layout

| File | Purpose |
| --- | --- |
| `dashboard.py` | HTTP API, UART access, sweeps, torque control, feedback and tracking loop |
| `camera_feed.py` | Shared Pi/USB camera capture and MJPEG frames |
| `tracking_control.py` | Configuration validation, bounded velocity control and target prediction |
| `index.html` | Dashboard controls and telemetry plots |
| `assets/head_detection.mjs` | Browser face detection and target observations |
| `assets/vision_bundle.mjs`, `assets/wasm/` | Bundled MediaPipe runtime |
| `assets/blaze_face_short_range.tflite` | Bundled face detector model |
| `scripts/install-service.sh` | Optional systemd startup installation |
| `../tests/test_tracking_control.py` | Hardware-free controller and prediction tests |

## Checks

```sh
(cd .. && python3 -m unittest discover -s tests -p 'test_tracking_control.py' -v)
python3 -m py_compile dashboard.py camera_feed.py tracking_control.py
bash -n scripts/install-service.sh
```

Tests do not connect to hardware or command movement. Physical direction, travel clearance, power delivery, and responsiveness must be checked on the assembled robot.

See [THIRD_PARTY.md](THIRD_PARTY.md) for the bundled detector sources and license information.
