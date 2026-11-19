# Network Contract

**Purpose:** the fixed facts every part of the system relies on: addresses, ports, and what flows where.
The **network setup** (WiFi access point, bridge, DHCP, time sync) is done in a separate stack/session.
That work must produce a network matching this document. The vision and transport code only depend on
what is written here.

If anything changes, update this file first and then `config.yaml` (the `streams:` and `output:` sections).

---

## 1. Topology (as built, 2026-11-19)

```
            Ethernet, 192.168.1.0/24                 WiFi "capstone-robots", 192.168.50.0/24
 Controller laptop ──────────────── Master Pi 4 ~~~~~~~~~~~~~~~~~~~~~~~~ Robot 1 Pi  .11
 192.168.1.10                       eth0 192.168.1.20  ~~~~~~~~~~~~~~~~~~ Robot 2 Pi  .12
 route: 192.168.50.0/24             wlan0 192.168.50.1 (access point)
        via 192.168.1.20            forwards between eth0 and wlan0
```

- **Two subnets, routed by the master Pi** (no bridge, no NAT). The laptop and the robot Pis still reach
  each other directly by IP; the master Pi just forwards the packets. Set up by
  `sudo bash scripts/setup_robot_ap.sh` on the master Pi (`--undo` removes it).
- Why routed instead of the originally planned bridge: the laptop's only link to the master Pi is eth0,
  and re-plumbing eth0 into a bridge risks cutting that link mid-setup. Routing leaves eth0 untouched.
- The laptop needs **one persistent route** (admin PowerShell, once):
  `route -p add 192.168.50.0 mask 255.255.255.0 192.168.1.20`
- The robots use `192.168.50.1` as their **gateway**, so their replies reach the laptop's 192.168.1.10.
- The laptop stays **on Ethernet only**. It must not also join the robot WiFi, because that would use
  airtime the robots need. The laptop's own WiFi stays free for internet.
- No internet is required on the robot network.

## 2. Addresses

| Device | IP (static) | Interface |
|---|---|---|
| Controller laptop | `192.168.1.10` | Ethernet (static, no gateway) + route to 192.168.50.0/24 via `.1.20` |
| Master Pi | `192.168.1.20` | eth0 (laptop side) |
| Master Pi | `192.168.50.1` | wlan0 (access point, robot side) |
| Robot 1 Pi | `192.168.50.11` | wlan0, gateway `192.168.50.1` |
| Robot 2 Pi | `192.168.50.12` | wlan0, gateway `192.168.50.1` |
| Spare / dev devices | `192.168.50.100`–`.199` | DHCP pool from the master Pi |

Code and config use **IP addresses**, not hostnames, because hostname lookup (mDNS) on an isolated
network is unreliable. With the route in place the laptop can reach the master Pi at either address.

## 3. WiFi

| Setting | Value |
|---|---|
| SSID | `capstone-robots` (hidden: no) |
| Security | WPA2-PSK (CCMP). The password is kept outside the repo. |
| Band / channel | 2.4 GHz, channel 6. The robot Pis are Pi 3s, which have no 5 GHz radio. |
| Country code | `US` (kernel command line `cfg80211.ieee80211_regdom=US`) |
| NetworkManager profile | `robot-ap` on the master Pi, starts on boot |

## 4. Ports and data flows

| Port | Proto | Server (binds) | Client (connects) | What |
|---|---|---|---|---|
| **5600** | TCP (ZeroMQ PUB) | Master Pi | Laptop | Birds-eye camera frames (JPEG + metadata), ~15 fps 1920×1080 |
| **5601** | TCP (ZeroMQ PUB) | Robot 1 Pi | Laptop | Robot 1 camera frames, ~10 fps 768×432 |
| **5602** | TCP (ZeroMQ PUB) | Robot 2 Pi | Laptop | Robot 2 camera frames, ~10 fps 768×432 |
| **5005** | UDP | Laptop (receiver) | Vision process (on the laptop, `127.0.0.1`) | Vision output: robot + user positions, 10 Hz JSON |
| **5700** | UDP | Each robot Pi (`robot/drive_server.py`) | Laptop (`control.main`) | Robot drive commands: body velocity JSON, 10 Hz; status reply to the sender ([MVP_RUNBOOK.md](MVP_RUNBOOK.md)) |
| 5701–5799 | — | *reserved* | | Future robot commands (e.g. pan/tilt) |
| 8080 | HTTP | Whichever machine runs `--stream` | Browser | Debug view (development only) |
| 123 | UDP (NTP) | Master Pi (chrony) | Slave Pis, laptop | Time sync |
| 22 | TCP | Every Pi | Laptop | SSH |

**Camera servers bind and the laptop connects.** Each Pi doesn't need to know the laptop's address,
the laptop can connect to any subset of cameras, and several programs on the laptop can watch the same
camera (for example the robot-camera processing and a viewer).

Firewall: the laptop must allow inbound UDP 5005 on the Ethernet network (Windows: allow Python on
**Private** networks, and mark the Ethernet network as Private).

## 5. Bandwidth budget

| Stream | Link | Typical rate |
|---|---|---|
| Birds-eye, 1920×1080 JPEG q85 @ 15 fps | Ethernet only | ~25–40 Mbps (gigabit link: no problem) |
| Each robot camera, 768×432 JPEG q80 @ 10 fps | WiFi | ~2–4 Mbps |
| Robot commands + vision output | WiFi / local | < 0.1 Mbps |

Total WiFi load is about **5–10 Mbps**. That fits comfortably on the Pi 4's access point at room range.
Measure it on the real network with `python -m camstream.viewer --stats` on the laptop.
If WiFi struggles, lower the robot cameras' `fps`, `quality` or resolution in `config.yaml` before
changing anything else.

## 6. Time

- The Pis have **no battery-backed clock**, and the network has no internet, so time must be synced locally.
- **The master Pi is the time server** for the subnet (chrony with `local stratum 8`, `allow 192.168.50.0/24`).
- **Slave Pis must sync to `192.168.50.1`.** This matters because robot-camera timestamps get compared with
  birds-eye timestamps.
- Laptop: syncing is optional. It is only needed for absolute latency measurements; everything else uses
  the master Pi's capture times.
- Target: all Pis within about 5 ms of each other.

## 7. Network checklist

1. Done 2026-11-19: the master Pi runs the AP from §3 on wlan0 (`192.168.50.1/24`), serves DHCP for
   `.100`–`.199`, and forwards between eth0 and wlan0. The laptop never touches its DHCP or DNS (the
   laptop isn't on the robot WiFi), so the laptop's internet is unaffected.
2. Laptop: add the route from §1.
3. The robot Pis join `capstone-robots` automatically on boot, with the static IPs and gateway from §2.
   (On a Pi 3 the single radio can't also run its bench hotspot; give the client profile the higher
   NetworkManager `autoconnect-priority` so the hotspot only comes up when `capstone-robots` is out of range.)
4. Not done: chrony as in §6. The master Pi's clock currently drifts (it was 33 days behind on 2026-11-19).
   The MVP doesn't depend on it.
4. Verify:
   - from the laptop, `ping 192.168.50.11` and `ping 192.168.50.12` work;
   - `ssh` works to all three Pis;
   - `chronyc tracking` on each slave shows it synced to `192.168.50.1`.
5. Survives reboot of any device, in any order.
