# Network Contract

**Purpose:** the fixed facts every part of the system relies on: addresses, ports, and what flows where.
The **network setup** (WiFi access point, bridge, DHCP, time sync) is done in a separate stack/session.
That work must produce a network matching this document. The vision and transport code only depend on
what is written here.

If anything changes, update this file first and then `config.yaml` (the `streams:` and `output:` sections).

---

## 1. Topology

```
                Ethernet (gigabit)                      WiFi (access point on the master Pi)
 Controller laptop ─────────────── Master Pi 4 ~~~~~~~~~~~~~~~~~~~~~~~~ Robot 1 Pi (slave)
 (planning, vision processing,     (birds-eye camera,     ~~~~~~~~~~~~~~ Robot 2 Pi (slave)
  person detection)                 WiFi AP, bridge eth0<->wlan0)
```

- **One flat subnet.** The master Pi **bridges** `eth0` and `wlan0`, so the laptop and the slave Pis reach
  each other directly. Nothing at the application level relays traffic through the master Pi.
- The laptop stays **on Ethernet only**. It must not also join the robot WiFi, because that would use
  airtime the robots need. The laptop's own WiFi stays free for internet.
- No internet is required on the robot network.

## 2. Addresses

| Device | Hostname | IP (static) | Interface |
|---|---|---|---|
| Master Pi (bridge + AP) | `master-pi` | `192.168.50.1` | `br0` (eth0 + wlan0) |
| Controller laptop | — | `192.168.50.10` | Ethernet (static, no gateway) |
| Robot 1 Pi | `robot1-pi` | `192.168.50.11` | wlan0 |
| Robot 2 Pi | `robot2-pi` | `192.168.50.12` | wlan0 |
| Spare / dev devices | — | `192.168.50.100`–`.199` | DHCP pool from the master Pi |

Code and config use **IP addresses**, not hostnames, because hostname lookup (mDNS) on an isolated
network is unreliable.

## 3. WiFi

| Setting | Value |
|---|---|
| SSID | `capstone-robots` (hidden: no) |
| Security | WPA2-PSK. The password is kept outside the repo. |
| Band / channel | 5 GHz, channel 36 (fallback: 2.4 GHz, channel 1, 6 or 11, whichever is least crowded) |
| Country code | `US` (required for 5 GHz) |

## 4. Ports and data flows

| Port | Proto | Server (binds) | Client (connects) | What |
|---|---|---|---|---|
| **5600** | TCP (ZeroMQ PUB) | Master Pi | Laptop | Birds-eye camera frames (JPEG + metadata), ~15 fps 1920×1080 |
| **5601** | TCP (ZeroMQ PUB) | Robot 1 Pi | Laptop | Robot 1 camera frames, ~10 fps 768×432 |
| **5602** | TCP (ZeroMQ PUB) | Robot 2 Pi | Laptop | Robot 2 camera frames, ~10 fps 768×432 |
| **5005** | UDP | Laptop (receiver) | Vision process (on the laptop, `127.0.0.1`) | Vision output: robot + user positions, 10 Hz JSON |
| 5700–5799 | — | *reserved* | | Robot commands (controls team; protocol is theirs to define) |
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

## 7. Requirements for the network-setup session

1. The master Pi runs the AP from §3 and bridges eth0 and wlan0 into `br0` with `192.168.50.1/24`. It
   serves DHCP for `.100`–`.199` only, and must **not** hand out a default gateway or DNS that breaks the
   laptop's internet over its own WiFi.
2. The slave Pis connect to the AP automatically on boot, with the static IPs from §2.
3. chrony is set up as in §6.
4. Verify:
   - from the laptop, `ping 192.168.50.11` and `ping 192.168.50.12` work;
   - `ssh` works to all three Pis;
   - `chronyc tracking` on each slave shows it synced to `192.168.50.1`.
5. Survives reboot of any device, in any order.
