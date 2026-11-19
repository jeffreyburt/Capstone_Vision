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
 Controller laptop ──────────────── Master Pi 4 ~~~~~~~~~~~~~~~~~~~~~~~~ Robot 1 Pi  .105
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

| Device | IP | Interface |
|---|---|---|
| Controller laptop | `192.168.1.10` | Ethernet (static, no gateway) + route to 192.168.50.0/24 via `.1.20` |
| Master Pi | `192.168.1.20` | eth0 (laptop side) |
| Master Pi | `192.168.50.1` | wlan0 (access point, robot side) |
| Robot 1 Pi (replacement) | `192.168.50.105` | wlan0, DHCP; username `robot-pi1`, hostname `raspberrypi`; reserve this lease for MAC `b8:27:eb:35:63:17` |
| Robot 2 Pi | `192.168.50.12` | wlan0, gateway `192.168.50.1` |
| Spare / dev devices | `192.168.50.100`–`.199` | DHCP pool from the master Pi |

Code and config use **IP addresses**, not hostnames, because hostname lookup (mDNS) on an isolated
network is unreliable. With the route in place the laptop can reach the master Pi at either address.

## 3. WiFi

| Setting | Value |
|---|---|
| SSID | `capstone-robots` (hidden: no) |
| Security | WPA2-PSK (CCMP). The password is kept outside the repo. |
| Band / channel | 2.4 GHz, channel 1 (moved from 6 on 2026-11-19, off the channel robot 1's fallback hotspot uses). Robot 1 is a Pi 3, which has no 5 GHz radio. |
| Country code | `US` (kernel command line `cfg80211.ieee80211_regdom=US`) |
| NetworkManager profile | `robot-ap` on the master Pi, starts on boot |

## 4. Ports and data flows

| Port | Proto | Server (binds) | Client (connects) | What |
|---|---|---|---|---|
| **5600** | TCP (ZeroMQ PUB) | Master Pi | Laptop | Birds-eye camera frames (JPEG + metadata), ~15 fps 1920×1080 |
| **5601** | TCP (ZeroMQ PUB) | Robot 1 Pi | Laptop | Robot 1 camera frames, ~10 fps 768×432 |
| **5602** | TCP (ZeroMQ PUB) | Robot 2 Pi | Laptop | Robot 2 camera frames, ~10 fps 768×432 |
| **5005** | UDP | Laptop (receiver) | Vision process (on the laptop, `127.0.0.1`) | Vision output: robot + user positions, 10 Hz JSON |
| **5700** | UDP | Each robot Pi (`robot/drive_server.py`) | Laptop (`control.main`) | Robot drive commands: body velocity JSON, 10 Hz; status reply to the sender ([MVP_RUNBOOK.md](MVP_RUNBOOK.md)). The system GUI also sends a read-only `{"type": "status"}` query every 2 s |
| 5701–5799 | — | *reserved* | | Future robot commands (e.g. pan/tilt) |
| 5007 | UDP | Laptop, `127.0.0.1` (system GUI) | `vision.main`, `control.main` | Status heartbeats, 5 Hz JSON, laptop only (`monitor/beacon.py`) |
| 8080 | HTTP | Laptop (`vision.main --gui` / `--stream`); robot Pis (head tracker dashboard) | Browser; system GUI reads the robots' `/api/state` | Vision GUI / debug view; head tracker |
| 8090 | HTTP | Laptop, `127.0.0.1` (`python -m monitor`) | Browser | System GUI: status of every Pi, robot and laptop program |
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

**Known issue (2026-11-19): robot 1's Pi 3 cannot join `capstone-robots`.** Its onboard WiFi (BCM43430/1,
firmware 7.45.98) sends association requests the master Pi 4's access point (BCM4345/6) never answers
(`ASSOC-REJECT status_code=16`, nothing in the AP log), on channel 6 and channel 1 alike. The same Pi 3 joins
an iPhone hotspot fine, a Pi 4 (robot 2) and an iPhone join `capstone-robots` fine, and robot 1's own
hotspot (identical NetworkManager settings) accepts another Pi 3. A reboot and
`brcmfmac feature_disable=0x82000` did not help. Planned fix: a USB WiFi adapter on robot 1.

## Dashboard access without a laptop route (Mac or Windows)

The main camera stream uses the main Pi's **Ethernet address `192.168.1.20:5600`** directly;
`vision.main --gui` supplies its overlays to the monitor's local `/video/birdseye` route.

The current `system.probe_via: vision@192.168.1.20` setting makes the **monitor only** run its TCP,
HTTP and read-only UDP status probes on the main Pi over SSH. Camera streams are forwarded over SSH
and served from the local dashboard. The laptop keeps its Internet WiFi; no administrator route or
network switch is needed for monitoring. Python 3 must be available on the main Pi (already verified).

Set up SSH public-key login to all three Pis using the team's normal process. Keep credentials and private
keys outside the repository. Add these entries to the laptop's `~/.ssh/config`, using the actual private
key filenames on that laptop:

```sshconfig
Match host 192.168.1.20 user vision
    IdentityFile ~/.ssh/MAIN_PI_KEY
    IdentitiesOnly yes
Match host 192.168.50.105 user robot-pi1
    IdentityFile ~/.ssh/ROBOT1_KEY
    IdentitiesOnly yes
    ProxyJump vision@192.168.1.20
Match host 192.168.50.12 user capstone-pi2
    IdentityFile ~/.ssh/ROBOT2_KEY
    IdentitiesOnly yes
    ProxyJump vision@192.168.1.20
Match all
```

Verify `ssh -o BatchMode=yes USER@IP hostname` for each entry, then run `python -m monitor` and open
`http://localhost:8090/`. A laptop that previously connected to another Pi at `192.168.1.20` must verify
the main Pi's host identity before updating its SSH trust settings. This Mac uses the verified
`HostKeyAlias 192.168.50.1` for `vision@192.168.1.20` to preserve both devices' known identities.

Set `system.probe_via: null` to retain direct routed monitoring. `--no-ssh` skips the detailed health
scripts; gateway-mode network checks still require SSH. Gateway reachability is explicitly labeled and
does **not** prove the laptop's UDP driving route works. Vision and control still need the routed network
above; this feature does not forward control commands or synchronize clocks. Old wheel servers may
report an allow-list error because gateway status probes originate at `192.168.50.1`; that warning is
shown rather than hidden. No wheel/servo services need restarting to use this dashboard.

Robot 1's `.105` address is a verified DHCP lease, not a static reservation. If it changes, update
`streams.robot1.host` and `control.robots.1.host` together in `config.yaml`. The former `.11`/`user`
entry refers to the retired Pi, not this replacement.
