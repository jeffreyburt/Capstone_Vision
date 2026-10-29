# Robot network setup

This folder builds the network described in [docs/NETWORK.md](../docs/NETWORK.md): one flat subnet
`192.168.50.0/24` with the laptop on Ethernet and the robot Pis on the master Pi's WiFi.

```
Laptop 192.168.50.10 ──Ethernet── master-pi 192.168.50.1 (br0 = eth0 + wlan0 AP "capstone-robots")
                                        ~~WiFi~~ robot1-pi 192.168.50.11
                                        ~~WiFi~~ robot2-pi 192.168.50.12
```

| File | Run on | What it does |
|---|---|---|
| `setup_master.sh` | master Pi | WiFi access point + bridge `br0` (NetworkManager), DHCP `.100`–`.199` with no gateway/DNS (dnsmasq), time server (chrony) |
| `setup_robot.sh 1` / `2` | robot Pis | Join `capstone-robots` at boot with a static IP, sync time to the master Pi |
| `setup_laptop.ps1` | laptop (admin) | Static `192.168.50.10/24` on Ethernet with no gateway, Private profile, firewall rules |
| `check.sh` | any Pi | PASS/FAIL checks for NETWORK.md section 7 (detects master or robot by itself) |
| `check_laptop.ps1` | laptop | PASS/FAIL checks from the laptop: pings, SSH ports, NTP, optional chrony sync via SSH |
| `wifi_mode.sh` | any Pi | Temporarily switch the Pi back to normal WiFi for `apt`/`pip`/`git` |
| `common.sh` | — | Shared addresses and helpers (sourced by the scripts above) |

All Pi scripts are **safe to re-run**, and none of them switches the network right away: they set
everything up, then **reboot** to switch. Run them while the Pi still has internet, because they
install packages (`chrony`, plus `dnsmasq` on the master Pi).

**How it's built:** NetworkManager (`nmcli`) runs the bridge and the WiFi access point, so no
hostapd is needed. DHCP comes from a small standalone `dnsmasq` instead of NetworkManager's
"shared" mode, because shared mode always hands out a gateway and DNS server and also turns on NAT.
NetworkManager profiles are named `capstone-*`, and your existing profiles (home WiFi,
`Wired connection 1`) are kept. Only their autoconnect is switched off.

> **Tested status:** these scripts were syntax-checked and linted (`bash -n`, shellcheck) and
> `check_laptop.ps1` was run on a laptop with no Pis attached. **Nothing has been run on a real Pi yet.**
> The riskiest part is the WiFi AP as a bridge port (step 2). If it fails, see the
> [hostapd fallback](#fallback-hostapd-access-point) below.

---

## 0. Before you start

- Each Pi has done [PI_GUIDE.md](../docs/PI_GUIDE.md) parts 1–5: Raspberry Pi OS Lite 64-bit, SSH on,
  connected to your home WiFi, this repo in `~/Capstone_Vision`. Run `git pull` (or copy again) so the
  `network/` folder is on the Pi.
- **Choose the WiFi password** (8–63 characters). It goes on every Pi and is **never committed**.
  Each script reads it from the first of these it finds:
  1. the `CAPSTONE_WIFI_PSK` environment variable;
  2. the file `network/wifi_password.txt` on that Pi (gitignored; the script offers to create it);
  3. a prompt.
- Order: **laptop → master Pi → robot Pis**. The robots can be set up before the master Pi is on.
  They keep retrying until the access point appears.

## 1. Laptop (Windows 11)

1. Plug the Ethernet cable from the laptop into the master Pi.
2. Open **PowerShell as administrator** (Start → type PowerShell → right-click → Run as administrator),
   `cd` to the Capstone_Vision folder and run:

```powershell
powershell -ExecutionPolicy Bypass -File network\setup_laptop.ps1
```

If the laptop has more than one wired adapter (for example a USB-Ethernet dongle), it lists them.
Re-run it with `-InterfaceAlias "Ethernet 2"` (use the name it shows).

What it does, if you'd rather click through it yourself:
- **Settings → Network & internet → Ethernet → IP assignment → Edit → Manual, IPv4 on**:
  IP `192.168.50.10`, subnet mask `255.255.255.0`, **gateway empty**, DNS empty.
- **Ethernet → Network profile type → Private.**
- Firewall: an inbound allow rule for UDP 5005 and one for ping (ICMPv4 echo), limited to the robot
  subnet `192.168.50.0/24` (plus `127.0.0.1` for UDP 5005). When Windows asks "Allow Python?" the
  first time you run the vision code, tick **Private networks**.

Expected output ends with `Done. Check with: ...`. Run the check once the master Pi is up (step 4).

**Windows and "Unidentified network":** an Ethernet network without a gateway shows as
"Unidentified network", and Windows may set it back to *Public* after a reboot. The firewall rules
work either way. To make it stay Private, open `secpol.msc` → **Network List Manager Policies** →
**Unidentified Networks** → **Location type: Private**.

The laptop's internet keeps working over its WiFi because the Ethernet adapter has no gateway.
**Don't connect the laptop to the `capstone-robots` WiFi**; that uses up the robots' airtime.

## 2. Master Pi

SSH in over your home WiFi (`ssh <user>@master-pi.local`), then:

```bash
cd ~/Capstone_Vision && git pull
```
```bash
sudo bash network/setup_master.sh
```

Expected: sections `== Hostname and WiFi country ==`, `== Packages ==` (installs chrony and dnsmasq),
a password prompt, then `== Bridge br0 ...`, `== eth0 -> br0 ==`, `== wlan0 access point ...`,
`== DHCP ...`, `== Time server ...`, and finally `Setup staged ... Reboot now? [Y/n]`. Press Enter.

After the reboot the master Pi has **left your home WiFi**: wlan0 is now the access point. Reach it
from the laptop over Ethernet:

```bash
ssh <user>@192.168.50.1
```
```bash
sudo bash ~/Capstone_Vision/network/check.sh
```

Before the robots are set up, expect everything to PASS except `ping robot1-pi` / `ping robot2-pi`
(FAIL) and the client count / camera port (WARN). On your phone you should now see a WiFi network
called `capstone-robots`.

**5 GHz problems?** If `capstone-robots` doesn't appear or the robots can't connect, switch to 2.4 GHz
(any of channel 1, 6 or 11, whichever is least crowded where you are):

```bash
sudo bash network/setup_master.sh --band 2.4 --channel 6
```

The robots don't need any change for that.

## 3. Robot Pis

On **robot 1**, over your home WiFi (`ssh <user>@robot1-pi.local`):

```bash
cd ~/Capstone_Vision && git pull
```
```bash
sudo bash network/setup_robot.sh 1
```

On **robot 2**, run the same thing with `2`. Use the same password as on the master Pi.
After the reboot, each robot joins `capstone-robots` (if the master Pi is on) and is reachable only
on the robot network. From the laptop:

```bash
ssh <user>@192.168.50.11
```
```bash
sudo bash ~/Capstone_Vision/network/check.sh
```

## 4. Verify (NETWORK.md section 7)

**On each Pi:** `sudo bash ~/Capstone_Vision/network/check.sh`. A good robot looks like:

```
Network check on robot1-pi - role: robot1 - ...
== WiFi ==
  PASS  connected to capstone-robots
  PASS  signal -48 dBm, 5180 MHz, tx 433.3 MBit/s VHT-MCS 9 80MHz short GI VHT-NSS 1
  PASS  WiFi power saving off
== Reachability ==
  PASS  ping master-pi (192.168.50.1)
  PASS  ping other robot (192.168.50.12)
  PASS  ping laptop (192.168.50.10)
  PASS  master-pi SSH port open
  WARN  robot1 camera stream port 127.0.0.1:5601 closed (fine if 'python -m camstream.sender' isn't running)
== Time sync (chronyc tracking) ==
        Reference ID    : C0A83201 (192.168.50.1)
        Stratum         : 9
        ...
  PASS  synced to master-pi 192.168.50.1
  PASS  leap status Normal
  PASS  clock offset 0.000213 s (< 5 ms)

RESULT: ALL REQUIRED CHECKS PASSED (1 WARN)
```

Right after a boot, chrony needs up to about a minute before it says `synced`. Run it again if so.

**On the laptop** (normal PowerShell, no admin needed):

```powershell
powershell -ExecutionPolicy Bypass -File network\check_laptop.ps1
```

Expected: PASS for the static IP, no gateway, firewall rules, internet route via WiFi, then for each Pi
`ping`, `SSH port 22 open`, and `master-pi answers NTP`. Camera ports are WARN unless the senders run.
It ends with `RESULT: ALL REQUIRED CHECKS PASSED`.

To also test real SSH logins and the robots' chrony status from the laptop, set up an SSH key once
(no password prompts after that), then pass your Pi username:

```powershell
ssh-keygen -t ed25519
```
```powershell
type $env:USERPROFILE\.ssh\id_ed25519.pub | ssh <user>@192.168.50.11 "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys"
```

Repeat that for `.1` and `.12`, then:

```powershell
powershell -ExecutionPolicy Bypass -File network\check_laptop.ps1 -User <user>
```

**Reboot test (section 7.5):** reboot devices in different orders (for example robots first, then the
master Pi; then the master Pi alone) and re-run the checks. Robots retry the WiFi forever, so give
them up to about 1 minute after the master Pi's access point comes up.

**Streams:** finally run the real thing from [PI_GUIDE.md](../docs/PI_GUIDE.md) parts 7 and 13
(`python -m camstream.viewer --stats` on the laptop).

---

## Internet for package installs

The robot network has no internet. To `apt install`, `pip install` or `git pull` on a Pi, switch it
back to your home WiFi for a moment:

```bash
sudo bash ~/Capstone_Vision/network/wifi_mode.sh home
```

Do your installs, then switch back (or just reboot; a reboot always returns to the robot network):

```bash
sudo bash ~/Capstone_Vision/network/wifi_mode.sh robot
```

Notes:
- **Robot Pi:** once it's on home WiFi, the laptop can't reach it at `.11`/`.12`. Your SSH session over
  the robot WiFi drops, so run the commands in one go, or SSH in via `robotN-pi.local` from a machine on
  the home WiFi:
  `sudo bash network/wifi_mode.sh home && sudo apt update && sudo apt install -y <pkg>; sudo bash network/wifi_mode.sh robot`
- **Master Pi:** the laptop stays connected over Ethernet (`192.168.50.1` still works), but the robots
  lose WiFi until you switch back.
- `wifi_mode.sh list` shows saved WiFi profiles. `home "Name"` picks a specific one. The top of
  `wifi_mode.sh` shows how to add a new one (for example a phone hotspot).
- Code updates without internet: copy from the laptop with `scp` (PI_GUIDE part 4, option B) using
  the robot-network IPs.

## Optional: sync the laptop clock to the master Pi

Not needed (NETWORK.md section 6). The vision output uses the master Pi's capture time. Only do this
for absolute latency measurements. It replaces Windows' internet time source until you undo it.
Admin PowerShell:

```powershell
w32tm /config /manualpeerlist:192.168.50.1 /syncfromflags:manual /update; Restart-Service w32time; w32tm /resync
```

Undo: `w32tm /config /manualpeerlist:time.windows.com /syncfromflags:manual /update; w32tm /resync`

---

## Troubleshooting

| Symptom | Look at / fix |
|---|---|
| `capstone-robots` doesn't show up | On the master: `nmcli con show --active` (is `capstone-ap` active?), `journalctl -b -u NetworkManager \| grep -i -e wlan0 -e capstone`, `iw reg get` (should say `country US`). Try 2.4 GHz: `--band 2.4`. |
| Robot sees the SSID but won't connect | Wrong password: re-run `setup_robot.sh` (delete `network/wifi_password.txt` first if it's stale). On the robot: `journalctl -b -u NetworkManager \| grep -i capstone`. If it fails at the 4-way handshake, use the [hostapd fallback](#fallback-hostapd-access-point). |
| Robot connected but can't ping `.1` | On the master: `bridge link` should list both `eth0` and `wlan0` under `br0`. If `wlan0` is missing, NetworkManager couldn't bridge the AP: use the hostapd fallback. |
| Laptop can't ping `.1` | Laptop: `ipconfig` should show `192.168.50.10` on Ethernet. Master: `ip -br addr show br0` should show `192.168.50.1/24`, and `bridge link` should list `eth0`. Check the cable. |
| Robot ping works, laptop ping (`.10`) from a Pi doesn't | Windows firewall: re-run `setup_laptop.ps1` as admin. |
| Laptop lost its internet | Ethernet got a gateway: `check_laptop.ps1` shows FAIL on "default gateway". Re-run `setup_laptop.ps1`. |
| Robot keeps joining the home WiFi | Its home profile still autoconnects: re-run `setup_robot.sh`, or `sudo nmcli con mod "<name>" connection.autoconnect no`. |
| `chronyc tracking` doesn't show `192.168.50.1` | `chronyc -n sources -v` on the robot. `^?` next to `192.168.50.1` means unreachable: check ping, and that chrony runs on the master (`systemctl status chrony`). Wait a minute after boot. |
| Clock offset well above 5 ms | Usually fixes itself within a few minutes. Check the WiFi signal in `check.sh`. |
| Stream slow / `dropped` rising | `check.sh` signal and tx bitrate. Lower robot camera fps/quality in `config.yaml` (NETWORK.md section 5). |
| `$'\r': command not found` | Windows line endings: `sed -i 's/\r$//' network/*.sh` |
| Spare device gets no IP from DHCP | Master: `systemctl status dnsmasq`, `journalctl -u dnsmasq -b`. |

Useful commands on a Pi: `nmcli con show`, `nmcli con show --active`, `ip -br addr`, `bridge link`,
`iw dev wlan0 info`, `iw dev wlan0 link` (robot), `iw dev wlan0 station dump` (master: connected
robots), `chronyc -n sources -v`, `chronyc -n clients` (master).

### Fallback: hostapd access point

Use this only if NetworkManager can't run the AP as a bridge port (robots fail the WPA handshake, or
`wlan0` never joins `br0`). It keeps the bridge, DHCP and chrony from `setup_master.sh` and lets
hostapd run the WiFi instead. On the master Pi, while it has internet
(`wifi_mode.sh home`, or Ethernet internet):

```bash
sudo apt install -y hostapd
```
```bash
sudo nmcli con delete capstone-ap
```
```bash
printf '[keyfile]\nunmanaged-devices=interface-name:wlan0\n' | sudo tee /etc/NetworkManager/conf.d/capstone-wlan0.conf
```

Create `/etc/hostapd/hostapd.conf` with `sudo nano /etc/hostapd/hostapd.conf`, then set
`sudo chmod 600 /etc/hostapd/hostapd.conf`. Use `hw_mode=g` and `channel=6` for 2.4 GHz:

```
interface=wlan0
bridge=br0
driver=nl80211
country_code=US
ieee80211d=1
hw_mode=a
channel=36
ieee80211n=1
ieee80211ac=1
wmm_enabled=1
ssid=capstone-robots
wpa=2
wpa_key_mgmt=WPA-PSK
rsn_pairwise=CCMP
wpa_passphrase=<your password>
```

```bash
sudo systemctl unmask hostapd && sudo systemctl enable hostapd && sudo reboot
```

In this mode `wifi_mode.sh home` doesn't work on the master Pi (NetworkManager no longer manages
wlan0). Use Ethernet internet or undo the fallback. Tell whoever maintains this folder, so the
script can be switched over for good.

### Undo everything on a Pi

```bash
sudo nmcli con delete capstone-ap capstone-br0-eth0 capstone-br0 capstone-robots
```

Then turn your old profiles back on (`nmcli con show` lists them; Imager's WiFi profile is usually
`preconfigured`), restore chrony, and on the master turn DHCP off:

```bash
sudo nmcli con mod preconfigured connection.autoconnect yes
```
```bash
sudo cp /etc/chrony/chrony.conf.orig /etc/chrony/chrony.conf && sudo systemctl restart chrony
```
```bash
sudo systemctl disable --now dnsmasq
```

Some of these print "not found" depending on the Pi; that's fine. Reboot afterwards.

### Undo on the laptop

Admin PowerShell (replace `Ethernet` with your adapter name):

```powershell
Remove-NetIPAddress -InterfaceAlias Ethernet -IPAddress 192.168.50.10 -Confirm:$false; Set-NetIPInterface -InterfaceAlias Ethernet -Dhcp Enabled; Get-NetFirewallRule -Group "Capstone robot network" | Remove-NetFirewallRule
```
