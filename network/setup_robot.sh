#!/usr/bin/env bash
# Robot Pi network setup (docs/NETWORK.md): join the master Pi's WiFi 'capstone-robots' at boot
# with a static IP, and sync time to the master Pi with chrony.
#
#   sudo bash network/setup_robot.sh 1      # robot1-pi, 192.168.50.11
#   sudo bash network/setup_robot.sh 2      # robot2-pi, 192.168.50.12
#   add --yes to reboot at the end without asking
#
# Safe to re-run. Run it while the Pi still has internet (home WiFi), because it installs chrony.
# Nothing switches until the reboot at the end.
set -euo pipefail
source "$(dirname "$0")/common.sh"
need_root "$@"

N=""; YES=0
for a in "$@"; do
    case "$a" in
        1|2) N="$a" ;;
        --yes) YES=1 ;;
        *) die "usage: sudo bash network/setup_robot.sh <1|2> [--yes]" ;;
    esac
done
[ -n "$N" ] || die "usage: sudo bash network/setup_robot.sh <1|2> [--yes]"
if [ "$N" = 1 ]; then MY_IP="$ROBOT1_IP"; else MY_IP="$ROBOT2_IP"; fi

systemctl is-active --quiet NetworkManager || die "NetworkManager is not running (needs Raspberry Pi OS Bookworm or newer)"

say "Hostname and WiFi country"
set_hostname "robot$N-pi"
set_country

say "Packages"
install_pkgs chrony iw

read_psk

say "WiFi '$SSID' on wlan0, static $MY_IP/24, no gateway"
con_exists "$CON_ROBOT" || nmcli con add type wifi ifname wlan0 con-name "$CON_ROBOT" ssid "$SSID"
# powersave 2 = off: WiFi power saving adds 100+ ms latency spikes to the camera stream.
# autoconnect-retries 0 = keep retrying forever, so the robot joins whenever the master Pi comes up.
nmcli con mod "$CON_ROBOT" \
    802-11-wireless.ssid "$SSID" 802-11-wireless.mode infrastructure 802-11-wireless.powersave 2 \
    802-11-wireless-security.key-mgmt wpa-psk 802-11-wireless-security.proto rsn \
    802-11-wireless-security.pmf disable 802-11-wireless-security.psk "$WIFI_PSK" \
    ipv4.method manual ipv4.addresses "$MY_IP/24" ipv4.gateway "" ipv4.dns "" ipv4.never-default yes \
    ipv6.method disabled \
    connection.autoconnect yes connection.autoconnect-priority 100 connection.autoconnect-retries 0

say "Other WiFi profiles: autoconnect off (kept for internet access later)"
disable_other_autoconnect 802-11-wireless "$CON_ROBOT"

say "Time (chrony): follow the master Pi $MASTER_IP"
write_chrony_conf "# Managed by network/setup_robot.sh (docs/NETWORK.md section 6). Original: chrony.conf.orig
server $MASTER_IP iburst prefer minpoll 4 maxpoll 6
# Internet servers: only reachable when the Pi is temporarily on home WiFi; harmless otherwise.
pool 2.debian.pool.ntp.org iburst
# Step the clock whenever it is >0.1 s off (no RTC, and the master's clock may have jumped).
makestep 0.1 -1
driftfile /var/lib/chrony/chrony.drift
logdir /var/log/chrony"

finish_with_reboot "$YES"
