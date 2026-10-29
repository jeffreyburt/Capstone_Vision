#!/usr/bin/env bash
# Master Pi network setup (docs/NETWORK.md): WiFi access point + bridge br0 (eth0 + wlan0) at
# 192.168.50.1, DHCP for .100-.199 with no gateway/DNS, and the chrony time server.
#
#   sudo bash network/setup_master.sh                 # 5 GHz, channel 36
#   sudo bash network/setup_master.sh --band 2.4      # fallback: 2.4 GHz, channel 6
#   sudo bash network/setup_master.sh --band 2.4 --channel 11
#   add --yes to reboot at the end without asking
#
# Safe to re-run. Run it while the Pi still has internet (home WiFi), because it installs
# chrony and dnsmasq. Nothing switches until the reboot at the end.
set -euo pipefail
source "$(dirname "$0")/common.sh"
need_root "$@"

BAND=5; CHANNEL=""; YES=0
while [ $# -gt 0 ]; do
    case "$1" in
        --band) BAND="$2"; shift 2 ;;
        --channel) CHANNEL="$2"; shift 2 ;;
        --yes) YES=1; shift ;;
        *) die "unknown option $1 (see the top of this script)" ;;
    esac
done
case "$BAND" in
    5)   NM_BAND="a";  CHANNEL="${CHANNEL:-36}" ;;
    2.4) NM_BAND="bg"; CHANNEL="${CHANNEL:-6}" ;;
    *)   die "--band must be 5 or 2.4" ;;
esac

systemctl is-active --quiet NetworkManager || die "NetworkManager is not running (needs Raspberry Pi OS Bookworm or newer)"

say "Hostname and WiFi country"
set_hostname master-pi
set_country

say "Packages"
install_pkgs chrony dnsmasq iw

read_psk

say "Bridge br0 = $MASTER_IP/24, no gateway"
con_exists "$CON_BRIDGE" || nmcli con add type bridge ifname br0 con-name "$CON_BRIDGE"
nmcli con mod "$CON_BRIDGE" \
    bridge.stp no \
    ipv4.method manual ipv4.addresses "$MASTER_IP/24" ipv4.gateway "" ipv4.dns "" ipv4.never-default yes \
    ipv6.method disabled \
    connection.autoconnect yes connection.autoconnect-slaves yes

say "eth0 -> br0"
con_exists "$CON_ETH" || nmcli con add type ethernet ifname eth0 con-name "$CON_ETH" master br0 slave-type bridge
nmcli con mod "$CON_ETH" connection.autoconnect yes

say "wlan0 access point '$SSID' (band $BAND GHz, channel $CHANNEL) -> br0"
con_exists "$CON_AP" || nmcli con add type wifi ifname wlan0 con-name "$CON_AP" ssid "$SSID" master br0 slave-type bridge
# pmf disable: the Pi 4's WiFi chip doesn't handle protected management frames in AP mode.
nmcli con mod "$CON_AP" \
    802-11-wireless.ssid "$SSID" 802-11-wireless.mode ap 802-11-wireless.hidden no \
    802-11-wireless.band "$NM_BAND" 802-11-wireless.channel "$CHANNEL" 802-11-wireless.powersave 2 \
    802-11-wireless-security.key-mgmt wpa-psk 802-11-wireless-security.proto rsn \
    802-11-wireless-security.pairwise ccmp 802-11-wireless-security.group ccmp \
    802-11-wireless-security.pmf disable 802-11-wireless-security.psk "$WIFI_PSK" \
    connection.autoconnect yes connection.autoconnect-priority 100 connection.autoconnect-retries 0

say "Other network profiles: autoconnect off (kept for internet access later)"
disable_other_autoconnect 802-3-ethernet "$CON_ETH"
disable_other_autoconnect 802-11-wireless "$CON_AP"

say "DHCP (dnsmasq): $DHCP_FIRST-$DHCP_LAST on br0, no gateway, no DNS"
mkdir -p /etc/dnsmasq.d
cat > /etc/dnsmasq.d/capstone.conf <<EOF
# Managed by network/setup_master.sh (docs/NETWORK.md section 7.1)
# DHCP only for spare/dev devices. The laptop and robot Pis use static IPs.
port=0                          # no DNS server
interface=br0
bind-dynamic                    # br0 may appear after dnsmasq starts
dhcp-range=$DHCP_FIRST,$DHCP_LAST,255.255.255.0,12h
dhcp-option=option:router       # empty = send no default gateway (keeps the laptop's internet working)
dhcp-option=option:dns-server   # empty = send no DNS server
dhcp-authoritative
EOF
mkdir -p /etc/systemd/system/dnsmasq.service.d
cat > /etc/systemd/system/dnsmasq.service.d/capstone.conf <<'EOF'
[Unit]
After=NetworkManager.service network-online.target
Wants=network-online.target

[Service]
Restart=on-failure
RestartSec=5
EOF
systemctl daemon-reload
systemctl enable dnsmasq >/dev/null 2>&1
systemctl restart dnsmasq || echo "(dnsmasq restart failed now; it is retried after the reboot)"

say "Time server (chrony): local stratum 8, allow $SUBNET"
write_chrony_conf "# Managed by network/setup_master.sh (docs/NETWORK.md section 6). Original: chrony.conf.orig
# Internet servers: only reachable when the Pi is temporarily on home WiFi; harmless otherwise.
pool 2.debian.pool.ntp.org iburst
# No internet on the robot network: serve our own clock to the subnet.
local stratum 8
allow $SUBNET
makestep 1 -1
driftfile /var/lib/chrony/chrony.drift
logdir /var/log/chrony"

finish_with_reboot "$YES"
