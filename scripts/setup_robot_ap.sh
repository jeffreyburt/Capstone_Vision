#!/bin/bash
# Turn the master Pi's WiFi into the access point the robot Pis join (docs/NETWORK.md).
#
#   sudo bash scripts/setup_robot_ap.sh          # set up (asks for the WiFi password)
#   sudo bash scripts/setup_robot_ap.sh --undo   # remove everything this script added
#
# Result: SSID "capstone-robots" (WPA2, 2.4 GHz channel 6, because the robot Pis are Pi 3s), the Pi at
# 192.168.50.1 on wlan0, DHCP for 192.168.50.100-199, and packet forwarding between wlan0 and eth0.
# It is ROUTED, not NATed or bridged:
#   - eth0 (the laptop link, 192.168.1.x) is not touched, so SSH from the laptop keeps working.
#   - The laptop reaches the robots through a route: 192.168.50.0/24 via 192.168.1.20 (added on the laptop).
#   - Robots use 192.168.50.1 as their gateway, so their replies find their way back to the laptop.
# NetworkManager's "shared" mode is used only for its DHCP server; its NAT/firewall rules are switched off
# (firewall-backend=none) because they would block the laptop from opening connections to the robots.
set -euo pipefail

CON=robot-ap
SSID=capstone-robots
CHANNEL=6
ADDR=192.168.50.1/24
DHCP_RANGE=192.168.50.100,192.168.50.199
NM_CONF=/etc/NetworkManager/conf.d/90-robot-ap.conf
SYSCTL_CONF=/etc/sysctl.d/90-robot-ap.conf

if [ "$(id -u)" != 0 ]; then echo "Run with sudo."; exit 1; fi

if [ "${1:-}" = "--undo" ]; then
    nmcli con delete "$CON" 2>/dev/null || true
    rm -f "$NM_CONF" "$SYSCTL_CONF"
    sysctl -qw net.ipv4.ip_forward=0
    systemctl restart NetworkManager
    nmcli radio wifi off
    echo "Removed the robot access point; WiFi is off again."
    exit 0
fi

read -rsp "New WiFi password for $SSID (8-63 characters): " PSK; echo
read -rsp "Repeat it: " PSK2; echo
if [ "$PSK" != "$PSK2" ]; then echo "The passwords differ; nothing was changed."; exit 1; fi
if [ ${#PSK} -lt 8 ] || [ ${#PSK} -gt 63 ] || ! [[ "$PSK" =~ ^[[:print:]]+$ ]]; then
    echo "Use 8-63 printable characters; nothing was changed."; exit 1
fi

echo "1/4 NetworkManager: DHCP for the access point, without its NAT firewall"
printf '[main]\nfirewall-backend=none\n' > "$NM_CONF"
systemctl restart NetworkManager   # existing connections (eth0) are kept across a restart
sleep 3

echo "2/4 Forwarding between eth0 (laptop) and wlan0 (robots)"
echo "net.ipv4.ip_forward=1" > "$SYSCTL_CONF"
sysctl -qw net.ipv4.ip_forward=1

echo "3/4 Access point profile '$CON' (starts on every boot)"
nmcli radio wifi on
sleep 2
nmcli con delete "$CON" >/dev/null 2>&1 || true
nmcli con add type wifi ifname wlan0 con-name "$CON" autoconnect yes ssid "$SSID" \
    802-11-wireless.mode ap 802-11-wireless.band bg 802-11-wireless.channel "$CHANNEL" \
    802-11-wireless.powersave 2 \
    wifi-sec.key-mgmt wpa-psk wifi-sec.proto rsn wifi-sec.pairwise ccmp wifi-sec.group ccmp wifi-sec.pmf 1 \
    ipv4.method shared ipv4.addresses "$ADDR" ipv4.shared-dhcp-range "$DHCP_RANGE" \
    ipv6.method disabled >/dev/null
nmcli con modify "$CON" wifi-sec.psk "$PSK"

echo "4/4 Starting it"
nmcli con up "$CON"
sleep 2

echo
echo "== Check =="
nmcli -f DEVICE,TYPE,STATE,CONNECTION dev
ip -br addr show wlan0
echo "ip_forward = $(cat /proc/sys/net/ipv4/ip_forward)"
if nft list tables 2>/dev/null | grep -q nm-shared; then
    echo "WARNING: NetworkManager added NAT/firewall rules anyway (nft table nm-shared-*);"
    echo "         the laptop may not be able to reach the robots. Check 'nft list ruleset'."
else
    echo "No NetworkManager NAT/firewall rules: good."
fi
echo
echo "Done. Robots: join '$SSID' and use static 192.168.50.11 (robot 1) / .12 (robot 2), gateway 192.168.50.1."
echo "Laptop (admin PowerShell, once): route -p add 192.168.50.0 mask 255.255.255.0 192.168.1.20"
