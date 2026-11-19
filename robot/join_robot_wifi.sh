#!/bin/bash
# Join a robot Pi to the master Pi's robot WiFi (docs/NETWORK.md). Asks for the WiFi password.
#
#   sudo bash join_robot_wifi.sh <robot id>          # 1 -> 192.168.50.11, 2 -> 192.168.50.12
#   sudo bash join_robot_wifi.sh --undo              # remove the profile again
#
# Adds a NetworkManager profile "capstone-robots" (static address, gateway 192.168.50.1 so replies reach the
# laptop on 192.168.1.0/24) that connects on boot. Wired Ethernet settings are left alone.
set -euo pipefail
CON=capstone-robots
if [ "$(id -u)" != 0 ]; then echo "Run with sudo."; exit 1; fi
if [ "${1:-}" = "--undo" ]; then nmcli con delete "$CON"; exit 0; fi
case "${1:-}" in 1|2) IP="192.168.50.1${1}";; *) echo "usage: sudo bash $0 <robot id 1|2>"; exit 1;; esac

read -rsp "capstone-robots WiFi password: " PSK; echo
if [ ${#PSK} -lt 8 ] || [ ${#PSK} -gt 63 ]; then echo "8-63 characters; nothing was changed."; exit 1; fi

nmcli radio wifi on
nmcli con delete "$CON" >/dev/null 2>&1 || true
nmcli con add type wifi ifname wlan0 con-name "$CON" ssid "$CON" \
    wifi-sec.key-mgmt wpa-psk 802-11-wireless.powersave 2 \
    ipv4.method manual ipv4.addresses "$IP/24" ipv4.gateway 192.168.50.1 ipv4.route-metric 600 \
    ipv6.method disabled connection.autoconnect yes connection.autoconnect-priority 20 >/dev/null
nmcli con modify "$CON" wifi-sec.psk "$PSK"
nmcli con up "$CON"
sleep 2
ip -br addr show wlan0
if ping -c 2 -W 2 192.168.50.1 >/dev/null; then
    echo "OK: reached the master Pi (192.168.50.1) over capstone-robots as $IP"
else
    echo "WARNING: 192.168.50.1 did not answer. Is the master Pi powered? Wrong password?"
fi
