#!/bin/bash
# Move the master Pi's robot access point (NetworkManager profile robot-ap) to another 2.4 GHz channel.
#   sudo bash scripts/set_robot_ap_channel.sh 1      # 1, 6 or 11
# The AP restarts: connected robots drop for a few seconds and rejoin on their own (they scan all channels).
# Moved 6 -> 1 on 2026-11-19, off the channel robot 1's fallback hotspot uses (docs/NETWORK.md).
set -euo pipefail
[ "$(id -u)" = 0 ] || { echo "Run with sudo."; exit 1; }
CH="${1:?channel (1, 6 or 11)}"
echo "robot-ap: channel $(nmcli -g 802-11-wireless.channel con show robot-ap) -> $CH"
nmcli con modify robot-ap 802-11-wireless.channel "$CH"
nmcli con up robot-ap
sleep 2
nmcli -t -f GENERAL.STATE,GENERAL.CONNECTION dev show wlan0 | paste -sd' '
