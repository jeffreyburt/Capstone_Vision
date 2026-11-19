#!/bin/bash
# Can this robot Pi join ANY WiFi network as a client? Join one (e.g. a phone hotspot), report, clean up.
#   sudo bash wifi_client_test.sh "iphone"          # first visible SSID containing this text (case-insensitive)
# It takes the robot's own bench hotspot (capstone-hotspot) down for the test, asks for the network's password,
# prints the result and the wpa_supplicant log, then deletes the test profile and brings the hotspot back.
# Used 2026-11-19 on robot 1 (Pi 3): it joined an iPhone hotspot but not the master Pi's capstone-robots AP.
# iPhone hotspots: turn on "Maximize Compatibility" (Pi 3 is 2.4 GHz only) and keep that screen open.
set -u
[ "$(id -u)" = 0 ] || { echo "Run with sudo."; exit 1; }
MATCH="${1:?text to look for in the SSID}"
TEST=wifi-client-test

cleanup() {
    nmcli con delete "$TEST" >/dev/null 2>&1
    nmcli con up capstone-hotspot >/dev/null 2>&1 && echo "Cleanup: test profile removed, bench hotspot back up."
}
trap cleanup EXIT

nmcli con down capstone-hotspot >/dev/null 2>&1
sleep 3
echo "== Networks this Pi can see:"
nmcli -f SSID,CHAN,SIGNAL,SECURITY dev wifi list --rescan yes
SSID="$(nmcli -t -f SSID dev wifi list | grep -iF "$MATCH" | head -1)"
[ -n "$SSID" ] || { echo "No SSID containing '$MATCH' is visible."; exit 1; }
echo
echo "== Joining '$SSID' (type its password when asked)"
START="$(date '+%Y-%m-%d %H:%M:%S')"
nmcli --ask dev wifi connect "$SSID" ifname wlan0 name "$TEST"
sleep 3
echo
echo "== Result"
nmcli -t -f GENERAL.STATE,GENERAL.CONNECTION dev show wlan0 | paste -sd' '
ip -br -4 addr show wlan0
GW="$(ip route | awk '/default .* wlan0/{print $3; exit}')"
[ -n "$GW" ] && ping -c 2 -W 2 "$GW" | tail -1
echo "== wpa_supplicant during the attempt"
journalctl -u wpa_supplicant --since "$START" --no-pager | grep -E "Trying to associate|ASSOC-REJECT|Associated|CONNECTED|WRONG_KEY" | tail -8
