#!/usr/bin/env bash
# Temporarily switch a Pi's WiFi between the robot network and a normal WiFi with internet
# (for apt / pip / git). Works on the master Pi and the robot Pis.
#
#   sudo bash network/wifi_mode.sh home            # join your normal WiFi (first saved non-capstone profile)
#   sudo bash network/wifi_mode.sh home "MyWiFi"   # ...a specific saved profile
#   sudo bash network/wifi_mode.sh robot           # back to the robot network
#   sudo bash network/wifi_mode.sh list            # show saved WiFi profiles
#
# A reboot always returns to the robot network (only the capstone profiles autoconnect).
# Master Pi: in home mode the robots lose WiFi, but the laptop still reaches 192.168.50.1 over Ethernet.
# To add a new home WiFi profile:  sudo nmcli con add type wifi ifname wlan0 con-name home ssid "<SSID>" \
#     wifi-sec.key-mgmt wpa-psk wifi-sec.psk "<password>" connection.autoconnect no
set -euo pipefail
source "$(dirname "$0")/common.sh"
need_root "$@"

if con_exists "$CON_AP"; then CAPSTONE="$CON_AP"; else CAPSTONE="$CON_ROBOT"; fi
con_exists "$CAPSTONE" || die "no capstone WiFi profile found. Run setup_master.sh or setup_robot.sh first."

wifi_profiles() {
    nmcli -t -f NAME,TYPE con show | while IFS=: read -r name t; do
        [ "$t" = 802-11-wireless ] && [ "$name" != "$CON_AP" ] && [ "$name" != "$CON_ROBOT" ] && echo "$name"
    done
    return 0
}

case "${1:-}" in
    home)
        HOME_CON="${2:-$(wifi_profiles | head -n1)}"
        [ -n "$HOME_CON" ] || die "no saved home WiFi profile. Add one (see the top of this script)."
        echo "Switching wlan0 to '$HOME_CON'. If you are connected over the robot WiFi, this SSH session will drop."
        nmcli con up "$HOME_CON"
        echo "Done. Test with: ping -c2 deb.debian.org   (go back with: sudo bash network/wifi_mode.sh robot)"
        ;;
    robot)
        echo "Switching wlan0 back to '$CAPSTONE'."
        nmcli con up "$CAPSTONE"
        echo "Done. Check with: sudo bash network/check.sh"
        ;;
    list)
        echo "Saved home WiFi profiles:"; wifi_profiles | sed 's/^/  /'
        echo "Robot network profile: $CAPSTONE"
        ;;
    *)
        die "usage: sudo bash network/wifi_mode.sh home [profile] | robot | list"
        ;;
esac
