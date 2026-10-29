# Shared helpers for the network scripts (sourced, not run directly). See network/README.md.
# Values here come from docs/NETWORK.md; change that file first if any of them change.

SUBNET=192.168.50.0/24
MASTER_IP=192.168.50.1
LAPTOP_IP=192.168.50.10
ROBOT1_IP=192.168.50.11
ROBOT2_IP=192.168.50.12
DHCP_FIRST=192.168.50.100
DHCP_LAST=192.168.50.199
SSID=capstone-robots
COUNTRY=US

# NetworkManager profile names created by these scripts
CON_BRIDGE=capstone-br0
CON_ETH=capstone-br0-eth0
CON_AP=capstone-ap
CON_ROBOT=capstone-robots

NETWORK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PSK_FILE="$NETWORK_DIR/wifi_password.txt"   # gitignored

say()  { echo; echo "== $* =="; }
die()  { echo "ERROR: $*" >&2; exit 1; }

# Re-run the calling script as root, keeping the password env var if it was set.
need_root() {
    if [ "$EUID" -ne 0 ]; then
        exec sudo --preserve-env=CAPSTONE_WIFI_PSK bash "$0" "$@"
    fi
}

# Sets WIFI_PSK from $CAPSTONE_WIFI_PSK, then network/wifi_password.txt, then a prompt.
read_psk() {
    if [ -n "${CAPSTONE_WIFI_PSK:-}" ]; then
        WIFI_PSK="$CAPSTONE_WIFI_PSK"
    elif [ -s "$PSK_FILE" ]; then
        WIFI_PSK="$(head -n1 "$PSK_FILE" | tr -d '\r\n')"
        echo "Using WiFi password from $PSK_FILE"
    else
        read -r -s -p "WiFi password for '$SSID' (8-63 chars, same on every Pi): " WIFI_PSK; echo
        read -r -s -p "Repeat: " again; echo
        [ "$WIFI_PSK" = "$again" ] || die "passwords don't match"
        read -r -p "Save it to $PSK_FILE (gitignored) so re-runs don't ask? [y/N] " ans
        if [[ "$ans" =~ ^[Yy] ]]; then
            (umask 077; printf '%s\n' "$WIFI_PSK" > "$PSK_FILE")
        fi
    fi
    [ "${#WIFI_PSK}" -ge 8 ] && [ "${#WIFI_PSK}" -le 63 ] || die "WPA2 password must be 8-63 characters"
}

# apt-install only the packages that are missing, so re-runs work without internet.
install_pkgs() {
    local missing=()
    for p in "$@"; do dpkg -s "$p" >/dev/null 2>&1 || missing+=("$p"); done
    if [ "${#missing[@]}" -gt 0 ]; then
        echo "Installing: ${missing[*]} (needs internet)"
        apt-get update && apt-get install -y "${missing[@]}" \
            || die "apt failed. Is the Pi on a WiFi with internet? See network/README.md -> 'Internet for package installs'"
    else
        echo "Already installed: $*"
    fi
}

con_exists() { nmcli -g NAME connection show | grep -qxF "$1"; }

# Stop every other profile of the given type ("802-11-wireless" or "802-3-ethernet") from
# autoconnecting, so the capstone profiles always win at boot. The profiles are kept, so
# `nmcli con up <name>` (or network/wifi_mode.sh home) still works for package installs.
disable_other_autoconnect() {
    local type="$1"; shift
    local name t
    while IFS=: read -r name t; do
        [ "$t" = "$type" ] || continue
        case " $* " in *" $name "*) continue ;; esac
        if [ "$(nmcli -g connection.autoconnect con show "$name")" != "no" ]; then
            echo "Disabling autoconnect on '$name' (kept; use it again with: sudo bash network/wifi_mode.sh home)"
            nmcli con mod "$name" connection.autoconnect no
        fi
    done < <(nmcli -t -f NAME,TYPE connection show)
}

# WiFi needs a country before it can transmit (and for 5 GHz channels).
set_country() {
    if command -v raspi-config >/dev/null; then
        raspi-config nonint do_wifi_country "$COUNTRY" || iw reg set "$COUNTRY" || true
    else
        iw reg set "$COUNTRY" || true
    fi
    rfkill unblock wifi || true
}

set_hostname() {
    local name="$1"
    if [ "$(hostname)" != "$name" ]; then
        echo "Setting hostname to $name"
        hostnamectl set-hostname "$name"
    fi
    if grep -q '^127\.0\.1\.1' /etc/hosts; then
        sed -i "s/^127\.0\.1\.1.*/127.0.1.1\t$name/" /etc/hosts
    else
        printf '127.0.1.1\t%s\n' "$name" >> /etc/hosts
    fi
}

# Back up the distro chrony.conf once, then replace it with ours ($1 = full file contents).
write_chrony_conf() {
    [ -f /etc/chrony/chrony.conf.orig ] || cp /etc/chrony/chrony.conf /etc/chrony/chrony.conf.orig
    printf '%s\n' "$1" > /etc/chrony/chrony.conf
    systemctl enable chrony >/dev/null 2>&1 || true
    systemctl restart chrony
}

finish_with_reboot() {
    local yes="$1"
    echo
    echo "Setup staged. Nothing has switched yet: the new network takes over on the next boot."
    echo "After the reboot, check it with:  sudo bash network/check.sh"
    if [ "$yes" = 1 ]; then
        ans=y
    else
        read -r -p "Reboot now? [Y/n] " ans
    fi
    if [[ ! "$ans" =~ ^[Nn] ]]; then
        echo "Rebooting..."
        systemctl reboot
    else
        echo "Not rebooting. Run 'sudo reboot' when ready."
    fi
}
