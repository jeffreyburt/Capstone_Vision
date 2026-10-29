#!/usr/bin/env bash
# Check the robot network on any Pi (docs/NETWORK.md section 7). Finds out by itself whether it is
# the master Pi or a robot Pi.
#
#   sudo bash network/check.sh
#
# Prints PASS / WARN / FAIL per check. Exit code = number of FAILs (0 = all good).
# WARN means "probably fine, but look at it" (e.g. the laptop doesn't answer ping, or no sender running).
set -uo pipefail
source "$(dirname "$0")/common.sh"
need_root "$@"

FAILS=0; WARNS=0
pass() { printf '  \e[32mPASS\e[0m  %s\n' "$*"; }
warn() { printf '  \e[33mWARN\e[0m  %s\n' "$*"; WARNS=$((WARNS + 1)); }
fail() { printf '  \e[31mFAIL\e[0m  %s\n' "$*"; FAILS=$((FAILS + 1)); }
check() {  # check "description" command...   -> PASS if the command succeeds, else FAIL
    local d="$1"; shift
    if "$@" >/dev/null 2>&1; then pass "$d"; else fail "$d"; fi
}
can_ping() { ping -c 2 -W 1 -q "$1" >/dev/null 2>&1; }
tcp_open() { timeout 2 bash -c "</dev/tcp/$1/$2" 2>/dev/null; }
has_ip()   { ip -4 -o addr show dev "$1" 2>/dev/null | grep -q " $2/24 "; }

ping_check() {  # ping_check ip name required(1|0)
    if can_ping "$1"; then pass "ping $2 ($1)"
    elif [ "$3" = 1 ]; then fail "ping $2 ($1)"
    else warn "ping $2 ($1) - no answer (Windows blocks ping unless allowed; see network/README.md)"
    fi
}

camera_port() {  # camera_port ip port name
    if tcp_open "$1" "$2"; then pass "$3 camera stream port $1:$2 open"
    else warn "$3 camera stream port $1:$2 closed (fine if 'python -m camstream.sender' isn't running)"
    fi
}

if has_ip br0 "$MASTER_IP"; then ROLE=master
elif has_ip wlan0 "$ROBOT1_IP"; then ROLE=robot1; OTHER_IP="$ROBOT2_IP"
elif has_ip wlan0 "$ROBOT2_IP"; then ROLE=robot2; OTHER_IP="$ROBOT1_IP"
else ROLE=unknown
fi

echo "Network check on $(hostname) - role: $ROLE - $(date '+%F %T')"

say "Basics"
check "NetworkManager running" systemctl is-active --quiet NetworkManager
check "chrony running" systemctl is-active --quiet chrony
check "SSH server running" systemctl is-active --quiet ssh
if [ "$ROLE" = unknown ]; then
    fail "this Pi has none of the expected addresses ($MASTER_IP on br0, $ROBOT1_IP / $ROBOT2_IP on wlan0)"
    echo; echo "Current addresses:"; ip -4 -br addr
    echo "Active connections:"; nmcli -t -f NAME,DEVICE con show --active
    echo "Hints: did setup_master.sh / setup_robot.sh run and was the Pi rebooted? Is the master Pi up?"
    echo "       Is the Pi in home-WiFi mode? (sudo bash network/wifi_mode.sh robot)"
    echo; echo "RESULT: $FAILS FAIL, $WARNS WARN"; exit "$FAILS"
fi
pass "address for $ROLE is set"
case "$ROLE" in master) want=master-pi ;; *) want="$ROLE-pi" ;; esac
if [ "$(hostname)" = "$want" ]; then pass "hostname is $want"; else warn "hostname is $(hostname), expected $want"; fi
if ip route show default | grep -q .; then
    warn "a default route exists: $(ip route show default | head -n1) (fine if that's home WiFi/Ethernet; it must not be via 192.168.50.x)"
fi

if [ "$ROLE" = master ]; then
    say "Bridge and access point"
    check "eth0 is a port of br0" bash -c "ip -o link show master br0 | grep -q ' eth0'"
    check "wlan0 is a port of br0" bash -c "ip -o link show master br0 | grep -q ' wlan0'"
    if ip route show default | grep -q 'dev br0'; then fail "br0 has a default gateway (it must not)"; else pass "no default gateway on br0"; fi
    info="$(iw dev wlan0 info 2>/dev/null)"
    if echo "$info" | grep -q 'type AP'; then pass "wlan0 is in AP mode"; else fail "wlan0 is not in AP mode"; fi
    if echo "$info" | grep -q "ssid $SSID"; then pass "SSID is $SSID"; else fail "SSID is not $SSID"; fi
    chan="$(echo "$info" | sed -n 's/.*channel \([0-9]*\) (\([0-9]*\) MHz).*/channel \1 (\2 MHz)/p')"
    if [ -n "$chan" ]; then pass "WiFi $chan"; else warn "couldn't read WiFi channel"; fi
    reg="$(iw reg get 2>/dev/null | grep country | tr '\n' ' ')"
    if echo "$reg" | grep -q "country $COUNTRY"; then pass "regulatory domain $COUNTRY"; else warn "regulatory domain: ${reg:-unknown} (expected $COUNTRY)"; fi
    stations="$(iw dev wlan0 station dump 2>/dev/null | grep -c '^Station')"
    if [ "$stations" -ge 2 ]; then pass "$stations WiFi client(s) connected"; else warn "$stations WiFi client(s) connected (expected 2 robots)"; fi

    say "DHCP"
    check "dnsmasq running" systemctl is-active --quiet dnsmasq
    check "DHCP server listening on UDP 67" bash -c "ss -ulpn | grep -q 'dnsmasq' && ss -ulpn | grep -q ':67 '"
    if grep -qs '^dhcp-option=option:router$' /etc/dnsmasq.d/capstone.conf; then pass "DHCP sends no gateway"; else fail "DHCP config missing 'no gateway' line"; fi

    say "Time server"
    check "chrony serves the subnet (allow $SUBNET)" grep -q "^allow $SUBNET" /etc/chrony/chrony.conf
    check "chrony listening on UDP 123" bash -c "ss -ulpn | grep chronyd | grep -q ':123 '"
    echo "  chrony clients seen (robots should be listed):"
    chronyc -n clients 2>/dev/null | sed 's/^/        /'

    say "Reachability"
    ping_check "$ROBOT1_IP" robot1-pi 1
    ping_check "$ROBOT2_IP" robot2-pi 1
    ping_check "$LAPTOP_IP" laptop 0
    camera_port 127.0.0.1 5600 birdseye
else
    say "WiFi"
    link="$(iw dev wlan0 link 2>/dev/null)"
    if echo "$link" | grep -q "SSID: $SSID"; then pass "connected to $SSID"; else fail "not connected to $SSID"; fi
    sig="$(echo "$link" | sed -n 's/.*signal: \(-\?[0-9]*\) dBm.*/\1/p')"
    freq="$(echo "$link" | sed -n 's/.*freq: \([0-9.]*\).*/\1/p')"
    rate="$(echo "$link" | sed -n 's/.*tx bitrate: \(.*\)/\1/p')"
    if [ -n "$sig" ]; then
        if [ "$sig" -ge -70 ]; then pass "signal $sig dBm, $freq MHz, tx $rate"
        else warn "weak signal $sig dBm, $freq MHz, tx $rate (move closer / check the antenna side of the Pi)"; fi
    fi
    if iw dev wlan0 get power_save 2>/dev/null | grep -q 'off'; then pass "WiFi power saving off"
    else warn "WiFi power saving is on (adds latency). Re-run setup_robot.sh"; fi

    say "Reachability"
    ping_check "$MASTER_IP" master-pi 1
    ping_check "$OTHER_IP" "other robot" 1
    ping_check "$LAPTOP_IP" laptop 0
    if tcp_open "$MASTER_IP" 22; then pass "master-pi SSH port open"; else fail "master-pi SSH port closed"; fi
    if [ "$ROLE" = robot1 ]; then camera_port 127.0.0.1 5601 robot1; else camera_port 127.0.0.1 5602 robot2; fi

    say "Time sync (chronyc tracking)"
    tracking="$(chronyc -n tracking 2>/dev/null)"
    echo "$tracking" | sed 's/^/        /'
    if echo "$tracking" | grep -q "^Reference ID.*($MASTER_IP)"; then pass "synced to master-pi $MASTER_IP"
    else fail "not synced to $MASTER_IP yet (just booted? wait 1 min. Else check 'chronyc -n sources')"; fi
    if echo "$tracking" | grep -q '^Leap status *: Normal'; then pass "leap status Normal"; else fail "leap status not Normal (not synchronised)"; fi
    off="$(echo "$tracking" | awk '/^System time/ {print $4}')"
    if [ -n "$off" ] && awk -v o="$off" 'BEGIN { exit !(o < 0.005) }'; then pass "clock offset ${off} s (< 5 ms)"
    else warn "clock offset ${off:-?} s (target < 5 ms; it shrinks over the first minutes)"; fi
fi

echo
if [ "$FAILS" -eq 0 ]; then echo "RESULT: ALL REQUIRED CHECKS PASSED ($WARNS WARN)"
else echo "RESULT: $FAILS FAIL, $WARNS WARN - see network/README.md -> Troubleshooting"; fi
exit "$FAILS"
