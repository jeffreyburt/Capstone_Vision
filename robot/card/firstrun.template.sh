#!/bin/bash
# First-boot setup for a capstone robot Pi. Generated from robot/card/firstrun.template.sh by
# robot/card/prepare_card.py. It runs once as root (from cmdline.txt's systemd.run), then deletes itself,
# which also removes the password hash and WiFi key it contains. Log: <boot>/robot-setup/firstrun.log
set +e
BOOT=/boot/firmware
[ -f "$BOOT/cmdline.txt" ] || BOOT=/boot
SETUP="$BOOT/robot-setup"
exec >>"$SETUP/firstrun.log" 2>&1
echo "=== robot first-boot setup started $(date)"

ROBOT_ID=__ROBOT_ID__
NEW_HOSTNAME=__HOSTNAME__
USERNAME=__USERNAME__
PWHASH='__PWHASH__'
PUBKEY='__PUBKEY__'
IP=__IP__
PSKHEX=__PSKHEX__

echo "--- hostname $NEW_HOSTNAME"
CURRENT="$(tr -d ' \t\n\r' < /etc/hostname)"
if [ -x /usr/lib/raspberrypi-sys-mods/imager_custom ]; then
    /usr/lib/raspberrypi-sys-mods/imager_custom set_hostname "$NEW_HOSTNAME"
else
    echo "$NEW_HOSTNAME" > /etc/hostname
    sed -i "s/127.0.1.1.*$CURRENT/127.0.1.1\t$NEW_HOSTNAME/g" /etc/hosts
fi

echo "--- user $USERNAME (creates it, or renames this image's first user)"
if [ -x /usr/lib/userconf-pi/userconf ]; then
    /usr/lib/userconf-pi/userconf "$USERNAME" "$PWHASH"
elif ! id "$USERNAME" >/dev/null 2>&1; then
    useradd -m -s /bin/bash "$USERNAME"
    echo "$USERNAME:$PWHASH" | chpasswd -e
fi
usermod -aG sudo,i2c,dialout,gpio "$USERNAME"
HOME_DIR="$(getent passwd "$USERNAME" | cut -d: -f6)"

echo "--- SSH with the laptop's key"
install -d -m 700 -o "$USERNAME" -g "$USERNAME" "$HOME_DIR/.ssh"
grep -qxF "$PUBKEY" "$HOME_DIR/.ssh/authorized_keys" 2>/dev/null || echo "$PUBKEY" >> "$HOME_DIR/.ssh/authorized_keys"
chown "$USERNAME:$USERNAME" "$HOME_DIR/.ssh/authorized_keys"
chmod 600 "$HOME_DIR/.ssh/authorized_keys"
systemctl enable ssh

echo "--- WiFi capstone-robots, static $IP, gateway 192.168.50.1"
raspi-config nonint do_wifi_country US
rfkill unblock wifi
for f in /var/lib/systemd/rfkill/*:wlan; do [ -e "$f" ] && echo 0 > "$f"; done
NMFILE=/etc/NetworkManager/system-connections/capstone-robots.nmconnection
cat > "$NMFILE" <<EOF
[connection]
id=capstone-robots
type=wifi
interface-name=wlan0
autoconnect=true
autoconnect-priority=20

[wifi]
mode=infrastructure
ssid=capstone-robots
powersave=2

[wifi-security]
key-mgmt=wpa-psk
psk=$PSKHEX

[ipv4]
method=manual
address1=$IP/24,192.168.50.1

[ipv6]
method=disabled
EOF
chmod 600 "$NMFILE"

echo "--- I2C (wheel board) and UART for the servo bus, no serial login console"
raspi-config nonint do_i2c 0
if raspi-config nonint do_serial_cons 1 2>/dev/null; then
    raspi-config nonint do_serial_hw 0
else
    raspi-config nonint do_serial 2
fi
echo "--- boot to the console (a robot needs no desktop)"
raspi-config nonint do_boot_behaviour B1

echo "--- robot code"
sudo -u "$USERNAME" -H bash "$SETUP/install_robot_code.sh" "$ROBOT_ID"

echo "--- drive server service (installed, NOT enabled)"
cat > /etc/systemd/system/robot-drive.service <<EOF
[Unit]
Description=Robot $ROBOT_ID drive server (UDP 5700 -> wheels)
After=network-online.target

[Service]
User=$USERNAME
WorkingDirectory=$HOME_DIR/capstone-base
ExecStart=$HOME_DIR/capstone-base/.venv/bin/python $HOME_DIR/drive_server.py --robot-id $ROBOT_ID --hardware
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

echo "--- removing this script (it holds secrets) and the one-shot boot hook"
rm -f "$BOOT/firstrun.sh"
sed -i 's| systemd.run.*||g' "$BOOT/cmdline.txt"
echo "=== robot first-boot setup finished $(date); rebooting"
exit 0
