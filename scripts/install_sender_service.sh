#!/usr/bin/env bash
# Start this Pi's camera sender automatically on boot.
#   bash scripts/install_sender_service.sh birdseye    # on the master Pi
#   bash scripts/install_sender_service.sh robot1      # on robot 1's Pi
# Logs:  journalctl -u camstream-<name> -f      Stop:  sudo systemctl stop camstream-<name>
set -euo pipefail
NAME="${1:?usage: $0 <stream name: birdseye | robot1 | robot2>}"
cd "$(dirname "$0")/.."
DIR="$PWD"

sudo tee "/etc/systemd/system/camstream-${NAME}.service" > /dev/null <<EOF
[Unit]
Description=Capstone camera sender (${NAME})
After=network-online.target
Wants=network-online.target

[Service]
User=${USER}
WorkingDirectory=${DIR}
Environment=PYTHONUNBUFFERED=1
ExecStart=${DIR}/.venv/bin/python -m camstream.sender ${NAME}
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now "camstream-${NAME}"
echo "Installed and started camstream-${NAME}. Logs: journalctl -u camstream-${NAME} -f"
