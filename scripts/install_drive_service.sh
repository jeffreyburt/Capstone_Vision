#!/usr/bin/env bash
# Install a wheel server with an explicitly supplied, tested robot-specific mapping.
# Run as the robot user, not root. Hardware initialization sends zero speed only.
# Usage: bash install_drive_service.sh 2 /path/to/python /path/to/drive_server.py /path/to/wheels.json 192.168.1.10 0.16
set -euo pipefail
if [[ $# -lt 5 || $# -gt 6 || ! $1 =~ ^[1-9][0-9]*$ || $EUID == 0 ]]; then
  echo 'Usage (as robot user): install_drive_service.sh ROBOT_ID PYTHON SERVER VERIFIED_WHEEL_JSON LAPTOP_IP [TURN_RADIUS]' >&2
  exit 2
fi
robot_id=$1
python_path=$2
server_path=$3
mapping_path=$4
laptop_ip=$5
turn_radius=${6:-0.2}
for path in "$python_path" "$server_path" "$mapping_path"; do
  [[ $path == /* && -f $path && $path != *$'\n'* && $path != *'"'* ]] || { echo "Invalid absolute file path: $path" >&2; exit 2; }
done
"$python_path" - "$mapping_path" "$laptop_ip" "$turn_radius" <<'PY'
import ipaddress, json, sys, math
import smbus2
from capstone_base.hiwonder_i2c import HiwonderConfig
ipaddress.IPv4Address(sys.argv[2])
if not math.isfinite(float(sys.argv[3])) or not 0 < float(sys.argv[3]) <= 2:
    raise SystemExit('Turn radius must be positive and <= 2 meters')
with open(sys.argv[1]) as f:
    data = json.load(f)
hw = data.get('hiwonder', data)
if sorted(hw.get('wheel_channels', [])) != [1, 2, 3, 4] or len(hw.get('motor_signs', [])) != 4:
    raise SystemExit('A verified four-wheel channel/sign mapping is required')
fields = HiwonderConfig.__dataclass_fields__
HiwonderConfig(**{k: v for k, v in hw.items() if k in fields})
print('Driver imports and mapping validated; no I2C opened')
PY
"$python_path" "$server_path" --help | grep -q -- '--require-compensated-pose'
unit="capstone-drive-${robot_id}.service"
unit_file=$(mktemp)
trap 'rm -f "$unit_file"' EXIT
cat > "$unit_file" <<EOF
[Unit]
Description=Capstone robot ${robot_id} wheel server (compensated-pose commands only)
After=network-online.target
Wants=network-online.target

[Service]
User=$(id -un)
WorkingDirectory=$(dirname "$server_path")
Environment=PYTHONUNBUFFERED=1
ExecStart="${python_path}" -u "${server_path}" --robot-id ${robot_id} --hardware --hiwonder-config "${mapping_path}" --allow-from ${laptop_ip} --require-compensated-pose --turn-radius ${turn_radius} --timeout 0.5
Restart=on-failure
RestartSec=3
TimeoutStopSec=5
NoNewPrivileges=true

[Install]
WantedBy=multi-user.target
EOF
sudo install -m 644 "$unit_file" "/etc/systemd/system/$unit"
sudo systemctl daemon-reload
sudo systemctl enable --now "$unit"
echo "Installed $unit. Logs: journalctl -u $unit -f"
