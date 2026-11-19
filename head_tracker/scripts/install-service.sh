#!/bin/bash
set -eu
if [ "$(id -u)" -ne 0 ] || [ -z "${SUDO_USER:-}" ] || [ "$SUDO_USER" = root ]; then
  echo 'Run from your normal Pi account: sudo bash scripts/install-service.sh' >&2
  exit 1
fi
CAPSTONE_REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
export CAPSTONE_REPO_DIR
/usr/bin/python3 - <<'PY'
import os,pwd,subprocess
from pathlib import Path
root=Path(os.environ['CAPSTONE_REPO_DIR']).resolve()
user=os.environ['SUDO_USER']
pwd.getpwnam(user)
for name in ('dashboard.py','camera_feed.py','tracking_control.py','index.html'):
    if not (root/name).is_file():raise SystemExit(f'Missing application file: {name}')
if any(c in str(root) for c in ('\n','\r','%')):raise SystemExit('Unsupported repository path')
def quote(value):return '"'+str(value).replace('\\','\\\\').replace('"','\\"')+'"'
quoted=quote(root)
script=quote(root/'dashboard.py')
unit=f'''[Unit]
Description=Capstone camera and servo dashboard
After=network.target

[Service]
Type=simple
User={user}
WorkingDirectory={quoted}
ExecStart=/usr/bin/python3 -u {script}
Restart=on-failure
RestartSec=3
TimeoutStopSec=5

[Install]
WantedBy=multi-user.target
'''
p=Path('/etc/systemd/system/capstone-dashboard.service')
if p.exists() and p.read_text()!=unit:
    from datetime import datetime
    p.with_name(p.name+'.backup-'+datetime.now().strftime('%Y%m%d-%H%M%S')).write_text(p.read_text())
p.write_text(unit)
subprocess.run(['systemctl','daemon-reload'],check=True)
subprocess.run(['systemctl','enable','capstone-dashboard.service'],check=True)
subprocess.run(['systemctl','restart','capstone-dashboard.service'],check=True)
print('Dashboard service enabled. Starting the service does not start motion.')
PY
