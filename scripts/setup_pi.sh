#!/usr/bin/env bash
# One-time setup on the Raspberry Pi. Run from anywhere:  bash scripts/setup_pi.sh
set -euo pipefail
cd "$(dirname "$0")/.."

echo "== Installing system packages (Picamera2 comes from apt, not pip) =="
sudo apt update
sudo apt install -y --no-install-recommends python3-picamera2 python3-venv python3-pip libgl1 git

echo "== Creating virtualenv .venv (with access to the system Picamera2) =="
if [ ! -d .venv ]; then
    python3 -m venv --system-site-packages .venv
fi
.venv/bin/pip install --upgrade pip
.venv/bin/pip install -r requirements.txt

echo "== Checking imports =="
if .venv/bin/python -c "import numpy, cv2, yaml, picamera2; print('numpy', numpy.__version__, '| opencv', cv2.__version__, '| picamera2 OK')"; then
    echo
    echo "Setup OK. Next:  source .venv/bin/activate && python tools/pi_check.py"
else
    echo
    echo "Import check FAILED. If the error mentions numpy (e.g. 'dtype size changed' or"
    echo "'compiled using NumPy 1.x'), the pip numpy doesn't match the apt-built Picamera2. Fix with:"
    echo "    .venv/bin/pip install \"numpy<2\" \"opencv-python==4.10.0.84\""
    echo "then run this script again. See docs/PI_GUIDE.md -> Troubleshooting."
    exit 1
fi
