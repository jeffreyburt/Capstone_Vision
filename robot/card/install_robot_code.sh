#!/bin/bash
# Install the robot code on a robot Pi, offline (the robot WiFi has no internet). Runs as the robot user;
# first boot runs it automatically from the boot partition. Safe to re-run:
#     bash /boot/firmware/robot-setup/install_robot_code.sh <robot id>
#
# Installs:
#   ~/capstone-base/          Nate's robot code (NateMG4/NEU-capstone-robot), with its .venv
#   ~/capstone-base/.venv     python venv with smbus2 + pyserial (from wheels/) and capstone_base importable
#   ~/capstone-base/.venv/bin/capstone-teleop etc.   the repo's command-line tools
#   ~/Capstone_Vision/        this repo (head_tracker/, drive_with_head_lock.py, robot/drive_server.py)
#   ~/drive_server.py         link to ~/Capstone_Vision/robot/drive_server.py
set -eu
SRC="$(cd "$(dirname "$0")" && pwd)"
cd "$HOME"
STAMP="$(date +%Y%m%d%H%M%S)"

if [ -d capstone-base ]; then
    mv capstone-base "capstone-base.old.$STAMP"
fi
tar xzf "$SRC/capstone-base.tar.gz"
if [ -f "$SRC/Capstone_Vision.tar.gz" ]; then
    if [ -d Capstone_Vision ]; then mv Capstone_Vision "Capstone_Vision.old.$STAMP"; fi
    tar xzf "$SRC/Capstone_Vision.tar.gz"
    ln -sf "$HOME/Capstone_Vision/robot/drive_server.py" "$HOME/drive_server.py"
else
    cp "$SRC/drive_server.py" "$HOME/drive_server.py"
fi

# The head tracker (head_tracker/README.md) uses these system packages with /usr/bin/python3.
for pkg in python3-serial python3-picamera2 ffmpeg rpicam-apps; do
    dpkg -s "$pkg" >/dev/null 2>&1 && echo "apt package $pkg: present" || echo "apt package $pkg: MISSING"
done

VENV="$HOME/capstone-base/.venv"
if python3 -m venv "$VENV" 2>/dev/null && "$VENV/bin/python" -m pip --version >/dev/null 2>&1; then
    "$VENV/bin/python" -m pip install --no-index --find-links "$SRC/wheels" smbus2 pyserial
else
    # No ensurepip on this image: a pip-less venv, and unpack the pure-Python wheels into it directly.
    rm -rf "$VENV"
    python3 -m venv --without-pip "$VENV"
    "$VENV/bin/python" - "$SRC/wheels" <<'EOF'
import glob, sys, sysconfig, zipfile
site = sysconfig.get_paths()["purelib"]
for whl in glob.glob(sys.argv[1] + "/*.whl"):
    zipfile.ZipFile(whl).extractall(site)
    print("unpacked", whl)
EOF
fi

# Make capstone_base importable without building the package (an editable install would need hatchling).
SITE="$("$VENV/bin/python" -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')"
echo "$HOME/capstone-base/src" > "$SITE/capstone_base.pth"

# The repo's console scripts (pyproject.toml [project.scripts])
for spec in capstone-base:cli capstone-motor-test:motor_test capstone-teleop:keyboard_teleop \
            capstone-servo-test:servo_test capstone-dashboard:dashboard capstone-pan-tilt:pan_tilt_dashboard \
            capstone-control:control_dashboard; do
    name="${spec%%:*}"; module="${spec#*:}"
    printf '#!%s\nfrom capstone_base.%s import main\nraise SystemExit(main())\n' "$VENV/bin/python" "$module" \
        > "$VENV/bin/$name"
    chmod +x "$VENV/bin/$name"
done

"$VENV/bin/python" -c "import capstone_base.hiwonder_i2c, smbus2, serial; print('robot code OK')"
