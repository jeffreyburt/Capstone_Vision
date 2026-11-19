"""Prepare a Raspberry Pi OS (Bookworm) SD card, mounted on this laptop, to boot as a capstone robot Pi.

On its first boot the Pi then sets its hostname, login user, SSH key, the capstone-robots WiFi with a
static address, I2C and the servo UART, installs the robot code offline (Nate's capstone-base and this
repo's committed HEAD), and reboots. No flashing: the card must already hold Raspberry Pi OS Bookworm.

    python robot/card/prepare_card.py --boot D:\\ --robot 2 --bundle <dir>

<dir> holds what the Pi can't download itself (the robot WiFi has no internet):
    capstone-base.tar.gz   git archive --prefix=capstone-base/ of NateMG4/NEU-capstone-robot
    wheels/                smbus2 and pyserial wheels (pip download smbus2 pyserial --no-deps)

It asks for the capstone-robots WiFi password and a login password for the robot user. Neither is stored
in plain text: the WiFi password becomes the WPA key (PBKDF2), the login password a SHA-512 crypt hash,
and both are deleted from the card by the Pi on first boot.
"""

import argparse
import getpass
import hashlib
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
SSID = "capstone-robots"


def ask_twice(prompt, min_len, max_len):
    while True:
        a = getpass.getpass(prompt)
        if not min_len <= len(a) <= max_len or not a.isprintable():
            print(f"Use {min_len}-{max_len} printable characters.")
            continue
        if getpass.getpass("Repeat it: ") == a:
            return a
        print("They differ, try again.")


def find_openssl():
    for p in (shutil.which("openssl"), r"C:\Program Files\Git\usr\bin\openssl.exe",
              r"C:\Program Files\Git\mingw64\bin\openssl.exe"):
        if p and Path(p).exists():
            return p
    sys.exit("openssl not found (it comes with Git for Windows)")


def main():
    ap = argparse.ArgumentParser(description="Prepare a Bookworm SD card as a robot Pi")
    ap.add_argument("--boot", required=True, help="the card's boot partition (bootfs), e.g. D:\\")
    ap.add_argument("--robot", type=int, required=True, choices=(1, 2))
    ap.add_argument("--bundle", required=True, help="folder with capstone-base.tar.gz and wheels/")
    ap.add_argument("--username", default="user")
    ap.add_argument("--pubkey", default=str(Path.home() / ".ssh" / "id_ed25519.pub"))
    args = ap.parse_args()

    boot, bundle = Path(args.boot), Path(args.bundle)
    cmdline_path = boot / "cmdline.txt"
    if not cmdline_path.exists() or not (boot / "config.txt").exists():
        sys.exit(f"{boot} doesn't look like a Raspberry Pi boot partition")
    cmdline = cmdline_path.read_text().strip()
    if "systemd.run=" in cmdline:
        sys.exit("cmdline.txt already has a first-boot hook (systemd.run=); boot the card once or remove it")
    if not (bundle / "capstone-base.tar.gz").exists() or not list((bundle / "wheels").glob("*.whl")):
        sys.exit(f"{bundle} needs capstone-base.tar.gz and wheels/*.whl")
    pubkey = Path(args.pubkey).read_text().strip()

    hostname, ip = f"robot{args.robot}-pi", f"192.168.50.{10 + args.robot}"
    print(f"Card {boot}: robot {args.robot} -> hostname {hostname}, {ip} on {SSID}, user '{args.username}'")
    wifi = ask_twice(f"{SSID} WiFi password: ", 8, 63)
    login = ask_twice(f"New login password for '{args.username}' on the robot: ", 4, 128)
    psk_hex = hashlib.pbkdf2_hmac("sha1", wifi.encode(), SSID.encode(), 4096, 32).hex()
    pwhash = subprocess.run([find_openssl(), "passwd", "-6", "-stdin"], input=login, capture_output=True,
                            text=True, check=True).stdout.strip()
    if not pwhash.startswith("$6$"):
        sys.exit("openssl did not return a SHA-512 crypt hash")

    setup = boot / "robot-setup"
    if setup.exists():
        shutil.rmtree(setup)
    (setup / "wheels").mkdir(parents=True)
    shutil.copy2(bundle / "capstone-base.tar.gz", setup)
    for whl in (bundle / "wheels").glob("*.whl"):
        shutil.copy2(whl, setup / "wheels")

    def copy_lf(src, dst):  # the Pi's bash needs LF line endings
        dst.write_bytes(src.read_bytes().replace(b"\r\n", b"\n"))

    copy_lf(REPO / "robot" / "drive_server.py", setup / "drive_server.py")
    copy_lf(HERE / "install_robot_code.sh", setup / "install_robot_code.sh")
    # This repo as committed (head_tracker/, drive_with_head_lock.py, robot/), unpacked to ~/Capstone_Vision
    subprocess.run(["git", "-c", "core.autocrlf=false", "archive", "--format=tar.gz", "--prefix=Capstone_Vision/",
                    "-o", str(setup / "Capstone_Vision.tar.gz"), "HEAD"], cwd=REPO, check=True)

    script = (HERE / "firstrun.template.sh").read_text().replace("\r\n", "\n")
    fields = {"__ROBOT_ID__": str(args.robot), "__HOSTNAME__": hostname, "__USERNAME__": args.username,
              "__PWHASH__": pwhash, "__PUBKEY__": pubkey, "__IP__": ip, "__PSKHEX__": psk_hex}
    for key, value in fields.items():
        script = script.replace(key, value)
    if any(key in script for key in fields):
        sys.exit("internal error: a template field was not filled")
    with open(boot / "firstrun.sh", "w", newline="\n") as f:
        f.write(script)

    if "cfg80211.ieee80211_regdom=" not in cmdline:
        cmdline += " cfg80211.ieee80211_regdom=US"
    cmdline += (" systemd.run=/boot/firmware/firstrun.sh systemd.run_success_action=reboot"
                " systemd.unit=kernel-command-line.target")
    with open(cmdline_path, "w", newline="\n") as f:
        f.write(cmdline + "\n")

    print(f"\nCard ready. Eject it, put it in the robot Pi (3 or 4) and power on. First boot takes a few minutes and reboots.")
    print(f"Then from the laptop:  ssh {args.username}@{ip}")
    print(f"Setup log on the Pi:   /boot/firmware/robot-setup/firstrun.log")


if __name__ == "__main__":
    main()
