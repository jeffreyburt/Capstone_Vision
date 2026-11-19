"""Checks the system GUI runs against each Pi. All read-only; nothing has to be installed on the Pis.

    tcp_check        is the Pi up? (TCP connect to SSH; "refused" also proves the Pi is up)
    drive_status     robot/drive_server.py: UDP {"type": "status"} query
    http_json        head tracker dashboard: GET /api/state
    ssh_health       temperature, power, CPU, memory, disk, WiFi, clock, running programs (SSH with the
                     laptop's key, runs HEALTH_SCRIPT in bash; no password prompts: BatchMode)
"""

import json
import os
from pathlib import Path
import shlex
import shutil
import socket
import subprocess
import time
import urllib.request


def network_probe(host, ssh_port=22, wifi_host=None, streams=None, drive_port=None, dashboard_port=None):
    """Read-only network checks; also runs unchanged on an SSH gateway."""
    result = {"at": time.time(), "ping": tcp_check(host, ssh_port)}
    if wifi_host:
        result["ping_wifi"] = tcp_check(wifi_host, 22)
    if result["ping"]["up"]:
        result["ports"] = {name: tcp_check(s["host"], s["port"]) for name, s in (streams or {}).items()}
        if drive_port:
            result["drive"] = drive_status(host, drive_port)
        if dashboard_port:
            result["dashboard"] = http_json(f"http://{host}:{dashboard_port}/api/state")
    else:
        result.update(ports=None, drive=None, dashboard=None, health=None, health_error=None)
    return result


def gateway_probe(ssh, gateway, spec, timeout=15):
    """Run only the fixed read-only probes on the gateway; no Pi installation required."""
    source = Path(__file__).read_text() + '\nimport sys\nprint(json.dumps(network_probe(**json.load(sys.stdin))))\n'
    cmd = [ssh, "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=3", gateway,
           "python3 -c " + shlex.quote(source)]
    try:
        if not ssh:
            raise ValueError("no SSH client found on this laptop")
        p = subprocess.run(cmd, input=json.dumps(spec).encode(), capture_output=True, timeout=timeout)
        if p.returncode:
            raise ValueError(ssh_error_text(p.stderr.decode(errors="replace").strip().splitlines()[-1]
                                           if p.stderr else f"SSH exit {p.returncode}"))
        result = json.loads(p.stdout)
        result["at"] = time.time()  # gateway clocks may be days behind the laptop
        result["probe_via"] = gateway
        return result
    except (OSError, ValueError, subprocess.TimeoutExpired) as e:
        error = f"gateway {gateway}: {_short_error(e)}"
        failed = {"up": False, "open": False, "ms": None, "error": error}
        return {"at": time.time(), "probe_via": gateway, "ping_wifi": failed,
                "ping": failed,
                "ports": None, "drive": None, "dashboard": None, "health": None, "health_error": error}


def tcp_check(host, port, timeout=1.0):
    """{"up": bool (the host answered), "open": bool (something listens), "ms": connect time, "error"}."""
    t0 = time.perf_counter()
    try:
        socket.create_connection((host, int(port)), timeout=timeout).close()
        return {"up": True, "open": True, "ms": round((time.perf_counter() - t0) * 1000, 1), "error": None}
    except ConnectionRefusedError:  # the host sent a reset: it is up, the port is closed
        return {"up": True, "open": False, "ms": round((time.perf_counter() - t0) * 1000, 1), "error": "refused"}
    except OSError as e:  # timeout, no route, host unreachable
        return {"up": False, "open": False, "ms": None, "error": _short_error(e)}


def drive_status(host, port, timeout=0.5):
    """Ask a drive server for its status. Returns {"reply": dict or None, "ms", "error"}.

    Old drive servers (before the status query existed) still answer, with
    error "unknown command type 'status'" and without the detail fields."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("0.0.0.0", 0))
        sock.settimeout(timeout)
        t0 = time.perf_counter()
        sock.sendto(json.dumps({"type": "status"}).encode(), (host, int(port)))
        while True:
            data, _ = sock.recvfrom(4096)
            try:
                reply = json.loads(data)
            except ValueError:
                continue
            if isinstance(reply, dict) and reply.get("type") == "status":
                return {"reply": reply, "ms": round((time.perf_counter() - t0) * 1000, 1), "error": None}
    except socket.timeout:
        return {"reply": None, "ms": None, "error": "no reply"}
    except ConnectionResetError:  # Windows: ICMP port unreachable -> the Pi is up, nothing on that port
        return {"reply": None, "ms": None, "error": "not running"}
    except OSError as e:
        return {"reply": None, "ms": None, "error": _short_error(e)}
    finally:
        sock.close()


def http_json(url, timeout=1.5):
    """GET a JSON document. Returns {"data": ..., "ms", "error"}."""
    t0 = time.perf_counter()
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            data = json.loads(r.read())
        return {"data": data, "ms": round((time.perf_counter() - t0) * 1000, 1), "error": None}
    except ConnectionRefusedError:
        return {"data": None, "ms": None, "error": "not running"}
    except (OSError, ValueError) as e:  # URLError is an OSError
        reason = getattr(e, "reason", e)
        if isinstance(reason, ConnectionRefusedError):
            return {"data": None, "ms": None, "error": "not running"}
        return {"data": None, "ms": None, "error": _short_error(reason)}


def _short_error(e):
    if isinstance(e, (socket.timeout, TimeoutError)):
        return "timeout"
    text = str(e) or type(e).__name__
    return text.split("] ", 1)[-1][:120]


# ---------------------------------------------------------------------------------------------------------
# Pi health over SSH

# Prints key=value lines; repeated keys (proc, station, neigh) become lists. Every command is read-only and
# allowed to fail. The time comes last so that it is measured just before the output returns to the laptop.
HEALTH_SCRIPT = r"""
export LC_ALL=C PATH="$PATH:/usr/sbin:/sbin"
echo "hostname=$(hostname)"
echo "model=$(cat /proc/device-tree/model 2>/dev/null | tr -d '\0')"
echo "os=$(. /etc/os-release 2>/dev/null; echo "$PRETTY_NAME")"
echo "uptime=$(cut -d' ' -f1 /proc/uptime)"
echo "load=$(cut -d' ' -f1-3 /proc/loadavg)"
echo "ncpu=$(nproc 2>/dev/null)"
echo "temp_mc=$(cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null)"
t=$(vcgencmd get_throttled 2>/dev/null | cut -d= -f2)
[ -z "$t" ] && t=$(cat /sys/devices/platform/soc/soc:firmware/get_throttled 2>/dev/null)
echo "throttled=$t"
awk '/^MemTotal:/{print "mem_total_kb="$2} /^MemAvailable:/{print "mem_avail_kb="$2}' /proc/meminfo
df -P / | awk 'NR==2{for(i=NF;i>0;i--) if($i ~ /%$/){sub("%","",$i); print "disk_used_pct="$i; exit}}'
for i in eth0 wlan0; do echo "ip_$i=$(ip -4 -o addr show "$i" 2>/dev/null | awk '{print $4}' | head -1)"; done
mode=$(iw dev wlan0 info 2>/dev/null | awk '$1=="type"{print $2}')
echo "wifi_mode=$mode"
if [ "$mode" = AP ]; then
  iw dev wlan0 info 2>/dev/null | awk '$1=="ssid"{print "wifi_ssid="$2} $1=="channel"{print "wifi_channel="$2}'
  # one line per associated client; the signal is missing on drivers that don't report it (Pi 4 in AP mode)
  iw dev wlan0 station dump 2>/dev/null | awk '$1=="Station"{if(mac)print "station="mac" "sig; mac=$2; sig=""}
    $1=="signal:"{sig=$2} END{if(mac)print "station="mac" "sig}'
  ip neigh show dev wlan0 2>/dev/null | awk '{print "neigh="$1" "$3" "$NF}'
else
  iw dev wlan0 link 2>/dev/null | awk '$1=="SSID:"{sub(/^[ \t]*SSID: /,""); print "wifi_ssid="$0}
    $1=="signal:"{print "wifi_signal="$2} $1=="tx" && $2=="bitrate:"{print "wifi_bitrate="$3}'
  awk '$1=="wlan0:"{sub(/\.$/,"",$4); print "wifi_level="$4}' /proc/net/wireless 2>/dev/null
fi
chronyc -n tracking 2>/dev/null | awk -F' *: ' '/^Reference ID/{print "ntp_ref="$2} /^System time/{print "ntp_offset="$2} /^Leap status/{print "ntp_leap="$2}'
ps -eo pid=,etimes=,args= | grep -E 'camstream[.]sender|drive_serve[r]|drive_with_head_loc[k]|dashboard[.]py|vision[.]main' | sed 's/^ */proc=/'
echo "time=$(date +%s.%N)"
"""

THROTTLE_BITS = {0: "under-voltage", 1: "CPU speed capped", 2: "throttled", 3: "soft temperature limit"}


def find_ssh(configured=None):
    """The ssh client to use: config `system.ssh`, else Windows' OpenSSH, else whatever is on PATH."""
    if configured:
        return configured
    win = os.path.join(os.environ.get("SystemRoot", r"C:\Windows"), "System32", "OpenSSH", "ssh.exe")
    if os.name == "nt" and os.path.isfile(win):
        return win  # Git Bash's ssh may use a different home folder (and so different keys / known hosts)
    return shutil.which("ssh")


def run_ssh(ssh, user, host, port=22, script=HEALTH_SCRIPT, timeout=10.0):
    """Run a bash script on a Pi. Returns (stdout, error message or None, laptop time when it returned)."""
    if not ssh:
        return "", "no ssh client found on this laptop", time.time()
    cmd = [ssh, "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=3", "-o", "StrictHostKeyChecking=accept-new",
           "-o", "ServerAliveInterval=2", "-o", "ServerAliveCountMax=2", "-p", str(port), f"{user}@{host}",
           "bash -s"]
    flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)  # no console window flashing up on Windows
    try:
        # Bytes, not text mode: on Windows text mode would turn the script's newlines into CRLF for bash.
        p = subprocess.run(cmd, input=script.encode(), capture_output=True, timeout=timeout, creationflags=flags)
    except subprocess.TimeoutExpired:
        return "", "ssh timed out", time.time()
    except OSError as e:
        return "", f"ssh failed to start: {e}", time.time()
    done = time.time()
    out, err = p.stdout.decode("utf-8", "replace"), p.stderr.decode("utf-8", "replace")
    if p.returncode != 0 and "time=" not in out:
        lines = [ln for ln in err.strip().splitlines() if ln and not ln.startswith("Warning: Permanently")]
        return out, ssh_error_text(lines[-1] if lines else f"exit code {p.returncode}"), done
    return out, None, done


def ssh_error_text(line):
    low = line.lower()
    if "permission denied" in low:
        return "SSH key login refused (copy the laptop's key with ssh-copy-id, or check ssh_user)"
    if "host key verification failed" in low or "remote host identification has changed" in low:
        return "SSH host key changed (Pi reinstalled?): remove its line from ~/.ssh/known_hosts"
    if "timed out" in low or "no route" in low or "unreachable" in low:
        return "SSH: no connection"
    if "refused" in low:
        return "SSH server not running"
    return line[:160]


def parse_health(text, returned_at=None):
    """HEALTH_SCRIPT output -> dict of typed values. returned_at: laptop time.time() when ssh returned."""
    raw, lists = {}, {"proc": [], "station": [], "neigh": []}
    for line in text.splitlines():
        key, sep, value = line.partition("=")
        if not sep:
            continue
        value = value.strip()
        if key in lists:
            lists[key].append(value)
        else:
            raw[key] = value

    def num(key, kind=float):
        try:
            return kind(raw[key])
        except (KeyError, ValueError):
            return None

    h = {"hostname": raw.get("hostname") or None, "model": raw.get("model") or None, "os": raw.get("os") or None,
         "uptime_s": num("uptime"), "ncpu": num("ncpu", int), "disk_used_pct": num("disk_used_pct")}
    try:
        h["load"] = [float(x) for x in raw["load"].split()]
    except (KeyError, ValueError):
        h["load"] = None
    mc = num("temp_mc")
    h["temp_c"] = None if mc is None else round(mc / 1000, 1)
    total, avail = num("mem_total_kb"), num("mem_avail_kb")
    h["mem_used_pct"] = round(100 * (1 - avail / total), 1) if total and avail is not None else None
    h["throttled"] = decode_throttled(raw.get("throttled"))
    h["ips"] = {i: raw[f"ip_{i}"].split("/")[0] for i in ("eth0", "wlan0") if raw.get(f"ip_{i}")}

    signal = num("wifi_signal")
    if signal is None:
        signal = num("wifi_level")
    h["wifi"] = {"mode": raw.get("wifi_mode") or None, "ssid": raw.get("wifi_ssid") or None,
                 "signal_dbm": signal, "bitrate_mbps": num("wifi_bitrate"), "channel": num("wifi_channel", int)}
    macs = {}
    for n in lists["neigh"]:
        parts = n.split()
        if len(parts) >= 2 and ":" in parts[1]:
            macs[parts[1].lower()] = parts[0]
    stations = []
    for s in lists["station"]:
        parts = s.split()
        if not parts:
            continue
        try:
            dbm = float(parts[1]) if len(parts) > 1 else None
        except ValueError:
            dbm = None
        stations.append({"mac": parts[0], "ip": macs.get(parts[0].lower()), "signal_dbm": dbm})
    h["stations"] = stations
    h["ntp"] = {"ref": raw.get("ntp_ref") or None, "offset": raw.get("ntp_offset") or None,
                "leap": raw.get("ntp_leap") or None}
    pi_time = num("time")
    h["clock_offset_s"] = round(pi_time - returned_at, 3) if pi_time is not None and returned_at else None
    h["processes"] = [p for p in (parse_process(line) for line in lists["proc"]) if p]
    return h


def decode_throttled(value):
    """vcgencmd get_throttled value ("0x50005") -> {"raw", "now": [...], "since_boot": [...]} or None."""
    if not value:
        return None
    try:
        bits = int(value, 16)
    except ValueError:
        return None
    return {"raw": value, "now": [name for b, name in THROTTLE_BITS.items() if bits >> b & 1],
            "since_boot": [name for b, name in THROTTLE_BITS.items() if bits >> (b + 16) & 1]}


def parse_process(line):
    """'1234 3600 python -m camstream.sender birdseye' -> {"pid", "uptime_s", "role", "detail", "args"}."""
    parts = line.split(None, 2)
    if len(parts) < 3:
        return None
    try:
        pid, uptime = int(parts[0]), float(parts[1])
    except ValueError:
        return None
    args = parts[2]
    words = args.split()

    def option(name):
        return words[words.index(name) + 1] if name in words[:-1] else None

    if "camstream.sender" in args:
        i = next((k for k, w in enumerate(words) if w.endswith("camstream.sender")), None)
        stream = words[i + 1] if i is not None and i + 1 < len(words) and not words[i + 1].startswith("-") else "?"
        role, detail = "camera", f"camera sender {stream}" + (" (test pattern)" if option("--source") == "test" else "")
    elif "drive_with_head_lock" in args:
        role, detail = "drive", f"drive server + head lock, robot {option('--robot-id') or '?'} (hardware)"
    elif "drive_server" in args:
        hw = "hardware" if "--hardware" in words else "simulation"
        role, detail = "drive", f"drive server, robot {option('--robot-id') or '?'} ({hw})"
    elif "dashboard.py" in args:
        role, detail = "head", "head tracker dashboard" + (" + pose feedback" if "--pose-host" in words else "")
    elif "vision.main" in args:
        role, detail = "vision", "vision.main"
    else:
        return None
    return {"pid": pid, "uptime_s": uptime, "role": role, "detail": detail, "args": args[:300]}
