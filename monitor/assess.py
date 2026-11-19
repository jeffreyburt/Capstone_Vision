"""Turn raw probe results and heartbeats into status rows for the system GUI.

Every machine gets a list of checks, {"label", "level", "text"}, where level is one of
    ok       working
    warn     working, but something needs attention
    bad      broken: the system can't work like this
    off      not running, and that is not necessarily a problem (e.g. head tracker during a drive session)
    unknown  not checked yet, or can't be checked (e.g. the Pi is offline)
plus a one-line summary for the network diagram. Pure functions: no I/O, easy to test.
"""

import math

OK, WARN, BAD, OFF, UNKNOWN = "ok", "warn", "bad", "off", "unknown"
_RANK = {OFF: 0, UNKNOWN: 1, OK: 2, WARN: 3, BAD: 4}  # a machine's level is its highest-ranked row

TEMP_WARN_C, TEMP_BAD_C = 70.0, 80.0         # a Pi 4 starts throttling at 80-85 C
WIFI_WARN_DBM, WIFI_BAD_DBM = -67.0, -75.0
CLOCK_WARN_S = 0.4                            # vision's joint feedback rejects samples older than this
HEARTBEAT_LOST_S = 2.0                        # a laptop program that hasn't reported for this long has stopped
FRAME_STALL_S = 3.0


def check(label, level, text):
    return {"label": label, "level": level, "text": text}


def worst(checks):
    """bad > warn > ok > unknown > off: one problem makes the machine a problem; "off" rows only count when
    nothing else is known."""
    return max((c["level"] for c in checks), key=_RANK.get, default=UNKNOWN)


def fmt_duration(s):
    if s is None:
        return "?"
    s = abs(s)
    if s < 10:
        return f"{s:.1f} s"
    if s < 120:
        return f"{s:.0f} s"
    if s < 7200:
        return f"{s / 60:.0f} min"
    if s < 2 * 86400:
        return f"{s / 3600:.1f} h"
    return f"{s / 86400:.1f} d"


def fmt_offset(s):
    """Clock offset vs the laptop, as words."""
    if abs(s) < 0.05:
        return "in sync with the laptop"
    return f"{fmt_duration(s)} {'ahead of' if s > 0 else 'behind'} the laptop"


def heartbeat(msg, age_s):
    """The message if that laptop program is running, else None."""
    if msg is None or age_s is None or age_s > HEARTBEAT_LOST_S or msg.get("state") == "exited":
        return None
    return msg


# ---------------------------------------------------------------------------------------------------------
# Pieces shared by the Pis

def network_check(label, host, ping, hint):
    if ping is None:
        return check(label, UNKNOWN, "checking...")
    if ping["up"]:
        return check(label, OK, f"{host} answers in {ping['ms']:.0f} ms")
    return check(label, BAD, f"no answer from {host} ({ping['error']}): {hint}")


def health_checks(raw):
    """System, power and clock rows from the SSH health probe. raw: the node's probe results."""
    h, err = raw.get("health"), raw.get("health_error")
    if h is None:
        text = err or ("checking..." if raw.get("ssh_enabled", True) else "SSH checks are off (system.ssh_interval_s)")
        level = WARN if err and "no connection" not in err else UNKNOWN
        return [check("System", level, text)]
    out = []

    parts, level = [], OK
    if h["temp_c"] is not None:
        parts.append(f"{h['temp_c']:.0f}°C")
        level = BAD if h["temp_c"] >= TEMP_BAD_C else WARN if h["temp_c"] >= TEMP_WARN_C else OK
    if h["load"]:
        parts.append(f"load {h['load'][0]:.2f}")
        if h["ncpu"] and h["load"][0] > h["ncpu"]:
            level = max(level, WARN, key=_RANK.get)
    if h["mem_used_pct"] is not None:
        parts.append(f"memory {h['mem_used_pct']:.0f}%")
        if h["mem_used_pct"] >= 90:
            level = max(level, WARN, key=_RANK.get)
    if h["disk_used_pct"] is not None:
        parts.append(f"disk {h['disk_used_pct']:.0f}%")
        if h["disk_used_pct"] >= 90:
            level = max(level, BAD if h["disk_used_pct"] >= 98 else WARN, key=_RANK.get)
    if h["uptime_s"] is not None:
        parts.append(f"up {fmt_duration(h['uptime_s'])}")
    out.append(check("System", level, " · ".join(parts) or "no data"))

    t = h["throttled"]
    if t is None:
        out.append(check("Power", UNKNOWN, "can't read (vcgencmd missing?)"))
    elif t["now"]:
        out.append(check("Power", BAD, ", ".join(t["now"]) + " NOW: check the supply / battery"))
    elif t["since_boot"]:
        out.append(check("Power", WARN, ", ".join(t["since_boot"]) + " happened since boot"))
    else:
        out.append(check("Power", OK, "no under-voltage or throttling since boot"))

    off = h["clock_offset_s"]
    if off is not None:
        ntp = h["ntp"]["ref"]
        sync = f" · chrony: {ntp}" if ntp else ""
        level = OK if abs(off) < CLOCK_WARN_S else WARN
        hint = "" if level == OK else " (matters for head-pose compensation)"
        out.append(check("Clock", level, f"{fmt_offset(off)}{sync}{hint}"))
    return out


def wifi_level(dbm):
    return OK if dbm >= WIFI_WARN_DBM else WARN if dbm >= WIFI_BAD_DBM else BAD


def processes(raw, role):
    h = raw.get("health")
    return [p for p in (h or {}).get("processes", []) if p["role"] == role]


def _details(raw):
    h = raw.get("health") or {}
    return {"model": h.get("model"), "os": h.get("os"), "hostname": h.get("hostname"), "ips": h.get("ips"),
            "processes": h.get("processes"), "health_at": raw.get("health_at"), "probe_at": raw.get("at")}


# ---------------------------------------------------------------------------------------------------------
# Laptop

def assess_laptop(vision_msg, vision_age, control_msg, control_age, now):
    vision, control = heartbeat(vision_msg, vision_age), heartbeat(control_msg, control_age)
    checks = []
    if vision is None:
        checks.append(check("Vision", OFF, "not running: python -m vision.main --gui"))
    elif vision.get("state") == "waiting":
        checks.append(check("Vision", WARN, f"started, waiting for the first frame from {vision.get('camera')}"))
    else:
        stalled = now - vision.get("last_frame_t", now)
        markers = vision.get("markers") or []
        if stalled > FRAME_STALL_S:
            checks.append(check("Vision", WARN, f"no new frame for {fmt_duration(stalled)} from {vision.get('camera')}"))
        else:
            fps = vision.get("fps")
            checks.append(check("Vision", OK, f"{fps:.1f} fps from {vision.get('camera')}" if fps else
                                f"running on {vision.get('camera')}"))
        checks.append(check("Markers", OK if markers else WARN,
                            f"seeing IDs {', '.join(map(str, markers))}" if markers else "no markers in view"))
        if vision.get("calibrated"):
            checks.append(check("Floor calibration", OK, "loaded"))
        else:
            checks.append(check("Floor calibration", BAD, "missing: no floor positions (tools/calibrate_floor.py)"))
        user = (vision.get("message") or {}).get("user")
        if not vision.get("people"):
            checks.append(check("Person detection", OFF, "off (people.enabled, or ultralytics not installed)"))
        elif user:
            seen = "" if user["age"] < 1 else f", last seen {fmt_duration(user['age'])} ago"
            checks.append(check("Person detection", OK, f"user at ({user['x']:.2f}, {user['y']:.2f}) m{seen}"))
        else:
            checks.append(check("Person detection", WARN, "nobody in view"))

    if control is None:
        checks.append(check("Control", OFF, "not running: python -m control.main"))
    else:
        mode = control.get("mode", "?")
        ids = ", ".join(sorted(control.get("robots", {}), key=str))
        text = {"ARMED": f"ARMED: driving robots {ids}", "PAUSED": f"PAUSED: robots {ids} stopped",
                "DRY RUN": f"dry run for robots {ids} (sends nothing)"}.get(mode, mode)
        checks.append(check("Control", WARN if mode == "PAUSED" else OK, text))

    tokens = []
    if vision is not None:
        tokens.append(f"vision {vision['fps']:.0f} fps" if vision.get("fps") else "vision starting")
    if control is not None:
        tokens.append(f"control {control.get('mode', '?').lower()}")
    return {"key": "laptop", "name": "Laptop", "kind": "laptop", "address": "this computer",
            "level": worst(checks), "summary": " · ".join(tokens) or "nothing running", "checks": checks,
            "details": {}, "camera": None}


# ---------------------------------------------------------------------------------------------------------
# Master Pi

def assess_master(node, raw, vision_msg, vision_age, robot_names):
    """robot_names: {ip: "Robot 1"} to label the access point's clients."""
    vision = heartbeat(vision_msg, vision_age)
    ping = raw.get("ping")
    checks = [network_check("Ethernet", node.host, ping, "powered? cable plugged in?")]
    online = bool(ping and ping["up"])
    if node.wifi_host and online:
        pw = raw.get("ping_wifi")
        if pw is not None and not pw["up"]:
            checks.append(check("Route to robots", BAD, f"SSH gateway check failed: {pw['error']}" if raw.get("probe_via") else
                                f"laptop can't reach {node.wifi_host}: access point down, "
                                "or the laptop's route is missing (admin PowerShell: "
                                f"route -p add 192.168.50.0 mask 255.255.255.0 {node.host})"))
        elif pw is not None:
            via = raw.get("probe_via")
            checks.append(check("Route to robots", OK, f"{node.wifi_host} reachable" +
                                (f" via SSH ({via}); laptop route not checked" if via else "")))

    h = raw.get("health")
    if online:
        checks += health_checks(raw)
        if h is not None:
            wifi = h["wifi"]
            if wifi["mode"] == "AP":
                names = []
                for st in h["stations"]:
                    who = robot_names.get(st["ip"]) or st["ip"] or st["mac"]
                    names.append(f"{who} {st['signal_dbm']:.0f} dBm" if st["signal_dbm"] is not None else who)
                clients = ("clients: " + ", ".join(names)) if names else "no clients connected"
                checks.append(check("Robot WiFi", OK if names else WARN,
                                    f"{wifi['ssid']} ch {wifi['channel']} · {clients}"))
            else:
                checks.append(check("Robot WiFi", BAD, "wlan0 is not an access point: the robots have no network "
                                    "(NetworkManager profile robot-ap)"))

    for name, port in node.streams.items():
        p = (raw.get("ports") or {}).get(name)
        procs = [x for x in processes(raw, "camera") if f"sender {name}" in x["detail"]]
        uptime = f", up {fmt_duration(procs[0]['uptime_s'])}" if procs else ""
        if not online or p is None:
            checks.append(check("Camera sender", UNKNOWN, "Pi offline" if not online else "checking..."))
        elif p["open"]:
            checks.append(check("Camera sender", OK, f"{name} on port {port}{uptime}"))
        else:
            checks.append(check("Camera sender", BAD, f"not running: python -m camstream.sender {name}"))

    if vision is None:
        checks.append(check("Frames to vision", OFF, "vision.main not running"))
    else:
        s = vision.get("stream")
        if s is None:
            checks.append(check("Frames to vision", OFF, f"vision.main reads {vision.get('camera')}, not the Pi"))
        elif s.get("since_last_s") is None:
            checks.append(check("Frames to vision", BAD, "vision.main has received no frames yet"))
        elif s["since_last_s"] > FRAME_STALL_S:
            checks.append(check("Frames to vision", BAD, f"stalled: last frame {fmt_duration(s['since_last_s'])} ago"))
        else:
            checks.append(check("Frames to vision", OK, f"{s['fps']:.1f} fps, {s['mbps']:.0f} Mbps, "
                                f"{s['kb_per_frame']:.0f} kB/frame, {s['dropped']} dropped"))

    tokens = ["offline"] if ping is not None and not online else []
    if online and h is not None and h["temp_c"] is not None:
        tokens.append(f"{h['temp_c']:.0f}°C")
    if online and any(c["label"] == "Camera sender" and c["level"] == OK for c in checks):
        tokens.append("camera on")
    if online and h is not None and h["wifi"]["mode"] == "AP":
        tokens.append(f"WiFi {len(h['stations'])} client{'s' if len(h['stations']) != 1 else ''}")
    camera = None
    if vision is not None and vision.get("gui_port"):
        camera = {"port": vision["gui_port"], "path": "/video/birdseye", "label": "birds-eye, from the vision GUI"}
    return {"key": node.key, "name": node.name, "kind": "master", "address": node.host,
            "level": BAD if ping is not None and not online else worst(checks), "summary": " · ".join(tokens),
            "checks": checks, "details": {**_details(raw), "ssh_user": node.ssh_user,
                                          "probe_via": raw.get("probe_via"),
                                          "stations": (h or {}).get("stations")}, "camera": camera}


# ---------------------------------------------------------------------------------------------------------
# Robots

def assess_robot(node, raw, master_raw, vision_msg, vision_age, control_msg, control_age):
    vision, control = heartbeat(vision_msg, vision_age), heartbeat(control_msg, control_age)
    rid = node.robot_id
    ping = raw.get("ping")
    online = bool(ping and ping["up"])
    checks = [network_check("Network", node.host, ping, "powered? on the capstone-robots WiFi?")]
    h = raw.get("health")

    # WiFi signal: the robot's own measurement, else what the access point hears from it
    dbm, source = None, ""
    if h is not None and h["wifi"]["signal_dbm"] is not None:
        dbm = h["wifi"]["signal_dbm"]
        source = f" on {h['wifi']['ssid']}" if h["wifi"]["ssid"] else ""
    else:
        for st in ((master_raw.get("health") or {}).get("stations") or []):
            if st["ip"] == node.host and st["signal_dbm"] is not None:
                dbm, source = st["signal_dbm"], " (heard by the access point)"
    if dbm is not None:
        checks.append(check("WiFi", wifi_level(dbm), f"{dbm:.0f} dBm{source}"))

    if online:
        checks += health_checks(raw)

    # Drive server
    ctl_robot = (control or {}).get("robots", {}).get(str(rid))
    armed = control is not None and control.get("mode") == "ARMED" and ctl_robot is not None
    d = raw.get("drive")
    drive_token = None
    if not online or d is None:
        checks.append(check("Drive server", UNKNOWN, "Pi offline" if ping and not online else "checking..."))
    elif d["reply"] is not None:
        rep = d["reply"]
        if rep.get("robot_id") != rid:
            checks.append(check("Drive server", BAD, f"answers as robot {rep.get('robot_id')}: wrong --robot-id"))
        else:
            state = rep.get("state", "?")
            motors = rep.get("motors")
            if motors and any(motors):
                state += f", motors {motors}"
            if "hardware" in rep:
                kind = "hardware" if rep["hardware"] else "simulation (no motors)"
                drive_token = "drive HW" if rep["hardware"] else "drive SIM"
                text = f"{kind}: {state}"
                if rep.get("last_cmd_age_s") is not None:
                    text += f", last command {fmt_duration(rep['last_cmd_age_s'])} ago"
                if rep.get("error"):
                    text += f" · last error: {rep['error']}"
                checks.append(check("Drive server", WARN if rep.get("error") else OK, text))
            else:  # older drive server: answers, but without the detail fields
                drive_token = "drive on"
                error = rep.get("error")
                checks.append(check("Drive server", WARN if error else OK, f"running: {state} (update robot/drive_server.py on the "
                                    "Pi for hardware/simulation and command details)" +
                                    (f" · last error: {error}" if error else "")))
    else:
        level = BAD if armed else OFF
        checks.append(check("Drive server", level, f"not running (no answer on UDP {node.drive_port})"))

    # Head tracker dashboard
    dash = raw.get("dashboard")
    cam_live = None
    if node.dashboard_port and online and dash is not None:
        st = dash["data"]
        if st is not None:
            pos = st.get("positions") or [None, None]
            torque = st.get("torque") or [None, None]
            tr = st.get("tracking") or {}
            cam = st.get("camera") or {}
            cam_live = cam if cam.get("live") else None
            parts = [st.get("status") or "?"]
            if all(isinstance(v, int) for v in pos):
                parts.append(f"pan {pos[0]} tilt {pos[1]}")
            parts.append("torque " + ("on" if all(t == 1 for t in torque) else
                                      "off" if all(t == 0 for t in torque) else str(torque)))
            parts.append("tracking " + ("on" if tr.get("active") else "off"))
            err = st.get("position_error")
            checks.append(check("Head tracker", WARN if err else OK,
                                " · ".join(parts) + (f" · servo error: {err}" if err else "")))
        else:
            checks.append(check("Head tracker", OFF, f"not running ({dash['error']})"))

    # Camera
    stream = next(iter(node.streams), None)
    port = (raw.get("ports") or {}).get(stream) if stream else None
    camera = None
    if port is not None and port["open"]:
        checks.append(check("Camera", OK, f"camstream sender {stream} on port {node.streams[stream]}"))
        if vision is not None and vision.get("gui_port"):  # the vision GUI already receives it: no extra WiFi load
            camera = {"port": vision["gui_port"], "path": f"/video/{stream}", "label": f"{stream} via the vision GUI"}
        else:
            camera = {"port": None, "path": f"/video/{stream}", "label": f"{stream} camstream"}
    elif cam_live is not None:
        checks.append(check("Camera", OK, f"head tracker camera live ({cam_live.get('model') or 'camera'})"))
        camera = {"url": f"http://{node.host}:{node.dashboard_port}/camera.mjpg", "label": "head tracker camera"}
    elif online:
        camera_status = ((dash or {}).get("data") or {}).get("camera") or {}
        checks.append(check("Camera", WARN if camera_status.get("status") else OFF,
                            camera_status.get("status") or "no camera stream (camstream sender or head tracker)"))

    # Seen by the birds-eye camera
    pose = None
    if vision is None:
        checks.append(check("Seen by vision", OFF, "vision.main not running"))
    else:
        pose = next((r for r in (vision.get("message") or {}).get("robots", []) if r["id"] == rid), None)
        if pose is None:
            checks.append(check("Seen by vision", WARN, "not in view of the birds-eye camera"))
        else:
            heading = "?" if pose.get("heading") is None else f"{pose['heading']:.0f}°"
            text = f"({pose['x']:.2f}, {pose['y']:.2f}) m, heading {heading}"
            if pose["age"] > 1.0:
                checks.append(check("Seen by vision", WARN, f"{text}, last seen {fmt_duration(pose['age'])} ago"))
            else:
                checks.append(check("Seen by vision", OK, text))

    # Control loop
    if control is None:
        checks.append(check("Control", OFF, "control.main not running"))
    elif ctl_robot is None:
        checks.append(check("Control", OFF, "control.main isn't driving this robot (--robots)"))
    else:
        reason, cmd, link = ctl_robot.get("reason", "?"), ctl_robot.get("cmd") or [0, 0, 0], ctl_robot.get("link_age_s")
        text = f"{control.get('mode')}: {reason}"
        if any(cmd):
            text += f", command ({cmd[0]:+.2f}, {cmd[1]:+.2f}) m/s {math.degrees(cmd[2]):+.0f}°/s"
        if ctl_robot.get("target"):
            tx, ty, th = ctl_robot["target"]
            text += f" → target ({tx:.2f}, {ty:.2f}) {th:.0f}°"
        level = OK
        if control.get("mode") == "ARMED":
            if link is None or link > 1.0:
                level, text = BAD, text + (" · no reply from the drive server" if link is None else
                                           f" · drive server silent for {fmt_duration(link)}")
            elif reason in ("robot not seen", "compensated pose unavailable", "no vision"):
                level = WARN
        checks.append(check("Control", level, text))

    tokens = []
    if ping is not None and not online:
        tokens.append("offline")
    else:
        if dbm is not None:
            tokens.append(f"{dbm:.0f} dBm")
        if drive_token:
            tokens.append(drive_token)
        if pose is not None:
            tokens.append("seen" if pose["age"] <= 1.0 else "lost")
    return {"key": node.key, "name": node.name, "kind": "robot", "robot_id": rid, "address": node.host,
            "wifi_dbm": dbm if online else None,
            "level": BAD if ping is not None and not online else worst(checks), "summary": " · ".join(tokens),
            "checks": checks, "camera": camera,
            "details": {**_details(raw), "drive": (d or {}).get("reply"), "ssh_user": node.ssh_user,
                        "probe_via": raw.get("probe_via")}}
