"""Read the control black box (control/blackbox.py) after a test run. Meant for people and agents alike.

    python tools/blackbox.py list                         # sessions, newest first
    python tools/blackbox.py summary [SESSION]            # what happened: modes, robots seen/lost, reasons, edge distance
    python tools/blackbox.py events [SESSION] [--robot 1] [--kind robot_lost,view_return]
    python tools/blackbox.py window [SESSION] --at 42.5 [--before 3 --after 3] [--robot 1] [--every 2]

SESSION defaults to the newest; give a folder name (20261122_153012), a unique prefix, or a path.
--at takes seconds since the start, a clock time (15:31:07), "mark" / "mark2" (first / second dashboard Mark),
or "event:N" (the Nth line of `events`). Times print as seconds since the session started.
Snapshots (annotated main-camera JPEGs) are in SESSION/snapshots/, named by clock time and event.
"""

import argparse
import datetime
import gzip
import json
import sys
import zlib
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

DEFAULT_DIR = ROOT / "logs" / "blackbox"


def sessions(base):
    return sorted((p for p in Path(base).iterdir() if p.is_dir() and (p / "meta.json").exists()), reverse=True) \
        if Path(base).exists() else []


def find_session(name, base):
    if name and Path(name).is_dir():
        return Path(name)
    found = sessions(base)
    if not found:
        raise SystemExit(f"No black-box sessions in {base}")
    if not name:
        return found[0]
    matches = [p for p in found if p.name.startswith(name)]
    if len(matches) != 1:
        raise SystemExit(f"{len(matches)} sessions match {name!r}; use `list`")
    return matches[0]


def read_events(session):
    path = session / "events.jsonl"
    if not path.exists():
        return []
    out = []
    for line in path.read_text(encoding="utf-8").splitlines():
        try:
            out.append(json.loads(line))
        except ValueError:
            pass  # a line cut off by a crash
    return out


def read_ticks(session):
    """Every recorded control step, in order. Tolerates a segment cut off by a crash."""
    for path in sorted(session.glob("ticks-*.jsonl.gz")):
        try:
            with gzip.open(path, "rt", encoding="utf-8") as f:
                for line in f:
                    try:
                        yield json.loads(line)
                    except ValueError:
                        return
        except (EOFError, OSError, zlib.error):
            continue  # truncated tail: everything before it was already yielded


def meta(session):
    return json.loads((session / "meta.json").read_text())


def rel(t, start):
    return f"{t - start:8.2f}"


def _fmt(v, n=2):
    return "-" if v is None else (f"{v:+.{n}f}" if isinstance(v, (int, float)) else str(v))


def resolve_at(at, session, start):
    events = read_events(session)
    if at.startswith("mark"):
        marks = [e for e in events if e["event"] == "mark"]
        i = int(at[4:] or 1) - 1
        if not 0 <= i < len(marks):
            raise SystemExit(f"only {len(marks)} marks in this session")
        return marks[i]["t"]
    if at.startswith("event:"):
        return events[int(at[6:]) - 1]["t"]
    if ":" in at:
        day = datetime.datetime.fromtimestamp(start)
        clock = datetime.datetime.strptime(at, "%H:%M:%S" if at.count(":") == 2 else "%H:%M")
        t = day.replace(hour=clock.hour, minute=clock.minute, second=clock.second, microsecond=0).timestamp()
        return t + 86400 if t < start - 60 else t
    return start + float(at)


def cmd_list(args):
    for s in sessions(args.dir):
        m = meta(s)
        events = read_events(s)
        end = events[-1]["t"] if events else m["start"]
        size = sum(f.stat().st_size for f in s.rglob("*") if f.is_file()) / 1e6
        snaps = len(list((s / "snapshots").glob("*.jpg")))
        marks = sum(e["event"] == "mark" for e in events)
        git = m.get("git") or {}
        print(f"{s.name}  {m['start_local']}  {end - m['start']:7.0f} s  {len(events):5d} events  {marks} marks  "
              f"{snaps:3d} snapshots  {size:6.1f} MB  {git.get('branch')}@{(git.get('commit') or '')[:7]}"
              f"{' (dirty)' if git.get('dirty') else ''}  {' '.join(m.get('argv', [])[1:])}")


def _event_detail(e):
    skip = {"t", "local", "event", "robot", "polygon"}
    parts = []
    for k, v in e.items():
        if k in skip:
            continue
        if isinstance(v, float):
            v = round(v, 3)
        parts.append(f"{k}={json.dumps(v) if not isinstance(v, str) else v}")
    if "polygon" in e:
        parts.append(f"polygon={len(e['polygon'] or [])} points")
    return " ".join(parts)


def cmd_events(args):
    session = find_session(args.session, args.dir)
    start = meta(session)["start"]
    kinds = set(args.kind.split(",")) if args.kind else None
    for i, e in enumerate(read_events(session), 1):
        if kinds and e["event"] not in kinds:
            continue
        if args.robot is not None and e.get("robot") not in (None, args.robot):
            continue
        robot = f"R{e['robot']}" if "robot" in e else "  "
        print(f"{i:5d} {rel(e['t'], start)} {e.get('local', '')}  {e['event']:<12} {robot:<3} {_event_detail(e)}")


def cmd_window(args):
    session = find_session(args.session, args.dir)
    start = meta(session)["start"]
    centre = resolve_at(args.at, session, start)
    lo, hi = centre - args.before, centre + args.after
    print(f"{session.name}: {lo - start:.2f} .. {hi - start:.2f} s (centre {centre - start:.2f} s)")
    n = 0
    for tick in read_ticks(session):
        if tick["t"] < lo:
            continue
        if tick["t"] > hi:
            break
        n += 1
        if (n - 1) % args.every:
            continue
        u = tick.get("user")
        head = (f"{rel(tick['t'], start)} {tick.get('nav_mode') or '':<9} {tick.get('mode') or '':<7} "
                f"vision {_fmt(tick.get('msg_age'))}s user {('(' + _fmt(u['x']) + ',' + _fmt(u['y']) + ')') if u else '-'}")
        print(head)
        for rid, out in sorted(tick.get("out", {}).items()):
            if args.robot is not None and int(rid) != args.robot:
                continue
            r = tick.get("robots", {}).get(rid)
            t = tick.get("targets", {}).get(rid)
            est = (tick.get("reckon") or {}).get(rid)
            link = (tick.get("links") or {}).get(rid) or {}
            pose = (f"({_fmt(r['x'])},{_fmt(r['y'])},{_fmt(r.get('heading'), 0)}) age {_fmt(r.get('age'))}"
                    if r else "unseen")
            parts = [f"   R{rid} {pose}"]
            if t:
                parts.append(f"-> ({_fmt(t[0])},{_fmt(t[1])})")
            parts.append("cmd (" + ",".join(_fmt(v) for v in out["cmd"]) + ")")
            if est and est.get("since", 0) > 0.05:
                parts.append(f"est ({_fmt(est['pose']['x'])},{_fmt(est['pose']['y'])}) {est['since']:.1f}s/{est['travelled']:.2f}m")
            if (tick.get("guard") or {}).get(rid):
                parts.append(f"guard={tick['guard'][rid]}")
            if link.get("age") is None or link["age"] > 1.0:
                parts.append(f"link {_fmt(link.get('age'))}s")
            parts.append(f"| {out['reason']}")
            print(" ".join(parts))
    if not n:
        print("(no ticks in this window)")


def cmd_summary(args):
    from control.blackbox import reason_kind
    from control.view_guard import CameraView
    session = find_session(args.session, args.dir)
    m = meta(session)
    start = m["start"]
    events = read_events(session)
    view = None
    for e in events:
        if e["event"] == "view_changed" and e.get("polygon"):
            view = CameraView(e["polygon"])  # the latest view is used for edge distances
    modes, reasons, seen = Counter(), {}, {}
    closest, outside, last_t, first_t, prev_t, n = {}, Counter(), None, None, None, 0
    for tick in read_ticks(session):
        n += 1
        t = tick["t"]
        dt = 0.0 if prev_t is None else min(1.0, t - prev_t)
        prev_t = t
        first_t = first_t or t
        last_t = t
        modes[(tick.get("nav_mode"), tick.get("mode"))] += dt
        for rid, out in tick.get("out", {}).items():
            reasons.setdefault(rid, Counter())[reason_kind(out["reason"])] += dt
            r = tick.get("robots", {}).get(rid)
            ok = r is not None and r.get("heading") is not None and (r.get("age") or 0) <= 0.7
            seen.setdefault(rid, Counter())[ok] += dt
            if ok and view is not None:
                d = view.boundary(r["x"], r["y"])[0]
                if rid not in closest or d < closest[rid][0]:
                    closest[rid] = (d, t)
                if d < 0:
                    outside[rid] += dt
    git = m.get("git") or {}
    print(f"{session.name}  started {m['start_local']}  {git.get('branch')}@{(git.get('commit') or '')[:7]}"
          f"{' (dirty)' if git.get('dirty') else ''}  {' '.join(m.get('argv', [])[1:])}")
    if not n:
        print("no control steps recorded")
        return
    print(f"{n} control steps over {last_t - first_t:.1f} s; {len(events)} events")
    print("time per mode:  " + ", ".join(f"{a or '-'}/{b or '-'} {v:.0f}s" for (a, b), v in modes.most_common()))
    counts = Counter((e["event"], e.get("robot")) for e in events)
    for rid in sorted(reasons):
        total = sum(seen[rid].values()) or 1
        print(f"\nRobot {rid}: located by vision (with a heading) {100 * seen[rid][True] / total:.0f}% of the time; "
              f"lost {counts[('robot_lost', int(rid))]}x, view-guard returns {counts[('view_return', int(rid))]}x, "
              f"gave up (lost outside view) {counts[('view_lost', int(rid))]}x")
        if rid in closest:
            d, t = closest[rid]
            print(f"  closest to the camera-view edge while seen: {d:+.2f} m at {t - start:.1f} s"
                  f"{f'; outside the view for {outside[rid]:.1f} s' if outside[rid] else ''}")
        print("  time per reason:")
        for kind, v in reasons[rid].most_common(8):
            print(f"    {v:7.1f} s  {kind}")
    marks = [e for e in events if e["event"] == "mark"]
    if marks:
        print("\nMarks:")
        for e in marks:
            print(f"  {e['t'] - start:8.2f} s {e.get('local')}  {e.get('note') or ''}")
    for k in ("vision_lost", "user_lost"):
        if counts[(k, None)]:
            print(f"{k.replace('_', ' ')}: {counts[(k, None)]}x")
    snaps = sorted((session / "snapshots").glob("*.jpg"))
    if snaps:
        print(f"\n{len(snaps)} snapshots in {session / 'snapshots'}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", default=str(DEFAULT_DIR), help="black-box folder (default logs/blackbox)")
    sub = ap.add_subparsers(dest="command", required=True)
    sub.add_parser("list")
    for name in ("summary", "events", "window"):
        p = sub.add_parser(name)
        p.add_argument("session", nargs="?")
        p.add_argument("--robot", type=int)
        if name == "events":
            p.add_argument("--kind", help="comma-separated event kinds, e.g. robot_lost,view_return,mark")
        if name == "window":
            p.add_argument("--at", required=True)
            p.add_argument("--before", type=float, default=3.0)
            p.add_argument("--after", type=float, default=3.0)
            p.add_argument("--every", type=int, default=1, help="print every Nth step (20 per second)")
    args = ap.parse_args(argv)
    {"list": cmd_list, "summary": cmd_summary, "events": cmd_events, "window": cmd_window}[args.command](args)


if __name__ == "__main__":
    main()
