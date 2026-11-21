"""UDP plumbing: the vision output coming in, drive commands going out to the robots.

Everything uses the laptop's own clock. The vision message's `t` is the master Pi's capture time, which
need not be synchronized with the laptop, so freshness is judged by when a message *arrived* here plus
the per-object `age` field (a time difference, so clock offsets don't matter).
"""

import json
import socket
import time


class VisionListener:
    """Receives the vision system's UDP JSON (vision.main output) and keeps only the newest message."""

    def __init__(self, port=5005, host="127.0.0.1"):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.sock.setblocking(False)
        self.msg = None
        self.received_at = None  # laptop time.time() when self.msg arrived

    def poll(self):
        """Read everything waiting. Returns (message, received_at) for the newest one, or (None, None)."""
        while True:
            try:
                data, _ = self.sock.recvfrom(65536)
            except (BlockingIOError, ConnectionResetError):
                break
            try:
                self.msg, self.received_at = json.loads(data), time.time()
            except ValueError:
                pass
        return self.msg, self.received_at

    def close(self):
        self.sock.close()


class RobotLink:
    """Sends drive/stop commands to every robot's drive server and collects their status replies."""

    def __init__(self, robots):
        """robots: {robot_id: (host, port)}"""
        self.addrs = {int(rid): (h, int(p)) for rid, (h, p) in robots.items()}
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 0))  # Windows can't recvfrom() an unbound socket (dry run never sends)
        self.sock.setblocking(False)
        self.seq = 0
        self.status = {}       # robot_id -> last status reply
        self.status_at = {}    # robot_id -> time.time() it arrived

    def _send(self, rid, msg):
        self.seq += 1
        msg = {"robot_id": rid, "seq": self.seq, **msg}
        try:
            self.sock.sendto(json.dumps(msg).encode(), self.addrs[rid])
        except OSError as e:  # network down / unreachable: keep going, the robot's watchdog stops it
            self.status[rid] = {"state": "send failed", "error": str(e)}

    def drive(self, rid, vx, vy, wz, *, pose_compensated=False):
        msg = {"type": "drive", "vx": round(vx, 4), "vy": round(vy, 4), "wz": round(wz, 4)}
        if pose_compensated:
            msg["pose_compensated"] = True
        self._send(rid, msg)

    def manual(self, rid, vx, vy, wz=0):
        self._send(rid, {'type':'manual','vx':round(vx,4),'vy':round(vy,4),'wz':round(wz,4)})

    def stop(self, rid):
        self._send(rid, {"type": "stop"})

    def stop_all(self, repeats=3):
        for _ in range(repeats):  # UDP can drop a packet; the robots' watchdogs are the backstop
            for rid in self.addrs:
                self.stop(rid)
            time.sleep(0.02)

    def poll(self):
        """Collect status replies. Returns {robot_id: seconds since its last reply (or None)}."""
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
            except (BlockingIOError, ConnectionResetError):
                break
            try:
                msg = json.loads(data)
                rid = int(msg["robot_id"])
            except (ValueError, KeyError, TypeError):
                continue
            self.status[rid], self.status_at[rid] = msg, time.time()
        now = time.time()
        return {rid: (now - self.status_at[rid]) if rid in self.status_at else None for rid in self.addrs}

    def close(self):
        self.sock.close()
