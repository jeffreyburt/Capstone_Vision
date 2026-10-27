"""Sending positions to the controller laptop as JSON over UDP.

Message format (one UDP packet per update):
{
  "t": 1728170000.123,          # time of the frame (seconds since epoch)
  "frame": 4512,
  "robots": [{"id": 1, "x": 1.42, "y": 3.10, "heading": 87.5, "vx": 0.0, "vy": 0.1, "age": 0.05}, ...],
  "user": null                  # person detection comes in a later phase: {"x", "y", "age", "confidence"}
}
Distances in meters, heading in degrees CCW from world +x, age = seconds since last seen.
"""

import json
import socket


def make_message(t, frame, robots, user=None):
    def r(v, n):
        return None if v is None else round(float(v), n)

    return {
        "t": round(t, 3),
        "frame": frame,
        "robots": [{"id": b["id"], "x": r(b["x"], 3), "y": r(b["y"], 3), "heading": r(b.get("heading"), 1),
                    "vx": r(b.get("vx", 0.0), 3), "vy": r(b.get("vy", 0.0), 3), "age": r(b["age"], 3)}
                   for b in robots],
        "user": user,
    }


class UdpPublisher:
    def __init__(self, host="127.0.0.1", port=5005, broadcast=False):
        self.addr = (host, int(port))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if broadcast:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    def send(self, message):
        try:
            self.sock.sendto(json.dumps(message).encode(), self.addr)
        except OSError as e:  # e.g. network down; keep running and try again next time
            print(f"UDP send failed: {e}")

    def close(self):
        self.sock.close()
