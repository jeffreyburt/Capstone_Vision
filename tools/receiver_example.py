"""Minimal receiver for the vision output: a starting point for the controller code.

    python tools/receiver_example.py            # listens on UDP port 5005

Run it on the controller laptop. The Pi's config.yaml output.host must be this laptop's IP
(or use broadcast). No dependencies beyond the Python standard library: copy this file freely.
"""

import argparse
import json
import socket


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5005)
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", args.port))
    print(f"Listening on UDP port {args.port} ...")
    while True:
        data, addr = sock.recvfrom(65536)
        msg = json.loads(data)
        robots = "  ".join(
            f"R{r['id']}: ({r['x']:.2f}, {r['y']:.2f}) m, {r['heading'] if r['heading'] is not None else '?'} deg, age {r['age']:.2f}s"
            for r in msg["robots"]) or "no robots"
        user = msg.get("user")
        user = f"user: ({user['x']:.2f}, {user['y']:.2f})" if user else "user: -"
        print(f"[{addr[0]} frame {msg['frame']}] {robots}   {user}")


if __name__ == "__main__":
    main()
