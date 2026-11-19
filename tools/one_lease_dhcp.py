"""Find a Pi plugged straight into the laptop's Ethernet port, and give it an address.

A Pi on a direct cable gets no IPv4 address (nothing hands out DHCP there), and the laptop's wired port has
IPv6 off, so `ssh user@name.local` doesn't work. The Pi does broadcast DHCP requests with its MAC and
hostname, so:

    python tools/one_lease_dhcp.py --listen                       # 20 s: print who is asking (MAC, hostname)
    python tools/one_lease_dhcp.py e4:5f:01:02:10:7f 192.168.1.50 # answer ONLY that MAC with that address

The server answers one MAC only, hands out no gateway or DNS, binds to the laptop's wired address
(--server, default 192.168.1.10) so nothing leaves on another network, and exits after --minutes
(default 15). The Pi keeps the 12 h lease after that. Used 2026-11-19: robot 2 -> .50, robot 1 -> .51.
"""

import argparse
import socket
import struct
import time

COOKIE = b"\x63\x82\x53\x63"


def options(data):
    out, i = {}, 240
    while i < len(data) and data[i] != 255:
        if data[i] == 0:
            i += 1
            continue
        out[data[i]] = data[i + 2:i + 2 + data[i + 1]]
        i += 2 + data[i + 1]
    return out


def open_socket(server_ip):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.bind((server_ip, 67))
    s.settimeout(1.0)
    return s


def requests(s, seconds):
    """Yield (mac, options, packet) for DHCP client requests seen within `seconds`."""
    end = time.time() + seconds
    while time.time() < end:
        try:
            data, _ = s.recvfrom(2048)
        except (socket.timeout, ConnectionResetError):
            continue
        if len(data) >= 240 and data[0] == 1 and data[236:240] == COOKIE:
            yield data[28:34].hex(":"), options(data), data


def listen(s, seconds):
    seen = set()
    for mac, opts, _ in requests(s, seconds):
        host = opts.get(12, b"").decode(errors="replace") or None
        if (mac, host) not in seen:
            seen.add((mac, host))
            print(f"DHCP request from {mac}  hostname {host}", flush=True)
    print(f"done: {len(seen)} device(s). b8:27:eb = Pi 3, e4:5f:01 / dc:a6:32 = Pi 4")


def serve(s, mac, client_ip, server_ip, seconds):
    print(f"Serving {client_ip} to {mac} only, from {server_ip}, for {seconds / 60:g} min", flush=True)
    for req_mac, opts, data in requests(s, seconds):
        reply = {1: 2, 3: 5}.get(opts.get(53, b"\x00")[0])  # DISCOVER -> OFFER, REQUEST -> ACK
        if req_mac != mac or reply is None:
            continue
        pkt = bytearray(240)
        pkt[0:4] = bytes([2, 1, 6, 0])
        pkt[4:8], pkt[10:12] = data[4:8], data[10:12]        # xid, flags
        pkt[16:20] = socket.inet_aton(client_ip)               # yiaddr
        pkt[20:24] = socket.inet_aton(server_ip)               # siaddr
        pkt[28:44] = data[28:44]                               # chaddr
        pkt[236:240] = COOKIE
        pkt += bytes([53, 1, reply, 54, 4]) + socket.inet_aton(server_ip)
        pkt += bytes([51, 4]) + struct.pack("!I", 12 * 3600)
        pkt += bytes([1, 4]) + socket.inet_aton("255.255.255.0") + b"\xff"
        s.sendto(bytes(pkt), ("255.255.255.255", 68))
        print(time.strftime("%H:%M:%S"), {2: "OFFER", 5: "ACK"}[reply], client_ip, "->", mac, flush=True)
    print("Stopped.")


def main():
    ap = argparse.ArgumentParser(description="Find / address a Pi on a direct Ethernet cable")
    ap.add_argument("mac", nargs="?", help="the Pi's MAC (from --listen)")
    ap.add_argument("ip", nargs="?", help="address to give it, e.g. 192.168.1.50")
    ap.add_argument("--listen", action="store_true", help="only print DHCP requests for 20 s")
    ap.add_argument("--server", default="192.168.1.10", help="the laptop's wired address")
    ap.add_argument("--minutes", type=float, default=15.0)
    args = ap.parse_args()
    s = open_socket(args.server)
    if args.listen:
        listen(s, 20)
    elif args.mac and args.ip:
        serve(s, args.mac.lower(), args.ip, args.server, args.minutes * 60)
    else:
        ap.error("give --listen, or a MAC and an IP")


if __name__ == "__main__":
    main()
