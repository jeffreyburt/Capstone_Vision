"""Wire format: one ZeroMQ message per frame.

    b"CS1" | header length (uint32, little-endian) | JSON header | JPEG bytes

Header fields:
    camera     stream name ("birdseye", "robot1", ...)
    seq        frame counter from the sender (gaps = dropped frames)
    t_capture  when the frame was captured (sender's clock, seconds since epoch)
    t_sent     when it was sent (sender's clock)
    width, height, encoding ("jpeg")
    meta       free-form dict set by the sender's owner, e.g. {"pan_deg": 12.5, "tilt_deg": -3.0}
"""

import json
import struct

MAGIC = b"CS1"
VERSION = 1


def pack(header, payload):
    h = json.dumps(header, separators=(",", ":")).encode()
    return MAGIC + struct.pack("<I", len(h)) + h + payload


def unpack(msg):
    """bytes -> (header dict, payload bytes). Raises ValueError for anything that isn't a camstream frame."""
    if len(msg) < 7 or msg[:3] != MAGIC:
        raise ValueError("not a camstream message")
    (n,) = struct.unpack("<I", msg[3:7])
    header = json.loads(msg[7:7 + n])
    return header, msg[7 + n:]
