"""Robot head dashboard /api/home against a simulated LX-224 servo bus (no hardware, no pyserial needed)."""
import importlib
import json
import sys
import threading
import types
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

import pytest


class FakeBus:
    """Two LX-224 servos: answers position (28) and torque (32) reads, executes move-wait (7) on start (11)."""
    def __init__(self, positions, reach=True):
        self.pos, self.pending, self.torque, self.reach, self.out = list(positions), {}, [0, 0], reach, bytearray()
        self.moves = []

    def __call__(self, *args, **kwargs):  # serial.Serial(...)
        return self

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def close(self):
        pass

    def flush(self):
        pass

    def reset_input_buffer(self):
        self.out.clear()

    @property
    def in_waiting(self):
        return len(self.out)

    def read(self, n):
        data, self.out[:] = bytes(self.out[:n]), self.out[n:]
        return data

    def _reply(self, sid, cmd, payload):
        body = [sid, len(payload) + 3, cmd, *payload]
        self.out.extend(bytes([0x55, 0x55, *body, (~sum(body)) & 255]))

    def write(self, packet):
        sid, cmd, args = packet[2], packet[4], packet[5:-1]
        if cmd == 28:
            self._reply(sid, 28, list(int(self.pos[sid - 1]).to_bytes(2, "little", signed=True)))
        elif cmd == 32:
            self._reply(sid, 32, [self.torque[sid - 1]])
        elif cmd == 7:
            self.pending[sid] = args[0] | args[1] << 8
            self.moves.append((sid, self.pending[sid], args[2] | args[3] << 8))
        elif cmd == 11:
            for s, v in self.pending.items():
                if self.reach:
                    self.pos[s - 1] = v
                self.torque[s - 1] = 1  # a move loads a released servo
            self.pending.clear()


@pytest.fixture
def robot(monkeypatch):
    monkeypatch.setitem(sys.modules, "serial", types.SimpleNamespace(Serial=None))
    dash = importlib.import_module("head_tracker.dashboard")
    servers = []

    def start(bus):
        monkeypatch.setattr(dash.serial, "Serial", bus)
        dash.state.update(controlling=False, running=False)
        server = ThreadingHTTPServer(("127.0.0.1", 0), dash.Handler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        servers.append(server)
        return f"http://127.0.0.1:{server.server_port}", dash

    yield start
    for s in servers:
        s.shutdown()
        s.server_close()


def home(base, data):
    request = urllib.request.Request(base + "/api/home", data=json.dumps(data).encode(), method="POST",
                                     headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(request, timeout=10) as response:
        return response.status


def test_home_moves_both_servos_in_one_timed_move_and_reports_it(robot):
    bus = FakeBus([600, 820])
    base, dash = robot(bus)
    assert home(base, {"positions": [688, 692], "duration": 0.4}) == 204
    assert bus.moves == [(1, 688, 400), (2, 692, 400)]
    assert dash.state["positions"] == [688, 692] and dash.state["torque"] == [1, 1]
    assert dash.state["status"].startswith("Homed") and dash.state["controlling"] is False
    assert dash.stop.is_set()  # any head tracking / sweep was told to stop first


def test_home_rejects_bad_requests_and_reports_a_missed_pose(robot):
    base, dash = robot(FakeBus([600, 820], reach=False))
    for bad in ({"positions": [688]}, {"positions": [688, 1200]}, {"positions": [688, 692], "duration": 9}):
        with pytest.raises(urllib.error.HTTPError) as error:
            home(base, bad)
        assert error.value.code == 400
    with pytest.raises(urllib.error.HTTPError) as error:
        home(base, {"positions": [688, 692], "duration": 0.3})
    assert error.value.code == 503 and "not reached" in dash.state["status"]


def test_observation_records_its_source_and_the_last_head_camera_face(robot):
    import time
    base, dash = robot(FakeBus([600, 820]))
    dash.state["tracking"] = {"active": True, "status": "x", "center": None, "targets": None}

    def observe(center, source):
        body = json.dumps({"center": center, "frameTime": time.time(), "target": "face", "source": source}).encode()
        request = urllib.request.Request(base + "/api/tracking/observation", data=body, method="POST",
                                         headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(request, timeout=5) as response:
            return response.status

    assert observe([0.5, 0.5], "head camera") == 204
    face_time = dash.state["tracking"]["faceTime"]
    assert dash.tracking_observation["source"] == "head camera"
    time.sleep(0.01)
    assert observe([0.4, 0.5], "main camera") == 204 and observe(None, "prediction") == 204
    assert dash.tracking_observation["source"] == "prediction"
    assert dash.state["tracking"]["faceTime"] == face_time  # only a head-camera face moves it
    assert observe(None, "made up") == 204 and dash.tracking_observation["source"] is None
