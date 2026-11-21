"""Robot-side drive server: body velocities from the laptop over UDP -> wheel commands, with a watchdog.

In the MVP the laptop closes the position loop (it sees every robot through the birds-eye camera) and
sends each robot a small velocity command about 10 times a second. This program turns that command into
mecanum wheel speeds and writes them to the Hiwonder motor board. If commands stop arriving for
`--timeout` seconds (laptop crashed, WiFi dropped, Ctrl+C), it stops the wheels.

Run it on the robot Pi with Nate's robot environment, because it reuses his Hiwonder I2C driver and the
same wheel mapping config his keyboard teleop uses:

    cd ~/capstone-base
    .venv/bin/python ~/drive_server.py --robot-id 1                 # simulation: prints motor commands
    .venv/bin/python ~/drive_server.py --robot-id 1 --hardware      # real motors (lift the wheels first!)

Protocol: UDP JSON on port 5700 (docs/NETWORK.md reserves 5700-5799 for robot commands).
    laptop -> robot   {"type": "drive", "robot_id": 1, "seq": 17, "vx": 0.10, "vy": 0.0, "wz": 0.2}
                      {"type": "manual", "robot_id": 1, "vx": 0.05, "vy": 0.0, "wz": 0.0}
                      {"type": "stop",  "robot_id": 1}
    robot -> laptop   {"type": "status", "robot_id": 1, "seq": 17, "state": "driving",
                       "motors": [5, -5, -5, 5], "error": null}      (sent back to whoever sent the command)
    any -> robot      {"type": "status"}     read-only query (the system GUI): changes nothing, accepted from
                                             any host and for any robot_id; the reply adds "hardware",
                                             "uptime_s", "last_cmd_age_s", "target", "received", "rejected"
vx = forward m/s, vy = left m/s, wz = counter-clockwise rad/s, all in the robot's own (body) frame.
"Forward" is the robot's front, which must be the same face the vision system calls the front.

Manual commands use the same source-IP restriction, acceleration and watchdog; they explicitly
do not require a camera pose. Autonomous drive commands retain their existing pose requirement.

Only the Python standard library is needed, plus capstone_base (Nate's repo) with --hardware.
"""

import argparse
import collections
from contextlib import ExitStack
import json
import math
import os
import signal
import socket
import time

# Fields of Nate's HiwonderConfig that we pass through from his JSON config.
HIWONDER_FIELDS = ("motor_type", "encoder_polarity", "wheel_channels", "motor_signs", "encoder_signs",
                   "counts_per_wheel_rev", "max_speed_counts", "bus_number", "address")


class DriveParams:
    """How body velocities become wheel commands.

    counts_per_meter: encoder counts per meter of wheel travel. The default is an estimate: Hiwonder
        motor type 3 (JGB37-520, 44 counts x 90:1 = 3960 counts per wheel turn) on a ~97 mm wheel.
        Measure the real value with `python -m control.drive_test` on the laptop.
    turn_radius_m: (wheelbase + track width) / 2, i.e. how far each wheel effectively sits from the
        center when the robot spins in place.
    """

    def __init__(self, wheel_channels, motor_signs, counts_per_meter=13000.0, turn_radius_m=0.2,
                 max_counts=25, accel_counts_per_s=40.0, disabled_channels=()):
        if sorted(wheel_channels) != [1, 2, 3, 4]:
            raise ValueError("wheel_channels must be a permutation of 1, 2, 3, 4 (FL, FR, RL, RR order)")
        if any(s not in (-1, 1) for s in motor_signs) or len(motor_signs) != 4:
            raise ValueError("motor_signs must be four values of +1 or -1")
        disabled_channels = tuple(disabled_channels)
        if len(disabled_channels) > 1 or any(type(c) is not int or c not in (1, 2, 3, 4) for c in disabled_channels):
            raise ValueError("disabled_channels must be empty or contain one channel from 1 to 4")
        self.disabled_channels = disabled_channels
        self.wheel_channels = list(wheel_channels)
        self.motor_signs = list(motor_signs)
        self.counts_per_meter = float(counts_per_meter)
        self.turn_radius_m = float(turn_radius_m)
        self.max_counts = min(int(max_counts), 10) if disabled_channels else int(max_counts)
        self.accel_counts_per_s = float(accel_counts_per_s)


def wheel_counts(vx, vy, wz, p):
    """Body velocity -> FL, FR, RL, RR wheel speeds in encoder counts per 10 ms (floats).

    X-layout mecanum mixing, the same pattern as Nate's kinematics.py and teleop.py. If any wheel would
    exceed max_counts, all wheels are scaled down together so the direction of motion is kept.
    """
    k = p.turn_radius_m * wz
    speeds = (vx - vy - k, vx + vy + k, vx + vy - k, vx - vy + k)  # wheel surface speed, m/s
    counts = [s * p.counts_per_meter / 100.0 for s in speeds]  # per 10 ms
    # An unpowered wheel is free to roll; command only the remaining wheel constraints.
    counts = [0.0 if channel in p.disabled_channels else value
              for value, channel in zip(counts, p.wheel_channels)]
    peak = max(abs(c) for c in counts)
    if peak > p.max_counts:
        counts = [c * p.max_counts / peak for c in counts]
    return counts


def to_motors(wheels, p):
    """FL, FR, RL, RR floats -> M1..M4 signed integers, using the board's channel mapping and signs."""
    motors = [0, 0, 0, 0]
    for value, channel, sign in zip(wheels, p.wheel_channels, p.motor_signs):
        if channel not in p.disabled_channels:
            motors[channel - 1] = int(round(value)) * sign
    return motors


def parse_command(data, robot_id, *, require_compensated_pose=False):
    """Decode one datagram. Returns ("drive", (vx, vy, wz), seq), ("stop", None, seq) or ("status", None, seq);
    raises ValueError."""
    msg = json.loads(data)
    if not isinstance(msg, dict):
        raise ValueError("expected a JSON object")
    kind = msg.get("type")
    seq = msg.get("seq")
    if kind == "status":  # read-only query: any robot_id (the reply says which robot this is)
        return "status", None, seq
    if msg.get("robot_id") != robot_id:
        raise ValueError(f"command for robot {msg.get('robot_id')!r}, this is robot {robot_id}")
    if kind == "stop":
        return "stop", None, seq
    if kind not in ("drive", "manual"):
        raise ValueError(f"unknown command type {kind!r}")
    if kind == "drive" and require_compensated_pose and msg.get("pose_compensated") is not True:
        raise ValueError("drive command requires a compensated pose")
    v = []
    for name in ("vx", "vy", "wz"):
        x = msg.get(name)
        if isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x):
            raise ValueError(f"{name} must be a finite number")
        v.append(float(x))
    return kind, tuple(v), seq


class DriveServer:
    """Receives commands, ramps wheel speeds, enforces the command timeout. One instance owns the motors."""

    def __init__(self, motors, params, robot_id, port=5700, host="0.0.0.0", timeout_s=0.5, rate_hz=20.0,
                 allow_from=None, max_speed_m_s=0.3, max_yaw_rad_s=1.0, clock=time.monotonic, log=print,
                 *, before_motor_tick=None, require_compensated_pose=False):
        self.motors, self.p, self.robot_id = motors, params, robot_id
        self.timeout_s, self.period = timeout_s, 1.0 / rate_hz
        self.allow_from = allow_from
        self.max_speed_m_s, self.max_yaw_rad_s = max_speed_m_s, max_yaw_rad_s
        self.clock, self.log = clock, log
        self.before_motor_tick = before_motor_tick
        self.require_compensated_pose = require_compensated_pose
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.port = self.sock.getsockname()[1]
        self.target = (0.0, 0.0, 0.0)
        self.wheels = [0.0, 0.0, 0.0, 0.0]
        self.last_cmd = None  # clock time of the last accepted drive command
        self.last_motors = None
        self.last_tick = None
        self.state = "stopped"
        self.error = None
        self.received = self.rejected = 0
        self.started = clock()

    def _clamp(self, vx, vy, wz):
        speed = math.hypot(vx, vy)
        if speed > self.max_speed_m_s:
            vx, vy = vx * self.max_speed_m_s / speed, vy * self.max_speed_m_s / speed
        wz = max(-self.max_yaw_rad_s, min(self.max_yaw_rad_s, wz))
        return vx, vy, wz

    def _status_reply(self, seq, now):
        """Answer to a read-only status query: the normal reply plus details. Changes nothing."""
        return {"type": "status", "robot_id": self.robot_id, "seq": seq, "state": self.state,
                "motors": self.last_motors, "error": self.error or getattr(self.motors,"fault",None),
                "hardware": not isinstance(self.motors, PrintMotors),
                "uptime_s": round(now - self.started, 1),
                "last_cmd_age_s": None if self.last_cmd is None else round(now - self.last_cmd, 2),
                "target": [round(v, 3) for v in self.target], "received": self.received,
                "rejected": self.rejected, "timeout_s": self.timeout_s,
                "require_compensated_pose": self.require_compensated_pose,
                "manual_control": True,
                "mixed_drive": self.motors.telemetry() if hasattr(self.motors,"telemetry") else None,
                "disabled_channels": list(self.p.disabled_channels),
                "max_counts": self.p.max_counts}

    def handle(self, data, addr, now):
        """Process one datagram (also used directly by tests)."""
        seq = parse_error = None
        try:
            kind, v, seq = parse_command(
                data, self.robot_id, require_compensated_pose=self.require_compensated_pose
            )
        except (ValueError, TypeError, UnicodeError) as e:
            kind, parse_error = None, e
        if kind == "status":  # read-only: never counted, never touches the target, the error or the watchdog
            try:
                self.sock.sendto(json.dumps(self._status_reply(seq, now)).encode(), addr)
            except OSError:
                pass
            return
        self.received += 1
        try:
            if self.allow_from and addr[0] != self.allow_from:
                raise ValueError(f"ignoring commands from {addr[0]}")
            if parse_error is not None:
                raise parse_error
            if kind == "stop":
                self.stop_now("stopped (stop command)")
            else:
                self.target = self._clamp(*v)
                self.last_cmd = now
                self.state = "driving (keyboard)" if kind == "manual" else "driving"
            self.error = None
        except (ValueError, TypeError, UnicodeError) as e:
            self.rejected += 1
            self.error = str(e)
        reply = {"type": "status", "robot_id": self.robot_id, "seq": seq, "state": self.state,
                 "motors": self.last_motors, "error": self.error or getattr(self.motors,"fault",None), "manual_control": True,
                 "mixed_drive": self.motors.telemetry() if hasattr(self.motors,"telemetry") else None}
        try:
            self.sock.sendto(json.dumps(reply).encode(), addr)
        except OSError:
            pass

    def stop_now(self, state="stopped"):
        self.target = (0.0, 0.0, 0.0)
        self.wheels = [0.0, 0.0, 0.0, 0.0]
        self.last_cmd = None
        self.state = state

    def tick(self, now):
        """One control step: watchdog, ramp toward the commanded wheel speeds, write the motors."""
        if self.before_motor_tick is not None:
            self.before_motor_tick()
        dt = self.period if self.last_tick is None else min(max(now - self.last_tick, 0.0), 0.1)
        self.last_tick = now
        if self.last_cmd is not None and now - self.last_cmd > self.timeout_s:
            self.stop_now("stopped (no commands for %.1f s)" % self.timeout_s)
            self.log("Command timeout: wheels stopped")
        if self.last_cmd is None:
            self.wheels = [0.0, 0.0, 0.0, 0.0]  # stopping is immediate, never ramped
        else:
            goal = wheel_counts(*self.target, self.p)
            diffs = [g - w for g, w in zip(goal, self.wheels)]
            peak = max(abs(d) for d in diffs)
            scale = min(1.0, self.p.accel_counts_per_s * dt / peak) if peak else 1.0
            self.wheels = [w + d * scale for w, d in zip(self.wheels, diffs)]
        motors = to_motors(self.wheels, self.p)
        if motors == [0, 0, 0, 0]:
            self.motors.stop()
        else:
            self.motors.set_motor_speed_counts(tuple(motors))
        motors = list(getattr(self.motors,"last_values",motors))
        if getattr(self.motors,"fault",None):
            self.state = "stopped (motor fault)"
        if motors != self.last_motors:
            self.log(f"M1..M4 counts/10ms: {motors}   ({self.state})")
        self.last_motors = motors

    def run(self):
        self.motors.stop()
        self.log(f"Robot {self.robot_id}: listening for drive commands on UDP {self.port}. Ctrl+C to stop.")
        next_tick = self.clock()
        try:
            while True:
                now = self.clock()
                if now >= next_tick:  # the tick always runs on time, however many packets arrive
                    self.tick(now)
                    next_tick = now + self.period
                    continue
                self.sock.settimeout(next_tick - now)
                try:
                    data, addr = self.sock.recvfrom(4096)
                except (socket.timeout, BlockingIOError, ConnectionResetError):
                    continue  # (Windows reports ICMP "port unreachable" as ConnectionResetError)
                self.handle(data, addr, self.clock())
        finally:
            try:
                self.motors.stop()
            finally:
                self.sock.close()


class PrintMotors:
    """Simulation backend: records commands, touches no hardware."""

    def __init__(self):
        self.values = (0, 0, 0, 0)
        self.history = collections.deque(maxlen=1000)

    def set_motor_speed_counts(self, values):
        self.values = tuple(values)
        self.history.append(self.values)

    def stop(self):
        self.set_motor_speed_counts((0, 0, 0, 0))


def load_hiwonder_json(path):
    with open(path) as f:
        data = json.load(f)
    if "hiwonder" in data:  # robot.local.json style: board settings nested under "hiwonder"
        data = data["hiwonder"]
    return {k: data[k] for k in HIWONDER_FIELDS if k in data}


def _interrupt(signum, frame):
    raise KeyboardInterrupt


def main(argv=None, *, before_motor_tick=None):
    ap = argparse.ArgumentParser(description="UDP velocity commands -> mecanum wheels, with a watchdog")
    ap.add_argument("--robot-id", type=int, required=True, help="only accept commands addressed to this robot")
    ap.add_argument("--hardware", action="store_true", help="drive the real motors (default: simulation)")
    ap.add_argument("--hiwonder-config", default=os.path.expanduser("~/capstone-base/config/teleop.json"),
                    help="Nate's board/wheel-mapping JSON (the one keyboard teleop drives with)")
    ap.add_argument("--require-compensated-pose", action="store_true",
                    help="accept drive commands only from compensated vision; dashboard owns UART")
    ap.add_argument("--port", type=int, default=5700)
    ap.add_argument("--allow-from", help="only accept commands from this IP (the laptop)")
    ap.add_argument("--counts-per-meter", type=float, default=13000.0,
                    help="encoder counts per meter of wheel travel (measure with control.drive_test)")
    ap.add_argument("--turn-radius", type=float, default=0.2, help="(wheelbase + track) / 2 in meters")
    ap.add_argument("--max-counts", type=int, default=70, help="wheel speed cap in counts/10ms")
    ap.add_argument("--max-speed", type=float, default=1.0, help="body speed clamp in m/s")
    ap.add_argument("--accel", type=float, default=100.0, help="wheel acceleration in counts/10ms per second")
    ap.add_argument("--timeout", type=float, default=0.5, help="stop if no command for this many seconds")
    args = ap.parse_args(argv)

    if args.require_compensated_pose and (not args.allow_from or before_motor_tick is not None):
        ap.error("--require-compensated-pose needs --allow-from and the standalone drive server")

    hw = load_hiwonder_json(args.hiwonder_config)
    with open(args.hiwonder_config) as f:
        drive_config = json.load(f)
        disabled_channels = drive_config.get("disabled_channels", [])
    mixed_config = drive_config.get('tb6612')
    mixed_profile = None
    if mixed_config:
        path = mixed_config['calibration']
        if not os.path.isabs(path):
            path = os.path.join(os.path.dirname(os.path.abspath(args.hiwonder_config)),path)
        with open(path) as saved:
            mixed_profile = json.load(saved)['tb6612_calibration']
        if disabled_channels:
            ap.error('Mixed four-wheel config must not disable logical channels')
        if mixed_config.get('logical_channel') != 1:
            ap.error('Mixed driver only replaces logical M1')
    if hw.get("wheel_channels") is None or hw.get("motor_signs") is None:
        ap.error(f"{args.hiwonder_config} needs wheel_channels and motor_signs")
    cap = min(args.max_counts, int(hw.get("max_speed_counts", args.max_counts)))
    if mixed_profile:
        cap = min(cap, math.floor(mixed_profile['maximum_count_rate'] / mixed_profile['reference_counts_per_command'] + 1e-9))
    params = DriveParams(hw["wheel_channels"], hw["motor_signs"], args.counts_per_meter, args.turn_radius,
                         cap, args.accel, disabled_channels=disabled_channels)
    cap = params.max_counts
    print(f"{'HARDWARE' if args.hardware else 'SIMULATION (no I2C)'}: robot {args.robot_id}, "
          f"FL/FR/RL/RR channels {params.wheel_channels}, signs {params.motor_signs}, "
          f"{params.counts_per_meter:g} counts/m, turn radius {params.turn_radius_m:g} m, cap {cap} counts/10ms")

    if params.disabled_channels:
        print(f"Experimental three-wheel mode: disabled channels {params.disabled_channels}; "
              "faulty motor must be physically disconnected")

    for sig in (signal.SIGINT, signal.SIGTERM, getattr(signal, "SIGHUP", None)):
        if sig is not None:
            signal.signal(sig, _interrupt)
    try:
        if args.hardware:
            from capstone_base.hiwonder_i2c import HiwonderConfig, HiwonderController
            with ExitStack() as stack:
                board = stack.enter_context(HiwonderController(HiwonderConfig(**hw)))
                board.initialize()
                backend = board
                if mixed_config:
                    try:
                        from .mixed_motors import MixedMotors, TB6612Motor
                    except ImportError:
                        from mixed_motors import MixedMotors, TB6612Motor
                    tb = TB6612Motor(mixed_profile,mixed_config['pins'])
                    stack.callback(tb.close)
                    backend = MixedMotors(board,tb,mixed_profile,direction=mixed_config.get('direction',1))
                    stack.callback(backend.stop)
                DriveServer(backend, params, args.robot_id, args.port, timeout_s=args.timeout,
                            allow_from=args.allow_from, before_motor_tick=before_motor_tick,
                            max_speed_m_s=args.max_speed,
                            require_compensated_pose=args.require_compensated_pose).run()
        else:
            DriveServer(PrintMotors(), params, args.robot_id, args.port, timeout_s=args.timeout,
                        allow_from=args.allow_from, max_speed_m_s=args.max_speed,
                        require_compensated_pose=args.require_compensated_pose).run()
    except KeyboardInterrupt:
        print("\nStopped.")
    except (OSError, RuntimeError, ValueError) as e:  # e.g. I2C failure: the driver has already tried to stop
        print(f"\nDrive server failed: {e}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
