"""Run the hardware drive server while verifying that the head holds neutral."""

import argparse
import sys
import threading
import time

from robot import drive_server


class NeutralMonitor:
    """Latch feedback faults without blocking the wheel control loop on UART reads."""

    def __init__(self, port, read, neutral, tolerance):
        self.port = port
        self.read = read
        self.neutral = neutral
        self.tolerance = tolerance
        self.last_check = None
        self.fault = None
        self.done = threading.Event()
        self.thread = threading.Thread(target=self.poll)

    def sample(self):
        for servo, target in enumerate(self.neutral, 1):
            position = self.read(self.port, servo, 28)
            torque = self.read(self.port, servo, 32)
            if position is None or not 0 <= position <= 1000 or torque != 1:
                raise RuntimeError(f"Servo {servo} requires valid feedback and engaged torque")
            if abs(position - target) > self.tolerance:
                raise RuntimeError(f"Servo {servo} is at {position}, outside neutral tolerance of {target}")
        self.last_check = time.monotonic()

    def poll(self):
        while not self.done.wait(0.2):
            try:
                self.sample()
            except (OSError, RuntimeError) as error:
                self.fault = str(error)
                return

    def check(self):
        if self.fault is not None:
            raise RuntimeError(self.fault)
        if self.last_check is None or time.monotonic() - self.last_check > 0.8:
            raise RuntimeError("Neutral servo feedback is stale")

    def __enter__(self):
        self.sample()
        self.thread.start()
        return self

    def __exit__(self, *_):
        self.done.set()
        self.thread.join()


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Hardware drive with neutral servo feedback. Pass drive server options after --."
    )
    parser.add_argument("--neutral", required=True, type=int, nargs=2, metavar=("PAN", "TILT"))
    parser.add_argument("--tolerance", type=int, default=5, help="Allowed error in servo units (default: 5)")
    parser.add_argument("drive_options", nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    if not all(0 <= position <= 1000 for position in args.neutral) or not 1 <= args.tolerance <= 10:
        parser.error("neutral positions must be 0–1000 and tolerance must be 1–10")
    if args.drive_options[:1] != ["--"]:
        parser.error("pass drive server options after --")
    options = args.drive_options[1:]
    if "-h" in options or "--help" in options:
        return drive_server.main(options)

    try:
        import serial
        from head_tracker.dashboard import read

        # The dashboard uses the same exclusive UART lock for all servo commands.
        with serial.Serial("/dev/serial0", 115200, timeout=0.01, write_timeout=0.12, exclusive=True) as port:
            with NeutralMonitor(port, read, args.neutral, args.tolerance) as monitor:
                print("Neutral position verified. The drive server owns the servo UART.", flush=True)
                return drive_server.main(["--hardware", *options], before_motor_tick=monitor.check)
    except (ImportError, OSError, RuntimeError) as error:
        print(f"Neutral drive check failed: {error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
