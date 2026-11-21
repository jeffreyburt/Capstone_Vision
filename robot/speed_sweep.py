"""Lifted-wheel top-speed sweep for Robot 1: TB6612 open-loop duty and Hiwonder count commands.

LIFT THE CHASSIS FIRST: every wheel spins up to full speed. Run on Robot 1 as robot-pi1:

    PYTHONPATH=~/capstone-base/src python3 ~/capstone-drive/speed_sweep.py > sweep.jsonl

Stops capstone-drive-1.service (and restarts it afterwards if it was running), holds the motor
diagnostic lock, bounds every step in time, and stops all outputs on error, Ctrl+C, or when stdin
closes (SSH dropped). Prints one JSON line per measured step.
"""
import fcntl
import json
import os
import select
import subprocess
import sys
import time
from contextlib import ExitStack

HERE = os.path.dirname(os.path.abspath(__file__))
SERVICE = 'capstone-drive-1.service'
TB_DUTIES = [.1, .15, .2, .25, .3, .4, .5, .6, .7, .8, .9, 1.0]
HW_COUNTS = [5, 10, 15, 20, 30, 40, 50, 60, 70, 80, 100, 127]
SETTLE_S, MEASURE_S, REST_S = 1.0, 1.0, .4


def emit(**data):
    print(json.dumps(data), flush=True)


def stdin_closed():
    if not select.select([sys.stdin], [], [], 0)[0]:
        return False
    return sys.stdin.read(1) == ''  # EOF: the SSH session went away


def wait(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if stdin_closed():
            raise KeyboardInterrupt('stdin closed')
        time.sleep(.02)


def main():
    with open(os.path.join(HERE, 'robot1_four_wheels.json')) as f:
        wiring = json.load(f)
    pins = wiring['tb6612']['pins']
    from capstone_base.hiwonder_i2c import HiwonderConfig, HiwonderController
    from gpiozero import DigitalOutputDevice, DigitalInputDevice
    from gpiozero.pins.lgpio import LGPIOFactory
    sys.path.insert(0, HERE)
    from motor_speed import PrecisionPWM

    fields = HiwonderConfig.__dataclass_fields__
    hw = {k: v for k, v in wiring['hiwonder'].items() if k in fields}
    hw['max_speed_counts'] = 127  # test only; the drive config stays as deployed

    with open('/tmp/capstone-motor-diagnostic.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        active = subprocess.run(['systemctl', 'is-active', '--quiet', SERVICE]).returncode == 0
        if active:
            subprocess.run(['sudo', '-n', '/usr/bin/systemctl', 'stop', SERVICE], check=True)
        try:
            with ExitStack() as stack:
                board = stack.enter_context(HiwonderController(HiwonderConfig(**hw)))
                board.initialize()
                board.stop()
                stack.callback(board.stop)
                factory = stack.enter_context(LGPIOFactory())
                pwm = stack.enter_context(PrecisionPWM(pins['pwm']))
                def out(name):
                    return stack.enter_context(DigitalOutputDevice(pins[name], pin_factory=factory, initial_value=False))
                standby, in1, in2 = out('standby'), out('in1'), out('in2')
                a = stack.enter_context(DigitalInputDevice(pins['encoder_a'], pin_factory=factory, pull_up=True))
                b = stack.enter_context(DigitalInputDevice(pins['encoder_b'], pin_factory=factory, pull_up=True))
                edges = [0, 0]
                def ea(): edges[0] += 1
                def eb(): edges[1] += 1
                a.when_activated = a.when_deactivated = ea
                b.when_activated = b.when_deactivated = eb
                def tb_stop():
                    pwm.off(); standby.off(); in1.off(); in2.off()
                stack.callback(tb_stop)
                try:
                    emit(kind='start', battery_v=board.read_battery_voltage())
                except Exception as e:
                    emit(kind='start', battery_error=str(e))

                for sign in (1, -1):
                    for duty in TB_DUTIES:
                        in1.value, in2.value = sign > 0, sign < 0
                        standby.on()
                        pwm.value = max(duty, .08)  # brief breakaway at low duty
                        wait(.2)
                        pwm.value = duty
                        wait(SETTLE_S - .2)
                        e0, t0 = edges.copy(), time.monotonic()
                        wait(MEASURE_S)
                        e1, t1 = edges.copy(), time.monotonic()
                        dt = t1 - t0
                        emit(kind='tb6612', direction=sign, duty=duty,
                             count_rate=(sum(e1) - sum(e0)) / dt,
                             phase_rates=[(x - y) / dt for x, y in zip(e1, e0)])
                    tb_stop()
                    wait(REST_S + .3)

                for sign in (1, -1):
                    for counts in HW_COUNTS:
                        board.set_motor_speed_counts((0, sign * counts, sign * counts, sign * counts))
                        wait(SETTLE_S)
                        c0, t0 = board.read_encoder_counts(), time.monotonic()
                        wait(MEASURE_S)
                        c1, t1 = board.read_encoder_counts(), time.monotonic()
                        dt = t1 - t0
                        rates = [(((y - x + 2**31) % 2**32) - 2**31) / dt for x, y in zip(c0, c1)]
                        emit(kind='hiwonder', direction=sign, command_counts=counts, rates=rates)
                    board.stop()
                    wait(REST_S + .3)
                try:
                    emit(kind='end', battery_v=board.read_battery_voltage())
                except Exception:
                    pass
        finally:
            if active:
                subprocess.run(['sudo', '-n', '/usr/bin/systemctl', 'start', SERVICE], check=True)


def validate(commands=(20, 40, 60, 70)):
    """Closed-loop check through the real MixedMotors path: logical M1..M4 all at each command."""
    with open(os.path.join(HERE, 'robot1_four_wheels.json')) as f:
        wiring = json.load(f)
    with open(os.path.join(HERE, wiring['tb6612']['calibration'])) as f:
        profile = json.load(f)['tb6612_calibration']
    from capstone_base.hiwonder_i2c import HiwonderConfig, HiwonderController
    sys.path.insert(0, HERE)
    from mixed_motors import MixedMotors, TB6612Motor
    fields = HiwonderConfig.__dataclass_fields__
    hw = {k: v for k, v in wiring['hiwonder'].items() if k in fields}
    with open('/tmp/capstone-motor-diagnostic.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        active = subprocess.run(['systemctl', 'is-active', '--quiet', SERVICE]).returncode == 0
        if active:
            subprocess.run(['sudo', '-n', '/usr/bin/systemctl', 'stop', SERVICE], check=True)
        try:
            with ExitStack() as stack:
                board = stack.enter_context(HiwonderController(HiwonderConfig(**hw)))
                board.initialize(); board.stop()
                tb = TB6612Motor(profile, wiring['tb6612']['pins'])
                stack.callback(tb.close)
                motors = MixedMotors(board, tb, profile, direction=wiring['tb6612'].get('direction', 1))
                stack.callback(motors.stop)
                for sign in (1, -1):
                    for command in commands:
                        start = time.monotonic(); c0 = t0 = None
                        while time.monotonic() - start < 3.0:
                            motors.set_motor_speed_counts([sign * command] * 4)
                            if motors.fault:
                                raise RuntimeError(motors.fault)
                            if c0 is None and time.monotonic() - start >= 2.0:
                                c0, t0 = board.read_encoder_counts(), time.monotonic()
                            wait(.05)
                        c1, t1 = board.read_encoder_counts(), time.monotonic()
                        t = tb.telemetry()
                        emit(kind='validate', direction=sign, command_counts=command,
                             tb6612_target=t['target_count_rate'], tb6612_measured=t['measured_count_rate'],
                             tb6612_duty_percent=t['duty_percent'],
                             hiwonder_rates=[(((y - x + 2**31) % 2**32) - 2**31) / (t1 - t0) for x, y in zip(c0, c1)])
                    motors.stop(); wait(.6)
        finally:
            if active:
                subprocess.run(['sudo', '-n', '/usr/bin/systemctl', 'start', SERVICE], check=True)


def _terminate(signum, frame):
    raise KeyboardInterrupt(f'signal {signum}')  # run every cleanup callback; never leave a motor on


if __name__ == '__main__':
    import signal
    signal.signal(signal.SIGTERM, _terminate)
    signal.signal(signal.SIGHUP, _terminate)
    try:
        validate() if '--validate' in sys.argv else main()
    except KeyboardInterrupt as e:
        emit(kind='stopped', reason=str(e) or 'interrupted')
        sys.exit(1)
    except Exception as e:
        emit(kind='error', error=str(e))
        sys.exit(1)
