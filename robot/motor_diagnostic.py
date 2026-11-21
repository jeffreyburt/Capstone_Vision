"""One bounded, exclusive motor test; newline or SSH stdin loss stops the test."""
import argparse
from contextlib import ExitStack
import fcntl
import json
import os
import select
import subprocess
import sys
import time


def request(data):
    rid, channel = data.get('robot_id'), data.get('channel')
    speed, duration = data.get('speed'), data.get('seconds')
    if type(rid) is not int or rid not in (1, 2):
        raise ValueError('Select robot 1 or 2')
    if channel not in ('M1', 'M2', 'M3', 'M4', 'TB6612') or (rid == 2 and channel == 'TB6612'):
        raise ValueError('Invalid motor channel')
    if rid == 1 and channel == 'M1':
        raise ValueError('Robot 1 M1 is disabled; use its TB6612 connection')
    if type(speed) is not int or not 1 <= abs(speed) <= 50:
        raise ValueError('Speed must be a signed percentage from 1 to 50')
    if type(duration) not in (int, float) or not 0.2 <= duration <= 5:
        raise ValueError('Duration must be between 0.2 and 5 seconds')
    return rid, channel, speed, duration


def emit(**data):
    print(json.dumps(data), flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('request')
    args = ap.parse_args()
    rid, channel, speed, duration = request(json.loads(args.request))
    service = f'capstone-drive-{rid}.service'
    with open('/tmp/capstone-motor-diagnostic.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        active = subprocess.run(['systemctl', 'is-active', '--quiet', service]).returncode == 0
        if active:
            subprocess.run(['sudo', '-n', 'systemctl', 'stop', service], check=True)
        try:
            with ExitStack() as stack:
                sys.path.insert(0, os.path.expanduser('~/capstone-base/src'))
                from capstone_base.hiwonder_i2c import HiwonderController, HiwonderConfig
                config = os.path.expanduser('~/capstone-drive/robot1_three_wheels.json' if rid == 1
                                           else '~/capstone-base/config/robot2_wheels.json')
                with open(config) as f:
                    hw = json.load(f)
                hw = hw.get('hiwonder', hw)
                fields = HiwonderConfig.__dataclass_fields__
                board = stack.enter_context(HiwonderController(HiwonderConfig(**{k:v for k,v in hw.items() if k in fields})))
                board.stop()
                if channel == 'TB6612':
                    from gpiozero import DigitalOutputDevice, PWMOutputDevice, DigitalInputDevice
                    from gpiozero.pins.lgpio import LGPIOFactory
                    factory = stack.enter_context(LGPIOFactory())
                    def output(cls, pin, **kw):
                        return stack.enter_context(cls(pin, pin_factory=factory, **kw))
                    pwm = output(PWMOutputDevice, 18, initial_value=0, frequency=1000)
                    standby = output(DigitalOutputDevice, 22, initial_value=False)
                    a1 = output(DigitalOutputDevice, 17, initial_value=False)
                    a2 = output(DigitalOutputDevice, 27, initial_value=False)
                    a = output(DigitalInputDevice, 23, pull_up=True)
                    b = output(DigitalInputDevice, 24, pull_up=True)
                    edges = [0, 0]
                    def ea(): edges[0] += 1
                    def eb(): edges[1] += 1
                    a.when_activated = a.when_deactivated = ea
                    b.when_activated = b.when_deactivated = eb
                    def stop():
                        pwm.off(); standby.off(); a1.off(); a2.off()
                    def read():
                        return dict(counts=edges.copy(), levels=[int(a.pin.state), int(b.pin.state)],
                                    labels=['C1 edges', 'C2 edges'], units='edges', command=speed)
                    stack.callback(stop)
                    def drive():
                        a1.value = speed > 0; a2.value = speed < 0
                        standby.on(); pwm.value = abs(speed) / 100
                else:
                    board.initialize()
                    baseline = board.read_encoder_counts()
                    command = round(speed / 100 * board.config.max_speed_counts)
                    command = command or (1 if speed > 0 else -1)
                    values = [0] * 4
                    values[int(channel[1]) - 1] = command
                    stop = board.stop
                    def read():
                        raw = board.read_encoder_counts()
                        delta = [((v-z+2**31) % 2**32)-2**31 for v,z in zip(raw, baseline)]
                        return dict(counts=delta, raw=list(raw), labels=['M1','M2','M3','M4'],
                                    units='counts', command=command)
                    def drive(): board.set_motor_speed_counts(tuple(values))
                start = time.monotonic()
                previous = read()['counts']
                previous_t = 0
                emit(state='running', elapsed=0, rates=[0]*len(previous), **read())
                if select.select([sys.stdin], [], [], 0)[0]:
                    emit(state='finished', elapsed=0, rates=[0]*len(previous), **read())
                    return
                drive()
                try:
                    while True:
                        if select.select([sys.stdin], [], [], .1)[0]:
                            break  # stop message or connection EOF
                        elapsed = time.monotonic() - start
                        sample = read()
                        rates = [(v-p)/(elapsed-previous_t) for v,p in zip(sample['counts'], previous)]
                        emit(state='running', elapsed=round(elapsed,3), rates=rates, **sample)
                        previous, previous_t = sample['counts'], elapsed
                        if elapsed >= duration: break
                finally:
                    stop()
                emit(state='finished', elapsed=round(time.monotonic()-start,3), rates=[0]*len(previous), **read())
        finally:
            if active:
                subprocess.run(['sudo', '-n', 'systemctl', 'start', service], check=True)

if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        emit(state='error', error=str(error))
        sys.exit(1)
