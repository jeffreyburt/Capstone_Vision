"""Dashboard motor diagnostics over existing authenticated SSH robot connections."""
import copy
import json
import shlex
import subprocess
import threading
import time
from robot.motor_diagnostic import request


class MotorTests:
    def __init__(self, monitor):
        self.monitor = monitor
        self.lock = threading.RLock()
        self.process = None
        self.state = {'state': 'idle', 'samples': [], 'message': 'Select one motor to test.'}

    def snapshot(self):
        with self.lock:
            data = copy.deepcopy(self.state)
            data['running'] = self.process is not None
            data['sample_age_s'] = time.time()-data['received_at'] if 'received_at' in data else None
            return data

    def start(self, data):
        rid, channel, speed, seconds = request(data)
        with self.lock:
            if self.process is not None:
                raise ValueError('A motor test is already running')
            node = next(n for n in self.monitor.nodes if n.robot_id == rid)
            if not node.ssh_user:
                raise ValueError('Robot SSH username is not configured')
            self.monitor.waypoints.arm(False)
            self.monitor.waypoints.clear()
            payload = dict(robot_id=rid, channel=channel, speed=speed, seconds=seconds)
            command = 'python3 -u ~/capstone-drive/motor_diagnostic.py ' + shlex.quote(json.dumps(payload))
            self.state = {**payload, 'state':'connecting', 'samples':[], 'received_at':time.time()}
            self.process = subprocess.Popen([self.monitor.ssh, '-T', '-o', 'BatchMode=yes',
                '-o', 'ConnectTimeout=5', '-o', 'ServerAliveInterval=2', '-o', 'ServerAliveCountMax=2',
                '-p', str(node.ssh_port), f'{node.ssh_user}@{node.host}', command],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            threading.Thread(target=self._read, args=(self.process,), daemon=True).start()
            timer = threading.Timer(15, self._expire, args=(self.process,))
            timer.daemon = True
            timer.start()
            return self.snapshot()

    def _expire(self, process):
        with self.lock:
            if self.process is process:
                self.stop()
                process.terminate()

    def _read(self, process):
        try:
            for line in process.stdout:
                try:
                    sample = json.loads(line)
                except ValueError:
                    continue
                with self.lock:
                    self.state.update(sample, received_at=time.time())
                    self.state['samples'] = (self.state['samples'] + [sample])[-100:]
            error = process.stderr.read().strip()
            code = process.wait()
            with self.lock:
                if code:
                    self.state.update(state='error', error=self.state.get('error') or error[-1500:] or f'SSH exited {code}')
                elif self.state.get('state') != 'finished':
                    self.state.update(state='error', error='Test ended without a final motor-off report')
        finally:
            process.stdin.close(); process.stdout.close(); process.stderr.close()
            with self.lock:
                if self.process is process:
                    self.process = None

    def stop(self):
        with self.lock:
            if self.process is not None:
                try:
                    self.process.stdin.write('stop\n'); self.process.stdin.flush()
                    self.state['state'] = 'stopping'
                except (OSError, ValueError):
                    pass
            return self.snapshot()
