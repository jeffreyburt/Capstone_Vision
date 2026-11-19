"""Local TCP forwards for read-only camera streams through the main Pi."""

import socket
import subprocess
import threading
import time


class CameraTunnels:
    def __init__(self, ssh, gateway):
        self.ssh, self.gateway = ssh, gateway
        self.lock = threading.Lock()
        self.processes = {}

    def endpoint(self, host, port):
        if not self.gateway:
            return host, port
        with self.lock:
            key = (host, port)
            existing = self.processes.get(key)
            if existing and existing[0].poll() is None:
                return "127.0.0.1", existing[1]
            with socket.socket() as sock:
                sock.bind(("127.0.0.1", 0))
                local_port = sock.getsockname()[1]
            proc = subprocess.Popen(
                [self.ssh, "-N", "-o", "BatchMode=yes", "-o", "ConnectTimeout=3",
                 "-o", "ExitOnForwardFailure=yes", "-o", "ServerAliveInterval=5",
                 "-o", "ServerAliveCountMax=2", "-L", f"127.0.0.1:{local_port}:{host}:{port}", self.gateway],
                stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            for _ in range(50):
                if proc.poll() is not None:
                    break
                try:
                    socket.create_connection(("127.0.0.1", local_port), timeout=.1).close()
                    self.processes[key] = (proc, local_port)
                    return "127.0.0.1", local_port
                except OSError:
                    time.sleep(.1)
            proc.terminate()
            proc.wait()
            raise OSError("camera SSH forward could not start; check gateway key login")

    def close(self):
        with self.lock:
            for proc, _ in self.processes.values():
                if proc.poll() is None:
                    proc.terminate()
                proc.wait()
            self.processes.clear()
