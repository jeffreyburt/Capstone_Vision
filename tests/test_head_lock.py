import unittest
from unittest.mock import patch

from drive_with_head_lock import NeutralMonitor
from robot.drive_server import DriveParams, DriveServer, PrintMotors


class HeadLockTests(unittest.TestCase):
    def setUp(self):
        self.feedback = {(1, 28): 400, (2, 28): 600, (1, 32): 1, (2, 32): 1}
        self.monitor = NeutralMonitor(
            None, lambda port, servo, command: self.feedback[servo, command], (400, 600), 5
        )

    def test_neutral_feedback_allows_driving(self):
        with self.monitor:
            self.monitor.check()
        self.assertFalse(self.monitor.thread.is_alive())

    def test_missing_feedback_or_out_of_range_position_rejects_start(self):
        for key, value in [((1, 28), None), ((1, 28), 406), ((2, 32), 0)]:
            with self.subTest(key=key, value=value), patch.dict(self.feedback, {key: value}):
                with self.assertRaises(RuntimeError):
                    self.monitor.__enter__()
                self.assertFalse(self.monitor.thread.is_alive())

    def test_runtime_fault_latches_and_drive_server_stops(self):
        self.monitor.sample()
        self.feedback[2, 32] = 0
        with patch.object(self.monitor.done, "wait", return_value=False):
            self.monitor.poll()
        self.feedback[2, 32] = 1
        # Even if the feedback recovers, an operator must restart the server.
        motors = PrintMotors()
        params = DriveParams([1, 3, 2, 4], [1, -1, -1, 1])
        with patch("robot.drive_server.socket.socket") as socket_factory:
            sock = socket_factory.return_value
            sock.getsockname.return_value = ("127.0.0.1", 5700)
            server = DriveServer(motors, params, 1, before_motor_tick=self.monitor.check,
                                 log=lambda message: None)
        with self.assertRaisesRegex(RuntimeError, "Servo 2"):
            server.run()
        self.assertEqual(motors.values, (0, 0, 0, 0))
        self.assertEqual(list(motors.history), [(0, 0, 0, 0)] * 2)
        sock.close.assert_called_once()

    def test_drive_fails_closed_if_feedback_goes_stale(self):
        self.monitor.sample()
        with patch("drive_with_head_lock.time.monotonic", return_value=self.monitor.last_check + 1):
            with self.assertRaisesRegex(RuntimeError, "stale"):
                self.monitor.check()


if __name__ == "__main__":
    unittest.main()
