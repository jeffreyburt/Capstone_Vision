import math
import unittest

from head_tracker.tracking_control import TargetMemory, correction, image_to_servo, servo_to_image, validate


class TrackingTests(unittest.TestCase):
    def config(self, **updates):
        config = dict(limits=[[0, 1000], [100, 900]], signs=[1, -1], speed=180)
        config.update(updates)
        return validate(config)

    def test_pan_range_and_tilt_bounds(self):
        config = self.config(speed=400, limits=[[0, 1000], [100, 900]])
        self.assertEqual(config['limits'], [[0, 1000], [100, 900]])
        self.assertEqual(self.config(limits=[[311, 471], [690, 785]])['limits'], [[311, 471], [690, 785]])
        positions, velocity, active = [500, 500], [0, 0], [False, False]
        for _ in range(2000):
            positions, velocity = correction(positions, velocity, [1, 1], config, .025, active)
            self.assertLessEqual(max(map(abs, velocity)), 400)
            self.assertTrue(0 <= positions[0] <= 1000)
            self.assertTrue(100 <= positions[1] <= 900)
        self.assertEqual(positions, [1000, 100])

    def test_center_noise_does_not_start_movement(self):
        config = self.config()
        positions, velocity, active = [500, 500], [0, 0], [False, False]
        for tick in range(500):
            center = [.5 + .05 * math.sin(tick), .5 + .05 * math.cos(tick)]
            positions, velocity = correction(positions, velocity, center, config, .025, active)
        self.assertEqual(positions, [500, 500])

    def test_hysteresis_and_acceleration(self):
        config = self.config()
        active = [False, False]
        positions, velocity = correction([500, 500], [0, 0], [.9, .9], config, .025, active)
        self.assertGreater(positions[0], 500)
        self.assertLess(positions[1], 500)
        self.assertLessEqual(max(map(abs, velocity)), 180 * 8 * .025)
        correction(positions, velocity, [.55, .55], config, .025, active)
        self.assertEqual(active, [True, True])
        correction(positions, velocity, [.52, .52], config, .025, active)
        self.assertEqual(active, [False, False])

    def test_moderate_error_reaches_full_speed_quickly(self):
        config = self.config(speed=300)
        positions, velocity, active = [500, 500], [0, 0], [False, False]
        for _ in range(8):  # 0.2 s with the face 25% of the frame off center
            positions, velocity = correction(positions, velocity, [.75, .5], config, .025, active)
        self.assertEqual(velocity[0], 300)
        self.assertEqual(velocity[1], 0)

    def test_invalid_settings(self):
        for speed in (0, 401, float('nan'), float('inf')):
            with self.subTest(speed=speed), self.assertRaises(ValueError):
                self.config(speed=speed)
        for prediction in (-1, 3.1, float('nan')):
            with self.subTest(prediction=prediction), self.assertRaises(ValueError):
                self.config(prediction=prediction)
        with self.assertRaises(ValueError):
            self.config(limits=[[0, 1000], [900, 100]])

    def test_prediction_velocity_expiry_and_reacquisition(self):
        memory = TargetMemory()
        self.assertIsNone(memory.predict(10, 1.2))
        memory.observe([.7, .4], 10)
        memory.observe([.8, .4], 10.1)
        predicted = memory.predict(10.3, 1.2)
        self.assertGreater(predicted[0], .8)
        self.assertEqual(predicted[1], .4)
        self.assertIsNone(memory.predict(11.4, 1.2))
        self.assertIsNone(memory.predict(10.3, 0))
        memory.observe([.2, .4], 12)
        self.assertEqual(memory.velocity, [0, 0])
        memory.observe([.9, .1], 12.05)
        self.assertTrue(all(abs(value) <= 1.5 for value in memory.velocity))

    def test_prediction_in_head_coordinates_stops_where_the_face_should_be(self):
        # Face drifts right across the image while the head stands still, then leaves the frame.
        config = self.config(speed=300, signs=[1, 1], limits=[[0, 1000], [100, 900]])
        memory = TargetMemory(max_speed=400, bounds=(0, 1000))
        memory.observe(image_to_servo([.8, .5], [500, 500], [1, 1]), 10.0)
        memory.observe(image_to_servo([.9, .5], [500, 500], [1, 1]), 10.1)
        predicted_at_1s = memory.predict(11.1, 1.5)
        self.assertAlmostEqual(predicted_at_1s[0], 500 + .4 * 216 + 1.0 * 216 * .6, delta=1)
        # Drive the head with the prediction: it turns to the predicted spot and settles, not past it.
        positions, velocity, active = [500.0, 500.0], [0, 0], [False, False]
        goal = memory.predict(10.6, 1.5)
        for _ in range(200):
            center = servo_to_image(goal, positions, config['signs'])
            positions, velocity = correction(positions, velocity, center, config, .025, active)
        self.assertLess(abs(positions[0] - goal[0]), .1 * 216)
        self.assertEqual(round(positions[1]), 500)


if __name__ == '__main__':
    unittest.main()
