import math
import numpy as np
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
        from head_tracker.tracking_control import UNITS_PER_FRAME
        u = UNITS_PER_FRAME[0]
        self.assertAlmostEqual(predicted_at_1s[0], 500 + .4 * u + 1.0 * u * .6, delta=1)
        # Drive the head with the prediction: it turns to the predicted spot and settles, not past it.
        positions, velocity, active = [500.0, 500.0], [0, 0], [False, False]
        goal = memory.predict(10.6, 1.5)
        for _ in range(200):
            center = servo_to_image(goal, positions, config['signs'])
            positions, velocity = correction(positions, velocity, center, config, .025, active)
        self.assertLess(abs(positions[0] - goal[0]), .1 * u)
        self.assertEqual(round(positions[1]), 500)


if __name__ == '__main__':
    unittest.main()


def _simulate(controller, speed=400, delay=0.15, frame_dt=1 / 30, seconds=3.0, face=541.0):
    """Head on one axis chasing a still face at `face` units, seen through a delayed 30 fps camera.
    Returns the commanded pan positions over time."""
    import collections
    from head_tracker.tracking_control import UNITS_PER_FRAME, approach, correction, image_to_servo, validate
    config = validate({"limits": [[0, 1000], [100, 900]], "signs": [1, 1], "speed": speed})
    target, actual, velocity, active = [391.0, 500.0], [391.0, 500.0], [0.0, 0.0], [False, False]
    history, frames, t, dt, out, next_frame = collections.deque(), [], 0.0, 0.025, [], 0.0
    while t < seconds:
        if t >= next_frame:  # capture: the face's image position from where the head actually points now
            centre = [0.5 + (face - actual[0]) / UNITS_PER_FRAME[0], 0.5]
            frames.append((t + delay, t, centre, actual[:]))
            next_frame += frame_dt
        while frames and frames[0][0] <= t:
            _, shot, centre, aimed = frames.pop(0)
            history.append((centre, aimed))
        if history:
            centre, aimed = history[-1]
            if controller == "approach":
                goal = image_to_servo(centre, aimed, config["signs"])
                target, velocity = approach(target, velocity, goal, config, dt, active)
            else:
                target, velocity = correction(target, velocity, centre, config, dt, active)
        actual = [a + (c - a) * (1 - math.exp(-dt / 0.06)) for a, c in zip(actual, target)]  # servo lag
        out.append(actual[0])
        t += dt
    return out


class HeadOvershootTests(unittest.TestCase):
    def test_servo_space_goal_does_not_overshoot_at_full_speed(self):
        path = _simulate("approach")
        self.assertLess(max(path) - 541, 6)          # no overshoot beyond the dead zone
        self.assertLess(abs(path[-1] - 541), 13)     # settles on the face
        self.assertLess(next(i for i, p in enumerate(path) if p > 530) * 0.025, 1.0)  # and gets there quickly

    def test_old_image_error_chase_overshoots_with_camera_delay(self):
        path = _simulate("correction")
        self.assertGreater(max(path) - 541, 20)      # the behaviour this replaces


class ScaleTests(unittest.TestCase):
    def test_wrong_image_scale_overshoots_and_the_measured_one_does_not(self):
        """The tracker assumed 216 x 171 units across the image; the heads measure ~170 x 117. Aiming with the
        too-large scale overshoots the face on every detection; the measured one lands on it."""
        import head_tracker.tracking_control as tc
        real = (170.0, 117.0)
        face = [470.0, 650.0]
        for assumed, should_overshoot in (((216.0, 171.0), True), (real, False)):
            head = [391.0, 770.0]
            centre = [0.5 + (f - h) * -1 / u for f, h, u in zip(face, head, real)]  # signs [-1, -1] as on Robot 1
            goal = tc.image_to_servo(centre, head, [-1, -1], assumed)
            overshoot = max(abs(g - h) - abs(f - h) for g, h, f in zip(goal, head, face))
            self.assertEqual(overshoot > 10, should_overshoot)
        with self.assertRaises(ValueError):
            tc.validate({"limits": [[0, 1000], [100, 900]], "signs": [1, 1], "units_per_frame": [10, 117]})


class PredictionStabilityTests(unittest.TestCase):
    def test_jitter_does_not_become_a_fast_prediction_and_travel_is_bounded(self):
        rng = np.random.default_rng(3)
        twitchy = TargetMemory(max_speed=400, bounds=(0, 1000))
        steady = TargetMemory(max_speed=150, bounds=(0, 1000), window_s=.3, travel_s=.6)
        for k in range(30):  # a still face with +-4 units of detection jitter at 30 fps
            sample = [500 + rng.normal(0, 4), 700 + rng.normal(0, 4)]
            twitchy.observe(sample, 10 + k / 30)
            steady.observe(sample, 10 + k / 30)
        far = lambda m: max(abs(p - c) for p, c in zip(m.predict(11.5, 1.5), (500, 700)))
        self.assertLess(far(steady), 25)                      # holds near the face
        self.assertGreater(far(twitchy), far(steady))         # the old two-frame estimate wanders further
        walker = TargetMemory(max_speed=150, bounds=(0, 1000), window_s=.3, travel_s=.6)
        for k in range(10):  # a face really moving at 300 units/s: capped at 150, and only for 0.6 s
            walker.observe([400 + 10 * k, 700], 10 + k / 30)
        self.assertLessEqual(walker.predict(10.3 + 1.4, 1.5)[0] - 490, 150 * .6 + 1)


class SpeakerOffsetTests(unittest.TestCase):
    def test_face_sits_below_centre_by_the_speaker_angle_at_any_distance(self):
        """Speaker 0.1016 m below the camera: the head must look up by atan(offset / distance), so the face sits
        that angle below the image centre. Checked against the true angle at 0.5-3 m."""
        from head_tracker.tracking_control import aim_offset_fraction, validate
        config = validate({"limits": [[0, 1000], [100, 900]], "signs": [-1, 1], "units_per_frame": [170, 117],
                           "aim_offset_m": 0.1016, "face_width_m": 0.15})
        deg_per_unit = 0.24
        hfov = 170 * deg_per_unit
        for distance in (0.5, 1.0, 2.0, 3.0):
            face_deg = 2 * math.degrees(math.atan(0.15 / 2 / distance))
            size = face_deg / hfov                                   # face box width as a fraction of the image
            want_units = math.degrees(math.atan(0.1016 / distance)) / deg_per_unit
            got_units = aim_offset_fraction(size, config) * 117
            self.assertAlmostEqual(got_units, want_units, delta=max(1.0, want_units * 0.03))
        self.assertEqual(aim_offset_fraction(None, config), 0.0)     # no face size (main-camera aim): no offset here
        self.assertEqual(aim_offset_fraction(0.2, validate({"limits": [[0, 1000], [100, 900]], "signs": [1, 1]})), 0.0)
        with self.assertRaises(ValueError):
            validate({"limits": [[0, 1000], [100, 900]], "signs": [1, 1], "aim_offset_m": 2})
