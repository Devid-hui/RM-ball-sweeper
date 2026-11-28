import json
import math
import unittest
from pathlib import Path
from unittest.mock import patch

from tasks.ball_collect.navigation import Navigator, STOP


class CollectionRouteTest(unittest.TestCase):
    def setUp(self):
        self.config = json.loads((Path(__file__).resolve().parents[1] / "configs/collector.json").read_text())
        self.config["lane_starts"] = [[1.1, 0.0], [1.1, 0.25]]
        self.nav = Navigator(self.config)
        self.nav.step((1.1, 0.0, math.pi), [])

    def test_adjacent_lanes_start_at_the_end_just_reached(self):
        self.nav.step((0.4, 0.0, math.pi), [])
        velocity, _ = self.nav.step((0.4, 0.0, math.pi), [])
        self.assertAlmostEqual(velocity[0], 0.0)
        self.assertLess(velocity[1], 0.0)
        self.nav.step((0.4, 0.25, math.pi), [])
        velocity, _ = self.nav.step((0.4, 0.25, math.pi), [])
        self.assertLess(velocity[0], 0.0)
        self.nav.step((1.1, 0.25, math.pi), [])
        self.assertEqual(self.nav.phase, "finished")

    def test_two_deliveries_turn_then_resume_the_interrupted_lane(self):
        for count, intercept_x in enumerate((0.90, 0.65), 1):
            with self.subTest(delivery=count):
                intercept = (intercept_x, 0.0, math.pi)
                for _ in range(2):
                    self.nav.step(intercept, [(0.10, 0.0)])
                self.nav.step(intercept, [(0.10, 0.0)])
                self.assertEqual(self.nav.phase, "push")
                self.assertEqual(self.nav.step(None, [])[0], STOP)
                stop_x = 0.23 + self.config["fork_forward_m"] + 0.017
                self.nav.step((stop_x, 0.0, math.pi), [])
                self.assertEqual(self.nav.phase, "reverse")
                self.assertEqual(self.nav.step(None, [])[0], STOP)
                release = (stop_x + self.config["reverse_m"], 0.0, math.pi)
                self.nav.step(release, [])
                self.assertEqual(self.nav.phase, "turn")
                self.assertEqual(self.nav.step(None, [])[0], STOP)
                velocity, _ = self.nav.step(None, [], wheel_yaw=math.pi)
                self.assertEqual(velocity[:2], (0.0, 0.0))
                self.assertGreater(velocity[2], 0.0)
                self.nav.step(None, [], wheel_yaw=-math.pi / 2)
                self.assertEqual(self.nav.deliveries, count)
                velocity, _ = self.nav.step(None, [], wheel_yaw=-math.pi / 2)
                self.assertEqual(velocity[:2], (0.0, 0.0))
                self.assertLess(velocity[2], 0.0)
                self.assertEqual(self.nav.step(None, [], wheel_yaw=math.pi)[0], STOP)
                self.nav.step(intercept, [])
                self.assertEqual(self.nav.phase, "search")
                self.assertEqual(self.nav.lane, 0)

    def test_new_visual_target_skips_a_ball_already_in_the_fork(self):
        velocity, _ = self.nav.step(None, [(0.02, 0.0), (0.35, -0.1)])
        self.assertGreater(velocity[0], 0.0)
        self.assertLess(velocity[1], 0.0)

    def test_short_loss_of_both_tag_and_ball_can_recover(self):
        pose = (0.90, 0.0, math.pi)
        ball = [(0.25, 0.0)]
        for _ in range(2):
            self.nav.step(pose, ball)
        self.assertEqual(self.nav.phase, "approach")
        self.assertEqual(self.nav.step(None, [])[0], STOP)
        self.assertGreater(self.nav.step(pose, ball)[0][0], 0.0)

    def test_persistent_loss_resumes_interrupted_lane_after_three_stops(self):
        pose = (0.90, 0.0, math.pi)
        ball = [(0.25, 0.0)]
        for _ in range(2):
            self.nav.step(pose, ball)
        for _ in range(2):
            self.assertEqual(self.nav.step(pose, [])[0], STOP)
            self.assertEqual(self.nav.phase, "approach")
        self.assertGreater(self.nav.step(pose, ball)[0][0], 0.0)
        for _ in range(2):
            self.assertEqual(self.nav.step(pose, [])[0], STOP)
            self.assertEqual(self.nav.phase, "approach")
        self.assertEqual(self.nav.step(pose, [])[0], STOP)
        self.assertEqual(self.nav.phase, "lane_start")
        self.nav.step(pose, [])
        self.assertEqual(self.nav.phase, "search")
        self.assertEqual(self.nav.deliveries, 0)

    def test_loss_in_fork_blind_spot_keeps_car_stopped(self):
        pose = (0.90, 0.0, math.pi)
        for _ in range(2):
            self.nav.step(pose, [(0.25, 0.0)])
        closer = (0.76, 0.0, math.pi)
        for _ in range(4):
            self.assertEqual(self.nav.step(closer, [])[0], STOP)
        self.assertEqual(self.nav.phase, "approach")

    def test_aligns_at_fourteen_centimeters_before_ball_leaves_view(self):
        pose = (0.90, 0.0, math.pi)
        ball = [(0.14, 0.0)]
        for _ in range(2):
            self.nav.step(pose, ball)
        self.assertEqual(self.nav.phase, "approach")
        self.assertEqual(self.nav.step(pose, ball), (STOP, "fork aligned"))
        self.assertEqual(self.nav.phase, "push")

    def test_new_target_uses_calibrated_near_cutoff_but_locked_ball_can_approach(self):
        nav = Navigator(self.config, minimum_candidate_forward_m=0.20)
        nav.step((1.1, 0.0, math.pi), [])
        for _ in range(2):
            nav.step((0.9, 0.0, math.pi), [(0.12, 0.0), (0.25, 0.0)])
        self.assertEqual(nav.visual_target, (0.25, 0.0))
        nav.step((0.75, 0.0, math.pi), [(0.10, 0.0)])
        self.assertEqual(nav.phase, "push")

    def test_target_replacement_cannot_bypass_near_cutoff(self):
        nav = Navigator(self.config, minimum_candidate_forward_m=0.20)
        pose = (0.9, 0.0, math.pi)
        nav.step(None, [(0.55, 0.0)])
        for _ in range(4):
            nav.step(pose, [(0.11, 0.0)])
        self.assertNotIn(nav.phase, ("approach", "push"))

    def test_turn_timeout_stops_if_encoders_do_not_advance(self):
        with patch("tasks.ball_collect.navigation.monotonic", return_value=1.0):
            self.nav.phase = "reverse"
            self.nav.release = (0.5, 0.0)
            self.nav.step((0.5, 0.0, math.pi), [])
        with patch("tasks.ball_collect.navigation.monotonic", return_value=1.0 + self.config["turn_timeout_s"]):
            with self.assertRaises(TimeoutError):
                self.nav.step(None, [], wheel_yaw=math.pi)
