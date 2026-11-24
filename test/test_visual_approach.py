import json
import math
import unittest
from pathlib import Path

from tasks.ball_collect.navigation import Navigator, STOP


class VisualApproachTest(unittest.TestCase):
    def setUp(self):
        path = Path(__file__).resolve().parents[1] / "configs/collector.json"
        self.config = json.loads(path.read_text())
        self.nav = Navigator(self.config)
        self.pose = (0.75, 0.0, math.pi)

    def test_approaches_in_chassis_coordinates_and_stops_on_loss(self):
        velocity, _ = self.nav.step(None, [(0.45, 0.25), (0.40, -0.10)])
        forward, left, turn = velocity
        self.assertGreater(forward, 0)
        self.assertLess(left, 0)
        self.assertEqual(turn, 0)
        self.assertAlmostEqual(math.hypot(forward, left), self.config["visual_approach_speed_m_s"])
        self.assertEqual(self.nav.step(None, [])[0], STOP)
        self.assertEqual(self.nav.step(None, [(0.60, 0.30)])[0], STOP)

    def test_standoff_survives_missing_ball_distractor_and_flickering_tag(self):
        start = self.config["fork_forward_m"] + self.config["visual_standoff_m"]
        ball = [(start, 0.0)]
        for pose, balls in [(None, ball), (None, []), (None, [(0.5, 0.1)]),
                            (self.pose, ball), (None, [(start + 0.03, 0.0)]), (self.pose, ball)]:
            self.assertEqual(self.nav.step(pose, balls)[0], STOP)
            self.assertEqual(self.nav.phase, "wait_for_tag")
        self.assertEqual(self.nav.step(self.pose, ball)[0], STOP)
        self.assertEqual(self.nav.phase, "approach")

        # 车向墙前进 5 cm，同一个球在车体系中也应近 5 cm。
        closer = (self.pose[0] - 0.05, 0.0, math.pi)
        self.nav.step(closer, [(start - 0.05, 0.0)])
        self.assertEqual(self.nav.phase, "push")
        self.assertGreater(self.nav.step(closer, [])[0][0], 0)
        self.assertEqual(self.nav.step(None, [(0.40, 0.0)])[0], STOP)
        self.assertEqual(self.nav.phase, "push")

    def test_waits_for_alignment_and_skips_ball_in_drop_zone(self):
        near = [(0.14, 0.0)]
        self.nav.step(None, near)
        angled = (0.75, 0.16, math.pi)
        for _ in range(2):
            self.nav.step(angled, near)
        self.assertNotEqual(self.nav.step(angled, near)[0][2], 0)
        self.assertEqual(self.nav.phase, "approach")
        self.assertEqual(self.nav.step(None, near)[0], STOP)
        # 新的墙标观测把球定位到投放区内。
        self.assertEqual(self.nav.step((0.40, 0.0, math.pi), near)[0], STOP)
        self.assertEqual(self.nav.phase, "lane_start")


if __name__ == "__main__":
    unittest.main()
