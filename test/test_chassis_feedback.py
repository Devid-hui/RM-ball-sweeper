import json
import math
import tempfile
import unittest
from pathlib import Path

from ball_chassis import Mecanum
from tasks.ball_collect import openrf1
from tasks.ball_collect.run import ROOT, load_config


class ChassisFeedbackTest(unittest.TestCase):
    def test_preview_defaults_cannot_enable_the_wheels(self):
        path = ROOT / "configs/collector.json"
        self.assertGreater(load_config(path, False)["encoder_counts_per_rev"], 0)
        with self.assertRaisesRegex(ValueError, "drive_calibrated"):
            load_config(path, True)

    def test_previous_calibrated_config_still_works(self):
        settings = {"encoder_counts_per_rev": 1560, "drive_sign_abcd": [-1, 1, -1, 1]}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "vehicle.json"
            path.write_text(json.dumps(settings))
            self.assertEqual(load_config(path, True)["encoder_counts_per_rev"], 1560)
            path.write_text('{"drive_calibrated": true}')
            with self.assertRaises(ValueError):
                load_config(path, True)

    def test_abcd_yaw_wraparound_and_visual_reanchor(self):
        model = Mecanum(0.05, 0.30, 0.20)
        heading = openrf1.WheelHeading(model, 1000, 6.0)
        self.assertIsNone(heading.update([65534] * 4, None, 0.0))
        self.assertEqual(heading.update([65534] * 4, 1.0, 0.1), 1.0)
        # 左轮反转、右轮正转时，ABCD 原始计数都增加。
        self.assertAlmostEqual(heading.update([8] * 4, None, 0.2), 1 + 0.05 / 0.25 * math.tau / 100)
        # 直行使 A/C 原始计数减小，B/D 增大，航向应保持。
        expected = heading.update([0, 16, 0, 16], None, 0.3)
        self.assertAlmostEqual(expected, 1 + 0.05 / 0.25 * math.tau / 100)
        self.assertAlmostEqual(heading.update([0, 16, 0, 16], -0.42, 0.4), -0.42)
        with self.assertRaisesRegex(ValueError, "编码器"):
            heading.update([20000] * 4, None, 0.5)
