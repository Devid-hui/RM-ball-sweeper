import json
import unittest
from pathlib import Path

import cv2
import numpy as np

from tasks.ball_detect import ground_position
from tasks.ball_detect.hsv import HsvBallDetector


ROOT = Path(__file__).resolve().parents[1]


class DiameterVisionTest(unittest.TestCase):
    def test_focal_length_and_known_diameter_recover_distance(self):
        camera = ground_position.GroundProjector(ROOT / "configs/hikrobot_DA6415777.yaml")
        camera.image_size = (640, 480)
        camera.matrix = np.array([[500., 0., 320.], [0., 500., 240.], [0., 0., 1.]])
        camera.distortion = np.zeros(5)
        box = (311.5, 231.5, 328.5, 248.5)
        point = camera.position_from_diameter(box)
        self.assertAlmostEqual(point["x_m"], 0.5, places=3)
        self.assertAlmostEqual(point["y_m"], 0.0)
        corrected = camera.position_from_diameter(box, scale=1.10)
        self.assertAlmostEqual(corrected["x_m"], 1.1 * point["x_m"])
        self.assertIsNone(camera.position_from_diameter((0., 0., 17., 17.)))
        self.assertIsNone(camera.position_from_diameter((319., 239., 321., 241.)))

    def test_hsv_accepts_round_ball_and_rejects_long_stripe(self):
        config = json.loads((ROOT / "configs/collector.json").read_text())
        hsv = np.zeros((120, 160, 3), dtype=np.uint8)
        cv2.circle(hsv, (42, 55), 12, (28, 140, 180), -1)
        cv2.rectangle(hsv, (80, 58), (144, 62), (28, 140, 180), -1)
        boxes, _ = HsvBallDetector(config).detect(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR))
        self.assertEqual(len(boxes), 1)
        np.testing.assert_allclose(boxes[0], (30, 43, 54, 67), atol=1)
