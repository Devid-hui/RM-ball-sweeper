import io
import json
import math
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from ball_chassis import Mecanum
from tasks.ball_collect import openrf1
from tasks.ball_collect.run import ROOT, frame_age_ms, load_config, run


class ChassisFeedbackTest(unittest.TestCase):
    def test_follow_ball_sends_zero_positive_then_zero_over_collect1(self):
        args = SimpleNamespace(
            calibration=None, vision="hsv-diameter", follow_ball=True, images=None, serial="test",
            port="test", drive=True, frames=3, output=None,
        )
        config = load_config(ROOT / "configs/collector.json", False)
        image = SimpleNamespace(shape=(1080, 1440, 3))
        camera = Mock()
        camera.read.return_value = (image, None)
        ground = Mock(image_size=(1440, 1080), minimum_candidate_forward_m=0.06)
        ground.position_from_diameter.side_effect = [{"x_m": 0.35, "y_m": 0.0}, {"x_m": 0.34, "y_m": 0.0}]
        detector = Mock()
        detector.detect.side_effect = [([object()], None), ([object()], None), ([], None)]
        serial = Mock()
        serial.read_until.side_effect = [
            b"COLLECT1\n", b"OK\n", b"W 1 0 0 0 0\n", b"W 2 0 0 0 0\n", b"W 3 0 0 0 0\n",
        ]
        with (
            patch.dict("sys.modules", {"ball_camera": SimpleNamespace(HikCamera=Mock(return_value=camera))}),
            patch("tasks.ball_collect.run.GroundProjector", return_value=ground),
            patch("tasks.ball_collect.run.HsvBallDetector", return_value=detector),
            patch.object(openrf1, "open_serial", return_value=serial),
            redirect_stdout(io.StringIO()) as output,
        ):
            run(args, config)
        commands = [call.args[0].split() for call in serial.write.call_args_list]
        self.assertEqual(commands[2][:6], b"W 1 0 0 0 0".split())
        self.assertEqual(commands[3][:2], b"W 2".split())
        self.assertTrue(all(int(wheel) > 0 for wheel in commands[3][2:6]))
        self.assertEqual(commands[4][:6], b"W 3 0 0 0 0".split())
        self.assertEqual(commands[5:], [[b"STOP"]])
        reasons = [json.loads(line)["reason"] for line in output.getvalue().splitlines()]
        self.assertEqual(reasons, ["confirming target", "following ball", "visual target lost"])

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
            config = load_config(path, True)
            self.assertEqual(config["encoder_counts_per_rev"], 1560)
            self.assertEqual(config["wheel_order_abcd"], list(openrf1.LEGACY_CHANNEL_ORDER))
            order = ["fl", "fr", "rl", "rr"]
            path.write_text(json.dumps(dict(settings, wheel_order_abcd=order)))
            self.assertEqual(load_config(path, True)["wheel_order_abcd"], order)
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

    def test_new_wiring_uses_same_channel_order_for_commands_and_feedback(self):
        transport = Mock()
        transport.read_until.side_effect = [b"COLLECT1\n", b"OK\n", b"W 1 0 0 0 0\n"]
        with patch.dict("sys.modules", {"serial": SimpleNamespace(Serial=Mock(return_value=transport))}):
            link = openrf1.OpenRF1("/dev/test", 1000, [1, 1, 1, 1], openrf1.WHEEL_ORDER)
        unit = math.tau / 20000
        link.send([unit, 2 * unit, 3 * unit, 4 * unit])
        transport.write.assert_called_with(openrf1.checked_line("W 1 1 2 3 4"))

        model = Mock()
        model.forward.return_value = (0.0, 0.0, 0.0)
        heading = openrf1.WheelHeading(model, 1000, 6.0, openrf1.WHEEL_ORDER)
        heading.update([100] * 4, 0.0, 0.0)
        heading.update([99, 102, 97, 104], None, 0.1)
        model.forward.assert_called_once_with([k * heading.radians_per_count for k in (1, 2, 3, 4)])

    def test_frame_age_rejects_stale_image(self):
        with patch("tasks.ball_collect.run.time.monotonic_ns", return_value=1_000_000_000):
            self.assertAlmostEqual(frame_age_ms(700_000_000, 0.60), 300)
            with self.assertRaisesRegex(TimeoutError, "过期"):
                frame_age_ms(100_000_000, 0.60)

    def test_factory_command_converts_units_and_respects_channel_order(self):
        order = ["rl", "fr", "fl", "rr"]
        self.assertEqual(
            openrf1.factory_car_command([2, -2, 4, 0], 0.05, order),
            "$Car:0.2,-0.1,0.1,0!",
        )
        command = openrf1.factory_car_command([0, 0, -0.0001, 0], 0.05)
        self.assertEqual(command.encode("ascii"), openrf1.FACTORY_STOP)

    def test_factory_probe_sends_only_zero_and_closes_even_on_bad_reply(self):
        for reply in (b"cmdOk", b""):
            with self.subTest(reply=reply):
                transport = Mock()
                transport.read_until.return_value = reply
                with patch.object(openrf1, "open_serial", return_value=transport):
                    if reply == b"cmdOk":
                        openrf1.check_factory_car_link("ch340")
                    else:
                        with self.assertRaisesRegex(RuntimeError, "cmdOk"):
                            openrf1.check_factory_car_link("ch340")
                transport.write.assert_called_once_with(b"$Car:0,0,0,0!")
                transport.read_until.assert_called_once_with(size=5)
                transport.close.assert_called_once()
