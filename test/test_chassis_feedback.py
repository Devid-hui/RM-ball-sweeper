import io
import json
import math
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, call, patch

from ball_chassis import Mecanum
from tasks.ball_collect import openrf1
from tasks.ball_collect.run import ROOT, frame_age_ms, load_config, main, run


class ChassisFeedbackTest(unittest.TestCase):
    def test_default_drive_stops_before_opening_camera_or_serial(self):
        with (
            patch("sys.argv", ["collector", "--serial", "test", "--follow-ball", "--drive"]),
            patch("tasks.ball_collect.run.run") as start,
            redirect_stderr(io.StringIO()) as errors,
            self.assertRaises(SystemExit) as exit_status,
        ):
            main()
        self.assertEqual(exit_status.exception.code, 2)
        self.assertIn("C26", errors.getvalue())
        start.assert_not_called()

    def test_follow_ball_sends_zero_positive_then_zero_over_collect1(self):
        args = SimpleNamespace(
            calibration=None, vision="hsv-diameter", follow_ball=True, images=None, serial="test",
            port="test", drive=True, protocol="collect1", frames=3, output=None,
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

    def test_c26_cli_completes_delivery_and_resumes_lane_using_encoder_turns(self):
        config = load_config(ROOT / "configs/collector.json", False)
        config.update(lane_starts=[[1.1, 0.0]], drive_calibrated=True, c26_watchdog_verified=True)
        stop_x = 0.23 + config["fork_forward_m"] + 0.017
        front = math.pi
        start = (1.1, 0.0, front)
        intercept = (0.9, 0.0, front)
        intercept_ball = (intercept, True)
        # Each row supplies a wall pose, a ball observation and the raw encoder offset.
        scene = [
            (start, False, 0), ((1.0, 0.0, front), False, 0),
            (*intercept_ball, 0), (*intercept_ball, 0), (*intercept_ball, 0),
            (None, False, 0), (intercept, False, 0),
            ((stop_x, 0.0, front), False, 0), ((stop_x + 0.05, 0.0, front), False, 0),
            ((stop_x + config["reverse_m"], 0.0, front), False, 0),
        ]
        # All four raw counters increase for a left turn. Give the first turn
        # one frame with no motion, then reverse the counts to face the wall again.
        quarter_counts = round(config["encoder_counts_per_rev"] * 0.25 / config["wheel_radius_m"] / 4)
        turn_counts = [round(quarter_counts * step / 10) for step in range(11)]
        scene.extend((None, False, count) for count in turn_counts)
        scene.extend((None, False, count) for count in reversed(turn_counts))
        scene.extend([(intercept, False, 0), ((0.4, 0.0, front), False, 0)])

        image = SimpleNamespace(shape=(1080, 1440, 3))
        camera = Mock()
        camera.read.return_value = (image, None)
        ground = Mock(image_size=(1440, 1080), minimum_candidate_forward_m=0.06)
        ground.position_from_diameter.return_value = {"x_m": 0.10, "y_m": 0.0}
        detector = Mock()
        detector.detect.side_effect = [([object()] if ball else [], None) for _, ball, _ in scene]
        tag = Mock()
        tag.pose.side_effect = [pose for pose, _, _ in scene]
        transport = Mock()
        replies = []
        for index, (_, _, offset) in enumerate(scene):
            sample = (2**32 - 290 + index * 20) % 2**32
            count = (2**32 - 100 + offset) % 2**32
            replies.append(f"ENC {sample} {count} {count} {count} {count}\r\n".encode("ascii"))
        feedback = iter(replies)
        transport.read_until.side_effect = lambda size: b"cmdOk" if size == 5 else next(feedback)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "vehicle.json"
            path.write_text(json.dumps(config))
            with (
                patch("sys.argv", ["collector", "--serial", "test", "--drive", "--config", str(path)]),
                patch.dict("sys.modules", {"ball_camera": SimpleNamespace(HikCamera=Mock(return_value=camera))}),
                patch("tasks.ball_collect.run.GroundProjector", return_value=ground),
                patch("tasks.ball_collect.run.HsvBallDetector", return_value=detector),
                patch("tasks.ball_collect.run.WallTag", return_value=tag),
                patch.object(openrf1, "open_serial", return_value=transport) as open_link,
                redirect_stdout(io.StringIO()) as output,
            ):
                main()

        open_link.assert_called_once_with("ch340")
        rows = [json.loads(line) for line in output.getvalue().splitlines()]
        by_reason = {row["reason"]: (i, row) for i, row in enumerate(rows)}
        self.assertEqual(len(rows), len(scene))
        self.assertEqual(rows[-1]["phase"], "finished")
        self.assertEqual(rows[-1]["releases"], 1)
        self.assertIsNone(by_reason["quarter turn complete"][1]["tag_pose"])
        self.assertAlmostEqual(by_reason["quarter turn complete"][1]["heading_rad"], -math.pi / 2, delta=0.05)
        self.assertGreater(by_reason["quarter turn"][1]["velocity"][2], 0)
        self.assertLess(by_reason["face wall tag"][1]["velocity"][2], 0)
        self.assertEqual(by_reason["wall tag unavailable"][1]["velocity"], [0, 0, 0])
        commands = [entry.args[0] for entry in transport.write.call_args_list]
        self.assertEqual(commands[0], openrf1.FACTORY_STOP)
        self.assertEqual(commands[1:-1:2], [b"$ENC!"] * len(scene))
        self.assertEqual(commands[2 + 2 * by_reason["push ball"][0]], b"$Car:0.05,0.05,0.05,0.05!")
        self.assertEqual(commands[-2:], [openrf1.FACTORY_STOP] * 2)
        transport.close.assert_called_once()

    def test_c26_sends_stop_if_command_acknowledgment_is_missing(self):
        args = SimpleNamespace(
            calibration=None, vision="hsv-diameter", follow_ball=True, images=None, serial="test",
            port="test", drive=True, protocol="c26", frames=2, output=None,
        )
        config = load_config(ROOT / "configs/collector.json", False)
        ground = Mock(image_size=(1440, 1080), minimum_candidate_forward_m=0.06)
        ground.position_from_diameter.return_value = {"x_m": 0.35, "y_m": 0.0}
        detector = Mock()
        detector.detect.return_value = ([object()], None)
        transport = Mock()
        transport.read_until.side_effect = [
            b"cmdOk", b"ENC 100 0 0 0 0\r\n", b"cmdOk", b"ENC 101 0 0 0 0\r\n", b"",
        ]
        frame = (SimpleNamespace(shape=(1080, 1440, 3)), None)
        with (
            patch("tasks.ball_collect.run.collection_frames", return_value=(item for item in [frame, frame])),
            patch("tasks.ball_collect.run.GroundProjector", return_value=ground),
            patch("tasks.ball_collect.run.HsvBallDetector", return_value=detector),
            patch.object(openrf1, "open_serial", return_value=transport),
            redirect_stdout(io.StringIO()),
            self.assertRaisesRegex(RuntimeError, "cmdOk"),
        ):
            run(args, config)
        stop = call(openrf1.FACTORY_STOP)
        query = call(b"$ENC!")
        move = call(b"$Car:0.05,0.05,0.05,0.05!")
        self.assertEqual(transport.write.call_args_list, [stop, query, stop, query, move, stop])
        transport.close.assert_called_once()

    def test_c26_rejects_frozen_encoder_sample(self):
        transport = Mock()
        transport.read_until.side_effect = [b"cmdOk", *([b"ENC 100 0 0 0 0\r\n"] * 3)]
        with patch.object(openrf1, "open_serial", return_value=transport):
            chassis = openrf1.C26("test", {})
        with patch.object(openrf1, "monotonic", side_effect=[0.0, 0.01, 0.16]):
            self.assertEqual(chassis.read_feedback(), ([0, 0, 0, 0], 100))
            self.assertEqual(chassis.read_feedback(), ([0, 0, 0, 0], 100))
            with self.assertRaisesRegex(TimeoutError, "采样号"):
                chassis.read_feedback()
        chassis.close()

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
