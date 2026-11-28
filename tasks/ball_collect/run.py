"""采球主入口。图片回放和相机预演输出轮速，实车模式通过 UART 下发。"""
import argparse
import json
import signal
import sys
import time
from contextlib import ExitStack
from functools import partial
from math import isfinite, tau
from pathlib import Path

import cv2

from tasks.ball_collect.navigation import BallFollower, Navigator
from tasks.ball_collect.openrf1 import LEGACY_CHANNEL_ORDER, WHEEL_ORDER, factory_car_command
from tasks.ball_collect.wall_tag import WallTag
from tasks.ball_detect.detect_cpu import BallDetector, image_frames
from tasks.ball_detect.ground_position import BALL_RADIUS_M, GroundProjector
from tasks.ball_detect.hsv import HsvBallDetector


ROOT = Path(__file__).resolve().parents[2]


def load_config(path, live):
    config = json.loads((ROOT / "configs/collector.json").read_text())
    settings = json.loads(path.read_text())
    config.update(settings)
    for key in ("wheel_radius_m", "wheelbase_m", "track_m", "fork_forward_m", "fork_width_m",
                "tag_size_m", "search_speed_m_s", "approach_speed_m_s", "push_speed_m_s",
                "visual_approach_speed_m_s", "visual_standoff_m", "push_entry_gap_m", "reverse_m",
                "max_wheel_rad_s", "max_frame_age_s", "camera_startup_timeout_s",
                "run_limit_s", "turn_timeout_s", "diameter_scale"):
        if not isfinite(config[key]) or config[key] <= 0:
            raise ValueError(f"{key} 须为正数")
    if config["fork_width_m"] <= 2 * BALL_RADIUS_M:
        raise ValueError("前叉内宽必须大于球径")
    if config["visual_standoff_m"] <= BALL_RADIUS_M:
        raise ValueError("无墙标时，球心与前叉前缘的预留距离须大于球半径")
    xmin, ymin, xmax, ymax = config["drop_zone"]
    if (not all(map(isfinite, (xmin, ymin, xmax, ymax))) or xmin <= 0
            or min(xmax - xmin, ymax - ymin) <= 2 * BALL_RADIUS_M):
        raise ValueError("drop_zone 须为有效的 [xmin, ymin, xmax, ymax]")
    end = config["lane_end_x_m"]
    if not isfinite(end) or end <= config["fork_forward_m"]:
        raise ValueError("lane_end_x_m 须大于前叉前伸长度")
    if not config["lane_starts"] or any(
        not isfinite(x) or not isfinite(y) or x <= end for x, y in config["lane_starts"]
    ):
        raise ValueError("lane_starts 须填写位于巡线终点后方的起点")
    if type(config["tag_id"]) is not int or not 0 <= config["tag_id"] < 587:
        raise ValueError("tag_id 超出 AprilTag 36h11 范围")
    if type(config["max_deliveries"]) is not int or config["max_deliveries"] < 1:
        raise ValueError("max_deliveries 须为正整数")
    if type(config["turn_direction"]) is not int or abs(config["turn_direction"]) != 1:
        raise ValueError("turn_direction 须为 1（左转）或 -1（右转）")
    if live:
        # 旧的实车配置已经要求用户显式填写这两项；预演的示例值不能驱动车轮。
        counts, signs = settings.get("encoder_counts_per_rev"), settings.get("drive_sign_abcd")
        if settings.get("drive_calibrated", counts is not None and signs is not None) is not True:
            raise ValueError("实测每圈计数和轮向后，在实车配置中设置 drive_calibrated 为 true")
        if type(counts) is not int or counts <= 0:
            raise ValueError("实车模式需要填写每轮完整一圈的 encoder_counts_per_rev")
        if not isinstance(signs, list) or len(signs) != 4 or any(
            type(sign) is not int or abs(sign) != 1 for sign in signs
        ):
            raise ValueError("实车模式需要填写 A、B、C、D 的 drive_sign_abcd，每项为 -1 或 1")
        if config["max_wheel_rad_s"] * counts * 20 / tau > 1000000:
            raise ValueError("轮速上限和编码器分辨率超过固件指令范围")
        # 老的实测文件没有轮序字段，按当时使用的接线解释。
        config["wheel_order_abcd"] = settings.get("wheel_order_abcd", list(LEGACY_CHANNEL_ORDER))
    order = config["wheel_order_abcd"]
    if not isinstance(order, list) or len(order) != 4 or any(order.count(wheel) != 1 for wheel in WHEEL_ORDER):
        raise ValueError("wheel_order_abcd 须按 A/B/C/D 顺序填入 fl、fr、rl、rr 各一次")
    return config


def frame_age_ms(received_ns, limit_s):
    if received_ns is None:
        return None
    age = (time.monotonic_ns() - received_ns) / 1e9
    if not 0 <= age <= limit_s:
        raise TimeoutError("相机帧已过期，停车")
    return age * 1000


def run(args, config):
    from ball_chassis import Mecanum

    cv2.setNumThreads(1)
    ground = GroundProjector(args.calibration)
    vision = args.vision or config["vision"]
    if vision == "hsv-diameter":
        detector = HsvBallDetector(config)
        locate = partial(ground.position_from_diameter, scale=config["diameter_scale"])
    elif vision == "onnx-ground":
        detector = BallDetector(threads=args.threads)
        locate = ground.position
    else:
        raise ValueError(f"未知的 vision 配置 {vision!r}")
    follower = navigator = tag = None
    if args.follow_ball:
        follower = BallFollower(config, ground.minimum_candidate_forward_m)
    else:
        navigator = Navigator(config, ground.minimum_candidate_forward_m)
        tag = WallTag(ground, config["tag_id"], config["tag_size_m"])
    mecanum = Mecanum(config["wheel_radius_m"], config["wheelbase_m"], config["track_m"])

    with ExitStack() as stack:
        output = stack.enter_context(args.output.open("x")) if args.output else sys.stdout
        if args.images:
            frames = image_frames(args.images, raw_orientation=True)
            read = lambda: next(frames, None)
        else:
            from ball_camera import HikCamera
            camera = HikCamera(args.serial)
            read = camera.read
        chassis = None
        heading = None
        if args.drive:
            from tasks.ball_collect.openrf1 import OpenRF1, WheelHeading
            order = config["wheel_order_abcd"]
            chassis = OpenRF1(args.port, config["encoder_counts_per_rev"], config["drive_sign_abcd"], order)
            stack.callback(chassis.close)
            if navigator is not None:
                heading = WheelHeading(mecanum, config["encoder_counts_per_rev"], config["max_wheel_rad_s"], order)
        start = last_frame = time.monotonic()
        number = 0
        while args.frames is None or number < args.frames:
            if not args.images and time.monotonic() - start >= config["run_limit_s"]:
                break
            sample = read()
            if sample is None:
                if args.images:
                    break
                timeout_key = "max_frame_age_s" if number else "camera_startup_timeout_s"
                if time.monotonic() - last_frame > config[timeout_key]:
                    raise TimeoutError("相机帧超时，停车" if number else "等待相机首帧超时")
                continue
            image, received_ns = sample
            last_frame = time.monotonic()
            frame_age_ms(received_ns, config["max_frame_age_s"])
            if (image.shape[1], image.shape[0]) != ground.image_size:
                raise ValueError(f"图像是 {image.shape[1]}×{image.shape[0]}，与标定 {ground.image_size} 不符")
            counts = chassis.read_counts() if heading is not None else None
            sampled_at = time.monotonic()
            boxes, _ = detector.detect(image)
            positions = [locate(box) for box in boxes]
            balls = [(point["x_m"], point["y_m"]) for point in positions if point is not None]
            pose = tag.pose(image) if tag is not None else None
            wheel_yaw = None
            if heading is not None:
                wheel_yaw = heading.update(counts, pose[2] if pose is not None else None, sampled_at)
            if follower is not None:
                velocity, reason = follower.step(balls)
                phase, releases = "follow_ball", 0
            else:
                velocity, reason = navigator.step(pose, balls, wheel_yaw=wheel_yaw)
                phase, releases = navigator.phase, navigator.deliveries
            wheels = mecanum.inverse(velocity, config["max_wheel_rad_s"])
            age_ms = frame_age_ms(received_ns, config["max_frame_age_s"])
            counts = chassis.send(wheels) if chassis is not None else None
            number += 1
            print(json.dumps(dict(frame=number, vision=vision, detections=len(boxes), balls_m=balls, tag_pose=pose,
                                  phase=phase, reason=reason, velocity=velocity, wheels=wheels,
                                  factory_car_preview=factory_car_command(
                                      wheels, config["wheel_radius_m"], config["wheel_order_abcd"],
                                  ) if not args.drive else None,
                                  encoder_abcd=counts, heading_rad=wheel_yaw, releases=releases,
                                  frame_age_ms=age_ms)),
                  file=output, flush=True)
            if phase == "finished":
                break
        if number == 0:
            raise RuntimeError("没有处理到图像")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--images", nargs="+", type=Path)
    source.add_argument("--serial", metavar="CAMERA_ID")
    parser.add_argument("--drive", action="store_true", help="向实车下发轮速，须使用海康相机")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--mission", action="store_true", help="兼容原有命令，默认使用完整任务逻辑")
    mode.add_argument("--follow-ball", action="store_true", help="仅用相机跟随球，到前叉预留距离即停车")
    parser.add_argument("--port", default="/dev/ttyS4", help="串口设备，或 ch340（通过 libusb）")
    parser.add_argument("--config", type=Path, default=ROOT / "configs/collector.json")
    parser.add_argument("--calibration", type=Path, default=ROOT / "configs/hikrobot_DA6415777.yaml")
    parser.add_argument("--threads", type=int)
    parser.add_argument("--vision", choices=["hsv-diameter", "onnx-ground"], help="覆盖配置中的识球和测距方案")
    parser.add_argument("--frames", type=int)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.drive and args.images:
        parser.error("图片回放不能驱动车轮")
    if args.frames is not None and args.frames < 1:
        parser.error("--frames 须为正整数")
    run(args, load_config(args.config, args.drive))


def stop_on_signal(_signum, _frame):
    raise KeyboardInterrupt


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, stop_on_signal)
    try:
        main()
    except KeyboardInterrupt:
        pass
    except (ValueError, OSError, RuntimeError, ImportError) as error:
        print(error, file=sys.stderr)
        raise SystemExit(1)
