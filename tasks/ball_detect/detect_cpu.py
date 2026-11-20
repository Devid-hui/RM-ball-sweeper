"""用采球车的固定尺寸 ONNX 模型检测原图中的小球。"""
import argparse
import hashlib
import json
import sys
import time
from contextlib import ExitStack
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort


ASSETS = Path(__file__).resolve().parents[2] / "asset" / "ball_detector_cpu_tuning"


class BallDetector:
    """每个推理线程拥有一个实例，依次处理 BGR8 图像。"""

    def __init__(self, asset_dir=ASSETS, *, threads=None):
        config = json.loads((asset_dir / "candidate_config.json").read_text())
        model = asset_dir / "ball_800.onnx"
        if hashlib.sha256(model.read_bytes()).hexdigest() != config["sha256"]:
            raise ValueError("模型和参数文件的 SHA256 不匹配")
        self.height, self.width = config["input_height"], config["input_width"]
        self.confidence, self.iou = config["confidence"], config["nms_iou"]
        threads = config["threads"] if threads is None else threads
        if threads < 1:
            raise ValueError("线程数必须为正整数")
        options = ort.SessionOptions()
        options.intra_op_num_threads = threads
        options.inter_op_num_threads = 1
        options.add_session_config_entry("session.intra_op.allow_spinning", "0")
        self.session = ort.InferenceSession(str(model), options, providers=["CPUExecutionProvider"])
        model_input, = self.session.get_inputs()
        output, = self.session.get_outputs()
        if model_input.shape != [1, 3, self.height, self.width] or output.shape != [1, 300, 6]:
            raise ValueError(f"不支持的模型输入或输出尺寸 {model_input.shape}, {output.shape}")
        self.canvas = np.empty((self.height, self.width, 3), np.uint8)
        self.tensor = np.empty(model_input.shape, np.float32)
        self.predictions = np.empty(output.shape, np.float32)
        self.binding = self.session.io_binding()
        self.binding.bind_cpu_input(model_input.name, self.tensor)
        self.binding.bind_output(output.name, "cpu", 0, np.float32, output.shape,
                                 self.predictions.ctypes.data)
        self.last_region = None

    def detect(self, image):
        if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3 or not image.size:
            raise ValueError("输入必须是非空 BGR8 图像")
        image_height, image_width = image.shape[:2]
        scale = min(self.width / image_width, self.height / image_height)
        resized_width = max(1, round(image_width * scale))
        resized_height = max(1, round(image_height * scale))
        left = (self.width - resized_width) // 2
        top = (self.height - resized_height) // 2
        region = (left, top, resized_width, resized_height)
        # 固定相机画幅时，灰色填充区只写一次。切换画幅时重新填充。
        if region != self.last_region:
            self.canvas.fill(114)
            self.last_region = region
        content = self.canvas[top:top + resized_height, left:left + resized_width]
        cv2.resize(image, (resized_width, resized_height), dst=content)
        self.tensor[0] = self.canvas[:, :, ::-1].transpose(2, 0, 1)
        self.tensor /= 255.0
        self.session.run_with_iobinding(self.binding)
        rows = self.predictions[0]
        valid = rows[(rows[:, 4] >= self.confidence) & (rows[:, 5] == 0)]
        boxes, scores = valid[:, :4], valid[:, 4]
        boxes[:, ::2] = np.clip((boxes[:, ::2] - left) / scale, 0, image_width)
        boxes[:, 1::2] = np.clip((boxes[:, 1::2] - top) / scale, 0, image_height)
        xywh = np.column_stack((boxes[:, :2], boxes[:, 2:] - boxes[:, :2]))
        keep = np.asarray(cv2.dnn.NMSBoxes(xywh.tolist(), scores.tolist(), 0.0, self.iou),
                          dtype=int).ravel()
        return boxes[keep], scores[keep]


def camera_frames(serial):
    from ball_camera import HikCamera

    camera = HikCamera(serial)
    while True:
        sample = camera.read()
        if sample is not None:
            yield sample


def image_frames(paths):
    for path in paths:
        image = cv2.imread(str(path))
        if image is None:
            raise ValueError(f"无法读取 {path}")
        yield image, None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--serial", help="海康相机序列号")
    source.add_argument("--images", type=Path, nargs="+", help="一个或多个图片文件")
    parser.add_argument("--threads", type=int, help="覆盖模型配置中的 CPU 推理线程数")
    parser.add_argument("--frames", type=int, help="最多处理多少帧，实时模式默认持续运行")
    parser.add_argument("--output", type=Path, help="写入新的 JSONL 文件，每帧一行")
    args = parser.parse_args()
    if args.frames is not None and args.frames < 1:
        parser.error("--frames 必须为正整数")
    if args.threads is not None and args.threads < 1:
        parser.error("--threads 必须为正整数")
    cv2.setNumThreads(1)
    detector = BallDetector(threads=args.threads)
    stream = camera_frames(args.serial) if args.serial else image_frames(args.images)
    with ExitStack() as stack:
        stack.callback(stream.close)
        output = stack.enter_context(args.output.open("x")) if args.output else sys.stdout
        for number, (image, received_ns) in enumerate(stream, 1):
            start = time.monotonic_ns()
            boxes, scores = detector.detect(image)
            end = time.monotonic_ns()
            record = dict(frame=number, received_ns=received_ns, width=image.shape[1], height=image.shape[0],
                          detect_ms=(end - start) / 1e6,
                          frame_age_ms=(end - received_ns) / 1e6 if received_ns is not None else None,
                          detections=[dict(xyxy=box.tolist(), confidence=float(score))
                                      for box, score in zip(boxes, scores)])
            if args.images:
                record["image"] = str(args.images[number - 1])
            print(json.dumps(record), file=output, flush=True)
            if number == args.frames:
                break


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
