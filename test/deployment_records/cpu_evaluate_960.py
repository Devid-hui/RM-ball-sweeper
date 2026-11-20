import argparse
import csv
import json
import time
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort
import psutil

HEIGHT, WIDTH = 736, 960


def make_input(image):
    height, width = image.shape[:2]
    scale = min(HEIGHT / height, WIDTH / width)
    resized = cv2.resize(image, (round(width * scale), round(height * scale)))
    padding_width = WIDTH - resized.shape[1]
    padding_height = HEIGHT - resized.shape[0]
    left, top = round(padding_width / 2 - 0.1), round(padding_height / 2 - 0.1)
    padded = cv2.copyMakeBorder(resized, top, padding_height - top, left, padding_width - left,
                                cv2.BORDER_CONSTANT, value=(114, 114, 114))
    tensor = np.ascontiguousarray(padded[:, :, ::-1].transpose(2, 0, 1)[None], dtype=np.float32)
    tensor /= 255.0
    return tensor, (scale, left, top, width, height)


def read_image(path):
    image = cv2.imread(str(path))
    if image is None:
        raise ValueError(f"Cannot read {path}")
    return image


def load_model(model, threads):
    options = ort.SessionOptions()
    options.intra_op_num_threads = threads
    options.inter_op_num_threads = 1
    options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    options.add_session_config_entry("session.intra_op.allow_spinning", "0")
    session = ort.InferenceSession(str(model), sess_options=options, providers=["CPUExecutionProvider"])
    assert session.get_inputs()[0].shape == [1, 3, HEIGHT, WIDTH]
    return session


def detect(session, image):
    tensor, (scale, left, top, width, height) = make_input(image)
    predictions = session.run(None, {session.get_inputs()[0].name: tensor})[0][0]
    assert predictions.shape == (300, 6)
    selected = predictions[(predictions[:, 4] >= 0.30) & (predictions[:, 5] == 0)]
    boxes, scores = selected[:, :4].copy(), selected[:, 4]
    boxes[:, 0::2] = np.clip((boxes[:, 0::2] - left) / scale, 0, width)
    boxes[:, 1::2] = np.clip((boxes[:, 1::2] - top) / scale, 0, height)
    xywh = np.column_stack((boxes[:, :2], boxes[:, 2:] - boxes[:, :2]))
    keep = np.asarray(cv2.dnn.NMSBoxes(xywh.tolist(), scores.tolist(), 0.0, 0.7)).astype(int)
    return boxes[keep], scores[keep]


def match(boxes, scores, truth):
    available = set(range(len(truth)))
    matched = 0
    for index in np.argsort(-scores):
        if not available:
            break
        candidate = boxes[index]
        candidates = sorted(available)
        target = truth[candidates]
        sides = np.maximum(np.minimum(candidate[2:], target[:, 2:]) -
                           np.maximum(candidate[:2], target[:, :2]), 0)
        intersection = np.prod(sides, axis=1)
        union = np.prod(candidate[2:] - candidate[:2]) + np.prod(target[:, 2:] - target[:, :2], axis=1) - intersection
        overlap = intersection / np.maximum(union, 1e-6)
        best = int(np.argmax(overlap))
        if overlap[best] >= 0.5:
            available.remove(candidates[best])
            matched += 1
    return matched


def evaluate(session, manifest):
    rows = json.loads(manifest.read_text())
    counts = dict(images=0, tp=0, fp=0, fn=0, positives=0, positives_detected=0, negatives=0, negatives_with_boxes=0)
    for row in rows:
        image = read_image(manifest.parent / row["image"])
        truth = np.array(row["boxes"], dtype=np.float32).reshape(-1, 4)
        truth *= [image.shape[1], image.shape[0], image.shape[1], image.shape[0]]
        boxes, scores = detect(session, image)
        matched = match(boxes, scores, truth)
        counts["images"] += 1
        counts["tp"] += matched
        counts["fp"] += len(boxes) - matched
        counts["fn"] += len(truth) - matched
        counts["positives"] += bool(len(truth))
        counts["positives_detected"] += bool(matched)
        counts["negatives"] += not len(truth)
        counts["negatives_with_boxes"] += not len(truth) and bool(len(boxes))
    counts["precision"] = counts["tp"] / max(1, counts["tp"] + counts["fp"])
    counts["recall"] = counts["tp"] / max(1, counts["tp"] + counts["fn"])
    print(json.dumps(counts), flush=True)


def benchmark(session, manifest, seconds, report):
    rows = json.loads(manifest.read_text())
    images = [read_image(manifest.parent / row["image"]) for row in rows]
    process = psutil.Process()
    for _ in range(4):
        detect(session, images[0])
    zones = []
    for path in Path("/sys/class/thermal").glob("thermal_zone*"):
        if "core-thermal" in (path / "type").read_text():
            zones.append(path / "temp")
    cpu_start = sum(process.cpu_times()[:2])
    start = time.perf_counter()
    elapsed = 0.0
    timings = []
    while elapsed < seconds:
        before = time.perf_counter()
        detect(session, images[len(timings) % len(images)])
        end = time.perf_counter()
        timings.append(1000 * (end - before))
        elapsed = end - start
    metrics = dict(seconds=elapsed, frames=len(timings), fps=len(timings)/elapsed,
                   p50_ms=float(np.percentile(timings, 50)), p95_ms=float(np.percentile(timings, 95)),
                   cpu_pct=100*(sum(process.cpu_times()[:2])-cpu_start)/elapsed,
                   rss_mib=process.memory_info().rss/1048576,
                   temp_c=max((float(p.read_text())/1000 for p in zones), default=None))
    report.write_text(json.dumps(metrics, indent=2) + "\n")
    print(json.dumps(metrics), flush=True)


if __name__ == "__main__":
    command = argparse.ArgumentParser()
    command.add_argument("model", type=Path)
    command.add_argument("manifest", type=Path)
    command.add_argument("--threads", type=int, default=4)
    command.add_argument("--seconds", type=int, default=0)
    command.add_argument("--report", type=Path)
    args = command.parse_args()
    cv2.setNumThreads(1)
    session = load_model(args.model, args.threads)
    print(f"ORT {ort.__version__} {session.get_providers()} threads={args.threads} model={args.model}", flush=True)
    if args.seconds:
        benchmark(session, args.manifest, args.seconds, args.report)
    else:
        evaluate(session, args.manifest)
