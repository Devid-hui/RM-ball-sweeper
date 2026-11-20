"""Remove the P5 detection path from the project's YOLO26n checkpoint."""

import argparse
from copy import deepcopy
from pathlib import Path

from ultralytics import YOLO
from ultralytics.nn.tasks import DetectionModel


def prune(source, output):
    if output.exists():
        raise FileExistsError(output)
    wrapper = YOLO(source)
    original = wrapper.model
    head = original.model[-1]
    config = deepcopy(original.yaml)
    if head.f != [16, 19, 22] or not head.end2end or original.names != {0: "ball"}:
        raise ValueError("Expected the project's three-scale end-to-end ball detector")

    # Layers 20-22 feed only the P5 output. The deep backbone also feeds P3/P4.
    config["head"] = config["head"][:-4] + [[[16, 19], 1, "Detect", ["nc"]]]
    pruned = DetectionModel(config, verbose=False)
    for target, trained in zip(pruned.model[:-1], original.model[:-4], strict=True):
        target.load_state_dict(trained.state_dict(), strict=True)
    trained_head = head.state_dict()
    new_head = pruned.model[-1]
    new_head.load_state_dict({key: trained_head[key] for key in new_head.state_dict()}, strict=True)
    pruned.names = deepcopy(original.names)
    pruned.args = deepcopy(original.args)
    wrapper.model = pruned.eval()
    # A new architecture starts a new training run, with its own metrics and optimizer.
    wrapper.ckpt = {"epoch": -1, "train_args": pruned.args}
    wrapper.save(output)


if __name__ == "__main__":
    directory = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=directory.parent / "ball_detector_precision/selected_model.pt")
    parser.add_argument("--output", type=Path, default=directory / "ball_p3p4.pt")
    args = parser.parse_args()
    prune(args.source, args.output)
