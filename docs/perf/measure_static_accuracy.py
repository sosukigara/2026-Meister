"""Measure class accuracy and IoU on a hand-labelled still image.

This is a KNOWN-OBJECT REGRESSION, not COCO mAP: the image and the ground-truth
boxes are fixed by hand, and the detector under test is the very same RKNN
model/postprocessing used by the live node.  Ground truth is independent of the
model output (boxes were drawn by looking at the image), so the comparison is
not circular.  INT8 quantisation makes most scores sit at 0.5; that is a known
property of this model and is reported, not hidden.

    python3 docs/perf/measure_static_accuracy.py \
        --model ros/src/umeonigiri_vision/models/yolov8n-zoo.rknn \
        --image docs/perf/bus.jpg --out docs/perf/logs/static_accuracy.json

The decode mirrors src/umeonigiri_vision/src/rknn_yolo.cpp: letterbox with pad
114, DFL expected value over 16 bins per side, per-class argmax, NMS at 0.45.
RKNNLite returns already-dequantised float32 tensors, so the explicit Dequant()
in C++ is implicit here.
"""
import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np

# Hand-labelled from the displayed images, NOT copied from detector output.
# [class, x0, y0, x1, y1] in source-image pixels.
GROUND_TRUTH = {
    "bus.jpg": [
        ["person", 49, 397, 244, 904],
        ["person", 222, 405, 352, 857],
        ["person", 669, 382, 810, 882],
        ["bus", 19, 232, 795, 756],
    ],
    "person_dog_horse.jpg": [
        ["person", 190, 98, 275, 375],
        ["dog", 68, 262, 205, 347],
        ["horse", 405, 140, 600, 342],
    ],
}
PAD_VALUE = 114
DFL_BINS = 16
CONF = 0.35
IOU_NMS = 0.45
IOU_MATCH = 0.5


def labels_from(path):
    return [line.strip() for line in Path(path).read_text(encoding="utf-8").splitlines()
            if line.strip()]


def letterbox(image, size):
    scale = min(size / image.shape[1], size / image.shape[0])
    new_w = int(image.shape[1] * scale)
    new_h = int(image.shape[0] * scale)
    pad_x = (size - new_w) // 2
    pad_y = (size - new_h) // 2
    canvas = np.full((size, size, 3), PAD_VALUE, dtype=np.uint8)
    canvas[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = cv2.resize(
        image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    return cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB), scale, pad_x, pad_y


def decode(outputs, size, conf, iou_thres):
    boxes = []
    for branch in range(3):
        reg = outputs[branch * 3][0]      # (64, H, W)
        cls = outputs[branch * 3 + 1][0]  # (80, H, W)
        _, grid_h, grid_w = cls.shape
        stride_x = size / grid_w
        stride_y = size / grid_h
        best = cls.argmax(axis=0)
        best_raw = cls[best, np.arange(grid_h)[:, None], np.arange(grid_w)[None, :]]
        ys, xs = np.where(best_raw >= conf)
        for gy, gx in zip(ys.tolist(), xs.tolist()):
            dist = []
            for side in range(4):
                values = reg[side * DFL_BINS:(side + 1) * DFL_BINS, gy, gx]
                values = np.exp(values - values.max())
                dist.append(float((values / values.sum() *
                                   np.arange(DFL_BINS)).sum()))
            boxes.append({
                "x0": (gx + 0.5 - dist[0]) * stride_x,
                "y0": (gy + 0.5 - dist[1]) * stride_y,
                "x1": (gx + 0.5 + dist[2]) * stride_x,
                "y1": (gy + 0.5 + dist[3]) * stride_y,
                "score": float(best_raw[gy, gx]),
                "class_id": int(best[gy, gx]),
            })
    boxes.sort(key=lambda box: -box["score"])
    kept = []
    removed = [False] * len(boxes)
    for i, box in enumerate(boxes):
        if removed[i]:
            continue
        kept.append(box)
        for j in range(i + 1, len(boxes)):
            other = boxes[j]
            if removed[j] or box["class_id"] != other["class_id"]:
                continue
            if iou(box, other) > iou_thres:
                removed[j] = True
    return kept


def iou(a, b):
    x0 = max(a["x0"], b["x0"])
    y0 = max(a["y0"], b["y0"])
    x1 = min(a["x1"], b["x1"])
    y1 = min(a["y1"], b["y1"])
    inter = max(0.0, x1 - x0) * max(0.0, y1 - y0)
    area_a = max(0.0, a["x1"] - a["x0"]) * max(0.0, a["y1"] - a["y0"])
    area_b = max(0.0, b["x1"] - b["x0"]) * max(0.0, b["y1"] - b["y0"])
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def to_source_coords(box, scale, pad_x, pad_y, shape):
    return {
        "x0": max(0.0, min((box["x0"] - pad_x) / scale, shape[1] - 1)),
        "y0": max(0.0, min((box["y0"] - pad_y) / scale, shape[0] - 1)),
        "x1": max(0.0, min((box["x1"] - pad_x) / scale, shape[1] - 1)),
        "y1": max(0.0, min((box["y1"] - pad_y) / scale, shape[0] - 1)),
        "score": round(box["score"], 4),
    }


def evaluate(predictions, ground_truth, labels):
    pairs = []
    used = set()
    for gt_index, gt in enumerate(ground_truth):
        gt_box = {"x0": gt[1], "y0": gt[2], "x1": gt[3], "y1": gt[4]}
        best_iou = 0.0
        best_index = None
        for index, pred in enumerate(predictions):
            if index in used:
                continue
            overlap = iou(gt_box, pred)
            if overlap > best_iou:
                best_iou = overlap
                best_index = index
        matched = best_iou >= IOU_MATCH
        class_ok = bool(matched and labels[predictions[best_index]["class_id"]] == gt[0])
        if matched:
            used.add(best_index)
        pairs.append({"gt_class": gt[0], "gt_box": gt_box,
                      "iou": round(best_iou, 4),
                      "pred_class": (labels[predictions[best_index]["class_id"]]
                                     if matched else None),
                      "pred_score": (predictions[best_index]["score"]
                                     if matched else None),
                      "matched": matched, "class_ok": class_ok})
    false_positive = [{"class": labels[p["class_id"]], "score": round(p["score"], 4),
                       "box": [round(p[k], 1) for k in ("x0", "y0", "x1", "y1")]}
                      for i, p in enumerate(predictions) if i not in used]
    return {
        "ground_truth_count": len(ground_truth),
        "prediction_count": len(predictions),
        "matched_count": sum(p["matched"] for p in pairs),
        "class_correct_count": sum(p["class_ok"] for p in pairs),
        "class_accuracy": (round(sum(p["class_ok"] for p in pairs) /
                                 len(ground_truth), 4) if ground_truth else None),
        "mean_iou_matched": (round(math.fsum(p["iou"] for p in pairs if p["matched"]) /
                                   max(1, sum(p["matched"] for p in pairs)), 4)),
        "false_positive_count": len(false_positive),
        "pairs": pairs,
        "false_positives": false_positive,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True)
    parser.add_argument("--image", required=True)
    parser.add_argument("--labels", default="ros/src/umeonigiri_vision/config/coco_labels.txt")
    parser.add_argument("--conf", type=float, default=CONF,
                        help="detection confidence threshold")
    parser.add_argument("--out", default="/tmp/static_accuracy.json")
    args = parser.parse_args()

    from rknnlite.api import RKNNLite
    labels = labels_from(args.labels)
    image = cv2.imread(args.image, cv2.IMREAD_COLOR)
    if image is None:
        raise SystemExit(f"画像を読めません: {args.image}")
    canvas, scale, pad_x, pad_y = letterbox(image, 640)

    runtime = RKNNLite()
    if runtime.load_rknn(args.model) != 0 or \
            runtime.init_runtime(core_mask=RKNNLite.NPU_CORE_0) != 0:
        raise SystemExit("RKNN の初期化に失敗しました")
    try:
        outputs = runtime.inference([canvas[None, ...]])
    finally:
        runtime.release()
    if outputs is None:
        raise SystemExit("推論に失敗しました")

    raw_boxes = decode(outputs, 640, args.conf, IOU_NMS)
    predictions = []
    for box in raw_boxes:
        source = to_source_coords(box, scale, pad_x, pad_y, image.shape)
        source["class_id"] = box["class_id"]
        source["label"] = labels[box["class_id"]]
        predictions.append(source)

    name = Path(args.image).name
    gt = GROUND_TRUTH.get(name)
    if gt is None:
        raise SystemExit(f"この画像の手作業 GT が未定義です: {name}")
    result = {
        "definition": "Known-object regression on a hand-labelled still image, NOT COCO mAP",
        "model": str(Path(args.model).resolve()),
        "model_output_used_as_ground_truth": False,
        "image": str(Path(args.image).resolve()),
        "image_size": [image.shape[1], image.shape[0]],
        "input_size": 640, "conf": args.conf, "iou_nms": IOU_NMS, "iou_match": IOU_MATCH,
        "evaluation": evaluate(predictions, gt, labels),
        "predictions": predictions,
    }
    Path(args.out).write_text(json.dumps(result, ensure_ascii=False, indent=2),
                              encoding="utf-8")
    print(json.dumps(result["evaluation"], ensure_ascii=False, indent=2))
    print(f"-> {args.out}")


if __name__ == "__main__":
    main()
