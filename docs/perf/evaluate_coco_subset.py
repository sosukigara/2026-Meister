"""Evaluate RKNN detector on a COCO val2017 subset with official GT boxes.

    python3 docs/perf/evaluate_coco_subset.py \
      --annotations /tmp/coco_eval/instances_val2017.json \
      --images /tmp/coco_eval --ids /tmp/coco_eval/ids.txt \
      --model <yolov8n-zoo.rknn> --conf 0.45

Ground truth comes from the official COCO annotations, never from model output.
A detection matches a GT box when IoU >= 0.5 and the class label agrees.
"""
import argparse
import importlib.util
import json
from pathlib import Path

import cv2
import numpy as np
from rknnlite.api import RKNNLite


def load_helper(path):
    spec = importlib.util.spec_from_file_location("static_acc", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def to_source(box, scale, pad_x, pad_y):
    return {
        "x0": (box["x0"] - pad_x) / scale,
        "y0": (box["y0"] - pad_y) / scale,
        "x1": (box["x1"] - pad_x) / scale,
        "y1": (box["y1"] - pad_y) / scale,
        "score": box["score"],
        "class_id": box["class_id"],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--annotations", required=True)
    parser.add_argument("--images", required=True)
    parser.add_argument("--ids", required=True)
    parser.add_argument("--model", required=True)
    parser.add_argument("--labels", default="ros/src/umeonigiri_vision/config/coco_labels.txt")
    parser.add_argument("--conf", type=float, default=0.45)
    parser.add_argument("--iou-nms", type=float, default=0.45)
    parser.add_argument("--tiles", type=int, default=1,
                        help="split each image into N x N tiles before inference")
    parser.add_argument("--out", default="/tmp/coco_eval_summary.json")
    args = parser.parse_args()

    helper = load_helper(Path(__file__).with_name("measure_static_accuracy.py"))
    labels = [line.strip() for line in Path(args.labels).read_text(encoding="utf-8").splitlines()
              if line.strip()]
    ann = json.loads(Path(args.annotations).read_text(encoding="utf-8"))
    meta = {image["id"]: image for image in ann["images"]}
    cats = {cat["id"]: cat["name"] for cat in ann["categories"]}
    by_image = {}
    for a in ann["annotations"]:
        if a.get("iscrowd", 0) == 0 and a["bbox"][2] > 0 and a["bbox"][3] > 0:
            by_image.setdefault(a["image_id"], []).append(a)

    ids = [int(line) for line in Path(args.ids).read_text().splitlines() if line.strip()]
    runtime = RKNNLite()
    assert runtime.load_rknn(args.model) == 0
    assert runtime.init_runtime(core_mask=RKNNLite.NPU_CORE_0) == 0

    stats = {"images": 0, "gt": 0, "predictions": 0, "tp": 0, "fp": 0, "fn": 0,
             "class_correct": 0, "by_area": {}, "per_image": []}

    def bucket(area):
        if area < 32 * 32:
            return "small(<1024px2)"
        if area < 96 * 96:
            return "medium(<9216px2)"
        return "large"

    def area_of(box):
        return max(0.0, box["x1"] - box["x0"]) * max(0.0, box["y1"] - box["y0"])
    for image_id in ids:
        info = meta[image_id]
        image = cv2.imread(str(Path(args.images) / info["file_name"]), cv2.IMREAD_COLOR)
        if image is None:
            print(f"missing {info['file_name']}")
            continue
        preds = []
        tiles = args.tiles
        tile_h, tile_w = image.shape[0] // tiles, image.shape[1] // tiles
        for ty in range(tiles):
            for tx in range(tiles):
                x0, y0 = tx * tile_w, ty * tile_h
                x1 = image.shape[1] if tx == tiles - 1 else (tx + 1) * tile_w
                y1 = image.shape[0] if ty == tiles - 1 else (ty + 1) * tile_h
                tile = image[y0:y1, x0:x1]
                canvas, scale, pad_x, pad_y = helper.letterbox(tile, 640)
                outputs = runtime.inference(inputs=[canvas[None, :, :, :]], data_format="nhwc")
                for box in helper.decode(outputs, 640, args.conf, args.iou_nms):
                    local = to_source(box, scale, pad_x, pad_y)
                    local["x0"] += x0
                    local["x1"] += x0
                    local["y0"] += y0
                    local["y1"] += y0
                    preds.append(local)
        if tiles > 1:
            preds.sort(key=lambda p: -p["score"])
            kept = []
            for p in preds:
                if all(helper.iou(p, k) <= args.iou_nms or
                       p["class_id"] != k["class_id"] for k in kept):
                    kept.append(p)
            preds = kept
        gt = []
        for a in by_image.get(image_id, []):
            x, y, w, h = a["bbox"]
            gt.append({"x0": x, "y0": y, "x1": x + w, "y1": y + h,
                       "class_id": cats[a["category_id"]]})
        matched = set()
        class_ok = 0
        for p in preds:
            best_iou, best_idx = 0.0, -1
            for idx, g in enumerate(gt):
                if idx in matched or g["class_id"] != labels[p["class_id"]]:
                    continue
                value = helper.iou(p, g)
                if value > best_iou:
                    best_iou, best_idx = value, idx
            if best_iou >= 0.5:
                matched.add(best_idx)
                stats["tp"] += 1
                class_ok += 1
            else:
                stats["fp"] += 1
        stats["images"] += 1
        stats["gt"] += len(gt)
        for idx, g in enumerate(gt):
            key = bucket(area_of(g))
            counts = stats["by_area"].setdefault(key, {"gt": 0, "matched": 0})
            counts["gt"] += 1
            if idx in matched:
                counts["matched"] += 1
        stats["predictions"] += len(preds)
        stats["fn"] += len(gt) - len(matched)
        stats["class_correct"] += class_ok
        stats["per_image"].append({"id": image_id, "gt": len(gt), "pred": len(preds),
                                   "matched": len(matched)})
    runtime.release()
    stats["conf"] = args.conf
    stats["recall"] = stats["tp"] / stats["gt"] if stats["gt"] else 0.0
    stats["precision"] = stats["tp"] / stats["predictions"] if stats["predictions"] else 0.0
    Path(args.out).write_text(json.dumps(stats, indent=2), encoding="utf-8")
    print(json.dumps({k: stats[k] for k in ("images", "gt", "predictions", "tp", "fp", "fn",
                                            "recall", "precision")}, indent=2))


if __name__ == "__main__":
    main()
