"""YOLOv8n ONNX 物体検出器。

onnxruntime + 軽量な YOLOv8n ONNX モデル (COCO 80クラス) のみを使った
依存性ゼロの検出クラス。torch / ultralytics は使わない。

パイプライン:
  1. letterbox 前処理 (アスペクト比を保ったまま 640x640 にパディング)
  2. onnxruntime で推論 (入力: (1,3,640,640) float32 / 出力: (1,84,8400))
  3. 出力をパース (cx,cy,w,h + 80クラススコア) → xyxy に変換
  4. 純粋な NumPy 実装の NMS で重複抑制

入力画像は BGR (OpenCV 形式) を想定する。
"""
from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np

__all__ = [
    "COCO_CLASSES",
    "Detection",
    "YOLODetector",
    "letterbox",
    "nms",
    "resolve_model_path",
]

# モデル固定入力サイズ (YOLOv8n のデフォルト)
INPUT_SIZE = 640
# letterbox パディング色 (YOLO 標準の灰色 114)
PAD_COLOR = 114

# 標準 COCO 80 クラス名リスト
COCO_CLASSES: List[str] = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
    "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon",
    "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
    "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant",
    "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote",
    "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush",
]

# YOLOv8 ONNX 出力の各予測のオフセット (出力は (1, 84, 8400))
#   [0:4]  = cx, cy, w, h (640x640 座標系)
#   [4:84] = COCO 80クラスそれぞれのスコア
_BBOX_COLS = 4
_NUM_CLASSES = 80
_GRID_CELLS = 8400


@dataclass
class Detection:
    """1 つの検出結果。xyxy は元画像座標系の [x1, y1, x2, y2]。"""

    class_id: int
    class_name: str
    confidence: float
    xyxy: List[float]


def letterbox(
    image: np.ndarray,
    new_shape: Tuple[int, int] = (INPUT_SIZE, INPUT_SIZE),
    color: int = PAD_COLOR,
) -> Tuple[np.ndarray, float, int, int]:
    """アスペクト比を保ったままリサイズし、灰色でパディングする。

    Returns:
        (padded, ratio, pad_w, pad_h)
          - padded : (new_shape[0], new_shape[1], 3) の画像
          - ratio  : 元画像に適用したスケール倍率
          - pad_w  : 左右に追加したパディング幅
          - pad_h  : 上下に追加したパディング高さ
    """
    h, w = image.shape[:2]
    target_h, target_w = new_shape
    ratio = min(target_h / h, target_w / w)
    new_h, new_w = int(round(h * ratio)), int(round(w * ratio))

    # 縮尺が同じなら resize をスキップ (コーナーケース対策)
    resized = image if (new_h, new_w) == (h, w) else cv2.resize(
        image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

    canvas = np.full((target_h, target_w, 3), color, dtype=np.uint8)
    pad_h = (target_h - new_h) // 2
    pad_w = (target_w - new_w) // 2
    canvas[pad_h:pad_h + new_h, pad_w:pad_w + new_w] = resized
    return canvas, ratio, pad_w, pad_h


def nms(
    boxes: np.ndarray,
    scores: np.ndarray,
    iou_threshold: float = 0.45,
) -> np.ndarray:
    """純粋な NumPy 実装の貪欲 NMS。

    Args:
        boxes : (N, 4) の [x1, y1, x2, y2] 配列
        scores: (N,) のスコア配列 (降順にソート済みである必要はない)
        iou_threshold: この IoU を超えたボックスを抑制する

    Returns:
        採用されたボックスのインデックス配列 (降順のスコア順)
    """
    if boxes.size == 0:
        return np.zeros(0, dtype=np.int64)

    boxes = np.asarray(boxes, dtype=np.float64)
    scores = np.asarray(scores, dtype=np.float64)

    # 面積は重複計算を避けるため先に算出
    areas = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
    order = np.argsort(-scores)

    keep: List[int] = []
    while order.size > 0:
        i = int(order[0])
        keep.append(i)
        if order.size == 1:
            break

        rest = order[1:]
        # 交差領域の左上/右下
        xx1 = np.maximum(boxes[i, 0], boxes[rest, 0])
        yy1 = np.maximum(boxes[i, 1], boxes[rest, 1])
        xx2 = np.minimum(boxes[i, 2], boxes[rest, 2])
        yy2 = np.minimum(boxes[i, 3], boxes[rest, 3])

        inter_w = np.maximum(0.0, xx2 - xx1)
        inter_h = np.maximum(0.0, yy2 - yy1)
        inter = inter_w * inter_h
        union = areas[i] + areas[rest] - inter
        iou = np.where(union > 0.0, inter / union, 0.0)

        order = rest[iou <= iou_threshold]

    return np.asarray(keep, dtype=np.int64)


def resolve_model_path() -> str:
    """モデルファイルのパスを次の優先順位で解決する。

    1. 環境変数 MEISTER_VISION_MODEL
    2. インストール先の共有ディレクトリ (install/share/meister_vision/models)
    3. ソースツリー (src/meister_vision/models)
    """
    env = os.environ.get("MEISTER_VISION_MODEL")
    if env:
        return env

    candidates: List[Path] = []
    # インストール先 (ament_index_python が解決できる場合のみ)
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(
            Path(get_package_share_directory("meister_vision")) / "models"
            / "yolov8n.onnx")
    except Exception:
        pass
    # ソースツリー (テスト/未インストール時)
    candidates.append(Path(__file__).resolve().parents[1] / "models"
                      / "yolov8n.onnx")

    for path in candidates:
        if path.exists():
            return str(path)
    return str(candidates[-1])


class YOLODetector:
    """YOLOv8n ONNX モデルをラップする物体検出器。"""

    def __init__(
        self,
        model_path: Optional[str] = None,
        conf_threshold: float = 0.25,
        iou_threshold: float = 0.45,
        providers: Optional[List[str]] = None,
    ) -> None:
        self.model_path = model_path or resolve_model_path()
        if not os.path.exists(self.model_path):
            raise FileNotFoundError(
                f"モデルが見つかりません: {self.model_path}\n"
                "ダウンロードスクリプトを実行してください:\n"
                "  python3 src/meister_vision/scripts/download_model.py\n"
                "  または環境変数 MEISTER_VISION_MODEL にパスを設定")

        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self._session = None
        self._input_name = ""
        self._output_name = ""

        import onnxruntime as ort
        self._session = ort.InferenceSession(
            self.model_path,
            providers=providers or ["CPUExecutionProvider"],
        )
        self._input_name = self._session.get_inputs()[0].name
        self._output_name = self._session.get_outputs()[0].name
        # 入力名が images の YOLOv8 モデルは BGR→RGB 変換が必要
        self._needs_rgb = self._input_name == "images"

    def _preprocess(
        self, image_bgr: np.ndarray,
    ) -> Tuple[np.ndarray, float, int, int]:
        """BGR 画像を (1,3,640,640) float32 の ONNX 入力に変換する。"""
        padded, ratio, pad_w, pad_h = letterbox(image_bgr)
        blob = padded[:, :, ::-1].transpose(2, 0, 1)[np.newaxis]
        blob = np.ascontiguousarray(blob, dtype=np.float32) / 255.0
        return blob, ratio, pad_w, pad_h

    def _postprocess(
        self,
        output: np.ndarray,
        ratio: float,
        pad_w: int,
        pad_h: int,
        image_shape: Tuple[int, int],
    ) -> List[Detection]:
        """(1, 84, 8400) の出力を元画像座標系の Detection リストに変換する。"""
        pred = output[0].T  # (8400, 84)
        boxes_xywh = pred[:, :_BBOX_COLS]
        class_scores = pred[:, _BBOX_COLS:_BBOX_COLS + _NUM_CLASSES]

        scores = class_scores.max(axis=1)
        class_ids = class_scores.argmax(axis=1)
        mask = scores >= self.conf_threshold
        if not np.any(mask):
            return []

        boxes_xywh = boxes_xywh[mask]
        scores = scores[mask]
        class_ids = class_ids[mask]

        # cx,cy,w,h (640x640 座標) → 元画像座標の xyxy
        cx, cy, w, h = (boxes_xywh[:, 0], boxes_xywh[:, 1],
                        boxes_xywh[:, 2], boxes_xywh[:, 3])
        x1 = (cx - w / 2.0 - pad_w) / ratio
        y1 = (cy - h / 2.0 - pad_h) / ratio
        x2 = (cx + w / 2.0 - pad_w) / ratio
        y2 = (cy + h / 2.0 - pad_h) / ratio
        xyxy = np.stack([x1, y1, x2, y2], axis=1)

        img_h, img_w = image_shape
        xyxy[:, 0] = np.clip(xyxy[:, 0], 0.0, img_w)
        xyxy[:, 1] = np.clip(xyxy[:, 1], 0.0, img_h)
        xyxy[:, 2] = np.clip(xyxy[:, 2], 0.0, img_w)
        xyxy[:, 3] = np.clip(xyxy[:, 3], 0.0, img_h)

        keep = nms(xyxy, scores, iou_threshold=self.iou_threshold)
        detections: List[Detection] = []
        for idx in keep:
            class_id = int(class_ids[idx])
            detections.append(Detection(
                class_id=class_id,
                class_name=COCO_CLASSES[class_id],
                confidence=float(scores[idx]),
                xyxy=[float(v) for v in xyxy[idx]],
            ))
        return detections

    def detect(self, image_bgr: np.ndarray) -> List[Detection]:
        """BGR 画像に対し物体検出を実行し、Detection のリストを返す。

        Args:
            image_bgr: OpenCV 形式 (H, W, 3) の BGR 画像

        Returns:
            信頼度の高い順に並んだ検出結果。無検出なら空リスト。
        """
        if image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
            raise ValueError(
                f"入力画像は (H, W, 3) の BGR 画像が必要です (got {image_bgr.shape})")

        blob, ratio, pad_w, pad_h = self._preprocess(image_bgr)
        outputs = self._session.run(
            [self._output_name], {self._input_name: blob})
        output = np.asarray(outputs[0])
        return self._postprocess(output, ratio, pad_w, pad_h,
                                 image_shape=image_bgr.shape[:2])
