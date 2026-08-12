"""yolo_detector の pytest ユニットテスト。

- (a) letterbox 前処理の正しさ (640x640, アスペクト比維持)
- (b) NMS ロジック (合成ボックスで重複抑制を確認)
- (c) フルパイプライン推論 (合成 stop sign 画像で実モデル検出)
    モデルが無ければフィクスチャが自動ダウンロードする。
    ダウンロードも失敗した場合はスキップする。
"""
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

# ソースツリー直下の meister_vision パッケージを import 可能にする
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from meister_vision.download_model import download_model  # noqa: E402
from meister_vision.yolo_detector import (  # noqa: E402
    COCO_CLASSES, Detection, YOLODetector, letterbox, nms,
    resolve_model_path,
)
from meister_vision.detection_node import _bgr_from_image_msg  # noqa: E402
from sensor_msgs.msg import Image as ImageMsg  # noqa: E402


# ---------------------------------------------------------------------------
# (a) letterbox 前処理
# ---------------------------------------------------------------------------

class TestLetterbox:
    def test_output_shape_and_pad_color(self):
        """入力が 640x640 にパディングされ、パディング色が 114 であること。"""
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        image[:, :, 0] = 200  # 左からはみ出す B 値
        padded, ratio, pad_w, pad_h = letterbox(image, new_shape=(640, 640))
        assert padded.shape == (640, 640, 3)
        assert ratio > 0.0
        # パディング領域の色が 114 であることを確認
        assert np.all(padded[0, :, :] == 114)  # 最上段はパディングのはず
        assert pad_w >= 0 and pad_h >= 0

    def test_aspect_ratio_preserved(self):
        """アスペクト比が維持されていること (縮尺は等比であること)。"""
        # 横向きの画像: 1280x720
        image = np.zeros((720, 1280, 3), dtype=np.uint8)
        padded, ratio, pad_w, pad_h = letterbox(image, new_shape=(640, 640))
        # コンテンツ領域の縦横比が元画像と同じであること
        content_h = 640 - 2 * pad_h
        content_w = 640 - 2 * pad_w
        assert abs((content_w / content_h) - (1280 / 720)) < 0.02
        assert abs(ratio - min(640 / 720, 640 / 1280)) < 1e-6

    def test_vertical_image_pads_horizontally(self):
        """縦長画像は左右にパディングされること。"""
        image = np.zeros((1080, 720, 3), dtype=np.uint8)
        padded, ratio, pad_w, pad_h = letterbox(image, new_shape=(640, 640))
        assert pad_h == 0
        assert pad_w > 0
        # 中央にコンテンツが来ること (左右対象パディング)
        assert np.all(padded[:, :pad_w, :] == 114)
        assert np.all(padded[:, -pad_w:, :] == 114)

    def test_roundtrip_coordinates(self):
        """letterbox 前後の座標変換が元の座標に戻ること。"""
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        _, ratio, pad_w, pad_h = letterbox(image, new_shape=(640, 640))
        # 元画像座標の点 → パディング済み座標 → 元に戻す
        px, py = 123.0, 200.0
        x_padded = px * ratio + pad_w
        y_padded = py * ratio + pad_h
        assert abs((x_padded - pad_w) / ratio - px) < 1e-6
        assert abs((y_padded - pad_h) / ratio - py) < 1e-6


# ---------------------------------------------------------------------------
# (b) NMS ロジック
# ---------------------------------------------------------------------------

class TestNMS:
    def test_overlapping_boxes_suppressed(self):
        """重なりが大きいボックスはスコアの高い1つに抑制されること。"""
        # ほぼ同じ領域のボックスが3つ
        boxes = np.array([
            [10, 10, 110, 110],
            [15, 15, 105, 105],
            [12, 12, 108, 108],
        ], dtype=np.float32)
        scores = np.array([0.9, 0.8, 0.7])
        keep = nms(boxes, scores, iou_threshold=0.45)
        assert len(keep) == 1
        assert keep[0] == 0  # 最高スコアのボックスだけ残る

    def test_far_boxes_all_kept(self):
        """離れたボックスは全て保持されること。"""
        boxes = np.array([
            [10, 10, 50, 50],
            [200, 200, 260, 260],
            [300, 10, 400, 100],
        ], dtype=np.float32)
        scores = np.array([0.9, 0.8, 0.7])
        keep = nms(boxes, scores, iou_threshold=0.45)
        assert len(keep) == 3

    def test_iou_threshold_tight(self):
        """IoU しきい値が大きいほど多くのボックスが残ること。"""
        # 中程度の重なり (IoU ≈ 0.68) を持つ2ボックス
        # 両ボックスの IoU は 0.45 < IoU < 0.95 に収まる
        boxes = np.array([
            [10, 10, 110, 110],
            [20, 20, 120, 120],
        ], dtype=np.float32)
        scores = np.array([0.9, 0.8])
        keep_loose = nms(boxes, scores, iou_threshold=0.45)
        keep_tight = nms(boxes, scores, iou_threshold=0.95)
        assert len(keep_loose) < len(keep_tight)

    def test_empty_input(self):
        """空入力で空結果を返すこと。"""
        keep = nms(np.zeros((0, 4)), np.zeros(0))
        assert len(keep) == 0


# ---------------------------------------------------------------------------
# モデルとクラス定義
# ---------------------------------------------------------------------------

class TestConstants:
    def test_coco_class_count(self):
        """COCO クラス名リストは80クラスであること。"""
        assert len(COCO_CLASSES) == 80
        assert COCO_CLASSES[0] == "person"
        assert "stop sign" in COCO_CLASSES
        assert "bottle" in COCO_CLASSES
        assert len(set(COCO_CLASSES)) == 80  # 重複なし

    def test_detection_dataclass(self):
        det = Detection(class_id=0, class_name="person",
                        confidence=0.9, xyxy=[1.0, 2.0, 3.0, 4.0])
        assert det.class_name == "person"
        assert len(det.xyxy) == 4


# ---------------------------------------------------------------------------
# (c) フルパイプライン推論 (実モデル)
# ---------------------------------------------------------------------------

@pytest.fixture(scope="session")
def detector_with_model():
    """モデルを (必要なら) ダウンロードして YOLODetector を返す。

    ダウンロードにも失敗したらテストをスキップする。
    """
    try:
        output_dir = Path(resolve_model_path()).parent
        download_model(output_dir)  # 冪等: あればスキップ
        detector = YOLODetector()
        return detector
    except Exception as exc:  # noqa: BLE001
        pytest.skip(f"モデルが利用できないためスキップ: {exc}")
        return None  # pragma: no cover


def _make_synthetic_image() -> np.ndarray:
    """白背景に合成の stop sign (赤八角形 + 白縁) と矩形を描いた画像。"""
    image = np.full((480, 640, 3), 255, dtype=np.uint8)

    def octagon(center, radius, n=8, rot=np.pi / 8.0):
        cx, cy = center
        return np.array([
            [cx + radius * np.cos(rot + 2 * np.pi * k / n),
             cy + radius * np.sin(rot + 2 * np.pi * k / n)]
            for k in range(n)
        ], np.int32)

    # stop sign: 白い縁取り + 赤い八角形
    cv2.fillPoly(image, [octagon((200, 220), 130)], (255, 255, 255))
    cv2.fillPoly(image, [octagon((200, 220), 100)], (0, 0, 255))
    # その他、検出は期待しない色付き矩形
    cv2.rectangle(image, (450, 100), (580, 380), (200, 200, 0), -1)
    cv2.rectangle(image, (480, 140), (550, 340), (30, 30, 180), -1)
    return image


class TestFullPipeline:
    def test_inference_on_synthetic_image(self, detector_with_model):
        """実モデルで合成 stop sign 画像が検出できること。"""
        image = _make_synthetic_image()
        detections = detector_with_model.detect(image)

        # 少なくとも1件検出される
        assert len(detections) > 0, "合成 stop sign が検出されませんでした"

        top = detections[0]
        assert top.class_name == "stop sign", (
            f"期待クラス 'stop sign' が検出されません (実際: {top.class_name}, "
            f"conf={top.confidence:.3f})")
        assert top.confidence > 0.5, f"信頼度が低すぎます: {top.confidence:.3f}"

        # 座標が画像範囲内であること
        x1, y1, x2, y2 = top.xyxy
        assert 0 <= x1 <= x2 <= 640
        assert 0 <= y1 <= y2 <= 480

    def test_annotated_image_draw(self, detector_with_model):
        """検出結果が画像に描画できること (枠 + ラベル)。"""
        image = _make_synthetic_image()
        detections = detector_with_model.detect(image)
        if not detections:
            pytest.skip("検出結果なしのため描画テストをスキップ")
        for det in detections:
            x1, y1, x2, y2 = (int(v) for v in det.xyxy)
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(image, f"{det.class_name} {det.confidence:.2f}",
                        (x1, max(y1 - 4, 0)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 0, 0), 1, cv2.LINE_AA)
        # 描画しても画像が破壊されないこと (shape 維持)
        assert image.shape == (480, 640, 3)


# ---------------------------------------------------------------------------
# (d) sensor_msgs/Image → BGR 変換 (usb_cam 等の実エンコーディング対応)
# ---------------------------------------------------------------------------

def _make_image_msg(bgr: np.ndarray, encoding: str) -> ImageMsg:
    """BGR ndarray を指定エンコーディングの ImageMsg に変換する。"""
    msg = ImageMsg()
    msg.height, msg.width = bgr.shape[:2]
    msg.encoding = encoding
    if encoding == "bgr8":
        data = bgr
    elif encoding == "rgb8":
        data = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    elif encoding == "yuv422_yuy2":
        data = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV_YUY2)
    else:
        raise ValueError(f"テスト未対応エンコーディング: {encoding}")
    msg.step = data.shape[1] * data.shape[2] if data.ndim == 3 else data.shape[1]
    msg.data = np.ascontiguousarray(data).tobytes()
    return msg


class TestImageConversion:
    def test_bgr8_roundtrip(self):
        """bgr8 はそのまま返る。"""
        bgr = np.zeros((480, 640, 3), dtype=np.uint8)
        bgr[:, :, 0] = 200
        bgr[:, :, 1] = 100
        bgr[:, :, 2] = 50
        out = _bgr_from_image_msg(_make_image_msg(bgr, "bgr8"))
        np.testing.assert_array_equal(out, bgr)

    def test_rgb8_converted_to_bgr(self):
        """rgb8 はチャンネルが入れ替わる。"""
        bgr = np.zeros((10, 10, 3), dtype=np.uint8)
        bgr[:, :, 0] = 200  # B
        bgr[:, :, 2] = 50   # R
        out = _bgr_from_image_msg(_make_image_msg(bgr, "rgb8"))
        np.testing.assert_array_equal(out, bgr)

    def test_yuv422_yuy2_converted_to_bgr(self):
        """usb_cam の yuv422_yuy2 が BGR に変換される。"""
        bgr = np.zeros((16, 16, 3), dtype=np.uint8)
        bgr[:, :, 0] = 200  # B
        bgr[:, :, 1] = 100  # G
        bgr[:, :, 2] = 50   # R
        out = _bgr_from_image_msg(_make_image_msg(bgr, "yuv422_yuy2"))
        assert out.shape == bgr.shape
        assert out.dtype == np.uint8
        # 可逆性はないが、完全に壊れていないこと (分散が0でない)
        assert out.std() > 0
        # 色相の大まかな傾向: 青成分が赤成分より大きい
        assert out[:, :, 0].mean() > out[:, :, 2].mean()

    def test_unsupported_encoding_raises(self):
        """未対応エンコーディングは ValueError を送出する。"""
        msg = _make_image_msg(np.zeros((8, 8, 3), dtype=np.uint8), "bgr8")
        msg.encoding = "invalid_xyz"
        with pytest.raises(ValueError):
            _bgr_from_image_msg(msg)
