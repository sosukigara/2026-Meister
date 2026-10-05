"""手の keypoint を推定する ONNX 推論器。

onnxruntime のみを使う。mediapipe / tflite / torch は使わない
(yolo_detector が既に onnxruntime に依存しており、setup.py が
「依存を増やさない」方針のため)。

このモジュールのモデル契約 (★要確認★)
--------------------------------------
手 keypoint モデルはリポジトリに無く .gitignore (`*.onnx`) で除外されて
いるため、一次情報は未取得。下の形は模型的都合から定めた契約であり、
実模型的定義と一致するかは未検証。所詮ここは「壊れた keypoint を
黙って流すくらいなら例外で落とす」側に倒す (AGENTS.md R4)。

  入力  session の入力メタデータから静的 4 次元かつチャンネル次元が 3 の
        ものを NHWC (1, S, S, 3) と NCHW (1, 3, S, S) として受け入れる。
        S = 入力辺長。動的形状や 3 チャンネルでない形は ValueError。
  出力  (1, K, 3) の float32。既定 K = 21。
          [:, 0] = x … letterbox 済みキャンバス基準で 0.0-1.0 に正規化
          [:, 1] = y … 同上
          [:, 2] = score … 0.0-1.0

K 個の landmark の並び順は模型的学習の規約に従う。21 点なら
MediaPipe 規約 (0 手首 / 1-4 親指 / 5-8 人差し指 / 9-12 中指 /
13-16 薬指 / 17-20 小指) を想定しているが、これも ★要確認★。
添字順の規約を勝手に固定せず K だけ設定で変えられるようにしているのは
、後から規約の違うモデルへ差し替えても添字の読み違えを避けられるため。

入力画像は BGR (OpenCV 形式) を想定する。
"""
from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, List, Optional, Tuple

import numpy as np

from .yolo_detector import letterbox

__all__ = [
    "DEFAULT_INPUT_SIZE",
    "DEFAULT_NUM_LANDMARKS",
    "HandLandmarkDetector",
    "HandLandmarks",
    "MODEL_ENV_VAR",
    "MODEL_FILENAME",
    "resolve_hand_model_path",
]

MODEL_FILENAME = "hand_landmarks.onnx"
MODEL_ENV_VAR = "MEISTER_VISION_HAND_MODEL"
DEFAULT_INPUT_SIZE = 224
DEFAULT_NUM_LANDMARKS = 21


@dataclass
class HandLandmarks:
    """1 隻の手の結果。

    landmarks は (x, y) のピクセル座標で、模型の添字順のまま並ぶ。
    """

    landmarks: List[Tuple[float, float]]
    confidence: float


def _candidate_paths() -> List[Path]:
    """モデルを探す候補パスを優先順位順に返す (探索未必)。"""
    candidates: List[Path] = []
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(
            Path(get_package_share_directory("meister_vision"))
            / "models" / MODEL_FILENAME)
    except Exception:  # noqa: BLE001
        pass
    candidates.append(Path(__file__).resolve().parents[1] / "models"
                      / MODEL_FILENAME)
    return candidates


def resolve_hand_model_path() -> str:
    """手 keypoint モデルのパスを次の優先順位で解決する。

    1. 環境変数 MEISTER_VISION_HAND_MODEL
    2. インストール先の共有ディレクトリ (models/hand_landmarks.onnx)
    3. ソースツリー (src/meister_vision/models/hand_landmarks.onnx)

    どれも存在しなくても最後の候補を返す。実在の判定は
    HandLandmarkDetector に任せ、解決と検証を混ぜない。
    """
    env = os.environ.get(MODEL_ENV_VAR)
    if env:
        return env
    candidates = _candidate_paths()
    for path in candidates:
        if path.exists():
            return str(path)
    return str(candidates[-1])


def _make_session(model_path: str, providers: Optional[List[str]],
                  intra_op_num_threads: int) -> Any:
    """onnxruntime の推論セッションを作る。

    import は関数内に置く。onnxruntime が無い環境 (onnxruntime なしの
    テスト) でも、このモジュールを import できる状態を保つため。
    """
    import onnxruntime as ort

    options = ort.SessionOptions()
    options.intra_op_num_threads = max(1, int(intra_op_num_threads))
    options.inter_op_num_threads = 1
    return ort.InferenceSession(
        model_path,
        sess_options=options,
        providers=providers or ["CPUExecutionProvider"],
    )


def _probe_input(session: Any) -> Tuple[str, int]:
    """session の入力メタデータから (layout, 入力辺長) を取り出す。

    記憶で NCHW / NHWC を決めると外した時だけ静かに壊れる。モデルが
    自ら宣言している形を一次情報として使う。
    """
    meta = session.get_inputs()[0]
    shape = list(getattr(meta, "shape", None) or [])
    static = [d for d in shape if isinstance(d, int) and d > 0]
    if len(shape) == 4 and len(static) == 4:
        if shape[3] == 3:
            return "nhwc", int(shape[1])
        if shape[1] == 3:
            return "nchw", int(shape[2])
    raise ValueError(
        f"未対応の入力形状 {shape}。静的 4 次元でチャンネル次元が 3 のモデル "
        "(NHWC (1,S,S,3) / NCHW (1,3,S,S)) のみ受け付けます。")


class HandLandmarkDetector:
    """手 keypoint ONNX モデルをラップする検出器。

    session を渡すとモデル解決と読み込みを飛ばす。onnxruntime の無い
    環境や、復号ロジックだけを確かめたいテストの差し込み口。
    """

    def __init__(
        self,
        model_path: Optional[str] = None,
        *,
        score_threshold: float = 0.2,
        num_landmarks: int = DEFAULT_NUM_LANDMARKS,
        session: Optional[Any] = None,
        providers: Optional[List[str]] = None,
        intra_op_num_threads: int = 2,
    ) -> None:
        self.score_threshold: float = float(score_threshold)
        self.num_landmarks: int = int(num_landmarks)

        resolved = model_path
        if session is None:
            resolved = model_path or resolve_hand_model_path()
            if not os.path.exists(resolved):
                # 解像結果が候補そのものであると二重に並ぶ。順序は保つ。
                tried = list(dict.fromkeys(
                    [resolved] + [str(p) for p in _candidate_paths()]))
                raise FileNotFoundError(
                    f"手 keypoint モデルが見つかりません: {resolved}\n"
                    "探索したパス:\n"
                    + "".join(f"  - {p}\n" for p in tried)
                    + f"配置方法は 3 つ: パラメータ model_path / 環境変数 "
                    f"{MODEL_ENV_VAR} / models/{MODEL_FILENAME}\n"
                    "yolov8n.onnx は COCO 80 クラスで手 keypoint を"
                    "出さないため、このノードには別のモデルが必要です。")
            session = _make_session(resolved, providers, intra_op_num_threads)

        self.model_path: Optional[str] = resolved
        self._session: Any = session
        self._input_name: str = self._session.get_inputs()[0].name
        self._output_name: str = self._session.get_outputs()[0].name
        self._layout: str
        self._input_size: int
        self._layout, self._input_size = _probe_input(self._session)

    def _preprocess(self, image_bgr: np.ndarray) -> Tuple[np.ndarray, float,
                                                         int, int]:
        """BGR 画像を ONNX 入力テンソルに変換する。

        Returns:
            (blob, ratio, pad_w, pad_h) — blob は正規化済み入力を、
            残りは出力を元画像座標へ戻すための letterbox の逆変換値。
        """
        padded, ratio, pad_w, pad_h = letterbox(
            image_bgr, new_shape=(self._input_size, self._input_size))
        # keypoint モデルは RGB で学習されている前提。入力名で分岐する
        # 手法は、名前が違う書き出しで静かに外れるので採らない。
        rgb = padded[:, :, ::-1]
        if self._layout == "nhwc":
            blob = rgb[np.newaxis]
        else:
            blob = rgb.transpose(2, 0, 1)[np.newaxis]
        blob = np.ascontiguousarray(blob, dtype=np.float32) / 255.0
        return blob, ratio, pad_w, pad_h

    def _decode(self, output: np.ndarray, ratio: float, pad_w: int, pad_h: int,
                image_shape: Tuple[int, ...]) -> Optional[HandLandmarks]:
        """(1, K, 3) の出力を元画像座標の HandLandmarks に変換する。

        手が無いときは None を返す。例外は「契約違反」だけに使う。
        """
        expected = (1, self.num_landmarks, 3)
        shape = tuple(np.asarray(output).shape)
        if shape != expected:
            raise ValueError(
                f"未対応の出力形状 {shape}。期待値は {expected}"
                " ([x, y, score] を landmark ごとに並べた形のみ対応)。")

        rows = np.asarray(output, dtype=np.float32)[0]
        # landmark ごとに score を持つ形なので、最も低い点で hand 全体の
        # 信頼性を決める。平均だと 1 点だけ崩れた landmark を隠す。
        confidence = float(rows[:, 2].min())
        if confidence < self.score_threshold:
            return None

        h, w = image_shape
        canvas = rows[:, :2] * float(self._input_size)
        xy = (canvas - np.array([pad_w, pad_h], dtype=np.float32)) / ratio
        xy[:, 0] = np.clip(xy[:, 0], 0.0, float(w))
        xy[:, 1] = np.clip(xy[:, 1], 0.0, float(h))
        return HandLandmarks(
            landmarks=[(float(x), float(y)) for x, y in xy],
            confidence=confidence,
        )

    def detect(self, image_bgr: np.ndarray) -> Optional[HandLandmarks]:
        """BGR 画像から手の keypoint を推定する。

        Returns:
            手が映っていなければ None。「手なし」は正常系なので例外に
            しない (Publisher 側が「結果ゼロ」をそのまま配送できる)。
        """
        if image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
            raise ValueError(
                f"入力画像は (H, W, 3) の BGR 画像が必要です "
                f"(got {image_bgr.shape})")

        blob, ratio, pad_w, pad_h = self._preprocess(image_bgr)
        outputs = self._session.run(
            [self._output_name], {self._input_name: blob})
        return self._decode(outputs[0], ratio, pad_w, pad_h,
                            image_shape=image_bgr.shape[:2])