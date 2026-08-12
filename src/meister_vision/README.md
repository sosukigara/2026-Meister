# meister_vision

画像認識の基盤パッケージ (ROS2 ament_python)。機能09「物体把持」の PC 側
汎用物体検出レイヤを提供する。

- **推論エンジン**: onnxruntime (torch / ultralytics 非依存)
- **モデル**: YOLOv8n ONNX (COCO 80クラス、約 12.3 MiB)
- **処理**: 画像トピック購読 → 検出 → `Detection2DArray` + 描画済み画像を配信

本パッケージは**検出基盤のみ**を実装する。アーム・IK・把持・Web UI などは
後続のフェーズで実装する。

## ディレクトリ構成

```
src/meister_vision/
├── package.xml / setup.py / setup.cfg
├── resource/meister_vision      # ament index 用マーカー
├── meister_vision/
│   ├── yolo_detector.py         # ONNX YOLOv8n 検出クラス (依存ゼロ)
│   ├── download_model.py        # モデルダウンロードの実体
│   └── detection_node.py        # ROS2 ノード
├── scripts/download_model.py    # ダウンロード用スタンドアロンラッパー
├── launch/detection.launch.py   # 起動ランチャー
├── test/test_detector.py        # pytest (letterbox / NMS / 実推論)
└── README.md
```

## 必要な環境

- ROS2 Jazzy (rclpy, sensor_msgs, vision_msgs, cv_bridge)
- `python3-opencv` (cv2)
- `python3-numpy`
- `onnxruntime` (apt に無い場合は pip で導入)

```bash
pip install onnxruntime
```

## ビルド

```bash
# ワークスペースルート (/tmp/opencode/meister-esp-vision) で
colcon build --packages-select meister_vision --symlink-install
source install/setup.bash
```

## モデルのダウンロード

```bash
# ソースツリーから直接実行 (src/meister_vision/models/yolov8n.onnx に保存)
python3 src/meister_vision/scripts/download_model.py

# またはインストール後
ros2 run meister_vision download_model
```

既にダウンロード済みならスキップされる (冪等)。サイズの簡易チェック
(5〜30 MiB) で途中終了を検出する。

## ノードの起動

```bash
# カメラ画像トピックが image_raw の場合
ros2 launch meister_vision detection.launch.py

# カメラトピックやパラメータを指定する場合
ros2 launch meister_vision detection.launch.py \
    image_topic:=/camera/image_raw \
    conf_threshold:=0.3 \
    model_path:=$HOME/models/yolov8n.onnx
```

モデルのパスは以下の優先順位で解決される:

1. パラメータ `model_path`
2. 環境変数 `MEISTER_VISION_MODEL`
3. パッケージ共有ディレクトリの `models/yolov8n.onnx`
4. ソースツリーの `src/meister_vision/models/yolov8n.onnx`

## トピック

| 方向 | トピック | 型 | 説明 |
|------|----------|-----|------|
| 購読 | `image_raw` (パラメータ変更可) | `sensor_msgs/Image` | 入力画像 |
| 配信 | `detections` | `vision_msgs/Detection2DArray` | 検出結果 (bbox / クラス / 信頼度) |
| 配信 | `detection_image` | `sensor_msgs/Image` | 検出枠・ラベルを描画した画像 |

`detections` の各要素 (`Detection2D`):

- `id`: クラス名 (例: `stop sign`)
- `results[0].hypothesis.class_id`: クラス ID (`str`)
- `results[0].hypothesis.score`: 信頼度
- `bbox.center.position`: 中心座標 (ピクセル)
- `bbox.size_x` / `bbox.size_y`: 幅 / 高さ

ノードは既定で最大 **10 Hz** にレート制限し、負荷を軽く保つ。

## 確認方法

```bash
# トピック確認
ros2 topic list
ros2 topic echo /detections

# 検出結果を確認
ros2 topic echo /detections --once

# 描画画像を保存して確認
ros2 run rqt_image_view rqt_image_view
```

## テスト

```bash
# ワークスペースルートで (モデルが無ければ自動ダウンロード)
python3 -m pytest src/meister_vision/test/ -v
```

> **注意**: ROS Jazzy の `launch_testing` 系 pytest プラグインは pytest 9 と
> 非互換のため、プラグイン自動ロードを無効にして実行する必要がある環境がある。

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/meister_vision/test/ -v
```

テスト内容:

- **letterbox 前処理**: 640x640 へのパディングとアスペクト比維持
- **NMS**: 重なりボックスの抑制・独立ボックスの保持
- **フルパイプライン**: 合成 stop sign 画像で実モデル検出

## ノードの内部設計

`YOLODetector` (yolo_detector.py):

1. `letterbox()`: アスペクト比を保って 640x640 にリサイズ + 灰色パディング
2. onnxruntime で推論 (入力 `(1,3,640,640)` float32, 出力 `(1,84,8400)`)
3. 出力を `[cx, cy, w, h]` + 80クラススコアとしてパース → 元画像座標の xyxy
4. 純粋 NumPy 実装の `nms()` で重複抑制

`cv_bridge` は使える環境なら利用するが、numpy 2.x との ABI 非互換で
import できない環境では手動のエンコーディング変換にフォールバックする。
