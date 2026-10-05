# meister_vision

画像認識の基盤パッケージ (ROS2 ament_cmake)。機能09「物体把持」の PC 側
汎用物体検出レイヤを提供する。

- **推論エンジン**: ONNX Runtime (C++ API / torch / ultralytics 非依存)
- **モデル**: YOLOv8n ONNX (COCO 80クラス、約 12.3 MiB)
- **処理**: 画像トピック購読 → 検出 → `Detection2DArray` + 描画済み画像を配信

本パッケージは**検出基盤のみ**を実装する。アーム・IK・把持・Web UI などは
後続のフェーズで実装する。

## ディレクトリ構成

```
src/meister_vision/
├── package.xml / CMakeLists.txt
├── include/meister_vision/
│   ├── letterbox.hpp            # 640 正方形化的パディング
│   ├── nms.hpp                  # 重複抑制 (cv::dnn::NMSBoxes)
│   ├── onnx_session.hpp         # ONNX Runtime の薄いラッパ
│   ├── yolo_detector.hpp        # 検出器 + 後処理
│   ├── hand_landmark_detector.hpp
│   ├── image_util.hpp           # Image <-> cv::Mat
│   └── draw.hpp                 # 検出枠の描画
├── src/                         # 上記の実装 + ノード 3 本
├── scripts/download_model.py    # モデルのダウンロード (Python  据え置き)
├── launch/                      # 起動ランチャー (Python / launch は Python 専用 API)
├── models/                      # 実モデル (gitignore 済み)
└── test/                        # gtest
```

`scripts/download_model.py` だけ Python のまま。ネットワーク取得だけで C++ にしても
libcurl 依存が増えるだけなので、移行対象外にした。

## 必要な環境

- ROS2 Jazzy (rclcpp, sensor_msgs, vision_msgs, geometry_msgs, ament_index_cpp)
- OpenCV 4.6 (`libopencv-dev`)
- ONNX Runtime (`ros-jazzy-onnxruntime-vendor`)

```bash
sudo apt install libopencv-dev ros-jazzy-onnxruntime-vendor
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
colcon test --packages-select meister_vision
colcon test-result --all
```

54 ケース。**実モデル不要**で動く純粋関数のみを対象にしている。

- **letterbox**: 640 正方形化、アスペクト比維持、小画像の拡大
- **後処理**: cx/cy/w/h → xyxy、letterbox の逆変換、クラス argmax、clamp
- **NMS**: 重なり抑制・独立ボックス保持・降順
- **hand landmark**: 入力 layout 推定、decoded 点列、閾値、clip
- **Image 変換**: 9 種の encoding、step/data の整合、行パディング

## ノードの内部設計

`YoloDetector`:

1. `Letterbox()`: アスペクト比を保って 640x640 にリサイズ + 灰色パディング
2. `OnnxSession` で推論 (入力 `(1,3,640,640)` float32, 出力 `(1,84,8400)`)
3. `YoloPostprocess()` で `[cx, cy, w, h]` + 80クラススコアを xyxy に復元
4. `NonMaximumSuppression()` で重複抑制

`cv_bridge` は使わない。numpy 2.x との ABI 非互換で import できない環境が
あり、`image_util.hpp` の手動変換だけを使う。未知の encoding は拒否する
(uint8 のまま読むとチャンネル数がズレて静かに壊れるため)。

## ★要確認★ のモデル契約

`hand_landmarks.onnx` は .gitignore 済みで一次情報を取得できていない。
`hand_landmark_detector.hpp` の冒頭に残した前提がそのまま実装，影响する。

- 入力は静的な 4 次元で channels == 3 (NCHW / NHWC のどちらも受理)
- 出力は `(1, K, 3)`、`K = 21`
- landmark の並び順は MediaPipe の 21 点想定 (**★要確認★**)

実モデルを取得して出力 shape と添字順を実測で確定するまで、この前提は
確定的ではない。検出が外れても例外にはせず「手なし」を返すのは Python 版から
受け継いだ契約。
