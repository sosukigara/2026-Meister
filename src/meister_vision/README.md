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
| 配信 | `target` | `vision_msgs/Detection2DArray` | アームを向ける 1 個 (0 または 1 要素) |

`detections` の各要素 (`Detection2D`):

- `id`: クラス名 (例: `stop sign`)
- `results[0].hypothesis.class_id`: クラス ID (`str`)
- `results[0].hypothesis.score`: 信頼度
- `bbox.center.position`: 中心座標 (ピクセル)
- `bbox.size_x` / `bbox.size_y`: 幅 / 高さ

### `target` — アーム先端カメラの視覚サーボ用

アーム先端カメラで物体中心へ寄せる用途では、毎フレーム別の物体を掴むと
制御が振動する。`target` は次の規則で 1 個だけを選んで配信する。

1. 前フレームの対象が `target_lock_radius` [px] 以内に残っていれば追従する
2. 見失ったときだけ画像中心に最も近い候補へ乗り換える
3. 該当なしは空配列 (要素数 0) を配信する

見失ってすぐ別の物体へ乗り換えるとアームが飛ぶため、既定では 3 フレーム
(10 Hz で 300 ms) は保持し、その間は空配列を配信する。

パラメータ: `target_class` (追うクラス ID、-1 で指定なし)、
`target_lock_radius` (既定 120.0 px)、`lock_grace_frames` (既定 3)。

画像中心からの正規化オフセットは次の式で得られる (`w`, `h` は入力画像):

```text
offset_x = (bbox.center.position.x - w/2) / (w/2)   # 右が正
offset_y = (bbox.center.position.y - h/2) / (h/2)   # 下が正
```

ノードは既定で `rate=30 Hz`（上限なし相当）で処理する。実際のレートは
推論が追いつくところで頭打ちになる。カメラは処理レート以上で回すこと
(`camera_fps`、既定 30 Hz)。同レートだと位相のズレでフレームが落ちる。

### 実測性能 (開発機 / Ryzen 5 7430U, 4 スレッド)

| 入力 | 1 フレーム | 処理レート | 撮像→配信 | 640 比の再現率 |
|---|---|---|---|---|
| **640** (既定) | 41 ms | **18.3 Hz** | 104 ms | 100 % (基準) |
| 480 | 24 ms | **29.1 Hz** | 60 ms | 68.8 % |
| 416 | 15 ms | **29.4 Hz** | 54 ms | 62.5 % |

開発機のカメラ (EMEET C960) は 640x480 で 29.5 fps が上限なので、
480/416 はカメラ側で頭打ちになる。精度を優先するなら 640 のまま、
速度を優先するなら 480 が現実的な妥協点。

### 別解像度のモデルを作る

`models/` は gitignore 済み。手元で生成する場合:

```bash
python3 -m venv /tmp/mlenv
/tmp/mlenv/bin/pip install --index-url https://download.pytorch.org/whl/cpu torch torchvision
/tmp/mlenv/bin/pip install ultralytics onnx onnxslim
cd /tmp && /tmp/mlenv/bin/yolo export model=yolov8n.pt format=onnx imgsz=480
# 生成物を models/ に置き model_path で指定する
ros2 launch meister_vision pc_vision.launch.py use_pc_camera:=true \
    model_path:=/path/to/yolov8n_480.onnx
```

入力解像度と格子数はモデルの入出力形から自動で読むので、コード変更は
要らない。

### ボトルネック (実測)

- 推論が 1 フレームの約 8 割。CPU 実行プロバイダのみで、GPU / NPU は無い
- カメラ側の `cv::CAP_PROP_BUFFERSIZE=1` は取得レートを半分にする
  (30 fps 機で 15.6 fps)。2 にする
- 固定周期ポーリングはカメラ周期と位相がぶつかると半分を取りこぼす。
  取得・推論とも「到着駆動 + レート上限」にしてある

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
