# より多くの物体を検出する（2026-11-26）

## 結論

RKNN モデルを `yolov8n-zoo.rknn` → **`yolov8s-zoo.rknn`** に変更し、
既定で s を選ぶようにした。COCO val2017 の混雑画像 11 枚・**537 物体**（公式 GT）で
recall が **21.0% → 38.0%** に上がり、25Hz と CPU 16% 台は維持できた。

ただし **混雑シーンでは依然として約 6 割の物体を取りこぼす**。
「すべての物体を確実に検出」したわけではない（後述の未達を明示）。

## 評價セット

- 公式アノテーション: COCO val2017 `instances_val2017.json`（20MB、5000 画像・36781 anns）
- 物体数が多い上位 11 画像を選択（1 画像 41〜62 物体、合計 **537 GT**）
- GT は公式 bbox（iscrowd=0）をそのまま使用。モデルの出力は正解に使っていない。
- 評価器: [evaluate_coco_subset.py](evaluate_coco_subset.py)、IoU≥0.5 でクラス一致してマッチ。

## 結果（conf 0.45 / NMS 0.45）

| モデル / 方式 | TP | FP | recall | precision | small | medium | large |
|---|---|---|---|---|---|---|---|
| yolov8n（現行） | 113 | 21 | **21.0%** | 84.3% | 6/244 | 64/224 | 43/69 |
| yolov8n conf 0.25 | 159 | 92 | 29.6% | 63.3% | — | — | — |
| yolov8n 2x2 タイル | 178 | 129 | 33.1% | 58.0% | 34/244 | 108/224 | 36/69 |
| **yolov8s（採用）** | 204 | 58 | **38.0%** | 77.9% | 33/244 | 119/224 | 52/69 |
| yolov8s conf 0.35 | 235 | 107 | 43.8% | 68.7% | 49/244 | 131/224 | 55/69 |

- しきい値を 0.25 まで下げると recall は上がるが precision が 63% に落ち、
  「確実」とは言えないため採用しない。
- 2x2 タイルは recall を上げるが FP が 129 に増え、推論も 4 倍になるため不採用。
- yolov8s は small で 6→33、large で 43→52 と全サイズで改善し、
  precision も 78% を維持した。

### 既存の簡単な回帰セット（GT を修正）

bus.jpg の左端に写っていた部分人物を GT に追加（以前は誤検出として扱っていた）:

| モデル | bus.jpg | person_dog_horse.jpg | 平均 IoU |
|---|---|---|---|
| yolov8n | 4/5 | 3/3 | 0.918 / 0.945 |
| **yolov8s** | **5/5** | **3/3** | **0.960 / 0.960** |

## 実装

- [convert_yolov8.py](../tools/rknn/convert_yolov8.py) を追加（aarch64 の rknn-toolkit2 2.5.0 で動作）。
  Rockchip 最適化 ONNX を INT8 量子化して RKNN に変換する。
  キャリブは COCO 画像 60 枚（元画像＋反転＋明度変更＋4 分割クロップ）。
- `pc_vision.launch.py` と `vision_node_common.py` の既定モデルを
  **yolov8s 優先・無ければ yolov8n** に変更（R2: 選択は起動側の 1 箇所ずつ）。
- `.rknn` は gitignore 対象のためモデル自体はコミットしない。
  生成手順は [tools/rknn/README.md](../tools/rknn/README.md) と下記を参照。

```bash
# モデル生成（この機体で実施済み）
wget -O /tmp/yolov8s.onnx '<Rockchip model-zoo yolov8s.onnx URL>'
python3 tools/rknn/convert_yolov8.py \
  --onnx /tmp/yolov8s.onnx --dataset /tmp/coco_eval/calib.txt \
  --out ros/src/umeonigiri_vision/models/yolov8s-zoo.rknn
./build.sh   # install share へ配置
```

## 実行時（既定 launch）

```bash
ros2 launch umeonigiri_vision pc_vision.launch.py backend:=npu \
  output_fps:=25.0 annotated_width:=320 conf_threshold:=0.45
```

実測: `/detections` **24.723 Hz**、`yolo_grasp_node` CPU **16.5%**、
Web UI **24.7 fps**（frame_age 13ms）。起動パラメータの `model` は
`.../models/yolov8s-zoo.rknn` であることを確認済み。ビルド成功・233 tests 0 failures。

## 未達 / 注意

- COCO の混雑画像では recall 38% に留まり、**すべての物体を検出できていない**。
  特に small(<1024px²) は 33/244（13.5%）。さらに検出数を増やすには
  より高精度なモデル・高解像度入力・マルチスケール推論が必要で、
  25Hz/CPU 40% の両立は未検証（★要確認★）。
- 実カメラの現在の映し方には COCO クラスの物体が無く、
  リアルタイム映像での「検出数増加」は目視確認できていない。
- INT8 量子化の score 張り付きは引き続き残る。
