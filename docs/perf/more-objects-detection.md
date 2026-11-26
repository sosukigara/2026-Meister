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

## 実カメラのしきい値調整（2026-11-27）

Web UI で「物が映っているのに検出されない」との指摘を受け、
実カメラ（ベッド周辺が映っている）で 20 秒間の非空検出率を conf ごとに計測した。

| conf | 非空 / 全 detections | 主なラベル | CPU | /detections |
|---|---|---|---|---|
| 0.45（旧既定） | 54 / 496（11%） | bed 51 | 29.2% | 24.7 Hz |
| 0.40 | 43 / 435（10%） | bed 43 | 40.2% | 24.7 Hz |
| 0.35 | 139 / 440（32%） | bed 139, book 1 | 43.8% | 24.7 Hz |
| **0.32（新既定）** | **331 / 495（67%）** | bed 342, person 3, toilet 1 | **47.0%** | **24.7 Hz** |
| 0.30 | 358 / 433（83%） | bed 387, book 9, remote 1, person 2 | 48.9% | 24.7 Hz |

COCO 537 物体での精度トレードオフ:

| conf | recall | precision | F1 |
|---|---|---|---|
| 0.45 | 38% | 77% | 51 |
| **0.32** | **45%** | **66%** | **54** |
| 0.30 | 46% | 64% | 54 |

0.30 はさらに非空率が上がるが CPU 48.9% で 50% に近づくため、
**0.32 を新既定**にした（`pc_vision.launch.py` の `conf_threshold`）。
25Hz と CPU 50% 以下を維持しつつ、実映像で検出が出るフレームを 11%→67% に増やした。

`person` / `toilet` の少数検出は誤検出の可能性が高く、完全に消えたわけではない
（★要確認★）。しきい値を上げれば減るが、その分ベッドの検出も減る。

### CPU 余裕の確保

conf 0.32 のまま既定 launch で再計測すると CPU が 49.6% まで上がり、
50% すれすれだった。把持輪郭解析 (`ComputeGrasp`) は `/grasp_image` の
購読者が居るときだけ実行し、画像も同じく購読者が居るときだけ配信するよう
C++ ノードを変更した (Web UI は `/detections` と `/detection_image` だけを見る)。

最終実測（既定 launch、`conf=0.32` / `annotated_width=320` / `output_fps=25`）:

| 項目 | 値 |
|---|---|
| 非空検出 | **495 / 495**（20 秒、主に `bed`、まれに `book`） |
| `/detections` | **24.707 Hz** |
| 検出ノード CPU | **39.2%**（50% 以下に余裕あり） |
| Web UI | **24.7 fps**、frame_age 36ms、320x240 |

Web UI の認識モニターに緑枠と `bed 59%` が表示されることを目視確認した。

## 未達 / 注意

- COCO の混雑画像では conf 0.32 でも recall 45% に留まり、
  **すべての物体を検出できていない**。特に small(<1024px²) は依然として弱い。
  さらに検出数を増やすにはより高精度なモデル・高解像度入力・マルチスケール推論が必要で、
  25Hz/CPU 50% との両立は未検証（★要確認★）。
- 実カメラにはベッドなど COCO クラスの物体が映っており、
  conf 0.32 で継続的に検出できていることを確認した。
  一方で手先カメラとしての実機マウントや、現場の全物体の検出は ★要確認★。
- INT8 量子化の score 張り付きは引き続き残る。
