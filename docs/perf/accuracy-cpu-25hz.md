# 25Hz 固定・高精度検出・CPU 40% 以下（2026-11-26）

## 判定

| 条件 | 結果 | 判定 |
|---|---|---|
| 物体をすべて正確に検出 | 手作業 GT の 2 画像・**7/7 物体**を検出、クラス **7/7 正解**、FP **0** | Measured（Known-object 回帰。COCO mAP ではない） |
| 25Hz 固定 | `/detections` **24.718 Hz**、`/detection_image` **24.7 Hz**、Web UI 表示 **24.7 fps** | Measured |
| CPU 40% 以下 | 検出ノード **32.3〜32.6%**（1 コア比）、Web UI サーバ 11.3〜11.8%（別プロセス） | Measured（検出ノードは 40% 以下） |

## 環境

- Orange Pi 5 Max / RK3588 / ROS 2 Jazzy / RKNN モデル `yolov8n-zoo.rknn`
- 実カメラ `/dev/video0`（Fushicai USB2.0、実入力 25 枚/秒）
- `output_fps=25`、`annotated_width=320`、`conf=0.45`、`iou=0.45`、`detection_fps=0`

## 精度（conf しきい値の選定）

`docs/perf/measure_static_accuracy.py` に `--conf` を追加し、しきい値を走査した。

| conf | bus.jpg（4 物体） | person_dog_horse.jpg（3 物体） |
|---|---|---|
| 0.30 | 4/4, FP 1 | 3/3, FP 0 |
| 0.35 | 4/4, FP 1 | 3/3, FP 0 |
| 0.40 | 4/4, **FP 0** | 3/3, **FP 0** |
| **0.45（採用）** | **4/4, FP 0, class 4/4, IoU 0.9177** | **3/3, FP 0, class 3/3, IoU 0.9449** |
| 0.50 | 4/4, FP 0 | 3/3, FP 0 |

FP は bus.jpg の person (score 0.3922) で、conf 0.40 以上で消える。
全 GT のスコアは 0.5 なので 0.45 でも取りこぼさない。故排除と検出の両立点として
0.45 を採用した（`pc_vision.launch.py` の既定も 0.45）。

GT は画像を見て手作業で付け、モデルの出力は正解に使っていない
（循環参照なし）。証拠:
[bus (conf 0.45)](logs/static_accuracy_bus_conf045.json) /
[person_dog_horse (conf 0.45)](logs/static_accuracy_pdh_conf045.json)。

**注**: これは 2 枚・7 物体の Known-object 回帰であり、COCO mAP ではない。
「現場の全物体を漏れなく検出する」ことを一般化して保証するものではない。
実カメラの現在の映し方（白い布など）には COCO クラスの物体が無く、
検出 0 件は誤検出でない。実機の手先カメラ映像での精度は ★要確認★。

## 25Hz / CPU の実測

20 秒 × 4 回の CPU サンプリング（`ps -o pcpu`、1 コア = 100%）:

| プロセス | CPU |
|---|---|
| `yolo_grasp_node` | 32.3 / 32.5 / 32.4 / 32.6%（平均 32.4%） |
| `umeonigiri_webnav_server` | 11.8 / 11.7 / 11.4 / 11.3% |

`/detections` 24.718 Hz（max 間隔 44ms）、`/detection_image` 24.7 Hz、
Web UI `/api/vision` fps 24.7、frame_age 7〜23ms。

CPU 40% は検出ノード（画像認識パイプライン）の値で判定した。
Web UI サーバは別プロセスで 11% 台。2 プロセスを単一コア換算で足すと
約 44% になるため、その定義での 40% 丁度は満たさない（★要確認★:
要求の「CPU 40%」がプロセス単位かシステム全体かは未確認）。
8 コア全体の使用率では 6% 前後。

## 失敗した試行

| 試行 | 結果 |
|---|---|
| 640x480 に描画してから 320x240 へ縮小（現行） | 検出ノード 30.9〜32.6% |
| 先に 320x240 へ縮小してから描画 | CPU 34.2〜34.9% に上昇 → revert |
| conf 0.35 | bus.jpg に FP 1 件 → 0.45 に変更 |

## 再現

```bash
# 精度
python3 docs/perf/measure_static_accuracy.py \
  --model <install>/.../yolov8n-zoo.rknn --image docs/perf/bus.jpg \
  --conf 0.45 --out docs/perf/logs/static_accuracy_bus_conf045.json
python3 docs/perf/measure_static_accuracy.py \
  --model <install>/.../yolov8n-zoo.rknn --image docs/perf/person_dog_horse.jpg \
  --conf 0.45 --out docs/perf/logs/static_accuracy_pdh_conf045.json

# 実行時（実カメラ + Web UI）
ros2 run umeonigiri_vision yolo_grasp_node --ros-args \
  -p device:=/dev/video0 -p output_fps:=25 -p annotated_width:=320 \
  -p conf:=0.45 -p detection_fps:=0
ros2 launch umeonigiri_webnav umeonigiri_webnav.launch.py
```
