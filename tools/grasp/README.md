# 把持候補の可視化（yolov8n / NPU）

カメラ → NPU 推論（yolov8n, 80 クラス）→ 物体名と枠 →
枠の内側を OpenCV で輪郭解析して中心・角度・幅 → `/grasp_image` を RViz で表示。

![RViz での表示](grasp_rviz_window.png)

## 使い方（既定の経路）

**NPU 実装が既定**になった。`launch.sh` の実機モードは、RKNN モデルがあれば
自動で NPU を選ぶ。ROS から直接起動する場合:

```bash
# 既定 backend:=npu（RKNN）。ONNX(CPU) へ戻すなら backend:=onnx
ros2 launch meister_vision pc_vision.launch.py
ros2 launch meister_vision pc_vision.launch.py backend:=onnx

rviz2 -d tools/grasp/grasp_view.rviz   # /grasp_image を見る場合
```

- 実体は `ros/src/meister_vision/scripts/yolo_grasp_stream.py`（パッケージに install 済み）。
- モデルは `ros/src/meister_vision/models/yolov8n-zoo.rknn` に置くと share へ入る。
  `tools/rknn/README.md` の手順で生成する（`.rknn` は git 管理外）。
- ラベルは `ros/src/meister_vision/config/coco_labels.txt`。
- 出力は既存と同じ `./detections`（vision_msgs/Detection2DArray）、
  `./detection_image`、`./target`。加えて `./grasp_image`（把持候補の描画）。
  **Web UI とアーム制御は ONNX 版と共通のまま**で、backend を切り替えられる。

- 一番大きい枠（＝手前にあるとみなす）を **把持ターゲット**として緑枠で強調し、
  画面下に `TARGET <名前> (x,y) angle <度> width <px>` を出す。
- 他の検出は灰枠。マスクは出さない（セグメンテーションはしない）。
- 角度は `cv2.minAreaRect` の長辺方向。`width` は長辺の画素長で、
  指の開き幅の目安。**実寸への換算は未実装**（カメラ距離が必要）。

## 実測（2026-11-20 / Orange Pi 5 Max）

| 項目 | 値 |
|---|---|
| 表示レート | 23 fps（カメラ律速、単独実行時） |
| 推論（NPU のみ） | 20.7 ms（`tools/rknn` のベンチ、48 fps） |
| 推論（このノードの実測） | 114 ms（アイドル時）／176 ms（RViz 同居時） |

### C++ 移植後の実測（2026-11-20）

| 実装 | 推論 | CPU（1 コア換算） |
|---|---|---|
| Python（rknnlite） | 157 ms | 133% |
| **C++（librknnrt 直叩き）** | **24.6〜25.7 ms** | **33%** |

**目標（30 fps / CPU 40%）を両方満たした。** 25 ms は約 40 fps 相当。
理由は単純で、C++ 版は NPU の出力を int8 のまま受け取り、しきい値を超えた
アンカーだけ逆量子化して DFL を回す。Python 版は `rknnlite` が 9 本の出力
（54 万要素）を毎フレーム float へ展開し、さらに配信スレッドと GIL を奪い合っていた。

`yolov8n-zoo.rknn` はキャリブ画像が暗所のフレームだったため、クラス確率の
量子化レンジが 0〜0.5 に圧縮されている（実測: cls の scale 0.00196 / zp -128）。
明るい画像で再キャリブすれば、より高い確信度が出せる。★要確認★

### Python 実装（参考・予備）

`ros/src/meister_vision/scripts/yolo_grasp_stream.py` は同じ処理の Python 版。
既定では使わない（C++ 版が既定）。挙動比較や、librknnrt が無い環境での
代替として残してある。

### 旧: CPU 使用率（C++ 移植前の記録）

| 条件 | CPU（1 コア換算） | 推論 |
|---|---|---|
| 初期実装（conf 0.25） | **138%** | 158 ms |
| スコア先行足切り＋配信 15 fps 上限 | **133%** | 157 ms |

**後処理を速くしても CPU はほとんど変わらなかった。** 内訳を測ると NPU 自体は
20.7 ms なのに 1 フレーム 157 ms かかっており、差分の大半は `rknnlite.inference()`
が **9 本の出力（合計 54 万要素）を毎回 float へ展開する処理**と、Python 側の
GIL 競合（配信スレッドが 921 KB のコピーを回す間、推論スレッドが止まる）。

→ **40% を狙うなら C++ へ移すのが本筋**（`tools/fastsam/fastsam_stream.cc` と同じ構成で、
出力を int8 のまま受け取りデコードする）。未着手。

### 目標に対する現状

- 速度: 表示 23 fps（カメラ律速）／推論 157 ms → **未達**（目標 33 ms）
- CPU: **133%** → **未達**（目標 40%）
- どちらも Python 実装が律速。NPU 性能（20.7 ms）には余力がある。

## 未検証・注意

- 暗い室内では検出が誤検出だらけになる（実測: 暗所で 7 件、内容は概ねノイズ）。
  明るくコントラストのある場面で、対象物に枠が正しく付くかは ★要確認★。
- 角度・幅は **画素単位**。把持に使うにはカメラ〜物体距離との対応（キャリブ）が要る。
- 601 クラス（oiv7）へ差し替える場合は、このノードの `--labels` を差し替え、
  出力チャネル数（`[1,605,8400]`）に合わせたポストプロセスが要る
  （`tools/rknn/README.md` の変換手順も未解決）。
- モデルは `.rknn` のため git 管理外。無い場合は先に `tools/rknn` の手順で生成する。
