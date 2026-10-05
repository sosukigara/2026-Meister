# 実行ターゲットと画像認識性能の実測基準

> 対象機能: [features/09-object-grasping.md](features/09-object-grasping.md) /
> [features/10-auto-control.md](features/10-auto-control.md)
> 作成日: 2026-10-01 | ステータス: 開発機は実測済み。Orange Pi 5 Max は未実測

このファイルが持つのは 2 つだけ。

1. **実行マシンの確定**（Orange Pi 5 Max / 8GB）— 宣言はこの場所以外へ置かない
2. **画像認識 1 フレームあたりの実測コスト**— 実装判断の材料

「重い」という判断の根拠となる数字をここに固定する。
推測で埋めた値には必ず「要実測」を付ける。

---

## 1. 実行ターゲット

### 1.1 確定値（出典つき）

出典: Orange Pi 公式 User Manual **v1.2**
（`orangepi.net/wp-content/uploads/2024/09/OrangePi_5_Max_RK3588_User-Manual_vv1.2-1.pdf`）
および `orangepi.org` の製品ページ。2026-10-01 取得。

| 項目 | 値 | 使い道 |
|---|---|---|
| SoC | Rockchip **RK3588**（8nm LP） | ツールチェーン選定（aarch64） |
| CPU | **4xCortex-A76 @2.4GHz** + 4xCortex-A55 @1.8GHz |推論に使える core は 4 個が上限 |
| GPU | **Mali-G610 MP4**（OpenCL 2.2 / Vulkan 1.2） | CUDA はない。`onnxruntime-gpu` は使えない |
| NPU | **6 TOPS**（INT4/INT8/INT16 混在対応） | YOLO を載せる唯一の現実的な高速化経路 |
| メモリ | **8GB LPDDR5** | Nav2 と Gazebo と推論の同時起動可否の制約 |
| OS | Orange Pi OS / Debian 11・12 / Ubuntu 20.04・22.04 / Android 13 | 開発環境の選択 |

### 1.2 派生する制約（設計の前提）

- **CUDA 経路は消える。** `onnxruntime-gpu` の CUDA 実行プロバイダは
  Orange Pi 5 Max で動かない。開発機で providers を列挙すると
  `AzureExecutionProvider` と `CPUExecutionProvider` しか無い状態と**同じ**になる。
  GPU を使うなら CUDA ではなく **Mali-G610（OpenCL / Vulkan）または NPU（RKNN）**。
- **並列 core の上限が 6 から 4 に減る。** 開発機は 12 スレッド。
  並列化の勘定を開発機の数字で済ませると 2〜3 倍だけ楽観になる。
- **メモリの余裕が無い。** 開発機（ノートPC）は全 13811 MiB のうち
  使用 11340 MiB・free 345 MiB・available 2470 MiB。8GB 機ではさらに厳しい。
  Gazebo と RViz と Nav2 の同時起動が重い部分は Orange Pi 側で切り分ける必要が出る。
- **NPU を使うなら推論経路そのものを差し替える。**
  onnxruntime の実行プロバイダを差し込むだけでは NPU に乗らない。RKNN Runtime が要る。

### 1.3 要実測（Orange Pi 実機。すべて未確認）

Orange Pi 5 Max 8GB 実機で以下を測っていない。開発機の値からの外挿はしていない。

- [ ] `onnxruntime` の CPU プロバイダで yolov8n 640x640 の 1 フレーム遅延（4 スレッド）
- [ ] Gazebo と Nav2 と推論を同時に起動したときの core 内訳
- [ ] 8GB で 3 つを同時起動したときの RSS 合計と OOM 発生点
- [ ] RKNN 変換後（`.rknn`）の推論遅延。6 TOPS のうち yolov8n で
      実用的な fps が出るかは NPU 側の実装（量子化）次第で、TOPS から推定しない

---

## 2. 画像認識のコスト（開発機で実測）

### 2.1 測定条件

| 項目 | 値 |
|---|---|
| 測定機 | **ノートPC** / **AMD Ryzen 5 7430U**（6 core / 12 thread、AVX2、AVX512 は無し） |
| OS | Ubuntu / x86_64 |
| ランタイム | `onnxruntime` 1.28.0 / `CPUExecutionProvider` |
| モデル | `src/meister_vision/models/yolov8n.onnx`（Ultralytics YOLOv8n / COCO 80 クラス） |
| 入力 | `[1, 3, 640, 640]` float32（入力サイズは `letterbox.hpp` の `kYoloInputSize = 640` で固定） |
| 出力 | `[1, 84, 8400]`（`end2end:False`。NMS は実行器の責務） |
| 入力データ | ランダムノイズ画像（**推論時間**の測定であり、検出精度の測定ではない） |
| 方法 | 事前の暖機 3 回のあとに 15 回の wall clock 平均。`intra_op_num_threads` のみ変更 |

> **この節の数値は移行前の Python 版（onnxruntime 1.28.0）で測った旧基準。**
> C++ 化で組み込まれた ONNX Runtime は `ros-jazzy-onnxruntime-vendor` の
> **1.24.3** でバージョンが違う。上の数値を C++ の性能基準として引用しないこと。
> C++ での再測定は **✗ 未検証**（Orange Pi 5 Max と開発機の双方で未実施）。

### 2.2 実測結果

| intra_op_num_threads | 1 フレーム | 理論上限 |
|---|---|---|
| 1 | **197.6 ms** | 5.1 Hz |
| 2 | **115.6 ms** | 8.7 Hz |
| 4 | **69.2 ms** | 14.4 Hz |

### 2.3 この数字が示すこと（開発機上の事実）

- `detection_node` の既定のレート制限は 10 Hz
  （`docs/design/09-object-grasping.md:26`）。**1 スレッドでは 10 Hz に
  届かない**（天井 5.1 Hz）。2 スレッドでも 8.7 Hz で届かない。
  つまり開発機でも「10 Hz 設定」は実際の達成値になっていない。
- 4 スレッドでようやく 14.4 Hz の天井に達する。推論 1 本が CPU 4 コア分の
  帯域を食うのが、Nav2 と SLAM と Gazebo と同一 core で競合する理由。
- NMS は C++ 側（`cv::dnn::NMSBoxes`）で、この測定には**含まれていない**。
  実運用では推論時間と NMS 時間と letterbox 時間が加算される。

### 2.4 Orange Pi 5 Max にそのまま当てはめない理由

開発機と Orange Pi 5 Max では CPU が別物。

| | 開発機 | Orange Pi 5 Max |
|---|---|---|
| 大 core | Zen3 @ 最高 4.388GHz（AVX2） | Cortex-A76 @ 2.4GHz（NEON、AVX512 は無し） |
| 並列 core 数 | 6（12 スレッド） | **4** |
| 数値性能 | 上表のとおり | **未実測**（外挿しない） |

方向性として「開発機より大きく遅くなる」ことは確実だが、
何倍になるかは Orange Pi 実機で測るまで書かない。

---

## 3. この記録で問題視しないこと（未検証の明示）

| 項目 | 判定 |
|---|---|
| C++ 化後の推論の遅延（開発機 / Orange Pi） | **✗ 未検証**（ONNX Runtime が 1.28.0 → 1.24.3 に変わっている。2.1 の手順で再測定する） |
| 開発機での yolov8n の遅延 | **一次情報で実測**（本ファイル 2.2） |
| 開発機の並列 core 数と RAM の余裕 | **実測**（`lscpu` / `free -m` / providers の一覧） |
| Orange Pi 5 Max の SoC 構成 | **一次情報で確定**（公式マニュアル v1.2） |
| Orange Pi 5 Max での推論の遅延 | **未検証**（2.1 の条件を実機で同じ手順で適用する） |
| NPU（RKNN）での実用 fps | **未検証**（TOPS から推定しない） |
| Gazebo と Nav2 と推論の同時起動の可否 | **未検証**（8GB 機で未計測） |

---

## 4. 関連ドキュメント

- 機能要件: [features/09-object-grasping.md](features/09-object-grasping.md)
- 設計: [design/09-object-grasping.md](design/09-object-grasping.md)
- コードマップ: [code-map.md](code-map.md)
- 起動スクリプト: `start_meister.sh`（Gazebo と SLAM と Nav2 を起動。
  vision は `mapping_nav.launch.py` 経由で上がる）