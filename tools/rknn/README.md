# NPU (RKNN) で推論を速くする — 2026-11-20 実測

画像認識の推論は **1 フレームの 98%** を占める（`docs/bench/2026-11-20-cpu-analysis.md` §12）。
CPU では 320 入力で 57 ms、640 入力で 217 ms。NPU に載せると桁が変わる。

## 実測結果（Orange Pi 5 Max / RK3588）

| 実行先 | 入力 | 1 フレーム | レート |
|---|---|---|---|
| CPU (ONNX Runtime, 2 threads, A76 固定) | 320 | 57 ms | 17.2 fps |
| CPU (同上) | 640 | 217 ms | 4.5 fps |
| **NPU (RKNN INT8, 1 core)** | **640** | **20.7 ms** | **48.3 fps** |
| NPU (RKNN INT8, 3 cores) | 640 | 24.1 ms | 41.5 fps |

`rknnlite` の Python 計測（`bench_inference.py` 相当の手順、40 回平均）。
**640 入力で CPU の 10 倍**。入力も 2 倍細かいので、速度と品質が同時に上がる。
3 core より 1 core が速いのは、このモデルが 1 core で足りているため（実測）。

## この機体だけで変換できる（重要な発見）

`rknn-toolkit2` は従来 x86_64 前提と言われていたが、**2.5.0 に aarch64 wheel がある**。

```bash
pip install --break-system-packages rknn-toolkit2 rknn-toolkit-lite2
```

そのため「x86 PC が要る」という前提は不要。実測でこの機体上で変換できた。

## 手順

1. runtime を入れる（`rknpu2` の aarch64 版）

   ```bash
   # https://github.com/rockchip-linux/rknpu2 の runtime/RK3588/Linux/librknn_api/aarch64
   sudo cp librknnrt.so /usr/lib/ && sudo ldconfig
   # ヘッダは /usr/local/include/rknn/ に置く（C++ から使う場合）
   ```

2. 変換用 ONNX を取得する。**公式 Ultralytics のエクスポート（1 出力）は変換できない**
   （実測: `IndexError: list index out of range`）。Rockchip の最適化モデルを使う。

   ```bash
   git clone --depth 1 --filter=blob:none --sparse https://github.com/airockchip/rknn_model_zoo.git
   cd rknn_model_zoo && git sparse-checkout set examples/yolov8
   cd examples/yolov8/model && ./download_model.sh   # yolov8n.onnx (opset 12, 9 outputs)
   ```

3. キャリブ画像を 50〜100 枚用意して 1 行 1 パスで列挙する。

   ```bash
   find /path/to/images -name '*.jpg' | head -60 > /tmp/calib/dataset.txt
   python3 tools/rknn/convert_yolov8n.py \
       --onnx /tmp/zoo/examples/yolov8/model/yolov8n.onnx \
       --dataset /tmp/calib/dataset.txt --out /tmp/yolov8n-zoo.rknn
   ```

4. 実行（Python で確認する場合）

   ```python
   from rknnlite.api import RKNNLite
   r = RKNNLite(); r.load_rknn("yolov8n-zoo.rknn")
   r.init_runtime(core_mask=RKNNLite.NPU_CORE_0)
   outs = r.inference(inputs=[img_nhwc], data_format="nhwc")  # (1,640,640,3) の uint8
   ```

## 注意

- **入力は `(1, H, W, 3)` の 4 次元にすること。** 3 次元だと
  `Catch exception when setting inputs` で `None` が返る（実測）。
- 出力は 9 本（3 スケール × 箱/クラス/合計）。既存の `/detections` を作るには
  Rockchip のデコード（`rknn_model_zoo/examples/yolov8/python/yolov8.py`）を
  C++ へ移植する必要がある。**この移植は未着手**。
- INT8 量子化の精度劣化は **✗ 未検証**（ラベル付きデータでの mAP 比較が必要）。
- 変換ログに `size_with_stride larger than model origin size` の警告が大量に出るが、
  推論結果は得られている（実測）。抑制方法は未確認。
- `.rknn` は生成物なので git 管理外（`.gitignore`）。

## 任意モデル（yolov8n-oiv7 など）を変換する場合の現状 — 2026-11-20 実測

yolov8n-oiv7（601 クラス）を 30 fps で回すために変換を試した記録。
**変換は未完了**。詰まっている点を残す。

### できたこと

- `yolo export model=yolov8n-oiv7.pt format=onnx opset=12 imgsz=640` は成功する。
  出力は `[1, 605, 8400]`（4 box + 601 クラス、objectness 無し）14 MB。
- `rknn-toolkit2` はこの機体で動く（numpy 1.26 / torch 2.2.0 で確認）。

### 詰まっている点

1. **標準エクスポートの ONNX は RKNN に変換できない。**
   `rknn.build()` がグラフ融合で落ちる:
   `IndexError: list index out of range`（`rules/conv.py:4860 _p_seperate_conv_rs_split`）。
   `optimization_level=0`、`do_quantization=False` でも同じ。
   Rockchip 配布の最適化 ONNX（`rknn_model_zoo`）はこの規則に当たらないので通る
   （yolov8n で実測済み）。
2. **Rockchip 版 ultralytics（`airockchip/ultralytics_yolov8`）でのエクスポートが未完。**
   フォーク自体は読み込める（8.2.82）が、エクスポート時に `onnxscript` を要求し、
   その依存 `ml_dtypes` が numpy 2 前提で、システムの numpy 1.26 と衝突する。

### 依存を触った結果（重要）

`pip install ultralytics` は **torch 2.15 と numpy 2 を入れてしまい**、
- `rknn-toolkit2` の要求（torch<=2.2.0）に反する
- システムの matplotlib が壊れる（numpy 2 非対応ビルド）
- `colcon` と競合する setuptools 84 が入る

復旧として `torch==2.2.0` / `numpy<2` / `setuptools<80` に固定し、
`colcon build` と `rknn` の import が通ることを確認した。
**次に試すなら venv を切ってから**（システム Python を汚さない）。

### 代替（今すぐ 30 fps を出す方法）

`rknn_model_zoo` の `yolov8n.onnx`（80 クラス、Rockchip 最適化済み）は変換でき、
**NPU で 20.7 ms = 48 fps**（実測、`tools/rknn/README.md` 冒頭）。
クラス数は 80 だが速度は要件を満たす。
