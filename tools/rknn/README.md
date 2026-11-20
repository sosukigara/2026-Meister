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
