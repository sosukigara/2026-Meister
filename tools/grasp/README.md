# 把持候補の可視化（yolov8n / NPU）

カメラ → NPU 推論（yolov8n, 80 クラス）→ 物体名と枠 →
枠の内側を OpenCV で輪郭解析して中心・角度・幅 → `/grasp_image` を RViz で表示。

![RViz での表示](grasp_rviz_window.png)

## 使い方

```bash
# モデルは tools/rknn の手順で用意する（tools/rknn/models/yolov8n-zoo.rknn）
python3 tools/grasp/yolo_grasp_stream.py \
  --model tools/rknn/models/yolov8n-zoo.rknn \
  --labels tools/grasp/coco_labels.txt

rviz2 -d tools/grasp/grasp_view.rviz
```

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

**推論時間の大半は Python 後処理**（DFL 展開 + NMS）。NPU 自体は 20.7 ms なので、
後処理を numpy で絞る（スコアで先に足切りしてから DFL する）か C++ へ移せば
数倍速くなる見込み。**未着手**。

## 未検証・注意

- 暗い室内では検出が誤検出だらけになる（実測: 暗所で 7 件、内容は概ねノイズ）。
  明るくコントラストのある場面で、対象物に枠が正しく付くかは ★要確認★。
- 角度・幅は **画素単位**。把持に使うにはカメラ〜物体距離との対応（キャリブ）が要る。
- 601 クラス（oiv7）へ差し替える場合は、このノードの `--labels` を差し替え、
  出力チャネル数（`[1,605,8400]`）に合わせたポストプロセスが要る
  （`tools/rknn/README.md` の変換手順も未解決）。
- モデルは `.rknn` のため git 管理外。無い場合は先に `tools/rknn` の手順で生成する。
