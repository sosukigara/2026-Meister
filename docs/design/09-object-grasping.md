# 設計: 4軸アーム + カメラ + 画像認識 + 自動把持

> 対象機能: [features/09-object-grasping.md](../features/09-object-grasping.md) | 作成日: 2026-08-12 | ステータス: 物体検出のベース実装済み（アーム/IK/カメラ/把持は未実装）
> 関連ゴール: [G2](../やりたいこと.md#g2-自動でアームでものを回収)

## 設計メモ

- **姿勢推定の難所**: 任意物体の 6DoF 姿勢推定は既知の難題。開発初期は「マーカー（AprilTag/ArUco）併用」または「机上平面上の物体を真上から掴む」方式から始め、段階的に汎用化するのが現実的。
- **段階的運用**:
  - 第1段階: 対象がアーム可動域内にある場合のみ**停止したまま把持**
  - 第2段階: 対象が可動域外なら台車も移動して**接近してから把持**（自動移動機能と連携）
- **認識の置き場所**: PC/ROS 2 で実行（ESP32 は認識処理を持たない）。
- カメラは手首にあるため「アイインハンド」構成。ハンドアイキャリブレーションが必要。

## 認識パイプライン（実装済みベース）

物体検出のベースを `src/meister_vision/`（ROS2 ament_python パッケージ）として実装済み。
検出は「認識の置き場所」メモどおり PC / ROS 2 側で実行し、ESP32 は認識処理を持たない。

- **推論エンジン**: onnxruntime + YOLOv8n ONNX（COCO 80クラス）。軽量で torch / ultralytics 非依存。
- **処理**: letterbox 前処理（640x640）→ onnxruntime 推論 → 純 NumPy 実装の NMS で重複抑制。
- **トピック**:
  - 購読: `image_raw`（`sensor_msgs/Image`、パラメータで変更可）
  - 配信: `detections`（`vision_msgs/Detection2DArray`、bbox / クラス / 信頼度）
  - 配信: `detection_image`（`sensor_msgs/Image`、検出枠・ラベル描画済み）
- ノードは既定で最大 10 Hz にレート制限して負荷を軽く保つ。
- **実カメラ対応**: `yuv422_yuy2`（YUYV 4:2:2）エンコーディングに対応。`usb_cam` など一般的な USB カメラの生出力を直接扱える（`_bgr_from_image_msg` の手動変換パス）。
- **リアルタイム監視**: rviz2 の Image 表示で検出結果を確認できる。`detection.launch.py start_rviz:=true` で検出ノードと rviz2（`rviz/detection.rviz`、`/detection_image` 表示）を一括起動。
- **実装状況**: 物体検出のベース実装済み（PC 側）・実カメラでの動作確認済み。アーム / IK / カメラ映像取り込み / 把持は未実装（後続）。

## 設計図化対象

- [ ] アームの座標系と逆運動学（IK）の設計（4軸: 肩→肘→手首→グリッパー）
- [ ] サーボ制御系統図（肩: DS3225/PWM、肘・手首・グリッパー: STS3215/シリアルバス）
- [x] 認識パイプライン（カメラ → 検出 → 位置姿勢推定 → 把持計画）… 検出部のベースのみ実装済み（[meister_vision](../../src/meister_vision/)）
- [ ] 把持シーケンス状態遷移（検出 → 選択 → 接近/停止 → 把持 → 持ち上げ → 解放）
- [ ] ハンドアイキャリブレーション手順

## 関連ドキュメント

- 機能要件: [features/09-object-grasping.md](../features/09-object-grasping.md)
- 機能分解: [functions/09-object-grasping.md](../functions/09-object-grasping.md)
- 関連設計: [07-esp32-uart.md](07-esp32-uart.md)（アームサーボの制御経路） / [05-web-ui.md](05-web-ui.md)（対象選択 UI） / [10-auto-control.md](10-auto-control.md)（接近把持の移動連携）
