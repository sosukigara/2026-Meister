# 機能: 実機対応

> 対象リポジトリ: `sosukigara/2026-Meister` | 作成日: 2026-08-12 | ステータス: 計画中
> 設計: [design/06-real-robot.md](../design/06-real-robot.md)
> 機能分解: [functions/06-real-robot.md](../functions/06-real-robot.md)
> 関連ゴール: [G1](../やりたいこと.md#g1-ロッカーボギーロボット自動走行)

## 概要

シミュレーションで検証した SLAM / Nav2 スタックを実ロボットで動作させるための対応。launch ファイルの実装痕跡は存在するが、実機での動作検証は未実施。

## 実装済み機能

- [x] 実機起動 launch: `real_robot.launch.py` で `use_sim_time: false` に設定し、robot_state_publisher + SLAM + Nav2 を起動できる（外部の LiDAR / odom / Micro-ROS ドライバを前提）
- [x] 起動切替: `./launch.sh` が実機モード (既定)、`./launch.sh --sim [warehouse|maze|empty]` がシミュレーションモード。モード選択は `launch.sh` の 1 箇所に集約
- [x] LD-D500 ドライバ: `src/ros2_ldlidar_d500` に vendoring (MIT。上流 SamuelSRI/ros2_ldlidar_d500@ca450aa。公式に D500 用 ROS 2 ドライバはない)。`/scan` (230400 bps, frame `lidar_link`) を配信。由来と更新手順は `src/ros2_ldlidar_d500/VENDOR.md`
- [x] 実機配線: `real_robot.launch.py` に `d500_node` + `serial_bridge` + Web UI + 画像認識 (PCカメラ固定) を含めた。ポートは launch 引数 `serial_port` (既定 `/dev/ttyUSB0`) / `lidar_port` (既定 `/dev/ttyUSB1`) の 1 箇所で定義し、`launch.sh` は環境変数 `MEISTAR_SERIAL_PORT` / `MEISTAR_LIDAR_PORT` で上書きする

## 計画中機能

- [ ] 実機での SLAM 地図作成の検証
- [ ] 実機でのナビゲーション動作の検証
- [ ] 外部ドライバ（LiDAR・モータ・odom）との統合確認

## 未確定・要確認

- `★要確認★` D500 の `clockwise` / `angle_offset_deg` は実機搭載向きに合わせて調整すること (既定 `true` / `0.0`)。鏡像・回転していたら `ros2_ldlidar_d500/config/d500.yaml` ではなく `real_robot.launch.py` の引数で直す
- `★要確認★` 実機 odom: `serial_bridge` は現状 `/odom` を配信しない (FB_STATE→`/esp32/state` のみ)。SLAM は static `odom->base_footprint` で動作中だが、Nav2 の自己位置推定精度は実機走行で確認すること
- `✗ 未検証` vendored ドライバ自体の実機 `/scan` 出力 (`ros2 topic hz /scan` + RViz 表示)

## 実測 (2026-11-11、実機モード)

- LD-D500 (`/dev/ttyUSB0`、CP210x): `/scan` 9.95 Hz、720 bins、RViz で点群表示OK
- SLAM: `/map` 1 Hz (224x156 → 成長中、5 cm)。Web UI 地図表示 OK、Nav2 `nav2_ready:true`
- 画像認識 (EMEET C960 UVC、`/dev/video0`): `/camera/image_raw` 約30 Hz、`person 0.93` 等 14〜16 件検出、Web UI 認識モニター表示 OK。`/detection_image` は 0.8 Hz 前後（描画コスト）
- 権限: ユーザ `so` が `dialout` 未加入だとシリアルが開けない。暫定 `sudo chmod 666 /dev/ttyUSB0`、恒久は `usermod -aG dialout` 後再ログイン
- 既知の未接続: ESP32 (`serial_bridge` は `/dev/ttyUSB9` に逃がして待機中)、`/odom` なし
