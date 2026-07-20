# チームうめおにぎり 2026マイスター - 自動走行ロボット

Team うめおにぎり が 2026 マイスター作品として製作する**自動走行ロボット**。
3層アーキテクチャで構成：ROS2 Nav2 自律制御 / YOLO画像認識 / ESP32 micro-ROS低レイヤ制御。
Gazeboシミュレーションで開発後、実機へ移行する。

## 作品コンセプト

「センサで環境を認識し、自律的に走行するロボット」

人を追跡しながら障害物を回避して目的地に移動できる。カメラ画像から人と物を判別し、
状況に応じた行動を選択する。シミュレーションと実機の両方で動作する。

## システムアーキテクチャ

```
┌──────────────────────────────────────────────────────────┐
│                    ROS2 (Jazzy)                          │
│                                                          │
│  ┌──────────────────┐   ┌──────────────────────────┐    │
│  │ 画像認識ノード      │   │ 自律制御 (Nav2)           │    │
│  │ yolo_ros          │   │ Nav2 stack               │    │
│  │ ・物体検出/分類     │   │ ・SLAM Toolbox            │    │
│  │ ・人体追跡          │──→│ ・MPPI controller        │    │
│  │ ・3D位置推定       │   │ ・Behavior Tree           │    │
│  └────────┬─────────┘   │ ・Frontier探索             │    │
│           │              └──────────────────────────┘    │
│           │ pub/sub                                       │
│           ▼                                               │
│  ┌──────────────────────────────────────────────────┐    │
│  │ 統合ノード (fusion_node)                           │    │
│  │ 画像認識→速度指令変換, 状態管理                     │    │
│  └──────────────────────────────────────────────────┘    │
│                           │                               │
│                    micro-ROS Agent                        │
│                    (Serial/UDP bridge)                    │
└───────────────────────────┼───────────────────────────────┘
                            │
                    ┌───────┴───────┐
                    │   ESP32 (micro-ROS)                  │
                    │ ・/cmd_vel 購読                      │
                    │ ・PID速度制御 (20Hz)                  │
                    │ ・エンコーダ読み取り                   │
                    │ ・/odom 配信                         │
                    │ ・IMU (BNO085) 読み取り              │
                    └───────┬───────┘
                            │
                    ┌───────┴───────┐
                    │   モータドライバ                     │
                    │   L298N / MDD10A                    │
                    │   DCモータ x2 (差分駆動)             │
                    └─────────────────────────────────────┘
```

### 3層アーキテクチャ

| 層 | 役割 | 技術 |
|---|---|---|
| **上位層** | 自律制御・判断 | ROS2 Nav2 (SLAM, 経路計画, 障害物回避, Behavior Tree) |
| **認識層** | 視覚認識・状況理解 | YOLOv11 + ByteTrack 画像認識 (物体検出/人体追跡/3D位置推定) |
| **制御層** | モータ制御・センサ融合 | ESP32 micro-ROS (PID速度制御, エンコーダ, IMU) |

## 実装計画（6フェーズ）

| Phase | 内容 | 状態 |
|---|---|---|
| **Phase 0** | 基盤整備 — ROS2環境構築、パッケージ復元、GitHub設定 | ✅ 完了 |
| **Phase 1** | ESP32 低レイヤ制御 (PlatformIO / micro-ROS) — モータ制御、エンコーダ、PID | ✅ 完了（ハードウェアテスト除く） |
| **Phase 2** | 画像認識ノード (YOLOv11 + ByteTrack) — 物体検出、人体追跡 | ✅ 完了 |
| **Phase 3** | 自律走行統合 — fusion_node, Nav2 BT カスタマイズ | ✅ 完了 |
| **Phase 4** | シミュレーション検証 — Gazebo E2E (人検出→追跡→追従, 障害物回避) | ⏳ 進行中（Gazebo init がボトルネック） |
| **Phase 5** | 実機製作 — シャーシ/配線組立、実機パラメータ調整、走行テスト | 📝 未着手 |
| **Phase 6** | 作品仕上げ — launch統一、ドキュメント完成、デモ/発表準備 | 📝 未着手 |

### コンポーネント進捗状況

| Phase | コンポーネント | 状態 | 説明 |
|-------|--------------|------|------|
| **0** | 基盤整備 | ✅ | ROS2 Jazzy 環境構築、ワークスペース復元、GitHub管理 |
| **1** | ESP32 ファームウェア | ✅ | micro-ROS 初期化、cmd_vel 購読、odom 配信 |
| **1** | モータ制御 | ✅ | 逆運動学 → PWM出力、L298N/MDD10A対応 |
| **1** | エンコーダ | ✅ | PCNT モジュールによるホイールエンコーダ読み取り |
| **1** | PID 速度制御 | ✅ | 20Hz PID 閉ループ制御、各車輪独立 |
| **1** | IMU (BNO085) | ✅ | I2C 経由回転ベクトル 100Hz 読み取り |
| **1** | ハードウェア結合テスト | ⏳ | ESP32実機 + モータドライバ + エンコーダ統合テスト |
| **2** | YOLO 認識ノード | ✅ | YOLOv11 物体検出パイプライン (yolo_ros) |
| **2** | 人体追跡 | ✅ | ByteTrack による人物追跡 + 3D位置推定 |
| **3** | 統合ノード (fusion_node) | ✅ | FSM 状態管理 (follow/search/pause/explore) |
| **3** | Nav2 カスタム BT | ✅ | person_follow.xml (5Hz 再計画、段階的リカバリ) |
| **3** | ミッションサーバ | ✅ | 目標管理・状態遷移 |
| **4** | Gazebo シミュレーション | ⏳ | ワールド起動確認済み、E2E 検証は Gazebo init 問題で保留 |
| **5** | 実機製作 | 📝 | シャーシ組立、配線、実機パラメータ調整 |
| **6** | ドキュメント/発表 | 📝 | README完成、デモ準備 |

### Must-Have (スコープ内)

- ROS2 Nav2 による自律走行 (SLAM + 経路計画 + 障害物回避)
- カメラ画像からの人検出 + 追跡
- ESP32 micro-ROS によるモータ制御
- Gazebo シミュレーション対応
- 1コマンド起動

### Must-NOT-Have (初期スコープ外)

- 強化学習 (後日拡張可能)
- マルチロボット
- クラウド連携
- 音声認識

## 技術スタック

| 層 | 技術 | バージョン |
|---|---|---|
| OS | Ubuntu 24.04 LTS | Noble |
| ROS | ROS2 Jazzy | Jazzy |
| ナビゲーション | Nav2 + SLAM Toolbox | Jazzy |
| シミュレーション | Gazebo Harmonic | Harmonic |
| 画像認識 | YOLOv11 + ByteTrack | Ultralytics |
| マイコン | ESP32 + micro-ROS | PlatformIO |
| IMU | BNO085 | I2C |
| モータ制御 | L298N / MDD10A | PWM |
| 言語 | Python / C++ (ROS) + C++ (ESP32) | - |

## ディレクトリ構成

```
Meister/
├── README.md
├── .omo/                        # Plan artifacts
├── src/
│   ├── diff_drive_robot/        # ROS2 パッケージ (自律制御/統合)
│   │   ├── launch/              #   起動ファイル (slam_nav, robot, bringup…)
│   │   ├── config/              #   Nav2 パラメータ、BT XML、gz_bridge
│   │   │   └── bt/              #   Behavior Tree XML (person_follow.xml…)
│   │   ├── urdf/                #   ロボットモデル
│   │   ├── rviz/                #   RViz 設定
│   │   ├── scripts/             #   実行可能Pythonノード
│   │   │   ├── fusion_node.py   #   [Phase 3] 統合ノード (FSM)
│   │   │   ├── mission_server.py#   [Phase 3] ミッション管理
│   │   │   ├── bt_executor.py   #   BT 制御
│   │   │   └── ...              #   他スクリプト
│   │   ├── maps/                #   地図データ
│   │   ├── worlds/              #   Gazebo ワールド
│   │   └── bt_editor/           #   BT エディタ設定
│   │
│   └── perception/              # ROS2 パッケージ (画像認識)
│       ├── launch/
│       ├── config/
│       ├── perception/
│       │   ├── __init__.py
│       │   └── human_tracker.py #   [Phase 2] ByteTrack 人体追跡
│       └── setup.py
│
├── firmware/                    # [Phase 1] ESP32 PlatformIO ファームウェア
│   ├── src/
│   │   ├── main.cpp             #   micro-ROS エントリ
│   │   ├── motor_control.cpp    #   モータPWM制御
│   │   ├── encoder.cpp          #   PCNT エンコーダ読取
│   │   ├── pid.cpp              #   PID 速度制御
│   │   └── imu.cpp              #   BNO085 I2C 読取
│   ├── include/                 #   ヘッダファイル
│   └── platformio.ini
│
├── build/                       # colcon build output
├── install/                     # colcon install output
└── log/                         # colcon build logs
```

## ROS2 パッケージ詳細

### `diff_drive_robot` (自律制御・統合)

差動駆動ロボットのメインパッケージ。Nav2 ナビゲーションスタック、ロボットモデル、統合ノード、ミッション管理を含む。

| コンポーネント | ファイル | 役割 |
|--------------|---------|------|
| **ロボットモデル** | `urdf/` | 差動駆動ロボットのURDF/Xacro記述 |
| **Nav2 設定** | `config/nav2_params.yaml` | ナビゲーションパラメータ (コントローラ, コストマップ, プランナ) |
| **SLAM 設定** | `config/mapper_params_online_async.yaml` | オンライン非同期 SLAM Toolbox 設定 |
| **カメラブリッジ** | `config/gz_bridge.yaml` | Gazebo カメラトピック → ROS へのブリッジ設定 |
| **BT: 人物追従** | `config/bt/person_follow.xml` | 人物追従用カスタム Behavior Tree (5Hz 再計画, 段階的リカバリ) |
| **BT: 巡回** | `config/bt/patrol_loop.xml` | 複数地点巡回用 BT |
| **BT: ナビゲーション** | `config/bt/navigate_w_recovery.xml` | 標準回復動作付きナビゲーション BT |
| **統合ノード** | `scripts/fusion_node.py` | FSM 状態管理 (follow/search/pause/explore) で認識→速度指令変換 |
| **ミッションサーバ** | `scripts/mission_server.py` | 目標管理・タスク状態遷移 |
| **BT エグゼキュータ** | `scripts/bt_executor.py` | Nav2 BT トリガ・監視 |
| **フロンティア探索** | `scripts/frontier_explorer.py` | 未知領域自律探索 |
| **フロンティア調整** | `scripts/frontier_coordinator.py` | マルチロボット探索調整 |
| **衝突監視** | `scripts/collision_monitor.py` | 衝突リスク監視・緊急停止 |
| **デッドロック回復** | `scripts/deadlock_recovery.py` | スタック検出・回復行動 |

**起動ファイル**

| ファイル | 内容 |
|---------|------|
| `slam_nav.launch.py` | フルスタック: SLAM + Nav2 + Gazebo + RViz |
| `robot.launch.py` | ロボット+Gzシミュレーションのみ |
| `slam.launch.py` | SLAM 単独起動 |
| `nav2.launch.py` | Nav2 単独起動 |
| `bringup_launch.py` | ロボットブリングアップ |
| `rsp.launch.py` | ロボット状態パブリッシャ |

### `perception` (画像認識)

YOLOv11 + ByteTrack による画像認識・人体追跡パッケージ。

| コンポーネント | ファイル | 役割 |
|--------------|---------|------|
| **検出設定** | `config/yolo_ros_params.yaml` | YOLOv11 モデル・閾値設定 |
| **人体追跡** | `perception/human_tracker.py` | ByteTrack 追跡 + 3D位置推定ノード |
| **起動** | `launch/perception.launch.py` | yolo_ros + human_tracker 一括起動 |

`human_tracker_node` は `perception` パッケージのエントリポイントとして登録されており、追跡結果を `tracked_person` トピックに配信。fusion_node が購読して速度指令に変換する。

## ファームウェア詳細 (ESP32 / PlatformIO)

**環境**: ESP32 DevKitC, Arduino フレームワーク, micro-ROS Jazzy, シリアル転送

| モジュール | ファイル | 役割 |
|----------|---------|------|
| **メイン** | `src/main.cpp` | micro-ROS ノード初期化、cmd_vel 購読 (20Hz 制御ループ)、odom 配信 |
| **モータ制御** | `src/motor_control.cpp` | 逆運動学計算 → LEDC PWM 出力、L298N/MDD10A Hブリッジ制御 |
| **エンコーダ** | `src/encoder.cpp` | ESP32 PCNT モジュールでホイールエンコーダパルス読み取り |
| **PID** | `src/pid.cpp` | 20Hz PID 速度制御 (各車輪独立、目標値は逆運動学から算出) |
| **IMU** | `src/imu.cpp` | BNO085 I2C 経由 100Hz 回転ベクトル (クォータニオン) 読み取り |
| **設定** | `include/conf_hardware.h` | GPIO ピン割当、PID ゲイン、制御ループ周波数 |
| **ビルド** | `platformio.ini` | PlatformIO 設定、micro-ROS Jazzy 依存、シリアル 115200 baud |

制御フロー: `cmd_vel` (ROS) → 逆運動学 → 目標各車輪速度 → PID → PWM → DC モータ → エンコーダ → 実速度フィードバック → odom 配信

## ハードウェア要件

| コンポーネント | 型番 | 数量 | 備考 |
|--------------|------|:----:|------|
| **メインコンピュータ** | Raspberry Pi 4/5 または Jetson Nano/Orin | 1 | Ubuntu 24.04, ROS2 Jazzy 実行 |
| **マイクロコントローラ** | ESP32 DevKitC (ESP-WROOM-32) | 1 | micro-ROS ファームウェア実行 |
| **IMU** | BNO085 (Adafruit BNO08x ボード) | 1 | I2C, 100Hz 回転ベクトル出力 |
| **モータドライバ** | L298N または MDD10A | 1 | デュアルHブリッジ, PWM 入力 |
| **DC モータ** | エンコーダ付き DC モータ x2 | 2 | 差動駆動, エンコーダパルス → PCNT |
| **カメラ** | USB カメラまたは Raspberry Pi Camera | 1 | Gazebo シミュレーションでは仮想カメラ |
| **電源** | 12V DC (モータ用) + 5V (ロジック用) | 1 | モータドライバ・ESP32 へ給電 |
| **バッテリ** | LiPo 3S 11.1V または類似 | 1 | 実機自立電源 |
| **シャーシ** | 2輪差動駆動シャーシ | 1 | 市販または自作 |

配線概要: ESP32 GPIO → L298N IN1-IN4,ENA,ENB → DC モータ x2。エンコーダ A/B 相 → ESP32 PCNT 入力。BNO085 SDA/SCL → ESP32 I2C。USB カメラ → メインコンピュータ。メインコンピュータ UART → ESP32 (micro-ROS シリアル転送)。

## 開発メモ・既知の課題

- **ROS_DOMAIN_ID**: 全ノードで `ROS_DOMAIN_ID=42` を使用。実機とシミュレーションの混信防止。`.bashrc` への追跡推奨: `export ROS_DOMAIN_ID=42`
- **micro-ROS エージェント**: ESP32 との通信には `micro_ros_agent` シリアルブリッジが必要。起動例: `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0`
- **Gazebo Harmonic init**: Phase 4 シミュレーション検証は Gazebo 初期化問題で保留中。ワールド起動は確認済みだが、E2E 人追跡テストが未完了。
- **fusion_node 配置**: `fusion_node.py` は `diff_drive_robot/scripts/` にあり、パッケージ内の Python モジュールディレクトリではない。CMakeLists.txt で `scripts/` ディレクトリをインストール対象として登録。
- **colcon build 順序**: `perception` パッケージは `diff_drive_robot` に依存しないため、並行ビルド可能。両方とも `colcon build --symlink-install` で一括ビルド。
- **PCNT エンコーダ**: ESP32 PCNT モジュールはハードウェアパルスカウント。ピン割当は `conf_hardware.h` で設定。エンコーダの CPR に応じて `ENC_PULSES_PER_REV` を調整。
- **PID ゲイン調整**: `PID_GAINS_DEFAULT` は `conf_hardware.h` で定義。実機に合わせて Kp/Ki/Kd を調整すること。
- **BV ツリーファイル**: デザイン資料は `references/` ディレクトリに格納。システムブロック図、UI モックアップなど。

## クイックスタート

### 前提条件

- Ubuntu 24.04 LTS
- ROS2 Jazzy インストール済み
- Gazebo Harmonic インストール済み

### 環境変数

```bash
export ROS_DOMAIN_ID=42          # 混信防止
source /opt/ros/jazzy/setup.bash
```

### ビルド

```bash
# ワークスペースルートでビルド
colcon build --symlink-install

# 環境設定
source install/setup.bash
```

### 起動

```bash
# フルスタック起動 (SLAM + Nav2 + Gazebo + RViz)
ros2 launch diff_drive_robot slam_nav.launch.py world_name:=maze headless:=false

# headless 起動 (GUIなし)
ros2 launch diff_drive_robot slam_nav.launch.py world_name:=maze headless:=true

# ロボット + Gazebo (ナビなし)
ros2 launch diff_drive_robot robot.launch.py world_name:=maze

# SLAM のみ
ros2 launch diff_drive_robot slam_launch.py

# 画像認識パイプライン (YOLO + 人体追跡)
ros2 launch perception perception.launch.py

# micro-ROS エージェント (ESP32 実機接続時)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

ビルド確認用スクリプト:

```bash
./build.sh
```

### ファームウェア (ESP32)

```bash
cd firmware/
pio run                   # ビルド
pio run --target upload   # 書き込み
pio device monitor        # シリアルモニタ (115200 baud)
```

## 参考リポジトリ

- [micro-ROS PlatformIO](https://github.com/micro-ROS/micro_ros_platformio)
- [yolo_ros](https://github.com/mgonzs13/yolo_ros)
- [Ros2-Tracking-Node](https://github.com/Team-Robo/Ros2-Tracking-Node)

---

*Plan: [.omo/plans/meister-autonomous-robot.md](.omo/plans/meister-autonomous-robot.md)*
