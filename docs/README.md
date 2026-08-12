# Meister ロボット制御ソフトウェア ドキュメント

> 対象リポジトリ: `sosukigara/2026-Meister`
> 作成日: 2026-08-12
> 言語: 日本語のみ

## ドキュメント構成

docs は「**やりたいこと（ゴール）**」「**何を作るか**」「**何ができるか**」「**どう作るか**」の4層で管理する。各機能は features / functions / design の3ファイルが相互リンクで対応し、上位のゴール（[やりたいこと.md](やりたいこと.md)）に紐づく。

| フォルダ | 内容 | 対象 |
|---|---|---|
| [やりたいこと.md](やりたいこと.md) | **ゴール** — 最終的に実現したいこと（G1: 自動走行 / G2: アーム回収 / G3: 手動操縦） | 方向性確認 |
| [features/](features/) | **何を作るか** — 機能の要件（概要・実装済み/計画中機能） | 要件確認・受け入れテスト |
| [functions/](functions/) | **何ができるか** — 機能分解（コンポーネントが実際にすること） | 機能理解・実装確認 |
| [design/](design/) | **どう作るか** — 設計メモと設計図化対象（設計図へ深化予定） | 実装・設計レビュー |

## 概要

ROS 2 (Jazzy) + Gazebo をベースとした自律移動ロボット制御ソフトウェア。
**シミュレーション（開発用）と実機（計画中）は同じロボットの異なる段階**であり、シミュレーションは実機開発の前段として機能する。

- **シミュレーション段階（実装済み）**: SLAM による地図作成、Nav2 による自律ナビゲーション、Web UI による遠隔操作（features/01-06 参照）
- **実機（計画中）**: 6輪ロッカーボギー + ESP32 + 4軸アーム + LD-D500 LiDAR。画像認識による自動把持と自動制御を目指す（features/07-10 参照）

マイコン層（ESP32 ファームウェア）は今後追加予定。「モータ制御」はシミュレーションでは Gazebo プラグイン、実機では ESP32 が担う。

## 機能一覧

| # | 機能 | ゴール | 要件 | 機能分解 | 設計 | ステータス |
|---|---|---|---|---|---|---|
| 01 | 起動・実行環境 | [G1](やりたいこと.md#g1-ロッカーボギーロボット自動走行) [G3](やりたいこと.md#g3-手動操縦も可能) | [features/01-startup.md](features/01-startup.md) | [functions/01-startup.md](functions/01-startup.md) | [design/01-startup.md](design/01-startup.md) | 実装済み |
| 02 | ロボットモデル・シミュレーション | [G1](やりたいこと.md#g1-ロッカーボギーロボット自動走行) | [features/02-robot-model.md](features/02-robot-model.md) | [functions/02-robot-model.md](functions/02-robot-model.md) | [design/02-robot-model.md](design/02-robot-model.md) | 実装済み |
| 03 | SLAM・地図作成 | [G1](やりたいこと.md#g1-ロッカーボギーロボット自動走行) | [features/03-slam.md](features/03-slam.md) | [functions/03-slam.md](functions/03-slam.md) | [design/03-slam.md](design/03-slam.md) | 実装済み |
| 04 | ナビゲーション | [G1](やりたいこと.md#g1-ロッカーボギーロボット自動走行) [G3](やりたいこと.md#g3-手動操縦も可能) | [features/04-navigation.md](features/04-navigation.md) | [functions/04-navigation.md](functions/04-navigation.md) | [design/04-navigation.md](design/04-navigation.md) | 実装済み |
| 05 | Web UI | [G1](やりたいこと.md#g1-ロッカーボギーロボット自動走行) [G2](やりたいこと.md#g2-自動でアームでものを回収) | [features/05-web-ui.md](features/05-web-ui.md) | [functions/05-web-ui.md](functions/05-web-ui.md) | [design/05-web-ui.md](design/05-web-ui.md) | 実装済み |
| 06 | 実機対応 | [G1](やりたいこと.md#g1-ロッカーボギーロボット自動走行) | [features/06-real-robot.md](features/06-real-robot.md) | [functions/06-real-robot.md](functions/06-real-robot.md) | [design/06-real-robot.md](design/06-real-robot.md) | 計画中 |
| 07 | ESP32 ↔ PC UART 通信 | [G1](やりたいこと.md#g1-ロッカーボギーロボット自動走行) [G2](やりたいこと.md#g2-自動でアームでものを回収) [G3](やりたいこと.md#g3-手動操縦も可能) | [features/07-esp32-uart.md](features/07-esp32-uart.md) | [functions/07-esp32-uart.md](functions/07-esp32-uart.md) | [design/07-esp32-uart.md](design/07-esp32-uart.md) | 計画中 |
| 08 | 6輪ロッカーボギー・ステアリング・モータ制御 | [G1](やりたいこと.md#g1-ロッカーボギーロボット自動走行) [G3](やりたいこと.md#g3-手動操縦も可能) | [features/08-rocker-bogie.md](features/08-rocker-bogie.md) | [functions/08-rocker-bogie.md](functions/08-rocker-bogie.md) | [design/08-rocker-bogie.md](design/08-rocker-bogie.md) | 計画中 |
| 09 | 4軸アーム + カメラ + 画像認識 + 自動把持 | [G2](やりたいこと.md#g2-自動でアームでものを回収) | [features/09-object-grasping.md](features/09-object-grasping.md) | [functions/09-object-grasping.md](functions/09-object-grasping.md) | [design/09-object-grasping.md](design/09-object-grasping.md) | 計画中 |
| 10 | 自動制御（自動移動 → 完全自律） | [G1](やりたいこと.md#g1-ロッカーボギーロボット自動走行) [G2](やりたいこと.md#g2-自動でアームでものを回収) | [features/10-auto-control.md](features/10-auto-control.md) | [functions/10-auto-control.md](functions/10-auto-control.md) | [design/10-auto-control.md](design/10-auto-control.md) | 計画中 |

> features/01-06 はシミュレーション段階の記録。実機のハードウェア構成は features/07-10 を参照。

## システム構成

```
Meister/
├── build.sh                         # ビルドスクリプト（4パッケージ並列）
├── start_meister.sh                 # 起動スクリプト（ビルドなし・起動のみ）
└── src/
    ├── meistar_description/         # ロボットモデル・シミュレーション定義
    ├── meister_vision/              # 画像認識（onnxruntime YOLO）
    ├── ros2_autonomous_nav/         # SLAM・ナビゲーションの中核パッケージ
    └── meister_web_nav/             # Web UI（:8088）
```

### 実機構成（計画中）

```
PC (ROS 2: Nav2 / 画像認識 / Web UI)
  │  UART 双方向通信
ESP32 (6輪モータ PWM / ステアリングサーボ / アームサーボ)
  │
6輪ロッカーボギー + 4軸アーム + カメラ + LD-D500 LiDAR
```

※ `ros2_autonomous_nav` 内の動画編集・ナレーション生成スクリプト群（YouTube 用）はロボット制御と無関係のため仕様対象外。

## ロードマップ

1. **機能リスト**（features/ ・現在地）— 各機能の「できること」を列挙
2. **詳細仕様** — 要件 ID 付与（例: `REQ-NAV-001`）、確認方法・数値条件の明記
3. **設計図**（design/）— モジュール構造・データフロー・状態遷移
4. **実装** — 機能ごとに AI と相談しながら実装を進める

### 実装優先順位（実機）

1. **自動アーム**（最優先）— 4軸アーム + 画像認識 + 自動把持（09）
2. **自動移動** — LD-D500 LiDAR + 既存 SLAM/Nav2 の実機移植（10）
3. **完全自律**（理想）— 自動巡回 + 障害物回避 + 物体回収の統合

機能ごとのファイルが独立しているため、1機能ずつ AI と相談しながら深化させることができる。
