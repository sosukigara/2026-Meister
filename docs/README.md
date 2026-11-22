# Meister ロボット制御ソフトウェア ドキュメント

> 対象リポジトリ: `sosukigara/2026-Meister`
> 更新日: 2026-11-22（4 層構成を廃止し、**現状の開発に必要な情報だけ**に絞った）

もとは「やりたいこと → features → functions → design」の 4 層で管理していたが、
features / functions が同じ内容の写しになり、design の大半が未記入の TODO になったため
2026-11-22 に廃止した。いまの docs は**生きた仕様・実測・罠**だけで構成する。
削除した内容は git 履歴に残っている。

## 1. どこに何があるか

| 場所 | 内容 | 読むとき |
|---|---|---|
| [../AGENTS.md](../AGENTS.md) | 開発ルールの索引（R1〜R13）、検証コマンド、暗黙知 | 作業を始める前に必ず |
| [architecture.md](architecture.md) | 全体設計、レイヤと依存方向、モジュール責務 | 構成を変えるとき |
| [code-map.md](code-map.md) | ファイル単位の地図（どのファイルが何をするか） | 触るファイルを探すとき |
| [coding-standards.md](coding-standards.md) | 命名・禁止事項・ライブラリ・コメント・テスト配置 | コードを書くとき |
| [commands.md](commands.md) | ビルド・テスト・起動・実機検証・計測のコマンド | 実行するとき |
| [pitfalls.md](pitfalls.md) | 実際に踏んだ罠（P1〜P18）と未確認値 | 詰まったとき |
| [superpowers/specs/](superpowers/specs/) | 設計判断と実装仕様（プロトコル・ブリッジ・ナビ・Web UI）、未実装範囲 | 仕様を確認するとき |
| [superpowers/plans/](superpowers/plans/) | 実装計画と、その結果の実測 | 経緯を追うとき |
| [bench/](bench/) | ナビ・CPU・画像認識の実測値（条件つき） | 数字を根拠にするとき |
| [debug/](debug/) | 解決済み障害の記録（現象 → 切り分け → 原因） | 同種の症状が出たとき |

## 2. ゴール

| ID | ゴール | 担うもの |
|---|---|---|
| G1 | 6 輪ロッカーボギーが LiDAR と SLAM / Nav2 で自動走行する | `ros2_autonomous_nav` / `ros2_ldlidar_d500` / `umeonigiri_serial_bridge` / `firmware/foc_rover` |
| G2 | 4 軸アームと手首カメラで物体を自動で掴んで回収する | `umeonigiri_vision`（検出のみ）。アーム・把持は未実装（[計画](superpowers/specs/2026-11-22-real-machine-and-arm-plan.md)） |
| G3 | 手動操縦もできる | `umeonigiri_webnav` とブリッジ経由の指令経路 |

## 3. 現状（実装済み / 未実装）

**シミュレーション段階（実装済み・動作確認済み）** — SLAM による地図作成、Nav2 による
自律ナビゲーション、Web UI からの遠隔操作。地図モード（mapping / localization）は Web UI から切り替える。

**実機（一部のみ）**

- LD-D500 LiDAR、SLAM、Nav2、Web UI、CPU 側の画像認識は実機で動作確認済み
  （[実測と未検証](superpowers/specs/2026-11-22-real-machine-and-arm-plan.md) §1）
- ESP32 `main_board` と PC ブリッジのバイナリプロトコルは双方向通信を確認済み。**実配線・
  `esp32dev`（Serial2）での運用は未検証**
- FOC 基板（`foc_rover`）はベンチのオープンループ動作のみ。走行前提の閉ループは未達

**未実装** — アーム・逆運動学・把持、完全自律、6 輪での走行
（[計画](superpowers/specs/2026-11-22-real-machine-and-arm-plan.md) §2〜3）

## 4. システム構成

```
Meister/
├── build.sh                         # ビルドスクリプト（6 パッケージ並列）
├── launch.sh                        # 実機モード（既定） / --sim で Gazebo + SLAM + Nav2 + Web UI
├── launch_vision.sh                 # 画像認識だけ起動（PC カメラ）
├── kill.sh                          # domain 5 の ROS / Gazebo プロセスを終了
├── firmware/
│   ├── main_board/                  # ESP32（プロトコル・ステアリング・アーム）
│   ├── foc_rover/                   # FOC 走行用（3 軸 × 2ch）
│   └── foc_test/                    # FOC 回転テスト専用
└── ros/                             # ROS 2 ワークスペース（colcon はここで実行）
    ├── config/meister_robot.yaml    # ロボット設定の唯一の出所（tools/gen_config.py が読む）
    └── src/
        ├── umeonigiri_description/  # ロボットモデル・シミュレーション定義
        ├── ros2_autonomous_nav/     # SLAM・ナビゲーションの中核
        ├── ros2_ldlidar_d500/       # LD-D500 ドライバ（vendored, MIT）
        ├── umeonigiri_serial_bridge/ # PC ↔ ESP32 ブリッジ（プロトコルの PC 側）
        ├── umeonigiri_vision/       # 画像認識（YOLOv8n / RKNN）
        └── umeonigiri_webnav/       # Web UI（:8088）
```

実機の接続関係は [architecture.md](architecture.md) §1 の図を見る。

※ `ros2_autonomous_nav` 内の動画編集・ナレーション生成スクリプト群（YouTube 用）はロボット制御と無関係のため対象外。

## 5. ドキュメントを増やすとき

- **コードを読めば分かることは書かない**（関数の実装や全ファイル一覧は [code-map.md](code-map.md) が持つ）。
- **同じことを 2 箇所に書かない**。判断が要る内容は specs、数値は bench、罠は pitfalls、コマンドは commands のどれか 1 つに置く。
- **未確定は 4 段階で書く**（一次情報で確定 / 実測 / ★要確認★ / ✗ 未検証）。
- **レビューで 2 回同じ指摘をしたら追記する**だけにする。先回りして網羅しない。
