# CLAUDE.md — Meister

> Claude Code 用エントリーポイント。詳細は [AGENTS.md](AGENTS.md) と [docs/](docs/) を参照。

## プロジェクト

ROS 2 (Jazzy) + Gazebo ベースの自律移動ロボット制御ソフトウェア。シミュレーションと実機は同じロボットの異なる段階。
マイコン層は `firmware/main/`（ESP32、PlatformIO + Arduino）。`firmware/foc/` は別プロジェクト。

## 技術スタック

Ubuntu / ROS 2 Jazzy / Gazebo (ros_gz) / colcon / PlatformIO / Python 3 / C++17 / ONNX Runtime (YOLOv8n) / Arduino (arduino-esp32 2.x/3.x)。

## 検証コマンド（cwd に注意）

- pio 系は `firmware/main/` を cwd にする。
- ros2 系はリポジトリルートを cwd にする。

```bash
./build.sh
cd firmware/main && pio run -e esp32dev -e esp32dev_usbuart
cd firmware/main && pio test -e native
python3 -m pytest src/meister_serial_bridge/test/
ros2 run meister_serial_bridge meister_comm_check --port /dev/ttyUSB0
```

## 絶対厳守（NG）

1. 1 ファイル 1 責務、300 行超で分割
2. 設定値は `meister_config.h` の 1 箇所のみ
3. 物理定数は出典つき表でコード内に
4. 未取得値は記憶で埋めず隔離・ゲート
5. 編集 reject 後は file:line で確認
6. push 前にレビュー 1 巡 + 4 ビルド + comm_check 3 手順
7. 破壊的変更は 1 箇所の設定に集約
8. レイヤー依存は片方向（hal は上位を知らない）
9. コメントは WHY だけ
10. 未検証を「問題なし」で終わらせない
11. 静的初期化順を当てにしない
12. コミットはレビュー可能な単位に

追加: `hal/` は上位を知らない / `src/` は `MSTE_*` を直接参照しない / プロトコル軸数変更時は C++ と Python を同時に直す / 既に動いているものを壊してまで先に進めるのは禁止。

## Docs ポインタ

- [AGENTS.md](AGENTS.md) — 開発ルール（詳細）
- [docs/architecture.md](docs/architecture.md) — 全体設計
- [docs/coding-standards.md](docs/coding-standards.md) — コーディング規約
- [docs/commands.md](docs/commands.md) — コマンド
- [docs/pitfalls.md](docs/pitfalls.md) — 落とし穴
- [docs/task-template.md](docs/task-template.md) — タスクテンプレート
- [docs/code-map.md](docs/code-map.md) — コードマップ
