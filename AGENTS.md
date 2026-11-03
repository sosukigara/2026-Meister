# AGENTS.md — Meister 開発ルール（インデックス）

> 対象リポジトリ: `sosukigara/2026-Meister`
> 全ルールは 2026-09-27 の開発で実際に踏んだミスを根拠にしている。
> **このファイルは最重要インデックス（200〜300 行以内）。詳細は `docs/` を参照。**
> コードマップ: [docs/code-map.md](docs/code-map.md)。ドキュメント全体の入口: [docs/README.md](docs/README.md)。

---

## 1. プロジェクトの目的・役割

ROS 2 (Jazzy) + Gazebo をベースとした自律移動ロボット制御ソフトウェア。
シミュレーション（開発用）と実機（計画中）は同じロボットの異なる段階であり、
シミュレーションは実機開発の前段として機能する。

- シミュレーション段階（実装済み）: SLAM による地図作成、Nav2 による自律ナビゲーション、
  Web UI による遠隔操作（features/01-06）
- 実機（計画中）: 6輪ロッカーボギー + ESP32 + 4軸アーム + LD-D500 LiDAR。
  画像認識による自動把持と自動制御を目指す（features/07-10）

マイコン層（ESP32 ファームウェア）は `firmware/main/`（PlatformIO + Arduino）。
`firmware/foc/` は別プロジェクト（MKS ESP32 FOC V2.0 ウィンチ）。

ゴール: G1 自動走行 / G2 アーム回収 / G3 手動操縦（[docs/やりたいこと.md](docs/やりたいこと.md)）。

## 2. 技術スタック

| 層 | 技術 |
|---|---|
| OS (PC) | Ubuntu |
| ミドルウェア | ROS 2 Jazzy |
| シミュレータ | Gazebo (ros_gz) |
| ビルド (ROS) | colcon |
| ビルド (FW) | PlatformIO |
| 言語 | Python 3 / C++17 / YAML |
| 検出 | ONNX Runtime + YOLOv8n |
| FW フレームワーク | Arduino (arduino-esp32 2.x/3.x 両対応) |
| テスト (C++) | Unity (`pio test -e native`) |
| テスト (Python) | pytest |

詳細は [docs/coding-standards.md](docs/coding-standards.md) §1-2。

## 3. 検証・コマンド集（要約）

pio 系は `firmware/main/` を cwd、ros2 系はリポジトリルートを cwd にする（R6）。

```bash
# ビルド
./build.sh
cd firmware/main && pio run -e esp32dev -e esp32dev_usbuart
# テスト
cd firmware/main && pio test -e native
python3 -m pytest src/meister_serial_bridge/test/
# 実機検証
ros2 run meister_serial_bridge meister_comm_check --port /dev/ttyUSB0
ros2 run meister_serial_bridge meister_hz_measure --port /dev/ttyUSB0 --expect-hz 100
```

実機 `comm_check` が 3 手順 PASS するまで commit しない。4 通りのビルドを回す。
完全版は [docs/commands.md](docs/commands.md)。

## 4. 絶対厳守ルール（NG リスト）

1. **1 ファイル 1 責務、300 行超で分割検討**（R1）→ [coding-standards §3](docs/coding-standards.md)
2. **設定値は 1 箇所以外で定義しない**（R2）→ [coding-standards §4](docs/coding-standards.md)
3. **物理定数は出典つき表でコード内に置く**（R3）→ [coding-standards §5](docs/coding-standards.md)
4. **未取得の値は記憶で埋めず隔離しゲートする**（R4）→ [pitfalls P1](docs/pitfalls.md)
5. **編集 reject 後は適用済みを file:line で確認**（R5）→ [pitfalls P2](docs/pitfalls.md)
6. **push 前にレビューを 1 巡**（R6）→ [pitfalls P3](docs/pitfalls.md)
7. **破壊的変更は 1 箇所の設定に集約してから入れる**（R7）→ [pitfalls P4](docs/pitfalls.md)
8. **レイヤー間依存は片方向**（R8）→ [architecture §2](docs/architecture.md) / [coding-standards §8](docs/coding-standards.md)
9. **コメントは WHY だけ**（R9）→ [coding-standards §7](docs/coding-standards.md)
10. **未検証を「問題なし」で終わらせない**（R10）→ [pitfalls P6](docs/pitfalls.md)
11. **静的初期化順を当てにしない**（R11）→ [pitfalls P5](docs/pitfalls.md)
12. **コミットはレビュー可能な単位に分ける**（R12）→ [pitfalls P7](docs/pitfalls.md)

追加の NG:
- `Any` 型の濫用を避ける（型ヒントを付ける）
- `hal/` は上位を知らない。上位から `hal/*` の内部を呼ばない
- `src/` は `MSTE_*` マクロを直接参照しない
- プロトコル軸数変更時は C++ と Python を同時に直す
- 既に動いているものを壊してまで先に進めるのは禁止

## 5. Docs へのポインタ

| ファイル | 内容 |
|---|---|
| [docs/architecture.md](docs/architecture.md) | 全体設計・モジュール責務・データフロー・設計パターン |
| [docs/coding-standards.md](docs/coding-standards.md) | 命名・禁止事項・ライブラリ・コメント・テスト配置 |
| [docs/commands.md](docs/commands.md) | 実行・テスト・検証コマンド |
| [docs/pitfalls.md](docs/pitfalls.md) | 過去に AI が誤爆した罠・プロジェクト固有の制約 |
| [docs/task-template.md](docs/task-template.md) | 仕様・タスク定義テンプレート |
| [docs/code-map.md](docs/code-map.md) | ファイル単位のコードマップ |
| [docs/README.md](docs/README.md) | ドキュメント全体の入口（4 層） |

## 6. コードを読んでもわからない暗黙知（抜粋）

- プロトコル: `protocol.py` (Python) と `meister_protocol.h` (C++) は**バイト単位で一致**。軸数変更時は両方直す。
- 通信周期: PC→ESP32 は到着時、ESP32 適用 100 Hz、FB_STATE 100 Hz、STS 読み戻しはラウンドロビン 30〜40 ms。
- 未確認値: `MSTE_BUS_RESP_TIMEOUT_US=3000µs`、`MSTE_BUS_INTER_FRAME_GAP_US=200µs`、`reg::*` アドレス、`kStepsPerRev=4096` はデータシート未照合。推測で埋めない。
- web search は全プロバイダ遮断。pip は使える。
- GPIO12 はストラップピン（グリッパー PWM）。実機配線時に変更推奨。
- `upload_port` は by-id で明示指定（M5Stack 誤書き込み防止）。
- `start_meister.sh` の `ROS_LOCALHOST_ONLY=1` は子プロセスのみ。

## 7. レビュー判定の語彙（R10）

- **一次情報で確定**（出典ファイル名と行が言える）
- **実測した**（測定条件と数値がある）
- **要確認**（未取得。`★要確認★` と隔離先を書く）
- **✗ 未検証**（まだ見ていない。"問題なし" と書かない）
