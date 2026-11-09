# コーディング規約と制約 — Meister

> 一般的な構文・フォーマットは Lint/Formatter に任せ、ここには**リポジトリ固有の作法**だけを書く。設計・レイヤーは [docs/architecture.md](architecture.md)、罠は [docs/pitfalls.md](pitfalls.md)。

## 1. 技術スタック（バージョンを混同させない）

- OS: Ubuntu（PC 側）/ ESP32-WROOM-32
- ROS: ROS 2 Jazzy
- シミュレータ: Gazebo (ros_gz)
- ビルド: colcon（ROS パッケージ）、PlatformIO（ファームウェア）
- 言語: Python 3（ROS ノード）、C++17（ファームウェア）、YAML（設定）
- 検出: ONNX Runtime + YOLOv8n
- ファームウェア: Arduino フレームワーク（arduino-esp32 2.x/3.x 両対応）

## 2. 採用 / 禁止ライブラリ

| 用途 | 採用 | 禁止・注意 |
|---|---|---|
| シリアル通信 (PC) | pyserial | — |
| 物体検出 | onnxruntime | — |
| HTTP/WebSocket (UI) | 既存 `web_nav_server.py` | 新規 FW 追加は要相談 |
| ファームウェア PWM | LEDC（`hal/ledc_pwm`） | 直接 `ledcXxx` を呼ばない |
| テスト (C++) | Unity (`pio test`) | Arduino 依存コードは native テスト不可 |
| テスト (Python) | pytest | rclpy 依存は分離してテスト |

- 新しい依存を追加する前に、標準ライブラリ / 既存コードで代替できないか確認する。

## 3. ファイル・責務・サイズ (R1)

- **1 ファイル 1 責務**。300 行を超えたら分割を検討する。
- `main.cpp` は setup/loop と配線だけ。ロジックを置かない。
- ハードウェア依存は `hal/` に閉じ込める。
- ディレクトリ目安:
  ```
  firmware/main_board/include/hal/   ハードウェア抽象（Arduino 依存 OK）
  firmware/main_board/src/hal/       同上、実装
  firmware/main_board/include/*.h    機構（base_chassis / arm）と設定
  firmware/main_board/src/*.cpp      同上、実装
  firmware/main_board/src/main.cpp   setup / loop と配線だけ
  ```

## 4. 設定値は 1 箇所以外で定義しない (R2)

- `MSTE_*` の既定値は `firmware/main_board/include/meister_config.h` に集約。
- 優先順位は「コンパイル時 `-D` → `meister_config.h` のフォールバック」の **1 経路だけ**。
- `platformio.ini` の `build_flags` と `meister_config.h` の `#ifndef` フォールバックが両方にある状態で片方だけ直すと不整合が起きる。両方揃える。
- `ros/src/` は `MSTE_*` マクロを直接参照しない。

## 5. 物理定数（ピン / ID / レジスタ）と単位・範囲 (R3)

- データシートは 1 回だけ見て、規範はコードに残す。
- `firmware/main_board/include/hal/feetech_sts_registers.h` が例: フレーム構造・命令コード・チェックサム算出・上限サイズを**出典（SDK のファイル名と行）つき**で保有。
- 出典が分かるものだけ確定値として書き、出典が無いものは「要確認」節に隔離（R4）。
- ピン割当・値域は `meister_config.h` の表に単位・範囲を併記する。

## 6. 命名規則

- ファイル名: `snake_case`（ROS パッケージ・Python・C++ 共通）。ROS パッケージ名は `meister_*` / `meistar_*`。
- C++ クラス: `PascalCase`。関数/変数: `camelCase`。定数: `kPascalCase`（`config::k...`）または `MSTE_*` マクロ。
- Python: PEP 8（`snake_case`）。ROS ノードは `*_node.py`。
- テストファイル: `test_*.py` / `test_*/`（Unity）。
- 未確定値には `★要確認★` をコメントに残す。

## 7. コメントは WHY だけ (R9)

- 装飾的な区切り罫 `// =====` をファイル内に増やさない。
- 引数や戻り値を `///` で書き起こさない（シグネチャに既出）。
- 残すのは「この単位は何か」「なぜ固定長か」「0 を返すのは何の失敗か」。
- 「何をするか」のコメントは消して、コードを直す。
- 機械チェック（`.omo/session-work/check_header.sh`）: 文字化けスキャンは (1) 日本語の直後に英単語 3 文字以上が続く綴り、(2) ハングル、(3) U+FFFD の 3 つのみ。CJK 範囲で判定すると正常な日本語が全滅するので使わない。

## 8. レイヤー間依存は片方向 (R8)

- `main.cpp → command_dispatch → base_chassis / arm → hal/*`。`hal/` は上位を知らない。
- `hal/bus_protocol` は Arduino を include しない（host test 可能）。
- `meister_protocol` は Arduino 非依存。

## 9. エラーハンドリングの方針

- ESP32: フレームのチェックサム不一致・長さ不正は破棄（受信 FSM）。FB_STATE に `err_flag` を含める。
- PC: `comm_check` の 3 手順（uplink / downlink / err_flag）で検証。`feedback_hz_measure` で周期・欠落を実測。
- 未検証の項目は「✗ 未検証」と書く。「問題なし」とは書かない（R10）。
- 4 つの状態を区別する: **一次情報で確定**（出典ファイル名と行）/ **実測した**（条件と数値）/ **要確認**（未取得、`★要確認★`）/ **✗ 未検証**（まだ見ていない）。

## 10. テスト配置と実行

- C++: `firmware/main_board/test/test_protocol/`, `test/test_bus_protocol/`, `test/test_kinematics/`（Unity、`pio test -e native`）。
- Python: `ros/src/meister_serial_bridge/test/`（pytest）。
- プロトコル変更時は C++ と Python の両テストの期望値を更新する。

## 11. コミット単位 (R12)

1 コミットに 1000 行超の refactor + 新 HAL + プロトコル変更 + 文書を入れない。推奨:
1. **pure refactor**（挙動を 1 バイトも変えない。回帰証明）
2. **機能追加**（既存挙動に影響しないフラグで無効化されているもの）
3. **文書・ルール**

pure refactor のコミットでは、実機ゲートを通してから commit する。
