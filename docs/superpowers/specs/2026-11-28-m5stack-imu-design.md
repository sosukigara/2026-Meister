# M5Stack Tab5 IMU ファームウェア 設計（短い設計）

## 意図 / なぜ

M5Stack Tab5 の内蔵 BMI270 6軸 IMU を読み、PC へ確実に送り、同時に Tab5 自身の 5 インチ画面で状態を可視化したい。既存の `main_board` ファームウェアは BNO055 の yaw のみを `FB_IMU` で送るため、6軸データを扱う新規ファームウェアを `firmware/` に追加する。GUI は **Tab5 本体画面** に表示する（PC 側 GUI は本次の範囲外、利用者が選択済み）。

## ゴール / 非ゴール / アンチゴール

- ゴール
  - Tab5 の加速度 3 軸・角速度 3 軸を M5Unified 経由で周期取得する。
  - 6軸を USB CDC シリアルで PC へ 20 Hz 送信する。
  - Tab5 画面に 6軸の数値・棒グラフ・roll/pitch・送信レート・TX 状態を表示する。
  - 送信フレームのエンコード/デコードを native 単体テストで証明する。
  - ESP32-P4 向けビルドが通ることを証明する。
- 非ゴール
  - PC 側の受信アプリ / GUI / ROS ノードの新規作成。
  - `main_board` のプロトコル・設定・動作変更。
  - 姿勢推定（ complementary filter 等）の高度な実装。表示用の簡易 roll/pitch のみ。
- アンチゴール
  - 未接続の Tab5 を想定して実機動作を「確認済み」と書くこと。
  - 推測した物理値・レジスタ値を実測値として扱うこと（R4）。

## 制約

- AGENTS.md R1〜R13 を守る。特に R2（設定は 1 箇所）、R3（物理定数は出典付きテーブル）、R8（層の依存は片方向）、R13（作業は worktree、main は触らない）。
- 新規プロジェクト名は `firmware/m5stack_imu/`（既存ディレクトリはすべて小文字 snake_case）。
- ボードは PlatformIO `m5stack-tab5-p4`（`pio boards` で確認済み）。Arduino フレームワーク。
- ライブラリは M5Unified >= 0.2.23 / M5GFX >= 0.2.30（Tab5 の画面リセット処理に必要、[M5Stack 公式](https://docs.m5stack.com/en/core/Tab5)）。
- データの単位は accel = G、gyro = deg/s（[M5Unified IMU ドキュメント](https://context7.com/m5stack/m5unified/llms.txt)、[M5Unified ソースの変換パラメータ](https://github.com/m5stack/M5Unified/blob/master/src/utility/IMU_Class.cpp)）。
- 送信は USB CDC（Arduino `Serial`、115200 baud）。実機のポート名は環境依存のため `★要確認★`（過去の環境では `/dev/ttyACM0`、AGENTS.md の注意書きに記載）。
- 現在の開発 PC には Tab5 が接続されていない（`ls /dev/serial/by-id` で main_board の `/dev/ttyUSB0` のみ）。実機確認は接続後。
- 画面ドライバは個体で異なる（ILI9881C+GT911 / ST7123 / ST7121）。M5Unified + M5GFX の新しめのバージョンに任せ、ファームウェア側でドライバ選択はしない。

## アプローチ（検討した 2 案）

1. **独自バイタリフレーム（採用）** — 既存プロトコルの `0xA5 + type + XOR` を踏襲し、type `0x84` で 6軸を int16 固定小数点＋タイムスタンプで送る。誤フレーム検出が強く、`payload` の単体テストが書きやすい。PC 側に軽いパーサが必要だが、本次は PC 側を作らないため、README にフレーム仕様を記載して後から実装できるようにする。
2. **ASCII 行送信** — `IMU,t,ax,ay,az,gx,gy,gz` を改行区切りで送る。シリアルモニタで即読める利点はあるが、サイズが大きく、欠損行の扱いと数値精度の規則が曖昧になる。採用しない。

利用者が A を選択（2026-11-28）。

## 決めた作り方

- `firmware/m5stack_imu/` に PlatformIO プロジェクトを新規作成する。`include/` にプロトコルと設定、`src/` に IMU 取得・GUI 描画・送信の 3 つの小さなモジュールを置く。
- フレーム形式（20 バイト）: `[0xA5][0x84][timestamp_ms u32 LE][ax,ay,az,gx,gy,gz: int16 LE][status u8][xor]`
  - スケールは accel 1 G = 1000、gyro 1 deg/s = 100（プロトコル表現の選択であり実測値ではない）。範囲外はクランプし、クランプ発生を status のビットに立てる。
  - XOR はフレーム全バイト（チェックサム自身を含む）が 0 になる既存方式を使う。
- 周期は 20 Hz 既定。設定値は `include/imu_config.h` に 1 箇所だけ置く（R2）。
- GUI は M5GFX の `M5.Display` に直接描画する。毎回全画面クリアせず、値エリアだけを上書きしてちらつきを抑える。
- 送信は `Serial.write()` でバイナリ。ログ用にシリアルモニタへテキストを混ぜない（フレーム汚染を避ける）。起動時のみログを 1 行出す。
- テストは `pio test -e native` でエンコード、デコード、チェックサム不良、クランプ、境界値を検証する。ESP32-P4 実機は `★要確認★`。

## 残された事項（ビルダー判断）

- ファイル名・関数名・テスト名は実装時につけてよい。ただし分離する責任（protocol / imu_sampler / display_view）は変えない。
- 画面レイアウトの細かい座標・色は実装時に決めてよい。6軸・roll/pitch・Hz・TX 状態が同時に見えることは満たす。
- 実機未接続のため、ボードへの書き込みと表示確認は利用者が Tab5 を接続した後に行う。README に手順を書く。
