# umeonigiri_serial_bridge リファクタリング設計

> 日付: 2026-11-20 / 状態: ユーザー承認済み（方式・スコープ・実機ゲートは後日）
> 対象: `ros/src/meister_serial_bridge`（改称後 `ros/src/umeonigiri_serial_bridge`）
> 計画書: [../plans/2026-11-20-umeonigiri-serial-bridge-refactor.md](../plans/2026-11-20-umeonigiri-serial-bridge-refactor.md)

## 1. 背景と目的

`meister_serial_bridge` は PC と ESP32 の唯一の接点で、`/cmd_vel` を UART フレームへ変換し、
FB_STATE / FB_IMU を ROS へ戻す。2026-10-05 の Python → C++（ament_cmake）移行後、
シリアル I/O 層と実機検証 CLI に、テストでは見えない欠陥が残っている。

本作業の目的は次の 3 つ。

1. パッケージ名を `umeonigiri_serial_bridge` に統一する（ノード名は既にこの名前）。
2. 一次情報またはホスト実測で確定した不具合を直す。特に「USB 抜けで落ちる」「受信が
   1 バイトずつ」「実機の hz_measure が常に NG」を解消する。
3. 実機ゲートを後日通せる形で、検証手順と未検証項目を明示する。

## 2. 調査で確定した事実

| # | 事実 | 出典 / 証拠の種類 |
|---|---|---|
| F1 | `LibSerialIo::Read` は `Read(dbuf, 1, timeout)` の後に `Read(dbuf, 0, 0)` を呼ぶ。libserial の `Read(numberOfBytes=0, msTimeout=0)` は**即 return して 1 バイトも追加しない**。結果、実動作は最大 1 バイト/呼び出し。`timeout_ms == 0` は常に 0 バイト | libserial 1.0.0-9build1 の pty 実測（下記 §8）と `/usr/include/libserial/SerialPort.h:307-325` |
| F2 | ハングアップ（USB 抜け相当）で `Read` は `std::runtime_error`、`Write`/`FlushOutputBuffer` は I/O error を投げる。`RxLoop` と購読コールバックは無捕捉なので `std::terminate` | pty 実測。コードは `serial_bridge_node.cpp:224-262`, `:88-101` |
| F3 | libserial 1.0.0 の `~SerialPort()` は `noexcept` のまま例外を投げ、ハングアップ後に破棄すると terminate する。したがって「壊れたリンクを reset して再オープン」は単純には実装できない | `SerialPort.h:87`（`virtual ~SerialPort() noexcept;`）と pty 実測 |
| F4 | `BridgeCore::DropLink()` は定義のみで呼び出し元が無い。再オープン経路が無い | `bridge_core.cpp:44-47` とリポジトリ全体 grep |
| F5 | `Measurement::LostFrames()` は `nbytes / kMaxFrameSize(17)` を期待フレーム数にする。firmware は FB_IMU（6 バイト）を 20 Hz で送る（IMU 既定有効）ため、実機では毎秒約 7 フレームの偽の欠落になる | `feedback_hz_measure.hpp:25-30`、`firmware/main_board/include/meister_config.h:95-101`、`src/imu_reporter.cpp:32-58` |
| F6 | `RunCommCheck` の手順 3 は意図的に `error_flags` を立てる。firmware の `rxErrorCount_` は再起動まで減らない。成功後もフラグが残り、直後の `hz_measure` が protocol error で NG になる | `comm_check.cpp:167-186`、`firmware/main_board/src/command_dispatch.cpp:63,99-101` |
| F7 | `EncodeArm` は `kMinJoint/kMaxJoint` で丸めるが、CMD_ARM_ANGLE の実経路は `Arm::setAngles` → `PwmServoChannel` の `kMinArmAngle/kMaxArmAngle`。現在値は同じ 0..1800 なので挙動は偶然一致 | `frames.cpp:200-207`、`firmware/main_board/src/arm.cpp:40-44`、`firmware/main_board/include/arm.h:20-24` |
| F8 | 未対応ボーレートは `BAUD_DEFAULT`（= `BAUD_115200`）へ無言でフォールバックする | `serial_link.cpp:18-29`、`/usr/include/libserial/SerialPortConstants.h:208,228` |
| F9 | `ros2_autonomous_nav/package.xml` は launch で起動する当パッケージへの `exec_depend` を持たない。ノード名は launch 経路で `serial_bridge` と `umeonigiri_serial_bridge` に分裂 | `real_robot.launch.py:70,120-122`、`serial_bridge.launch.py:22` |
| F10 | `serial_bridge_node.cpp` は 327 行で R1 の 300 行目安を超える | `wc -l` |

## 3. ゴール / 非ゴール

### ゴール

- パッケージ名・ディレクトリ・C++ namespace・include パス・実行ファイル名・launch 名を
  `umeonigiri_serial_bridge` / `umeonigiri_*` に揃える。
- F1〜F9 を修正し、ホストで再現できるものには回帰テストを付ける。
- 実機でしか確認できない項目を `✗ 未検証` として計画に隔離する。

### 非ゴール

- firmware（`firmware/main_board/`）の変更。プロトコルのバイト列・軸数・値域は変えない。
- libserial の termios 置換。今回は「壊れた `SerialPort` を破棄しない」隔離で回避し、
  置換は別計画とする（F3 の根治は termios 移行時に判断）。
- 本設計時点での `serial_bridge_node.cpp` 分割。R1 は「超えたら分割を検討」であり、
  当時は別計画へ送った。2026-11-20 の後続タスクで実施済み（§6.9）。
- 日付付きの過去仕様書（`docs/superpowers/specs/2026-11-18-*.md`）の書き換え。
  これらは当時の判断の記録であり、現行仕様ではない。

## 4. 制約

- AGENTS.md R13: 作業・ビルド・テスト・commit は `.worktrees/serial-bridge-refactor`
  のみ。main チェックアウトは触らない。push は main で merge 後、実機ゲート通過後。
- R12: 純粋リネームと挙動変更を同じ commit に混ぜない。
- R7: プロトコル変更を伴う破壊的変更は無い（名前のみ）。軸数を変えない。
- R10: 実機未実施の項目を「問題なし」と書かない。`✗ 未検証` を使う。
- AGENTS §3 の「実機 `comm_check` が 3 手順通るまで commit しない」は、
  **main への着地（merge / push）を指す**と解釈する。worktree 内の commit は
  レビュー単位として行い、merge は Task 10 通過後まで行わない。この解釈は
  ユーザーに明示し、異論があれば作業前に止める。
- 依存追加なし。libserial はシステムパッケージのまま（rosdep キーが無い）。

## 5. 検討した方式

| 方式 | 内容 | 評価 |
|---|---|---|
| A. libserial 最小修正 + リネーム（採用） | `Read` を一括読みへ修正、I/O 例外を捕捉、壊れたリンクをプロセス寿命まで隔離、pty テスト追加 | 実機リスクが小さく、F1/F2/F3 をホストで再現・検証できる。隔離リークは既知の負債として明示 |
| B. termios 実装へ置換 | libserial を外し POSIX termios で実装。F3 を根治し依存も削減 | 差分が大きく、DTR/RTS・ボーレート・行制御の実機再検証が必須。実機が後日のため今回は不採用 |
| C. リネームのみ | 名前だけ揃える | USB 抜けで落ちる問題が残り、目的 2 を満たさない |

## 6. 設計判断

### 6.1 リネーム範囲（ユーザー選択: 全部）

| 種別 | 旧 | 新 |
|---|---|---|
| ディレクトリ | `ros/src/meister_serial_bridge` | `ros/src/umeonigiri_serial_bridge` |
| include | `include/meister_serial_bridge/` | `include/umeonigiri_serial_bridge/` |
| namespace | `meister_serial_bridge` | `umeonigiri_serial_bridge` |
| CMake project | `meister_serial_bridge` | `umeonigiri_serial_bridge` |
| ライブラリ | `meister_serial_bridge_core` | `umeonigiri_serial_bridge_core` |
| 実行ファイル | `serial_bridge` | `umeonigiri_serial_bridge` |
| 実行ファイル | `meister_comm_check` | `umeonigiri_comm_check` |
| 実行ファイル | `meister_hz_measure` | `umeonigiri_hz_measure` |
| launch | `serial_bridge.launch.py` | `umeonigiri_serial_bridge.launch.py` |
| ノード名 | `umeonigiri_serial_bridge`（変更なし） | 同左 |

ソースファイル名（`serial_bridge_node.cpp` 等）とクラス名（`SerialBridgeNode`）は
役割名として維持する。インストールされる識別子だけを揃える。

### 6.2 `Read` のアルゴリズム（F1）

1. `timeout_ms > 0` のときだけ `Read(dbuf, 1, timeout_ms)` で 1 バイト到着を待つ
   （低遅延を維持）。`ReadTimeout` は「データ無し」で 0 バイト返し。
2. `GetNumberOfBytesAvailable()` で到着済みバイト数を取り、`max` までを
   `Read(dbuf, want, 1)` で一括取得する。timeout を 0 にしないのは、libserial が
   「要求バイト数が揃うまでブロック」するため（競合時の無限待ち防止）。
3. `timeout_ms == 0` は待たず、到着済みの分だけ返す。無ければ即 0。

pty 実測で「到着済み 6 バイト → 1 + 5 バイトを 0 ms で取得」を確認済み。

### 6.3 I/O 例外とリンク隔離（F2/F3/F4）

- `LibSerialIo::Read` は `ReadTimeout` だけを握りつぶし、それ以外の例外は伝播させる。
- ROS 層（`OnCmdVel` / `OnArmJoint` / `OnGripper` / `RxLoop` / `SendStopFrames`）は
  `std::exception` を捕捉し、throttle 付きログ + `BridgeCore::DropLink()` を行う。
- `DropLink()` はリンクを `reset()` せず、**プロセス寿命の隔離リスト**へ移す。
  libserial の `~SerialPort()` が noexcept のまま投げるため、破棄すると terminate する。
  隔離リストは意図的にリークし、`libserial` 由来の負債としてコメントに残す
  （USB 抜け回数で有界。termios 移行時に削除）。
- 再オープンは既存の `CurrentLink()` の再試行間隔（既定 2 秒）に従う。
  失敗が続く間はログのみで、プロセスは生存する。

### 6.4 欠落フレーム計測（F5）

`Measurement` に `decoded_bytes`（全種別の `frame.frameSize` 合計）と
`buffered_bytes`（受信末尾の未完成フレーム）を追加し、

```
LostFrames() = max(0, nbytes - decoded_bytes - buffered_bytes) / kMaxFrameSize
```

とする。FB_IMU が混ざっても 0 になり、壊れた 17 バイトフレームは 1 として検出できる。
`kMaxFrameSize` で割るのは FB_STATE が最大長である前提の近似（ヘッダに明記）。

### 6.5 `comm_check` の後始末と受信 flush（F6）

- 手順 2 の直前に `ResetInputBuffer()` + `parser.Clear()` を入れ、指令前の
  FB_STATE を応答と誤認しない。
- 手順 3 の後、`reset` が有効なら `ResetEsp32()` をもう一度実行して
  `error_flags` を消す（`★要確認★` 実機の自動リセット配線に依存。Task 10 で確認する）。`--no-reset` のときは `CheckResult::note` に
  「再起動するまでフラグが残る」と明記し、`main` が表示する。
- 手順数は 3 のまま（AGENTS の実機ゲート記述を変えない）。

### 6.6 アームクランプの出典（F7）

`EncodeArm` の値域を `meister::config::kMinArmAngle/kMaxArmAngle` に変更する。
これは `Arm::setAngles` → `PwmServoChannel::setAngleTenths` が実際に使う表である。
`kMinJoint/kMaxJoint` は `ClampArm`（別経路）の表であり、現時点で値は同じだが
出典が違う。テストのコメントも実経路に合わせる。

### 6.7 パラメータ検証（F9 の一部）

ノード生成時に `max_linear_vel` / `max_steering_deg` / `cmd_timeout` / `baud` の
正値性を検証し、不正なら `std::invalid_argument`。`main` が捕捉して
`RCLCPP_FATAL` + 終了コード 1 を返す。購読コールバックの捕捉（§6.3）は防御として残す。

### 6.8 ノード名と依存（F9）

- `constexpr char kNodeName[] = "umeonigiri_serial_bridge";` を 1 箇所に置き、
  `Node(kNodeName)` と logger の両方で使う（R2）。
- `real_robot.launch.py` の `name=` を `umeonigiri_serial_bridge` に統一。
- `ros2_autonomous_nav/package.xml` に `<exec_depend>umeonigiri_serial_bridge</exec_depend>`。
- 当パッケージに `<exec_depend>launch</exec_depend>` と
  `<exec_depend>launch_ros</exec_depend>` を追加。

### 6.9 R1 の扱い（F10）

`serial_bridge_node.cpp` は 327 行で、例外処理・検証を加えると 395 行になった。
2026-11-20 の後続タスクで、rclcpp 依存の出版を `OdometryPublisher`（`/odom` と TF）と
`FeedbackPublisher`（`/esp32/state` と `/imu/data`）へ分離した。ノードは **284 行**になり、
`BridgeCore` と CLI/gtest は ROS 非依存のまま。トピック名・QoS・frame_id・共分散・
50 Hz の出版レートは変えていない。

## 7. 受入基準

- ホスト: `colcon build` と `colcon test` が成功し、既存 55 件 + 追加テストが全て PASS。
- pty テストが「17 バイト一括返却」「未対応 baud の明示エラー」を固定する。
- `LostFrames()` が FB_STATE 10 + FB_IMU 5 の混在で 0 になる。
- `comm_check` が `reset=false` で `note` を返し、`reset=true` で RTS を 2 回パルスする。
- `DropLink()` がリンクを破棄せず、2 回目の `OnCmdVel` が新しいリンクへ書く。
- 不正パラメータでノードが終了コード 1（実機不要）。
- `ros2 launch ... --show-args` が新名で通る。
- 実機 3 手順 + hz_measure + 抜き差しは `✗ 未検証`（§8）。

## 8. 未検証・要確認

- `✗ 未検証` 実機の `comm_check` 3 手順、`hz_measure --expect-hz 100`、走行中の USB 抜き差し。
  実機が用意でき次第、計画書 Task 10 のコマンドで確認する。
- `★要確認★` DTR/RTS の極性が実機の自動リセット回路（`comm_check.cpp:31-41` の想定）と
  一致すること。現行実装の前提を変えていないが、今回の変更で実機確認は増えていない。
- `★要確認★` libserial の隔離リークが長時間運用で問題にならないこと（USB 抜け回数に比例）。
  termios 移行の判断材料にする。
- ホスト実測の条件: Ubuntu / libserial 1.0.0-9build1 / pty (`posix_openpt`)。
  実 UART ドライバ（CH340/CP210x）での `GetNumberOfBytesAvailable()` の挙動は
  `✗ 未検証`。実機ゲートで `hz_measure` の欠落 0 を確認する。
