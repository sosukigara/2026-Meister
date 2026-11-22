# umeonigiri_serial_bridge リファクタリング実施計画

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `ros/src/meister_serial_bridge` を `umeonigiri_serial_bridge` に改称し、USB 抜けでの即死・1 バイト受信・実機 hz_measure の偽 NG を、ホストで回帰テストできる形で修正する。

**Architecture:** パッケージ名・namespace・実行ファイル名・launch 名を一括で揃える純粋リネームを最初のコミットに置く。挙動変更は `SerialIo` 実装（libserial 境界）に閉じ込め、ROS 層は例外を捕捉してリンクを隔離・再オープンする薄い層に留める。プロトコルのバイト列と firmware は変えない。

**Tech Stack:** ROS 2 Jazzy / C++17 / colcon / gtest / libserial 1.0.0 / cpp-httplib（変更なし） / pty（テストのみ）

**Spec:** [docs/superpowers/specs/2026-11-20-umeonigiri-serial-bridge-design.md](../specs/2026-11-20-umeonigiri-serial-bridge-design.md)

## Global Constraints

- 作業場所は `/home/so/Meistar/.worktrees/serial-bridge-refactor` のみ（R13）。main チェックアウトでは編集・ビルド・commit をしない。
- ブランチは `chore/serial-bridge-refactor`。worktree 内から `git push` しない。
- ROS コマンドは worktree の `ros/` を cwd にする（R6）。`source /opt/ros/jazzy/setup.bash` を先に行う。
- 純粋リネーム（Task 1）と挙動変更（Task 2〜7）を同じ commit に混ぜない（R12）。
- firmware（`firmware/main_board/`）、プロトコルのバイト列、軸数、値域、ピンを変えない（R7）。
- 新しい依存を追加しない。libserial はシステムパッケージのまま。
- 未実施の実機確認を「問題なし」と書かない。`✗ 未検証` / `★要確認★` を使う（R10）。
- 既存テスト 55 件を壊さない。追加テストは各 Task の受入に含める。
- AGENTS §3 の実機ゲートは **main への merge / push の条件**として扱う。worktree 内の
  commit はレビュー単位として行うが、Task 10 が通るまで main へ merge しない。
- 実機ゲート（Task 10）が通るまで main へ merge しない。今回の環境では pio が無いため、firmware のビルドは `✗ 未検証` として残す（firmware に差分は無い）。

## Review Focus

1. **USB 抜け / ESP32 リセット中のブリッジ**: プロセスが落ちず、再挿入後に `/esp32/state` が再開する（Task 3 の単体テスト + Task 10 の実機）。
2. **1 回の UART read に複数フレームが到着**: 17 バイトが 1 回で返り、フレームが欠けない（Task 2 の pty テスト）。
3. **FB_IMU が混在する実機**: `umeonigiri_hz_measure --expect-hz 100` が lost frames 0 で終了コード 0（Task 4 の混在テスト）。
4. **comm_check 直後の hz_measure**: 手順 3 の error_flags が残らず、直後の計測が protocol error 0（Task 5 の reset テスト + Task 10 の実機）。
5. **不正なボーレート / 速度上限**: 無言のフォールバックやコールバック例外で落ちず、起動時に明示エラー（Task 2 の baud テスト + Task 7 の CLI 検証）。

---

### Task 0: 設計書と計画書を commit する

**Files:**
- Create: `docs/superpowers/specs/2026-11-20-umeonigiri-serial-bridge-design.md`
- Create: `docs/superpowers/plans/2026-11-20-umeonigiri-serial-bridge-refactor.md`

- [ ] **Step 1: 2 ファイルだけを commit**

```bash
cd /home/so/Meistar/.worktrees/serial-bridge-refactor
git status --short
git add docs/superpowers/specs/2026-11-20-umeonigiri-serial-bridge-design.md \
        docs/superpowers/plans/2026-11-20-umeonigiri-serial-bridge-refactor.md
git commit -m "docs(superpowers): add the umeonigiri_serial_bridge design and plan" \
  -m "Records the confirmed serial I/O defects, the chosen minimal libserial fix, the full rename surface, and the deferred hardware gate."
```

### Task 1: パッケージ・namespace・実行ファイルを umeonigiri_* へ改称

**Files:**
- Move: `ros/src/meister_serial_bridge/` → `ros/src/umeonigiri_serial_bridge/`
- Move: `ros/src/umeonigiri_serial_bridge/include/meister_serial_bridge/` → `include/umeonigiri_serial_bridge/`
- Move: `ros/src/umeonigiri_serial_bridge/launch/serial_bridge.launch.py` → `launch/umeonigiri_serial_bridge.launch.py`
- Modify: `package.xml`, `CMakeLists.txt`, `src/*.cpp`, `include/umeonigiri_serial_bridge/*.hpp`, `test/*.cpp`, `test/fake_esp32.hpp`
- Modify: `build.sh`
- Modify: `ros/src/umeonigiri_autonomy/launch/real_robot.launch.py`
- Modify: `ros/src/umeonigiri_autonomy/package.xml`

**Interfaces:**
- Produces: package `umeonigiri_serial_bridge`; executables `umeonigiri_serial_bridge`, `umeonigiri_comm_check`, `umeonigiri_hz_measure`; namespace `umeonigiri_serial_bridge`; include prefix `umeonigiri_serial_bridge/`.
- Launch: `ros2 launch umeonigiri_serial_bridge umeonigiri_serial_bridge.launch.py`.

- [ ] **Step 1: worktree と branch を確認**

Run:
```bash
cd /home/so/Meistar/.worktrees/serial-bridge-refactor
git branch --show-current   # chore/serial-bridge-refactor
git status --short          # 空（この計画・設計書を commit した後は docs のみ）
```

- [ ] **Step 2: ディレクトリとファイルを git mv**

```bash
cd /home/so/Meistar/.worktrees/serial-bridge-refactor/ros/src
git mv meister_serial_bridge umeonigiri_serial_bridge
cd umeonigiri_serial_bridge
git mv include/meister_serial_bridge include/umeonigiri_serial_bridge
git mv launch/serial_bridge.launch.py launch/umeonigiri_serial_bridge.launch.py
```

- [ ] **Step 3: パッケージ内の識別子を一括置換**

```bash
cd /home/so/Meistar/.worktrees/serial-bridge-refactor/ros/src/umeonigiri_serial_bridge
grep -rl 'meister_serial_bridge' . | xargs sed -i 's/meister_serial_bridge/umeonigiri_serial_bridge/g'
grep -rl 'meister_comm_check' . | xargs sed -i 's/meister_comm_check/umeonigiri_comm_check/g'
grep -rl 'meister_hz_measure' . | xargs sed -i 's/meister_hz_measure/umeonigiri_hz_measure/g'
```

- [ ] **Step 4: `serial_bridge` 実行ファイル名だけを対象に変更**

CMake の 3 箇所（`add_executable` / `target_link_libraries` / `install(TARGETS`）と launch の `executable=` を個別に置換する。`serial_bridge_node.cpp` と `SerialBridgeNode` は役割名として維持する。

Run:
```bash
cd /home/so/Meistar/.worktrees/serial-bridge-refactor/ros/src/umeonigiri_serial_bridge
sed -i 's/add_executable(serial_bridge/add_executable(umeonigiri_serial_bridge/' CMakeLists.txt
sed -i 's/target_link_libraries(serial_bridge/target_link_libraries(umeonigiri_serial_bridge/' CMakeLists.txt
sed -i 's/install(TARGETS serial_bridge/install(TARGETS umeonigiri_serial_bridge/' CMakeLists.txt
sed -i "s/executable='serial_bridge'/executable='umeonigiri_serial_bridge'/" launch/umeonigiri_serial_bridge.launch.py
grep -n 'serial_bridge' CMakeLists.txt launch/umeonigiri_serial_bridge.launch.py
```
Expected: CMake の対象は `umeonigiri_serial_bridge` と `src/serial_bridge_node.cpp` のみ。launch は `executable='umeonigiri_serial_bridge'`。

- [ ] **Step 5: パッケージ外の起動参照を更新**

```bash
cd /home/so/Meistar/.worktrees/serial-bridge-refactor
sed -i 's/meister_serial_bridge/umeonigiri_serial_bridge/g' build.sh
sed -i 's/meister_serial_bridge/umeonigiri_serial_bridge/g' ros/src/umeonigiri_autonomy/launch/real_robot.launch.py
sed -i "s/executable='serial_bridge'/executable='umeonigiri_serial_bridge'/" ros/src/umeonigiri_autonomy/launch/real_robot.launch.py
sed -i "s/name='serial_bridge'/name='umeonigiri_serial_bridge'/" ros/src/umeonigiri_autonomy/launch/real_robot.launch.py
```

`umeonigiri_autonomy/package.xml` の `<exec_depend>meister_web_nav</exec_depend>` の後に次を追加する。

```xml
<exec_depend>umeonigiri_serial_bridge</exec_depend>
```

`umeonigiri_serial_bridge/package.xml` に launch 依存を追加する。

```xml
<exec_depend>launch</exec_depend>
<exec_depend>launch_ros</exec_depend>
```

- [ ] **Step 6: 旧名が残っていないことを確認**

Run:
```bash
cd /home/so/Meistar/.worktrees/serial-bridge-refactor
grep -rn 'meister_serial_bridge\|meister_comm_check\|meister_hz_measure' \
  ros/src/umeonigiri_serial_bridge ros/src/umeonigiri_autonomy/launch/real_robot.launch.py \
  build.sh ros/src/umeonigiri_autonomy/package.xml
```
Expected: 出力なし。文書（AGENTS/docs/firmware README）は Task 8 で更新するため、まだ残ってよい。

- [ ] **Step 7: ビルドと既存テスト**

Run:
```bash
cd /home/so/Meistar/.worktrees/serial-bridge-refactor
source /opt/ros/jazzy/setup.bash
cd ros
colcon build --symlink-install --packages-select umeonigiri_serial_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select umeonigiri_serial_bridge
colcon test-result --verbose
```
Expected: build 成功、55 tests / 0 failures。

- [ ] **Step 8: インストール名と launch 引数を確認**

Run:
```bash
cd /home/so/Meistar/.worktrees/serial-bridge-refactor
source /opt/ros/jazzy/setup.bash && source ros/install/setup.bash
ros2 pkg executables umeonigiri_serial_bridge
ros2 launch umeonigiri_serial_bridge umeonigiri_serial_bridge.launch.py --show-args
```
Expected: 3 実行ファイルが `umeonigiri_*` で並ぶ。launch 引数に `serial_port` と `baud` が出る。

- [ ] **Step 9: Commit**

```bash
cd /home/so/Meistar/.worktrees/serial-bridge-refactor
git add -A
git commit -m "refactor(serial_bridge): rename package and executables to umeonigiri_*" \
  -m "Pure rename: package, include prefix, C++ namespace, installed executables and launch file. Protocol bytes, firmware, and behavior are unchanged. Adds the exec_depend that real_robot.launch.py already assumed."
```

---

### Task 2: 受信を一括化し、未対応ボーレートを明示エラーにする

**Files:**
- Modify: `ros/src/umeonigiri_serial_bridge/src/serial_link.cpp`
- Create: `ros/src/umeonigiri_serial_bridge/test/test_serial_link.cpp`
- Modify: `ros/src/umeonigiri_serial_bridge/CMakeLists.txt`

**Interfaces:**
- Consumes: `umeonigiri_serial_bridge::OpenSerialPort(path, baud)` / `SerialIo::Read(buf, max, timeout_ms)`（シグネチャ変更なし）
- Produces: `Read` は到着済みの全バイトを最大 `max` まで返す。`timeout_ms == 0` は待たない。未対応 baud は `std::exception`。

- [ ] **Step 1: pty テストを書く**

`test/test_serial_link.cpp` を作る。pty は `posix_openpt` → `grantpt` → `unlockpt` → `ptsname` で開き、`PtyPair` は master と slave を保持して両方を close する。標準ヘッダのみを使う。

テスト名とアサーション:

```cpp
TEST(SerialLink, ReadReturnsEveryBufferedByte) {
  // master へ 17 バイト書いて 50 ms 待ってから Read(rx, 64, 100)
  EXPECT_EQ(io->Read(rx.data(), rx.size(), 100), 17u);  // 旧実装は 1
  EXPECT_EQ(std::memcmp(rx.data(), tx.data(), tx.size()), 0);
}

TEST(SerialLink, ReadWithoutTimeoutDrainsBufferedBytes) {
  // 4 バイト到着済み（50 ms 待つ）で Read(rx, 64, 0)
  EXPECT_EQ(io->Read(rx.data(), rx.size(), 0), 4u);     // 旧実装は 0
}

TEST(SerialLink, ReadReturnsZeroWhenIdle) {
  EXPECT_EQ(io->Read(rx.data(), rx.size(), 20), 0u);
}

TEST(SerialLink, RejectsUnsupportedBaud) {
  EXPECT_THROW(msb::OpenSerialPort(pty.slave(), 12345), std::exception);
}
```

- [ ] **Step 2: `CMakeLists.txt` にテストを登録**

`if(BUILD_TESTING)` の中に追加する。

```cmake
ament_add_gtest(test_serial_link test/test_serial_link.cpp)
target_link_libraries(test_serial_link umeonigiri_serial_bridge_core)
```

- [ ] **Step 3: テストが失敗することを確認**

```bash
cd /home/so/Meistar/.worktrees/serial-bridge-refactor
source /opt/ros/jazzy/setup.bash
cd ros
colcon build --symlink-install --packages-select umeonigiri_serial_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select umeonigiri_serial_bridge \
  --ctest-args -R test_serial_link --output-on-failure
```
Expected: `ReadReturnsEveryBufferedByte` が 1 で FAIL、`ReadWithoutTimeoutDrainsBufferedBytes` が 0 で FAIL、`RejectsUnsupportedBaud` が no-throw で FAIL。

- [ ] **Step 4: `serial_link.cpp` を実装**

- `Read` は「timeout>0 なら 1 バイト待つ → `GetNumberOfBytesAvailable()` の分を `Read(dbuf, want, 1)` で一括取得」の順にする。`timeout==0` は待たない。
- `ReadTimeout` だけを握りつぶし、それ以外の `std::exception` は伝播させる（Task 3 が受ける）。
- ボーレートは `int` → `LibSerial::BaudRate` の変換関数に分離し、未知の値は `std::invalid_argument` にする。`BAUD_DEFAULT` へ落とさない。
- `OpenSerialPort` は `LibSerial::OpenFailed` と設定例外を、パスを含む `std::runtime_error` へ包む。

- [ ] **Step 5: テストが通ることを確認**

```bash
cd /home/so/Meistar/.worktrees/serial-bridge-refactor
source /opt/ros/jazzy/setup.bash
cd ros
colcon build --symlink-install --packages-select umeonigiri_serial_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select umeonigiri_serial_bridge \
  --ctest-args -R test_serial_link --output-on-failure
colcon test-result --verbose
```
Expected: `test_serial_link` 4/4 PASS。既存テストも含め 0 failures。

- [ ] **Step 6: Commit**

```bash
cd /home/so/Meistar/.worktrees/serial-bridge-refactor
git add ros/src/umeonigiri_serial_bridge
git commit -m "fix(serial_bridge): return every buffered byte and reject unknown baud" \
  -m "Read() previously took one byte per call because Read(dbuf, 0, 0) is a no-op in libserial 1.0.0. Drain via GetNumberOfBytesAvailable() after the first blocking byte. Unsupported baud now fails instead of silently using 115200."
```

---

### Task 3: I/O 例外で落ちず、壊れたリンクを隔離して再オープンする

**Files:**
- Modify: `ros/src/umeonigiri_serial_bridge/include/umeonigiri_serial_bridge/bridge_core.hpp`
- Modify: `ros/src/umeonigiri_serial_bridge/src/bridge_core.cpp`
- Modify: `ros/src/umeonigiri_serial_bridge/src/serial_bridge_node.cpp`
- Modify: `ros/src/umeonigiri_serial_bridge/test/test_bridge_core.cpp`

**Interfaces:**
- `BridgeCore::DropLink()`: 変更なし（挙動だけ変更）。リンクを破棄せずプロセス寿命の隔離リストへ移す。
- ROS 層: 各コールバックと `RxLoop` が `std::exception` を捕捉し、`DropLink()` を呼ぶ。

- [ ] **Step 1: 失敗する回帰テストを書く**

`test/test_bridge_core.cpp` に追加する。ファクトリが毎回新しい `FakeEsp32` を返し、破棄回数を記録する。`BridgeParams` は既存 `Harness` と同じ値を使う。

```cpp
TEST(BridgeCore, DropLinkQuarantinesBrokenLinkAndReopens) {
  // opens: 1 -> DropLink -> 2, destroyed: 0 のまま
  // DropLink 後は 2 秒進めてから OnCmdVel する（reopen_interval_s = 2.0）
  EXPECT_EQ(opens, 1);
  core.DropLink();
  EXPECT_EQ(destroyed, 0);   // libserial の dtor 例外を避けるため破棄しない
  clock.Advance(2.1);
  core.OnCmdVel(0.5, 0.0);
  EXPECT_EQ(opens, 2);
  EXPECT_EQ(destroyed, 0);
}
```

- [ ] **Step 2: テストが失敗することを確認**

```bash
cd /home/so/Meistar/.worktrees/serial-bridge-refactor
source /opt/ros/jazzy/setup.bash
cd ros
colcon build --symlink-install --packages-select umeonigiri_serial_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select umeonigiri_serial_bridge \
  --ctest-args -R test_bridge_core --output-on-failure
```
Expected: `DropLinkQuarantinesBrokenLinkAndReopens` が `destroyed == 1` で FAIL（現状は reset するため）。

- [ ] **Step 3: `DropLink` を隔離方式にする**

`bridge_core.cpp` の匿名 namespace に、意図的にリークする隔離リストと mutex を置く。`DropLink()` は `link_mutex_` の下で `link_` をリストへ move する。コメントに「libserial 1.0.0 の `~SerialPort()` は noexcept のまま I/O error を投げるため、壊れたリンクを破棄すると terminate する。USB 抜け回数で有界。termios 移行時に削除」と残す。`bridge_core.hpp` の `DropLink` の説明も同じ理由に更新する。

- [ ] **Step 4: ROS 層で例外を捕捉**

`serial_bridge_node.cpp` の次を `try/catch (const std::exception&)` で囲み、`RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "serial bridge I/O error: %s", e.what())` の後に `core_->DropLink()` を呼ぶ。

- `OnCmdVel` の購読コールバック（オドメトリ更新は維持）
- `OnArmJoint`
- `OnGripper`
- `RxLoop` の `core_->RxOnce(...)`（捕捉後 100 ms sleep して continue）
- `SendStopFrames`

- [ ] **Step 5: テストとビルド**

```bash
cd /home/so/Meistar/.worktrees/serial-bridge-refactor
source /opt/ros/jazzy/setup.bash
cd ros
colcon build --symlink-install --packages-select umeonigiri_serial_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select umeonigiri_serial_bridge
colcon test-result --verbose
```
Expected: `test_bridge_core` 全 PASS（新規 1 件を含む）、全パッケージテスト 0 failures。

- [ ] **Step 6: Commit**

```bash
cd /home/so/Meistar/.worktrees/serial-bridge-refactor
git add ros/src/umeonigiri_serial_bridge
git commit -m "fix(serial_bridge): survive serial I/O errors and reopen the link" \
  -m "Catch I/O exceptions in the ROS callbacks and RX loop, then drop the link. DropLink quarantines the broken libserial object until process exit because its noexcept destructor throws on hangup (measured), and a reset would terminate the process."
```

---

### Task 4: FB_IMU を欠落フレームとして数えない

**Files:**
- Modify: `ros/src/umeonigiri_serial_bridge/include/umeonigiri_serial_bridge/feedback_hz_measure.hpp`
- Modify: `ros/src/umeonigiri_serial_bridge/src/feedback_hz_measure.cpp`
- Modify: `ros/src/umeonigiri_serial_bridge/test/test_hz_measure.cpp`

**Interfaces:**
- `Measurement` に `long decoded_bytes` と `long buffered_bytes` を追加。
- `LostFrames()` は `max(0, nbytes - decoded_bytes - buffered_bytes) / kMaxFrameSize` を返す。

- [ ] **Step 1: 失敗する混在テストを書く**

`StateFrame(0)` を 10 個と、`proto::EncodeImu` で作った 6 バイトの FB_IMU を 5 個つなげたストリームを `ScriptedEsp32` に渡す。

```cpp
TEST(FeedbackHz, DoesNotCountImuFramesAsLost) {
  // 10 FB_STATE + 5 FB_IMU -> nbytes 200, frames 10
  EXPECT_EQ(m.frames, 10);
  EXPECT_EQ(m.LostFrames(), 0);   // 旧実装は 1
}
```

- [ ] **Step 2: テストが失敗することを確認**

```bash
cd /home/so/Meistar/.worktrees/serial-bridge-refactor
source /opt/ros/jazzy/setup.bash
cd ros
colcon build --symlink-install --packages-select umeonigiri_serial_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select umeonigiri_serial_bridge \
  --ctest-args -R test_hz_measure --output-on-failure
```
Expected: `DoesNotCountImuFramesAsLost` が `LostFrames() == 1` で FAIL。

- [ ] **Step 3: 計測を実装**

`MeasureFeedbackHz` のループで、`frame.type` に関係なく `decoded_bytes += frame.frameSize` を積む。ループ終了時に `buffered_bytes = parser.buffered()` を入れる。`LostFrames()` を上記の式へ変更し、「FB_STATE が最大長である前提の近似」とヘッダに書く。

- [ ] **Step 4: 全 hz テストと既存欠落テスト**

```bash
cd /home/so/Meistar/.worktrees/serial-bridge-refactor
source /opt/ros/jazzy/setup.bash
cd ros
colcon build --symlink-install --packages-select umeonigiri_serial_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select umeonigiri_serial_bridge \
  --ctest-args -R test_hz_measure --output-on-failure
```
Expected: 6 テスト PASS（既存 5 + 新規 1）。`DetectsFrameDroppedOnTheWire` の期待値 1 は変わらない。

- [ ] **Step 5: Commit**

```bash
cd /home/so/Meistar/.worktrees/serial-bridge-refactor
git add ros/src/umeonigiri_serial_bridge
git commit -m "fix(serial_bridge): stop counting FB_IMU frames as lost" \
  -m "LostFrames() assumed every received byte belonged to a 17-byte FB_STATE. The firmware sends FB_IMU at 20 Hz, so the real robot always reported ~7 lost frames and hz_measure exited non-zero. Account for decoded frame sizes and the partial tail."
```

---

### Task 5: comm_check 後に ESP32 の error_flags を残さない

**Files:**
- Modify: `ros/src/umeonigiri_serial_bridge/include/umeonigiri_serial_bridge/comm_check.hpp`
- Modify: `ros/src/umeonigiri_serial_bridge/src/comm_check.cpp`
- Modify: `ros/src/umeonigiri_serial_bridge/src/comm_check_main.cpp`
- Modify: `ros/src/umeonigiri_serial_bridge/test/fake_esp32.hpp`
- Modify: `ros/src/umeonigiri_serial_bridge/test/test_comm_check.cpp`

**Interfaces:**
- `CheckResult` に `std::string note;` を追加。
- `RunCommCheck(..., reset=true, ...)` は手順 3 の後にもう一度 `ResetEsp32` する。
- `FakeEsp32` に `int rts_high_count() const` を追加（`SetRts(true)` の回数）。

- [ ] **Step 1: 失敗するテストを書く**

```cpp
TEST(CommCheck, ResetsAgainAfterErrFlagStepWhenResetEnabled) {
  msb::test::FakeEsp32 esp;
  const auto result = msb::RunCommCheck("fake", msb::kDefaultBaud, 0.2, true, Opener(&esp));
  EXPECT_TRUE(result.ok());
  EXPECT_EQ(esp.rts_high_count(), 2);   // 旧実装は 1
}

TEST(CommCheck, ReportsResidualErrorFlagWhenResetDisabled) {
  msb::test::FakeEsp32 esp;
  const auto result = msb::RunCommCheck("fake", msb::kDefaultBaud, 0.2, false, Opener(&esp));
  EXPECT_TRUE(result.ok());
  EXPECT_NE(result.note.find("error_flags"), std::string::npos);
  EXPECT_NE(result.note.find("--reset"), std::string::npos);   // 旧実装は note 空
}
```

- [ ] **Step 2: テストが失敗することを確認**

`FakeEsp32::SetRts` を「true の回数を数える」に変え、`rts_high_count()` を追加してから実行する。

```bash
cd /home/so/Meistar/.worktrees/serial-bridge-refactor
source /opt/ros/jazzy/setup.bash
cd ros
colcon build --symlink-install --packages-select umeonigiri_serial_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select umeonigiri_serial_bridge \
  --ctest-args -R test_comm_check --output-on-failure
```
Expected: 2 件 FAIL（RTS 1 回、note 空）。

- [ ] **Step 3: 実装**

- 手順 2 の `WriteAll` 直前に `link->ResetInputBuffer(); parser.Clear();` を入れる。
- 手順 3 の後、`reset` が真なら `ResetEsp32(*link)` をもう一度呼ぶ。`note` に「reset pulse を送った（error_flags は再起動で消える。実機配線は Task 10 で確認）」と入れる。
- `reset` が偽なら `note` に「error_flags は再起動まで残る。次回は --reset を付ける」と入れる。
- `comm_check_main` は `result.note` が空でなければ `note: ...` を表示する。
- 手順数は 3 のまま（AGENTS の実機ゲート記述を変えない）。

- [ ] **Step 4: 全 comm_check テスト**

```bash
cd /home/so/Meistar/.worktrees/serial-bridge-refactor
source /opt/ros/jazzy/setup.bash
cd ros
colcon build --symlink-install --packages-select umeonigiri_serial_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select umeonigiri_serial_bridge \
  --ctest-args -R test_comm_check --output-on-failure
```
Expected: 9 テスト PASS（既存 7 + 新規 2）。reset=true のテストは 1.2 秒かかる。

- [ ] **Step 5: Commit**

```bash
cd /home/so/Meistar/.worktrees/serial-bridge-refactor
git add ros/src/umeonigiri_serial_bridge
git commit -m "fix(serial_bridge): clear the ESP32 error flag after comm_check" \
  -m "Step 3 deliberately corrupts a frame, and the firmware keeps rxErrorCount_ until reboot. Reset once more when --reset is active, and warn when it is not, so the following hz_measure does not fail on a stale flag. Flush the input buffer before the downlink step so the reply cannot be a pre-command FB_STATE."
```

---

### Task 6: アームの丸め表を実際の firmware 経路に合わせる

**Files:**
- Modify: `ros/src/umeonigiri_serial_bridge/src/frames.cpp`
- Modify: `ros/src/umeonigiri_serial_bridge/test/test_protocol.cpp`

**Interfaces:**
- `EncodeArm(buf, cap, tenths)` のシグネチャは変更なし。値域だけ `kMinArmAngle/kMaxArmAngle` へ。

- [ ] **Step 1: `frames.cpp` を変更**

- `EncodeArm` の `ClampAll` を `meister::config::kMinArmAngle` / `kMaxArmAngle` に変える。
- コメントを「`Arm::setAngles` → `PwmServoChannel::setAngleTenths` が使う表。`ClampArm` は別経路（`kMinShoulder`/`kMinJoint`）であり、ここでは使わない」に直す。
- `EncodeSteering` / `EncodeVelocity` / `EncodeArm` の 6 要素手書き初期化を `std::copy(clamped.begin(), clamped.end(), channels)` に置き換え、軸数変更時に初期化子の長さで壊れないようにする。`<algorithm>` を include する。

- [ ] **Step 2: テストのコメントを実経路へ直す**

`test/test_protocol.cpp` の `ArmClampsToJointRange` のコメントを「`Arm::setAngles` のサーボ範囲」に直す。期待値 `{0, 900, 1800, 1800}` は変えない（現在の両表が同じ値のため）。

- [ ] **Step 3: ビルドとテスト**

```bash
cd /home/so/Meistar/.worktrees/serial-bridge-refactor
source /opt/ros/jazzy/setup.bash
cd ros
colcon build --symlink-install --packages-select umeonigiri_serial_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select umeonigiri_serial_bridge
colcon test-result --verbose
```
Expected: 全テスト PASS。値が同じため期待値の変更は不要。

- [ ] **Step 4: Commit**

```bash
cd /home/so/Meistar/.worktrees/serial-bridge-refactor
git add ros/src/umeonigiri_serial_bridge
git commit -m "fix(serial_bridge): clamp arm angles with the range the firmware path uses" \
  -m "CMD_ARM_ANGLE goes to Arm::setAngles and the servo channel range (kMinArmAngle/kMaxArmAngle), not ClampArm (kMinJoint/kMaxJoint). The values happen to match today; the table and comment were wrong."
```

---

### Task 7: 不正パラメータで起動時に落とす・ノード名を 1 箇所にする

**Files:**
- Modify: `ros/src/umeonigiri_serial_bridge/src/serial_bridge_node.cpp`

**Interfaces:**
- `constexpr char kNodeName[] = "umeonigiri_serial_bridge";`（namespace スコープ）
- 不正パラメータで `std::invalid_argument` → `main` が捕捉して終了コード 1。

- [ ] **Step 1: `kNodeName` を 1 箇所に置く**

`Node(kNodeName)` と `rclcpp::get_logger(kNodeName)` の両方で使う。`RCLCPP_ERROR` のハードコード文字列を置き換える。

- [ ] **Step 2: パラメータ検証を追加**

`declare_parameter` の直後、`BridgeCore` 生成前に次を検証する。

- `max_linear_vel > 0`
- `max_steering_deg > 0`
- `cmd_timeout > 0`
- `baud > 0`

不正なら `throw std::invalid_argument("<name> must be positive")`。

- [ ] **Step 3: `main` で捕捉**

`std::make_shared<SerialBridgeNode>()` を `try/catch (const std::exception&)` で囲み、`RCLCPP_FATAL(rclcpp::get_logger(umeonigiri_serial_bridge::kNodeName), "%s", e.what())` → `rclcpp::shutdown()` → `return 1`。

- [ ] **Step 4: 不正値で終了コード 1 を確認（実機不要）**

```bash
cd /home/so/Meistar/.worktrees/serial-bridge-refactor
source /opt/ros/jazzy/setup.bash
cd ros
colcon build --symlink-install --packages-select umeonigiri_serial_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
timeout 10 ros2 run umeonigiri_serial_bridge umeonigiri_serial_bridge \
  --ros-args -p max_linear_vel:=0.0; echo "exit=$?"
```
Expected: `exit=1`。`max_linear_vel must be positive` がログに出る。シリアルは開かない。

- [ ] **Step 5: 正常値では起動することを確認**

```bash
cd /home/so/Meistar/.worktrees/serial-bridge-refactor
source /opt/ros/jazzy/setup.bash && source ros/install/setup.bash
timeout 3 ros2 run umeonigiri_serial_bridge umeonigiri_serial_bridge; echo "exit=$?"
```
Expected: 存在しない `/dev/ttyUSB0` の open 失敗ログを出しつつ、SIGTERM で正常終了（exit=0）または timeout（exit=124）。134（abort）や 139（segfault）でないこと。

- [ ] **Step 6: Commit**

```bash
cd /home/so/Meistar/.worktrees/serial-bridge-refactor
git add ros/src/umeonigiri_serial_bridge/src/serial_bridge_node.cpp
git commit -m "fix(serial_bridge): fail fast on invalid parameters and centralize the node name" \
  -m "A zero max_linear_vel made TwistToActuators throw inside the /cmd_vel callback, which terminates rclcpp::spin. Validate before creating the core. Node name and logger now share one constant."
```

---

### Task 8: 現行ドキュメントを新しい名前とコマンドへ揃える

**Files:**
- Modify: `AGENTS.md`
- Modify: `docs/commands.md`
- Modify: `docs/architecture.md`
- Modify: `docs/code-map.md`
- Modify: `docs/coding-standards.md`
- Modify: `docs/features/07-esp32-uart.md`
- Modify: `docs/design/07-esp32-uart.md`
- Modify: `docs/design/08-rocker-bogie.md`
- Modify: `firmware/main_board/README.md`
- Modify: `ros/src/meister_web_nav/CMakeLists.txt`（コメントのみ）

**Interfaces:**
- 現行ドキュメントのコマンドが `ros2 run umeonigiri_serial_bridge umeonigiri_comm_check` / `umeonigiri_hz_measure` を指す。

- [ ] **Step 1: 旧名の残存箇所を列挙**

```bash
cd /home/so/Meistar/.worktrees/serial-bridge-refactor
grep -rn 'meister_serial_bridge\|meister_comm_check\|meister_hz_measure\|serial_bridge.launch.py' \
  AGENTS.md docs firmware/main_board/README.md ros/src --exclude-dir=build --exclude-dir=install
```
Expected: 現行 docs と AGENTS、firmware README、`meister_web_nav/CMakeLists.txt` のコメントが出る。`docs/superpowers/specs/2026-11-18-*` は歴史的記録なので変更しない。

- [ ] **Step 2: コマンドと名前を更新**

- `AGENTS.md` §2: 「Tests (Python) pytest」を現行の C++/gtest 構成に合わせて直す。
- `AGENTS.md` §3: テストを `cd ros && colcon test --packages-select umeonigiri_serial_bridge && colcon test-result --verbose` に置換（pytest は存在しない）。実機コマンドを `ros2 run umeonigiri_serial_bridge umeonigiri_comm_check` / `umeonigiri_hz_measure` に。
- `AGENTS.md` §4 NG リストの「軸数を変えるときは C++ と Python を直す」を、現行どおり「firmware の config と PC の両テスト」に直す。
- `AGENTS.md` §6 の「`protocol.py` と C++ が一致」を、現行どおり「firmware の `meister_protocol.h/.cpp` が唯一の実装で、PC は同じソースをコンパイルする」に直す。
- `docs/commands.md`: Python/pytest の節を gtest/colcon に置換。`ros2 run` の名前を更新。前提に `libserial-dev` を追加。
- `docs/architecture.md`: パッケージ名と図を更新。存在しない `protocol.py` / `kinematics.py` の行を、現行の C++ ファイル（`serial_io.hpp` / `bridge_core.hpp` / `frames.cpp` / `kinematics.cpp`）へ直す。
- `docs/code-map.md`: パッケージ表・内部ファイル表・検証コマンドを更新。テスト件数は Task 9 の実測値に合わせる。
- `docs/coding-standards.md`: シリアル通信の採用を `libserial` に、C++ テストの配置を `ros/src/umeonigiri_serial_bridge/test/`（gtest）に直す。
- `docs/features/07-esp32-uart.md`, `docs/design/07-esp32-uart.md`, `docs/design/08-rocker-bogie.md`: パスと名前のみ更新。
- `firmware/main_board/README.md`: `ros2 run` / `ros2 launch` の名前を更新。
- `ros/src/meister_web_nav/CMakeLists.txt` の比較コメントを `umeonigiri_serial_bridge` に。

- [ ] **Step 3: 旧名が残っていないことを確認**

```bash
cd /home/so/Meistar/.worktrees/serial-bridge-refactor
grep -rln 'meister_serial_bridge\|meister_comm_check\|meister_hz_measure' . \
  --exclude-dir=.git --exclude-dir=build --exclude-dir=install --exclude-dir=log \
  --exclude-dir=.worktrees --exclude-dir=superpowers
```
Expected: 出力なし。`docs/superpowers/specs/2026-11-18-*` と本計画/設計書は `superpowers` 除外で対象外。

- [ ] **Step 4: Commit**

```bash
cd /home/so/Meistar/.worktrees/serial-bridge-refactor
git add AGENTS.md docs firmware/main_board/README.md ros/src/meister_web_nav/CMakeLists.txt
git commit -m "docs: point commands and code maps at umeonigiri_serial_bridge" \
  -m "Replace the stale pytest instructions with colcon/gtest, update the installed executable and launch names, and remove the Python protocol/kinematics rows that no longer exist. Dated design specs are left as historical records."
```

---

### Task 9: ホスト検証をまとめて実施する

**Files:** なし（検証のみ）

- [ ] **Step 1: パッケージ単体のビルドとテスト**

```bash
cd /home/so/Meistar/.worktrees/serial-bridge-refactor
source /opt/ros/jazzy/setup.bash
cd ros
colcon build --symlink-install --packages-select umeonigiri_serial_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select umeonigiri_serial_bridge
colcon test-result --verbose
```
Expected: 0 failures。テスト件数を記録する（既存 55 + 新規: serial_link 4 / bridge_core 1 / hz_measure 1 / comm_check 2 = 63 の見込み）。

- [ ] **Step 2: ワークスペース全体のビルド**

```bash
cd /home/so/Meistar/.worktrees/serial-bridge-refactor
./build.sh
```
Expected: 6 パッケージが成功。モデル（`*.onnx`）が無くても CMake の `EXISTS` ガードで configure が通る。

- [ ] **Step 3: launch と CLI のスモーク**

```bash
cd /home/so/Meistar/.worktrees/serial-bridge-refactor
source /opt/ros/jazzy/setup.bash && source ros/install/setup.bash
ros2 pkg executables umeonigiri_serial_bridge
ros2 launch umeonigiri_serial_bridge umeonigiri_serial_bridge.launch.py --show-args
ros2 run umeonigiri_serial_bridge umeonigiri_comm_check --help
ros2 run umeonigiri_serial_bridge umeonigiri_hz_measure --help
```
Expected: 3 実行ファイル、launch 引数 `serial_port`/`baud`、両 CLI の usage が表示される。

- [ ] **Step 4: firmware の扱いを記録**

`pio` はこの環境に無い（`which pio` が空）。firmware に差分は無いため、本計画の受入はホスト側で完了とする。AGENTS の「4 ビルド」を厳密に適用する場合は、`pio` のある環境で
`cd firmware/main_board && pio run -e esp32dev -e esp32dev_usbuart && pio test -e native`
を実行し、結果を追記する。現状は `✗ 未検証`。

- [ ] **Step 5: diff をレビュー**

```bash
cd /home/so/Meistar/.worktrees/serial-bridge-refactor
git log --oneline main..HEAD
git diff main...HEAD --stat
git diff main...HEAD --check
```
Expected: 9 commits（計画/設計 1 + 実装 7 + docs 1）、空白エラーなし。`main` は変更しない。

---

### Task 10: 実機ゲート（後日・未実施）

**Files:** なし

実機が用意でき次第、次を順に実行する。1 つでも失敗したら merge しない。

- [ ] **Step 0: main の旧成果物を消す（merge 後）**

旧パッケージ名の build/install が残っていると、古い実行ファイルを拾う可能性がある。絶対パスを確認してから削除する。

```bash
cd /home/so/Meistar
pwd
ls -d ros/build/meister_serial_bridge ros/install/meister_serial_bridge 2>/dev/null
rm -rf ros/build/meister_serial_bridge ros/install/meister_serial_bridge
./build.sh
```
Expected: 旧ディレクトリが消え、`ros/install/umeonigiri_serial_bridge` だけになる。

- [ ] **Step 1: comm_check 3 手順**

```bash
source /home/so/Meistar/ros/install/setup.bash   # merge 後の main
ros2 run umeonigiri_serial_bridge umeonigiri_comm_check --port /dev/serial/by-id/<ESP32>
```
Expected: `uplink` / `downlink` / `err_flag` の 3 行が OK、`PASS`、終了コード 0。`note` に再起動でフラグを消した旨が出る。

- [ ] **Step 2: 直後の hz_measure**

```bash
ros2 run umeonigiri_serial_bridge umeonigiri_hz_measure --port /dev/serial/by-id/<ESP32> --expect-hz 100
```
Expected: `protocol errors : 0`、`lost frames : 0`、終了コード 0（FB_IMU 20 Hz が混ざった状態で）。

- [ ] **Step 3: 走行中の抜き差し**

ブリッジ起動中に USB を抜き、5 秒後に再挿入する。

Expected: プロセスが生存し、`serial open failed` の後に `/esp32/state` が再開する。落ちない。再オープン後に `umeonigiri_hz_measure` が通る。

- [ ] **Step 4: 記録**

結果（日時・ポート・firmware のビルド env・各コマンドの終了コード）を PR 本文か `docs/bench/` に残す。通るまで main へ merge しない。

---

## 実行順序と commit 一覧

1. `docs(superpowers): add the umeonigiri_serial_bridge design and plan`
2. `refactor(serial_bridge): rename package and executables to umeonigiri_*`
3. `fix(serial_bridge): return every buffered byte and reject unknown baud`
4. `fix(serial_bridge): survive serial I/O errors and reopen the link`
5. `fix(serial_bridge): stop counting FB_IMU frames as lost`
6. `fix(serial_bridge): clear the ESP32 error flag after comm_check`
7. `fix(serial_bridge): clamp arm angles with the range the firmware path uses`
8. `fix(serial_bridge): fail fast on invalid parameters and centralize the node name`
9. `docs: point commands and code maps at umeonigiri_serial_bridge`

Task 1 の後に一度、Task 9 の後に一度、全テストを回す。実機（Task 10）は未実施のまま `✗ 未検証` と記録する。
