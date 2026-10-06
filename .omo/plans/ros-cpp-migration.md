# ROS ノードの Python → C++/HPP 化計画

> 作成日: 2026-10-05
> 対象: `sosukigara/2026-Meister`
> 根拠: ROS 2 Jazzy / Ubuntu 24.04 (noble) で一次情報を実地確認済み
> 準拠: `AGENTS.md` R1/R2/R4/R5/R6/R8/R10/R12

---

## 0. 決定事項（ユーザー確認済み）

| 論点 | 決定 |
|---|---|
| スコープ | **ROS ノードのみ** C++ 化。launch / setup.py / 非ノードの Python ユーティリティは Python のまま |
| Nav2 ナビゲータ | `nav2_simple_commander` は Python 専用 → **C++ action client を自前実装** |
| 外部依存 | **全部 Ubuntu 標準パッケージ**（onnxruntime は `ros-jazzy-onnxruntime-vendor`、serial は libserial、http は cpp-httplib、json は nlohmann） |
| 進行 | **パッケージ単位に順次**（bridge → vision → web_nav）。1 パッケージ完結で検証通してから次へ |

### スコープに**含まれない**もの（明示）

- `src/*/launch/*.launch.py` — ROS 2 の launch は Python 専用 API。C++ 化不可。
- `src/*/setup.py` — `ros2 run` の実行ファイル**名を維持**する（launch 側の差分をゼロにするため）。
- `tools/gen_config.py` — 生成器。Python のまま残す（C++ 側だけを引き続き生成する）。
- `ros2_autonomous_nav` の非ノード Python: `generate_map.py`, `edit_video.py`, `create_short.py`, `create_thumbnail.py`, `generate_voiceover.py`, `render_code_snippets.py` — ロボット制御と無関係。
- `firmware/` — 既に C++。**変更しない**（参照のみ）。

---

## 1. 一次情報（実地確認済み / 出典つき）

### 1.1 利用可能な依存

| 依存 | apt パッケージ | バージョン | 状態 |
|---|---|---|---|
| ONNX Runtime C++ | `ros-jazzy-onnxruntime-vendor` | 0.1.0-2noble.20260623.230327（ORT 1.24.3 同梱） | 未 install。deb 内容を確認済み: `include/onnxruntime_c_api.h` `onnxruntime_cxx_api.h` `onnxruntime_cxx_inline.h`、`lib/cmake/onnxruntime/onnxruntimeConfig.cmake` + `onnxruntimeTargets.cmake`、`lib/libonnxruntime_providers_shared.so` ほか |
| serial | `libserial-dev` | 1.0.0-9build1 | apt 可 |
| HTTP server | `libcpp-httplib-dev` | 0.14.3+ds-1.1build2 | apt 可 |
| JSON | `nlohmann-json3-dev` | 3.11.3-1 | **導入済み** |
| OpenCV | `libopencv-dev` | 4.6.0 | 導入済み（`libopencv_dnn.so` あり） |
| gtest | `libgtest-dev` + `ament_cmake_gtest` | — | 導入済み |
| lint | `ament_lint_auto` | — | 導入済み（`meistar_description` で使用実績あり） |

CMake 側の参照先は `find_package(onnxruntime_vendor REQUIRED)` 相当（config は `<prefix>/opt/onnxruntime_vendor/lib/cmake/onnxruntime/` に配置される）。`ros-jazzy-onnxruntime-vendor` install 後に `ament` パッケージが `CMAKE_PREFIX_PATH` に `opt/onnxruntime_vendor` を含めるか**は ✗ 未検証**（Phase 0 で実測して確定させる）。

### 1.2 Nav2 の C++ ヘッダ

`/opt/ros/jazzy/include/nav2_msgs/nav2_msgs/action/` に `navigate_to_pose.h/.hpp`, `follow_waypoints.h/.hpp`, `navigate_through_poses.h/.hpp`, `spin.h`, `back_up.h`, `wait.h` 等 **20 種齊全**。

- **`nav2_simple_commander` は Python 専用**（`/opt/ros/jazzy/lib/python3.12/site-packages/nav2_simple_commander` のみ、C++ 版なし）。→ `NavWorker` の C++ 化では action client を自前実装する。
- `nav2_util` に `NodeThread`, `LifecycleNode` の C++ ヘッダあり。ライフサイクルチェックに使う。
- `vision_msgs` の `BoundingBox2D.h`, `Detection2D.h`, `Detection2DArray.h`, `ObjectHypothesis.h` も齊全。

### 1.3 リポジトリ状態（2026-10-05 時点）

- `main` = `5881810 Merge branch 'feat/orangepi-stack' into main`。**作業ツリーはクリーン、merge は完了済み**。（以前の調査時点であった未完了 merge は既に conclude されている）
- 移行着手のブロッカーは無い。

---

## 2. 移行パックの全体像

### 2.1 移行マトリクス

| パッケージ | Python ファイル | 行数 | 移行先 | 備考 |
|---|---|---|---|---|
| `meister_serial_bridge` | `serial_bridge_node.py` | 228 | `src/serial_bridge_node.cpp` | ROS ノード本体 |
| | `protocol.py` | 165 | **firmware の `meister_protocol.h` を再利用** | 二重定義を消せる最大の収穫 |
| | `kinematics.py` | 68 | `src/kinematics.cpp` + `include/kinematics.hpp` | 自転車モデル。firmware の `kinematics.h` とは別物 |
| | `comm_check.py` | 265 | `src/comm_check.cpp` | CLI。libserial + 抽象 I/O |
| | `feedback_hz_measure.py` | 172 | `src/feedback_hz_measure.cpp` | CLI |
| | `generated_config.py` | 22 | **削除** → `hal/generated_config.h` を include | 生成器の Python 出力をやめる |
| `meister_vision` | `detection_node.py` | 277 | `src/detection_node.cpp` | |
| | `yolo_detector.py` | 289 | `src/yolo_detector.cpp` + `include/yolo_detector.hpp` | ORT C++ API |
| | `hand_landmark_detector.py` | 251 | `src/hand_landmark_detector.cpp` + hpp | ★要確認★ 契約を C++ でも維持 |
| | `hand_landmarks_node.py` | 234 | `src/hand_landmarks_node.cpp` | |
| | `pc_camera_node.py` | 94 | `src/pc_camera_node.cpp` | |
| | `download_model.py` | 106 | Python のまま | ROS 非依存の CLI。onnxruntime 不要 |
| `meister_web_nav` | `web_nav_server.py` | 62 | `src/web_nav_server.cpp` | main() |
| | `http_app.py` | 191 | `src/http_app.cpp` + hpp | cpp-httplib |
| | `map_listener.py` | 91 | `src/map_listener.cpp` + hpp | |
| | `map_cache.py` | 126 | `src/map_cache.cpp` + hpp | libpng |
| | `nav_worker.py` | 106 | `src/nav_worker.cpp` + hpp | **rclcpp_action 自前実装** |
| `ros2_autonomous_nav` | `scripts/waypoint_follower.py` | 81 | `src/waypoint_follower.cpp` | 単独の ament_cmake 化。**Phase 3 で判断**（下記） |

合計 **約 2,600 行**の Python を C++ に置き換える。

### 2.2 `waypoint_follower.py` の扱い

`ros2_autonomous_nav` は既に `ament_python` だが、中身は launch 10 個 + shell スクリプトが主で、`waypoint_follower.py` だけがノード。**ノードを C++ 化すると `setup.py` と launch 10 個が混在する中途半端なパッケージになる。**

選択肢:
- (a) `scripts/waypoint_follower.py` を `meister_web_nav` の `nav_worker` に統合して消す。
- (b) `ros2_autonomous_nav` を `ament_cmake` 化。launch は `install(DIRECTORY launch)` で Python ファイルのまま配置できるので技術的には可能。
- (c) `ament_python` のまま `ros2 run` で C++ バイナリを呼ぶ（不可: `ros2 run` は package の executable を agg に探すが、`ament_python` では C++ バイナリを install する経路が無い）。

**Phase 3 の開始時に (a)/(b) を使うユーザーと決める。** 現状は `nav_worker` とほぼ同一の処理（`BasicNavigator` で waypoint 巡回）なので **(a) が整合的**。

---

## 3. フェーズ構成

各フェーズは「ビルド → テスト → 実機/実データ → 文書」の順を必ず踏む（`AGENTS.md` R6）。

### Phase 0: 基盤準備（依存導入・差分ゼロ）

**目的**: 移行着手前に、外部依存の解決経路を確定させる。

1. `sudo apt install ros-jazzy-onnxruntime-vendor libserial-dev libcpp-httplib-dev`
2. `find_package` が通ることを確認（最小 CMake プロジェクトを `/tmp/opencode/` に作って throwaway で試す）
3. `ros-jazzy-onnxruntime-vendor` が `CMAKE_PREFIX_PATH` に `opt/onnxruntime_vendor` を含めるか実測 → 含まれないなら `CMAKE_PREFIX_PATH` に明示追加する旨を計画に追記
4. `/opt/ros/jazzy/include` に firmware ヘッダの include パスを張れるか確認（firmware を変更せずに参照する仕組み）
5. `ament_lint_auto` の **cpplint を有効化**する（現状 `meistar_description` で `ament_cmake_cpplint_FOUND TRUE` で無効化されている。R9「装飾罫 `// =====` 禁止」「WHAT 書き起こし禁止」は cpplint だけでは判定できないので、他に機械チェックを足す）

**ゲート**: ビルドの成否を見る。**コミットしない**（環境変更のみなので、`docs/` に依存リストを書くだけ）。

---

### Phase 1: `meister_serial_bridge`

**最大の収穫がここにある。Python の `protocol.py` は firmware の `meister_protocol.h` とバイト単位で一致していなければならず、それが人手同期しているだけ。** firmware 側を直接 include すれば二重定義が消える。

#### 1a. 共通 codec の切り出し（pure refactor）

1. `tools/gen_config.py` から Python 出力生成を削除し、`firmware/include/hal/generated_config.h` のみを生成するようにする
   - `generated_config.py` の消費者は**ゼロ**（`grep` で確認済み: `protocol.py` は `NUM_DRIVE_MOTORS = 6` をハードコード、`generated_config.py` は import されていない）。→ **削除しても壊れない**（R2 違反の解消）
2. `src/meister_serial_bridge/` を `ament_cmake` 化
   - `package.xml`: `<build_type>ament_python</build_type>` → `ament_cmake`、`exec_depend` 追加（`python3-serial` を削除し、`serial_bridge` の実装を libserial に）
   - `setup.py` → `CMakeLists.txt`。`install(TARGETS serial_bridge meister_comm_check meister_hz_measure DESTINATION lib/${PROJECT_NAME})` で**実行ファイル名を据え置き**（launch 側無変更）
3. `protocol.py` を `include/meister_protocol.hpp` に展開。ただし**関数を書き写さず、`firmware/include/meister_protocol.h` を `#include` するだけ**にするのが理想
   - ブロッカ: firmware の `include/` は PlatformIO の src_dir 外。`target_include_directories` に `${CMAKE_SOURCE_DIR}/../../../firmware/include` を足す案と、`ros2_autonomous_nav` 等と firmware を並列 package 化（`install(DIRECTORY include/ DESTINATION include)` して ament の include 空間に乗せる）案がある。**後者を採用**（ament の流儀に乗る）
   - **注意**: firmware を `ament_cmake` 化すると `pio run` と `colcon build` の両方に影響する。`install(DIRECTORY include/)` だけ追加し、`CMakeLists.txt` の他は触らない（pure refactor として成立させる）

**docs**: `AGENTS.md` R2 の「設定値は 1 箇所以外で定義しない」を満たすので、R2 違反の解消として明記する。

#### 1b. 各ファイルの C++ 化

| 対象 | C++ 側 | 備考 |
|---|---|---|
| `kinematics.py` | `include/kinematics.hpp` / `src/kinematics.cpp` | `twist_to_actuators(vx, wz, wheelbase, max_linear_vel, max_steering_deg, steer_invert, drive_invert) -> (steer_tenths, vel_permille)` をそのまま移植。`EPSILON_VX=0.01`、`atan2(wheelbase*wz, vx)` |
| `serial_bridge_node.py` | `src/serial_bridge_node.cpp` | rclcpp。`/cmd_vel` subscribe、`/esp32/state` publish、cmd_timeout ウォッチドッグ。**Python の daemon thread による `_rx_loop` は rclcpp の `create_wall_timer` または専用 `std::jthread` に置き換える**（AGENTS.md R11：静的初期化順に依存しない） |
| `comm_check.py` | `src/comm_check.cpp` | libserial。uplink/downlink/err_flag の 3 手順 |
| `feedback_hz_measure.py` | `src/feedback_hz_measure.cpp` | libserial |

**テストの移植**: pytest → gtest（`ament_cmake_gtest`）。

| Python | ケース数 | C++ | 方針 |
|---|---|---|---|
| `test_protocol.py` | 5 | `test_protocol.cpp` | firmware 側 `firmware/test_native/test_protocol.cpp` と重複しうる。**一元化すれば重複削減になる** |
| `test_kinematics.py` | 7 | `test_kinematics.cpp` | 1:1 移植 |
| `test_comm_check.py` | 6 | `test_comm_check.cpp` | **`loop://` モックが C++ に無い** → I/O 抽象（`SerialPort` 仮想基底）を入れて `FakeSerial` に差し替える。同じく `test_hz_measure.py` 4 ケース |
| `test_serial_bridge_node.py` | 新規 staged | `test_serial_bridge_node.cpp` | rclcpp の `launch_testing` か、LIN 層のフェイクで代替 |

**方針**: ビットストリームを返す `SerialIo` 抽象 interface を `comm_check` / `hz_measure` / `serial_bridge_node` で共有する。これが `libserial` 直接触るのを防ぎ、テストを全部 Fake にできる。

**ゲート（AGENTS.md R6）**:
```
colcon build --symlink-install --packages-select meister_serial_bridge
colcon test --packages-select meister_serial_bridge
ros2 run meister_serial_bridge meister_comm_check --port /dev/ttyUSB0   # 実機 3 手順 PASS
pio run -e native && pio test -e native                                 # firmware 側の回帰なし
```

**コミット分割（R12）**:
1. `refactor(serial_bridge): gen_config.py の Python 出力生成を削除し generated_config.py を削除`（pure refactor、回帰証明）
2. `refactor(serial_bridge): ament_python → ament_cmake 化しヘッダ共有`（pure refactor）
3. `feat(serial_bridge): protocol/kinematics を C++ に移植し pytest → gtest`（機能追加、Python 実装は併存）
4. `refactor(serial_bridge): Python 実装を削除`

---

### Phase 2: `meister_vision`

`ros-jazzy-onnxruntime-vendor` を使う。onnxruntime の Python 版 1.28.0 は `~/.local` にあるだけなので、**C++ 化でシステム側のバージョン依存は解消できる**。

| 対象 | C++ 側 | 備考 |
|---|---|---|
| `yolo_detector.py` | `include/yolo_detector.hpp` / `src/yolo_detector.cpp` | `Ort::Env` / `Ort::Session`。letterbox 640、出力 (1,84,8400) パース、NMS は **OpenCV `cv::dnn::NMSBoxes`**（Python は `cv2.dnn.NMSBoxes` と同等）。`COCO_CLASSES` は 80 個の `constexpr std::array<const char*, 80>` |
| `detection_node.py` | `src/detection_node.cpp` | `vision_msgs`  publishers。`_bgr_from_image_msg` の numpy2/cv_bridge 不整合回避は、**C++ では cv_bridge を直接使い、フォールバックが不要になる**（Python 特有の回避コードが消える） |
| `hand_landmark_detector.py` | `include/hand_landmark_detector.hpp` / `src/hand_landmark_detector.cpp` | ★要確認★ 契約を維持。入力メタデータから動的形状を検査 → `ValueError` 相当を `throw`。**C++ でも「壊れた keypoint を黙って流す」より例外で落とす側に倒す（R4）** |
| `hand_landmarks_node.py` | `src/hand_landmarks_node.cpp` | `geometry_msgs/PoseArray` publish |
| `pc_camera_node.py` | `src/pc_camera_node.cpp` | `cv::VideoCapture` |
| `download_model.py` | **Python のまま** | ROS 非依存の CLI。onnxruntime 不要。C++ にすると libcurl 等が増えるだけ |

**パラメータは不変**（`model_path`, `conf_threshold`=0.25, `iou_threshold`=0.45, `image_topic`, `publish_annotated`=true, `rate`=3.0, `inference_threads`=2）。`pc_camera` の `device`/`width`/`height`/`fps`/`frame_id` も不変。

**テストの移植**: `test_detector.py` 16 ケース + `test_hand_landmarks.py` 新規 → gtest。onnxruntime 依存のため、実モデルがなければスキップする構造を C++ でも持つ（`ament_add_gtest` の `DISABLED_` か、`GTEST_SKIP()`）。

**性能ゲート**: `docs/vision-perf-baseline.md`（Orange Pi 5 Max 実測）との比較を必ず取る。ORT C++ API は Python API より同等〜僅速のはずだが、**`inference_threads` のマッピングが Python の `intra_op_num_threads` と同じ意味になるか**を実測で確認する。

**ゲート**:
```
colcon build --packages-select meister_vision
colcon test --packages-select meister_vision
start_vision.sh   # 実際の検出が従来と同じ出力を出すか目視 + topic 確認
```

**コミット分割**: 4 コミット（生成器の移植がないぶん 1 つ減る）。

---

### Phase 3: `meister_web_nav`

最も大きい変更（HTTP サーバ + Nav2 action client の自前実装）。

| 対象 | C++ 側 | 備考 |
|---|---|---|
| `map_cache.py` | `include/map_cache.hpp` / `src/map_cache.cpp` | `blake2b(digest_size=8)` → **`std::hash` か OpenSSL の EVP**。BLAKE2b の実装依存を持ち込まないなら **version を別の cheap なハッシュ（CRC32 / FNV-1a）に変える**。PNG は `cv::imencode(".png")`（Python は PIL）。**370ms → OpenCV なら数十 ms になる可能性**（`docs` に実測を書く） |
| `map_listener.py` | `include/map_listener.hpp` / `src/map_listener.cpp` | `/map`（TRANSIENT_LOCAL QoS）と `/tf` 購読。`robot_pose()` の 2D 合成（map→odom∘odom→base_footprint）を Eigen か手書きで |
| `nav_worker.py` | `include/nav_worker.hpp` / `src/nav_worker.cpp` | **`nav2_simple_commander` の代わりに rclcpp_action で `/navigate_to_pose` の action client を自前実装。`waitUntilNav2Active(localizer='robot_localization')` 相当は `nav2_util::LifecycleNode` でライフサイクル遷移を購読**。`waypoint_follower` が無いので `follow_waypoints` は使わず、`navigate_to_pose` を順次呼ぶ（現状仕様を維持） |
| `http_app.py` | `include/http_app.hpp` / `src/http_app.cpp` | cpp-httplib。GET `/`, `/index.html`, `/app.js`, `/style.css`, `/api/map`, `/api/map.png`, `/api/pose`, `/api/status`、POST `/api/nav`, `/api/cancel`。`MAX_BODY_BYTES=1_000_000` は httplib の `set_payload_max_length`。パストラバーサル対策（`/api/map.png` などの静的ファイル配信）を維持。ETag/304 は Python 版に既にあるので移植 |
| `web_nav_server.py` | `src/web_nav_server.cpp` | main()。**Python 版は ROS executor を別スレッドに逃がして MapListener を spin し、NavWorker も別スレッド** → C++ では `MultiThreadedExecutor` + 別スレッドで NavWorker を回す。**NavWorker 内のブロッキング action call には注意**：action client は自前なので、`spin_until_future_complete` を呼ぶスレッドが executor を掴まないようにする |

**スレッド設計（R11 / R8 の観点）**:
- MapListener: `MultiThreadedExecutor` 上（callback group を Reentrant に）
- HTTP server: httplib の内部スレッド or 自前 thread
- NavWorker: 専用 thread + 内部 action client。**ブロッキング呼び出しは executor を掴まない形にする**
- `map_cache` の `state_lock` → `memo_lock` の**順序固定**は C++ でも維持する（`std::mutex` 2 本、lock_guard を関数境界で固定）。デッドロック回避はここだけ

**テストの移植**: `test_http_app.py`（308 行）+ `test_map_cache.py`（151 行）→ gtest。`conftest.py` の fixture は直接の移植対象ではない。httplib は `httplib::Client` で in-process テスト（`Server::bind_to_any_port` 相当が使えるので、ポートを動的に取ってテスト可能）。

**ゲート**:
```
colcon build --packages-select meister_web_nav
colcon test --packages-select meister_web_nav
ros2 launch meister_web_nav web_nav.launch.py   # :8088 でブラウザから地図・ナビ・キャンセルが従来通り動く
```

**コミット分割**: 4 コミット（map_cache → map_listener → http_app → web_nav_server + Python 削除）。

---

### Phase 4: 統合・文書

1. `ros2_autonomous_nav/scripts/waypoint_follower.py` の扱い（§2.2 の (a)/(b)）を確定して実行
2. `build.sh` の確認（`--packages-select` のリストは据え置きなので**変更不要の予定**。変更したら理由を `git diff` で確認）
3. `kill_ros.sh` / `start_*.sh` の確認（バイナリ名を維持するので**変更不要の予定**）
4. `docs/code-map.md` の更新（パッケージ構成・依存）
5. `AGENTS.md` に「ROS ノードは C++」という規約を追記するか判断
6. 全パッケージを一括でビルドして **開発環境全体の整合性を確認**

**最終ゲート（AGENTS.md R6 の 4 通り + Python 側）**:
```
colcon build --symlink-install                 # 全パッケージ
colcon test                                     # 全 gtest
pio run -e esp32dev -e esp32dev_usbuart
PLATFORMIO_BUILD_FLAGS="-DMSTE_SERVO_DRIVER_DEBUG=1" pio run -e esp32dev
pio test -e native
ros2 run meister_serial_bridge meister_comm_check --port /dev/ttyUSB0
start_meister.sh && start_vision.sh            # 目視・topic 確認
```

---

## 4. リスクと対策

| # | リスク | 影響 | 対策 |
|---|---|---|---|
| R-1 | **`nav2_simple_commander` に C++ 版が無い**。action client の自前実装で落とし穴が多い（goal 応答、feedback 受信、cancel の応答待ち、`waitUntilNav2Active` のライフサイクル判定） | Phase 3 の工数増 | `nav2_msgs` の C++ ヘッダは齊全なので action client 自前実装は可能。`action_msgs` の `GoalStatus` の値を表で持つ。既存 Python 版の挙動（`navigate_to_pose` を waypoint ごとに順次、`followWaypoints` は使わない）を**厳密に合わせる** |
| R-2 | **onnxruntime-vendor の CMake 連携が不明**（`CMAKE_PREFIX_PATH` に `opt/onnxruntime_vendor` が入るか、`find_package(onnxruntime)` が通るか） | Phase 2 着手時に詰まる | Phase 0 で throwaway プロジェクトで先に確定させる。通らなければ `CMAKE_PREFIX_PATH` に明示追加 + `rosdep` ルールを追加 |
| R-3 | **firmware を ament パッケージ化するのが invasive**。PlatformIO と colcon の二重ビルドになる | Phase 1a | firmware は `install(DIRECTORY include/)` だけ追加し、他の挙動を変えない。`pio test -e native` が PASS することで回帰を確認する。Phase 0 で PoC を作る |
| R-4 | **performance が未検証で動く**。特に vision（onnxruntime C++ API の `intra_op_num_threads` のマッピング）と web_nav（PNG 生成 370ms → OpenCV で変わる） | 実運用性の変化 | 各 Phase で**移行前/後の実測値を取り、`docs/vision-perf-baseline.md` と新しい benchmark を更新**。R10 に従い、確定できるものは「問題なし」、できないものは「✗ 未検証」と書く |
| R-5 | **pyserial の `loop://` モックに相当する仕組みが C++ に無い**。`comm_check` / `hz_measure` のテスト（10 ケース）が書けなくなる | Phase 1 | `SerialIo` 抽象 interface → `FakeSerial`。**C++ 化の段階で interface を先に切る**（interface → 実装 の順） |
| R-6 | **AGENTS.md R9 の「機械で判定できる AI 感」が cpplint では判定できない** | コード品質の低下 | Phase 0 で `check_header.sh`（既に `.omo/session-work/` に存在）に C++ 用の検査を追加：装飾的区切り罫 `// =====`、波括弧平衡、名前空間の開閉、`★要確認★` マーカーの保持 |
| R-7 | **launch 10 個の無変更を保つ損**。`ros2 run` の実行ファイル名が変わると launch が壊れる | 起動壊れ | **実行ファイル名を `serial_bridge` / `web_nav_server` / `detection_node` / `pc_camera` / `hand_landmarks_node` のまま維持**。CMake の `install(TARGETS ... DESTINATION lib/${PROJECT_NAME})` で保証 |
| R-8 | **`ros2_autonomous_nav` の中途半端な `ament_cmake` 化** | Phase 4 | §2.2 で (a)/(b) を選択してから着手 |

---

## 5. 各 Phase 終了時の必須チェックリスト

- [ ] `colcon build --packages-select <対象>` がクリーン
- [ ] `colcon test --packages-select <対象>` が全 PASS
- [ ] `pio test -e native` が PASS（firmware 側の回帰なし）
- [ ] `ros2 run meister_serial_bridge meister_comm_check --port /dev/ttyUSB0` が実機で 3 手順 PASS
- [ ] 実 ROS ノードが起動し、購読/公開のメッセージが移行前と同一
- [ ] `check_header.sh` が PASS
- [ ] 未検証項目は「✗ 未検証」と明記
- [ ] コミットは pure refactor / 機能追加 / 文書 の 3 層に分けられている

---

## 6. 移行後の構成（目標）

```
src/
├── meister_serial_bridge/          # ament_cmake
│   ├── CMakeLists.txt
│   ├── package.xml                 # build_type: ament_cmake
│   ├── include/meister_serial_bridge/
│   │   ├── kinematics.hpp
│   │   └── serial_io.hpp
│   ├── src/
│   │   ├── serial_bridge_node.cpp
│   │   ├── comm_check.cpp
│   │   ├── feedback_hz_measure.cpp
│   │   └── kinematics.cpp
│   ├── include/hal/generated_config.h   # firmware と共有
│   ├── launch/serial_bridge.launch.py   # Python のまま
│   └── test/                            # gtest
├── meister_vision/                # ament_cmake（onnxruntime-vendor）
│   ├── CMakeLists.txt
│   ├── include/meister_vision/
│   │   ├── yolo_detector.hpp
│   │   └── hand_landmark_detector.hpp
│   ├── src/
│   │   ├── detection_node.cpp
│   │   ├── yolo_detector.cpp
│   │   ├── hand_landmarks_node.cpp
│   │   ├── hand_landmark_detector.cpp
│   │   ├── pc_camera_node.cpp
│   │   └── download_model.py         # Python のまま
│   └── test/
├── meister_web_nav/               # ament_cmake（cpp-httplib + nlohmann）
│   ├── CMakeLists.txt
│   ├── include/meister_web_nav/
│   │   ├── http_app.hpp
│   │   ├── map_cache.hpp
│   │   ├── map_listener.hpp
│   │   └── nav_worker.hpp
│   ├── src/
│   │   ├── web_nav_server.cpp
│   │   ├── http_app.cpp
│   │   ├── map_cache.cpp
│   │   ├── map_listener.cpp
│   │   └── nav_worker.cpp
│   ├── webui/                      # 静的資産（変更不要）
│   └── test/
└── ros2_autonomous_nav/           # launch は Python のまま
    └── launch/*.launch.py
```

---

## 7. 未検証項目（R10 に従い「問題なし」としない）

- ✗ **未検証**: `ros-jazzy-onnxruntime-vendor` install 後に `find_package` が通るか（Phase 0 で確定）
- ✗ **未検証**: ONNX Runtime C++ (1.24.3) の Python 版 (1.28.0) との推論結果一致（Phase 2 の実測で確定）
- ✗ **未検証**: OpenCV `cv::dnn::NMSBoxes` と Python の `cv2.dnn.NMSBoxes` の tie-break 挙動の一致
- ✗ **未検証**: `map_cache` の PNG 生成を `cv::imencode` に置き換えたときの 370ms からの変化（Phase 3 の実測）
- ✗ **未検証**: 6輪ロッカーボギー側の `firmware/kinematics.h` の `kGeometryNotFilled` ゲートが、Python から C++ へ移す過程で誤って解除されないこと（Phase 1a の `pio test -e native` で確認）
- ✗ **未検証**: `hand_landmark_detector` の ★要確認★ モデル契約（実モデルが無いので検証不能。C++ 化後も ★要確認★ のまま保持）
