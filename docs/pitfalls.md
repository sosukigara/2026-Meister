# 落とし穴・ノウハウ集 — Meister

> AI コーディングで最も効果を発揮するファイル。過去に実際に踏んだミスを根拠にしている。設計・規約は [docs/architecture.md](architecture.md) / [docs/coding-standards.md](coding-standards.md)。

## P1. 未取得の値を記憶で埋めない (R4)

- web search は全プロバイダ遮断。`web_search` は使えないと前提にする。
- pip は **使える**。`pip download --no-deps feetech-servo-sdk` で公式 SDK を取得できた。
- 取得済みの一次情報は**実際に走らせて** golden を採る。ソースを読むだけだと LENGTH フィールドの定義を 1 バイトずらして理解した。
- 未取得を表す値は到達不能な値（`0xFF` など）にし、`kRegisterMapVerified = false` の間は WRITE 命令をゲートする。
- 検証済みの項目はファイル冒頭の golden バイト列と突き合わせる。
- **隔離するだけでなく、確定までは機能を表さない経路を用意する**（R4 延長）: `kinematics.cpp` の `kGeometryNotFilled` のように、幾何が埋まるまで `Solve()` は必ず失敗し指令を出さない。
- 理由: 推測値で動くコードを書くと、`comm_check` の 3 手順は PASS したままで配線を誤る。「通信は動く = 機構も動く」ではない。過去に `kGripperPin = 0`、誤ったピン値、`axes.arm` を 4→2 にして既存経路を壊す、が起きた。

## P2. 編集が reject / 失敗したら適用済みを確認する (R5)

- 1 つの編集呼び出しに複数 PUT をまとめると、行番号アンカーの 1 つが「表示されていない」として**呼び出し全体が reject** された事例あり。
- 編集ツールが reject したら、**通った PUT も通っていない**疑いを持つ。
- 報告前に `git diff` または `grep -n` で**該当行が実際に変わったか**見る。
- ファイル全体を書き直す方が、個別 PUT を重ねるより安全（行番号 remap で隣り合う行が重複する事故が 3 回）。

## P3. push 前にレビューを 1 巡させる (R6)

- ゲートは「ビルドが通った」ではない。
- 条件コンパイルの分岐が静かに壊れても既定ビルドは緑のまま通る。4 通りのビルドを回す（[docs/commands.md](commands.md) §6）。
- 実機 `comm_check` が 3 手順 PASS するまで commit しない。
- レビュー基準は `.omo/session-work/REVIEW.md` に file:line 付きで書き出し、判定をそこに残す（コンテキスト圧縮で中断されないように）。

## P4. 破壊的変更は 1 箇所の設定に集約してから入れる (R7)

- プロトコルの軸数を変えると C++ / Python / Unity / pytest / ドキュメント / 実機リンクを同時に壊す。
- 軸数を `meister_config.h` の定数から導出させ、実物の確認後に 1 箇所の変更で済むようにする。
- **既に動いているものを壊してまで先に進めるのは禁止。** 破壊的変更（C++ / Python / 検証ツール / テスト / 実機）を 1 コミットで揃える。途中で push すると `origin/main` が「どれとどれが相互運用できるか分からない」状態になる。

## P5. 静的初期化順を当てにしない (R11)

- 同一翻訳単位なら宣言順に構築されるが、**翻訳単位をまたぐと静的初期化順序は未規定**。
- `static Gripper gripper(gripperServoChannel)` が未構築の channel を参照で捕捉する bug が入りうる。コンパイルも `pio test` も通り、**実機で初めて踏む**。
- 対策: 静的グローバルをやめて `setup()` 内で 1 か所に aggregate 構築する。

## P6. 未検証を「問題なし」で終わらせない (R10)

- レビューで判定できない項目は **「✗ 未検証」** と書く。「問題なし」とは書かない。
- 4 状態を区別: 一次情報で確定 / 実測した / 要確認 / ✗ 未検証。

## P7. コミットはレビュー可能な単位に分ける (R12)

- 1 コミットに 1000 行超の refactor + 新 HAL + プロトコル変更 + 文書を入れない。
- 推奨: pure refactor → 機能追加 → 文書・ルール。pure refactor は実機ゲートを通してから commit。

## P13. 作業は worktree で行い、merge してから push する (R13)

- `main` チェックアウト上で直接編集・commit・push しない。作業中に `main` が汚れていると、
  「今どの差分がレビュー対象か」が分からなくなる（R6 のレビュー前提が崩れる）。
- 手順: メインで `git worktree add -b <type>/<topic> .worktrees/<topic> origin/main` →
  worktree 内で編集・ビルド・テスト・commit → メインで `git merge --no-ff <branch>` →
  `git push origin main` → `git worktree remove` + `git branch -d`。
- worktree 内から `git push` しない。feature ブランチは明示依頼が無い限り push しない。
- `.worktrees/` は `.gitignore` 済み。worktree 内のビルド成果物（`build/` `install/` `log/`）は
  メインと別ディレクトリになるため、colcon は worktree 内で `ros/` を cwd にして回す。
- 理由: 複数タスクを並行すると、メインの作業ツリーに別タスクの未 commit 変更が混ざり、
  merge 時に「どの変更がどのレビュー結果に対応するか」を示せなくなる。

## P8. 環境特有の挙動

- **VPN 環境**: `launch.sh:59` で `ROS_LOCALHOST_ONLY=1` を設定するが子プロセスのみ。別ターミナルの `ros2` CLI には継承されない。
- **upload_port**: `platformio.ini` で by-id パスを明示指定。M5Stack ESP32-P4 (`/dev/ttyACM0`) が同時に接続されている状態でポート未指定の `pio run -t upload` が誤ったボードへ書き込むのを防ぐ。別の ESP32 本体に書き込む場合はここを変更。
- **GPIO12 はストラップピン**: グリッパーサーボ PWM ピン (`MSTE_GRIPPER_PIN=12`) は実機配線時に変更推奨。
- **web search 遮断**: 全プロバイダ遮断。pip は使える。

## P9. プロトコル同期の罠

- `protocol.py` は `meister_protocol.h` と**バイト単位で一致**しなければならない。
- 軸数を変えるときは両方直す（`test_protocol.py` / `test_protocol.cpp` が同じ入力で同じ出力を返すことを保証）。
- 現プロトコルは `steering 6ch` / `arm 4ch` / `gripper 1ch` で BOM（ステアリング 4 / アーム肩 PWM 1 + バス 5 / グリッパー 2）と不一致。軸数は `meister_config.h` の定数から導出する方針。

## P10. 未確認値（推測で埋めていない）

- `MSTE_BUS_RESP_TIMEOUT_US = 3000 µs`（データシート未取得）
- `MSTE_BUS_INTER_FRAME_GAP_US = 200 µs`（データシート未取得）
- `reg::eprom::*` / `reg::sram::*` のアドレス（未照合）
- `reg::kStepsPerRev = 4096`（未照合、360° 型かどうか）

一次情報（Feetech 公式 Python SDK 1.0.0、および `docs/datasheets/` の STS3235 仕様書）で確定できたのは
命令コード・フレーム構造・チェックサム・エラービット・ID 範囲・360°/4096 分解能・半二重 8N1・
38400〜1 Mbps（既定 1 Mbps）まで。レジスタアドレス表は仕様書に載っておらず未照合のまま。
実機のサーボは STS3225 で、手元にあるのは STS3235 の仕様書。**型番をまたいで値を流用しないこと。**

## P11. 文字化けスキャンの罠

- 判定範囲は 3 個に限る: (1) 日本語の直後に英単語 3 文字以上、(2) ハングル、(3) U+FFFD。
- 日本語の漢字も CJK 範囲 `[\x{4E00}-\x{9FFF}]` に含まれるので、CJK 範囲で判定すると正常な日本語が全滅する（`umeonigiri_webnav` で実際に全滅した）。

## P12. ROS 2 ノード名・購読の罠（2026-11-11 実測）

- launch の `Node(..., name='X')` は `-r __node:=X` になり、プロセス内手作りノードの**明示名も上書き**する。1 実行ファイル多ノード（`umeonigiri_webnav_server` 内の map/vision/commander）で `name` を付けると 3 ノードが同名 3 重登録になり、`ros2 node info` が「3 nodes with the exact name」と言う。対策: 多ノード実行ファイルの launch には `name` を付けない（`umeonigiri_webnav.launch.py`）。
- `create_subscription` の戻りを捨てると Jazzy では購読が破棄される。「Node が保持する」は誤り。`vision_listener`（メンバ保持、生存）と `map_listener`（破棄、死亡）で実証済み。購読はメンバで持つ。
- 実機に odom 発行源がないと SLAM の message filter が `/scan` を全捨てする（`discarding message because the queue is full`）。対策: `real_robot.launch.py` の static `odom->base_footprint`（不動仮定。ずれは SLAM の `map->odom` が補正）。ESP32 odom ができたら置き換える。
- 実機 launch の SLAM 起動は固定秒数 `activate` 禁止。`configure` 前に走ると両方 exit 1 で SLAM が `unconfigured` のまま固まる。`slam.launch.py` と同じ `OnStateTransition(inactive)` 駆動にする。

## P13. Gazebo の lidar が壊れる機体と、その回避策（2026-11-20 実測）

- 症状: `/scan` の全点が **0.092〜0.130 m の一定値**になる（障害物の有無によらず）。
  `/map` が十数セルで停滞し、`/pose` が出ず、ゴールは
  `Goal Coordinates ... outside bounds` で失敗する。
  **`ros2 topic hz /scan` は正常に流れて見える**ので、レートだけで判断しないこと。
  `ranges` の中身を見る。
- 原因: この機体の Mali-G610 (Panfrost) は **GL 3.3** しか出さず、
  Ogre2 のシェーダが必要とする SSBO は **GL 4.3 以降**。結果
  `~/.gz/rendering/ogre2.log` に
  `error: Too many vertex shader storage blocks (1/0)` が出て、深度バッファが壊れる。
- **回避策（有効・実測）**: ソフトウェア描画に切り替えると GL 4.5 が出て解決する。
  ```bash
  LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe ./launch.sh --sim maze
  ```
  `launch.sh` はシミュレーションモードで既定これを設定する
  （`MEISTER_GL=hardware` で無効化）。実 GPU のある機体では無効にすること。
- CPU レイ lidar への変更（`type="lidar"` / `type="ray"`）は **無効**。
  gz-sim 8 では CPU レイの lidar が 1 点も出さない（実測）。
- Vulkan（`--render-engine-api-backend vulkan`）も不可。ICD が無く、
  `mesa-vulkan-drivers` は lavapipe を含まない。
- ソフトウェア描画は重いので、センサを軽くする `sensor_profile:=lite`
  （lidar 360 点/5 Hz、カメラ 320x240/5 Hz）を併用する。既定は `full`。
  `maze` で実時間比 0.89、`warehouse` はメッシュが重く 0.2 程度。
- CPU の比較は `/proc/<pid>/stat` の utime+stime を区間で積分する（`top` の瞬間値は振れる）。
  シミュレーションの CPU 値は `/clock` の配信量に比例して膨らむので、実機の見積りに流用しない。

## P14. 画像認識のレートはカメラで決まる（2026-11-20 実測）

- 認識ノードを軽くしても、**カメラが 1〜11 Hz しか出ない環境ではレートは上がらない**。
  まず `ros2 topic hz /camera/image_raw` と `v4l2-ctl --list-formats-ext` を見る。
- つないでいるカメラが **YUYV のみ・露出制御なし** だと、暗所で自動露出が
  長秒側に張り付いて **1 Hz まで落ちる**。解像度を下げても改善しない（実測）。
- MJPEG 対応機なら `pixel_format`（既定 MJPEG）で 640x480 のまま 14.8 Hz が出る。
  YUYV 専用機では `CAP_PROP_FOURCC` を要求しても YUYV のままで、これは正常。
- 検出がカメラに追随しているかは、`record_detections.py` のフレーム数と
  `ros2 topic hz /camera/image_raw` を並べれば判定できる（同数なら取りこぼし無し）。

## P15. シミュレーションで AMCL が初期化できない（2026-11-20 実測）

- 症状: `/api/mode` で localization に切り替えても
  `AMCL cannot publish a pose or update the transform. Please set the initial pose...`
  が続き、`/api/nav` が即座に `failed` になる。
- 原因: `umeonigiri_webnav_server` のプロセス内ノードが **`use_sim_time` を持たない**ため、
  `/api/initialpose` が実時間でスタンプされる。ログには
  `Requested time 14.900 but the earliest data is at time 69.160` のように
  シミュレーション時刻とのずれが出る。AMCL のメッセージフィルタが一致せず、
  `map→odom` が永遠に張られない。
- 対策: `umeonigiri_webnav.launch.py` に `use_sim_time` 引数を足し、
  `mapping_nav.launch.py` から `true` を渡す（実機は既定の `false`）。
  多ノード実行ファイルでもコマンドラインのパラメータ上書きは全ノードに効く。
- **もう 1 段の罠**: `use_sim_time` を直しても、`/api/initialpose` は
  `SetInitialPose` が**1 回だけ publish** するため、モード切替直後で AMCL の
  購読がまだ張られていないと捨てられ、同じ警告のまま固まる（実測で再現。
  lite では通って full では落ちた＝タイミング依存）。0.5 秒かけて 10 回
  流すように直した（`mode_worker.cpp`）。
- 確認: 切り替え後に `/api/pose` が設定した座標付近を返せば初期化できている。
  `-4.0, -4.0` が**そのまま**返るなら初期化できていない（AMCL が補正していない）。

## P16. HTTP ハンドラを秒単位で待たせると、画面全体が無応答になる（2026-11-20 実測）

- 症状: Web UI を開いたまま 30 秒ほどで、地図 (`/api/map`) も姿勢 (`/api/pose`) も
  **タイムアウトして返らなくなる**。サーバーのスレッドは 25 本すべて `futex_wait` で
  止まっていた。ブラウザ側では `/api/mode` が 13 回すべて 4999 ms（`AbortSignal` の上限）
  で失敗していた。
- 原因: `/api/mode` が `get_mode` サービスを**呼び出しごとに新しいクライアントで探索**し、
  見つからない構成（`nav_mode_manager` 無し）では 2 秒探索 + 3 秒待ちの 5 秒を
  1 リクエストで消費していた。UI は 2 秒間隔で引くので待ち行列が伸び続け、
  8 本のプールが全部その待ちで埋まると、無関係な要求も通らなくなる
  （httplib は 1 接続 1 スレッド。プールが埋まった時点で受付が止まる）。
- 対策: (1) モード照会は `service_is_ready()` だけ見て、居なければ即 `unknown` を返す。
  (2) `get_mode` / `switch_to_localization` / `map_saver` / `/initialpose` を扱う
  `ModeWorker` は**専用 executor スレッドで常に回す**。HTTP 側は `spin` せず
  `future.wait_for(上限)` で待つだけにした。(3) UI のモード入力も
  `polling` フラグで多重要求を止める（地図・姿勢・認識は元から止めていた）。
- 確認: 40 本の `/api/mode` を同時に投げても 137 ms で全部返り、直後の `/api/map` は 1 ms。
- 教訓: **ポーリングされる API に「見つからなければ数秒待つ」を入れてはいけない**。
  待つのは人間が押した操作（切替・保存）だけにする。

## P17. httplib の既定設定で、同じ接続の 2 回目以降が 40 ms 遅れる（2026-11-20 実測）

- 症状: ポーリング API の p50 が 42 ms。`curl`（毎回新しい接続）では 0.7 ms なので
  気づきにくい。Python の keep-alive クライアントでは 1 本の接続で
  **5 回に 4 回が 42 ms**、5 回目（`Connection: close`）だけ速かった。
- 原因: cpp-httplib 0.14.3 は `CPPHTTPLIB_TCP_NODELAY` 既定 false。応答ヘッダと本文を
  別々の `send` で書くため、Nagle と delayed ACK が噛み合って 40 ms 待つ。
  さらに `CPPHTTPLIB_KEEPALIVE_MAX_COUNT` 既定 5 で、5 要求ごとに TCP を張り直す。
- 対策: `Server::set_tcp_nodelay(true)` / `set_keep_alive_max_count(1000)` /
  `set_keep_alive_timeout(10)`（`http_app.cpp` の `ConfigureServer`）。
  いずれも実行時 API なので、配布物の共有ライブラリでも効く。
- 実測: 同一接続 200 連続で p50 42.6 ms → 0.44 ms、`/app.js` 42.2 ms → 1.3 ms。
  `Keep-Alive: timeout=10, max=1000` になり、12 連続でも接続は切れない。
- 注意: `CPPHTTPLIB_LISTEN_BACKLOG` は**マクロなので効かない**。この環境の
  `libcpp-httplib` は共有ライブラリとして配布されており、`listen_internal()` は
  そちらに焼き込まれている。受付待ち行列を変えたいなら自前ビルドが要る。

## P18. 起動直後に `nav_mode_manager` が落ちて `/map` が出ない（2026-11-20 実測）

- 症状: シミュレーションを起動すると `nav_mode_manager` が
  `Node '/nav_mode_manager' has already been added to an executor.` で
  `abort`（`process has died, exit code -6`）。モード管理が死ぬので
  `slam_toolbox` が configure されず、Web UI は「地図を待っています」のまま。
  Nav2 の `global_costmap` も `Received map message is malformed` を出し続ける。
- 原因: `ApplyTransition()` のサービス探索ループが `rclcpp::spin_some(shared_from_this())`
  を呼んでいた。この Node は `main()` で `MultiThreadedExecutor` に載っているので、
  `spin_some` 内部の `executor.add_node()` が例外を投げる。起動 2 秒の BringUp 時点で
  `/slam_toolbox/change_state` がまだ見えていないと必ずここへ入る（＝毎回ではなく競合）。
- 対策: 探索ループを消し、`wait_for_service()` で待つだけにした。探索は
  MultiThreadedExecutor の他スレッドが進める。
- 教訓: **executor に載っている Node を `spin_some` してはいけない**。
  `spin_some` は「その場で executor を作って add_node する」実装なので、
  多重登録として弾かれる（同じ罠を webnav の `ModeWorker` からも外した）。
- 確認: 同じ maze ワールドで、修正前は起動のたびに落ち（1/1）、修正後は落ちずに
  `/api/mode` が `mapping`、`/api/map` が slam_toolbox の地図（例 71x69,
  origin -4.93/-4.91）を返した（1/1）。empty ワールドでは修正前も落ちなかった
  ので、これは起動順の競合であって常時再現ではない。★要確認★ 修正後に複数回の
  起動を繰り返した統計はまだ無い。

## P19. シミュレーションの画像認識が既定の NPU 版で死ぬ（2026-11-22 実測）

- 症状: `./launch.sh --sim maze` は起動するが `/detections` が一度も流れない。
  ログに `yolo_grasp_node ... exit code 1` と
  `RKNN モデルを読めません: .../models/yolov8n-zoo.rknn`。
- 原因: `mapping_nav.launch.py` が `pc_vision.launch.py` を引数なしで include して
  いたため、実機用の既定 `backend:=npu` が選ばれていた。
  NPU 版 (`yolo_grasp`) は `/dev/video0` を V4L2 で直接開く実装で、Gazebo が
  ROS トピック `/camera/image_raw` に出す映像を読めない。さらに `*.rknn` は
  gitignore 対象なので、worktree / fresh clone にはモデルが無い。
- 対策: シミュレーションでは `backend:=onnx`・`use_pc_camera:=false`・
  `model_path:=.../yolov8n-320.onnx` を明示する。ONNX モデルが無い場合は
  検出ノードを起動せず、`ros2 run umeonigiri_vision download_model` の案内だけを
  ログに出す（落ちるプロセスを増やさない）。
- スレッド数: ONNX を 4 スレッドにすると gz(llvmpipe) と CPU を食い合う。
  maze/lite/320 の実測で RTF 0.828（vision 無し）→ 0.641（4 スレッド）→
  0.680（2 スレッド）だった。2 スレッドでも検出は 3.8 Hz（カメラ 4.4 Hz）で
  追従し、検出ノードの CPU は 1.03 → 0.55 コアに下がる。
- 確認: `detection_node` がモデルを読み、`/camera/image_raw` を購読、
  `/detections` 3.7〜3.8 Hz、Web UI の認識モニターが 320×240 を表示（実測）。

## P20. Web UI のパラメータ API で踏みやすい 3 点（2026-11-22 実測）

- **経路の順序**: httplib は登録順に最初の一致を使う。`/.*` の静的配信より後に
  `/api/params` を登録すると、静的配信が先に受けて 404 になる。
  `RegisterRoutes` の中で `RegisterVisionRoutes` と並べて登録する。
- **ノード一覧の API**: `rclcpp::Node::get_node_names_and_namespaces()` は
  Jazzy の `rclcpp::Node` に無い。`get_node_names()` を使う。グラフには
  `/_ros2cli_daemon_*`・`/launch_ros_*`・`/transform_listener_impl_*`・
  `*_rclcpp_node` も載るので、画面の選択肢からは外す。
- **型を UI から決めない**: 設定前に `/get_parameters` で既存の
  `ParameterValue.type` を取り、その型へ変換する。UI の JSON 型をそのまま
  `SetParameters` に入れると、double のつもりが integer に化ける。
- スレッド: 呼び出しは HTTP から来るので、`ModeWorker` と同じく専用 executor で
  回し、`spin_some` は呼ばない。探索は `wait_for_service(200ms)`、応答は
  `future.wait_for(2s)` で上限を切る。
- 確認: Web UI で `FollowPath.desired_linear_vel` を 0.5 → 0.7 に変更し、
  `ros2 param get` が 0.7、同じ設定のまま Nav2 を走らせた `/cmd_vel` の最大
  linear.x が 0.700（実測）。型違いは 409、画面には理由を表示。

## P21. 保存地図はソースツリーではなく install 側に書かれる（2026-11-22 実測）

- `umeonigiri_webnav.launch.py` は `maps_dir` に
  `install/.../umeonigiri_autonomy/maps` を渡す。`--symlink-install` でも
  ディレクトリ全体は symlink にならず、既存の `nav_world.*` だけがソースへの
  symlink。`/api/map/save` で新規に保存したファイルは install 側の実ファイルに
  なる（実測: `launch_test_tmp.png/yaml` は
  `ros/install/umeonigiri_autonomy/share/umeonigiri_autonomy/maps/` にでき、
  `ros/src/.../maps/` にはできなかった）。
- 影響: **★要確認★** `rm -rf install` や clean build で消える。ソースに残したい
  場合は `maps_dir` の設計を見直す必要がある。今は「保存 → 一覧 → 削除」は
  同じ install 側で閉じているので動作はする。
