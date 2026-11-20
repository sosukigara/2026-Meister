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

## P8. 環境特有の挙動

- **VPN 環境**: `start_meister.sh:44-48` で `ROS_LOCALHOST_ONLY=1` を設定するが子プロセスのみ。別ターミナルの `ros2` CLI には継承されない。
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

一次情報（Feetech 公式 Python SDK 1.0.0）で確定できたのは命令コード・フレーム構造・チェックサム・エラービット・ID 範囲のみ。STS3215 のデータシートで照合すること。

## P11. 文字化けスキャンの罠

- 判定範囲は 3 個に限る: (1) 日本語の直後に英単語 3 文字以上、(2) ハングル、(3) U+FFFD。
- 日本語の漢字も CJK 範囲 `[\x{4E00}-\x{9FFF}]` に含まれるので、CJK 範囲で判定すると正常な日本語が全滅する（`meister_web_nav` で実際に全滅した）。

## P12. ROS 2 ノード名・購読の罠（2026-11-11 実測）

- launch の `Node(..., name='X')` は `-r __node:=X` になり、プロセス内手作りノードの**明示名も上書き**する。1 実行ファイル多ノード（`web_nav_server` 内の map/vision/commander）で `name` を付けると 3 ノードが同名 3 重登録になり、`ros2 node info` が「3 nodes with the exact name」と言う。対策: 多ノード実行ファイルの launch には `name` を付けない（`web_nav.launch.py`）。
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
- 計測の詳細: [bench/2026-11-20-cpu-analysis.md](bench/2026-11-20-cpu-analysis.md) §7・§10。

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
- 原因: `web_nav_server` のプロセス内ノードが **`use_sim_time` を持たない**ため、
  `/api/initialpose` が実時間でスタンプされる。ログには
  `Requested time 14.900 but the earliest data is at time 69.160` のように
  シミュレーション時刻とのずれが出る。AMCL のメッセージフィルタが一致せず、
  `map→odom` が永遠に張られない。
- 対策: `web_nav.launch.py` に `use_sim_time` 引数を足し、
  `mapping_nav.launch.py` から `true` を渡す（実機は既定の `false`）。
  多ノード実行ファイルでもコマンドラインのパラメータ上書きは全ノードに効く。
- 確認: 切り替え後に `/api/pose` が設定した座標付近を返せば初期化できている。
