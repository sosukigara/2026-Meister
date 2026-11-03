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
