# 設計: ナビゲーション最適化・地図運用・実機運動学

> 対象: `ros/src/ros2_autonomous_nav` / `ros/src/meister_web_nav` / `meister_serial_bridge` / `firmware/main_board`
> 作成日: 2026-11-18 | ステータス: レビュー待ち（未承認）

## 1. 背景と意図

自動走行の質を上げ、同時に消費計算資源を下げたい。調査の結果、現状には次の構造的な問題がある。

1. **自己位置推定が走行中ずっと SLAM と兼用になっている。** シミュも実機も `slam_toolbox` の `mode: mapping` で起動し、走りながら地図を作り続ける（`config/mapper_params_online_async.yaml:16`, `config/real_mapper_params_online_async.yaml:16`）。経路計画が参照する地図が走行中に変わる。
2. **「保存地図で自己位置推定する」経路が存在しない。** AMCL と map_server を起動するのは `launch/robot_nav.launch.py` だけで、これは `run.sh` ← `start_nav.sh` ← 呼び出し元なし、の死んだ経路にある。両モードの yaml に `amcl` ブロックと `map_server` ブロックは書かれているが、起動するノードがない（`config/mapping_nav_params.yaml:1-46, 245-248`, `config/real_nav2_params.yaml:1-46, 231-234`）。
3. **実機の速度指令が車輪に届かない。** ESP32 は `CMD_MOTOR_VELOCITY` を受理して破棄する（`firmware/main_board/src/command_dispatch.cpp:92-95`）。`/odom` は指令速度の積分＋BNO055 ヨーで作られており（`ros/src/meister_serial_bridge/include/meister_serial_bridge/ackermann_imu_odometry.hpp:104-124`）、エンコーダではない。Nav2 は「動いたつもり」のオドメトリ上を走る。
4. **実機の運動学が自転車モデルに退行している。** ブリッジが1つの舵角を `atan(vx·wz/L)` で作って6輪に同値を埋める（`ros/src/meister_serial_bridge/src/kinematics.cpp:26-66`）。実機は 6輪駆動・4輪操舵であり、この式は機構と一致しない。
5. **計算量の内訳が測られていない。** 唯一の実測記録は別の制御器（当時の DWB）に対するもので、制御ループが設計 20 Hz に対し実測 5–16 Hz だった原因も未解明のまま残っている（`docs/debug/webnav-stall-20260825.md:70-71`）。

## 2. ゴール / 非ゴール / アンチゴール

**ゴール**
- 地図生成と、保存地図上での自己位置推定の両方が、Web UI から操作できる
- 同じ条件で繰り返せるナビゲーション評価基盤を持ち、変更の効果を数値で示せる
- 上記を満たしたまま、Nav2 の消費計算資源を現状より下げる
- 実機で速度指令が車輪に届き、機構に合った運動学で走る

**非ゴール（今回やらない）**
- 自動探索によるマップ生成の自動化（設計に余地を残すだけ）
- アーム・グリッパー制御
- 画像認識（`meister_vision`）の変更
- メカナム／オムニ化などのハードウェア改造

**アンチゴール（これをやったら失敗）**
- 「速くなった気がする」で終わり、比較可能な数値が残らない
- シミュ用と実機用の設定が混ざり、どちらで何が動いているか分からなくなる
- 未測定の値を推測で埋めて、動かない原因を隠す

## 3. 制約

**ハードウェア（一次資料）**
- 実機: 6輪駆動・4輪操舵（前2・後2）、中央2輪は固定。`x = ±0.30 / 0`、`y = ±0.25`、`wheel_radius: 0.05`（`ros/config/meister_robot.yaml:111-123`）。幾何は `geometry_filled: false` で**未計測**。
- 舵角サーボの実数は BOM 上 4 個（`ros/config/meister_robot.yaml:97`）。ファームウェアの現行定義は 6ch（`firmware/main_board/include/hal/generated_config.h:18, 55, 59`）。**★要確認★**
- 駆動は FOC 基板（PWM 直駆動は廃止、`generated_config.h:53`）。エンコーダは無い。
- **メインボードから駆動モータへの経路が存在しない。** `BaseChassis` は舵角サーボ専用（`firmware/main_board/include/base_chassis.h`）、速度指令は破棄され（`command_dispatch.cpp:92-95`）、FOC ボード3枚（各2ch、それぞれ独立した ESP32）は USB シリアルのコンソールファームで、メインボードとの接続コードが無い（§6.6）。
- LiDAR: LD-D500、720 bins、0.03–12.0 m、10 Hz（`ros/src/ros2_ldlidar_d500/config/d500.yaml:3-15`）。
- シミュの車体は車輪を持たない自由飛行ボディ（`ros/src/umeonigiri_description/urdf/chassis.xacro`, `gazebo.xacro:7-11`）。つまりシミュはホロノミックで、実機より能力が高い。

**計算資源**
- 開発機: Intel i3-12100F（4コア/8スレッド）、31 GB、GTX 1050 Ti。Nav2 は GPU を使わない。
- 将来の搭載機: Orange Pi 5 Max（`docs/vision-perf-baseline.md`）。
- 計測は「GUI あり（`gz` + RViz）」の同一条件で行い、**変更前後の差分**を主指標とする。絶対値は環境依存として参考記録に留める。

**既存の合意事項（変更しない）**
- `vx, vy, wz` の3成分を PC から送る（`docs/superpowers/specs/2026-09-27-command-and-kinematics-design.md` D2）
- `vy` の非ゼロは `FB_ERROR` で拒否（同 D5）
- 展開は ESP32 側（同 D1）、コマンドは1つの型に集約（同 D4）

## 4. 検討したアプローチ

### 4.1 自己位置推定: AMCL / EMCL2 / slam_toolbox localization

| 案 | 判定 | 理由 |
|---|---|---|
| **AMCL（採用）** | ○ | 実測で1コアの 10–20%（Ekumen 2024）。lifecycle 管理下に置ける。粒子拡散がオドメトリ誤差を吸収する |
| EMCL2 | ✗ | ROS 2 移植版は存在し、本機の Jazzy でビルド・起動まで確認済み。ただし lifecycle 非対応（Q12 の設計に載らない）、デフォルトは360ビーム×500粒子を20 Hzで回しAMCLより重い、AMCL比の優位性を示す公開ベンチマークが無い。誘拐復帰が主課題になった時に再検討 |
| slam_toolbox localization | ✗ | ポーズグラフ（`.posegraph`）が必要で `.pgm` 地図を使えない。作者自身が「高品質なオドメトリが要る。初心者には AMCL を勧める」と明記。本機のオドメトリは指令積分＋IMUでこれに該当しない |

### 4.2 経路計画: SmacPlanner2D / Navfn

**SmacPlanner2D を維持（採用）**。12×12 m の地図（≤16万セル）では計画時間 5–50 ms で、1 Hz 運用に対して無視できる。Navfn は footprint を考慮せず、狭所で劣化する。実機用に SmacPlannerHybrid を用意するのは実機の旋回半径が確定してから。

### 4.3 追従: RPP / DWB / MPPI

**RPP を基準点とし、MPPI（DiffDrive）を比較対象とする。**
RPP は測定上 1 kHz 超で実質ゼロコスト、経路追従の忠実度は最も高い。一方で動的障害物回避の品質は MPPI が上（Nav2 公式チューニングガイド）。MPPI は 4世代 i5 で 50 Hz 以上という公表値があり、本機でも成立する見込み。**まず RPP で基準値を取り、差し替えて差分を測る。**

導入時に必要な設定（インストール済み Nav2 1.3.13 の実測デフォルト）：

- `motion_model` のデフォルトは **`DiffDrive`**。本機（`vy = 0`）はこれで正しい
- `batch_size` 1000、`time_steps` 56 ＝ 1サイクル 56k ロールアウト。20 Hz で回すなら `time_steps` を 30–40 に落とす
- **`critics` のデフォルトは空**で、明示しないと configure が失敗する。最小構成は PathAlign / PathDist / Goal / GoalAngle / Obstacles / Constraint
- `visualize: false` を明示する（true は大幅な速度低下を招く）

> 参考: `ros/src/umeonigiri_description/config/nav2_params.yaml:101` の MPPI 設定は `batch_size: 2000` × `time_steps: 56` ＝ **112k ロールアウト/サイクル**で、しかも `visualize: true`。これは死んだスタック用の設定であり、**復活させない**。

DWB は vy を出せるが、公表された CPU 値が無く、Ackermann/4WS 用のジェネレータも無いため今回は比較対象から外す。

### 4.4 地図運用: SLAM 併走 / 事前地図 + AMCL

**事前地図 + AMCL（採用）**。走行中の地図変動が無くなり、計算も粒子フィルタだけになる。地図生成は独立したモードとして残し、Web UI から切り替える。

## 5. 確定した決定

| # | 決定 | 根拠 |
|---|---|---|
| D-1 | 走行方式は **4WS 差動**（`vy = 0`、その場旋回可、横移動不可） | 中央固定輪が横速度を拘束するため（§6.1） |
| D-2 | 自己位置推定は **AMCL**（`likelihood_field`, `robot_model_type: nav2_amcl::DifferentialMotionModel`） | §4.1。AMCL が持つモデルは Differential と Omni のみで Ackermann は存在しない。本機は 4WS 差動（`vy = 0`）なので Differential が正しい |
| D-3 | 経路計画は **SmacPlanner2D** を維持 | §4.2 |
| D-4 | 追従は **RPP を基準、MPPI DiffDrive を比較対象** | §4.3 |
| D-5 | 地図生成モードと保存地図モードの**両方**を実装し、**Web UI から操作**する | 要求 |
| D-6 | モード切替は **lifecycle の activate/deactivate** で行い、slam_toolbox と AMCL を**排他**にする | TF を途切れさせない |
| D-7 | 地図名はユーザー入力。空欄なら `<world>_YYYYMMDD-HHMMSS`。上書きは確認 | ワールドと地図の対応を保つ |
| D-8 | 起動時は**地図生成モード**で開始する | 運用の素直さ |
| D-9 | **ROS は速度（`vx`, `wz`）を送るだけ。展開は ESP32** | 既存合意 D1/D2 |
| D-10 | 既存の `CMD_MOTOR_VELOCITY` / `CMD_STEERING_ANGLE` を**廃止**し、`CMD_TWIST` 1本に集約 | 両立は「どちらが効くか」の曖昧さを生む |
| D-11 | `vy` 非ゼロは **`FB_ERROR` で拒否**（維持） | 既存合意 D5 |
| D-12 | `use_rotate_to_heading` は**シミュで有効**、実機はソルバ完成後に有効化 | その場旋回は機構的に可能（§6.1） |
| D-13 | 実機の `/odom` 改善は**まず実測してから**判断 | 破綻の程度が不明 |
| D-14 | Nav2 の composition 採用は**計測後**に判断 | 効果と複雑さのトレードオフ |
| D-15 | 自動探索は**今回スコープ外**（設計に余地を残す） | 地図運用が先 |

## 6. アーキテクチャ

### 6.1 実機の運動学（4WS 差動）

```
        y=+0.25        y=-0.25
x=+0.30  [前左・操舵]   [前右・操舵]
x= 0.00  [中左・固定]   [中右・固定]
x=-0.30  [後左・操舵]   [後右・操舵]
```

中央の固定輪は横方向に転がれないので、その接地点の速度は車体 x 方向に限られる。剛体速度 `v(x,y) = (vx − ω·y, vy + ω·x)` に中央輪（`x = 0`）を代入すると `vy` が残るため、**`vy = 0` が拘束される**。結果：

- 前後移動: 可
- **その場旋回: 可**（中央輪の速度は `(−ω·0.25, 0)` で x 方向のみ＝横滑りしない）
- 横移動・斜め移動: 不可

したがって実機は「差動相当＋4WS による滑り低減」であり、スキッドステアより滑りが少なくオドメトリに有利。**目標とする車体ツイストは `(vx, 0, ω)`。**

### 6.2 ファームウェアの拘束ソルバ

**既存の `include/kinematics.h` / `src/kinematics.cpp` を拡張する（置き換えない）。** ここには既に D3（拘束ソルバ＋ボギー位置表）の骨組みと native テストがあり、`Solve()` は幾何未確定の間 `kGeometryNotFilled` を返す設計になっている（112 + 221 行）。捨てると、テスト済みの制約が失われる。

追加するのは、ボギーが直立している行に対する解析解：

- 旋回中心 `y₀ = vx/ω`
- 舵角 `θᵢ = atan2(xᵢ, y₀ − yᵢ)`（操舵輪のみ。固定輪の向きは x のまま）
- 輪速 `vᵢ = ω · √((yᵢ − y₀)² + xᵢ²)`（6輪すべて。中央輪の向きは x 方向で固定のまま整合する）
- `|vx| < ε` かつ `ω ≠ 0` のときは `y₀ → ±∞` の極限（その場旋回）として解く

ボギー軸が傾いた行では舵角の可動範囲が `±90° − ロッカー角` に狭まるため、既存の `WheelWithinLimits()` の判定がそのまま拘束になる。つまり**ロッカーは舵角の自由度を奪う形で効き、舵角の決定は表の行ごとに変わる**。

ボギー位置表の解釈自体が `★未確認★`（`ros/config/meister_robot.yaml:129`）なので、表が確定するまで `geometry_filled: false` を維持し、指令を一切出さない。ロッカーアクチュエータ（DS サーボ4軸）は BOM に含まれるため**概念は残す**。

### 6.3 処理の流れ（実機）

```
Nav2 controller (20 Hz) → /cmd_vel (vx, wz)
  → meister_serial_bridge: CMD_TWIST 1フレーム（vx, vy=0, wz）
  → メイン ESP32: 拘束ソルバ
        ├→ 舵角 4ch（サーボ PWM）
        └→ 輪速 6ch を FOC ボード3枚へ UART ブロードキャスト（§6.6）
  → FOC ボード: 速度閉ループ（AS5600）→ ホイール
  → FOC ボード → メイン ESP32: 実測輪速
  → FB_STATE (100 Hz) → /esp32/state（実測輪速を含む）
  → /odom（指令積分＋IMUヨー。将来は実測輪速に置換）→ AMCL が map→odom を補正
```

### 6.4 モード切替と排他

起動時に map_server / AMCL / slam_toolbox をすべて立ち上げ、lifecycle で切り替える。

| モード | active | inactive | Nav2 |
|---|---|---|---|
| 地図生成 | slam_toolbox (mapping) | AMCL, map_server | deactivate |
| 保存地図でナビ | AMCL, map_server | slam_toolbox | active |

- **排他**: `map→odom` を publish してよいのは常に1つ。切替時に旧ノードの `deactivate` 完了を待ってから新ノードを `activate` する。これを検証するテストを置く。
- **切替は一方通行（地図生成 → 保存 → 保存地図モード）**。地図生成に戻る場合は launch を再起動する。理由: `slam_toolbox` を再 activate したときに既存のポーズグラフを継続するか捨てるかが仕様として定まっておらず、その挙動に依存した設計は検証できない。双方向切替は将来の課題（§11）。
- **AMCL は初期位置が未設定の間、自己位置未確定として扱う**。Web UI の `POST /api/initialpose` を受けるまでゴール送信を受け付けない。固定の `initial_pose`（現行の死んだ設定にある `(-4,-4,0)`）は使わない。
- 地図生成中の Nav2 停止は、将来の自動探索で必要になった時点で見直す。

### 6.5 Web UI API

既存の `POST /api/nav`, `POST /api/cancel`, `GET /api/status`, `GET /api/pose`, `GET /api/map` に加える。

| API | 用途 |
|---|---|
| `POST /api/mode` `{mode: "mapping" \| "localization"}` | モード切替 |
| `GET /api/mode` | 現在のモードと切替状態 |
| `POST /api/map/save` `{name?}` | 地図保存。空欄なら `<world>_<timestamp>` |
| `GET /api/maps` | 保存済み地図の一覧 |
| `POST /api/map/select` `{name}` | 使用する地図の指定 |
| `DELETE /api/maps/{name}` | 地図の削除 |
| `POST /api/initialpose` `{x, y, yaw}` | 保存地図モードの初期位置指定 |

保存形式は `maps/<name>.yaml` + `<name>.pgm`（`nav2_map_server` の標準形式）。上書き時は UI 側で確認を出す。

### 6.6 駆動リンク（メインESP32 ↔ FOC ボード）

**現状この経路は存在しない。** `base_chassis` はサーボ専用（`base_chassis.h` に「駆動は foc drive 基板」）、`command_dispatch.cpp:92-95` は速度を捨てる、`foc_rover` は1枚2chのスタンドアロン・コンソールファーム（USB シリアルで人間が叩く前提）で、メインボードとの接続代码は無い。

設計:

- ローバーは FOC ボード3枚（前・中・後）× 2ch = 6輪。
- メイン ESP32 → 各 FOC ボードへ **1本の UART ブロードキャスト**（1 Mbaud 以上）。フレームに軸IDと左右速度を含め、各ボードは自分の軸IDの分だけ適用する。
- 送信は 100 Hz（既存 `control_hz` と同じ）。無通信 500 ms で FOC 側が 0 に落ちるデッドマンは `foc_rover` の既存ハートビート機構を流用する。
- フィードバックはポーリング。メインからの要求フレームに各ボードが自分のスロットで応答し、衝突を避ける。
- ★実配線（UART 本数、ピン、レベル変換の要否）は要確認★。ESP32 の空き UART は2本なので、3枚を1本のバスに載せる前提が崩れると配線からやり直しになる。

**AS5600 が各 FOC ボードに載っている**（`firmware/foc_rover/README.md`）ため、実測輪速を返せる。これは「指令が車輪に届いた」ことを証明できる唯一の一次データであり、将来のオドメトリ改善（`/odom` を指令積分から実測ベースへ）にもそのまま使える。

## 7. プロトコル変更

- `CMD_TWIST` を追加: `{int16 vx, int16 vy, int16 wz}`（単位は既存の千分率系に合わせる）
- `CMD_MOTOR_VELOCITY` / `CMD_STEERING_ANGLE` を削除
- `PayloadSize` / `ParseResult` の switch に1ケース追加、削除分を落とす
- 同時に修正するファイル: `firmware/main_board/include/meister_protocol.h`、`ros/src/meister_serial_bridge/include/meister_serial_bridge/frames.hpp`、`src/frames.cpp`、`src/bridge_core.cpp`、`src/kinematics.cpp`（Ackermann 変換を削除）、`firmware/main_board/src/command_dispatch.cpp`、各テスト
- `vy` 非ゼロはファームウェアで `FB_ERROR` を立てて拒否（D-11）
- フィードバックに **実測輪速（6ch）** を追加する。現行 `FB_STATE` の「エンコーダ」配列は時間から作った擬似値（`firmware/main_board/src/feedback.cpp:21-25`）で、実測ではない。FOC ボードの AS5600 から取得した値を載せる（§6.6）

> 注: `protocol.py` は現存しない（過去に存在したが削除済み）。AGENTS.md の「C++ と Python を同時に直す」は現状のコードベースには該当しない。

## 8. 計測と評価

### 8.1 合格基準

| 項目 | 基準 | 定義 |
|---|---|---|
| 成功率 | 30/30 | `goal_checker` が SUCCEEDED を返した回数 |
| 到達精度 | xy < 0.15 m、yaw < 0.15 rad | 終了時点の `/odom`（シミュでは真値）とゴールの差 |
| 経路品質 | ≤ 基準の 1.2 倍 | 走行軌跡の累積長 ÷ **同一地図・同一ゴールで SmacPlanner2D が返した経路長** |
| 最小障害物距離 | > 0.10 m | 記録した軌跡の各 pose で、0.6×0.4 の矩形を yaw で回転させた footprint と、地図の occupied セルとの最小距離 |
| 接触 | 0 | 上記 footprint 距離 < 0.02 m となった pose の数 = 0（接触センサは使わない。シミュの接触は publish されないため、幾何で判定する） |
| 制御ループ | 実効 20 Hz、p99 < 60 ms | `/cmd_vel_nav` の連続メッセージ間隔 |
| 計画 | p95 < 100 ms | `NavigateToPose` の planning 開始から `/plan` 受信まで |
| 計算量 | < 1コア換算 60% | 各 Nav2 ノードの `/proc/<pid>/stat` の utime+stime を区間で積分し経過時間で割った値の合計 |

### 8.2 手順

1. 対象ワールドは `warehouse` に固定。ゴールは自由空間から**固定10点**を選び `config/bench_goals.json` に保存する（抽選だと再現できない）。
2. GUI（`gz` + RViz）を出したまま、他プロセスを停止した状態で計測する。
3. 各ゴールを順に実行し、成否・所要時間・経路長・最終誤差・最小距離を記録する。
4. `ros2 topic` のタイムスタンプと `/proc/<pid>/stat` から周期と CPU を取得する。
5. 出力は `docs/bench/YYYY-MM-DD-<label>.json`（生データ）と同 `.md`（表）。

## 9. PR 分割と順序

| PR | 内容 | 合格条件 |
|---|---|---|
| PR1 | 整理: `.sh`/`.py` を `scripts/` へ移動、`README*`・`youtube_description.txt`・動画編集用 `.py` 5本を削除、`launch.sh` の `rviz.sh` パス修正、`package.xml` の maintainer/description 更新 | `./build.sh` が通る |
| PR2 | 計測基盤: `scripts/bench_nav.py`、固定ゴール、CPU/周期計測 | ベースラインが1本の JSON として残る |
| PR3 | パラメータ最適化（計測に基づく4グループ）: 局所化 / 計画 / 追従 / コストマップ | PR2 の基準に対し悪化なし、目標に前進 |
| PR4 | 自己位置推定: AMCL + map_server、モード切替（一方通行）、Web UI API、`real_robot.launch.py` を同じ構成へ移行、不要になった `robot_nav.launch.py` / `slam_map.launch.py` の削除 | 地図生成→保存→切替→初期位置指定→ナビが通しで動く |
| PR5 | 駆動リンク: メインESP32 ↔ FOC ボード3枚の UART ブロードキャスト、デッドマン、実測輪速の返送、運動試験ツール | 低速度指令で6輪が実際に回り、実測輪速が `/esp32/state` に現れる |
| PR6 | プロトコル/ソルバ: `CMD_TWIST` 化、既存 `Solve()` の拡張、`pio test -e native` | native テスト green、`comm_check` が新フレームで通る |
| PR7 | 制御器比較: MPPI DiffDrive の導入と RPP との数値比較 | 採用可否を数値で判断できる |

PR2 以降は PR1 の着地後。PR5・PR6 はファームウェア主体で PR3/PR4 とコードベースが独立しているため並行可能だが、**§10 の★測定が完了するまで着手しない**。

PR4 の「合格条件」には、`docs/features/04-navigation.md` と `docs/design/04-navigation.md` の記述更新（現在は AMCL も DWB も事実と異なる）を含める。

## 10. ★要確認★

| 項目 | 場所 | 確認方法 |
|---|---|---|
| ~~舵角サーボの実数（4 or 6）~~ **確定: 4（前後 2 対）** | `meister_config.h:269` は 6 本定義のまま | 2026-11-19 依頼者申告「旋回は4つ、6輪中前後4輪」 |
| 幾何 `x=±0.30, y=±0.25` が図面値か実測値か | `meister_robot.yaml:111-123` | 実測 |
| `wheel_radius: 0.05` | `meister_robot.yaml:112` | 実測 |
| `max_linear_mm_per_sec: 800` | `meister_robot.yaml:115` | 実測 |
| 実機トップ速度 | 未測定 | 実測 |
| BNO055 ヨー符号（CW 正の仮定） | `serial_io.hpp:84-85` | 実測 |
| エンコーダ無しでのオドメトリ精度 | `ackermann_imu_odometry.hpp:104-124` | PR2 以降で計測 |
| FOC ボードへの実配線（UART 本数・ピン・レベル） | 未着手 | 実機の結線を見る |
| ロッカー位置表の解釈（1軸×4輪か、前後2軸か） → **舵角 4 輪の並びは前後 2 対で確定**。ロッカー軸そのものは未確認 | `meister_robot.yaml:129` | 機構設計の確認 |
| ギア比・車輪径から m/s を出す換算 | 未測定 | 実測（`foc_rover` は rpm のみ既知） |

**確定済み（2026-11-19）**: 舵角は 4 輪（前後 2 対、中間 2 輪は固定）。
これで `meister_robot.yaml` の `steerable` パターン（前後 true / 中間 false）は確定。
ただし **x/y の座標値・車輪径・ギア比・FOC 配線は未確定**のままで、
`geometry_filled: false` も維持する。

**ゲート**: 上の★が全て埋まるまで、PR5（駆動リンク）と PR6（ソルバ）には着手しない。同じ理由で `geometry_filled: false` を維持する。推測値で動かすと、通信試験は PASS したまま配線と係数が誤った状態で固定される。

## 11. 将来

- 自動探索によるマップ生成（`slam_toolbox` + frontier exploration）。地図生成モード中に Nav2 をどう使うかを再設計する。
- 自己位置推定モードから地図生成モードへの双方向切替（`slam_toolbox` の再 activate 時のグラフ扱いを確定させてから）。
- 実機オドメトリ改善（IMU 角速度の活用、エンコーダ追加）。`robot_localization`（3.8.3）がインストール済みなので、EKF による `/odom` と `/imu/data` の融合が最初の候補になる。
- 旋回半径・速度上限が確定した後の `SmacPlannerHybrid` 導入。
- 4WS を活かした省スペース旋回の経路計画（`use_rotate_to_heading` の実機有効化）。


## 12. 実装ノート（2026-11-18 追記）

### 追加の決定

| # | 決定 | 理由 |
|---|---|---|
| D-16 | **プロジェクトのコードは C++ を原則とする**（Python は launch ファイルと、C++ 化する価値のない取得スクリプトに限る） | 依頼者判断（2026-11-18）。ベンチマーク一式と `generate_map` / `waypoint_follower` は C++ で実装済み |
| D-17 | 計画時間は **アクション受理から `/plan` 受信まで**で測る | 送信前を起点にすると DDS と行動サーバの往復を含み過大になる。§8.1 の 100 ms 判定に使う |
| D-18 | 占有セルは **`costmap_raw` の生値 254（LETHAL）のみ** | `255` は NO_INFORMATION なので障害物ではない。`>=254` で数えると接触が偽陽性になる |
| D-19 | ゴール選定は `bench_nav --auto-goals N` で**コストマップから決定論的に**行う | 手作業のクリックは再現できず、値の出所も残らない |

### 現状（2026-11-18 時点）

- **PR1・PR2 実装済み**。ベンチマークは C++ (`bench_nav`)、指標は ROS 非依存の `bench_metrics`。
- 単体テストは gtest 52 ケース（`colcon test`）。
- **PR2 の最後の1歩（30 run のベースライン計測）は未実施**。計測環境の確保が必要。
- PR1〜PR4・PR7 は完了（PR3 は 30 run で計測、PR7 は 10 ゴール比較）。
- **PR5・PR6 は §10 の★測定待ちで、設計上ここで止める**（推測値で動かすと
  通信試験だけ PASS した状態で係数が固定されるため）。
- `./build.sh` はこの開発機で `ros-jazzy-onnxruntime-vendor` 未インストールのため失敗する（本変更とは無関係。インストールで解消）。

### 計測基盤の現状（2026-11-19 追記）

`bench_nav` は 30 run のベースラインを完走し、全指標が欠損なく取得できた
（結果は `docs/bench/2026-11-18-baseline.md`）。実走で見つけて直した欠陥:

1. 出力ディレクトリを作らず、書けなくても成功扱いだった → 作成＋失敗時は非ゼロ終了
2. ゴールタイムアウト後に `future.get()` で永久ブロック → 上限付き待ちに変更
3. 計画長を「最後の `/plan`」から取っていた → 受理直後の最初の計画に変更
4. 計画時間が負になっていた → 受理応答が遅い場合は送信時刻を起点に落とす
5. **受理応答が約 8 秒遅れることがあり、その間の移動をサンプリングしていなかった**
   → 送信直後から採るよう変更（経路長が 0 になっていた原因）
6. 制御周期を `/cmd_vel_nav` だけで測っていた → `/cmd_vel` も購読

環境側の注意（コードでは解決できない）:

- Gazebo GUI は `DISPLAY=:1` を要求する（未設定だと Qt が abort する）
- 計測は launch と同一プロセスツリーで行う。別プロセスからは DDS が見えないことがある
