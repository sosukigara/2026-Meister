# 設計: コマンドモデルとロッカーボギー運動学

> 対象機能: [features/07-esp32-uart.md](../features/07-esp32-uart.md) [features/08-rocker-bogie.md](../features/08-rocker-bogie.md)
> 作成日: 2026-09-27 | ステータス: 設計合意済み（実装は未着手）

## 背景

現状の指令経路には 2 つの問題がある。

1. **種別ごとに別フレームで、1 つの「意思」が存在しない。** `CMD_MOTOR_VELOCITY`
   / `CMD_STEERING_ANGLE` / `CMD_ARM_ANGLE` / `CMD_GRIPPER` が別々に arrive するので、
   機構の apply に渡る前に状態が分割され、整合性が取れない。
2. **運動学が PC 側にあり、かつ自転車モデルに退行している。**
   `kinematics.py:3-4` は「全ステアリングチャネルに同一の舵角、全駆動チャネルに
   同一の速度百分率」で、`docs/design/08-rocker-bogie.md:11` の
   「全輪独立操舵により…超信地旋回が可能」という目標を満たせない
   （6 輪が同じ舵角だと超信地旋回は物理的に成立しない）。
   `kinematics.py:8-11` も「アッカーマン系はその場旋回できない」と限界を明記している。

`docs/design/07-esp32-uart.md:8` の「リアルタイム制御（PWM 生成・サーボバス制御）は
ESP32」の方針に従い、展開は ESP 側が行う。

## 決定事項（2026-09-27 合意）

| # | 決定 | 理由 |
|---|---|---|
| D1 | **`Twist → 各輪の舵角・速度`の展開は ESP32 側** | `docs/design/07:8` の方針。PC は判断だけ担う |
| D2 | PC が送る指令は **`vx, vy, wz` の 3 成分** | 逆キネマティクス（車体速度 ← 輪実測）の検算に 3 成分形が便利で、将来メカノム化したときに型を変えずに有効化できる。**横移動自体が今回必要なわけではない**（D5） |
| D3 | 運動学は **拘束ソルバ + ボギー軸の位置表** | ボギー軸は機械的に有限位置を持つので、連続運動は拘束から解き、ロック位置だけ表で持つ |
| D4 | **コマンドは 1 つの型に集約**し、種別分岐を機構側に散らさない | 状態の整合性を保つ |
| D5 | **`vy` は型に残すが、非ゼロは `FB_ERROR` で拒否**する | 車体には横方向の自由度がない |

## レイヤ構成

依存は片方向のみ（R8）。Arduino 非依存の層は `pio test -e native` で検証できる。

```
include/meister_protocol.h    フレームの encode/decode          native test
include/command.h              RobotCommand / TwistCommand      native test
include/kinematics.h           Twist → DriveSetpoint 求解        native test
include/hal/*.h                PWM / バスサーボ / モータ         ESP32 のみ
include/base_chassis.h         機構の apply
include/arm.h
```

`command` と `kinematics` は Arduino を include しない。ここが host テストの勘所。

## 1. コマンドモデル（`command.h`）

ロボットの意思を 1 つの構造体で表す。種別ごとのフレームは廃止する。

```cpp
/// 車体の目標速度 [mm/s] / [0.1 deg/s]。PC から送る唯一の基底指令。
struct TwistCommand {
  int16_t vx;   // 前後
  int16_t vy;   // 横
  int16_t wz;   // 回転（正 = 左）
};

/// 1 輪の指令。角度はすべて 0.1 度、速度は千分率。
struct WheelCommand {
  int16_t velocity;  // -1000..1000
  int16_t steering;  // -900..900
  bool    steerable; // ボギー位置によって舵角が変わるか
};

/// ボギー軸の指令。位置表の添字ではなく角度を持つ。
struct RockerCommand {
  int16_t angle[2];  // 左右のボギー軸。0 = ロック
};

/// アームの指令。
struct ArmCommand {
  int16_t shoulder_pitch;  // DS サーボ 150kg（PWM）
  int16_t joints[3];       // 肩方位角 / 肘 / 手首（STS3215 バス）
  int16_t gripper[2];      // 開閉① / 開閉②（STS3215 バス）
};

/// 機構に適用する 1 回分の指令。
struct RobotCommand {
  TwistCommand base;
  WheelCommand wheels[6];
  RockerCommand rocker;
  ArmCommand arm;
  bool arm_updated;  // アームの指示があったときだけ true
};
```

### `vy` の扱い

`vy != 0` の `CMD_BASE_TWIST` を受信したら、**車体指令を更新せず**
`FB_ERROR`（エラーコード「横移動非対応」）を返す。黙って前方移動に落とすと
PC 側のモデルと実挙動が食い違い、Nav2 がなぜ進まないかを診断できなくなる。

将来メカノム化した場合は、`FB_ERROR` の返り値を外して
`Solve` の拘束式に横方向成分を足す。型は変えない。

### 責務の線引き（逆キネマトリクス）

- **オドメトリは Nav2（PC 側）が担当**する。`docs/design/08:11` の
  「Nav2 の経路追従と整合させる」ため
- **ESP 側の逆変換はスリップ検知にのみ使う**。各輪の実測が車体の目標と
  ずれたら警報を返す
- ESP 側に第 2 のオドメトリを作らない。作ると Nav2 と二重になる

`arm_updated` を持たせるのは、PC がアームを操作していないときに
「前回値を保持」と「ゼロにクリア」を区別するため。
アームを操作しないと現在保持のままだ、という挙動が PC 側からは観測できない。

### 値のクランプ

各フィールドは範囲外の値を受け取った時点でクランプする。クランプは
`command.h` の `clamp_*` ヘルパが 1 か所で持ち、YAML の `ranges` から生成する。
**範囲外の値を黙ってゼロ clear しない**（attached もされない）のは危険なので、
clamp に倒す。

## 2. 運動学（`kinematics.h`）

```cpp
/// 幾何係数。YAML から生成する。
struct RobotGeometry {
  float body_length;          // m
  float body_width;           // m
  float wheel_radius;         // m
  // 6 輪の車体座標での位置と、最初は舵角が自由か
  struct Wheel { float x; float y; bool steerable; };
  Wheel wheels[6];
};

/// TwistCommand + 幾何 + ボギー位置表 → 各輪の指令
DriveSetpoint Solve(const TwistCommand& twist,
                    const RobotGeometry& geo,
                    const RockerPositionTable& table);
```

### 解法

1. `twist` の大きさと方向から、ボギー軸の位置を位置表で選ぶ。
   判定は「その配置で no-slip 拘束が解けるか」で、前進・後退・直線斜行・
   旋回・超信地旋回に対応する行を用意する。
2. 各輪について、車体座標での位置 \((x_i, y_i)\) から
   その接地点の速度ベクトルを求める:

   $$\mathbf{v}_i = (v_x - \omega y_i,\; v_y + \omega x_i)$$

3. **舵角が自由な輪**は、\(\mathbf{v}_i\) の方向に車輪を向ける
   （no-slip 条件。車輪は進行方向に Cusack  Spawn して滑らない）。
4. **舵角が固定された輪**は、向きは机构が拘束する。速度は
   \(|\mathbf{v}_i|\) のうち車輪方向の成分。横成分は滑る（steering が
   固定されている限り避けられない）。
5. 出力は各輪の `WheelCommand`。

### ボギー軸の位置表

```yaml
kinematics:
  rocker_positions:
    - name: locked          # ボギー軸がロック。前後輪が一直線。
      left: 0
      right: 0
    - name: unlocked        # 左右のロックを解除し、旋回を許可
      left: -35
      right: 35
```

表の各行に「その配置で解ける運動」をコメントで書く。`Solve` は
`twist` に最も近い行を選ぶ。表に無い組み合わせ（例: どちらも解除 +
直線斜行）は、表の定義を直すか机构の制約を追加する必要がある。

### 逆キネマティクス

`DriveSetpoint` から `TwistCommand` に戻す逆変換も同じファイルに置く。
スリップ検知と、PC 側の Nav2 によるオドメトリとの整合検証に使う。**同じ係数を使う**ので、
前方キネマティクスと逆で係数が食い違わないことをテストで確認する。

## 3. プロトコルの変更（`meister_protocol.h`）

| 種別 | 旧 | 新 |
|---|---|---|
| 0x01 | `CMD_MOTOR_VELOCITY` int16×6 | `CMD_BASE_TWIST` int16×3（vx, vy, wz） |
| 0x02 | `CMD_STEERING_ANGLE` int16×6 | （廃止。展開は ESP 側） |
| 0x03 | `CMD_ARM_ANGLE` int16×4 | `CMD_ARM` int16×6（肩 PWM + バス 3 + 開閉 2） |
| 0x04 | `CMD_GRIPPER` uint8 | `CMD_ROCKER` int16×2（手動上書き用。通常は ESP が選ぶ） |

`PROTOCOL_VERSION` を 3 に上げる。旧フレームは**受理しない**（長さが違うので
静かに壊れるより明示的に弾く）。`comm_check` がバージョンを表示して
不一致を 1 目で分かるようにする。

## 4. PC 側の変化

- `serial_bridge_node.py`: `/cmd_vel` を 3 成分に分解して `CMD_BASE_TWIST` を送る。
  展開はしない
- `kinematics.py`: `twist_to_actuators` は**削除**。逆キネマティクス用途だけ残すなら
  ESP 側と同じ係数を YAML から生成して使う
- 新: Python 側の運動学は、差分テスト用に C++ と同じ係数から生成する
  （オドメトリ検証用。指令には使わない）

## 5. テスト

| 層 | 手段 |
|---|---|
| `protocol` | `pio test -e native`。旧版との非互換を検証 |
| `command` | `pio test -e native`。クランプ、保持意味論 |
| `kinematics` | `pio test -e native`。前進で全輪直進、旋回で内外速度比、逆変換の往復 |
| C++ / Python 一致 | 同じ YAML 係数から両方を生成し、代表入力で差分テスト |
| 実機 | `meister_comm_check` の downlink を Twist + Arm に差し替え。`esp32dev_usbuart` で確認 |

## 6. 未検証・要確認

- **実機のボギー軸の位置**：表の値は機構設計から求めた推測。実物を見ないと
  解けない。`★要確認★`
- **車輪の配置**：6 輪の `(x, y)` も設計値。図面から取る必要あり
- **横滑りの可否**：車体には横方向の自由度がない。`vy != 0` は D5 のとおり `FB_ERROR` で拒否する

## 関連

- 設計: [07-esp32-uart.md](07-esp32-uart.md) / [08-rocker-bogie.md](08-rocker-bogie.md) / [09-object-grasping.md](09-object-grasping.md)
- 設定の唯一の出所: [config/meister_robot.yaml](../../config/meister_robot.yaml)
- コードマップ: [code-map.md](../code-map.md)
