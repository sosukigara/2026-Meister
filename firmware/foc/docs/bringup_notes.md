# センサレス bring-up 記録 (MKS ESP32 FOC V2.0 / ch M1)

> 日付: 2026-10-06
> 対象: `firmware/foc`
> 前提: SimpleFOC 2.2.1 / espressif32@6.9.0 / AS5600 エンコーダ未接続
> 根拠: 実機ログ、SimpleFOC 2.2.1 原文、公式 `makerbase-motor/MKS-ESP32FOC` V2.0

この文書は「なぜ回らなかったか」「なぜ一度発煙したか」「今の安全機構は何を保証し、
何を保証しないか」を、後から同じ穴に落ちないために残すもの。

---

## 1. 採用したピン定義（一次情報と一致）

出典: `github.com/makerbase-motor/MKS-ESP32FOC`, branch `MKS-ESP32-FOC-V2.0`,
dir `Test Code/`, `11_close_loop_velocity_example*.ino` ほか。

| 項目 | M1 (現行) | M0 (参考) |
|---|---|---|
| PWM A/B/C | 26 / 27 / 14 | 32 / 33 / 25 |
| ENABLE | 12 | 12 |
| 電流 A/B | 35 / 34 | 39 / 36 |
| I2C | bus 1, SDA 23, SCL 5, 400 kHz | bus 0, SDA 19, SCL 18, 400 kHz |
| 極数 | 7 | 7 |

M0・M1 は**両方とも実機で診断済み**。どちらも同じ結果（後述）で、ハード不良ではなく
ソフトが電圧を出していなかったことが分かった。

---

## 2. 「回らない」の原因（確定）

以前の発熱対策で `motor_.phase_resistance` に `0.05` を設定していた。

SimpleFOC 2.2.1 のオープンループは、`phase_resistance` が設定されていると印加電圧を
次の式で導出する (`BLDCMotor.cpp` velocityOpenloop / angleOpenloop):

```
float Uq = voltage_limit;
if (_isset(phase_resistance)) Uq = current_limit * phase_resistance;
```

これにより

```
Uq = current_limit(0.3 A) × phase_resistance(0.05 Ω) = 0.015 V
```

となる。24 V 供給での ESP32 LEDC は **10 bit**（`esp32_ledc_mcu.cpp` の
`_PWM_RES_BIT 10`, `_PWM_RES 1023`）なので 1 ステップは `24 / 1023 ≈ 0.0235 V`。
**0.015 V は最小ステップ未満なのでデューティが 0 に丸められ、出力が一切出ない。**
端子電圧 0 V・診断電流 ~0 A・無回転はすべてこれ1つで説明できた。ハードは正常だった。

> 診断コマンド `D` が `OPEN`（両相 ~0 A）を返したのは**誤判定**。実際には電圧が
> 出ていなかっただけで、巻線は導通していた（テスターで「ほぼ 0 Ω」= 低抵抗巻線）。

### 対策（採用）

- `phase_resistance` の設定を**削除**。オープンループが `voltage_limit` を使うように戻す。
- `kOpenLoopVoltageVolts = 1.0f`（かつて煙を出した 2.0 V は使わない。1.0 V は短時間用）。

低抵抗巻線に PWM で小さい電流を指令することは原理的にできない。
**ファーム側で電流を制限する手段は無い**（後述）。

---

## 3. 一度発煙した原因（確定）

安全改修前、オープンループは `voltage_limit = 2.0 V` を**電流制限なしで**印加していた
(`BLDCMotor.cpp:622` 近傍)。`phase_resistance` も未設定で、SimpleFOC 内蔵の安全弁も
無効だった。5010 は低抵抗（0.1 Ω 級）なので、2.0 V で約 20 A が巻線に流れ続けた。

- 熱源: 電源入力部/端子台の付け根。モータ本体も発熱 = 全体過電流。
- `kCurrentLimitAmps = 1.0` は**閉ループ専用**で、オープンループには効かなかった。

対策として `kOpenLoopVoltageVolts` を下げた（現在値 1.0 V、当初 0.5 V）。公式サンプルも
2.0 V を "excessive" と明記し、オープンループは必ず発熱するので 1 分以上回すなと警告している。

---

## 4. 単位バグ（rad/s と rev/s）

SimpleFOC のオープンループ目標と `velocity_limit` の単位は **rad/s**。config / README / UI
は当初これを **rev/s** と表示していた（`kVelocityLimitRps = 130` は実は rad/s で、公式の
20 rad/s の約 6.5 倍だった）。対策:

- `kVelocityLimitRps` を `20.0f`（公式値、rad/s）へ。
- `O` / `N` の入力は rev/s として受け、`target_ = rev/s × 2π` で rad/s に変換。
- テレメトリの速度は `shaft_velocity / 2π` で rev/s 表示。

---

## 5. 実装した安全機構（保証する範囲）

1. オープンループは `voltage_limit = 1.0 V`（短時間用）。かつての 2.0 V は使わない。
2. **10 秒で自動停止** (`kOpenLoopTimeoutMs`, `#TIMEOUT`)。
3. 停止後 **10 秒は再始動を拒否** (`kOpenLoopCooldownMs`, `#BUSY`)。デューティは 50% 以下。
4. 閉ループ（`T`/`W`/`A`）は `user_current_limit_` を保持/復元し、オープンループの
   低電流設定と混ざらないようにしている。

### 保証しないこと（重要）

- **ファームは巻線電流を制限できない。** 低抵抗巻線に PWM で小電流を指令するのは不可能。
- **ベンチ電源の CC は「電源電流」を制限するもので、巻線電流は制限しない。** 1.0 V では
  PWM デューティが約 4% なので、相電流は DC 入力電流の約 25 倍。CC=0.5 A が効き始めるのは
  相電流 ~12 A からで、この電圧では巻線の発熱を守らない。**発熱は `V^2 / R`** で決まる
  （1.0 V ≈ 8 W、0.5 V ≈ 1.9 W、2.0 V ≈ 31 W = 発煙した値）。
- 短絡した巻線・短絡した配線はソフトでは守れない。無人で回し続けない。

---

## 6. 実機検証（evidence）

24 V 投入、ベンチ電源 CC=0.5 A、`O 2`（2 rev/s）。※これは `kOpenLoopVoltageVolts = 0.5 V`
だったときの記録。現在は 1.0 V（1.0 V での実機検証は ✗ 未検証）:

```
#VIN 24.36                                   ← 24 V 投入 OK
mode=3, vel_rps=2.00, vq_v=0.500            ← 指令どおり（velocity_openloop）
#TIMEOUT open loop stopped                   ← 10 秒自動停止が発火
（再送）O 2 → #BUSY cooling, wait             ← クールダウン拒否が発火
S → #STOP                                     ← 停止
```

- モーターは実際に回転し、**発熱しなかった**（目視確認）。
- native unit test 7/7 PASS、`pio run -e foc_m1` 成功。
- M0・M1 の両チャネルで診断 `D` は同じ結果（両相 ~0 A）だったが、原因は §2 の
  「電圧がサブ分解能でゼロ」であり、チャネル側の故障ではなかった。

---

## 7. 誤解しやすい罠（後任向け）

- **`iq_a` / `id_a` が常に 0.000 でも故障ではない。** センサレス中は
  `FocDrive::poll()` が `motor_.loopFOC()` を**意図的にスキップ**するため、
  `motor_.current` が更新されず、テレメトリは常に 0 を出す。表示上の仕様。
- **USB のみ（24 V OFF）だと駆動はゲートされる。** `begin()` が `#ERR undervoltage` で
  早期 return し、`motor_inited_` が false のままなので `O` も拒否される。
  通電前は必ず boot ログの `#VIN` が ~24 V であることを確認する。
- **ESP32 はシリアルを開くたびにリセットする。** 回転中にモニタを開くな（ゲートが落ちて
  ドラムが空転する）。DTR/RTS をトグルしないと再起動しないので、boot ログを採るには
  リセット直後に読む必要がある。
- `#BUSY` の直後に `#ERR drive gated, refusing motion` が余計に出るのは、
  `ready_`（= initFOC 成功）がセンサレスでは常に false のため。文言のみの問題。

---

## 8. 要確認 / 未検証

| 項目 | 現状 | 未検証な点 |
|---|---|---|
| 低電圧しきい値 `kUndervoltageVolts = 20.0f` | 24.36 V を実測、4.65 V でゲート作動を確認 | 20.0 V 境界そのものは未測定。**✗ 未検証** |
| I2C bus index `kI2cBusIndex = 1` | 公式 M1 例に一致 | エンコーダ未接続のため AS5600 通信は未確認。**✗ 未検証** |
| 極数 `kPolePairs = 7` | 公式例に一致 | 実機のモータから実測していない。**✗ 未検証** |
| 巻線抵抗 | テスターで「ほぼ 0 Ω」（低抵抗・導通） | 数値としての実測なし。**✗ 未検証** |
| `zero_elec` 健全窓 3.12–3.42 rad | エンコーダ未接続のため未計測 | **✗ 未検証** |
