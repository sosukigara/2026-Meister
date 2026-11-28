# M5Stack IMU (Tab5)

M5Stack Tab5 の内蔵 BMI270 6軸 IMU を読み、ジャイロをマイコン内で積分した yaw とともに
USB CDC シリアルで PC へ送り、Tab5 本体の画面にダッシュボード表示するファームウェアです。

## 構成

| ファイル | 役割 |
|---|---|
| `include/imu_config.h` | 周期・ボーレート・固定小数点スケールの唯一の出所 |
| `include/imu_protocol.h` / `src/imu_protocol.cpp` | 22 バイトフレームのエンコード／デコード（M5 非依存） |
| `include/imu_sampling.h` / `src/imu_sampling.cpp` | 生値→固定小数点、簡易 roll/pitch（M5 非依存） |
| `include/imu_yaw.h` / `src/imu_yaw.cpp` | ジャイロ積分・零点補正・傾き補正による yaw（M5 非依存） |
| `include/imu_view.h` / `src/imu_view.cpp` | M5GFX ダッシュボード（オフスクリーンスプライト） |
| `src/main.cpp` | IMU 取得・20 Hz 送信・ループ |

## 必要なもの

- M5Stack Tab5（ESP32-P4、BMI270）
- PlatformIO Core 6.x
- `m5stack-tab5-p4` ボード（PlatformIO `espressif32` プラットフォーム）
- M5Unified 0.2.29 / M5GFX 0.2.36（`platformio.ini` の下限は 0.2.23 / 0.2.30）
- Arduino core 3.3.9 を `platform_packages` で明示指定しています。
  環境に古い arduino-esp32 2.x が残っていると `FRAMEWORK_DIR` が解決されずビルドが落ちるためです。

Tab5 のボード定義は `ARDUINO_USB_CDC_ON_BOOT=1` / `ARDUINO_USB_MODE=1` なので、
Arduino の `Serial` は USB CDC（Type-C）です。

## ビルド・テスト

```bash
cd firmware/m5stack_imu
pio test -e native                 # Unity: 18 テスト（M5 依存なし）
pio run -e m5stack-tab5-p4         # ファームウェアビルド
```

## 書き込み・モニタ

Tab5 を PC に接続し、ポートを確認してから書き込みます。

```bash
pio device list
pio run -e m5stack-tab5-p4 -t upload --upload-port /dev/ttyACM0
pio device monitor -b 115200 -p /dev/ttyACM0
```

`upload_port` を `platformio.ini` に固定する場合は by-id で指定してください
（誤書き込み防止。AGENTS.md §6 の注意と同じ理由）。ポート名は環境依存なので `★要確認★` です。

## 送信フレーム（22 バイト）

| Offset | Size | 内容 |
|---:|---:|---|
| 0 | 1 | Header `0xA5` |
| 1 | 1 | Type `0x84` |
| 2 | 4 | timestamp [ms]（uint32 little-endian、`imu_data_t::usec / 1000`） |
| 6 | 2 | accel X [milli-G]（int16 LE） |
| 8 | 2 | accel Y [milli-G]（int16 LE） |
| 10 | 2 | accel Z [milli-G]（int16 LE） |
| 12 | 2 | gyro X [centi-deg/s]（int16 LE） |
| 14 | 2 | gyro Y [centi-deg/s]（int16 LE） |
| 16 | 2 | gyro Z [centi-deg/s]（int16 LE） |
| 18 | 2 | yaw [centi-deg]（uint16 LE、0..36000） |
| 20 | 1 | status（`0x01` = クランプ、`0x02` = 1 秒以上 IMU の新しい値が無い） |
| 21 | 1 | XOR checksum（フレーム全バイトの XOR が 0 になる） |

M5Unified の `imu_data_t` は accel が G、gyro が deg/s です。
スケール 1G=1000 / 1deg/s=100 はプロトコル上の表現の選択であり、実測値ではありません。

yaw は起動後 1.5 秒間静止している前提でジャイロの零点を平均し、その後の角速度を
実際の時刻差で積分して求めます。加速度から得た roll/pitch で傾き補正しますが、
BMI270 には地磁気センサが無いため**絶対方位ではなく相対方位**です。時間と共にドリフトします。

PC 側の最小読み出し例:

```python
import functools
import operator
import serial
import struct

with serial.Serial("/dev/ttyACM0", 115200) as port:
    while True:
        if port.read(1) != b"\xa5":
            continue
        frame = b"\xa5" + port.read(21)
        if len(frame) != 22 or functools.reduce(operator.xor, frame, 0) != 0:
            continue
        ts, ax, ay, az, gx, gy, gz, yaw = struct.unpack("<I6hH", frame[2:20])
        status = frame[20]
        print(ts, ax, ay, az, gx, gy, gz, yaw / 100.0, hex(status))
```

## 画面

ダッシュボード構成です。

- 大きな方位コンパス（N/E/S/W、yaw 数値、指向針、進捗アーク）
- yaw の時系列トレンド（直近 32 点）
- ROLL / PITCH のカード
- 6軸の数値と棒グラフ
- フッターに送信 Hz・TX 成否・status（OK / CLAMP / NO-IMU）

背景・カード枠・ラベルは起動時に一度だけ描き、毎フレームは値・針・棒グラフ・フッターだけを
部分描画します（`startWrite()`/`endWrite()` でまとめて転送）。全画面クリアがないため
チカチカせず、更新周期は 16 ms（60 Hz 目標）です。送信は FreeRTOS タスクに分離してあるため、
描画が重くても 20 Hz を維持します。
簡易姿勢は加速度から `atan2` で求めています。Tab5 の個体・取り付け向きによる軸の向きは
`★要確認★` です（実機で水平時の roll/pitch が 0 になることを確認してください）。

## yaw の精度

- 積分の刻みは `imu_data_t::usec` のマイ秒値を使うため、ミリ秒量子化による誤差がありません。
- 起動後 1.5 秒間の静止時にジャイロ零点を平均してバイアスとします。
- その後も、加速度の大きさが 1 G 付近で角速度が小さい静止時にのみ、バイアスをゆっくり追従させます。
- 加速度から得た roll/pitch で傾き補正するため、傾いた状態でもヨー角速度が正しく積分されます。
- BMI270 には地磁気センサが無いため、**絶対方位は出ません**。長時間ではドリフトします。
  絶対方位が必要な場合は外部コンパスなどの別情報源とのフュージョンが必要です（未実装）。

## 検証状況（2026-11-28）

- **確認済み（ホスト）**: `pio test -e native` = 18/18 PASSED（protocol / sampling / yaw）。
  `pio run -e m5stack-tab5-p4` = SUCCESS（`firmware.bin` / `firmware.factory.bin` 生成）。
- **確認済み（実機）**: 芯片は ESP32-P4 revision v1.3（esptool で確認）。by-id 指定で
  書き込み・ベリファイ成功。起動ログ
  `M5Stack_IMU ready: imu=BMI270 frame=22 bytes interval=50 ms display=720x1280 yaw_cal=1500 ms`
  を確認。22 バイト・yaw 付きフレームを PC で受信（`status=0x00`、yaw が変化）。
  利用者が画面を目視し、コンパス付きダッシュボードの表示とチカチカ解消、
  部分描画化後の更新が滑らかになったことを確認。
- **補足**: この開発 PC では `cdc_acm` が未ロードで `/dev/ttyACM0` が出なかったため、
  `sudo modprobe cdc_acm` を実行してから書き込んでいます。USB 再接続時にポート番号が
  `/dev/ttyACM0` / `/dev/ttyACM1` と変わるため、書き込みには by-id を使ってください。
- **★要確認★**: 60 Hz の実測フレームレート、長時間の yaw ドリフト量、
  roll/pitch の軸向き（水平時に 0 になるか）。絶対方位は出力されません。
