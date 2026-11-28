# M5Stack IMU (Tab5)

M5Stack Tab5 の内蔵 BMI270 6軸 IMU を読み、USB CDC シリアルで PC へ送り、
同時に Tab5 本体の画面に状態を表示するファームウェアです。

## 構成

| ファイル | 役割 |
|---|---|
| `include/imu_config.h` | 周期・ボーレート・固定小数点スケールの唯一の出所 |
| `include/imu_protocol.h` / `src/imu_protocol.cpp` | 20 バイトフレームのエンコード／デコード（M5 非依存） |
| `include/imu_sampling.h` / `src/imu_sampling.cpp` | 生値→固定小数点、簡易 roll/pitch（M5 非依存） |
| `include/imu_view.h` / `src/imu_view.cpp` | M5GFX による画面描画 |
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
pio test -e native                 # Unity: 11 テスト（M5 依存なし）
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

## 送信フレーム（20 バイト）

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
| 18 | 1 | status（`0x01` = クランプ、`0x02` = IMU 未更新） |
| 19 | 1 | XOR checksum（フレーム全バイトの XOR が 0 になる） |

M5Unified の `imu_data_t` は accel が G、gyro が deg/s です。
スケール 1G=1000 / 1deg/s=100 はプロトコル上の表現の選択であり、実測値ではありません。

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
        frame = b"\xa5" + port.read(19)
        if len(frame) != 20 or functools.reduce(operator.xor, frame, 0) != 0:
            continue
        ts, ax, ay, az, gx, gy, gz = struct.unpack("<I6h", frame[2:18])
        status = frame[18]
        print(ts, ax, ay, az, gx, gy, gz, hex(status))
```

## 画面

6軸の数値と棒グラフ、簡易 roll/pitch、送信 Hz、TX 成否、
status フラグ（OK / CLAMP / NO-IMU）を表示します。値エリアだけを描き直します。

簡易姿勢は加速度から `atan2` で求めています。Tab5 の個体・取り付け向きによる軸の向きは
`★要確認★` です（実機で水平時の roll/pitch が 0 になることを確認してください）。

## 検証状況（2026-11-28）

- **確認済み**: `pio test -e native` = 11/11 PASSED。`pio run -e m5stack-tab5-p4` = SUCCESS
  （`firmware.bin` / `firmware.factory.bin` 生成、依存ライブラリのインストール完了）。
- **★要確認★**: 実機への書き込み、画面表示、IMU 値の変化、PC 側でのフレーム受信、
  シリアルポート名、姿勢の軸向き。Tab5 が開発 PC に接続されていないため未実施です。
