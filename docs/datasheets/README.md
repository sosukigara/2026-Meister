# 部品資料（一次情報）の置き場所

> 作成: 2026-11-22。実機の部品仕様を支える資料を置く。
> **値の出所はここに集約する**（値そのものの唯一の出所は `ros/config/meister_robot.yaml` の `robot:` 節）。

## 一覧

| ファイル | 何の資料か | 取得元 |
|---|---|---|
| `LA8308_datasheet.pdf` | Eaglepower LA8308（KV90 / 160 / 180 / 205）の公式仕様書・負荷テスト表（5 ページ） | Robu.in 経由の販社ミラー |
| `FEETECH_STS3235_datasheet.pdf` | FEETECH **STS3235 12V** の製品仕様書 Edition A/0（8 ページ） | Switch Science の仕様書ミラー |
| `FEETECH_STS3215_specification.pdf` | FEETECH **STS3215 7.4V** の製品仕様書 Edition A/0（8 ページ） | feetechrc.com 公式 |
| `FEETECH_serial_servo_tutorial.pdf` | FEETECH シリアルサーボ入門（12 ページ） | feetechrc.com 公式 |
| `LA8308_STS3235_仕様まとめ.md` | 上の PDF の要点まとめ（依頼者が作成・提供） | 依頼者 |

## ★重要★ 資料の型番と実機の型番が合わない

- 実機のステアリングは **STS3225 12V**（依頼者による仕様、2026-11-22）。
- 手元にある FEETECH 資料は **STS3235 12V / STS3215 7.4V**。**STS3225 の仕様書は未取得**。
- そのため **STS3225 のトルク・電流・ギヤ比を「STS3235 の値」で代用しないこと**（R4）。
  STS3235 の値は、同じ製造元の近い型番として参考に置くだけに留める。
- 取得を試して 404 だった先: `pages.switch-science.com/.../STS3225_datasheet.pdf`、
  `feetechrc.com/.../STS3225.pdf`。Switch Science の比較ページと FEETECH 公式 Wiki
  （`ftservo.github.io/ftservo-wiki`）にも STS3225 の項目は無い（2026-11-22 時点）。
  次の一手: FEETECH へ直接問い合わせ、または購入元の仕様書を入手する。

## 一次情報から確定した事実（コードと突き合わせ済み）

| 項目 | 値 | 出所 |
|---|---|---|
| STS 系の電子分解能 | 360°/4096 = **0.088°/パルス** | STS3235 仕様書 7-8 |
| STS 系の中立点 | 180° = **2048** | STS3235 仕様書 |
| STS 系の通信 | 半二重非同期シリアル、8bit・1stop・パリティ無し | STS3235 仕様書 7-2 |
| ボーレート範囲 | 38400 bps ～ 1 Mbps（**既定 1 Mbps**） | STS3235 仕様書 7-4 |
| 位置更新レート | 最大 1 ms | STS3235 仕様書 |
| コネクタ | 5264-3P、1=GND / 2=Vcc / 3=Signal（デイジーチェーン可） | STS3235 仕様書 |
| LA8308 の構成 | 36N40P（PDFのは 40 極 = **20 極対**。KV=90） | LA8308 仕様書 |
| LA8308 の想定電源 | 6S–12S LiPo（PDF の負荷試験は 12S = 46 V で実施） | LA8308 仕様書 / Robu.in |
| LA8308 の内部抵抗 | 0.186 Ω（KV90 のページ表記） | Robu.in |
| STS3215 の電源 | 4–7.4 V（**12 V 系には使えない**） | STS3215 仕様書 5-7 |

### コード側との突き合わせ結果

- `feetech_sts_registers.h` の `kStepsPerRev = 4096`（360° 型）と `kEndianness = kLittle` は
  **一次情報と整合**（4096・360° を確認）。★要確認★だった項目の一部がこれで確定した。
- EEPROM のレジスタアドレス（`kBaud=3` / `kTorqueEnable=40` / `kGoalPos=42` など）は
  **STS3235 仕様書に表として載っていない**ため未照合のまま。仕様書の 4〜5 ページは
  本文説明で、アドレス表は別資料（SDK / メモリマップ）にある。**推測で埋めない。**
- `ros/config/meister_robot.yaml` の `bus.baud: 1000000` は「既定 1 Mbps」と整合する。
  ただし実機のサーボが 1 Mbps に設定されているかは**実機で読み出すまで ✗ 未検証**。

## 12 V 運用に関する注意（LA8308）

LA8308 は 6S–12S（22–46 V）のドローンモータで、仕様書の負荷試験も 46 V。一方、依頼者の
仕様は 12 V 系。**KV90 は 12 V で 18 rev/s（無負荷）**になり、車軸（1:20）で 0.9 rev/s・
タイヤ 100 mm で 0.283 m/s（無荷重）という計算になる。この値は
`firmware/foc_rover/include/foc_config.h` のベンチ上限（5 rev/s）の内側に収まる。
**回転数として動くことは計算上成り立つ**が、トルク・電流・低電圧しきい値（現状 20.0 V）が
12 V 前提ではないので、**それらは ★要確認★ のまま**（`foc_rover/README.md` の表を参照）。
