# コードコメントから移行した知見（DOCS）

> 2026-11-28。R9「コメントは WHY だけ」に従い、コード内の説明的・AI 生成的な
> コメント（ファイル概要、区切りバナー、出所表、数式の書き写し）を削除した。
> ここには、削除すると失われる**一次情報・未検証契約・設計根拠**だけを移行する。
> 数値そのものの唯一の出所は `ros/config/meister_robot.yaml` の `robot:` 節と
> `docs/datasheets/` であり、この文書はそれらの補足である。

---

## 1. hand landmark モデルの契約（★要確認★）

出典: `ros/src/umeonigiri_vision/include/umeonigiri_vision/hand_landmark_detector.hpp`
（旧ファイル先頭コメント）。`hand_landmarks.onnx` は `.gitignore` 済みで
**一次情報を取得できていない**。下記はモデル側の都合から決めた契約であり、
実モデルの定義と一致するかは未検証（R4）。

- 入力: session の入力メタデータから静的 4 次元かつチャンネル次元が 3 のものを
  NHWC `(1, S, S, 3)` と NCHW `(1, 3, S, S)` として受け入れる。`S` は入力辺長。
  動的形状や 3 チャンネルでない形はエラー。
- 出力: `(1, K, 3)` の float32。既定 `K = 21`。
  `[:, 0] = x`、`[:, 1] = y`（letterbox 済みキャンバス基準で 0.0–1.0 に正規化）、
  `[:, 2] = score`（0.0–1.0）。
- 並び順: `K = 21` なら MediaPipe 規約（0 = 手首、1–4 = 親指、5–8 = 人差し指、
  9–12 = 中指、13–16 = 薬指、17–20 = 小指）を想定しているが、これも ★要確認★。
  `K` を設定で変えられるようにしているのは、規約の違うモデルへ差し替えても
  添字の読み違えを避けるため。
- 入力画像は BGR（OpenCV 形式）。
- 規約を読み違えた壊れた keypoint を黙って流すより、例外で落とす方に倒す（R4）。

> `docs/code-map.md` の「契約はヘッダ冒頭に隔離してある」という記述は、
> この文書へのポインタに置き換えた。

## 2. vision / autonomy の設計根拠・性能値

- `detection_node.cpp`: `bgr8` 入力は `BgrViewOrConvertFromImageMsg()` により
  921 KB のフレーム複製を回避。`TargetSelector` の α-β カルマンゲインは
  `alpha_pos = 0.70` / `beta_vel = 0.25`（走行中・アーム移動中の揺らぎ平滑化）。
  配信は `unique_ptr` によるゼロコピー。
- `letterbox.hpp`: `LetterboxInto()` は事前確保済み `cv::Mat` へ直接リサイズし、
  padding 帯のみを塗り潰す。毎フレームの 1.2 MB 再確保と中央 ROI の二重書き込みを排除。
- `rknn_yolo.hpp`: Rockchip RKNN で YOLOv8 系（検出ヘッドが reg/cls/sum の
  3 系統 × 3 スケール）を実行。NPU 出力は int8 のまま受け取り、
  必要なアンカーだけを逆量子化する。
- `yolo_grasp_node.cpp`: `detection_node`（ONNX/CPU）と同じトピックを出すため、
  Web UI とアーム制御から見ると差し替え可能。CPU 実装より速く、CPU 使用が少ない。
- `yolo_detector.hpp`: ONNX Runtime と YOLOv8n（COCO 80 クラス）のみを使用。
  torch / ultralytics は入れない。Python 版と同じ C API を呼ぶので
  移植しても推論結果は変わらない。
- `vision_cache.hpp`: `detection_node` は 3 Hz で描画済み画像を流す。
  毎回 PNG を渡すと 1 フレーム数十 KB が積み上がるため、JPEG に落として保持し、
  `/api/vision.jpg` と MJPEG が同じバイト列を共有する。
- `map_generator.hpp`: SLAM を回さずに Nav2 を試すためのもの。
  以前は `scripts/generate_map.py` が同じ計算をしていたが、2026-11-18 に C++ へ移行。
  出力は Python 版とバイト一致することをゴールデンファイルで確認している。
- `ackermann_imu_odometry.hpp`: 50 Hz の SE(2) 相補 / カルマン推定。
  Ackermann の速度・舵角コマンドと BNO055 方位（`FB_IMU` の `yaw_tenths` と
  calibration status）を融合し、外部 IPC なしで `odom -> base_footprint` TF と
  `/odom` を出す。
- `kinematics.hpp` の旧 `atan2` 定式コメントは、後退時バグ修正後に
  コードが `atan(L * wz / vx)` へ変わった後も残っていた**古い記述**だったため削除。
  現在の実装が正しい（回帰テストあり）。

## 3. FOC ファームウェアの出所・導出

### `firmware/foc_test/`（回転テスト専用）

- 包絡の算出: VENDOR の `R = 0.186 Ω`（line-to-line、KV90）に対し、
  2026-11-09 に一相へ 0.3 V を印加したところ 2.13 A を実測。
  よって経路は `phase + phase/2 = 0.14 Ω` で、`0.186 Ω` は相間値であって
  一相当たりではない。オープンループとアラインは約 2.1 A を引き、
  旧来の「1.6 A／相」読みは誤り。4 A のオープンループトリップと
  6 A のベンチ包絡の内側にある。
- クローズドループ電流は 1.0 A 固定。`10 rps` での BEMF は約 6.7 V で 12 V に収まる。
- 速度ループの加速度上限 30 rad/s² は、0 → 62.8 rad/s を約 2.1 s で上げる
  値（フリーシャフトに対する reasoned pick、実測ではない）。
- ゲインの出所: ARTICLE（5010 モータ）のベースライン。LA8308 では要ベンチ調整
  （コード側は短い `TODO(bench)` のみを残す）。
- 出典ファイル名: makerbase-motor/MKS-ESP32FOC の `MKS-ESP32-FOC-V2.0` ブランチ、
  `Test Code/3_dual_as5600_test.ino` と `11_close_loop_velocity_example.ino`。
- Infineon AS5600 データシート Fig 21–22: CONF 高位 `0x07` の bits[1:0] = SF、
  `0x03` が 2 倍速。AS5600 の PDF は `docs/datasheets/` に無いため ★要確認★。

### `firmware/foc_rover/`（走行用）

- 出所の凡例: `MKSREPO` = 上記公式リポジトリ / `SCH` = 同ブランチの
  `Hardware/MKS ESP32 FOC V2.0_SCH.pdf` / `VENDOR` = Eaglepower LA8308 KV90
  販売店スペック（一次資料ではない）。
- 公式サンプルのピン割当て:
  - M0: `BLDCDriver3PWM(32, 33, 25, 12)` / `InlineCurrentSense(0.01, 50.0, 39, 36)` /
    `TwoWire(0).begin(19, 18, 400000)`
  - M1: `BLDCDriver3PWM(26, 27, 14, 12)` / `InlineCurrentSense(0.01, 50.0, 35, 34)` /
    `TwoWire(1).begin(23, 5, 400000)`
- `0.186 Ω`（販売店値）は**低速の電流センサ妥当性チェックにのみ**使い、
  制御ゲインには使わない。別モーターで使う前に実測すること。
- `1.0 V` は `0.186 Ω` で約 `5.4 A` を要求する（電流センサ飽和の説明）。
- 20 極対（40 磁石）の根拠: 販売店ページには記載が無いが、独立した 2 件の
  ODrive コミュニティ報告と稼働中の ODrive 設定が一致する。
  - <https://discourse.odriverobotics.com/t/callibration-issue-when-motor-magnets-arent-alligned-with-coils/8806>
  - <https://discourse.odriverobotics.com/t/odrive-s1-motor-calibration-error/11326>
  - <https://github.com/AtomsClark/Integrated-Cycloidal-Robotic-Actuator/blob/main/firmware/odrivetool-session.md>
  - 別バリエーションに適用する前に実際の磁石数を数える（÷2）。`7` は古い 5010 の
    プレースホルダで、**絶対に使わない**。`initFOC() == 1` は極対不一致の
    証拠にならない（SimpleFOC は警告を出すだけで成功を返す）。
- `foc_limits.h` は Arduino ヘッダを含まない**意図的な**設計。
  console / drive / native テストが同じ定義をリンクし、上限が食い違わないようにするため。
- `platformio.ini` に `upload_port` を `[common]` へ書かないこと。
  3 枚のボードそれぞれに別のパスを指定する必要があるため。
- `TEMP D` はオープンループ上限で相を通電しシャント電流を報告する診断。
  開放巻線と電圧不足を切り分け、設定した巻線抵抗に対してセンサゲインを検証する。

> 表中の ★要確認★ 項目は、一次情報が取れた時点でここを更新する。
> 値そのものをコードに書き戻すときは R2（単一出所）と R4（未測定値の隔離）を守る。

## 4. プロトコル / キネマティクスの設計決定

- プロトコルを固定長バイナリにした理由: テキストベース（JSON 等）はデバッグには
  有利だが非決定的なため不採用（`meister_protocol.h` の旧ファイル先頭コメント）。
- `kinematics` の設計決定（旧 `kinematics.h` / `kinematics.cpp` の先頭コメント）:
  - **D1**: Twist の展開は ESP32 側で行う。PC は判断だけを担う。
  - **D3**: 拘束ソルバとボギー軸の位置表。ボギー軸は連続運動を拘束から解き、
    ロック位置だけを表で持つ。
  - 旧コメントは `docs/superpowers/specs/2026-09-27-command-and-kinematics-design.md`
    を参照していたが、そのパスはリポジトリに存在しない。決定内容だけをここに残す。
- `kinematics.cpp` の「呼び出し側禁止」ブロックはコード側に残してある
  （同じ知識が `docs/pitfalls.md` にもある）。

## 5. BNO055 レジスタ表（★要確認★）

出典: `firmware/main_board/src/hal/imu_bno055.cpp` の旧コメント。
Bosch BNO055 datasheet v1.4 で照合していない。web search が遮断されていた
ため記憶値で実装しており、実機ベンチで chip ID と heading 変化を確認するまで
値を根拠に使わない（R4）。

| 名称 | アドレス | 値 |
|---|---|---|
| CHIP_ID | `0x00` | `0xA0` 期待 |
| EULER_H_L | `0x1A` | heading 下位（続く `0x1B` が上位） |
| CALIB_STAT | `0x35` | sys[7:6] gyro[5:4] acc[3:2] mag[1:0] |
| OPR_MODE | `0x3D` | CONFIG=`0x00` / NDOF=`0x0C` |
| PWR_MODE | `0x3E` | NORMAL=`0x00` |
| SYS_TRIG | `0x3F` | RST=`0x20` |
| PAGE_ID | `0x07` | 0（page 0 固定） |

- 単位: Euler 1 LSB = 1/16°。heading 生値 0..5760 → 0.1° 単位 0..3600。
- 復帰待ち: reset 後 650 ms、NDOF 切替後 30 ms（データシートの立上り時間、
  ★要確認★ 未照合）。

## 6. 補足（FOC 電流定格）

- `6 A/ch` は冷却を前提とした**ピーク値**であり、実測の連続定格ではない。
  `firmware/foc_rover/README.md` の表にも「連続 6 A×2 は ✗ 未検証」とある。
