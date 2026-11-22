# 実機対応・アーム把持・自動制御の計画と制約

> 作成: 2026-11-22。`docs/features/06,09,10` `docs/functions/*` `docs/design/06,09,10` を
> 廃止した際に、**どのファイルにも重複して持たれていなかった内容だけ**を移した。
> 実装済みの事実は [architecture.md](../../architecture.md) / [commands.md](../../commands.md) /
> [code-map.md](../../code-map.md) と各 spec が持つ。ここは「まだ実装していないこと」と
> 「実機で踏んだ制約」を置く場。

## 1. 実機対応 — 実機で確認できたこと / できていないこと

起動方法は [commands.md](../../commands.md) §5。ここは launch 構造ではなく**実測**を残す。

### 実測（2026-11-11、実機モード）

- LD-D500 (`/dev/ttyUSB0`、CP210x): `/scan` 9.95 Hz、720 bins、RViz で点群表示 OK
- SLAM: `/map` 1 Hz（224x156 → 成長中、5 cm）。Web UI 地図表示 OK、Nav2 `nav2_ready:true`
- 画像認識（EMEET C960 UVC、`/dev/video0`）: `/camera/image_raw` 約 30 Hz、
  `person 0.93` 等 14〜16 件検出、Web UI 認識モニター表示 OK。
  `/detection_image` は 0.8 Hz 前後（描画コスト）
- 権限: ユーザが `dialout` 未加入だとシリアルが開けない。恒久対策は `usermod -aG dialout`
  後に再ログイン（暫定の `chmod 666` は再起動で消える）

### ★要確認★

- D500 の `clockwise` / `angle_offset_deg` は搭載向きに合わせて調整する（既定 `true` / `0.0`）。
  鏡像・回転していたら `umeonigiri_lidar/config/d500.yaml` ではなく `real_robot.launch.py`
  の引数で直す。
- 実機 odom: `umeonigiri_serial_bridge` は `/odom` を配信するようになったが（[code-map.md](../../code-map.md)）、
  **実機でエンコーダではなく指令速度積分 + IMU ヨー**である点は変わっていない。
  Nav2 の自己位置推定精度は実機走行で確認すること。
- 実機 launch の SLAM 起動は固定秒数待ちの `activate` 禁止（[pitfalls.md](../../pitfalls.md) P12）。

### ✗ 未検証

- vendored LD-D500 ドライバ自体の実機 `/scan` 出力（上記は 1 回の測定）
- 実機での SLAM 地図作成・ナビゲーション走行の繰り返し検証
- 外部ドライバ（LiDAR・モータ・odom）との統合確認

## 2. 4軸アーム + 画像認識 + 自動把持

### 実装状況

- 物体検出のベースは PC 側で実装済み・実カメラで動作確認済み
  （`umeonigiri_vision`。パイプラインと性能は
  [vision-perf-baseline.md](../../bench/2026-11-20-vision-perf-baseline.md)）
- **アーム / IK / 手首カメラの取り込み / 把持は未実装**

### 設計上の制約（判断の根拠になるもの）

- **6DoF 姿勢推定は難所**。初期は「マーカー（AprilTag / ArUco）併用」または
  「机上平面上の物体を真上から掴む」方式から始め、段階的に汎用化する。
- **段階的運用**:
  1. 対象がアーム可動域内にある場合のみ、台車を止めたまま把持
  2. 可動域外なら台車も移動して接近してから把持（§3 と連携）
- **認識は PC / ROS 2 側**。ESP32 は認識処理を持たない。
- カメラは手首搭載（アイインハンド）なので**ハンドアイキャリブレーション**が必要。
- 座標系と逆運動学（肩→肘→手首→グリッパー）、サーボ制御系統（肩: PWM・肘/手首/グリッパー:
  シリアルバス）、把持シーケンス（検出→選択→接近→把持→持ち上げ→解放）は**未設計**。

## 3. 自動制御（自動移動 → 完全自律）

理想は完全自律（人が介在しない運用）。実装順は **自動アーム → 自動移動 → 完全自律**。

- [ ] **自動移動**: 実機の LD-D500 で SLAM・自己位置推定し、Nav2 で自動走行する
- [ ] ウェイポイント巡回: Web UI で指定した複数地点を自動で巡回する
- [ ] **接近把持連携**: 対象が可動域外なら台車を移動させてから把持する（§2 の第 2 段階）
- [ ] **完全自律**（理想）: 自動巡回 + 障害物回避 + 物体把持・回収の統合

## 4. ゴールとの対応

| ゴール | 主に担うもの |
|---|---|
| G1 自動走行 | `umeonigiri_autonomy`（SLAM / Nav2）、`umeonigiri_lidar`、`umeonigiri_serial_bridge`、`foc_rover` |
| G2 自動でアームでものを回収 | §2・§3（未実装）、`umeonigiri_vision`（検出のみ実装済み） |
| G3 手動操縦 | `umeonigiri_webnav`（Web UI）、`umeonigiri_serial_bridge` 経由の指令 |

実装済みの範囲は [docs/README.md](../../README.md) §3 と [code-map.md](../../code-map.md) を見る。
