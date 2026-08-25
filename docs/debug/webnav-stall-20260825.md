# Web UI ナビ追従不良 — DWB が微小速度しか選ばずゴール到達失敗

日付: 2026-08-25
状態: **解決済み** — コントローラを DWB から Regulated Pure Pursuit (RPP) へ切替

## 現象
- Web UI からのゴール発行は正常 (`/api/nav` → BasicNavigator まで OK) なのに、
  ロボットがゴールへ追従しない
- `Failed to make progress` (SimpleProgressChecker, 0.5m/10s) が約10秒間隔で発生 →
  follow_path abort → 再計画のループ → Spin/Wait リカバリを消費して `Goal failed`
- ゴールまで 0.3m まで迫った後に失敗するケースもあり

## 証跡と切り分けの過程
| 疑説 | 検証 | 結果 |
|---|---|---|
| Gazebo が遅い (RTF低下) | `/clock` を10s実測 | **RTF=0.996 で否定** |
| 制御計算が重い | controller_server CPU 10s計測 | **~30%/1コアで否定** |
| TF 遅延 | `/tf` rate・年齢実測、ログ grep | map→odom ~44Hz・TFエラー皆無で否定 |
| DWB 軌道サンプリング過大 | `/evaluation` で軌道数確認 | vy_samples:20→1 で 8000→400 に削減(正しいが不十分) |
| BaseObstacle 過大 | critic スコア実測 | **scale 5.0 は nav2 標準 0.02 の250倍。修正で直線部の abort は消えた** |
| スコア勾配の弱さ | vx 別の critic raw を比較 | **スコア地形がほぼ平坦で微小動作が選ばれる**ことが確定 |

### 決定的データ (vx別スコア, GoalDist は前進で増加=勾配が逆/平坦)
```
 vx   | GoalDist PathDist GoalAlign PathAlign total
 0.03 |   31.0      2.0     31.0       5.0    32.60   ← 選ばれる
 0.11 |   32.0      5.0     34.0       8.0    31.60
 0.34 |   42.0     15.0     44.0      18.0    47.60
 0.50 |  (throw)   21.0     50.0      24.0    38.00   ← 遅い軌道と差 0.17 しかない
```
- 実効速度は常に max の ~1/5 (vx 0.03–0.15 / wz 0.03–0.16)
- odom が cmd にピッタリ追従 → アクチュエーション正常 (VelocityControl は運動学的)
- 一時的にフル速度 (vx=0.5, wz=1.0) を出せることも確認 → 系自体は健全

## 根本原因
DWB の軌道スコアリングがこの構成でほぼ平坦になり、**「微小幅の動作」が最良解として
選ばれ続ける**こと。位置ベース critic (GoalDist/PathDist) は旋回に不感、整合 critic
(GoalAlign/PathAlign) の forward_point 0.1m は 0.05m 解像度では ~2セル分の勾配しか
なく、進行方向の大幅修正が必要な場面で旋回がほぼ進まない。10秒で 0.5m 進まないため
progress_checker が毎回 abort し、リカバリを消費して失敗に至る。
加えて BaseObstacle.scale 5.0 (標準の250倍) + inflation cost_scaling 3.0 の太い
コスト帯が速度選好をさらに劣化させていた。

## 解決
`mapping_nav_params.yaml` の FollowPath を **Regulated Pure Pursuit** に変更:
```
plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
use_rotate_to_heading: true, rotate_to_heading_angular_vel: 1.0, ...
```
併せて velocity_smoother の角速度上限 0.6→1.0 / 角加速度 1.0→3.2 に引き上げ。

### 結果
| 項目 | DWB (修正前) | RPP (修正後) |
|---|---|---|
| 単一ゴール (0,0)→(-2,1) | 120s+ で failed | **10s で succeeded** |
| 3ウェイポイント巡回 | 失敗 | **24s で succeeded** |
| Failed to make progress | 14回/120s | **0回** |
| 実効速度 | vx≤0.15, wz≤0.16 | **vx 0.5, wz 1.0 (フル)** |

## 適用した変更
- `mapping_nav_params.yaml`
  - FollowPath: DWB → RPP (use_rotate_to_heading 有効)
  - velocity_smoother: max_velocity/max_accel の角成分を引き上げ
  - (過程で vy_samples 1 / BaseObstacle 0.02 / 標準 critic スケール — 最終構成では
    DWB ブロック自体を削除)

## 残課題・メモ
- `real_nav2_params.yaml` (実機用) にも同種の DWB 設定がある。実機での挙動確認の上、
  同様の RPP 化を検討
- controller_server の制御ループが設計 20Hz に対し実測 5–16Hz の謎は未解明
  (CPU 30% でブロッキング)。RPP では実害が出ていないが、実機移行時に再調査候補
