# 自動走行の安定化 ＆ 30fps 高精度・安定検出の同時成立（作業中）

## 目的と現状判定（2026-11-25 / Round 1）

前段の「手先カメラ NPU 検出 37.6Hz（非空 person）＋ヘッドレス SLAM」は
[arm-cam-live-detections.md](arm-cam-live-detections.md) のとおり達成済み。
ここでは **自動走行しながら 30fps 近傍の検出が途切れず、クラス／枠が安定し、
ゴールが 6/6 成功すること** を目標にする。

**Round 1 判定: 一部 Measured / 一部 ✗ Unverified。** 未達の条件は下記に具体的数値と
理由を書く。完了とは言わない。

## 1 コマンド起動（新規、既定挙動は無変更）

```bash
cd /home/so/Meistar/.worktrees/stable-drive-accurate-vision
export ROS_DOMAIN_ID=5 ROS_LOCALHOST_ONLY=1
export LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe
source /opt/ros/jazzy/setup.bash && source ros/install/setup.bash
ros2 launch umeonigiri_autonomy stable_drive_vision.launch.py \
    world:=maze vision_target:=true
```

- Gazebo は `-s -r`（サーバのみ）。`rviz2` は無く、`gz sim` は 1 プロセスのみ。
- SLAM + Nav2（controller / planner / behavior / bt_navigator / velocity_smoother）+
  手先カメラ + NPU 検出 + Web UI（http://127.0.0.1:8088）が同時に動く。
- NPU ノードの設定は [vision_node_common.py](../../ros/src/umeonigiri_autonomy/launch/vision_node_common.py)
  の 1 箇所。既存 `arm_vision_slam.launch.py` も同じ関数を使う（回帰テストあり）。
- Nav2 の実パラメータは `mapping_nav_params.yaml` であることに注意。
  手掛かりにあった `nav2_params.yaml` は navigation.launch.py からは読まれていない
  （Confirmed: [navigation.launch.py:14](../../ros/src/umeonigiri_autonomy/launch/navigation.launch.py#L14)）。
- 既存 `mapping_nav.launch.py` / `arm_vision_slam.launch.py` の既定挙動・base カメラ・
  Web UI 契約は変更していない（[test_stable_drive_launch.py](../../ros/src/umeonigiri_autonomy/test/test_stable_drive_launch.py)）。

## 実測: 自動走行（Round 1、2 セッション）

| 項目 | 実測 | 判定 | 証拠 |
|---|---|---|---|
| 6 ゴール連続 | **6/6 成功**（2 セッションとも） | Measured | [stability_r1.log](logs/stability_r1.log)、[stability_r2.log](logs/stability_r2.log) |
| 最終誤差 | max **0.183 m**（r1）/ **0.148 m**（r2）、全ゴール ≤0.25 m | Measured | 同上 |
| 所要時間 | 9.0〜24.1 s / ゴール（≤180 s） | Measured | 同上 |
| lifecycle | slam_toolbox / controller_server / planner_server / bt_navigator すべて `active [3]` | Measured | [health_r2.txt](logs/health_r2.txt) |
| プロセス再起動 | launch ログに `process has died` / `exit code` / lifecycle error なし | Measured | [stable_drive_launch_r1.log](logs/stable_drive_launch_r1.log) |
| TF | `tf2_echo map odom` が有効な変換を返す | Measured | [health_r2.txt](logs/health_r2.txt) |
| `/cmd_vel` | 13.2 Hz（記録側の取りこぼしあり）、linear 変化 >0.3 は 0 回、angular 変化 >0.5 は 16 回（要再計測） | ★要確認★ | [drive_r2_summary.json](logs/drive_r2_summary.json) |
| `/arm_camera/image_raw` | **36.835 Hz**（144.6 s） | Measured（≥28） | [drive_r2_summary.json](logs/drive_r2_summary.json) |
| `/detections` | **34.132 Hz**（144.6 s）、ただし **0.246 / 0.292 / 0.351 / 0.360 / 0.378 / 0.428 s の配信空白** | ✗ 条件未達（200 ms 超の空白あり） | 同上、生ログ [drive_r2.jsonl](logs/drive_r2.jsonl) |
| 非空検出 | 走行セッション全体で **0 / 4937**。走行開始直後に 2 → 0 に遷移 | ✗ 条件未達 | [stable_drive_launch_r1.log](logs/stable_drive_launch_r1.log) の yolo 60 フレーム毎ログ |
| ラベル正解率 | 非空が無いため算出不可 | ✗ Unverified | 同上 |
| bbox 中心揺れ | 非空が無いため算出不可 | ✗ Unverified | 同上 |
| 視野内判定 | 保存画像 1065 枚すべてが壁（色ヒストグラム距離 0.834〜0.902、しきい値 0.35）＝対象は視野外 | Measured | [drive_r2_summary.json](logs/drive_r2_summary.json)、画像は `/tmp/drive_images_r2` |
| RTF | 記録窓 20 s×7 の中央値 **0.814**（0.764〜0.873）。別途 simultaneous 計測では **0.628 / 0.709 / 0.734** | ★要確認★（≥0.8 は安定せず） | [drive_r2_summary.json](logs/drive_r2_summary.json)、[health_r2.txt](logs/health_r2.txt) |

画像の出所は前段と同じく `vision_target:=true` の bus.jpg テクスチャ平面であり、
**実カメラの現場認識ではない**。高精度の定義は「Known-object 回帰」であって
COCO mAP ではない（詳細は次ラウンドで計測・明記する）。

## 失敗と原因（Round 1）

| # | 試行 | 結果 | 判定 / 次_actions |
|---|---|---|---|
| 1 | `IncludeLaunchDescription(launch_arguments=dict(...))` | launch が `too many values to unpack` で即終了 | ✗ `.items()` に戻した（[stable_drive_launch_r1.log](logs/stable_drive_launch_r1.log) の初回は失敗ログで上書き済み） |
| 2 | 新規 launch から NPU ノード設定を共有化 | 最初 `Node` import を落とし、既存 `arm_vision_slam.launch.py` が `NameError` | ✗ 回帰テストで検出・修正（先に直すべきだった） |
| 3 | 最初の安定性テストを recorder 無しで実施 | 6/6 は取れたが cmd_vel / detections の生ログが無い | ✗ recorder を修正して再実施 |
| 4 | recorder の `saved += 1` が非ローカル変数 | `UnboundLocalError` で recorder が 1 秒で終了 | ✗ `nonlocal saved` で修正 |
| 5 | 6 ゴールを既定座標で走行 | 走行開始 ~2 s で対象が視野外へ。非空検出 0 | ✗ 次ラウンドで視野内に留まるゴール列（向き指定）を検討 |
| 6 | 1 スレッド recorder で 5 topic を購読 | cmd_vel が 13.2 Hz しか届かず angular の急変を過大評価 | ✗ cmd_vel 専用 recorder を追加する |

## ビルド・テスト（Round 1）

```bash
cd /home/so/Meistar/.worktrees/stable-drive-accurate-vision
PATH=/usr/bin:/bin:/usr/local/bin ./build.sh    # 6 packages finished, exit 0
cd ros && bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
  python3 -m pytest -q src/umeonigiri_autonomy/test/test_stable_drive_launch.py \
                       src/umeonigiri_autonomy/test/test_vision_target.py'
# => 7 passed
```

フル `colcon test` はコミット前に再実行する。

## 次のラウンドでやること

1. 対象が視野内に留まる 6 ゴール（位置＋yaw）を直接 action client / Web UI に投げ、
   走行中 60 s の非空検出・bbox 揺れを実測する。
2. `/detections` の 200 ms 空白を無くす（カメラレート／推論スレッドの調整）。
3. cmd_vel 専用 recorder で急変回数を再計測し、必要なら velocity_smoother を
   安定側へ 1 箇所で調整する。
4. RTF を走行同時セッションで 20 s×3 再計測し、改善策を試した記録を残す。
5. ラベル付き静止画像（person / bus）で class 正解率と IoU を計測し、出所を明記する。
