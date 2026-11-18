# 機能: ナビゲーション

> 対象リポジトリ: `sosukigara/2026-Meister` | 作成日: 2026-08-12 | ステータス: 実装済み
> 設計: [design/04-navigation.md](../design/04-navigation.md)
> 機能分解: [functions/04-navigation.md](../functions/04-navigation.md)
> 関連ゴール: [G1](../やりたいこと.md#g1-ロッカーボギーロボット自動走行) [G3](../やりたいこと.md#g3-手動操縦も可能)

## 概要

`ros2_autonomous_nav` パッケージの中核機能。Nav2 スタックによる自律ナビゲーションを提供し、
「SLAM で地図を作りながら動く」「既存地図で自己位置推定して動く」の 2 運用モードを持つ。

## 実装済み機能

- [ ] SLAM + ナビ統合: `mapping_nav.launch.py` で robot_state_publisher + シミュレーション + SLAM + Nav2 + Web UI を一括起動し、地図作成とナビゲーションを同時に行える（メインエントリ）
- [ ] 既存地図ナビ: `robot_nav.launch.py` で Gazebo + map_server + AMCL + Nav2 を起動し、保存済み地図上で自己位置推定しながら移動できる
- [ ] Nav2 ノード群: `navigation.launch.py` でコントローラ / プランナ / ビヘイビア / BT ナビゲータ / ベロシティスムーザ / ライフサイクルマネージャを起動できる
- [ ] 目標地点移動: RViz の「2D Nav Goal」指定または `navigate.sh`（`/navigate_to_pose` アクション）で単一目標へ自律移動できる
- [ ] ウェイポイント巡回: `waypoint_follower`（C++ 実装）で複数地点を順に巡回できる
- [ ] ウェイポイント CLI: `waypoints.sh` でスクリプト化された巡回を起動できる
- [ ] 手動操作: `drive.sh`（teleop_twist_keyboard）でキーボードによる手動走行ができる
- [ ] GPU/CPU 自動検出: `nvidia-smi` の有無で GPU/CPU 用 URDF（`my_robot_gpu.urdf` / `my_robot_cpu.urdf`）、LiDAR スペック（360°@10Hz / 180°@5Hz）、Gazebo 描画エンジン（ogre2 系）を自動選択する
- [ ] RViz 表示: `rviz.sh` + `config/nav2_rviz.rviz` で地図・コストマップ・経路・AMCL パーティクルを可視化する

## 追加実装（2026-11）

- [ ] 運用モードの切替: 地図生成（SLAM）と保存地図ナビ（AMCL）を、
      `nav_mode_manager` が各ノードの `/change_state` を直接叩いて切り替える。
      `map→odom` を publish するのは常に 1 ノードだけ
- [ ] Web UI からの操作: モード表示・切替、地図の保存/一覧/削除、初期位置の指定
- [ ] 評価基盤: `bench_nav` が固定ゴールを 3 反復走らせ、成功率・到達精度・
      経路長比・最小クリアランス・接触回数・制御周期・計画時間・ノード別 CPU を JSON に残す
- [ ] 制御器の切り替え: `MEISTER_LAUNCH_ARGS="controller:=mppi"` で MPPI に差し替え可能
      （比較結果は [docs/bench/2026-11-19-mppi.md](../bench/2026-11-19-mppi.md)）
- [ ] 計測結果: [docs/bench/2026-11-18-baseline.md](../bench/2026-11-18-baseline.md)、
      [docs/bench/2026-11-19-tuning.md](../bench/2026-11-19-tuning.md)

## 計画中機能

- （現時点で計画中の機能はありません）
