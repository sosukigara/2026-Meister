# 機能: SLAM・地図作成

> 対象リポジトリ: `sosukigara/2026-Meister` | 作成日: 2026-08-12 | ステータス: 実装済み

## 概要

`ros2_autonomous_nav` パッケージで SLAM（slam_toolbox）による環境地図の作成・保存・再利用を提供する。
シミュレーション内のワールドからリアルタイムに占有グリッド地図を生成できる。

## 実装済み機能

- [ ] SLAM 地図作成: `slam_map.launch.py` で Gazebo + slam_toolbox（ライフサイクル configure/activate）を起動し、走行しながら地図を作成できる
- [ ] オンライン非同期 SLAM 設定: `config/mapper_params_online_async.yaml` で非同期 SLAM パラメータを保持する
- [ ] 地図の保存: `save_map.sh` で `map_saver_cli` を実行し、作成した地図を PGM/YAML として保存できる
- [ ] 既製地図: `maps/nav_world.pgm` + `.yaml`（12 m × 12 m, 解像度 0.05 m/px）を同梱する
- [ ] 既知ジオメトリからの地図生成: `generate_map.py` で SDF の壁ジオメトリから直接占有グリッド地図（PGM/YAML）を生成できる（SLAM 不要）
- [ ] 実機向け SLAM 設定: `config/real_mapper_params_online_async.yaml` で実機用パラメータを用意する

## 計画中機能

- （現時点で計画中の機能はありません）

## 仕様化メモ

- SLAM の質（地図再現精度・ループクローズの有効性）の確認方法と数値条件を詳細仕様化する。
- 地図作成時の操作手順（手動テレオペ vs 自動巡回）の確定が必要。
