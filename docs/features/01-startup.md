# 機能: 起動・実行環境

> 対象リポジトリ: `sosukigara/2026-Meister` | 作成日: 2026-08-12 | ステータス: 実装済み
> 設計: [design/01-startup.md](../design/01-startup.md)
> 機能分解: [functions/01-startup.md](../functions/01-startup.md)
> 関連ゴール: [G1](../やりたいこと.md#g1-ロッカーボギーロボット自動走行) [G3](../やりたいこと.md#g3-手動操縦も可能)

## 概要

`start_meister.sh` を実行するだけで、ビルド → Gazebo 起動 → SLAM → Nav2 → Web UI → RViz まで一括起動できるワンショット起動フローを提供する。

## 実装済み機能

- [ ] ワンショット起動: `./start_meister.sh [warehouse|maze|empty]` で環境全体を起動できる
- [ ] ビルド自動化: `colcon build --symlink-install` で対象 3 パッケージ（ros2_autonomous_nav / meistar_description / meister_web_nav）を自動ビルドする
- [ ] ワールド選択: warehouse / maze / empty の 3 ワールドから起動時に選択できる
- [ ] 起動完了待機: `/clock` トピックの出現を最大 90 秒待機し、Gazebo 準備完了を検知してから RViz を起動する
- [ ] クリーン終了: Ctrl+C で関連プロセス（launch / RViz / Gazebo）をまとめて終了する
- [ ] ネットワーク制約対策: VPN 環境でのマルチキャスト障害を回避するため、Gazebo は TCP localhost（`GZ_IP=127.0.0.1`）、ROS 2 DDS は `ROS_LOCALHOST_ONLY=1` に固定する
- [ ] 旧版スクリプト: `scripts/start_mapping_nav.sh` に同フローの旧版が存在する

## 計画中機能

- （現時点で計画中の機能はありません）
