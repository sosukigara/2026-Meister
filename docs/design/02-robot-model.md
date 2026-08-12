# 設計: ロボットモデル・シミュレーション

> 対象機能: [features/02-robot-model.md](../features/02-robot-model.md) | 作成日: 2026-08-12 | ステータス: 実装済み

## 設計メモ

- ロボット実機のスペック（ホイール径・トレッド・モータ仕様など）が確定したらモデルパラメータとの対応を設計図化する。
- `nav2_params.yaml` の各コントローラ/プランナ設定値の意図（経路品質のチューニング結果）を文書化する価値がある。

## 設計図化対象

- [ ] URDF/xacro の構成図（リンク・ジョイント・TF ツリー）
- [ ] Gazebo プラグイン（VelocityControl / OdometryPublisher）とブリッジトピックの対応表
- [ ] `nav2_params.yaml` の主要パラメータの意図メモ（チューニング結果の記録）

## 関連ドキュメント

- 機能要件: [features/02-robot-model.md](../features/02-robot-model.md)
- 機能分解: [functions/02-robot-model.md](../functions/02-robot-model.md)
- 関連設計: [03-slam.md](03-slam.md)（`/scan` の利用元） / [04-navigation.md](04-navigation.md)（`/cmd_vel` `/odom` の利用元） / [07-esp32-uart.md](07-esp32-uart.md)（実機ではプラグインの役割を ESP32 が代替）
