# 設計: 実機対応

> 対象機能: [features/06-real-robot.md](../features/06-real-robot.md) | 作成日: 2026-08-12 | ステータス: 計画中
> 関連ゴール: [G1](../やりたいこと.md#g1-ロッカーボギーロボット自動走行)

## 設計メモ

- 実機のハードウェア構成（モータ・エンコーダ・LiDAR・マイコン）が確定したら、トピック/座標系/パラメータの対応関係を設計図化する。
- `use_sim_time` の切替、座標系（map/odom/base_link）の整合性を検証項目として詳細仕様化する。

> 注: 実機の具体的なハードウェア構成は 07-10（ESP32 / ロッカーボギー / アーム / 自動制御）に集約される。本ファイルはシミュレーション → 実機の移行パスの設計メモ。

## 設計図化対象

- [ ] 実機のトピック・座標系マッピング（map/odom/base_link とセンサ/モータ配線）
- [ ] `use_sim_time` 切替の影響範囲
- [ ] 検証項目リスト（SLAM・ナビ・ドライバ統合）

## 関連ドキュメント

- 機能要件: [features/06-real-robot.md](../features/06-real-robot.md)
- 機能分解: [functions/06-real-robot.md](../functions/06-real-robot.md)
- 関連設計: [07-esp32-uart.md](07-esp32-uart.md) 〜 [10-auto-control.md](10-auto-control.md)（実機の各コンポーネント）
