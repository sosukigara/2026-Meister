# 機能分解: 自動制御（自動移動 → 完全自律）

> 要件: [features/10-auto-control.md](../features/10-auto-control.md) | 設計: [design/10-auto-control.md](../design/10-auto-control.md) | 作成日: 2026-08-12
> 関連ゴール: [G1](../やりたいこと.md#g1-ロッカーボギーロボット自動走行) [G2](../やりたいこと.md#g2-自動でアームでものを回収)

## 機能（このコンポーネントがすること）

- LD-D500 LiDAR の `/scan` で SLAM 地図作成・自己位置推定を行う
- Nav2 で**自動走行**する（既存シミュレーション資産の実機移植）
- Web UI で指定したウェイポイントを**自動巡回**する
- 把持対象が可動域外なら**台車を移動させて接近**してから把持する（09 との連携）
- 完全自律（自動巡回 + 障害物回避 + 物体回収）を統合する

## 関連リンク

- 要件: [features/10-auto-control.md](../features/10-auto-control.md)
- 設計: [design/10-auto-control.md](../design/10-auto-control.md)
