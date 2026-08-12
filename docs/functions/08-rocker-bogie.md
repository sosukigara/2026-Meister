# 機能分解: 6輪ロッカーボギー・ステアリング・モータ制御

> 要件: [features/08-rocker-bogie.md](../features/08-rocker-bogie.md) | 設計: [design/08-rocker-bogie.md](../design/08-rocker-bogie.md) | 作成日: 2026-08-12

## 機能（このコンポーネントがすること）

- 6輪ロッカーボギーサスペンションで障害物を乗り越える
- ステアリングサーボ **6個**（シリアルバス）で**全輪独立操舵**する
- 駆動 DC モータ **6輪**（PWM）を駆動する
- ESP32 からの指令（目標速度・舵角）を受けて走行する
- 走行モード（前進 / 後進 / 斜行 / 超信地旋回）を実現する

## 関連リンク

- 要件: [features/08-rocker-bogie.md](../features/08-rocker-bogie.md)
- 設計: [design/08-rocker-bogie.md](../design/08-rocker-bogie.md)
