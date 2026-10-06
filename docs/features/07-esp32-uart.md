# 機能: ESP32 ↔ PC UART 通信

> 対象リポジトリ: `sosukigara/2026-Meister` | 作成日: 2026-08-12 | ステータス: 双方向通信の実機確認済み（PWM の実機検証は未）
> 設計: [design/07-esp32-uart.md](../design/07-esp32-uart.md)
> 機能分解: [functions/07-esp32-uart.md](../functions/07-esp32-uart.md)
> 関連ゴール: [G1](../やりたいこと.md#g1-ロッカーボギーロボット自動走行) [G2](../やりたいこと.md#g2-自動でアームでものを回収) [G3](../やりたいこと.md#g3-手動操縦も可能)

## 概要

ESP32 マイコンと PC を UART で接続し、**半分散型アーキテクチャ**でロボットを制御する。
リアルタイム性が要求されるモータ・サーボの PWM 制御は ESP32 が担い、ナビゲーション・画像認識・把持判断などの高度な処理は PC 上の ROS 2 が担う。

```
PC (ROS 2: Nav2 / 画像認識 / Web UI)
  │  UART 双方向通信
ESP32 (モータ・サーボ PWM 制御, センサ入力)
```

## 実装済み機能

- [x] `firmware/main/` の PlatformIO プロジェクト作成（Arduino framework、esp32dev / native 環境）
- [x] バイナリプロトコル定義（固定長フレーム + XOR チェックサム、コマンド 4 種 / フィードバック 2 種）
- [x] モータ PWM・サーボ制御のベース（LEDC PWM、駆動モータ 6ch / ステアリング 6ch / アーム 4ch / グリッパー 1ch）
- [x] コンパイル検証（`pio run`）
- [x] PC 側 ROS 2 シリアルブリッジ（`src/meister_serial_bridge`、`/cmd_vel` → 舵角 + PWM、`/esp32/state` に FB_STATE を配信）
- [x] PC↔ESP32 の双方向通信の実機確認（2026-09-27、USB ケーブルのみで検証できる
      `esp32dev_usbuart` 環境とベンチ確認 CLI `meister_comm_check` を追加）

## 計画中機能

- [ ] リアルタイム制御（PWM 生成・サーボバス制御）の実機検証
- [ ] モック ESP32（PC 側開発用の疑似 ESP32）
