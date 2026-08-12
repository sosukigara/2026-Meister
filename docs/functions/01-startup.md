# 機能分解: 起動・実行環境

> 要件: [features/01-startup.md](../features/01-startup.md) | 設計: [design/01-startup.md](../design/01-startup.md) | 作成日: 2026-08-12

## 機能（このコンポーネントがすること）

- ビルド → Gazebo → SLAM → Nav2 → Web UI → RViz を一括起動する
- 起動時にワールド（warehouse / maze / empty）を選択する
- `/clock` の出現を検知して Gazebo 準備完了を判定する
- Ctrl+C で関連プロセス（launch / RViz / Gazebo）をまとめて終了する
- VPN 環境でも動くよう Gazebo / DDS を localhost に固定する

## 関連リンク

- 要件: [features/01-startup.md](../features/01-startup.md)
- 設計: [design/01-startup.md](../design/01-startup.md)
