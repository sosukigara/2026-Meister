# 機能分解: Web UI

> 要件: [features/05-web-ui.md](../features/05-web-ui.md) | 設計: [design/05-web-ui.md](../design/05-web-ui.md) | 作成日: 2026-08-12
> 関連ゴール: [G1](../やりたいこと.md#g1-ロッカーボギーロボット自動走行) [G2](../やりたいこと.md#g2-自動でアームでものを回収)

## 機能（このコンポーネントがすること）

- ライブ地図（`/map` → PNG）をブラウザに表示する
- 地図上のクリック/ドラッグでウェイポイント（ヨー角含む）を指定する
- 指定したウェイポイントを Nav2 へ送信し、巡回を開始/キャンセルする
- ナビゲーション状態（待機/巡回中/完了/失敗）を表示する
- REST API（`/api/map` `/api/map.png` `/api/status` `/api/nav` `/api/cancel`）を提供する
- サーバ・地図リスナ・ナビワーカーの 3 スレッドで並行動作する

## 関連リンク

- 要件: [features/05-web-ui.md](../features/05-web-ui.md)
- 設計: [design/05-web-ui.md](../design/05-web-ui.md)
