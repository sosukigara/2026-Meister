# 機能分解: 4軸アーム + カメラ + 画像認識 + 自動把持

> 要件: [features/09-object-grasping.md](../features/09-object-grasping.md) | 設計: [design/09-object-grasping.md](../design/09-object-grasping.md) | 作成日: 2026-08-12

## 機能（このコンポーネントがすること）

- 4軸アーム（肩: DS3225/PWM、肘・手首・グリッパー: STS3215/シリアルバス）を制御する
- 逆運動学（IK）を解いて手先を目標位置に動かす
- 手首のカメラ映像を PC(ROS 2) に取り込む
- 映像から**物体を検出**する（汎用物体検出: YOLO 等）
- 検出結果を Web UI に表示し、**クリック or リスト選択**で対象を選ばせる
- 選択した物体の位置・姿勢を推定して**自動把持**する
- 把持後（持ち上げ → 移動 → 解放）を自動化する

## 関連リンク

- 要件: [features/09-object-grasping.md](../features/09-object-grasping.md)
- 設計: [design/09-object-grasping.md](../design/09-object-grasping.md)
