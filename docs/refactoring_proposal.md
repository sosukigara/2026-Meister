# ワークスペースのリファクタリング案 (構成の整理)

現在の `Meistar` ワークスペースには、複数の `build/install` フォルダが混在していたり、ROS 2 の標準的なフォルダ構成から外れている箇所があります。これらを整理して、管理しやすくするためのリファクタリング案をまとめました。

## 1. 提案する新しいフォルダ構成 (Standard ROS 2 Layout)

```bash
Meistar/
├── src/                          # 全てのソースコードをここに集約
│   ├── meistar_description/      # ロボットのモデル・見た目定義 (Xacro, Meshes)
│   └── ros2_autonomous_nav/      # ナビゲーション設定・Launch・Params
├── docs/                         # ドキュメント類を集約
│   ├── agent.md
│   ├── migration_guide.md
│   └── (その他のマニュアル)
├── scripts/                      # 起動スクリプトやツール類
│   ├── start_mapping_nav.sh
│   └── rviz.sh
├── build/                        # (自動生成) 統合されたビルドフォルダ
├── install/                      # (自動生成)
└── log/                          # (自動生成)
```

## 2. 主な変更点とメリット

### A. ソースコードを `src/` に集約
*   **現状**: `ros2-autonomous-nav` がルートにあり、`meistar_description` が `original/src/` の奥深くにあります。
*   **変更**: 全てのパッケージを `src/` 直下に並べます。
*   **メリット**: `colcon build` 一発で全てのパッケージが整合性を保ってビルドできるようになります。

### B. 重複・不要なフォルダの削除
*   **現状**: ルートと `original/` の両方に `build`, `install`, `log` が存在します。
*   **変更**: `original/` 内のビルド済みフォルダを削除し、ワークスペース全体を1つに統合します。

### C. ROS 2 命名規則への適合
*   **変更**: フォルダ名 `ros2-autonomous-nav` を `ros2_autonomous_nav` (ハイフンからアンダースコア) に変更します。
*   **メリット**: ROS 2 パッケージ名として標準的な形式になり、ツール類でのエラーを防げます。

### D. URDF/Xacro の一元管理
*   **現状**: `ros2-autonomous-nav` 内にコピーされた `.urdf` と、`meistar_description` 内の `.xacro` が混在しています。
*   **変更**: `meistar_description` 内の Xacro をマスターとし、Launchファイルから直接 Xacro を呼び出すように修正します。

## 3. リファクタリング実行の手順 (案)

1.  **バックアップの作成**: 現在の状態を一旦保存。
2.  **新フォルダ作成**: `mkdir -p src docs scripts`
3.  **移動**:
    *   `mv original/src/meistar_description src/`
    *   `mv ros2-autonomous-nav src/ros2_autonomous_nav`
    *   `mv *.md docs/` (agent.md など)
    *   `mv *.sh scripts/`
4.  **掃除**: `rm -rf original build install log` (古いビルドデータを削除)
5.  **再ビルド**: `colcon build --symlink-install`

---

この構成案について、進めてもよろしいでしょうか？あるいは「特定のファイルはこの場所に残したい」といったご要望があれば調整いたします。
