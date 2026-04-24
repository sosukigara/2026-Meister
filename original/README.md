# Meistar 自律移動ロボット プロジェクト

このリポジトリは、ROS 2 Jazzy と Gazebo Harmonic を使用した自律移動ロボット **Meistar** のシミュレーションおよび実機開発環境です。

## 快速スタート (Quick Start)

### 1. 環境構築
以下のスクリプトを実行して、必要なパッケージをインストールしてください。
```bash
# 詳細は docs/setup_guide.md を参照
./docs/setup_guide.md 
```

### 2. ビルド
```bash
colcon build --symlink-install
source install/setup.bash
```

### 3. シミュレーション起動
Gazebo、SLAM、Nav2、およびキーボード操作用のターミナルを一度に立ち上げます。
```bash
./start_all.sh
```

---

## ドキュメント一覧 (Documentation)

| 文書 | 内容 |
| :--- | :--- |
| [セットアップガイド](docs/setup_guide.md) | ROS 2 Jazzy 環境の構築、micro-ROS の設定 |
| [ナビゲーション解説](docs/navigation.md) | Nav2 (自律走行) の仕組みと設定について |

---

## フォルダ構成と役割

- `src/meistar_description/` : メインパッケージ
    - `urdf/` : ロボットの3Dモデルとセンサー定義
    - `worlds/` : Gazebo のシミュレーション環境
    - `config/` : Nav2, SLAM, Bridge の設定
    - `launch/` : システム起動スクリプト
- `kill_all.sh` : 暴走したプロセスや残ったプロセスを強制終了します
- `start_all.sh` : ビルドから起動までを自動化します

---

## よくある質問 (FAQ) / トラブルシューティング

### 画面がカクつく、または「Jump back in time」と出る
- **原因**: シミュレーションの計算負荷が高すぎるか、時計の同期がズレています。
- **対策**: `./kill_all.sh` を実行して一度リセットしてください。また、LiDARの解像度を下げるなどの調整が有効です。

### センサーの赤い光が邪魔
- `urdf/robot.urdf.xacro` 内の `<visualize>true</visualize>` を `false` に書き換えることで非表示にできます。
