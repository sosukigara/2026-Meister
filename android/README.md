# MeisterVision (Android)

ROS ノード `meister_vision` の推論をスマホに肩代わりさせるための Android アプリ。v1 基盤。

## v1 の範囲

- スマホ単体で画像認識 + プレビュー表示。通信なし (Q30 合意)。
- Kotlin + CameraX + onnxruntime-mobile 1.18.0。YOLOv8n 640 を同梱 (PC 側と同一モデル)。
- エミュレータ (x86_64) は非対応。APK は arm64 のみ。実機で動かす。
- UI が触るのは `Detector` interface のみ (将来の ROS 送り差し替え口)。

## 開き方

1. Android Studio で `android/` を開く。
2. モデルを配置する (任意・13MB のため git 管理外):
   `mkdir -p app/src/main/assets && cp ../ros/src/meister_vision/models/yolov8n.onnx app/src/main/assets/`
3. 実機 (OPPO A5 2020, Android 10 想定) を USB デバッグで繋いで Run。

モデル未配置でも起動する。その場合は `FakeDetector` (動く仮枠) で配線確認になる。

## 完成定義 (Q30)

実機で起動→プレビュー→Box→FPS/ms 表示→10 分落ちないこと。
FPS が何 Hz でも基盤としては合格。20Hz は tuning 課題。

## 宿題

- Q31: 実機の「設定→端末情報」の Android バージョンと RAM を確定し、minSdk を合わせる。
- tuning: 320 モデル化・NNAPI/GPU delegate・量子化は実測後に判断。
- しきい値の端末既定は 0.50 (PC 側 0.25 では幻検出が出るため。スライダーで変更可)。

## ADB デバッグ (画面の値を PC から抜く)

1. 実機の開発者向けオプションで USB デバッグを ON にする。
2. この PC に USB 接続する (`adb devices` に出ること)。
3. 取得する: `./tools/adb_fetch.sh` (出るのは logcat 要約 + screen.png + battery)。
4. エージェントに渡す: 出力先ディレクトリ (`/tmp/meister-adb-日時`) を教える。
