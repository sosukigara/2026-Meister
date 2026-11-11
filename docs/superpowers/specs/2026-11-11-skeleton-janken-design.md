# 設計: 骨格検出専用アプリ（手のグー・チョキ・パー識別）

> 作成日: 2026-11-11 | ステータス: 設計合意済み（実装は未着手）
> 前提ブランチ: `origin/android-vision-base`（YOLOアプリ `android/`）。既存は触らない。
> 実装開始点: `origin/android-vision-base` から新ブランチ（例：`android-skeleton-base`）を切って `android-skeleton/` を追加する。
> 要望: 「改造じゃなくて、新しく骨格検出用のアプリを作る」「じゃんけんはしなくてよくて、いま写ってる手がグーちょきぱーどれかを検出できればいい」
> 追加要望（2026-11-11）: HUDにCPU使用率とリアルタイム性（遅延）を表示する。10FPSで動作させる（2026-11-11に25→10へ変更）。

## 背景

- 既存 `android/` は YOLOv8n 物体検出＋枠＋FPS HUD（`Detector` interface のみを UI が触る構成）。
- ROS 側 `meister_vision` には `hand_landmark_detector`（21点・ONNX契約 `★要確認★`）があるがモデル自体はリポに無い。
- 方式は A（MediaPipe HandLandmarker＋指開閉ルール）で合意。B（ROS ONNX移植）はモデル探しからで重い、C（YOLO再学習）は骨格が出ないため不採用。

## 決定事項（2026-11-11 合意）

| # | 決定 | 理由 |
|---|---|---|
| D1 | **新規アプリ**として `android-skeleton/` に作る。既存 `android/` は一切触らない | 動くものを壊さない。YOLOアプリと同居させる |
| D2 | applicationId `com.meister.skeleton`、version 0.1.0/code 1、minSdk 29・arm64のみ | 既存（`com.meister.vision`）と並行インストール可能に。端末想定（OPPO A5 2020）は既存踏襲 |
| D3 | 依存は CameraX＋`tasks-vision` のみ。onnxruntime は入れない | 骨格は MediaPipe に一本化し、APK肥大とABI問題を避ける |
| D4 | 判定は **21点→グー/チョキ/パー/不明の純粋関数** `JankenClassifier` | Snapdragon 665でも回る軽さ。テスト容易 |
| D5 | YOLO系（`YoloDetector`・`YoloPostprocess`・320モデル）は持ち込まない | 骨格専用のため。削除PRも今回は出さない（既存側は無変更） |
| D6 | **HUDにCPU使用率・遅延・FPSを表示し、10FPSを目標にする** | 操作入力に使うなら体感遅延と処理落ちが見える必要がある。CPU%は自プロセス、遅延は撮影→描画の端 to 端 |
| D7 | **delegateはNPU→GPU→CPUの順に試す** | NPU非対応（実機で作成例外→GPUへフォールバック確認）。GPUで手なし18・手あり14FPSを確認 |

## 構成

```
android-skeleton/
  settings.gradle.kts
  build.gradle.kts
  gradle.properties / gradle/wrapper/ / gradlew
  gradle/libs.versions.toml        # CameraX + tasks-vision
  app/build.gradle.kts             # namespace/applicationId=com.meister.skeleton
  app/src/main/AndroidManifest.xml # MainActivity(portrait) + CAMERA権限
  app/src/main/java/com/meister/skeleton/
    MainActivity.kt                # 既存流用：preview+analyze+HUD+1Hz logcat
    HandLandmarkerDetector.kt      # 新規：HandLandmarkerラッパ
    JankenClassifier.kt            # 新規：純粋関数
    HandResult.kt                  # 新規：21点＋判定＋bbox＋score
    SkeletonOverlayView.kt         # 新規：骨格線＋点＋判定ラベル（centerCrop＋mirror流用）
    CpuMonitor.kt                  # 新規：自プロセスCPU使用率（/proc/self/stat＋/proc/stat差分、1Hz更新）
    FpsCounter.kt                  # 流用（fps＋推論ms。撮影→描画の端to端遅延はMainActivityで計測しHUD表示）
    ImageUtil.kt                   # 流用（toRgbBitmap）
    FakeSkeleton.kt                # 新規：配線確認用。グー→チョキ→パー巡回の仮21点
  app/src/main/res/...             # layout/themes/strings流用
  app/src/test/.../JankenClassifierTest.kt
  tools/fetch_model.sh             # hand_landmarker.task取得（gitignore、APK同梱運用）
  releases/meister-skeleton-v0.1.0-debug.apk
```

## 判定ロジック（§2合意）

- 21点の並びは MediaPipe 規約（0=手首、1-4=親指、5-8=人差し指、9-12=中指、13-16=薬指、17-20=小指。ROS側想定と一致）。
- 4指の開閉：「先端〜手首の距離 ＞ 第二関節〜手首の距離 ×1.1」なら開き。角度計算なし。
- 親指：「親指先端〜人差し指付け根の距離 ＞ 人差し指長 ×0.8」なら開き。
- 分類：開き本数で決定。0〜1本=グー、2本（人差し＋中）=チョキ、4〜5本=パー、それ以外=不明。信頼度は HandLandmarker の score をそのまま使う。

## データフロー＋エラー処理（§3合意）

- フロー：`CameraX → Bitmap → HandLandmarkerDetector.detect() → overlay.setHands() ＋ HUD`。Bitmap recycle・ミラー・centerCrop換算・`MeisterVision`改め `MeisterSkeleton` タグの1Hz logcatは既存流儀。
- 手なし：正常系。空リスト→クリア＋「0 hands」。
- 複数手：最大2手（`numHands=2`）。HUD件数は手の数、logcat `top=` は先頭1手。
- モデル無し：`assets/hand_landmarker.task` が無ければ `FakeSkeleton` に fallback し、HUDに理由表示（既存 YOLO→FAKE と同じ）。
- 推論例外：1フレーム捨てて継続。`onDestroy` の executor 終了待ち＋closeも既存通り。
- しきい値スライダ流用：意味は「この score 未満の手は捨てる」。既定 0.50 のまま。

## テスト＋完成定義（§4合意）

- 単体：`JankenClassifierTest`（合成21点で4値を検証）＋既存流儀の `./gradlew testDebugUnitTest`。
- 実機：既存 Q30 流用。「起動→プレビュー→骨格＋グー/チョキ/パー表示→10分落ちない」。加えてHUDのFPSが **10以上**（実機で手なし約18・手あり約14FPSを確認。320x240解析。30fpsはSDM665の天井外）。
- HUD追加：cpu（自プロセスCPU%）・lat（撮影→描画の端to端ms）。fps/ms/handsは既存通り。logcat要約にも `cpu=` `lat=` を足す。
- 確認物：`adb_fetch.sh` 流儀の logcat＋screen.png＋battery。グー・チョキ・パー各1枚。
- やらないこと：ROS送信、勝敗判定、自動回転対応。量子化は実測後に判断（D7のdelegate検討に含む）。
