# Skeleton手型判定アプリ Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 既存YOLOアプリに触らず、新規 `android-skeleton/` に骨格検出＋グー/チョキ/パー判定アプリを作る。

**Architecture:** MediaPipe Tasks Vision `HandLandmarker`（IMAGEモード、最大2手）を `HandLandmarkerDetector` で包み、21点→`JankenClassifier`純粋関数で4値分類し、`SkeletonOverlayView` に骨格＋判定を描く。モデル欠落時は `FakeSkeleton` にfallback。

**Tech Stack:** Kotlin + CameraX 1.3.4 + MediaPipe Tasks Vision（★要確認★：バージョンはTask 1で確定）+ AGP 8.5.2 + Gradle 8.7

**Spec:** [docs/superpowers/specs/2026-11-11-skeleton-janken-design.md](../specs/2026-11-11-skeleton-janken-design.md)

## Global Constraints

- 既存 `android/` は一切変更しない。新規は `android-skeleton/` 以下だけ。
- `applicationId = "com.meister.skeleton"`、`namespace = "com.meister.skeleton"`、`versionCode = 1`、`versionName = "0.1.0"`。
- `minSdk = 29`、`targetSdk = 35`、`compileSdk = 35`、`ndk.abiFilters += "arm64-v8a"`、Java/Kotlin `17`、viewBinding有効。
- 開始点は `origin/android-vision-base` から切った新ブランチ（例：`android-skeleton-base`）。
- モデル `hand_landmarker.task` はリポにコミットしない（DLスクリプト＋gitignore。現行 `*.onnx` と同じ運用）。
- しきい値既定 `0.50`、複数手は最大2手、logcatタグ `MeisterSkeleton` で1Hz要約（`cpu=` `lat=` を含む）。
- HUD追加：cpu（自プロセスCPU%）・lat（撮影→描画の端to端ms）。FPS目標 **25**（実機実測。未達のときだけGPU delegate検討）。
- 判定定数：4指しきい値 `1.1`、親指しきい値 `0.8`（Spec §2の値を verbatim）。
- コメントは WHY のみ（R9）。1ファイル1責務（R1）。

## Review Focus

- フロントカメラの鏡像：反転時に骨格と左右が食い違って見える → Task 4 で既存と同一式を流用し、Task 5 の前後カメラ各 screen.png で固定。
- 手が無いフレーム：クラッシュ／前フレーム残像 → Task 4 の空リスト描画テストで固定。
- 2手同時：2手目が欠落／HUD件数不一致 → Task 3＋4 の2手テストで固定。
- モデル未配置：起動時クラッシュ → Task 5 の欠落起動テスト（Fake fallback）で固定。
- 低端末での推論遅延：ANR／フレーム滞留 → Task 4 の裏スレッド＋KEEP_ONLY_LATEST＋解析解像度640x480上限で固定（実機10分＋FPS表示で確認）。
- CPU%の誤読：全体CPUと自プロセス混同／更新停止 → Task 4 の `/proc/self/stat` 差分＋1Hz更新テストで固定。

---

### Task 1: ブランチ＋足場（gradle・manifest・res・gitignore・モデル取得）

**Files:**
- Create: `android-skeleton/settings.gradle.kts`, `android-skeleton/build.gradle.kts`, `android-skeleton/gradle.properties`, `android-skeleton/gradle/libs.versions.toml`, `android-skeleton/app/build.gradle.kts`, `android-skeleton/app/src/main/AndroidManifest.xml`, `android-skeleton/app/src/main/res/layout/activity_main.xml`, `android-skeleton/app/src/main/res/values/strings.xml`, `android-skeleton/app/src/main/res/values/themes.xml`, `android-skeleton/.gitignore`, `android-skeleton/tools/fetch_model.sh`
- Copy (content verbatim from `origin/android-vision-base:android/...` then rename): gradle wrapper (`gradle/wrapper/*`, `gradlew`, `gradlew.bat`), `ImageUtil.kt`, `FpsCounter.kt`, layout XML, `adb_fetch.sh` 流儀（タグ名だけ `MeisterSkeleton` に）

**Interfaces:**
- Consumes: なし（最初のタスク）。
- Produces: `:app:assembleDebug` が通る空シェル。後続タスクは `com.meister.skeleton` パッケージと `libs.camerax.*`、`libs.tasks.vision` エイリアスを使う。

- [ ] **Step 1: tasks-vision の版とモデルURLを確定する（★要確認★解消）**

Maven Central で `com.google.mediapipe:tasks-vision` の最新安定版を確認し、Task 1 の `libs.versions.toml` に記録する版を決める。記憶の版番号を使わない。モデルは MediaPipe 公式の `hand_landmarker.task` 配布URLを確認する。

- [ ] **Step 2: 新ブランチを切る**

Run: `git checkout -b android-skeleton-base origin/android-vision-base`
Expected: `git branch --show-current` が `android-skeleton-base`。

- [ ] **Step 3: 足場ファイルを作成する**

`settings.gradle.kts` は `rootProject.name = "meister-skeleton"`＋`include(":app")`。`app/build.gradle.kts` の `defaultConfig` は Global Constraints の値 verbatim。`strings.xml` の `app_name` は `MeisterSkeleton`、`themes.xml` は `Theme.MeisterSkeleton`。`.gitignore` は既存内容に `*.task` を加えたもの。`tools/fetch_model.sh` は引数なしで `app/src/main/assets/hand_landmarker.task` に置く。

- [ ] **Step 4: 空シェルがビルドできることを確認する**

Run: `cd android-skeleton && ./gradlew :app:assembleDebug`
Expected: `BUILD SUCCESSFUL`、`app/build/outputs/apk/debug/app-debug.apk` 生成。

- [ ] **Step 5: Commit**

```bash
git add android-skeleton
git commit -m "feat(skeleton): scaffold android-skeleton app shell"
```

### Task 2: `JankenClassifier`（純粋関数＋単体テスト）

**Files:**
- Create: `android-skeleton/app/src/main/java/com/meister/skeleton/Janken.kt`, `android-skeleton/app/src/main/java/com/meister/skeleton/JankenClassifier.kt`
- Test: `android-skeleton/app/src/test/java/com/meister/skeleton/JankenClassifierTest.kt`

**Interfaces:**
- Consumes: なし。
- Produces: `enum class Janken { GOO, CHOKI, PAR, UNKNOWN }`（表示名 `グー/チョキ/パー/不明` を持つ）、`object JankenClassifier { fun classify(points: List<PointF>): Janken }`（`points` は正規化21点、0=手首の順。後続タスクが使う署名）。

- [ ] **Step 1: 失敗テストを書く**

```kotlin
@Test fun goo_allFolded_returnsGoo()
@Test fun choki_indexMiddleOpen_returnsChoki()
@Test fun par_allOpen_returnsPar()
@Test fun oneFinger_returnsUnknown()
```

各テストは合成21点（手首原点、指ごとに折り／開き座標）を作り、`classify()` の戻りが `GOO/CHOKI/PAR/UNKNOWN` であることを assert する。開閉の幾何は Spec §2 の距離比（4指 `1.1`、親指 `0.8`）を境界値で跨ぐ座標にすること。

- [ ] **Step 2: テストが失敗することを確認する**

Run: `cd android-skeleton && ./gradlew :app:testDebugUnitTest --tests "com.meister.skeleton.JankenClassifierTest"`
Expected: FAIL（クラス未定義）。

- [ ] **Step 3: `JankenClassifier.classify(points: List<PointF>): Janken` を実装する**

21点未満の入力は `UNKNOWN`。4指は `dist(先端,手首) > dist(第二関節,手首) * 1.1`、親指は `dist(指先,人差し指付け根) > 指長 * 0.8`。開き本数で GOO（0〜1本）/CHOKI（人差し＋中の2本）/PAR（4〜5本）/UNKNOWN（その他）。Android非依存の純粋関数にすること。

- [ ] **Step 4: テストが通ることを確認する**

Run: `cd android-skeleton && ./gradlew :app:testDebugUnitTest --tests "com.meister.skeleton.JankenClassifierTest"`
Expected: PASS（4テストとも）。

- [ ] **Step 5: Commit**

```bash
git add android-skeleton/app/src/main/java/com/meister/skeleton/Janken*.kt android-skeleton/app/src/test/java/com/meister/skeleton/JankenClassifierTest.kt
git commit -m "feat(skeleton): finger-rule janken classifier with tests"
```

### Task 3: `HandResult`＋`HandLandmarkerDetector`＋`FakeSkeleton`

**Files:**
- Create: `android-skeleton/app/src/main/java/com/meister/skeleton/HandResult.kt`, `android-skeleton/app/src/main/java/com/meister/skeleton/HandLandmarkerDetector.kt`, `android-skeleton/app/src/main/java/com/meister/skeleton/FakeSkeleton.kt`
- Test: `android-skeleton/app/src/test/java/com/meister/skeleton/FakeSkeletonTest.kt`（巡回順の検証）

**Interfaces:**
- Consumes: Task 2 の `Janken`、`JankenClassifier.classify(points: List<PointF>): Janken`。
- Produces: `data class HandResult(val points: List<PointF>, val janken: Janken, val score: Float, val bbox: RectF)`（`points` は正規化21点）、`class HandLandmarkerDetector(context: Context, threshold: Float)`（`fun detectHands(bitmap: Bitmap): List<HandResult>`、`fun label(): String`、`AutoCloseable`）、`class FakeSkeleton`（同署名で仮21点をグー→チョキ→パー巡回）。Task 4 が使う署名。

- [ ] **Step 1: 失敗テストを書く**

```kotlin
@Test fun fake_cyclesGooChokiPar()
```

`FakeSkeleton().detectHands()` を3回呼んで `janken` が `GOO→CHOKI→PAR` の順であることを assert する。

- [ ] **Step 2: テストが失敗することを確認する**

Run: `cd android-skeleton && ./gradlew :app:testDebugUnitTest --tests "com.meister.skeleton.FakeSkeletonTest"`
Expected: FAIL（クラス未定義）。

- [ ] **Step 3: 3ファイルを実装する**

`HandLandmarkerDetector` は Tasks Vision `HandLandmarker`（IMAGEモード、`numHands=2`）を包み、Bitmap→MPImage変換→カテゴリ/ハンドネスscore→`JankenClassifier`→`HandResult`。モデル欠落時は `IllegalStateException("model asset missing: hand_landmarker.task")`（呼び出し側が Fake に落とす）。`score < threshold` の手は捨てる。`FakeSkeleton` は Task 2 のテストと同じ合成点生成を流用し巡回する。

- [ ] **Step 4: テストが通ることを確認する**

Run: `cd android-skeleton && ./gradlew :app:testDebugUnitTest`
Expected: PASS（Task 2＋3の全テスト）。

- [ ] **Step 5: Commit**

```bash
git add android-skeleton/app/src/main/java/com/meister/skeleton/Hand*.kt android-skeleton/app/src/main/java/com/meister/skeleton/FakeSkeleton.kt android-skeleton/app/src/test/java/com/meister/skeleton/FakeSkeletonTest.kt
git commit -m "feat(skeleton): hand landmark detector and fake skeleton"
```

### Task 4: `SkeletonOverlayView`＋`MainActivity` 配線（HUD・logcat・カメラ）

**Files:**
- Create: `android-skeleton/app/src/main/java/com/meister/skeleton/SkeletonOverlayView.kt`, `android-skeleton/app/src/main/java/com/meister/skeleton/MainActivity.kt`, `android-skeleton/app/src/main/java/com/meister/skeleton/CpuMonitor.kt`
- Test: `android-skeleton/app/src/test/java/com/meister/skeleton/CpuMonitorTest.kt`（合成/proc内容での%計算）
- Modify: `android-skeleton/app/src/main/res/layout/activity_main.xml`（overlay のクラス名を `com.meister.skeleton.SkeletonOverlayView` に）

**Interfaces:**
- Consumes: Task 3 の `HandResult`、`HandLandmarkerDetector/FakeSkeleton` の `detectHands(bitmap: Bitmap): List<HandResult>`＋`label()`。
- Produces: 動く画面（プレビュー＋骨格＋判定ラベル＋HUD）。後続は APK のみ使う。

- [ ] **Step 1: オーバーレイの骨格接続を実装する**

`fun setHands(hands: List<HandResult>, imageWidth: Int, imageHeight: Int)`、`fun setMirror(mirror: Boolean)`。接続は MediaPipe 標準（`0-1-2-3-4`、`0-5-6-7-8`、`5-9-13-17-0`、`9-10-11-12`、`13-14-15-16`、`17-18-19-20`）。centerCrop換算（`max(scale)`＋dx/dy）と mirror 反転は既存 `DetectionOverlayView` と同一式。ラベルは「グー 0.92」形式。空リストでクリア。

- [ ] **Step 2: `CpuMonitor` を実装する（テスト先行）**

`class CpuMonitor { fun sample(): Float }` — `/proc/self/stat`（utime+stime）と `/proc/stat`（全体）を読み、前回差分から自プロセスCPU%を返す。初回は `Float.NaN`。読み取り元パスをコンストラクタ注入可能にしてテスト容易にする。`CpuMonitorTest` は合成のstat内容2回分を与えて期待%を assert する。HUD更新は1Hz（毎フレーム読まない）。

- [ ] **Step 3: `MainActivity` を配線する**

既存 `MainActivity.kt` の流儀 verbatim：裏スレッド初期化（失敗時 Fake にfallbackし理由を `hudDet` 表示）、`STRATEGY_KEEP_ONLY_LATEST`、解析解像度は640x480上限（`setTargetResolution(Size(640, 480))` — 25fps寄せの上限。画質優先で上げない）、Bitmap recycle、`FpsCounter`、HUD（det/fps/ms/hands **＋cpu/lat**）、撮影時刻→描画完了の端to端msを `lat` 表示、`Log.i("MeisterSkeleton", ...)` 1Hz（`cpu=` `lat=` 付き）、しきい値スライダ（既定0.50）、前後カメラ切替。

- [ ] **Step 4: ビルド＋単体テストが通ることを確認する**

Run: `cd android-skeleton && ./gradlew :app:testDebugUnitTest :app:assembleDebug`
Expected: 両方 SUCCESS。

- [ ] **Step 5: Commit**

```bash
git add android-skeleton/app/src/main/java/com/meister/skeleton/SkeletonOverlayView.kt android-skeleton/app/src/main/java/com/meister/skeleton/MainActivity.kt android-skeleton/app/src/main/java/com/meister/skeleton/CpuMonitor.kt android-skeleton/app/src/test/java/com/meister/skeleton/CpuMonitorTest.kt android-skeleton/app/src/main/res/layout/activity_main.xml
git commit -m "feat(skeleton): skeleton overlay and camera wiring"
```

### Task 5: モデル配置＋実機検証＋リリースAPK

**Files:**
- Create: `android-skeleton/releases/meister-skeleton-v0.1.0-debug.apk`（ビルド成果物のコピー）
- Modify: なし（`app/src/main/assets/hand_landmarker.task` は git 管理外）

**Interfaces:**
- Consumes: Task 4 までの完成アプリ。
- Produces: 実機で10分安定動作する APK。

- [ ] **Step 1: モデルを配置する**

Run: `./android-skeleton/tools/fetch_model.sh`
Expected: `app/src/main/assets/hand_landmarker.task` が存在する（`git status` に出ないこと）。

- [ ] **Step 2: releaseビルドする**

Run: `cd android-skeleton && ./gradlew :app:assembleDebug`
Expected: BUILD SUCCESSFUL。

- [ ] **Step 3: 実機にインストールする**

Run: `adb devices -l`（`d9a733bc device` を確認）→ `adb install -r android-skeleton/app/build/outputs/apk/debug/app-debug.apk`
Expected: `Success`、`pm list packages` に `com.meister.skeleton`。

- [ ] **Step 4: 実機でグー・チョキ・パー各1枚＋10分安定を確認する**

`adb logcat -d MeisterSkeleton:I '*:S'` に `top=グー/チョキ/パー` が出ること。HUDの `fps` が **10以上**・`cpu` と `lat` が更新されることを screen.png で目視する（未達なら測定値を記録しdelegate検討を次PRに回す。今回PRでは無理に載せない）。`tools/adb_fetch.sh` 流儀で screen.png＋batteryを取得し、グー・チョキ・パー各1枚を保存する。既存 YOLO アプリ（`com.meister.vision`）が併存していること。

- [ ] **Step 5: APKを releases に置いて Commit**

```bash
cp android-skeleton/app/build/outputs/apk/debug/app-debug.apk android-skeleton/releases/meister-skeleton-v0.1.0-debug.apk
git add android-skeleton/releases/meister-skeleton-v0.1.0-debug.apk
git commit -m "feat(skeleton): release v0.1.0 debug apk"
```
