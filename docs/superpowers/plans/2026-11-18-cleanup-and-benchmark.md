# ナビゲーション整理と計測基盤 実装計画

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `ros2_autonomous_nav` の死んだ資産を片付け、変更の効果を数値で比べられる計測基盤を作り、現状のベースラインを1本の JSON として残す。

**Architecture:** パッケージ直下に散らばったスクリプトを `scripts/` に集約して相対パスの階層ずれを直し、教材由来のファイルを削除する。その後、評価指標を純関数（pytest 可能）として切り出し、`rclpy` のランナーから呼ぶ。指標の計算とゴール検証は ROS に依存させず、単体テストで固定する。

**Tech Stack:** ROS 2 Jazzy, Python 3.12, rclpy, nav2_msgs, pytest

**Spec:** [docs/superpowers/specs/2026-11-18-navigation-design.md](../specs/2026-11-18-navigation-design.md)

> **実装ノート（2026-11-18）**: 実装途中で「Python を避け C++ を原則とする」決定が出たため、
> Task 3・4 の Python 部分は **C++ 実装に置き換わっている**（`bench_metrics` / `bench_nav`、
> gtest で検証）。Task 5 は未実施。詳細は設計書 §12 を参照。

## Global Constraints

- 実機の★測定（舵角サーボ数、幾何、車輪径、ギア比、FOC 配線）が完了するまで PR5/PR6（ファームウェア）に着手しない。
- 計測は GUI（`gz` + RViz）を出したまま、他プロジェクトの ROS プロセスを停止した状態で行う。
- 1ファイル1責務。300行を超えたら分割を検討する（AGENTS.md R1）。
- 設定値は1箇所に定義する（R2）。物理定数は出典付きの表に置く（R3）。
- 未測定値を推測で埋めない（R4）。`★要確認★` を消さない。
- 削除・移動の前に、対象が本当に意図したパスかを絶対パスで確認する。
- コミットはレビュー可能な単位に分ける（R12）。

## Review Focus

- **相対パスの階層ずれ**: スクリプトを1階層深い `scripts/` に移すと `$(dirname "$0")` 起点のパスが全て1つずれる。`rviz.sh` は RViz を出さなくなり、`save_map.sh` は別の場所に地図を書き、`run.sh`/`slam.sh` は launch を見つけられない。
- **`launch.sh` の旧パス参照**: `launch.sh:150` が旧パスを指したままだと、起動は成功するのに RViz だけ出ない。
- **削除した README への参照**: `docs/` からリンクが残るとリンク切れになる。
- **計測の汚染**: 別プロジェクトの Gazebo/RViz が動いていると、CPU と周期の数値が意味を失う。
- **座標系の取り違え**: 軌跡を `odom` で記録すべきところを `map` で記録すると、経路長とクリアランスが別物になる（本計画では `map` を使う。シミュの `/odom` は真値だが、自己位置推定の結果を見るのが目的なので `map` 基準に統一する）。

---

### Task 1: スクリプトを `scripts/` へ移動し、相対パスを直す

**Files:**
- Move: `ros/src/ros2_autonomous_nav/{rviz,run,slam,navigate,save_map,drive,waypoints}.sh` → `ros/src/ros2_autonomous_nav/scripts/`
- Modify: 移動後の各 `.sh`（下記のパス修正）
- Modify: `launch.sh:150`

**Interfaces:**
- Produces: `ros/src/ros2_autonomous_nav/scripts/rviz.sh`（`launch.sh` から実行される唯一のスクリプト）

- [ ] **Step 1: 移動する**

```bash
cd ros/src/ros2_autonomous_nav
git mv rviz.sh run.sh slam.sh navigate.sh save_map.sh drive.sh waypoints.sh scripts/
```

- [ ] **Step 2: 相対パスを修正する（7ファイル、計7箇所）**

`scripts/rviz.sh`:
- `source $(dirname "$0")/../../../install/setup.bash` → `source $(dirname "$0")/../../../../install/setup.bash`
- `-d $(dirname "$0")/config/nav2_rviz.rviz` → `-d $(dirname "$0")/../config/nav2_rviz.rviz`

`scripts/run.sh`:
- `$(dirname "$0")/launch/robot_nav.launch.py` → `$(dirname "$0")/../launch/robot_nav.launch.py`

`scripts/slam.sh`:
- `$(dirname "$0")/launch/slam_map.launch.py` → `$(dirname "$0")/../launch/slam_map.launch.py`

`scripts/waypoints.sh`:
- `$(dirname "$0")/scripts/waypoint_follower.py` → `$(dirname "$0")/waypoint_follower.py`

`scripts/save_map.sh`:
- `mkdir -p maps` → `mkdir -p "$(dirname "$0")/../maps"`
- `-f maps/my_new_map` → `-f "$(dirname "$0")/../maps/my_new_map"`

`scripts/navigate.sh`, `scripts/drive.sh`: パス参照なし。変更不要。

`launch.sh:150`:
- `"$WORKSPACE_ROOT/ros/src/ros2_autonomous_nav/rviz.sh" &` → `"$WORKSPACE_ROOT/ros/src/ros2_autonomous_nav/scripts/rviz.sh" &`

- [ ] **Step 3: 構文と参照を検証する**

```bash
for f in ros/src/ros2_autonomous_nav/scripts/*.sh; do bash -n "$f" || echo "SYNTAX FAIL: $f"; done
grep -rn "ros2_autonomous_nav/rviz.sh\|/scripts/scripts/" --include="*.sh" --include="*.py" --include="*.md" . | grep -v "^./ros/build/\|^./ros/install/"
```
Expected: 構文エラーなし。2つ目の grep が0件（旧パス参照が残っていない）。

- [ ] **Step 4: ビルドが通ることを確認する**

```bash
./build.sh
```
Expected: 6パッケージすべて成功。

- [ ] **Step 5: コミット**

```bash
git add -A ros/src/ros2_autonomous_nav launch.sh
git commit -m "refactor(nav): move helper scripts under scripts/ and fix relative paths"
```

---

### Task 2: 教材由来のファイルを削除し、パッケージのメタデータを直す

**Files:**
- Delete: `ros/src/ros2_autonomous_nav/{README.md,README_JA.md,youtube_description.txt,create_short.py,edit_video.py,create_thumbnail.py,generate_voiceover.py,render_code_snippets.py,start_nav.sh,start_mapping.sh}`
- Modify: `ros/src/ros2_autonomous_nav/setup.py:23-25`
- Modify: `ros/src/ros2_autonomous_nav/package.xml:6-7`

**Interfaces:**
- Consumes: Task 1 の移動結果（`start_nav.sh`/`start_mapping.sh` は移動していないので、未移動のまま削除する）
- Produces: なし

- [ ] **Step 1: 削除対象を確認する**

```bash
cd ros/src/ros2_autonomous_nav
ls README.md README_JA.md youtube_description.txt create_short.py edit_video.py \
   create_thumbnail.py generate_voiceover.py render_code_snippets.py \
   start_nav.sh start_mapping.sh
```
Expected: 10ファイルが表示される。1つでも欠けたら中断して報告する（既に誰かが消している）。

- [ ] **Step 2: 外部参照が無いことを確認する**

```bash
cd /home/so/Meister
grep -rn "README_JA\|youtube_description\|create_short\|edit_video\|create_thumbnail\|generate_voiceover\|render_code_snippets\|start_nav.sh\|start_mapping.sh" \
  --include="*.md" --include="*.sh" --include="*.py" --include="*.yaml" --include="*.txt" . \
  | grep -v "^./ros/build/\|^./ros/install/"
```
Expected: `docs/README.md:73` の「仕様対象外」の記述のみ。それ以外に参照があれば中断して報告する。

- [ ] **Step 3: 削除する**

```bash
cd ros/src/ros2_autonomous_nav
git rm README.md README_JA.md youtube_description.txt \
       create_short.py edit_video.py create_thumbnail.py \
       generate_voiceover.py render_code_snippets.py \
       start_nav.sh start_mapping.sh
```

- [ ] **Step 4: メタデータを更新する**

`setup.py`:
- `maintainer='Ahmed M. Abbas'` → `maintainer='Meister'`
- `maintainer_email='ahmed@example.com'` → `maintainer_email=''`（空文字。個人情報を埋めない）
- `description='Autonomous Navigation with Nav2 on a diff drive robot in Gazebo.'` → `description='Nav2/SLAM integration for the Meister rover (simulation and real machine).'`

`package.xml`:
- `<description>...</description>` → `<description>Nav2, SLAM and launch integration for the Meister rover.</description>`
- `<maintainer email="ahmed@example.com">Ahmed M. Abbas</maintainer>` → `<maintainer email="">Meister</maintainer>`

- [ ] **Step 5: リンク切れが無いことを確認する**

```bash
cd /home/so/Meister
grep -rn "ros2_autonomous_nav/README\|ros2_autonomous_nav/README_JA" docs/ *.md 2>/dev/null
```
Expected: 0件。

- [ ] **Step 6: ビルドが通ることを確認する**

```bash
./build.sh
```
Expected: 6パッケージすべて成功。

- [ ] **Step 7: コミット**

```bash
git add -A ros/src/ros2_autonomous_nav
git commit -m "chore(nav): drop tutorial leftovers and fix package metadata"
```

---

### Task 3: 評価指標の純関数を実装する

**Files:**
- Create: `ros/src/ros2_autonomous_nav/scripts/bench_metrics.py`
- Test: `ros/src/ros2_autonomous_nav/test/test_bench_metrics.py`

**Interfaces:**
- Produces:
  - `path_length(poses: Sequence[Sequence[float]]) -> float` — `[(x, y, yaw), ...]` の累積距離
  - `min_footprint_clearance(poses, occupied: Sequence[tuple[float, float]], half_length: float, half_width: float) -> float` — 各 pose の矩形 footprint と最近傍 occupied セルとの最小距離。`occupied` が空なら `math.inf`
  - `success_rate(results: Sequence[bool]) -> float`
  - `summarize(runs: Sequence[dict]) -> dict` — `runs` は `[{"success": bool, "duration_s": float, "path_length_m": float, "final_error_xy": float, "final_error_yaw": float, "min_clearance_m": float}, ...]`

- [ ] **Step 1: 失敗するテストを書く**

```python
# ros/src/ros2_autonomous_nav/test/test_bench_metrics.py
import math
import pytest

from bench_metrics import (
    min_footprint_clearance, path_length, success_rate, summarize,
)


def test_path_length_sums_segments():
    assert path_length([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0)]) == pytest.approx(2.0)


def test_path_length_of_single_pose_is_zero():
    assert path_length([(0.0, 0.0, 0.0)]) == pytest.approx(0.0)


def test_clearance_is_distance_to_nearest_occupied_cell():
    poses = [(0.0, 0.0, 0.0)]
    assert min_footprint_clearance(poses, [(1.0, 0.0)], 0.30, 0.20) == pytest.approx(0.70)


def test_clearance_accounts_for_yaw():
    poses = [(0.0, 0.0, math.pi / 2)]
    assert min_footprint_clearance(poses, [(0.0, 0.95)], 0.30, 0.20) == pytest.approx(0.65)


def test_clearance_is_infinite_without_obstacles():
    assert min_footprint_clearance([(0.0, 0.0, 0.0)], [], 0.30, 0.20) == math.inf


def test_success_rate_counts_successes():
    assert success_rate([True, True, False, True]) == pytest.approx(0.75)


def test_summarize_reports_counts_and_worst_case():
    runs = [
        {"success": True, "duration_s": 10.0, "path_length_m": 5.0,
         "final_error_xy": 0.05, "final_error_yaw": 0.02, "min_clearance_m": 0.50},
        {"success": False, "duration_s": 30.0, "path_length_m": 9.0,
         "final_error_xy": 0.40, "final_error_yaw": 0.30, "min_clearance_m": 0.08},
    ]
    out = summarize(runs)
    assert out["total"] == 2
    assert out["success_rate"] == pytest.approx(0.5)
    assert out["max_final_error_xy"] == pytest.approx(0.40)
    assert out["min_clearance_m"] == pytest.approx(0.08)
```

- [ ] **Step 2: テストが失敗することを確認する**

```bash
cd ros/src/ros2_autonomous_nav && python3 -m pytest test/test_bench_metrics.py -v
```
Expected: `ModuleNotFoundError: No module named 'bench_metrics'` で FAIL。

- [ ] **Step 3: 実装する**

`scripts/bench_metrics.py` に上記5関数を実装する。footprint は「中心からの半長 `half_length`・半幅 `half_width` の矩形を yaw で回転」とし、occupancy は点として扱う。矩形と点の距離は、点を車体座標系へ回転させた後、各軸で矩形の範囲外にはみ出した分のユークリッド長（内側なら 0）とする。`test/` から `scripts/` を import できるよう、テスト先頭で `sys.path` に `../scripts` を追加する。

- [ ] **Step 4: テストが通ることを確認する**

```bash
cd ros/src/ros2_autonomous_nav && python3 -m pytest test/test_bench_metrics.py -v
```
Expected: 7 passed。

- [ ] **Step 5: コミット**

```bash
git add ros/src/ros2_autonomous_nav/scripts/bench_metrics.py ros/src/ros2_autonomous_nav/test/test_bench_metrics.py
git commit -m "feat(nav): add benchmark metrics helpers with unit tests"
```

---

### Task 4: ゴール定義とベンチ実行ランナーを実装する

**Files:**
- Create: `ros/src/ros2_autonomous_nav/scripts/bench_nav.py`
- Create: `ros/src/ros2_autonomous_nav/config/bench_goals.json`
- Test: `ros/src/ros2_autonomous_nav/test/test_bench_nav.py`

**Interfaces:**
- Consumes: Task 3 の `path_length`, `min_footprint_clearance`, `summarize`
- Produces:
  - `load_goals(path: str) -> dict` — `{"world": str, "repeats": int, "goals": [{"x": float, "y": float, "yaw": float}]}`
  - `validate_goals(doc: dict, is_free: Callable[[float, float], bool]) -> list[str]` — 問題の説明文のリスト。空なら合格
  - CLI: `python3 scripts/bench_nav.py --goals config/bench_goals.json --out docs/bench/<label>.json --repeats 3`

- [ ] **Step 1: 失敗するテストを書く**

```python
# ros/src/ros2_autonomous_nav/test/test_bench_nav.py
import json
import pytest

from bench_nav import load_goals, validate_goals


def _write(tmp_path, doc):
    p = tmp_path / "goals.json"
    p.write_text(json.dumps(doc))
    return str(p)


def test_load_goals_reads_world_and_points(tmp_path):
    doc = {"world": "warehouse", "repeats": 2,
           "goals": [{"x": 1.0, "y": 2.0, "yaw": 0.0}]}
    out = load_goals(_write(tmp_path, doc))
    assert out["world"] == "warehouse"
    assert out["repeats"] == 2
    assert out["goals"][0]["x"] == pytest.approx(1.0)


def test_validate_rejects_goal_inside_obstacle():
    doc = {"world": "warehouse", "repeats": 1,
           "goals": [{"x": 1.0, "y": 2.0, "yaw": 0.0}]}
    problems = validate_goals(doc, lambda x, y: False)
    assert problems == ["goal 0 (1.0, 2.0) is not in free space"]


def test_validate_accepts_free_space_goals():
    doc = {"world": "warehouse", "repeats": 1,
           "goals": [{"x": 1.0, "y": 2.0, "yaw": 0.0}]}
    assert validate_goals(doc, lambda x, y: True) == []


def test_validate_rejects_empty_goal_list():
    doc = {"world": "warehouse", "repeats": 1, "goals": []}
    assert validate_goals(doc, lambda x, y: True) == ["no goals defined"]
```

- [ ] **Step 2: テストが失敗することを確認する**

```bash
cd ros/src/ros2_autonomous_nav && python3 -m pytest test/test_bench_nav.py -v
```
Expected: `ModuleNotFoundError: No module named 'bench_nav'` で FAIL。

- [ ] **Step 3: 実装する**

`scripts/bench_nav.py` は2層に分ける。`load_goals` / `validate_goals` は純関数（rclpy を import しない）。`main()` だけが `rclpy` / `nav2_msgs` / `tf2_ros` を import し、次を行う:

1. `--goals` を読む。`--repeats` があれば上書きする。
2. `/global_costmap/costmap` を購読してコストマップを取り、`is_free = cost < 50` で `validate_goals` を呼ぶ。問題があれば**1ゴールも実行せずに終了コード2で落ちる**。
3. 各ゴールについて `NavigateToPose` アクションを送り、`map` 座標系のロボット姿勢を `/tf` から 10 Hz で記録する。
4. 終了時に経路長・最終誤差・最小クリアランス（Task 3 の関数、occupied セルはコストマップから `cost >= 99` のセル）を計算する。
5. 各 Nav2 ノードの CPU を `/proc/<pid>/stat` から区間積分して記録する。PID は `/proc/*/cmdline` を走査して実行ファイル名で照合する。
6. 結果を JSON で書き出す。`summarize` の結果を同じファイルに `summary` キーで含める。

`config/bench_goals.json` は次の形式で置き、10点は**実行者が RViz で `warehouse` の自由空間をクリックして決める**（値は推測で埋めない）。JSON には決めた値を書く。

```json
{"world": "warehouse", "repeats": 3, "goals": []}
```

- [ ] **Step 4: テストが通ることを確認する**

```bash
cd ros/src/ros2_autonomous_nav && python3 -m pytest test/ -v
```
Expected: Task 3 の7件と本タスクの4件、計11 passed。

- [ ] **Step 5: コミット**

```bash
git add ros/src/ros2_autonomous_nav/scripts/bench_nav.py \
        ros/src/ros2_autonomous_nav/config/bench_goals.json \
        ros/src/ros2_autonomous_nav/test/test_bench_nav.py
git commit -m "feat(nav): add benchmark runner and fixed goal set"
```

---

### Task 5: ベースラインを計測して記録する

**Files:**
- Create: `docs/bench/2026-11-18-baseline.json`
- Create: `docs/bench/2026-11-18-baseline.md`

**Interfaces:**
- Consumes: Task 4 のランナー、`config/bench_goals.json`

- [ ] **Step 1: 計測環境を整える**

```bash
uptime
ps -eo pid,pcpu,args --sort=-pcpu | head -8
```
Expected: load average が 1 以下、`gz sim` / `rviz2` / `component_container` が他プロジェクト由来で動いていない。動いていれば `./kill.sh` ではなく**そのプロジェクトの正規の停止手順**で止め、停止できない場合は計測を中断して報告する。

- [ ] **Step 2: ゴールを決める**

`./launch.sh --sim warehouse` を起動し、RViz で `warehouse` の自由空間から10点を選んで `config/bench_goals.json` に記入する。10点は互いに 1 m 以上離し、壁から 0.5 m 以上離す。

- [ ] **Step 3: 1ゴールで予行する**

```bash
python3 ros/src/ros2_autonomous_nav/scripts/bench_nav.py \
  --goals ros/src/ros2_autonomous_nav/config/bench_goals.json \
  --out /tmp/bench_dryrun.json --repeats 1 --limit 1
```
Expected: 終了コード0、`/tmp/bench_dryrun.json` に `runs` が1件。`summary.success_rate` が 1.0、`min_clearance_m` が 0.10 以上。

- [ ] **Step 4: 本計測する**

```bash
python3 ros/src/ros2_autonomous_nav/scripts/bench_nav.py \
  --goals ros/src/ros2_autonomous_nav/config/bench_goals.json \
  --out docs/bench/2026-11-18-baseline.json --repeats 3
```
Expected: 30 run が記録される。

- [ ] **Step 5: サマリを Markdown に書く**

`docs/bench/2026-11-18-baseline.md` に、JSON の `summary` を表として転記し、次の6項目を必ず含める: 成功率、到達誤差の最大値、経路長比（走行距離 ÷ 計画経路長）、最小クリアランスの最小値、制御周期 p99、Nav2 ノードごとの CPU%。数値の横に「計測日時・負荷平均・GUI 有無」を書く。

- [ ] **Step 6: コミット**

```bash
git add docs/bench/2026-11-18-baseline.json docs/bench/2026-11-18-baseline.md \
        ros/src/ros2_autonomous_nav/config/bench_goals.json
git commit -m "docs(nav): record navigation baseline (30 runs)"
```

---

## Self-Review

**Spec coverage**（[設計書](../specs/2026-11-18-navigation-design.md) との対応）:
- §8.1 の合格基準 → Task 3（指標の定義）と Task 5（値の記録）で実装
- §8.2 の手順 → Task 4（ランナー）と Task 5（実行）
- §9 の PR1 → Task 1, 2
- §9 の PR2 → Task 3, 4, 5
- §9 の PR3（パラメータ最適化）→ 本計画の対象外。Task 5 のベースラインを入力として別計画で扱う
- §9 の PR4〜PR7 → 本計画の対象外（PR4 は別計画、PR5/PR6 は★測定待ち）

**未解決の前提**: `config/bench_goals.json` の10点は実行者が RViz で決める。推測値は入れない（R4）。

**型の一貫性**: `bench_metrics.py` の関数シグネチャは Task 3 の Interfaces に固定し、Task 4 はそれをそのまま呼ぶ。
