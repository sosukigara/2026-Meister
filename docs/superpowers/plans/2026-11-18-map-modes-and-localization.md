# 保存地図ナビとモード切替 実装計画

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 地図生成モードと、保存地図上での自己位置推定モードを Web UI から切り替えられるようにし、走行中に地図が伸び続ける現状を解消する。

**Architecture:** `map→odom` を publish できるのは常に1つという不変条件を、`nav_mode_manager` ノードが守る。slam_toolbox / map_server / AMCL は起動時にすべて立ち上げ、lifecycle の configure まで済ませて inactive で待たせ、切替時に「旧を deactivate → 新を activate」の順で遷移させる。Web UI は HTTP ハンドラからこのノードのサービスを呼ぶだけにする。

**Tech Stack:** ROS 2 Jazzy, C++17, rclcpp, nav2_msgs, lifecycle_msgs, gtest, 既存の meister_web_nav HTTP サーバ

**Spec:** [docs/superpowers/specs/2026-11-18-navigation-design.md](../specs/2026-11-18-navigation-design.md) §6.4 / §6.5 / D-5〜D-8

## Global Constraints

- コードは C++ を原則とする（設計書 D-16）。launch ファイルのみ Python。
- `map→odom` を同時に publish してよいのは1ノードだけ（設計書 D-6）。
- 地図生成 → 保存 → 自己位置推定の**一方通行**のみ対応。逆方向は launch 再起動（§6.4）。
- AMCL は初期位置が未設定の間、ゴール送信を受け付けない（§6.4）。
- 地図の保存形式は `maps/<name>.yaml` + `<name>.pgm`（`nav2_map_server` 標準）。
- 未測定値を推測で埋めない（R4）。ノード名やトピック名は実ファイルで確認する。
- 1ファイル1責務・300行超で分割検討（R1）。設定は1箇所（R2）。

## Review Focus

- **二重の `map→odom`**: 切替中に旧ノードの deactivate が完了する前に新ノードを activate すると、TF が2系統になって自己位置が飛ぶ。
- **inactive のままの slam_toolbox に `/scan` が溜まる**: configure 済みで購読している場合、切替までメモリとCPUを使い続ける。
- **地図名の衝突・パストラバーサル**: UI から来た名前で `../` を許すと任意パスに書き込める。
- **AMCL 初期位置なしのゴール**: 自己位置未確定のまま走ると、ゴールが別の場所として解釈される。
- **切替失敗の握り潰し**: サービスが失敗を返したのに UI が「切替済み」と表示すると、以後の全操作が嘘になる。

---

### Task 1: モード状態機械（純ロジック）

**Files:**
- Create: `ros/src/ros2_autonomous_nav/include/ros2_autonomous_nav/nav_mode.hpp`
- Create: `ros/src/ros2_autonomous_nav/src/nav_mode.cpp`
- Test: `ros/src/ros2_autonomous_nav/test/test_nav_mode.cpp`

**Interfaces:**
- Produces:
  - `enum class NavMode { kMapping, kLocalization };`
  - `enum class MapToOdomOwner { kSlamToolbox, kAmcl, kNone };`
  - `struct ModeTransition { bool allowed; MapToOdomOwner deactivate_first; MapToOdomOwner activate_after; std::string reason; };`
  - `ModeTransition PlanTransition(NavMode from, NavMode to);`
  - `NavMode ModeForOwner(MapToOdomOwner owner);`

- [ ] **Step 1: 失敗するテストを書く**

```cpp
// ros/src/ros2_autonomous_nav/test/test_nav_mode.cpp
#include <gtest/gtest.h>
#include "ros2_autonomous_nav/nav_mode.hpp"

namespace ros2_autonomous_nav {
namespace {

TEST(NavMode, MappingModeIsOwnedBySlamToolbox) {
  EXPECT_EQ(ModeForOwner(MapToOdomOwner::kSlamToolbox), NavMode::kMapping);
}

TEST(NavMode, LocalizationModeIsOwnedByAmcl) {
  EXPECT_EQ(ModeForOwner(MapToOdomOwner::kAmcl), NavMode::kLocalization);
}

TEST(NavMode, SwitchingToTheOtherModeDeactivatesBeforeActivating) {
  const auto plan = PlanTransition(NavMode::kMapping, NavMode::kLocalization);
  EXPECT_TRUE(plan.allowed);
  EXPECT_EQ(plan.deactivate_first, MapToOdomOwner::kSlamToolbox);
  EXPECT_EQ(plan.activate_after, MapToOdomOwner::kAmcl);
}

TEST(NavMode, SwitchingBackIsNotAllowed) {
  // 設計書 §6.4: 保存地図 → 地図生成 は launch 再起動。グラフの継続挙動が未定義のため。
  const auto plan = PlanTransition(NavMode::kLocalization, NavMode::kMapping);
  EXPECT_FALSE(plan.allowed);
  EXPECT_EQ(plan.deactivate_first, MapToOdomOwner::kNone);
}

TEST(NavMode, SwitchingToTheSameModeIsANoOp) {
  const auto plan = PlanTransition(NavMode::kMapping, NavMode::kMapping);
  EXPECT_FALSE(plan.allowed);
  EXPECT_EQ(plan.deactivate_first, MapToOdomOwner::kNone);
  EXPECT_EQ(plan.activate_after, MapToOdomOwner::kNone);
}

}  // namespace
}  // namespace ros2_autonomous_nav
```

- [ ] **Step 2: 失敗を確認する**

Run: `colcon build --packages-select ros2_autonomous_nav && colcon test --packages-select ros2_autonomous_nav`
Expected: `nav_mode.hpp` が無いためコンパイル失敗。

- [ ] **Step 3: 実装する**

上記の型と関数を実装する。`PlanTransition` は表引き1つで表現し、`reason` に日本語の短い説明を入れる（UI のメッセージに使う）。

- [ ] **Step 4: 通ることを確認する**

Run: `colcon test --packages-select ros2_autonomous_nav`
Expected: 既存 52 + 新規 5 が全て pass。

- [ ] **Step 5: コミット**

```bash
git add ros/src/ros2_autonomous_nav/include/ros2_autonomous_nav/nav_mode.hpp \
        ros/src/ros2_autonomous_nav/src/nav_mode.cpp \
        ros/src/ros2_autonomous_nav/test/test_nav_mode.cpp
git commit -m "feat(nav): add map-odom ownership state machine"
```

---

### Task 2: `nav_mode_manager` ノード

**Files:**
- Create: `ros/src/ros2_autonomous_nav/src/nav_mode_manager_main.cpp`
- Modify: `ros/src/ros2_autonomous_nav/CMakeLists.txt`
- Modify: `ros/src/ros2_autonomous_nav/package.xml`（`std_srvs`, `lifecycle_msgs`）

**Interfaces:**
- Consumes: Task 1 の `PlanTransition` / `NavMode`
- Produces:
  - サービス `switch_to_localization` (`std_srvs/srv/Trigger`) → `success=true` で切替完了、`message` に理由
  - サービス `get_mode` (`std_srvs/srv/Trigger`) → `message` が `"mapping"` / `"localization"`
  - トピック `/nav_mode` (`std_msgs/msg/String`) を遷移時に publish

- [ ] **Step 1: ノードを実装する**

`nav2_lifecycle_manager/srv/ManageLifecycleNodes` ではなく、`lifecycle_msgs/srv/ChangeState` を各ノードへ直接送る（各ノード名のサービス `/slam_toolbox/change_state`、`/amcl/change_state`、`/map_server/change_state`）。遷移順:
1. 旧ノードに `TRANSITION_DEACTIVATE`、成功を確認
2. 新ノードに `TRANSITION_ACTIVATE`、成功を確認
3. 失敗したら**元に戻す**（新を deactivate、旧を activate）し、`success=false` を返す
4. `/nav_mode` を publish

- [ ] **Step 2: 手動で動作確認する**

Run: `ros2 run ros2_autonomous_nav nav_mode_manager --ros-args -p autostart:=false`
Expected: サービスが現れる（`ros2 service list | grep switch_to_localization`）。

- [ ] **Step 3: コミット**

```bash
git commit -am "feat(nav): add nav_mode_manager node"
```

---

### Task 3: launch を「全部 inactive で待たせる」形にする

**Files:**
- Modify: `ros/src/ros2_autonomous_nav/launch/navigation.launch.py`
- Modify: `ros/src/ros2_autonomous_nav/launch/mapping_nav.launch.py`
- Modify: `ros/src/ros2_autonomous_nav/config/mapping_nav_params.yaml`（amcl, map_server を有効化）

**Interfaces:**
- Consumes: Task 2 のノード
- Produces: `slam_toolbox` が active、`amcl`/`map_server` が inactive の初期状態

- [ ] **Step 1: navigation.launch.py に map_server / amcl / nav_mode_manager を足し、lifecycle_manager の node_names を更新する**
- [ ] **Step 2: `ros2 launch` して状態を確認する**

Run: `ros2 lifecycle get /amcl` → `inactive`
Run: `ros2 lifecycle get /slam_toolbox` → `active`
Expected: 上記2つが満たされる。

- [ ] **Step 3: コミット**

---

### Task 4: 地図ストア（純ロジック）

**Files:**
- Create: `ros/src/ros2_autonomous_nav/include/ros2_autonomous_nav/map_store.hpp`
- Create: `ros/src/ros2_autonomous_nav/src/map_store.cpp`
- Test: `ros/src/ros2_autonomous_nav/test/test_map_store.cpp`

**Interfaces:**
- Produces:
  - `bool IsValidMapName(const std::string& name);` — 英数・`_`・`-` のみ、1〜64文字。`..` と `/` を拒否（トラバーサル対策）
  - `std::string DefaultMapName(const std::string& world, const std::string& timestamp);` → `<world>_<timestamp>`
  - `std::vector<std::string> ListMaps(const std::string& dir);` — `.yaml` を除いた名前を昇順で返す

- [ ] **Step 1: 失敗するテストを書く**（`../` 、空文字、64文字超、`.yaml` の混在）
- [ ] **Step 2: 失敗を確認する**
- [ ] **Step 3: 実装する**
- [ ] **Step 4: 通ることを確認する**
- [ ] **Step 5: コミット**

---

### Task 5: Web UI API（モード・地図・初期位置）

**Files:**
- Modify: `ros/src/meister_web_nav/include/meister_web_nav/http_app.hpp`
- Modify: `ros/src/meister_web_nav/src/http_app.cpp`
- Modify: `ros/src/meister_web_nav/src/web_nav_server.cpp`
- Test: `ros/src/meister_web_nav/test/test_http_app.cpp`（既存パターンに追記）

**Interfaces:**
- Consumes: Task 2 のサービス、Task 4 の `IsValidMapName`
- Produces（HTTP）:
  - `GET /api/mode` → `{"mode":"mapping"|"localization"}`
  - `POST /api/mode` `{"mode":...}` → サービス呼び出し。失敗時は 409 と理由
  - `GET /api/maps` → `{"maps":["..."]}`
  - `POST /api/map/save` `{"name":...}` → `nav2_msgs/srv/SaveMap` を呼ぶ。名前不正は 400
  - `POST /api/initialpose` `{"x":...,"y":...,"yaw":...}` → `/initialpose` を publish
  - `DELETE /api/maps/<name>` → ファイル削除。名前不正は 400

- [ ] **Step 1: テストを書く**（ルーティングとバリデーション。ROS サービスは既存のテスト用スタブに合わせて差し替え可能にする）
- [ ] **Step 2: 失敗を確認する**
- [ ] **Step 3: 実装する**
- [ ] **Step 4: 通ることを確認する**

Run: `colcon test --packages-select meister_web_nav`
Expected: 全 pass。

- [ ] **Step 5: コミット**

---

### Task 6: Web UI のボタン

**Files:**
- Modify: `ros/src/meister_web_nav/webui/index.html`
- Modify: `ros/src/meister_web_nav/webui/js/app.js`
- Modify: `ros/src/meister_web_nav/webui/js/nav_status.js`（モード表示）

- [ ] **Step 1: モード表示と切替ボタン、地図保存ボタン、地図選択を追加する**
- [ ] **Step 2: `POST /api/mode` の失敗を UI に出す**（成功時だけ「切替済み」と表示する）
- [ ] **Step 3: 手動確認する**（`./launch.sh --sim warehouse` → ブラウザ）

---

### Task 7: 通し検証

- [ ] **Step 1: 地図生成 → 保存 → 切替 → 初期位置 → ナビ**を1回通す
- [ ] **Step 2: 結果を `docs/bench/` に記録する**
- [ ] **Step 3: `docs/features/04-navigation.md` のチェックボックスを実態に合わせる**
