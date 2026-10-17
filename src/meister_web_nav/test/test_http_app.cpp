// SPDX-License-Identifier: MIT
/// HTTP 面のテスト。実サーバーを ephemeral port で立て、実クライアントで流す。
///
/// MapListener は ROS グラフ (rclcpp::init と購読登録) を要するので、HTTP が触る口
/// (MapSource / NavSink) は stub が満たす。ただし地図は本物の MapCache、メタデータは
/// 本物の MetadataOf を使う。ロジックを複製しないので、本物の経路を検証できる。
///
/// - 地図が変わらない連続リクエストで 2 回目が 304
/// - 地図を変えると 200 かつ別の bytes
/// - /api/map の JSON 形状が変わっていないこと (app.js が依存している)
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <httplib.h>
#include <nlohmann/json.hpp>

#include "meister_web_nav/http_app.hpp"
#include "meister_web_nav/map_cache.hpp"
#include "meister_web_nav/ports.hpp"

namespace meister_web_nav {
namespace {

using json = nlohmann::json;
using Grid = nav_msgs::msg::OccupancyGrid;

constexpr const char* kStrongTag = "\"923a0a5ea697ccb5\"";

bool HasPngMagic(const std::string& body) {
  return body.size() >= 4 && body.compare(0, 4, "\x89PNG", 4) == 0;
}

Grid::ConstSharedPtr MakeGrid(int size = 32, int wall_row = 10) {
  auto grid = std::make_shared<Grid>();
  grid->info.width = static_cast<uint32_t>(size);
  grid->info.height = static_cast<uint32_t>(size);
  grid->info.resolution = 0.05;
  grid->info.origin.position.x = -1.0;
  grid->info.origin.position.y = -2.0;
  grid->data.assign(static_cast<std::size_t>(size) * size, 0);
  if (wall_row >= 0) {
    for (int c = 0; c < size; ++c) {
      grid->data[static_cast<std::size_t>(wall_row) * size + c] = 100;
    }
  }
  return grid;
}

std::vector<std::string> KeysOf(const json& object) {
  std::vector<std::string> keys;
  for (auto it = object.begin(); it != object.end(); ++it) {
    keys.push_back(it.key());
  }
  std::sort(keys.begin(), keys.end());
  return keys;
}

/// 応答を値で持つ。httplib::Result は接続失敗で null になりうるので、test 側で
/// ->status を直接読むと未定義動作になる。status == 0 は「応答が無い」。
struct Reply {
  int status = 0;
  std::string body;
  httplib::Headers headers;

  std::string Header(const std::string& key) const {
    const auto found = headers.find(key);
    return found == headers.end() ? std::string() : found->second;
  }
};

Reply ToReply(const httplib::Result& result) {
  if (!result) {
    return Reply{};
  }
  return Reply{result->status, result->body, result->headers};
}

/// ROS グラフ無しで MapSource を満たす。地図と crop は本物の MapCache に任せる。
class StubMapSource : public MapSource {
 public:
  void SetGrid(Grid::ConstSharedPtr grid) { cache_.SetGrid(std::move(grid)); }
  MapCache::Stats stats() const { return cache_.stats(); }

  std::optional<json> Metadata() override {
    const MapSnapshot snapshot = cache_.Snapshot();
    if (!snapshot.grid) {
      return std::nullopt;
    }
    return MetadataOf(*snapshot.grid, snapshot.bounds);
  }
  std::optional<std::pair<std::string, std::string>> RenderPng() override {
    return cache_.RenderPng();
  }
  std::optional<json> RobotPose() override { return std::nullopt; }

 private:
  MapCache cache_;
};

class StubNavSink : public NavSink {
 public:
  json Status() const override { return json{{"state", "idle"}, {"message", "stub"}}; }
  void Submit(const std::vector<Waypoint>& waypoints) override { submitted = waypoints; }
  void Cancel() override { canceled = true; }

  std::vector<Waypoint> submitted;
  bool canceled = false;
};

class HttpServer {
 public:
  HttpServer() {
    RegisterRoutes(server_, std::filesystem::path(MEISTER_WEBUI_DIR), source_, sink_);
    port_ = server_.bind_to_any_port("127.0.0.1");
    thread_ = std::thread([this] { server_.listen_after_bind(); });
    client_ = std::make_unique<httplib::Client>("127.0.0.1", port_);
  }
  ~HttpServer() {
    server_.stop();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  HttpServer(const HttpServer&) = delete;
  HttpServer& operator=(const HttpServer&) = delete;

  StubMapSource& source() { return source_; }
  StubNavSink& sink() { return sink_; }

  Reply Get(const std::string& path) { return ToReply(client_->Get(path)); }
  Reply Get(const std::string& path, const httplib::Headers& headers) {
    return ToReply(client_->Get(path, headers));
  }
  Reply GetWithTag(const std::string& path, const std::string& tag) {
    return Get(path, httplib::Headers{{"If-None-Match", tag}});
  }
  Reply Post(const std::string& path, const std::string& body) {
    return ToReply(client_->Post(path, body, "application/json"));
  }

 private:
  httplib::Server server_;
  StubMapSource source_;
  StubNavSink sink_;
  int port_ = 0;
  std::thread thread_;
  std::unique_ptr<httplib::Client> client_;
};

TEST(HttpAppConditionalGet, UnchangedMapYields304) {
  HttpServer server;
  server.source().SetGrid(MakeGrid());

  const Reply first = server.Get("/api/map.png");
  ASSERT_EQ(first.status, 200);
  const std::string etag = first.Header("ETag");
  ASSERT_FALSE(etag.empty());
  EXPECT_TRUE(HasPngMagic(first.body));
  EXPECT_EQ(first.Header("Cache-Control"), "no-cache");

  const Reply second = server.GetWithTag("/api/map.png", etag);
  EXPECT_EQ(second.status, 304);
  EXPECT_TRUE(second.body.empty());
  EXPECT_EQ(second.Header("ETag"), etag);
}

TEST(HttpAppConditionalGet, EmittedTagIsRfcQuoted) {
  // RFC 9110 8.8.3: entity-tag は引用符付き。引用符無しだと規格に準ずるクライアント
  // (または tag を正規化する proxy) では 304 が成立しない。
  HttpServer server;
  server.source().SetGrid(MakeGrid());

  const Reply response = server.Get("/api/map.png");
  const std::string etag = response.Header("ETag");
  ASSERT_GE(etag.size(), 2u);
  EXPECT_EQ(etag.front(), '"');
  EXPECT_EQ(etag.back(), '"');
  EXPECT_FALSE(etag.substr(1, etag.size() - 2).empty());
  EXPECT_EQ(etag.find(' '), std::string::npos);
}

TEST(HttpAppConditionalGet, RfcFormsAllMatch) {
  // 強引用符 / weak / 混在リスト / 重複ヘッダ行 / 旧形式の未引用符。
  HttpServer server;
  server.source().SetGrid(MakeGrid());
  const Reply first = server.Get("/api/map.png");
  const std::string etag = first.Header("ETag");
  ASSERT_FALSE(etag.empty());
  const std::string raw = etag.substr(1, etag.size() - 2);

  EXPECT_EQ(server.GetWithTag("/api/map.png", etag).status, 304);
  EXPECT_EQ(server.GetWithTag("/api/map.png", "W/" + etag).status, 304);
  EXPECT_EQ(server.GetWithTag("/api/map.png", "  W/" + etag + "  ").status, 304);
  EXPECT_EQ(server.GetWithTag("/api/map.png", "\"other\", " + etag).status, 304);
  EXPECT_EQ(server.GetWithTag("/api/map.png", "\"other\", W/\"x\", " + etag).status, 304);
  EXPECT_EQ(server.GetWithTag("/api/map.png", "*").status, 304);
  EXPECT_EQ(server.GetWithTag("/api/map.png", raw).status, 304);
  EXPECT_EQ(server.Get("/api/map.png", httplib::Headers{{"If-None-Match", "\"other\""},
                                                        {"If-None-Match", etag}})
                .status,
            304);
}

TEST(HttpAppConditionalGet, StaleTagYieldsFullBody) {
  // 不一致は 200 + 本文。「常に 304」にすると地図が白紙で固まる。
  HttpServer server;
  server.source().SetGrid(MakeGrid());

  for (const std::string stale :
       {"\"deadbeefdeadbeef\"", "deadbeefdeadbeef", "W/\"deadbeefdeadbeef\"", "\"0\""}) {
    const Reply response = server.GetWithTag("/api/map.png", stale);
    EXPECT_EQ(response.status, 200) << stale;
    EXPECT_TRUE(HasPngMagic(response.body)) << stale;
    EXPECT_EQ(response.Header("Content-Type"), "image/png") << stale;
  }
}

TEST(HttpAppConditionalGet, ChangedMapYieldsNewBytes) {
  HttpServer server;
  server.source().SetGrid(MakeGrid());
  const Reply first = server.Get("/api/map.png");
  ASSERT_EQ(first.status, 200);

  server.source().SetGrid(MakeGrid(32, 20));
  const Reply second = server.GetWithTag("/api/map.png", first.Header("ETag"));
  EXPECT_EQ(second.status, 200);
  EXPECT_NE(second.Header("ETag"), first.Header("ETag"));
  EXPECT_NE(second.body, first.body);
  EXPECT_TRUE(HasPngMagic(second.body));
}

TEST(HttpAppConditionalGet, NotModifiedDoesNotRecomputePng) {
  HttpServer server;
  server.source().SetGrid(MakeGrid());

  const Reply first = server.Get("/api/map.png");
  ASSERT_EQ(first.status, 200);
  EXPECT_EQ(server.source().stats().crop_calls, 1u);
  EXPECT_EQ(server.source().stats().encode_calls, 1u);

  for (int i = 0; i < 5; ++i) {
    EXPECT_EQ(server.GetWithTag("/api/map.png", first.Header("ETag")).status, 304);
  }
  EXPECT_EQ(server.source().stats().crop_calls, 1u);
  EXPECT_EQ(server.source().stats().encode_calls, 1u);

  server.source().SetGrid(MakeGrid(32, 20));
  EXPECT_EQ(server.GetWithTag("/api/map.png", first.Header("ETag")).status, 200);
  EXPECT_EQ(server.source().stats().crop_calls, 2u);
  EXPECT_EQ(server.source().stats().encode_calls, 2u);
}

TEST(HttpAppConditionalGet, RepeatPollsStayNotModified) {
  HttpServer server;
  server.source().SetGrid(MakeGrid());
  const std::string etag = server.Get("/api/map.png").Header("ETag");
  ASSERT_FALSE(etag.empty());
  for (int i = 0; i < 5; ++i) {
    EXPECT_EQ(server.GetWithTag("/api/map.png", etag).status, 304);
  }
}

TEST(HttpAppConditionalGet, NoStoreIsGone) {
  // no-store のままだとブラウザが If-None-Match を送らない。
  HttpServer server;
  server.source().SetGrid(MakeGrid());
  EXPECT_NE(server.Get("/api/map.png").Header("Cache-Control"), "no-store");
}

TEST(HttpAppConditionalGet, QueryStringStillRoutes) {
  // ?t= 付きでも同じ route (旧 cache-buster との互換)。
  HttpServer server;
  server.source().SetGrid(MakeGrid());
  const Reply first = server.Get("/api/map.png?t=12345");
  ASSERT_EQ(first.status, 200);
  EXPECT_EQ(server.GetWithTag("/api/map.png?t=99999", first.Header("ETag")).status, 304);
}

TEST(HttpAppConditionalGet, NoMapIs404) {
  HttpServer server;
  EXPECT_EQ(server.Get("/api/map.png").status, 404);
}

TEST(HttpAppMapJson, MetadataKeys) {
  HttpServer server;
  server.source().SetGrid(MakeGrid(20));

  const Reply response = server.Get("/api/map");
  ASSERT_EQ(response.status, 200);
  EXPECT_EQ(response.Header("Content-Type"), "application/json");

  const json payload = json::parse(response.body);
  EXPECT_TRUE(payload.at("has_map").get<bool>());
  EXPECT_EQ(payload.at("image_url"), "/api/map.png");
  EXPECT_EQ(KeysOf(payload), (std::vector<std::string>{"has_map", "height", "image_url",
                                                       "origin", "resolution", "width"}));
  EXPECT_EQ(KeysOf(payload.at("origin")), (std::vector<std::string>{"x", "y"}));
}

TEST(HttpAppMapJson, NoMapShape) {
  HttpServer server;
  const Reply response = server.Get("/api/map");
  ASSERT_EQ(response.status, 200);
  // 波括弧の初期化子は macro 引数の区切りに見えるので丸括弧で包む。
  EXPECT_EQ(json::parse(response.body), (json{{"has_map", false}}));
}

TEST(EtagMatchesUnit, StrongMatches) {
  EXPECT_TRUE(EtagMatches(std::string(kStrongTag), kStrongTag));
}

TEST(EtagMatchesUnit, StaleDoesNotMatch) {
  EXPECT_FALSE(EtagMatches(std::string("\"deadbeefdeadbeef\""), kStrongTag));
  EXPECT_FALSE(EtagMatches(std::string("923a0a5ea697ccb"), kStrongTag));
  EXPECT_FALSE(EtagMatches(std::string("\"923a0a5ea697ccb5x\""), kStrongTag));
  EXPECT_FALSE(EtagMatches(std::string("x923a0a5ea697ccb5"), kStrongTag));
}

TEST(EtagMatchesUnit, WeakMatchesStrong) {
  // RFC 9110 13.1.2: If-None-Match は weak 比較。weakness marker は ABNF 由来で
  // 大文字小文字を区別しない。
  EXPECT_TRUE(EtagMatches(std::string("W/") + kStrongTag, kStrongTag));
  EXPECT_TRUE(EtagMatches(std::string("w/") + kStrongTag, kStrongTag));
  EXPECT_TRUE(EtagMatches(std::string("  W/  ") + kStrongTag + "  ", kStrongTag));
}

TEST(EtagMatchesUnit, StarMatches) {
  EXPECT_TRUE(EtagMatches(std::string("*"), kStrongTag));
}

TEST(EtagMatchesUnit, ListWithWhitespace) {
  EXPECT_TRUE(
      EtagMatches(std::string("\"aaaaaaaaaaaaaaa\", W/") + kStrongTag, kStrongTag));
  EXPECT_TRUE(EtagMatches(std::string("W/") + kStrongTag + ",\"a\",\"b\"", kStrongTag));
}

TEST(EtagMatchesUnit, LegacyUnquotedForm) {
  // 引用符無しは旧サーバーが送っていた形。理由は http_app.cpp の EtagMatches。
  const std::string raw = "923a0a5ea697ccb5";
  EXPECT_TRUE(EtagMatches(raw, kStrongTag));
  EXPECT_TRUE(EtagMatches("W/" + raw, kStrongTag));
  EXPECT_TRUE(EtagMatches("\"other\", " + raw, kStrongTag));
}

TEST(EtagMatchesUnit, AbsentHeader) {
  EXPECT_FALSE(EtagMatches(std::nullopt, kStrongTag));
  EXPECT_FALSE(EtagMatches(std::string(""), kStrongTag));
}

TEST(EtagMatchesUnit, UnparsableElements) {
  EXPECT_FALSE(EtagMatches(std::string("\"\""), kStrongTag));
  EXPECT_FALSE(EtagMatches(std::string("W/"), kStrongTag));
  EXPECT_FALSE(EtagMatches(std::string(",,"), kStrongTag));
  EXPECT_FALSE(EtagMatches(std::string("\"unclosed"), kStrongTag));
}

TEST(EtagMatchesUnit, EntityTagIsQuoted) {
  EXPECT_EQ(EntityTag("deadbeef"), "\"deadbeef\"");
  EXPECT_NE(EntityTag("deadbeef"), "deadbeef");
}

TEST(HttpAppNavApi, StatusIsPassedThrough) {
  HttpServer server;
  const Reply response = server.Get("/api/status");
  ASSERT_EQ(response.status, 200);
  EXPECT_EQ(json::parse(response.body).at("state"), "idle");
}

TEST(HttpAppNavApi, SubmitValidWaypoints) {
  HttpServer server;
  const Reply response = server.Post(
      "/api/nav", R"({"waypoints":[{"x":1.5,"y":-2,"yaw":0.5},{"x":"3","y":4}]})");
  ASSERT_EQ(response.status, 200);
  const json payload = json::parse(response.body);
  EXPECT_EQ(payload.at("status"), "submitted");
  EXPECT_EQ(payload.at("count"), 2);

  ASSERT_EQ(server.sink().submitted.size(), 2u);
  EXPECT_DOUBLE_EQ(server.sink().submitted[0].x, 1.5);
  EXPECT_DOUBLE_EQ(server.sink().submitted[0].y, -2.0);
  EXPECT_DOUBLE_EQ(server.sink().submitted[0].yaw, 0.5);
  // 数値文字列は Python 版の float(value) と同じく受け、yaw 省略は 0 を向く。
  EXPECT_DOUBLE_EQ(server.sink().submitted[1].x, 3.0);
  EXPECT_DOUBLE_EQ(server.sink().submitted[1].y, 4.0);
  EXPECT_DOUBLE_EQ(server.sink().submitted[1].yaw, 0.0);
}

TEST(HttpAppNavApi, RejectsBadBodies) {
  HttpServer server;
  for (const std::string body : {"", "not json", "{}", R"({"waypoints":[]})",
                                 R"({"waypoints":[{"y":1}]})",
                                 R"({"waypoints":[{"x":"a","y":1}]})"}) {
    const Reply response = server.Post("/api/nav", body);
    EXPECT_EQ(response.status, 400) << body;
    EXPECT_FALSE(json::parse(response.body).at("error").get<std::string>().empty())
        << body;
  }
  EXPECT_TRUE(server.sink().submitted.empty());
}

TEST(HttpAppNavApi, CancelRequestsCancellation) {
  HttpServer server;
  const Reply response = server.Post("/api/cancel", "");
  ASSERT_EQ(response.status, 200);
  EXPECT_EQ(json::parse(response.body), (json{{"status", "cancel_requested"}}));
  EXPECT_TRUE(server.sink().canceled);
}

TEST(HttpAppStaticFiles, ServesExplicitAssetsAndRejectsTraversal) {
  HttpServer server;
  const Reply index = server.Get("/");
  ASSERT_EQ(index.status, 200);
  EXPECT_EQ(index.Header("Content-Type"), "text/html; charset=utf-8");
  EXPECT_NE(index.body.find("html"), std::string::npos);

  EXPECT_EQ(server.Get("/app.js").status, 200);
  EXPECT_EQ(server.Get("/style.css").status, 200);
  // webui_dir の外へ出る経路と、無いファイルは 404。
  EXPECT_EQ(server.Get("/nope.js").status, 404);
  EXPECT_EQ(server.Get("/../AGENTS.md").status, 404);
}

}  // namespace
}  // namespace meister_web_nav
