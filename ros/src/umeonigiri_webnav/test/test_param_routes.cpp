// SPDX-License-Identifier: MIT
/// パラメータ API のテスト。実サーバーを ephemeral port で立て、実クライアントで流す。
///
/// ここで見るのは「HTTP 面が値を取り違えないこと」と「失敗理由を捨てないこと」。
/// 実際の ROS ノードに対する get/set は ParamWorker 側の責務で、この test は
/// 起動したシミュレーションで別途確認する（ROS 無しで HTTP を検証する方針は
/// test_http_app.cpp と同じ）。
#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <httplib.h>
#include <nlohmann/json.hpp>

#include "stub_param_source.hpp"
#include "umeonigiri_webnav/http_app.hpp"
#include "umeonigiri_webnav/ports.hpp"

namespace umeonigiri_webnav {
namespace {

using json = nlohmann::json;

class StubMapSource : public MapSource {
 public:
  std::optional<json> Metadata() override { return std::nullopt; }
  std::optional<MapImage> RenderPng() override { return std::nullopt; }
  std::optional<json> RobotPose() override { return std::nullopt; }
};

class StubNavSink : public NavSink {
 public:
  json Status() const override { return json{{"state", "idle"}}; }
  void Submit(const std::vector<Waypoint>&) override {}
  void Cancel() override {}
};

class StubVisionSource : public VisionSource {
 public:
  json Status() override { return json{{"monitor", true}, {"has_frame", false}}; }
  std::optional<VisionFrame> Latest(long, double) override { return std::nullopt; }
  void SetEnabled(bool) override {}
};

class StubModeSink : public ModeSink {
 public:
  std::string Mode() override { return "mapping"; }
  std::string SwitchToLocalization() override { return {}; }
  std::string SwitchToMapping() override { return {}; }
  std::vector<std::string> ListMaps() override { return {}; }
  std::string SaveMap(const std::string&) override { return {}; }
  std::string DeleteMap(const std::string&) override { return {}; }
  void SetInitialPose(double, double, double) override {}
};

struct Reply {
  int status = 0;
  std::string body;

  json Json() const { return json::parse(body, nullptr, false); }
};

Reply ToReply(const httplib::Result& result) {
  if (!result) {
    return Reply{};
  }
  return Reply{result->status, result->body};
}

class HttpServer {
 public:
  HttpServer() {
    // 本番と同じ順序で登録する。パラメータ経路を /.* の後ろに置くと
    // 静的配信に食われるため、この test が順序の退行を捕まえる。
    RegisterRoutes(server_, std::filesystem::path(UMEONIGIRI_WEBUI_DIR), map_, nav_,
                   vision_, mode_, params_);
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

  test::StubParamSource& params() { return params_; }
  Reply Get(const std::string& path) { return ToReply(client_->Get(path)); }
  Reply Post(const std::string& path, const std::string& body) {
    return ToReply(client_->Post(path, body, "application/json"));
  }

 private:
  httplib::Server server_;
  StubMapSource map_;
  StubNavSink nav_;
  StubVisionSource vision_;
  StubModeSink mode_;
  test::StubParamSource params_;
  int port_ = 0;
  std::thread thread_;
  std::unique_ptr<httplib::Client> client_;
};

TEST(ParamRoutesNodes, ListsNodesWithoutBeingShadowedByStaticFiles) {
  HttpServer server;

  const Reply reply = server.Get("/api/params/nodes");

  ASSERT_EQ(reply.status, 200);
  const json body = reply.Json();
  ASSERT_TRUE(body.contains("nodes"));
  ASSERT_EQ(body["nodes"].size(), 2u);
  EXPECT_EQ(body["nodes"][0], "/amcl");
  EXPECT_EQ(body["nodes"][1], "/controller_server");
}

TEST(ParamRoutesList, ReturnsNameTypeAndValue) {
  HttpServer server;

  const Reply reply = server.Get("/api/params?node=%2Fcontroller_server");

  ASSERT_EQ(reply.status, 200);
  const json body = reply.Json();
  EXPECT_EQ(body["node"], "/controller_server");
  ASSERT_TRUE(body["parameters"].is_array());
  ASSERT_EQ(body["parameters"].size(), 2u);
  EXPECT_EQ(body["parameters"][0]["name"], "enabled");
  EXPECT_EQ(body["parameters"][0]["type"], "bool");
  EXPECT_EQ(body["parameters"][0]["value"], true);
  EXPECT_EQ(body["parameters"][1]["name"], "desired_linear_vel");
  EXPECT_EQ(body["parameters"][1]["type"], "double");
  EXPECT_DOUBLE_EQ(body["parameters"][1]["value"].get<double>(), 0.5);
  EXPECT_EQ(server.params().last_list_node, "/controller_server");
}

TEST(ParamRoutesList, MissingNodeIs400) {
  HttpServer server;

  const Reply reply = server.Get("/api/params");

  EXPECT_EQ(reply.status, 400);
  EXPECT_TRUE(reply.Json().contains("error"));
}

TEST(ParamRoutesList, BackendErrorIsPassedThrough) {
  HttpServer server;
  server.params().list_error = "ノード /ghost のパラメータサービスが見つかりません";

  const Reply reply = server.Get("/api/params?node=%2Fghost");

  EXPECT_EQ(reply.status, 409);
  EXPECT_EQ(reply.Json()["error"],
            "ノード /ghost のパラメータサービスが見つかりません");
}

TEST(ParamRoutesSet, PostsTypedValueAndReturnsActual) {
  HttpServer server;

  const Reply reply = server.Post(
      "/api/params",
      json{{"node", "/controller_server"},
           {"name", "desired_linear_vel"},
           {"value", 0.75}}
          .dump());

  ASSERT_EQ(reply.status, 200);
  EXPECT_EQ(reply.Json()["status"], "ok");
  EXPECT_DOUBLE_EQ(reply.Json()["value"].get<double>(), 0.75);
  EXPECT_EQ(server.params().last_set_node, "/controller_server");
  EXPECT_EQ(server.params().last_set_name, "desired_linear_vel");
  EXPECT_DOUBLE_EQ(server.params().last_set_value.get<double>(), 0.75);
}

TEST(ParamRoutesSet, BackendRejectionReasonIsReturned) {
  HttpServer server;
  server.params().set_error = "パラメータ use_sim_time は読み取り専用です";

  const Reply reply = server.Post(
      "/api/params",
      json{{"node", "/amcl"}, {"name", "use_sim_time"}, {"value", false}}.dump());

  EXPECT_EQ(reply.status, 409);
  EXPECT_EQ(reply.Json()["error"], "パラメータ use_sim_time は読み取り専用です");
}

TEST(ParamRoutesSet, MissingFieldsAre400) {
  HttpServer server;

  EXPECT_EQ(server.Post("/api/params", json{{"node", "/amcl"}}.dump()).status, 400);
  EXPECT_EQ(server.Post("/api/params", json{{"name", "x"}, {"value", 1}}.dump()).status,
            400);
  EXPECT_EQ(server.Post("/api/params", "not json").status, 400);
}

}  // namespace
}  // namespace umeonigiri_webnav
