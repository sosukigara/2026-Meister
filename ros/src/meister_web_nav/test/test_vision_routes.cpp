// SPDX-License-Identifier: MIT
/// 認識モニターの経路のテスト。実サーバーを ephemeral port で立て、実クライアントで
/// 流す (test_http_app.cpp と同じ流儀)。
///
/// 経路は RegisterRoutes 経由で登録する。視覚の経路は /.* の静的配信より前に
/// 登録する必要があり、順序を間違えると 404 になる。ここで実地に確かめる。
#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <httplib.h>
#include <nlohmann/json.hpp>

#include "meister_web_nav/http_app.hpp"
#include "meister_web_nav/ports.hpp"

namespace meister_web_nav {
namespace {

using json = nlohmann::json;

/// 認識画像だけを差し替えられる stub。地図と航行は空で足りる。
class StubVisionSource : public VisionSource {
 public:
  json Status() override { return status_; }
  std::optional<VisionFrame> Latest(long after_seq, double) override {
    if (!has_frame_ || after_seq >= seq_) {
      // 実装は 1 秒待つ。stub が即座に返すと httplib の応答ループが空転する。
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      return std::nullopt;
    }
    return VisionFrame{seq_, jpeg_};
  }
  void SetEnabled(bool enabled) override {
    enabled_ = enabled;
    status_["monitor"] = enabled;
  }

  bool enabled() const { return enabled_; }
  void SetFrame(std::string jpeg) {
    jpeg_ = std::move(jpeg);
    has_frame_ = true;
    ++seq_;
    status_["has_frame"] = true;
  }

 private:
  json status_ = json{{"monitor", true}, {"has_frame", false}};
  std::string jpeg_;
  long seq_ = 0;
  bool has_frame_ = false;
  bool enabled_ = true;
};

class StubMapSource : public MapSource {
 public:
  std::optional<json> Metadata() override { return std::nullopt; }
  std::optional<std::pair<std::string, std::string>> RenderPng() override {
    return std::nullopt;
  }
  std::optional<json> RobotPose() override { return std::nullopt; }
};

class StubNavSink : public NavSink {
 public:
  json Status() const override { return json{{"state", "idle"}}; }
  void Submit(const std::vector<Waypoint>&) override {}
  void Cancel() override {}
};

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

class HttpServer {
 public:
  HttpServer() {
    RegisterRoutes(server_, std::filesystem::path(MEISTER_WEBUI_DIR), map_, nav_, vision_);
    port_ = server_.bind_to_any_port("127.0.0.1");
    thread_ = std::thread([this] { server_.listen_after_bind(); });
    client_ = std::make_unique<httplib::Client>("127.0.0.1", port_);
    client_->set_read_timeout(5, 0);
  }
  ~HttpServer() {
    server_.stop();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  HttpServer(const HttpServer&) = delete;
  HttpServer& operator=(const HttpServer&) = delete;

  StubVisionSource& vision() { return vision_; }
  Reply Get(const std::string& path) { return ToReply(client_->Get(path)); }
  Reply Post(const std::string& path, const std::string& body) {
    return ToReply(client_->Post(path, body, "application/json"));
  }
  /// ストリームは終わらないので、必要なバイトが来た時点で受信を打ち切る。
  std::string ReadStreamUntil(const std::string& path, const std::string& needle) {
    std::string received;
    client_->Get(path, [&received, &needle](const char* data, std::size_t length) {
      received.append(data, length);
      return received.find(needle) == std::string::npos;
    });
    return received;
  }

 private:
  httplib::Server server_;
  StubMapSource map_;
  StubNavSink nav_;
  StubVisionSource vision_;
  int port_ = 0;
  std::thread thread_;
  std::unique_ptr<httplib::Client> client_;
};

TEST(VisionRoutesStatus, ServesStatusWithUrls) {
  HttpServer server;

  const Reply reply = server.Get("/api/vision");

  ASSERT_EQ(reply.status, 200);
  EXPECT_EQ(reply.Header("Content-Type"), "application/json");
  const json payload = json::parse(reply.body);
  EXPECT_TRUE(payload.at("monitor").get<bool>());
  EXPECT_EQ(payload.at("image_url").get<std::string>(), "/api/vision.jpg");
  EXPECT_EQ(payload.at("stream_url").get<std::string>(), "/api/vision.mjpg");
}

TEST(VisionRoutesFrame, ServesJpegWithNoStore) {
  HttpServer server;
  const std::string jpeg = std::string("\xff\xd8\xff") + "payload" + "\xff\xd9";
  server.vision().SetFrame(jpeg);

  const Reply reply = server.Get("/api/vision.jpg");

  ASSERT_EQ(reply.status, 200);
  EXPECT_EQ(reply.Header("Content-Type"), "image/jpeg");
  // 1 枚ごとに別の絵なので保存させない。
  EXPECT_EQ(reply.Header("Cache-Control"), "no-store");
  EXPECT_EQ(reply.body, jpeg);
}

TEST(VisionRoutesFrame, NotFoundBeforeTheFirstFrame) {
  HttpServer server;

  const Reply reply = server.Get("/api/vision.jpg");

  EXPECT_EQ(reply.status, 404);
  EXPECT_TRUE(reply.body.empty());
}

TEST(VisionRoutesToggle, ReachesTheSourceAndIsReflectedInStatus) {
  HttpServer server;

  const Reply reply = server.Post("/api/vision", R"({"enabled": false})");

  ASSERT_EQ(reply.status, 200);
  EXPECT_EQ(json::parse(reply.body).at("monitor").get<bool>(), false);
  EXPECT_FALSE(server.vision().enabled());
  EXPECT_EQ(json::parse(server.Get("/api/vision").body).at("monitor").get<bool>(), false);
}

TEST(VisionRoutesToggle, RejectsNonBoolean) {
  HttpServer server;

  const Reply reply = server.Post("/api/vision", R"({"enabled": "true"})");

  EXPECT_EQ(reply.status, 400);
  EXPECT_TRUE(json::parse(reply.body).contains("error"));
  EXPECT_TRUE(server.vision().enabled());  // 触らない
}

TEST(VisionRoutesToggle, RejectsEmptyBody) {
  HttpServer server;

  const Reply reply = server.Post("/api/vision", "");

  EXPECT_EQ(reply.status, 400);
  EXPECT_TRUE(json::parse(reply.body).contains("error"));
}

TEST(VisionRoutesStream, IsNoContentWhileTheMonitorIsOff) {
  HttpServer server;
  server.vision().SetEnabled(false);

  const Reply reply = server.Get("/api/vision.mjpg");

  EXPECT_EQ(reply.status, 204);
}

TEST(VisionRoutesStream, PushesMultipartJpegFrames) {
  HttpServer server;
  server.vision().SetFrame(std::string("\xff\xd8\xff") + "frame" + "\xff\xd9");

  const std::string received = server.ReadStreamUntil("/api/vision.mjpg", "\xff\xd8\xff");

  EXPECT_NE(received.find("--meisterframe"), std::string::npos);
  EXPECT_NE(received.find("Content-Type: image/jpeg"), std::string::npos);
  EXPECT_NE(received.find("\xff\xd8\xff"), std::string::npos);
}

}  // namespace
}  // namespace meister_web_nav
