// SPDX-License-Identifier: MIT
/// 認識モニターの経路。/api/vision (状態 JSON)、/api/vision.jpg (1 枚)、
/// /api/vision.mjpg (MJPEG ストリーム)、POST /api/vision (監視 ON/OFF)。
#include "meister_web_nav/vision_routes.hpp"

#include <cstddef>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

#include "meister_web_nav/http_support.hpp"

namespace meister_web_nav {
namespace {

using nlohmann::json;

constexpr char kBoundary[] = "meisterframe";

/// ストリームが新しいフレームを待つ上限。0 にすると待ち無しで回り、httplib の
/// 応答ループが空転する。切断を検知する機会も兼ねる。
constexpr double kFrameWaitSeconds = 1.0;

/// 次のフレームを待つ間に本文へ書く 1 パート。
std::string MjpegPart(const VisionFrame& frame) {
  std::string part = "--";
  part += kBoundary;
  part += "\r\nContent-Type: image/jpeg\r\nContent-Length: ";
  part += std::to_string(frame.jpeg.size());
  part += "\r\nX-Seq: ";
  part += std::to_string(frame.seq);
  part += "\r\n\r\n";
  part += frame.jpeg;
  part += "\r\n";
  return part;
}

/// 監視 ON/OFF の本文。真偽値以外は 400 にする ("true" 文字列も受けない: UI は
/// 常に真偽値を送るので、受容を広げると誤りの在処が遠くなる)。
bool ParseEnabled(const std::string& body, bool& enabled, std::string& error) {
  if (body.empty()) {
    error = "invalid content length";
    return false;
  }
  try {
    const json parsed = json::parse(body);
    const json& value = parsed.at("enabled");
    if (!value.is_boolean()) {
      error = "enabled must be a boolean";
      return false;
    }
    enabled = value.get<bool>();
  } catch (const std::exception& exc) {
    error = exc.what();
    return false;
  }
  return true;
}

}  // namespace

void RegisterVisionRoutes(httplib::Server& server, VisionSource& vision_source) {
  server.Get("/api/vision", [&vision_source](const httplib::Request&,
                                             httplib::Response& res) {
    json payload = vision_source.Status();
    // URL は配信側ではなく HTTP 面が決める (地図の image_url と同じ扱い)。
    payload["image_url"] = "/api/vision.jpg";
    payload["stream_url"] = "/api/vision.mjpg";
    SendJson(res, payload);
  });

  server.Get("/api/vision.jpg", [&vision_source](const httplib::Request&,
                                                 httplib::Response& res) {
    const std::optional<VisionFrame> frame = vision_source.Latest(-1, 0.0);
    if (!frame) {
      res.status = 404;
      return;
    }
    // 1 枚ごとに別の絵なので ETag も no-cache も意味を持たない。ブラウザの
    // キャッシュに残すと次の更新が見えなくなるので no-store。
    res.set_header("Cache-Control", "no-store");
    res.set_content(frame->jpeg, "image/jpeg");
  });

  server.Get("/api/vision.mjpg", [&vision_source](const httplib::Request&,
                                                  httplib::Response& res) {
    const json status = vision_source.Status();
    if (!status.value("monitor", false)) {
      // 監視 OFF は推論も止まっている。繋いでも何も流れないので 204 で返す。
      res.status = 204;
      return;
    }
    res.set_header("Cache-Control", "no-store");
    auto last_seq = std::make_shared<long>(-1);
    res.set_chunked_content_provider(
        "multipart/x-mixed-replace; boundary=" + std::string(kBoundary),
        [&vision_source, last_seq](std::size_t, httplib::DataSink& sink) {
          const std::optional<VisionFrame> frame =
              vision_source.Latest(*last_seq, kFrameWaitSeconds);
          if (!frame) {
            // 新フレーム待ち。何も書かずに true を返し、httplib に再び呼ばせる。
            // ここで false を返すとブラウザ側の <img> が読み込みを終えてしまう。
            return true;
          }
          *last_seq = frame->seq;
          const std::string part = MjpegPart(*frame);
          return sink.write(part.data(), part.size());
        });
  });

  server.Post("/api/vision", [&vision_source](const httplib::Request& req,
                                              httplib::Response& res) {
    bool enabled = false;
    std::string error;
    if (!ParseEnabled(req.body, enabled, error)) {
      SendJson(res, json{{"error", error}}, 400);
      return;
    }
    vision_source.SetEnabled(enabled);
    SendJson(res, json{{"monitor", enabled}});
  });
}

}  // namespace meister_web_nav
