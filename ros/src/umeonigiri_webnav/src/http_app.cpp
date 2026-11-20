// SPDX-License-Identifier: MIT
#include "umeonigiri_webnav/http_app.hpp"

#include <cctype>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "umeonigiri_webnav/http_support.hpp"
#include "umeonigiri_webnav/map_store.hpp"
#include "umeonigiri_webnav/vision_routes.hpp"

namespace umeonigiri_webnav {
namespace {

using nlohmann::json;

/// Python 版の float(value) と同じ受容（数値と数値文字列）。型が違うだけで 400 に
/// すると、旧クライアントが送っていた "1.5" 形式を落とす。
double ToDouble(const json& value, const char* key) {
  if (value.is_number()) {
    return value.get<double>();
  }
  if (value.is_string()) {
    return std::stod(value.get<std::string>());
  }
  throw std::invalid_argument(std::string(key) + " must be a number");
}

/// 本文から地点列を取り出す。空文字なら成功、それ以外は 400 に載せる理由。
/// yaw を省略した地点は 0 を向く（Python 版 point.get('yaw', 0.0) と同じ）。
std::string ParseWaypoints(const std::string& body, std::vector<Waypoint>& waypoints) {
  if (body.empty()) {
    return "invalid content length";
  }
  try {
    const json parsed = json::parse(body);
    const json& points = parsed.at("waypoints");
    if (!points.is_array() || points.empty()) {
      return "waypoints must be a non-empty list";
    }
    for (const json& point : points) {
      const double yaw =
          point.contains("yaw") ? ToDouble(point.at("yaw"), "yaw") : 0.0;
      waypoints.push_back(
          Waypoint{ToDouble(point.at("x"), "x"), ToDouble(point.at("y"), "y"), yaw});
    }
  } catch (const std::exception& exc) {
    waypoints.clear();
    return exc.what();
  }
  return "";
}

}  // namespace

void ConfigureServer(httplib::Server& server) {
  server.set_payload_max_length(kMaxBodyBytes);
  // TCP_NODELAY と keep-alive 数を既定のままにすると、同じ接続の 2 回目以降の
  // 応答が 40 ms 遅れ、5 要求ごとに TCP を張り直す (実測)。
  server.set_tcp_nodelay(true);
  server.set_keep_alive_max_count(kKeepAliveMaxCount);
  server.set_keep_alive_timeout(kKeepAliveTimeoutSec);
}

void RegisterRoutes(httplib::Server& server, const std::filesystem::path& webui_dir,
                    MapSource& map_source, NavSink& nav_sink,
                    VisionSource& vision_source, ModeSink& mode_sink) {
  ConfigureServer(server);
  // 静的ファイルの表は 1 つだけ持つ。呼び手は一時オブジェクト (WebuiDir() や
  // std::filesystem::path(UMEONIGIRI_WEBUI_DIR)) を渡すので、参照で捕まえると
  // 登録が終わった時点で宙に浮く。
  const auto files = std::make_shared<StaticFiles>(webui_dir);

  // 毎回引かれる 4 本は経路として明示する (404 判定を通さずに済む)。
  server.Get("/", [files](const httplib::Request& req, httplib::Response& res) {
    files->Serve(res, "/index.html", JoinedIfNoneMatch(req));
  });
  server.Get("/index.html", [files](const httplib::Request& req, httplib::Response& res) {
    files->Serve(res, "/index.html", JoinedIfNoneMatch(req));
  });
  server.Get("/app.js", [files](const httplib::Request& req, httplib::Response& res) {
    files->Serve(res, "/app.js", JoinedIfNoneMatch(req));
  });
  server.Get("/style.css", [files](const httplib::Request& req, httplib::Response& res) {
    files->Serve(res, "/style.css", JoinedIfNoneMatch(req));
  });

  server.Get("/api/map", [&map_source](const httplib::Request&, httplib::Response& res) {
    std::optional<json> meta = map_source.Metadata();
    if (!meta) {
      SendJson(res, json{{"has_map", false}});
      return;
    }
    (*meta)["has_map"] = true;
    (*meta)["image_url"] = "/api/map.png";
    SendJson(res, *meta);
  });

  server.Get("/api/map.png",
             [&map_source](const httplib::Request& req, httplib::Response& res) {
               const std::optional<MapImage> image = map_source.RenderPng();
               if (!image) {
                 res.status = 404;
                 return;
               }
               // 引用符の付与は HTTP 面 (EntityTag) の責務。MapCache は entity-tag
               // ではなく生の content hash を返すだけ。
               const std::string etag = EntityTag(image->version);
               // no-cache は「保存してよいが必ず再検証せよ」の意味。no-store だと
               // ブラウザは If-None-Match を送らず、毎回 200 + 全バイトになる。
               // SLAM 中は地図が動くので stale は許さない。
               res.set_header("ETag", etag);
               res.set_header("Cache-Control", "no-cache");
               if (EtagMatches(JoinedIfNoneMatch(req), etag)) {
                 // 304 では bytes を 1 バイトもコピーしない (共有したまま捨てる)。
                 res.status = 304;
                 return;
               }
               res.set_content(*image->bytes, "image/png");
             });

  server.Get("/api/pose",
             [&map_source](const httplib::Request&, httplib::Response& res) {
               std::optional<json> pose = map_source.RobotPose();
               json payload{{"has_pose", pose.has_value()}};
               if (pose) {
                 payload.update(*pose);
               }
               SendJson(res, payload);
             });

  server.Get("/api/status", [&nav_sink](const httplib::Request&, httplib::Response& res) {
    SendJson(res, nav_sink.Status());
  });

  // ---- モードと保存地図 ----
  server.Get("/api/mode", [&mode_sink](const httplib::Request&, httplib::Response& res) {
    SendJson(res, json{{"mode", mode_sink.Mode()}});
  });

  server.Post("/api/mode", [&mode_sink](const httplib::Request& req, httplib::Response& res) {
    const json body = json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("mode") || !body["mode"].is_string()) {
      SendJson(res, json{{"error", "mode is required"}}, 400);
      return;
    }
    const std::string mode = body["mode"].get<std::string>();
    if (mode == "mapping") {
      // 地図生成へ戻るのは launch 再起動のみ（設計書 §6.4）。
      SendJson(res, json{{"error",
                          "地図生成へ戻るには launch を再起動してください"}}, 409);
      return;
    }
    if (mode != "localization") {
      SendJson(res, json{{"error", "unknown mode"}}, 400);
      return;
    }
    const std::string error = mode_sink.SwitchToLocalization();
    if (!error.empty()) {
      // 切り替わっていないのに成功を返すと、以後の操作が全部嘘になる。
      SendJson(res, json{{"error", error}}, 409);
      return;
    }
    SendJson(res, json{{"mode", mode_sink.Mode()}});
  });

  server.Get("/api/maps", [&mode_sink](const httplib::Request&, httplib::Response& res) {
    SendJson(res, json{{"maps", mode_sink.ListMaps()}});
  });

  server.Post("/api/map/save", [&mode_sink](const httplib::Request& req, httplib::Response& res) {
    const json body = req.body.empty() ? json::object()
                                       : json::parse(req.body, nullptr, false);
    if (body.is_discarded()) {
      SendJson(res, json{{"error", "invalid json"}}, 400);
      return;
    }
    std::string name;
    if (body.contains("name")) {
      if (!body["name"].is_string()) {
        SendJson(res, json{{"error", "name must be a string"}}, 400);
        return;
      }
      name = body["name"].get<std::string>();
    }
    if (!name.empty() && !IsValidMapName(name)) {
      SendJson(res, json{{"error",
                          "name must be 1-64 chars of [A-Za-z0-9_-]"}}, 400);
      return;
    }
    const std::string error = mode_sink.SaveMap(name);
    if (!error.empty()) {
      SendJson(res, json{{"error", error}}, 409);
      return;
    }
    SendJson(res, json{{"status", "saved"}});
  });

  server.Delete(R"(/api/maps/([^/]+))",
                [&mode_sink](const httplib::Request& req, httplib::Response& res) {
                  const std::string name = req.matches[1];
                  if (!IsValidMapName(name)) {
                    SendJson(res, json{{"error", "invalid map name"}}, 400);
                    return;
                  }
                  const std::string error = mode_sink.DeleteMap(name);
                  if (!error.empty()) {
                    SendJson(res, json{{"error", error}}, 409);
                    return;
                  }
                  SendJson(res, json{{"status", "deleted"}});
                });

  server.Post("/api/initialpose",
              [&mode_sink](const httplib::Request& req, httplib::Response& res) {
                const json body = json::parse(req.body, nullptr, false);
                if (body.is_discarded() || !body.contains("x") || !body.contains("y")) {
                  SendJson(res, json{{"error", "x and y are required"}}, 400);
                  return;
                }
                // 数値でない x / y を json::get<double> で読むと例外が
                // ハンドラを抜けて 500 になる (実測)。地点列と同じ受容に揃える。
                double x = 0.0;
                double y = 0.0;
                double yaw = 0.0;
                try {
                  x = ToDouble(body.at("x"), "x");
                  y = ToDouble(body.at("y"), "y");
                  yaw = body.contains("yaw") ? ToDouble(body.at("yaw"), "yaw") : 0.0;
                } catch (const std::exception& exc) {
                  SendJson(res, json{{"error", exc.what()}}, 400);
                  return;
                }
                mode_sink.SetInitialPose(x, y, yaw);
                SendJson(res, json{{"status", "ok"}});
              });



  RegisterVisionRoutes(server, vision_source);

  // 明示した 3 ファイル以外を webui_dir から配る。後から足した asset が 404 に
  // ならないようにする。webui_dir の外へ出る経路は canonical 比較で弾く。
  // この経路は /.* に一致するので、必ず最後に登録する (httplib は登録順に見る)。
  server.Get(R"(/.*)", [files](const httplib::Request& req, httplib::Response& res) {
    files->Serve(res, req.path, JoinedIfNoneMatch(req));
  });

  server.Post("/api/nav",
              [&nav_sink](const httplib::Request& req, httplib::Response& res) {
                std::vector<Waypoint> waypoints;
                const std::string error = ParseWaypoints(req.body, waypoints);
                if (!error.empty()) {
                  SendJson(res, json{{"error", error}}, 400);
                  return;
                }
                nav_sink.Submit(waypoints);
                SendJson(res, json{{"status", "submitted"},
                                   {"count", static_cast<int>(waypoints.size())}});
              });

  server.Post("/api/cancel", [&nav_sink](const httplib::Request&, httplib::Response& res) {
    nav_sink.Cancel();
    SendJson(res, json{{"status", "cancel_requested"}});
  });
}

}  // namespace umeonigiri_webnav
