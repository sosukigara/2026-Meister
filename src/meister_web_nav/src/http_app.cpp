// SPDX-License-Identifier: MIT
#include "meister_web_nav/http_app.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <vector>

#include <nlohmann/json.hpp>

namespace meister_web_nav {
namespace {

using nlohmann::json;

std::string Trim(std::string value) {
  const auto visible = [](unsigned char c) { return std::isspace(c) == 0; };
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), visible));
  value.erase(std::find_if(value.rbegin(), value.rend(), visible).base(), value.end());
  return value;
}

/// If-None-Match の 1 要素から、比較に使う中身だけ取り出す。
///
/// nullopt は「読めない要素」で比較対象外。weakness marker は ABNF の大文字小文字
/// 非区別 (RFC 5234 2.3) に従い w/ も受ける。引用符が無い値は旧形式としてそのまま
/// 返す（受容の理由は EtagMatches）。
std::optional<std::string> OpaqueTag(std::string candidate) {
  candidate = Trim(std::move(candidate));
  if (candidate.size() >= 2 &&
      (candidate.compare(0, 2, "W/") == 0 || candidate.compare(0, 2, "w/") == 0)) {
    candidate = Trim(candidate.substr(2));
  }
  if (candidate.size() >= 2 && candidate.front() == '"' && candidate.back() == '"') {
    return candidate.substr(1, candidate.size() - 2);
  }
  if (candidate.empty()) {
    return std::nullopt;
  }
  return candidate;
}

/// If-None-Match を全ヘッダ行まとめて 1 本にする。
///
/// RFC 9110 5.2 のフィールド合成（同名フィールドの連結）をそのままやる。
/// get_header_value だけだと 2 行目以降を落とし、正規のクライアントが 2 行送った
/// ときに対象のタグを見失う。
std::optional<std::string> JoinIfNoneMatch(const httplib::Request& req) {
  const std::size_t count = req.get_header_value_count("If-None-Match");
  if (count == 0) {
    return std::nullopt;
  }
  std::string joined = req.get_header_value("If-None-Match", 0);
  for (std::size_t i = 1; i < count; ++i) {
    joined += ',';
    joined += req.get_header_value("If-None-Match", i);
  }
  return joined;
}

void SendJson(httplib::Response& res, const json& payload, int status = 200) {
  res.status = status;
  res.set_content(payload.dump(), "application/json");
}

void SendFile(httplib::Response& res, const std::filesystem::path& path,
              const std::string& content_type) {
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    res.status = 404;
    return;
  }
  std::ostringstream buffer;
  buffer << file.rdbuf();
  res.set_content(buffer.str(), content_type);
}

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

/// target が root の中にあるか。root 自身は「ファイルではない」ので false。
bool IsInside(const std::filesystem::path& root, const std::filesystem::path& target) {
  if (target == root) {
    return false;
  }
  const std::string root_str = root.string();
  const std::string target_str = target.string();
  return target_str.size() > root_str.size() &&
         target_str.compare(0, root_str.size(), root_str) == 0 &&
         target_str[root_str.size()] == '/';
}

std::string MimeTypeOf(const std::filesystem::path& path) {
  static const std::map<std::string, std::string> kTypes = {
      {".css", "text/css; charset=utf-8"},
      {".html", "text/html; charset=utf-8"},
      {".ico", "image/x-icon"},
      {".js", "text/javascript; charset=utf-8"},
      {".json", "application/json"},
      {".png", "image/png"},
      {".svg", "image/svg+xml"},
      {".txt", "text/plain; charset=utf-8"},
  };
  std::string extension = path.extension().string();
  std::transform(extension.begin(), extension.end(), extension.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  const auto found = kTypes.find(extension);
  return found == kTypes.end() ? "application/octet-stream" : found->second;
}

}  // namespace

std::string EntityTag(const std::string& version) { return '"' + version + '"'; }

bool EtagMatches(const std::optional<std::string>& header, const std::string& etag) {
  /// RFC 9110 13.1.2 は If-None-Match に weak 比較（opaque-tag だけ見比べる）を
  /// 要求するので、weak なタグは strong な同じ値に一致する。entity-tag 全体の文字列で
  /// 比べると weakness marker が差として残り、weak なクライアントを取りこぼす。
  ///
  /// 引用符無しの値も比較対象に残す: 旧サーバーは引用符なしで ETag を送っていたので、
  /// サーバーだけを更新してブラウザの HTTP キャッシュは古いままという隙間では旧形式が
  /// そのまま If-None-Match で戻ってくる。無視すると更新直後の 1 リクエストだけ地図を
  /// 全バイト再送し、304 で削りたかったコストを同じ手で戻す。値は地図内容そのものの
  /// hash なので、形式が不正でも一致判定は正しい。
  if (!header || header->empty()) {
    return false;
  }
  const std::optional<std::string> target = OpaqueTag(etag);
  if (!target) {
    return false;
  }
  std::size_t start = 0;
  while (true) {
    const std::size_t comma = header->find(',', start);
    const std::string element = header->substr(
        start, comma == std::string::npos ? std::string::npos : comma - start);
    const std::optional<std::string> tag = OpaqueTag(element);
    if (tag && (*tag == "*" || *tag == *target)) {
      return true;
    }
    if (comma == std::string::npos) {
      return false;
    }
    start = comma + 1;
  }
}

void RegisterRoutes(httplib::Server& server, const std::filesystem::path& webui_dir,
                    MapSource& map_source, NavSink& nav_sink) {
  // webui_dir を捕まえる lambda は値でコピーする。呼び手は一時オブジェクト
  // (WebuiDir() や std::filesystem::path(MEISTER_WEBUI_DIR)) を渡すので、参照で
  // 捕まえると登録が終わった時点で宙に浮く。
  server.set_payload_max_length(kMaxBodyBytes);

  server.Get("/", [webui_dir](const httplib::Request&, httplib::Response& res) {
    SendFile(res, webui_dir / "index.html", "text/html; charset=utf-8");
  });
  server.Get("/index.html",
             [webui_dir](const httplib::Request&, httplib::Response& res) {
               SendFile(res, webui_dir / "index.html", "text/html; charset=utf-8");
             });
  server.Get("/app.js", [webui_dir](const httplib::Request&, httplib::Response& res) {
    SendFile(res, webui_dir / "app.js", "text/javascript; charset=utf-8");
  });
  server.Get("/style.css",
             [webui_dir](const httplib::Request&, httplib::Response& res) {
               SendFile(res, webui_dir / "style.css", "text/css; charset=utf-8");
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
               const auto rendered = map_source.RenderPng();
               if (!rendered) {
                 res.status = 404;
                 return;
               }
               // 引用符の付与は HTTP 面 (EntityTag) の責務。MapCache は entity-tag
               // ではなく生の content hash を返すだけ。
               const std::string etag = EntityTag(rendered->second);
               // no-cache は「保存してよいが必ず再検証せよ」の意味。no-store だと
               // ブラウザは If-None-Match を送らず、毎回 200 + 全バイトになる。
               // SLAM 中は地図が動くので stale は許さない。
               res.set_header("ETag", etag);
               res.set_header("Cache-Control", "no-cache");
               if (EtagMatches(JoinIfNoneMatch(req), etag)) {
                 res.status = 304;
                 return;
               }
               res.set_content(rendered->first, "image/png");
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

  // 明示した 3 ファイル以外を webui_dir から配る。後から足した asset が 404 に
  // ならないようにする。webui_dir の外へ出る経路は canonical 比較で弾く。
  server.Get(R"(/.*)", [webui_dir](const httplib::Request& req, httplib::Response& res) {
    std::error_code error;
    const std::filesystem::path root = std::filesystem::canonical(webui_dir, error);
    if (error) {
      res.status = 404;
      return;
    }
    const std::filesystem::path resolved =
        std::filesystem::weakly_canonical(root / req.path.substr(1), error);
    if (error || !IsInside(root, resolved) ||
        !std::filesystem::is_regular_file(resolved, error)) {
      res.status = 404;
      return;
    }
    SendFile(res, resolved, MimeTypeOf(resolved));
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

}  // namespace meister_web_nav
