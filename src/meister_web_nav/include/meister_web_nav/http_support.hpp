// SPDX-License-Identifier: MIT
/// HTTP 応答の共通部品。静的ファイル配信と JSON 送信をここ 1 箇所に置く。
///
/// http_app.cpp と vision_routes.cpp の両方が使う。片方に置いて他方から include
/// すると、経路を足すたびに依存の向きが入れ替わる。
#ifndef MEISTER_WEB_NAV_HTTP_SUPPORT_HPP_
#define MEISTER_WEB_NAV_HTTP_SUPPORT_HPP_

#include <filesystem>
#include <string>

#include <httplib.h>
#include <nlohmann/json.hpp>

namespace meister_web_nav {

void SendJson(httplib::Response& res, const nlohmann::json& payload, int status = 200);

void SendFile(httplib::Response& res, const std::filesystem::path& path,
              const std::string& content_type);

std::string MimeTypeOf(const std::filesystem::path& path);

/// target が root の中にあるか。root 自身は「ファイルではない」ので false。
bool IsInside(const std::filesystem::path& root, const std::filesystem::path& target);

/// webui_dir 配下の静的ファイルを配る。要求パスの ".." は lexical に潰して弾く
/// (シンボリックリンクは辿る: --symlink-install の share 配下はソースを指す)。
void ServeStatic(httplib::Response& res, const std::filesystem::path& webui_dir,
                 const std::string& request_path);

}  // namespace meister_web_nav

#endif  // MEISTER_WEB_NAV_HTTP_SUPPORT_HPP_
