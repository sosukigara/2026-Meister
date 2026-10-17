// SPDX-License-Identifier: MIT
/// Web UI と JSON API の経路を httplib::Server に登録する。
///
/// 中心は地図の ETag 配信。Web UI は 5 秒ごとに /api/map.png を叩くので、地図が
/// 変わっていないときは本文を送らない (304) ことがこの面の主な仕事になる。
#ifndef MEISTER_WEB_NAV_HTTP_APP_HPP_
#define MEISTER_WEB_NAV_HTTP_APP_HPP_

#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>

#include <httplib.h>

#include "meister_web_nav/ports.hpp"

namespace meister_web_nav {

/// 1 リクエストの本文上限。これを超える本文は httplib が 413 で切る。
inline constexpr std::size_t kMaxBodyBytes = 1000000;

/// content hash を RFC 9110 8.8.3 の entity-tag に包む。
std::string EntityTag(const std::string& version);

/// If-None-Match が etag に一致するか（weak 比較）。
bool EtagMatches(const std::optional<std::string>& header, const std::string& etag);

/// webui_dir 配下の静的ファイルと /api/* を配信する。地図とナビゲータは port
/// 経由でしか触らないので、この関数は ROS 無しで test できる。
void RegisterRoutes(httplib::Server& server, const std::filesystem::path& webui_dir,
                    MapSource& map_source, NavSink& nav_sink);

}  // namespace meister_web_nav

#endif  // MEISTER_WEB_NAV_HTTP_APP_HPP_
