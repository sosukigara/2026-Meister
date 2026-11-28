// SPDX-License-Identifier: MIT
#ifndef UMEONIGIRI_WEBNAV_HTTP_APP_HPP_
#define UMEONIGIRI_WEBNAV_HTTP_APP_HPP_

#include <cstddef>
#include <ctime>
#include <filesystem>

#include <httplib.h>

#include "umeonigiri_webnav/ports.hpp"

namespace umeonigiri_webnav {

/// 1 リクエストの本文上限。これを超える本文は httplib が 413 で切る。
inline constexpr std::size_t kMaxBodyBytes = 1000000;

/// keep-alive の上限。httplib の既定は 5 で、ポーリングする UI では 5 要求ごとに
/// TCP を張り直すことになる。
inline constexpr std::size_t kKeepAliveMaxCount = 1000;
inline constexpr time_t kKeepAliveTimeoutSec = 10;

/// 同時に処理する接続の数。httplib の既定は max(8, コア数) で、
/// keep-alive 中の接続も MJPEG のストリームも 1 本につき 1 スレッドを
/// 占有し続ける。既定のままだと画面 1 枚 (6 接続前後) + 映像で埋まり、
/// 実測で MJPEG 8 本のときに /api/pose が 5 秒タイムアウトした。
/// クライアント数 × (ポーリング接続 + 映像 1 本) に余裕を持たせる。
inline constexpr std::size_t kThreadPoolSize = 32;

/// httplib のソケット設定。経路の登録より先に呼ぶ。
///
/// TCP_NODELAY を入れないと、応答ヘッダと本文を別々に送ったときに Nagle と
/// delayed ACK が噛み合い、同一接続の 2 回目以降が 40 ms 待たされる (実測:
/// p50 42 ms -> 0.4 ms)。地図・姿勢・状態を 1 秒おきに引く画面では、これが
/// そのまま操作の重さになる。
///
/// listen の受付待ち行列 (CPPHTTPLIB_LISTEN_BACKLOG) はヘッダのマクロではなく
/// 配布物の共有ライブラリに焼き込まれているため、ここからは変えられない。
void ConfigureServer(httplib::Server& server);

/// webui_dir 配下の静的ファイルと /api/* を配信する。地図とナビゲータは port
/// 経由でしか触らないので、この関数は ROS 無しで test できる。
void RegisterRoutes(httplib::Server& server, const std::filesystem::path& webui_dir,
                    MapSource& map_source, NavSink& nav_sink,
                    VisionSource& vision_source, ModeSink& mode_sink,
                    ParamSource& param_source);

}  // namespace umeonigiri_webnav

#endif  // UMEONIGIRI_WEBNAV_HTTP_APP_HPP_
