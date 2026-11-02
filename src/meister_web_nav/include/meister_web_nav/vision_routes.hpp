// SPDX-License-Identifier: MIT
/// 認識モニターの経路を httplib::Server に登録する。
///
/// MJPEG は chunked で押し出す。ブラウザ側は <img> の src に張るだけでよく、
/// JS で描き直すより軽い (1 接続で済み、HTTP ポーリングが要らない)。
#ifndef MEISTER_WEB_NAV_VISION_ROUTES_HPP_
#define MEISTER_WEB_NAV_VISION_ROUTES_HPP_

#include <httplib.h>

#include "meister_web_nav/ports.hpp"

namespace meister_web_nav {

void RegisterVisionRoutes(httplib::Server& server, VisionSource& vision_source);

}  // namespace meister_web_nav

#endif  // MEISTER_WEB_NAV_VISION_ROUTES_HPP_
