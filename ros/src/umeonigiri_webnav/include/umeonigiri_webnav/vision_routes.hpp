// SPDX-License-Identifier: MIT
/// 認識モニターの経路を httplib::Server に登録する。
///
/// MJPEG は chunked で押し出す。ブラウザ側は <img> の src に張るだけでよく、
/// JS で描き直すより軽い (1 接続で済み、HTTP ポーリングが要らない)。
#ifndef UMEONIGIRI_WEBNAV_VISION_ROUTES_HPP_
#define UMEONIGIRI_WEBNAV_VISION_ROUTES_HPP_

#include <httplib.h>

#include "umeonigiri_webnav/ports.hpp"

namespace umeonigiri_webnav {

void RegisterVisionRoutes(httplib::Server& server, VisionSource& vision_source);

}  // namespace umeonigiri_webnav

#endif  // UMEONIGIRI_WEBNAV_VISION_ROUTES_HPP_
