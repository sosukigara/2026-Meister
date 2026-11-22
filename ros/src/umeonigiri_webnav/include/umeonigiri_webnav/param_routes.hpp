// SPDX-License-Identifier: MIT
/// ROS パラメータの HTTP 経路。
///
/// 一覧・取得・設定だけを扱い、ROS 型は ParamSource の向こうに閉じ込める。
/// 登録は http_app の静的配信 (.*) より前に行うこと。httplib は登録順に
/// 最初に一致した経路を使うため、後ろに置くと /api/params が静的配信に食われる。
#ifndef UMEONIGIRI_WEBNAV_PARAM_ROUTES_HPP_
#define UMEONIGIRI_WEBNAV_PARAM_ROUTES_HPP_

#include <httplib.h>

#include "umeonigiri_webnav/ports.hpp"

namespace umeonigiri_webnav {

void RegisterParamRoutes(httplib::Server& server, ParamSource& param_source);

}  // namespace umeonigiri_webnav

#endif  // UMEONIGIRI_WEBNAV_PARAM_ROUTES_HPP_
