// SPDX-License-Identifier: MIT
/// 保存地図のファイル名と一覧。UI から来た文字列をそのままパスに使わない。
#pragma once

#include <string>
#include <vector>

namespace meister_web_nav {

/// ファイル名として安全な地図名か。英数・`_`・`-` のみ、1〜64 文字。
/// `..` や `/` `\` ` ` `.` を含むものは拒否する（パストラバーサル対策）。
bool IsValidMapName(const std::string& name);

/// 空欄時の既定名 `<world>_<timestamp>`。world が使えなければ `map_<timestamp>`。
std::string DefaultMapName(const std::string& world, const std::string& timestamp);

/// `dir` 直下の `*.yaml` から拡張子を除いた名前を昇順で返す。無ければ空。
std::vector<std::string> ListMapNames(const std::string& dir);

}  // namespace meister_web_nav
