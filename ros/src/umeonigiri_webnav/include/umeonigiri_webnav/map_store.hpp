// SPDX-License-Identifier: MIT
/// 保存地図のファイル名と一覧。UI から来た文字列をそのままパスに使わない。
#pragma once

#include <string>
#include <vector>

namespace umeonigiri_webnav {

/// ファイル名として安全な地図名か。英数・`_`・`-` のみ、1〜64 文字。
/// `..` や `/` `\` ` ` `.` を含むものは拒否する（パストラバーサル対策）。
bool IsValidMapName(const std::string& name);

/// 空欄時の既定名 `<world>_<timestamp>`。world が使えなければ `map_<timestamp>`。
std::string DefaultMapName(const std::string& world, const std::string& timestamp);

/// 要求名が空なら既定名、そうでなければそのまま。名前の決定をここ 1 箇所に置く。
/// 空名をそのまま map_saver へ渡すと保存先がディレクトリになり、一覧にも出ない
/// 地図ができる（実測で `/api/map/save {}` がそうなっていた）。
std::string MapNameOrDefault(const std::string& requested, const std::string& world,
                             const std::string& timestamp);

/// `dir` 直下の `*.yaml` から拡張子を除いた名前を昇順で返す。無ければ空。
std::vector<std::string> ListMapNames(const std::string& dir);

/// `dir` 直下の地図一式を消す。空文字列なら成功、失敗なら理由。
/// map_saver は image_format 次第で `.png` も `.pgm` も書くので両方見る。
std::string DeleteMapFiles(const std::string& dir, const std::string& name);

}  // namespace umeonigiri_webnav
