// SPDX-License-Identifier: MIT
#ifndef UMEONIGIRI_SERIAL_BRIDGE__CLI_NUMBER_HPP_
#define UMEONIGIRI_SERIAL_BRIDGE__CLI_NUMBER_HPP_

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>

namespace umeonigiri_serial_bridge {

/// コマンドライン引数の数値を読む。読めなければ usage を出して終了する。
///
/// std::stoi / std::stod は例外を投げる。main の外へ抜けると std::terminate に
/// なり、"terminate called after throwing ... what(): stod" だけを残して core を
/// 吐く（実測）。利用者には何が悪いのか分からないので、ここで受けて終了する。
///
/// 有限値だけを受けるのは、NaN / 無限大がそのまま周期計算に入ると
/// 測定が無意味な値になるため。
inline void ExitOnBadNumber(const char* name, const char* text) {
  std::fprintf(stderr, "ERROR: %s の値が数値ではありません: %s\n", name, text);
  std::exit(2);
}

inline int ParseIntArgument(const char* text, const char* name) {
  std::size_t consumed = 0;
  int value = 0;
  try {
    value = std::stoi(text, &consumed);
  } catch (const std::exception&) {
    ExitOnBadNumber(name, text);
  }
  if (consumed != std::string(text).size()) {
    ExitOnBadNumber(name, text);
  }
  return value;
}

inline double ParseDoubleArgument(const char* text, const char* name) {
  std::size_t consumed = 0;
  double value = 0.0;
  try {
    value = std::stod(text, &consumed);
  } catch (const std::exception&) {
    ExitOnBadNumber(name, text);
  }
  if (consumed != std::string(text).size() || !std::isfinite(value)) {
    ExitOnBadNumber(name, text);
  }
  return value;
}

}  // namespace umeonigiri_serial_bridge

#endif  // UMEONIGIRI_SERIAL_BRIDGE__CLI_NUMBER_HPP_
