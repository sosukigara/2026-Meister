// SPDX-License-Identifier: MIT
/// bench_nav のコマンドライン引数。
#pragma once

#include <string>

namespace umeonigiri_autonomy {

struct BenchOptions {
  std::string goals;
  std::string out;
  std::string map_id;
  int repeats{-1};
  int limit{-1};
  int auto_goals{-1};
  /// 1 ゴールの制限時間。受理応答が完了時に返る構成なので、これは実質
  /// 「ゴールの所要時間上限」として働く。遅い制御器でも打ち切らない長さにする。
  double goal_timeout{120.0};
  double sample_hz{10.0};
  double min_spacing{1.0};
  double min_clearance{0.5};
  /// 計測時の GUI 有無。"unknown" のまま推測で埋めない。
  std::string gui{"unknown"};
  /// 0 より大きければ自己診断モード（TF と /clock の疎通確認）で秒数だけ回す。
  double self_check{0.0};
};

enum class ParseStatus { kOk, kHelp, kError };

/// 失敗時は stderr に理由を出し、戻り値は kHelp / kError。
ParseStatus ParseBenchArgs(int argc, char** argv, BenchOptions* options);

}  // namespace umeonigiri_autonomy
