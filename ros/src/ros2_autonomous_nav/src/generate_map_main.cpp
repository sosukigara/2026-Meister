// SPDX-License-Identifier: MIT
/// nav_world.sdf から nav_world.pgm / nav_world.yaml を書き出す。
///
/// 使い方:
///   ros2 run ros2_autonomous_nav generate_map --out-dir maps
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>

#include "ros2_autonomous_nav/map_generator.hpp"

int main(int argc, char** argv) {
  std::string out_dir = "maps";
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--out-dir") == 0 && i + 1 < argc) {
      out_dir = argv[++i];
    } else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
      std::printf("usage: generate_map [--out-dir DIR]\n");
      return 0;
    } else {
      std::fprintf(stderr, "ERROR: unknown argument %s\n", argv[i]);
      return 1;
    }
  }

  const ros2_autonomous_nav::MapSpec spec;
  const auto grid = ros2_autonomous_nav::BuildNavWorldGrid(spec);

  std::error_code ec;
  std::filesystem::create_directories(out_dir, ec);
  if (ec) {
    std::fprintf(stderr, "ERROR: cannot create %s: %s\n", out_dir.c_str(), ec.message().c_str());
    return 1;
  }

  const std::string pgm = out_dir + "/nav_world.pgm";
  const std::string yaml = out_dir + "/nav_world.yaml";
  // 書けなかったのに "map written" と出して 0 で終わると、地図が無いまま
  // 成功したように見える。1 つでも失敗したら非 0 で止める。
  if (!ros2_autonomous_nav::WritePgm(pgm, spec, grid)) {
    std::fprintf(stderr, "ERROR: cannot write %s\n", pgm.c_str());
    return 1;
  }
  if (!ros2_autonomous_nav::WriteMapYaml(yaml, spec, "nav_world.pgm")) {
    std::fprintf(stderr, "ERROR: cannot write %s\n", yaml.c_str());
    return 1;
  }

  std::printf("map written:\n  %s\n  %s\n  %dx%d px, %.2fx%.2f m\n", pgm.c_str(),
              yaml.c_str(), spec.width(), spec.height(), spec.width_m, spec.height_m);
  return 0;
}
