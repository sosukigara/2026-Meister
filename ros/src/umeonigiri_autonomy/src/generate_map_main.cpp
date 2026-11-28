// SPDX-License-Identifier: MIT
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>

#include "umeonigiri_autonomy/map_generator.hpp"

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

  const umeonigiri_autonomy::MapSpec spec;
  const auto grid = umeonigiri_autonomy::BuildNavWorldGrid(spec);

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
  if (!umeonigiri_autonomy::WritePgm(pgm, spec, grid)) {
    std::fprintf(stderr, "ERROR: cannot write %s\n", pgm.c_str());
    return 1;
  }
  if (!umeonigiri_autonomy::WriteMapYaml(yaml, spec, "nav_world.pgm")) {
    std::fprintf(stderr, "ERROR: cannot write %s\n", yaml.c_str());
    return 1;
  }

  std::printf("map written:\n  %s\n  %s\n  %dx%d px, %.2fx%.2f m\n", pgm.c_str(),
              yaml.c_str(), spec.width(), spec.height(), spec.width_m, spec.height_m);
  return 0;
}
