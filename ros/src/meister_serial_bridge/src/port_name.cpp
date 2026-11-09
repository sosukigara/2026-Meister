#include "meister_serial_bridge/serial_io.hpp"

#include <filesystem>

namespace meister_serial_bridge {

std::string DescribePort(const std::string& path) {
  std::error_code ec;
  const auto real = std::filesystem::weakly_canonical(path, ec);
  if (ec) {
    return path;
  }
  std::error_code iter_ec;
  for (const auto& entry :
       std::filesystem::directory_iterator("/dev/serial/by-id", iter_ec)) {
    if (iter_ec) {
      break;
    }
    if (std::filesystem::weakly_canonical(entry.path(), iter_ec) == real && !iter_ec) {
      return entry.path().string();
    }
  }
  return path;
}

}  // namespace meister_serial_bridge
