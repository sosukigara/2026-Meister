// SPDX-License-Identifier: MIT
#include "meister_web_nav/http_support.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <map>
#include <sstream>
#include <system_error>

namespace meister_web_nav {

void SendJson(httplib::Response& res, const nlohmann::json& payload, int status) {
  res.status = status;
  res.set_content(payload.dump(), "application/json");
}

void SendFile(httplib::Response& res, const std::filesystem::path& path,
              const std::string& content_type) {
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    res.status = 404;
    return;
  }
  std::ostringstream buffer;
  buffer << file.rdbuf();
  res.set_content(buffer.str(), content_type);
}

std::string MimeTypeOf(const std::filesystem::path& path) {
  static const std::map<std::string, std::string> kTypes = {
      {".css", "text/css; charset=utf-8"},
      {".html", "text/html; charset=utf-8"},
      {".ico", "image/x-icon"},
      {".js", "text/javascript; charset=utf-8"},
      {".json", "application/json"},
      {".png", "image/png"},
      {".svg", "image/svg+xml"},
      {".txt", "text/plain; charset=utf-8"},
  };
  std::string extension = path.extension().string();
  std::transform(extension.begin(), extension.end(), extension.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  const auto found = kTypes.find(extension);
  return found == kTypes.end() ? "application/octet-stream" : found->second;
}

// target が root の中にあるか。root 自身は「ファイルではない」ので false。
bool IsInside(const std::filesystem::path& root, const std::filesystem::path& target) {
  if (target == root) {
    return false;
  }
  const std::string root_str = root.string();
  const std::string target_str = target.string();
  return target_str.size() > root_str.size() &&
         target_str.compare(0, root_str.size(), root_str) == 0 &&
         target_str[root_str.size()] == '/';
}

void ServeStatic(httplib::Response& res, const std::filesystem::path& webui_dir,
                 const std::string& request_path) {
  std::error_code error;
  const std::filesystem::path root = std::filesystem::weakly_canonical(webui_dir, error);
  if (error) {
    res.status = 404;
    return;
  }
  // 脱出判定は要求文字列の lexical 正規化だけで行う。weakly_canonical で
  // 解決してから比較すると、--symlink-install の webui (share 配下の各ファイルが
  // ソースツリーを指すシンボリックリンク) が全部 root の外と判定され、
  // /js/*.js が 404 になる。webui にリンクを置けるのは install だけなので、
  // 要求パスの ".." を潰せば外へは出られない。
  const std::filesystem::path relative =
      std::filesystem::path(request_path.substr(1)).lexically_normal();
  if (relative.empty() || relative.is_absolute()) {
    res.status = 404;
    return;
  }
  const std::filesystem::path target = (root / relative).lexically_normal();
  if (!IsInside(root, target) ||
      !std::filesystem::is_regular_file(target, error)) {
    res.status = 404;
    return;
  }
  SendFile(res, target, MimeTypeOf(target));
}

}  // namespace meister_web_nav
