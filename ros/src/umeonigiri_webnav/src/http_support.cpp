// SPDX-License-Identifier: MIT
#include "umeonigiri_webnav/http_support.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iterator>
#include <map>
#include <system_error>
#include <utility>

namespace umeonigiri_webnav {
namespace {

/// キャッシュする静的ファイルの上限。webui は 15 本程度なので、これは
/// 「壊れたクライアントが無限に別パスを要求してもメモリが増えない」ための蓋。
constexpr std::size_t kMaxCachedFiles = 64;

std::string Trim(std::string value) {
  const auto visible = [](unsigned char c) { return std::isspace(c) == 0; };
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), visible));
  value.erase(std::find_if(value.rbegin(), value.rend(), visible).base(), value.end());
  return value;
}

/// If-None-Match の 1 要素から、比較に使う中身だけ取り出す。
///
/// nullopt は「読めない要素」で比較対象外。weakness marker は ABNF の大文字小文字
/// 非区別 (RFC 5234 2.3) に従い w/ も受ける。引用符が無い値は旧形式としてそのまま
/// 返す（受容の理由は EtagMatches）。
std::optional<std::string> OpaqueTag(std::string candidate) {
  candidate = Trim(std::move(candidate));
  if (candidate.size() >= 2 &&
      (candidate.compare(0, 2, "W/") == 0 || candidate.compare(0, 2, "w/") == 0)) {
    candidate = Trim(candidate.substr(2));
  }
  if (candidate.size() >= 2 && candidate.front() == '"' && candidate.back() == '"') {
    return candidate.substr(1, candidate.size() - 2);
  }
  if (candidate.empty()) {
    return std::nullopt;
  }
  return candidate;
}

}  // namespace

void SendJson(httplib::Response& res, const nlohmann::json& payload, int status) {
  res.status = status;
  res.set_content(payload.dump(), "application/json");
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

std::string EntityTag(const std::string& version) { return '"' + version + '"'; }

bool EtagMatches(const std::optional<std::string>& header, const std::string& etag) {
  /// RFC 9110 13.1.2 は If-None-Match に weak 比較（opaque-tag だけ見比べる）を
  /// 要求するので、weak なタグは strong な同じ値に一致する。entity-tag 全体の文字列で
  /// 比べると weakness marker が差として残り、weak なクライアントを取りこぼす。
  ///
  /// 引用符無しの値も比較対象に残す: 旧サーバーは引用符なしで ETag を送っていたので、
  /// サーバーだけを更新してブラウザの HTTP キャッシュは古いままという隙間では旧形式が
  /// そのまま If-None-Match で戻ってくる。無視すると更新直後の 1 リクエストだけ地図を
  /// 全バイト再送し、304 で削りたかったコストを同じ手で戻す。値は地図内容そのものの
  /// hash なので、形式が不正でも一致判定は正しい。
  if (!header || header->empty()) {
    return false;
  }
  const std::optional<std::string> target = OpaqueTag(etag);
  if (!target) {
    return false;
  }
  std::size_t start = 0;
  while (true) {
    const std::size_t comma = header->find(',', start);
    const std::string element = header->substr(
        start, comma == std::string::npos ? std::string::npos : comma - start);
    const std::optional<std::string> tag = OpaqueTag(element);
    if (tag && (*tag == "*" || *tag == *target)) {
      return true;
    }
    if (comma == std::string::npos) {
      return false;
    }
    start = comma + 1;
  }
}

std::optional<std::string> JoinedIfNoneMatch(const httplib::Request& req) {
  const std::size_t count = req.get_header_value_count("If-None-Match");
  if (count == 0) {
    return std::nullopt;
  }
  std::string joined = req.get_header_value("If-None-Match", 0);
  for (std::size_t i = 1; i < count; ++i) {
    joined += ',';
    joined += req.get_header_value("If-None-Match", i);
  }
  return joined;
}

StaticFiles::StaticFiles(std::filesystem::path webui_dir) {
  std::error_code error;
  root_ = std::filesystem::weakly_canonical(webui_dir, error);
  if (error) {
    root_.clear();
  }
}

void StaticFiles::Serve(httplib::Response& res, const std::string& request_path,
                        const std::optional<std::string>& if_none_match) {
  if (request_path.size() < 2 || request_path.front() != '/') {
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
  const std::filesystem::path target = (root_ / relative).lexically_normal();
  if (!IsInside(root_, target)) {
    res.status = 404;
    return;
  }
  const std::optional<Entry> entry = Load(target, relative.generic_string());
  if (!entry) {
    res.status = 404;
    return;
  }
  res.set_header("ETag", entry->etag);
  // no-cache は「保存してよいが必ず再検証せよ」。地図と同じ扱い。
  res.set_header("Cache-Control", "no-cache");
  if (EtagMatches(if_none_match, entry->etag)) {
    res.status = 304;
    return;
  }
  res.set_content(*entry->body, MimeTypeOf(target));
}

std::optional<StaticFiles::Entry> StaticFiles::Load(const std::filesystem::path& target,
                                                    const std::string& key) {
  std::error_code error;
  const std::filesystem::file_time_type mtime =
      std::filesystem::last_write_time(target, error);
  if (error) {
    return std::nullopt;
  }
  const std::uintmax_t size = std::filesystem::file_size(target, error);
  if (error) {
    return std::nullopt;
  }
  {
    std::lock_guard<std::mutex> lock(lock_);
    const auto found = entries_.find(key);
    if (found != entries_.end() && found->second.mtime == mtime &&
        found->second.size == size) {
      return found->second;
    }
  }
  std::ifstream file(target, std::ios::binary);
  if (!file) {
    return std::nullopt;
  }
  // 版は (mtime, size)。opaque-tag は「同じ中身なら同じ」だけでよく、内容の
  // hash を取るために毎回ファイルを読む必要はない。
  Entry entry;
  entry.mtime = mtime;
  entry.size = size;
  entry.etag = EntityTag(std::to_string(mtime.time_since_epoch().count()) + "-" +
                         std::to_string(size));
  entry.body = std::make_shared<const std::string>(std::istreambuf_iterator<char>(file),
                                                   std::istreambuf_iterator<char>());
  {
    std::lock_guard<std::mutex> lock(lock_);
    if (entries_.size() >= kMaxCachedFiles) {
      entries_.clear();
    }
    entries_[key] = entry;
  }
  return entry;
}

}  // namespace umeonigiri_webnav
