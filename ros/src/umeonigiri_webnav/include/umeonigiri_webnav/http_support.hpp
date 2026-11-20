// SPDX-License-Identifier: MIT
/// HTTP 応答の共通部品。静的ファイル配信・ETag・JSON 送信をここ 1 箇所に置く。
///
/// http_app.cpp と vision_routes.cpp の両方が使う。片方に置いて他方から include
/// すると、経路を足すたびに依存の向きが入れ替わる。
#ifndef UMEONIGIRI_WEBNAV_HTTP_SUPPORT_HPP_
#define UMEONIGIRI_WEBNAV_HTTP_SUPPORT_HPP_

#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include <httplib.h>
#include <nlohmann/json.hpp>

namespace umeonigiri_webnav {

void SendJson(httplib::Response& res, const nlohmann::json& payload, int status = 200);

std::string MimeTypeOf(const std::filesystem::path& path);

/// target が root の中にあるか。root 自身は「ファイルではない」ので false。
bool IsInside(const std::filesystem::path& root, const std::filesystem::path& target);

/// content hash を RFC 9110 8.8.3 の entity-tag に包む。
std::string EntityTag(const std::string& version);

/// If-None-Match が etag に一致するか（weak 比較）。
bool EtagMatches(const std::optional<std::string>& header, const std::string& etag);

/// If-None-Match を全ヘッダ行まとめて 1 本にする。
///
/// RFC 9110 5.2 のフィールド合成（同名フィールドの連結）をそのままやる。
/// get_header_value だけだと 2 行目以降を落とし、正規のクライアントが 2 行送った
/// ときに対象のタグを見失う。
std::optional<std::string> JoinedIfNoneMatch(const httplib::Request& req);

/// webui_dir 配下の静的ファイルを配る。
///
/// 中身は (mtime, size) が変わるまでメモリに保持する。ブラウザは 1 画面で
/// 13 本の asset を取るので、毎回ディスクから読むとページ表示のたびに無駄が積まる。
/// 変わっていないことは ETag で伝え、再読み込みは 304 で済ませる。
class StaticFiles {
 public:
  explicit StaticFiles(std::filesystem::path webui_dir);

  /// request_path を解決して応答を組み立てる。無ければ 404。
  /// if_none_match が etag と一致すれば 304 (本文なし)。
  ///
  /// 要求パスの ".." は lexical に潰して弾く (シンボリックリンクは辿る:
  /// --symlink-install の share 配下はソースを指す)。
  void Serve(httplib::Response& res, const std::string& request_path,
             const std::optional<std::string>& if_none_match);

 private:
  /// 一致判定に使う版。ファイルの (mtime, size) がそのまま版になるので、
  /// 中身を読み直さずに「変わったか」を判定できる。
  struct Entry {
    std::filesystem::file_time_type mtime{};
    std::uintmax_t size = 0;
    std::string etag;
    std::shared_ptr<const std::string> body;
  };

  /// キャッシュ済みの中身を返す。読めないファイルは nullopt。
  std::optional<Entry> Load(const std::filesystem::path& target, const std::string& key);

  std::filesystem::path root_;
  std::mutex lock_;
  std::unordered_map<std::string, Entry> entries_;
};

}  // namespace umeonigiri_webnav

#endif  // UMEONIGIRI_WEBNAV_HTTP_SUPPORT_HPP_
