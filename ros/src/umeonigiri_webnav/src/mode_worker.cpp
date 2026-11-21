// SPDX-License-Identifier: MIT
#include "umeonigiri_webnav/mode_worker.hpp"

#include <cmath>
#include <ctime>
#include <filesystem>
#include <future>

#include "umeonigiri_webnav/map_store.hpp"

namespace umeonigiri_webnav {
namespace {

// 応答待ちは長く取る。切替は lifecycle 遷移を 2 回挟むので 3 秒では足りない
// （成功していてもタイムアウトに見える）。探索待ちは短くする: ここで待つ間は
// HTTP スレッドが 1 本埋まるので、長く取ると画面の他の要求を巻き込む。
constexpr auto kServiceWait = std::chrono::seconds(2);
constexpr auto kSwitchResponseWait = std::chrono::seconds(40);
constexpr auto kSaveResponseWait = std::chrono::seconds(20);
constexpr auto kModeResponseWait = std::chrono::seconds(2);

std::string TimestampNow() {
  const std::time_t now = std::time(nullptr);
  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%Y%m%d-%H%M%S", std::localtime(&now));
  return buffer;
}

}  // namespace

ModeWorker::ModeWorker(const std::string& maps_dir)
    : node_(std::make_shared<rclcpp::Node>("umeonigiri_webnav_mode")) {
  maps_dir_ = maps_dir.empty()
                  ? node_->declare_parameter<std::string>("maps_dir", "maps")
                  : maps_dir;
  // 地図名の既定 <world>_<timestamp> に使う。launch から渡らない構成でも
  // map にフォールバックする (DefaultMapName)。
  world_ = node_->declare_parameter<std::string>("world", "");

  mode_client_ = node_->create_client<Trigger>("get_mode");
  switch_client_ = node_->create_client<Trigger>("switch_to_localization");
  mapping_client_ = node_->create_client<Trigger>("switch_to_mapping");
  save_client_ = node_->create_client<SaveMapService>("/map_saver/save_map");
  pose_publisher_ =
      node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/initialpose", 10);

  // 探索はこのスレッドでしか進まない。HTTP 側で spin すると executor と衝突する。
  executor_.add_node(node_);
  spinner_ = std::thread([this] { executor_.spin(); });
}

ModeWorker::~ModeWorker() { Stop(); }

void ModeWorker::Stop() {
  // 2 回目以降は何もしない。main() が rclcpp::shutdown() より先に Stop() を呼び、
  // そのあとデストラクタがもう一度呼ぶので、終了済みの context に対して
  // cancel() を撃たないようにする。
  if (!spinner_.joinable()) {
    return;
  }
  executor_.cancel();
  spinner_.join();
}

bool ModeWorker::CallTrigger(const rclcpp::Client<Trigger>::SharedPtr& client,
                             std::string* message) {
  auto future = client->async_send_request(std::make_shared<Trigger::Request>());
  if (future.wait_for(kModeResponseWait) != std::future_status::ready) {
    *message = std::string(client->get_service_name()) + " が応答しません";
    return false;
  }
  // future からは 1 回だけ取り出す。2 回 get() すると
  // "No associated state" で落ちる。
  const auto response = future.get();
  *message = response->message;
  return response->success;
}

std::string ModeWorker::Mode() {
  // 画面は 2 秒ごとに引く。サービスが居ない構成 (web_nav 単体起動) で待つと
  // HTTP スレッドが数秒埋まり、同じ画面の地図や姿勢の要求まで止まる。
  // 探索は executor スレッドが常に進めているので、ここでは一切待たない。
  if (!mode_client_->service_is_ready()) {
    return "unknown";
  }
  std::string message;
  if (!CallTrigger(mode_client_, &message)) {
    return "unknown";
  }
  return message;
}

std::string ModeWorker::SwitchToLocalization() {
  return SwitchVia(switch_client_, "switch_to_localization");
}

std::string ModeWorker::SwitchToMapping() {
  return SwitchVia(mapping_client_, "switch_to_mapping");
}

std::string ModeWorker::SwitchVia(const rclcpp::Client<Trigger>::SharedPtr& client,
                                  const char* name) {
  std::unique_lock<std::mutex> guard(busy_, std::try_to_lock);
  if (!guard.owns_lock()) {
    return "別の操作を実行中です";
  }
  if (!client->wait_for_service(kServiceWait)) {
    return std::string(name) + " が見つかりません";
  }
  auto future = client->async_send_request(std::make_shared<Trigger::Request>());
  if (future.wait_for(kSwitchResponseWait) != std::future_status::ready) {
    return std::string(name) + " が応答しません";
  }
  const auto response = future.get();
  if (response->success) {
    return {};
  }
  return response->message;
}

std::vector<std::string> ModeWorker::ListMaps() { return ListMapNames(maps_dir_); }

std::string ModeWorker::SaveMap(const std::string& name) {
  std::unique_lock<std::mutex> guard(busy_, std::try_to_lock);
  if (!guard.owns_lock()) {
    return "別の操作を実行中です";
  }
  // 空名は既定名にする。空のまま渡すと保存先が maps ディレクトリ自身になり、
  // 一覧 (拡張子 .yaml) に出ないファイルができる。
  const std::string resolved = MapNameOrDefault(name, world_, TimestampNow());

  std::error_code error;
  std::filesystem::create_directories(maps_dir_, error);
  // SaveMap は map_url を要求する（パラメータではない）。絶対パスに file:// を付ける。
  const std::filesystem::path target =
      std::filesystem::absolute(std::filesystem::path(maps_dir_) / resolved);
  std::filesystem::create_directories(target.parent_path(), error);

  if (!save_client_->wait_for_service(kServiceWait)) {
    return "map_saver が見つかりません";
  }
  auto request = std::make_shared<SaveMapService::Request>();
  request->map_topic = "/map";
  // nav2 の map_saver は file:// を剥がさずに ImageMagick へ渡すため、
  // 素の絶対パスを渡す（file:// を付けると "No encode delegate" で失敗する）。
  request->map_url = target.string();
  // この環境の ImageMagick は PGM の encode delegate を持たない。PNG で保存する。
  request->image_format = "png";
  request->map_mode = "trinary";
  request->free_thresh = 0.25f;
  request->occupied_thresh = 0.65f;
  auto future = save_client_->async_send_request(request);
  if (future.wait_for(kSaveResponseWait) != std::future_status::ready) {
    return "map_saver が応答しません";
  }
  if (!future.get()->result) {
    return "地図の保存に失敗しました";
  }
  return {};
}

std::string ModeWorker::DeleteMap(const std::string& name) {
  // map_saver は .yaml と画像 (.png / 旧 .pgm) の 2 ファイルを書く。
  // yaml だけ消すと画像が maps に溜まり続ける。
  return DeleteMapFiles(maps_dir_, name);
}

void ModeWorker::SetInitialPose(double x, double y, double yaw) {
  geometry_msgs::msg::PoseWithCovarianceStamped message;
  message.header.frame_id = "map";
  message.header.stamp = node_->now();
  message.pose.pose.position.x = x;
  message.pose.pose.position.y = y;
  message.pose.pose.orientation.z = std::sin(yaw / 2.0);
  message.pose.pose.orientation.w = std::cos(yaw / 2.0);
  for (double& element : message.pose.covariance) {
    element = 0.0;
  }
  message.pose.covariance[0] = 0.25;
  message.pose.covariance[7] = 0.25;
  message.pose.covariance[35] = 0.07;
  // 1 回だけだと落とすことがある。モード切替で AMCL を起こした直後は購読が
  // まだ張られておらず、単発 publish が捨てられて
  // 「Please set the initial pose...」のまま固まる（実測で再現）。
  // 0.5 秒かけて複数回流し、確実に届かせる。
  for (int attempt = 0; attempt < 10; ++attempt) {
    message.header.stamp = node_->now();
    pose_publisher_->publish(message);
    rclcpp::sleep_for(std::chrono::milliseconds(50));
  }
}

}  // namespace umeonigiri_webnav
