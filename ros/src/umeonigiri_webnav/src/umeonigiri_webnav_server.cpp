// SPDX-License-Identifier: MIT
/// Web UI サーバーの起動と配線。
///
/// 地図と認識画像は専用 Executor、ナビゲータは NavWorker の専用スレッドで動かし、
/// HTTP は httplib のスレッドプールに任せる。地図の Node を NavWorker と共有すると、
/// worker 側の spin_until_future_complete と Executor が同じ Node を奪い合う。
#include <chrono>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <thread>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <httplib.h>
#include <rclcpp/rclcpp.hpp>

#include "umeonigiri_webnav/http_app.hpp"
#include "umeonigiri_webnav/map_listener.hpp"
#include "umeonigiri_webnav/mode_worker.hpp"
#include "umeonigiri_webnav/nav_worker.hpp"
#include "umeonigiri_webnav/vision_listener.hpp"

namespace {

constexpr const char* kBindAddress = "127.0.0.1";
constexpr int kPort = 8088;

/// Web UI の実体。install 先しか見ない。実行ファイルは colcon build 後の install
/// から起動するので、Python 版が持っていたソースツリーへの fallback は不要。
///
/// 見つからないときに ament_index_cpp は例外を投げる。素で通すと std::terminate で
/// core を吐くだけになり、原因 (workspace を source していない) が画面に出ない。
std::optional<std::filesystem::path> WebuiDir() {
  try {
    return std::filesystem::path(
               ament_index_cpp::get_package_share_directory("umeonigiri_webnav")) /
           "webui";
  } catch (const std::exception& error) {
    std::cerr << "[umeonigiri_webnav] Web UI の share ディレクトリを解決できない: "
              << error.what() << std::endl;
    return std::nullopt;
  }
}

}  // namespace

int main(int argc, char** argv) {
  const std::optional<std::filesystem::path> webui_dir = WebuiDir();
  if (!webui_dir) {
    return 1;
  }

  rclcpp::init(argc, argv);

  auto map_listener = std::make_shared<umeonigiri_webnav::MapListener>();
  auto vision_listener = std::make_shared<umeonigiri_webnav::VisionListener>();
  rclcpp::executors::SingleThreadedExecutor ros_executor;
  ros_executor.add_node(map_listener);
  ros_executor.add_node(vision_listener);
  std::thread ros_thread([&ros_executor] { ros_executor.spin(); });

  umeonigiri_webnav::NavWorker nav_worker;
  nav_worker.Start();
  umeonigiri_webnav::ModeWorker mode_worker;

  httplib::Server server;
  umeonigiri_webnav::RegisterRoutes(server, *webui_dir, *map_listener, nav_worker,
                                    *vision_listener, mode_worker);

  // rclcpp の SIGINT ハンドラは rclcpp::ok() を false にしてこのコールバックを呼ぶ。
  // httplib の listen は自分で止まるまで戻らないので、ここで止めないと Ctrl+C の後も
  // ポートを掴んだままになる。
  rclcpp::on_shutdown([&server] { server.stop(); });

  if (!server.bind_to_port(kBindAddress, kPort)) {
    std::cerr << "[umeonigiri_webnav] " << kBindAddress << ":" << kPort
              << " を listen できない" << std::endl;
    mode_worker.Stop();
    nav_worker.Stop();
    ros_executor.cancel();
    ros_thread.join();
    rclcpp::shutdown();
    return 1;
  }
  // launch の output='screen' に出る唯一の起動合図。launch.sh の案内と揃えている。
  std::cout << "[umeonigiri_webnav] Web UI ready at http://localhost:" << kPort
            << std::endl;

  server.listen_after_bind();

  // rclcpp::shutdown() より先に、自前スレッドを止める。
  mode_worker.Stop();
  nav_worker.Stop();
  ros_executor.cancel();
  ros_thread.join();
  rclcpp::shutdown();
  return 0;
}
