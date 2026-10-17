// SPDX-License-Identifier: MIT
/// Web UI サーバーの起動と配線。
///
/// 地図は専用 Executor、ナビゲータは NavWorker の専用スレッドで動かし、HTTP は
/// httplib のスレッドプールに任せる。地図の Node を NavWorker と共有すると、
/// worker 側の spin_until_future_complete と Executor が同じ Node を奪い合う。
#include <chrono>
#include <filesystem>
#include <iostream>
#include <memory>
#include <thread>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <httplib.h>
#include <rclcpp/rclcpp.hpp>

#include "meister_web_nav/http_app.hpp"
#include "meister_web_nav/map_listener.hpp"
#include "meister_web_nav/nav_worker.hpp"

namespace {

constexpr const char* kBindAddress = "127.0.0.1";
constexpr int kPort = 8088;

/// Web UI の実体。install 先しか見ない。実行ファイルは colcon build 後の install
/// から起動するので、Python 版が持っていたソースツリーへの fallback は不要。
std::filesystem::path WebuiDir() {
  return std::filesystem::path(
             ament_index_cpp::get_package_share_directory("meister_web_nav")) /
         "webui";
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto map_listener = std::make_shared<meister_web_nav::MapListener>();
  rclcpp::executors::SingleThreadedExecutor map_executor;
  map_executor.add_node(map_listener);
  std::thread map_thread([&map_executor] { map_executor.spin(); });

  meister_web_nav::NavWorker nav_worker;
  nav_worker.Start();

  httplib::Server server;
  meister_web_nav::RegisterRoutes(server, WebuiDir(), *map_listener, nav_worker);

  // rclcpp の SIGINT ハンドラは rclcpp::ok() を false にしてこのコールバックを呼ぶ。
  // httplib の listen は自分で止まるまで戻らないので、ここで止めないと Ctrl+C の後も
  // ポートを掴んだままになる。
  rclcpp::on_shutdown([&server] { server.stop(); });

  if (!server.bind_to_port(kBindAddress, kPort)) {
    std::cerr << "[meister_web_nav] " << kBindAddress << ":" << kPort
              << " を listen できない" << std::endl;
    nav_worker.Stop();
    map_executor.cancel();
    map_thread.join();
    rclcpp::shutdown();
    return 1;
  }
  // launch の output='screen' に出る唯一の起動合図。launch.sh の案内と揃えている。
  std::cout << "[meister_web_nav] Web UI ready at http://localhost:" << kPort
            << std::endl;

  server.listen_after_bind();

  nav_worker.Stop();
  map_executor.cancel();
  map_thread.join();
  rclcpp::shutdown();
  return 0;
}
