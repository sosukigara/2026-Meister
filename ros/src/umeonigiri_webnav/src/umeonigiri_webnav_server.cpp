// SPDX-License-Identifier: MIT
/// 地図と認識画像は専用 Executor、ナビゲータは NavWorker の専用スレッドで動かし、
/// HTTP は httplib のスレッドプールに任せる。地図の Node を NavWorker と共有すると、
/// worker 側の spin_until_future_complete と Executor が同じ Node を奪い合う。
#include <chrono>
#include <cstdlib>
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
#include "umeonigiri_webnav/param_worker.hpp"
#include "umeonigiri_webnav/vision_listener.hpp"

namespace {

// 同じ LAN の別 PC から使いたいという要求があるため、全インタフェースで待ち受ける。
// 認証は無いので、公開範囲が要らないときは MEISTER_WEBUI_BIND=127.0.0.1 で絞る。
constexpr const char* kBindAddress = "0.0.0.0";
constexpr int kPort = 8088;

/// 待ち受けアドレスを決める。実機・シミュレーションどちらでも同じ値を環境変数から
/// 取れるようにして、起動スクリプト側に設定を二重に持たせない。
const char* BindAddress() {
  const char* from_env = std::getenv("MEISTER_WEBUI_BIND");
  return (from_env != nullptr && from_env[0] != '\0') ? from_env : kBindAddress;
}

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
  // パラメータ操作も専用 executor で回す。HTTP ハンドラから spin すると
  // 他のノードと executor を奪い合い、画面全体が止まる (P16/P18)。
  umeonigiri_webnav::ParamWorker param_worker;

  httplib::Server server;
  umeonigiri_webnav::RegisterRoutes(server, *webui_dir, *map_listener, nav_worker,
                                    *vision_listener, mode_worker, param_worker);

  // rclcpp の SIGINT ハンドラは rclcpp::ok() を false にしてこのコールバックを呼ぶ。
  // httplib の listen は自分で止まるまで戻らないので、ここで止めないと Ctrl+C の後も
  // ポートを掴んだままになる。
  rclcpp::on_shutdown([&server] { server.stop(); });

  const char* const bind_address = BindAddress();
  if (!server.bind_to_port(bind_address, kPort)) {
    std::cerr << "[umeonigiri_webnav] " << bind_address << ":" << kPort
              << " を listen できない" << std::endl;
    mode_worker.Stop();
    param_worker.Stop();
    nav_worker.Stop();
    ros_executor.cancel();
    ros_thread.join();
    rclcpp::shutdown();
    return 1;
  }
  // launch の output='screen' に出る唯一の起動合図。launch.sh の案内と揃えている。
  // 全インタフェース待ち受け時は localhost だけでは足りないので実アドレスも出す。
  std::cout << "[umeonigiri_webnav] Web UI ready at http://localhost:" << kPort
            << " (bind " << bind_address << ":" << kPort << ")" << std::endl;

  server.listen_after_bind();

  // rclcpp::shutdown() より先に、自前スレッドを止める。
  mode_worker.Stop();
  param_worker.Stop();
  nav_worker.Stop();
  ros_executor.cancel();
  ros_thread.join();
  rclcpp::shutdown();
  return 0;
}
