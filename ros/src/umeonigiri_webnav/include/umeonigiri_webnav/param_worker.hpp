// SPDX-License-Identifier: MIT
/// ROS パラメータの列挙・取得・設定（ros2 param 相当）を Web UI に提供する。
///
/// 呼び出しは HTTP スレッドから来る。ここで spin すると executor と衝突する
/// (docs/pitfalls.md P18) ので、ノードは専用スレッドの Executor で常に回し、
/// 呼び出し側は上限付きで future を待つだけにする。ノード探索も同じ理由で
/// 待たない: 一覧はグラフのキャッシュから返し、対象ノードのサービスが
/// 見つからなければその場で諦める。
#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/list_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "umeonigiri_webnav/ports.hpp"

namespace umeonigiri_webnav {

class ParamWorker : public ParamSource {
 public:
  ParamWorker();
  ~ParamWorker() override;

  ParamWorker(const ParamWorker&) = delete;
  ParamWorker& operator=(const ParamWorker&) = delete;

  /// executor スレッドを止める。rclcpp::shutdown() より先に呼ぶ。
  void Stop();

  std::vector<std::string> Nodes() override;
  std::string List(const std::string& node,
                   std::vector<Parameter>* parameters) override;
  std::string Set(const std::string& node, const std::string& name,
                  const nlohmann::json& value, nlohmann::json* actual) override;

 private:
  using ListParameters = rcl_interfaces::srv::ListParameters;
  using GetParameters = rcl_interfaces::srv::GetParameters;
  using SetParameters = rcl_interfaces::srv::SetParameters;
  using ParameterValue = rcl_interfaces::msg::ParameterValue;
  using ParameterType = rcl_interfaces::msg::ParameterType;

  struct Clients {
    rclcpp::Client<ListParameters>::SharedPtr list;
    rclcpp::Client<GetParameters>::SharedPtr get;
    rclcpp::Client<SetParameters>::SharedPtr set;
  };

  /// ノード名を ROS の完全名に正規化する。サービス名に化ける入力を弾く。
  static bool NormalizeNode(const std::string& node, std::string* out);

  /// ノードごとに 1 組のクライアントをキャッシュする。毎回作ると探索に数秒かかる。
  std::shared_ptr<Clients> ClientsFor(const std::string& node);

  /// 1 個の値を取る。List / Set の両方から使う。
  std::string GetOne(const std::shared_ptr<Clients>& clients,
                     const std::string& name, ParameterValue* value);

  static std::string TypeName(uint8_t type);
  static nlohmann::json ToJson(const ParameterValue& value);
  /// 既存の型に合わせて JSON を変換する。UI から型を信じないため。
  static bool FromJson(const nlohmann::json& value, uint8_t type,
                       ParameterValue* out, std::string* error);

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spinner_;
  /// 設定と取得は 1 本ずつ。同じノードへ同時に要求を積まない。
  std::mutex busy_;
  std::mutex clients_mutex_;
  std::map<std::string, std::shared_ptr<Clients>> clients_;
};

}  // namespace umeonigiri_webnav

