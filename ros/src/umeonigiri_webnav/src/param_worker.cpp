// SPDX-License-Identifier: MIT
#include "umeonigiri_webnav/param_worker.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>

namespace umeonigiri_webnav {
namespace {

// 一覧を引くたびに数秒待たない。グラフ探索は executor スレッドが進めているので、
// 見つからないなら「この構成には居ない」と判断してよい（P16 と同じ方針）。
constexpr auto kServiceWait = std::chrono::milliseconds(200);
constexpr auto kResponseWait = std::chrono::seconds(2);
// 1 リクエストで受け付ける配列長。本文上限だけでは短時間に大量の
// パラメータを積めるため、型変換の手前で切る。
constexpr std::size_t kMaxArrayLength = 10000;

/// rclcpp が内部で作る補助ノード。パラメータサービスはあっても、
/// 画面の操作対象としては意味がなく、選ぶとサービス名の取り違えに見える
/// エラーだけが増える。ノード一覧から外す。
bool IsHelperNode(const std::string& name) {
  if (name.rfind("/_ros2cli_", 0) == 0 || name.rfind("/launch_ros_", 0) == 0) {
    return true;
  }
  if (name.find("/transform_listener_impl_") != std::string::npos) {
    return true;
  }
  constexpr const char* kActionHelperSuffix = "_rclcpp_node";
  const std::string suffix(kActionHelperSuffix);
  return name.size() > suffix.size() &&
         name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0;
}

bool IsNodeChar(char value) {
  return (value >= 'a' && value <= 'z') || (value >= 'A' && value <= 'Z') ||
         (value >= '0' && value <= '9') || value == '_';
}

}  // namespace

ParamWorker::ParamWorker()
    : node_(std::make_shared<rclcpp::Node>("umeonigiri_webnav_param")) {
  executor_.add_node(node_);
  spinner_ = std::thread([this] { executor_.spin(); });
}

ParamWorker::~ParamWorker() { Stop(); }

void ParamWorker::Stop() {
  // ModeWorker と同じ理由で、終了済みの context に cancel() を撃たない。
  if (!spinner_.joinable()) {
    return;
  }
  executor_.cancel();
  spinner_.join();
}

std::vector<std::string> ParamWorker::Nodes() {
  std::vector<std::string> names;
  for (const std::string& name : node_->get_node_names()) {
    std::string normalized;
    if (!IsHelperNode(name) && NormalizeNode(name, &normalized)) {
      names.push_back(normalized);
    }
  }
  std::sort(names.begin(), names.end());
  names.erase(std::unique(names.begin(), names.end()), names.end());
  return names;
}

bool ParamWorker::NormalizeNode(const std::string& node, std::string* out) {
  if (node.empty() || node.size() > 256 || node.front() != '/') {
    return false;
  }
  std::string normalized = node;
  while (normalized.size() > 1 && normalized.back() == '/') {
    normalized.pop_back();
  }
  if (normalized == "/") {
    return false;
  }
  // 先頭の '/' は完全名の印。以降の '/' は名前空間の区切りとしてだけ許す。
  bool previous_slash = false;
  for (std::size_t index = 1; index < normalized.size(); ++index) {
    const char value = normalized[index];
    if (value == '/') {
      // "//" や末尾 "/" はサービス名の組み立てを壊すので受けない。
      if (previous_slash) {
        return false;
      }
      previous_slash = true;
      continue;
    }
    if (!IsNodeChar(value)) {
      return false;
    }
    previous_slash = false;
  }
  if (previous_slash) {
    return false;
  }
  *out = std::move(normalized);
  return true;
}

std::shared_ptr<ParamWorker::Clients> ParamWorker::ClientsFor(
    const std::string& node) {
  std::lock_guard<std::mutex> guard(clients_mutex_);
  const auto found = clients_.find(node);
  if (found != clients_.end()) {
    return found->second;
  }
  auto clients = std::make_shared<Clients>();
  clients->list = node_->create_client<ListParameters>(node + "/list_parameters");
  clients->get = node_->create_client<GetParameters>(node + "/get_parameters");
  clients->set = node_->create_client<SetParameters>(node + "/set_parameters");
  clients_.emplace(node, clients);
  return clients;
}

std::string ParamWorker::TypeName(uint8_t type) {
  using namespace rcl_interfaces::msg;  // NOLINT(build/namespaces)
  switch (type) {
    case ParameterType::PARAMETER_BOOL:
      return "bool";
    case ParameterType::PARAMETER_INTEGER:
      return "integer";
    case ParameterType::PARAMETER_DOUBLE:
      return "double";
    case ParameterType::PARAMETER_STRING:
      return "string";
    case ParameterType::PARAMETER_BYTE_ARRAY:
      return "byte_array";
    case ParameterType::PARAMETER_BOOL_ARRAY:
      return "bool_array";
    case ParameterType::PARAMETER_INTEGER_ARRAY:
      return "integer_array";
    case ParameterType::PARAMETER_DOUBLE_ARRAY:
      return "double_array";
    case ParameterType::PARAMETER_STRING_ARRAY:
      return "string_array";
    default:
      return "not_set";
  }
}

nlohmann::json ParamWorker::ToJson(const ParameterValue& value) {
  using namespace rcl_interfaces::msg;  // NOLINT(build/namespaces)
  switch (value.type) {
    case ParameterType::PARAMETER_BOOL:
      return value.bool_value;
    case ParameterType::PARAMETER_INTEGER:
      return value.integer_value;
    case ParameterType::PARAMETER_DOUBLE:
      return value.double_value;
    case ParameterType::PARAMETER_STRING:
      return value.string_value;
    case ParameterType::PARAMETER_BYTE_ARRAY:
      return value.byte_array_value;
    case ParameterType::PARAMETER_BOOL_ARRAY:
      return value.bool_array_value;
    case ParameterType::PARAMETER_INTEGER_ARRAY:
      return value.integer_array_value;
    case ParameterType::PARAMETER_DOUBLE_ARRAY:
      return value.double_array_value;
    case ParameterType::PARAMETER_STRING_ARRAY:
      return value.string_array_value;
    default:
      return nullptr;
  }
}

bool ParamWorker::FromJson(const nlohmann::json& value, uint8_t type,
                           ParameterValue* out, std::string* error) {
  using namespace rcl_interfaces::msg;  // NOLINT(build/namespaces)
  out->type = type;
  const std::string expected = TypeName(type);
  switch (type) {
    case ParameterType::PARAMETER_BOOL:
      if (!value.is_boolean()) {
        *error = "型が違います (期待: bool)";
        return false;
      }
      out->bool_value = value.get<bool>();
      return true;
    case ParameterType::PARAMETER_INTEGER: {
      if (!value.is_number_integer()) {
        *error = "型が違います (期待: integer)";
        return false;
      }
      if (value.is_number_unsigned() &&
          value.get<std::uint64_t>() >
              static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) {
        *error = "integer の範囲を超えています";
        return false;
      }
      out->integer_value = value.get<std::int64_t>();
      return true;
    }
    case ParameterType::PARAMETER_DOUBLE: {
      if (!value.is_number()) {
        *error = "型が違います (期待: double)";
        return false;
      }
      const auto number = value.get<double>();
      if (!std::isfinite(number)) {
        *error = "double が有限値ではありません";
        return false;
      }
      out->double_value = number;
      return true;
    }
    case ParameterType::PARAMETER_STRING:
      if (!value.is_string()) {
        *error = "型が違います (期待: string)";
        return false;
      }
      out->string_value = value.get<std::string>();
      return true;
    case ParameterType::PARAMETER_BYTE_ARRAY: {
      if (!value.is_array() || value.size() > kMaxArrayLength) {
        *error = "型が違います (期待: byte_array。配列で指定)";
        return false;
      }
      out->byte_array_value.reserve(value.size());
      for (const auto& element : value) {
        if (!element.is_number_integer()) {
          *error = "byte_array の要素は整数で指定してください";
          return false;
        }
        const auto byte = element.get<std::int64_t>();
        if (byte < 0 || byte > 255) {
          *error = "byte_array の要素は 0..255 です";
          return false;
        }
        out->byte_array_value.push_back(static_cast<std::uint8_t>(byte));
      }
      return true;
    }
    case ParameterType::PARAMETER_BOOL_ARRAY: {
      if (!value.is_array() || value.size() > kMaxArrayLength) {
        *error = "型が違います (期待: bool_array。配列で指定)";
        return false;
      }
      for (const auto& element : value) {
        if (!element.is_boolean()) {
          *error = "bool_array の要素は true/false で指定してください";
          return false;
        }
        out->bool_array_value.push_back(element.get<bool>());
      }
      return true;
    }
    case ParameterType::PARAMETER_INTEGER_ARRAY: {
      if (!value.is_array() || value.size() > kMaxArrayLength) {
        *error = "型が違います (期待: integer_array。配列で指定)";
        return false;
      }
      for (const auto& element : value) {
        if (!element.is_number_integer()) {
          *error = "integer_array の要素は整数で指定してください";
          return false;
        }
        if (element.is_number_unsigned() &&
            element.get<std::uint64_t>() >
                static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) {
          *error = "integer_array の要素が範囲外です";
          return false;
        }
        out->integer_array_value.push_back(element.get<std::int64_t>());
      }
      return true;
    }
    case ParameterType::PARAMETER_DOUBLE_ARRAY: {
      if (!value.is_array() || value.size() > kMaxArrayLength) {
        *error = "型が違います (期待: double_array。配列で指定)";
        return false;
      }
      for (const auto& element : value) {
        if (!element.is_number() || !std::isfinite(element.get<double>())) {
          *error = "double_array の要素は数値で指定してください";
          return false;
        }
        out->double_array_value.push_back(element.get<double>());
      }
      return true;
    }
    case ParameterType::PARAMETER_STRING_ARRAY: {
      if (!value.is_array() || value.size() > kMaxArrayLength) {
        *error = "型が違います (期待: string_array。配列で指定)";
        return false;
      }
      for (const auto& element : value) {
        if (!element.is_string()) {
          *error = "string_array の要素は文字列で指定してください";
          return false;
        }
        out->string_array_value.push_back(element.get<std::string>());
      }
      return true;
    }
    default:
      *error = "未対応のパラメータ型です (" + expected + ")";
      return false;
  }
}

std::string ParamWorker::GetOne(const std::shared_ptr<Clients>& clients,
                                const std::string& name,
                                ParameterValue* value) {
  auto request = std::make_shared<GetParameters::Request>();
  request->names = {name};
  auto future = clients->get->async_send_request(request);
  if (future.wait_for(kResponseWait) != std::future_status::ready) {
    return "get_parameters が応答しません";
  }
  const auto response = future.get();
  if (response->values.size() != request->names.size()) {
    return "get_parameters の応答数が一致しません";
  }
  using namespace rcl_interfaces::msg;  // NOLINT(build/namespaces)
  if (response->values.front().type == ParameterType::PARAMETER_NOT_SET) {
    return "パラメータ " + name + " は存在しません";
  }
  *value = response->values.front();
  return {};
}

std::string ParamWorker::List(const std::string& node,
                              std::vector<Parameter>* parameters) {
  std::string full;
  if (!NormalizeNode(node, &full)) {
    return "ノード名が不正です";
  }
  std::lock_guard<std::mutex> guard(busy_);
  const auto clients = ClientsFor(full);
  if (!clients->list->wait_for_service(kServiceWait)) {
    return full + " のパラメータサービスが見つかりません";
  }

  auto request = std::make_shared<ListParameters::Request>();
  // 0 は深さ制限なし。名前空間だけに絞ると FollowPath.* 等が欠ける。
  request->depth = 0;
  auto future = clients->list->async_send_request(request);
  if (future.wait_for(kResponseWait) != std::future_status::ready) {
    return "list_parameters が応答しません";
  }
  std::vector<std::string> names = future.get()->result.names;
  if (names.empty()) {
    parameters->clear();
    return {};
  }
  if (!clients->get->wait_for_service(kServiceWait)) {
    return "get_parameters が見つかりません";
  }

  auto get_request = std::make_shared<GetParameters::Request>();
  get_request->names = names;
  auto get_future = clients->get->async_send_request(get_request);
  if (get_future.wait_for(kResponseWait) != std::future_status::ready) {
    return "get_parameters が応答しません";
  }
  const auto values = get_future.get()->values;
  if (values.size() != names.size()) {
    return "get_parameters の応答数が一致しません";
  }

  std::vector<Parameter> listed;
  listed.reserve(names.size());
  for (std::size_t index = 0; index < names.size(); ++index) {
    listed.push_back(Parameter{names[index], TypeName(values[index].type),
                               ToJson(values[index])});
  }
  std::sort(listed.begin(), listed.end(),
            [](const Parameter& left, const Parameter& right) {
              return left.name < right.name;
            });
  *parameters = std::move(listed);
  return {};
}

std::string ParamWorker::Set(const std::string& node, const std::string& name,
                             const nlohmann::json& value,
                             nlohmann::json* actual) {
  std::string full;
  if (!NormalizeNode(node, &full)) {
    return "ノード名が不正です";
  }
  if (name.empty() || name.size() > 512) {
    return "パラメータ名が不正です";
  }
  std::lock_guard<std::mutex> guard(busy_);
  const auto clients = ClientsFor(full);
  if (!clients->get->wait_for_service(kServiceWait)) {
    return full + " のパラメータサービスが見つかりません";
  }

  // 型はノードが持っている既存の定義に合わせる。UI から送られた型を信じると、
  // double のつもりで integer を送ったときに静かに別の型へ変わる。
  ParameterValue current;
  std::string error = GetOne(clients, name, &current);
  if (!error.empty()) {
    return error;
  }
  ParameterValue converted;
  if (!FromJson(value, current.type, &converted, &error)) {
    return error;
  }

  auto request = std::make_shared<SetParameters::Request>();
  rcl_interfaces::msg::Parameter parameter;
  parameter.name = name;
  parameter.value = converted;
  request->parameters.push_back(parameter);
  if (!clients->set->wait_for_service(kServiceWait)) {
    return "set_parameters が見つかりません";
  }
  auto future = clients->set->async_send_request(request);
  if (future.wait_for(kResponseWait) != std::future_status::ready) {
    return "set_parameters が応答しません";
  }
  const auto response = future.get();
  if (response->results.size() != request->parameters.size()) {
    return "set_parameters の応答数が一致しません";
  }
  if (!response->results.front().successful) {
    const std::string reason = response->results.front().reason;
    return reason.empty() ? "ノードが値を受け付けませんでした" : reason;
  }

  // ノードによってはクランプ/正規化する。画面には実際の値を返す。
  ParameterValue applied;
  if (GetOne(clients, name, &applied).empty()) {
    *actual = ToJson(applied);
  } else {
    *actual = value;
  }
  return {};
}

}  // namespace umeonigiri_webnav
