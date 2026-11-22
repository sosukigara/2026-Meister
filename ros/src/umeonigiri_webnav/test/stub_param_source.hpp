// SPDX-License-Identifier: MIT
/// パラメータ経路の test 用 stub。HTTP 面だけを見るので ROS は起動しない。
///
/// 一覧・設定の戻り値と、最後に渡された値を差し替えられるようにしておく。
/// 「成功したように見えるが値が渡っていない」を test で捕まえるため。
#ifndef UMEONIGIRI_WEBNAV_TEST_STUB_PARAM_SOURCE_HPP_
#define UMEONIGIRI_WEBNAV_TEST_STUB_PARAM_SOURCE_HPP_

#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "umeonigiri_webnav/ports.hpp"

namespace umeonigiri_webnav {
namespace test {

class StubParamSource : public ParamSource {
 public:
  std::vector<std::string> Nodes() override { return nodes; }

  std::string List(const std::string& node,
                   std::vector<Parameter>* parameters) override {
    last_list_node = node;
    if (!list_error.empty()) {
      return list_error;
    }
    *parameters = parameters_;
    return {};
  }

  std::string Set(const std::string& node, const std::string& name,
                  const nlohmann::json& value,
                  nlohmann::json* actual) override {
    last_set_node = node;
    last_set_name = name;
    last_set_value = value;
    if (!set_error.empty()) {
      return set_error;
    }
    if (actual != nullptr) {
      *actual = value;
    }
    return {};
  }

  std::vector<std::string> nodes{"/amcl", "/controller_server"};
  std::vector<Parameter> parameters_{
      Parameter{"enabled", "bool", true},
      Parameter{"desired_linear_vel", "double", 0.5}};
  std::string list_error;
  std::string set_error;
  std::string last_list_node;
  std::string last_set_node;
  std::string last_set_name;
  nlohmann::json last_set_value;
};

}  // namespace test
}  // namespace umeonigiri_webnav

#endif  // UMEONIGIRI_WEBNAV_TEST_STUB_PARAM_SOURCE_HPP_
