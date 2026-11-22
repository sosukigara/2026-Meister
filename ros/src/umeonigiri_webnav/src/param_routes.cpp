// SPDX-License-Identifier: MIT
#include "umeonigiri_webnav/param_routes.hpp"

#include <exception>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "umeonigiri_webnav/http_support.hpp"

namespace umeonigiri_webnav {
namespace {

using json = nlohmann::json;

json ParameterToJson(const Parameter& parameter) {
  return json{{"name", parameter.name},
              {"type", parameter.type},
              {"value", parameter.value}};
}

}  // namespace

void RegisterParamRoutes(httplib::Server& server, ParamSource& param_source) {
  server.Get("/api/params/nodes",
             [&param_source](const httplib::Request&, httplib::Response& res) {
               try {
                 json nodes = json::array();
                 for (const std::string& node : param_source.Nodes()) {
                   nodes.push_back(node);
                 }
                 SendJson(res, json{{"nodes", nodes}});
               } catch (const std::exception& error) {
                 SendJson(res, json{{"error", error.what()}}, 500);
               }
             });

  server.Get("/api/params",
             [&param_source](const httplib::Request& req, httplib::Response& res) {
               const std::string node = req.get_param_value("node");
               if (node.empty()) {
                 SendJson(res, json{{"error", "node パラメータが必要です"}}, 400);
                 return;
               }
               std::vector<Parameter> parameters;
               const std::string error = param_source.List(node, &parameters);
               if (!error.empty()) {
                 // サービス未起動/ノード不在は操作の失敗。理由をそのまま出す。
                 SendJson(res, json{{"error", error}}, 409);
                 return;
               }
               json listed = json::array();
               for (const Parameter& parameter : parameters) {
                 listed.push_back(ParameterToJson(parameter));
               }
               SendJson(res, json{{"node", node}, {"parameters", listed}});
             });

  server.Post("/api/params",
              [&param_source](const httplib::Request& req, httplib::Response& res) {
                const json body = json::parse(req.body, nullptr, false);
                if (body.is_discarded() || !body.is_object()) {
                  SendJson(res, json{{"error", "invalid json"}}, 400);
                  return;
                }
                if (!body.contains("node") || !body["node"].is_string() ||
                    !body.contains("name") || !body["name"].is_string() ||
                    !body.contains("value")) {
                  SendJson(res,
                           json{{"error", "node, name, value が必要です"}}, 400);
                  return;
                }
                const std::string node = body["node"].get<std::string>();
                const std::string name = body["name"].get<std::string>();
                if (node.empty() || name.empty()) {
                  SendJson(res, json{{"error", "node と name は空にできません"}},
                           400);
                  return;
                }
                json actual = nullptr;
                const std::string error =
                    param_source.Set(node, name, body["value"], &actual);
                if (!error.empty()) {
                  // ノードの拒否理由を捨てると「押したのに変わらない」だけが残る。
                  SendJson(res, json{{"error", error}}, 409);
                  return;
                }
                SendJson(res, json{{"status", "ok"}, {"value", actual}});
              });
}

}  // namespace umeonigiri_webnav
