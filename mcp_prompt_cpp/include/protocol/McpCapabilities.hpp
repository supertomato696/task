#pragma once

#include <string>
#include <vector>
#include <optional>
#include <map>
#include <variant>
#include <nlohmann/json.hpp>
#include "McpRootsAndLoggingRequests.hpp"  // CancelledNotification, InitializedNotification, ProgressNotification, RootsListChangedNotification
#include "McpJsonRpc.hpp"               // JSONRPC* types
//#include "McpRootsAndLoggingRequests.hpp"

using json = nlohmann::json;

namespace mcp::protocol {


// ===== ClientCapabilities =====
struct ClientCapabilities {
    // 非标准实验性功能
    std::optional<std::map<std::string, json>> experimental;
    // 是否支持 roots 列表变化通知
    struct Roots { std::optional<bool> listChanged; };
    std::optional<Roots> roots;
    // 是否支持 sampling
    std::optional<json> sampling;
};

inline void to_json(json& j, const ClientCapabilities& c) {
    j = json::object();
    if (c.experimental) j["experimental"] = *c.experimental;
    if (c.roots) {
        j["roots"] = json::object();
        if (c.roots->listChanged) j["roots"]["listChanged"] = *c.roots->listChanged;
    }
    if (c.sampling) j["sampling"] = *c.sampling;
}

inline void from_json(const json& j, ClientCapabilities& c) {
    if (j.contains("experimental") && !j["experimental"].is_null()){
      c.experimental = j.at("experimental").get<std::map<std::string, json>>();
      }
//    if (j.contains("roots")) {
//        c.roots = ClientCapabilities::Roots{};
//        const auto& r = j["roots"];
//        if (r.contains("listChanged")) r.at("listChanged").get_to(c.roots->listChanged);
//    }
          if (j.contains("roots")) {
        c.roots = ClientCapabilities::Roots{};
        const auto& r = j["roots"];
        if (r.contains("listChanged")) {
            // 显式处理optional<bool>
            c.roots->listChanged = r.at("listChanged").get<bool>();
        }
    }
    if (j.contains("sampling")) j.at("sampling").get_to(c.sampling);
}

// ===== ClientNotification =====
using ClientNotification = std::variant<
    CancelledNotification,
    InitializedNotification,
    ProgressNotification,
    RootsListChangedNotification
>;

inline void to_json(json& j, const ClientNotification& n) {
    std::visit([&j](auto&& arg){ j = arg; }, n);
}

inline void from_json(const json& j, ClientNotification& n) {
    const std::string& method = j.at("method").get<std::string>();
    if (method == "notifications/cancelled")              n = j.get<CancelledNotification>();
    else if (method == "notifications/initialized")       n = j.get<InitializedNotification>();
    else if (method == "notifications/progress")          n = j.get<ProgressNotification>();
    else if (method == "notifications/roots/list_changed")n = j.get<RootsListChangedNotification>();
}

// ===== JSONRPCMessage =====
using JSONRPCMessage = std::variant<
    JSONRPCRequest,
    JSONRPCNotification,
    JSONRPCBatchRequest,
    JSONRPCResponse,
    JSONRPCErrorResponse,
    JSONRPCBatchResponse
>;

inline void to_json(json& j, const JSONRPCMessage& m) {
    std::visit([&j](auto&& arg){ j = arg; }, m);
}

inline void from_json(const json& j, JSONRPCMessage& m) {
    // 判别 JSON-RPC message 类型
    auto it = j.find("jsonrpc");
    if (it == j.end()) {
        // 批量请求或响应
        if (j.is_array()) {
            // 需更详细判别，此处默认
            m = j.get<JSONRPCBatchRequest>();
        }
        return;
    }
    std::string jsonrpc = j.at("jsonrpc").get<std::string>();
    if (jsonrpc != "2.0") return;
    if (j.contains("method") && j.contains("id"))          m = j.get<JSONRPCRequest>();
    else if (j.contains("method"))                          m = j.get<JSONRPCNotification>();
    else if (j.contains("result") && j.contains("id"))    m = j.get<JSONRPCResponse>();
    else if (j.contains("error")  && j.contains("id"))    m = j.get<JSONRPCErrorResponse>();
}

}