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


// ServerCapabilities (extension)
struct ServerCapabilities {
    std::optional<json> completions;
    std::optional<json> experimental;
    std::optional<json> logging;
    struct Prompts { std::optional<bool> listChanged;
   		 // 添加完整的序列化方法（替代宏）
        friend void to_json(json& j, const Prompts& p) {
            j = json{};
            if (p.listChanged) j["listChanged"] = *p.listChanged;
        }
        friend void from_json(const json& j, Prompts& p) {
            if (j.contains("listChanged")) {
                p.listChanged = j.at("listChanged").get<bool>();
            }
        }
    };
    std::optional<Prompts> prompts;
    struct Resources { std::optional<bool> listChanged; std::optional<bool> subscribe;
      	 // 添加完整的序列化方法
        friend void to_json(json& j, const Resources& r) {
            j = json{};
            if (r.listChanged) j["listChanged"] = *r.listChanged;
            if (r.subscribe)   j["subscribe"]   = *r.subscribe;
        }
        friend void from_json(const json& j, Resources& r) {
            if (j.contains("listChanged")) r.listChanged = j.at("listChanged").get<bool>();
            if (j.contains("subscribe"))   r.subscribe   = j.at("subscribe").get<bool>();
        }

    };
    std::optional<Resources> resources;
    struct Tools { std::optional<bool> listChanged;
 // 添加完整的序列化方法
        friend void to_json(json& j, const Tools& t) {
            j = json{};
            if (t.listChanged) j["listChanged"] = *t.listChanged;
        }
        friend void from_json(const json& j, Tools& t) {
            if (j.contains("listChanged")) {
                t.listChanged = j.at("listChanged").get<bool>();
            }
        }
    };
    std::optional<Tools> tools;
};
inline void to_json(json& j, const ServerCapabilities& s) {
    j = json{};
    if (s.completions)   j["completions"]   = *s.completions;
    if (s.experimental)  j["experimental"]  = *s.experimental;
    if (s.logging)       j["logging"]       = *s.logging;
    if (s.prompts) {
        j["prompts"]["listChanged"] = *s.prompts->listChanged;
    }
    if (s.resources) {
        auto& r = *s.resources;
        j["resources"]["listChanged"] = *r.listChanged;
        j["resources"]["subscribe"]   = *r.subscribe;
    }
    if (s.tools) j["tools"]["listChanged"] = *s.tools->listChanged;
}
inline void from_json(const json& j, ServerCapabilities& s) {
    if (j.contains("completions"))   j.at("completions").get_to(s.completions);
    if (j.contains("experimental"))  j.at("experimental").get_to(s.experimental);
    if (j.contains("logging"))       j.at("logging").get_to(s.logging);
    if (j.contains("prompts"))      s.prompts = j.at("prompts").get<ServerCapabilities::Prompts>();
    if (j.contains("resources"))    s.resources = j.at("resources").get<ServerCapabilities::Resources>();
    if (j.contains("tools"))        s.tools = j.at("tools").get<ServerCapabilities::Tools>();
}

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
//
//// ===== JSONRPCMessage =====
//using JSONRPCMessage = std::variant<
//    JSONRPCRequest,
//    JSONRPCNotification,
//    JSONRPCBatchRequest,
//    JSONRPCResponse,
//    JSONRPCErrorResponse,
//    JSONRPCBatchResponse
//>;
//
//inline void to_json(json& j, const JSONRPCMessage& m) {
//    std::visit([&j](auto&& arg){ j = arg; }, m);
//}
//
//inline void from_json(const json& j, JSONRPCMessage& m) {
//    // 判别 JSON-RPC message 类型
//    auto it = j.find("jsonrpc");
//    if (it == j.end()) {
//        // 批量请求或响应
//        if (j.is_array()) {
//            // 需更详细判别，此处默认
//            m = j.get<JSONRPCBatchRequest>();
//        }
//        return;
//    }
//    std::string jsonrpc = j.at("jsonrpc").get<std::string>();
//    if (jsonrpc != "2.0") return;
//    if (j.contains("method") && j.contains("id"))          m = j.get<JSONRPCRequest>();
//    else if (j.contains("method"))                          m = j.get<JSONRPCNotification>();
//    else if (j.contains("result") && j.contains("id"))    m = j.get<JSONRPCResponse>();
//    else if (j.contains("error")  && j.contains("id"))    m = j.get<JSONRPCErrorResponse>();
//}

}