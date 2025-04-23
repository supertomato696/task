#pragma once

#include <string>
#include <vector>
#include <optional>
#include <map>
#include <variant>
#include <nlohmann/json.hpp>
#include "McpRootsAndLoggingRequests.hpp"  // CancelledNotification, InitializedNotification, ProgressNotification, RootsListChangedNotification
#include "McpJsonRpc.hpp"               // JSONRPC* types

using json = nlohmann::json;

// ===== LoggingLevel =====
enum class LoggingLevel {
    alert,
    critical,
    debug,
    emergency,
    error,
    info,
    notice,
    warning
};

NLOHMANN_JSON_SERIALIZE_ENUM(LoggingLevel, {
    {LoggingLevel::alert, "alert"},
    {LoggingLevel::critical, "critical"},
    {LoggingLevel::debug, "debug"},
    {LoggingLevel::emergency, "emergency"},
    {LoggingLevel::error, "error"},
    {LoggingLevel::info, "info"},
    {LoggingLevel::notice, "notice"},
    {LoggingLevel::warning, "warning"}
})

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
    if (j.contains("experimental")) j.at("experimental").get_to(c.experimental);
    if (j.contains("roots")) {
        c.roots = Roots{};
        const auto& r = j["roots"];
        if (r.contains("listChanged")) r.at("listChanged").get_to(c.roots->listChanged);
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
