#pragma once

#include <string>
#include <vector>
#include <optional>
#include <nlohmann/json.hpp>
#include "McpContent.hpp"  // 包含 Annotations

using json = nlohmann::json;

// ===== 根列表 =====
// Root: 根 URI，客户端提供的可操作目录或文件
struct Root {
    std::string uri;
    std::optional<std::string> name;
};

inline void to_json(json& j, const Root& r) {
    j = json{{"uri", r.uri}};
    if (r.name) j["name"] = *r.name;
}
inline void from_json(const json& j, Root& r) {
    j.at("uri").get_to(r.uri);
    if (j.contains("name")) j.at("name").get_to(r.name);
}

// ListRootsRequest / ListRootsResult
struct ListRootsRequest {
    std::string method = "roots/list";
    struct Params { std::optional<json> _meta; } params;
};

inline void to_json(json& j, const ListRootsRequest::Params& p) {
    j = json::object();
    if (p._meta) j["_meta"] = *p._meta;
}
inline void from_json(const json& j, ListRootsRequest::Params& p) {
    if (j.contains("_meta")) j.at("_meta").get_to(p._meta);
}

inline void to_json(json& j, const ListRootsRequest& r) {
    j = json{{"method", r.method}, {"params", r.params}};
}
inline void from_json(const json& j, ListRootsRequest& r) {
    j.at("params").get_to(r.params);
}

struct ListRootsResult {
    std::optional<json> _meta;
    std::vector<Root> roots;
};

inline void to_json(json& j, const ListRootsResult& r) {
    j = json{};
    if (r._meta) j["_meta"] = *r._meta;
    j["roots"] = r.roots;
}
inline void from_json(const json& j, ListRootsResult& r) {
    if (j.contains("_meta")) j.at("_meta").get_to(r._meta);
    j.at("roots").get_to(r.roots);
}

// RootsListChangedNotification
struct RootsListChangedNotification {
    std::string method = "notifications/roots/list_changed";
    std::optional<json> _meta;
};
inline void to_json(json& j, const RootsListChangedNotification& n) {
    j = json{{"method", n.method}};
    json params = json::object();
    if (n._meta) params["_meta"] = *n._meta;
    j["params"] = params;
}
inline void from_json(const json& j, RootsListChangedNotification& n) {
    if (j.contains("params") && j["params"].contains("_meta"))
        j["params"].at("_meta").get_to(n._meta);
}

// ===== 日志等级设置 =====
// SetLevelRequest
struct SetLevelRequest {
    std::string method = "logging/setLevel";
    struct Params { std::string level; } params;
};
inline void to_json(json& j, const SetLevelRequest::Params& p) {
    j = json{{"level", p.level}};
}
inline void from_json(const json& j, SetLevelRequest::Params& p) {
    j.at("level").get_to(p.level);
}
inline void to_json(json& j, const SetLevelRequest& r) {
    j = json{{"method", r.method}, {"params", r.params}};
}
inline void from_json(const json& j, SetLevelRequest& r) {
    j.at("params").get_to(r.params);
}

// LoggingMessageNotification
struct LoggingMessageNotification {
    std::string method = "notifications/message";
    struct Params {
        std::variant<std::string, json> data;
        std::string level;
        std::optional<std::string> logger;
    } params;
};

inline void to_json(json& j, const LoggingMessageNotification::Params& p) {
    j = json{{"level", p.level}};
    // data 为任意 JSON
    j["data"] = p.data;
    if (p.logger) j["logger"] = *p.logger;
}
inline void from_json(const json& j, LoggingMessageNotification::Params& p) {
    j.at("data").get_to(p.data);
    j.at("level").get_to(p.level);
    if (j.contains("logger")) j.at("logger").get_to(p.logger);
}
inline void to_json(json& j, const LoggingMessageNotification& n) {
    j = json{{"method", n.method}, {"params", n.params}};
}
inline void from_json(const json& j, LoggingMessageNotification& n) {
    j.at("params").get_to(n.params);
}

// ===== 进度通知 =====
struct ProgressNotification {
    std::string method = "notifications/progress";
    struct Params {
        std::string progressToken;
        double progress;
        std::optional<double> total;
        std::optional<std::string> message;
    } params;
};

inline void to_json(json& j, const ProgressNotification::Params& p) {
    j = json{{"progressToken", p.progressToken}, {"progress", p.progress}};
    if (p.total)   j["total"]   = *p.total;
    if (p.message) j["message"] = *p.message;
}
inline void from_json(const json& j, ProgressNotification::Params& p) {
    j.at("progressToken").get_to(p.progressToken);
    j.at("progress").get_to(p.progress);
    if (j.contains("total"))   j.at("total").get_to(p.total);
    if (j.contains("message")) j.at("message").get_to(p.message);
}
inline void to_json(json& j, const ProgressNotification& n) {
    j = json{{"method", n.method}, {"params", n.params}};
}
inline void from_json(const json& j, ProgressNotification& n) {
    j.at("params").get_to(n.params);
}

// ===== 取消通知 =====
struct CancelledNotification {
    std::string method = "notifications/cancelled";
    struct Params {
        std::variant<std::string, int> requestId;
        std::optional<std::string> reason;
    } params;
};

inline void to_json(json& j, const CancelledNotification::Params& p) {
    j = json{{"requestId", p.requestId}};
    if (p.reason) j["reason"] = *p.reason;
}
inline void from_json(const json& j, CancelledNotification::Params& p) {
    j.at("requestId").get_to(p.requestId);
    if (j.contains("reason")) j.at("reason").get_to(p.reason);
}
inline void to_json(json& j, const CancelledNotification& n) {
    j = json{{"method", n.method}, {"params", n.params}};
}
inline void from_json(const json& j, CancelledNotification& n) {
    j.at("params").get_to(n.params);
}

// ===== 已初始化通知 =====
struct InitializedNotification {
    std::string method = "notifications/initialized";
    std::optional<json> _meta;
};
inline void to_json(json& j, const InitializedNotification& n) {
    j = json{{"method", n.method}};
    if (n._meta) {
        j["params"]["_meta"] = *n._meta;
    } else {
        j["params"] = json::object();
    }
}
inline void from_json(const json& j, InitializedNotification& n) {
    if (j.contains("params") && j["params"].contains("_meta")) {
        j["params"].at("_meta").get_to(n._meta);
    }
}
