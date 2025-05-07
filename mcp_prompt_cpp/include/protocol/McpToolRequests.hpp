#pragma once

#include <string>
#include <vector>
#include <optional>
#include <nlohmann/json.hpp>
#include "McpContent.hpp"  // 提前定义 TextContent, ImageContent, AudioContent, EmbeddedResource

namespace mcp::protocol {
using json = nlohmann::json;

// ===== ToolAnnotations =====
// 提供给客户端的工具行为提示
struct ToolAnnotations {
    std::optional<bool> destructiveHint;
    std::optional<bool> idempotentHint;
    std::optional<bool> openWorldHint;
    std::optional<bool> readOnlyHint;
    std::optional<std::string> title;
};

inline void to_json(json& j, const ToolAnnotations& t) {
    j = json{};
    if (t.destructiveHint) j["destructiveHint"] = *t.destructiveHint;
    if (t.idempotentHint)  j["idempotentHint"]  = *t.idempotentHint;
    if (t.openWorldHint)   j["openWorldHint"]   = *t.openWorldHint;
    if (t.readOnlyHint)    j["readOnlyHint"]    = *t.readOnlyHint;
    if (t.title)           j["title"]           = *t.title;
}

inline void from_json(const json& j, ToolAnnotations& t) {
    if (j.contains("destructiveHint") && !j["destructiveHint"].is_null()) {
        t.destructiveHint = j["destructiveHint"].get<bool>();  // 直接从 JSON 中提取 bool 值
    }
    if (j.contains("idempotentHint") && !j["idempotentHint"].is_null()) {
        t.idempotentHint = j["idempotentHint"].get<bool>();  // 同上
    }
    if (j.contains("openWorldHint") && !j["openWorldHint"].is_null()) {
        t.openWorldHint = j["openWorldHint"].get<bool>();  // 同上
    }
    if (j.contains("readOnlyHint") && !j["readOnlyHint"].is_null()) {
        t.readOnlyHint = j["readOnlyHint"].get<bool>();  // 同上
    }
    if (j.contains("title") && !j["title"].is_null()) {
        t.title = j["title"].get<std::string>();  // 处理 title 字段
    }
}

// ===== Tool =====
// 客户端可调用的工具定义
struct Tool {
    std::string name;
    json inputSchema;             // JSON Schema 描述参数
    std::optional<std::string> description;
    std::optional<ToolAnnotations> annotations;
};

inline void to_json(json& j, const Tool& t) {
    j = json{{"name", t.name}, {"inputSchema", t.inputSchema}};
    if (t.description)   j["description"]   = *t.description;
    if (t.annotations)   j["annotations"]   = *t.annotations;
}

inline void from_json(const json& j, Tool& t) {
    j.at("name").get_to(t.name);
    j.at("inputSchema").get_to(t.inputSchema);
   // 手动处理 description 字段，检查是否存在且不为 null
    if (j.contains("description") && !j["description"].is_null()) {
        t.description = j["description"].get<std::string>();
    }

    // 手动处理 annotations 字段，检查是否存在且不为 null
    if (j.contains("annotations") && !j["annotations"].is_null()) {
        t.annotations = j["annotations"].get<ToolAnnotations>();  // 从 JSON 提取 ToolAnnotations
    }
}

// ===== ListToolsRequest / ListToolsResult =====
struct ListToolsRequest {
    std::string method = "tools/list";
    struct Params { std::optional<std::string> cursor; } params;
};

inline void to_json(json& j, const ListToolsRequest::Params& p) {
    j = json{};
    if (p.cursor) j["cursor"] = *p.cursor;
}
inline void from_json(const json& j, ListToolsRequest::Params& p) {
    if (j.contains("cursor")) j.at("cursor").get_to(p.cursor);
}
inline void to_json(json& j, const ListToolsRequest& r) {
    j = json{{"method", r.method}, {"params", r.params}};
}
inline void from_json(const json& j, ListToolsRequest& r) {
    j.at("params").get_to(r.params);
}

struct ListToolsResult {
    std::optional<json> _meta;
    std::optional<std::string> nextCursor;
    std::vector<Tool> tools;
};

inline void to_json(json& j, const ListToolsResult& r) {
    j = json{};
    if (r._meta)       j["_meta"]      = *r._meta;
    if (r.nextCursor)  j["nextCursor"] = *r.nextCursor;
    j["tools"]        = r.tools;
}
inline void from_json(const json& j, ListToolsResult& r) {
    if (j.contains("_meta"))      j.at("_meta").get_to(r._meta);
    if (j.contains("nextCursor")) j.at("nextCursor").get_to(r.nextCursor);
    j.at("tools").get_to(r.tools);
}

// ===== CallToolRequest / CallToolResult =====
struct CallToolRequest {
    std::string method = "tools/call";
    struct Params {
        std::string name;
        json arguments;
    } params;
};

inline void to_json(json& j, const CallToolRequest::Params& p) {
    j = json{{"name", p.name}, {"arguments", p.arguments}};
}
inline void from_json(const json& j, CallToolRequest::Params& p) {
    j.at("name").get_to(p.name);
    j.at("arguments").get_to(p.arguments);
}
inline void to_json(json& j, const CallToolRequest& r) {
    j = json{{"method", r.method}, {"params", r.params}};
}
inline void from_json(const json& j, CallToolRequest& r) {
    j.at("params").get_to(r.params);
}

struct CallToolResult {
    std::optional<json> _meta;
    std::vector<std::variant<TextContent, ImageContent, AudioContent, EmbeddedResource>> content;
    std::optional<bool> isError;
};

inline void to_json(json& j, const CallToolResult& r) {
    if (r._meta) j["_meta"] = *r._meta;
    j["content"] = json::array();
    for (auto& item : r.content) {
        json elem;
        std::visit([&](auto&& arg){ elem = arg; }, item);
        j["content"].push_back(elem);
    }
    if (r.isError) j["isError"] = *r.isError;
}
inline void from_json(const json& j, CallToolResult& r) {
    if (j.contains("_meta")) j.at("_meta").get_to(r._meta);
    for (auto& elem : j.at("content")) {
        auto type = elem.at("type").get<std::string>();
        if (type == "text")     r.content.emplace_back(elem.get<TextContent>());
        else if (type == "image")   r.content.emplace_back(elem.get<ImageContent>());
        else if (type == "audio")   r.content.emplace_back(elem.get<AudioContent>());
        else if (type == "resource")r.content.emplace_back(elem.get<EmbeddedResource>());
    }
    if (j.contains("isError") && !j["isError"].is_null()) {
        r.isError = j["isError"].get<bool>();
	}
 }

// ===== ToolListChangedNotification =====
struct ToolListChangedNotification {
    std::string method = "notifications/tools/list_changed";
    struct Params {
        std::optional<json> _meta;
    } params;
};
    inline void to_json(json& j, const ToolListChangedNotification::Params& p) {
        j = json::object();
        if (p._meta) j["_meta"] = *p._meta;
    }

    inline void from_json(const json& j, ToolListChangedNotification::Params& p) {
        if (j.contains("_meta")) j.at("_meta").get_to(p._meta);
    }

    inline void to_json(json& j, const ToolListChangedNotification& n) {
        j = json{
            {"method", n.method},
            {"params", n.params}  // 自动调用Params的序列化
        };
    }

    inline void from_json(const json& j, ToolListChangedNotification& n) {
        j.at("method").get_to(n.method);
        j.at("params").get_to(n.params);
    }

// {
//     "method": "notifications/tools/list_changed",
//     "params": {
//         "_meta": {"traceId": "xyz789"}  // 符合Schema层级要求
//     }
// }

    // ToolListChangedNotification notif;
    // notif.params._meta = json{{"timestamp", "2023-07-01T12:00:00Z"}};
    //
    // // 序列化结果：
    // {
    // "method": "notifications/tools/list_changed",
    // "params": {
    //     "_meta": {
    //         "timestamp": "2023-07-01T12:00:00Z"
    //       }
    // }
    // }



}