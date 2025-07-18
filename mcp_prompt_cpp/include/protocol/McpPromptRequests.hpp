#pragma once

#include <string>
#include <map>
#include <optional>
#include <vector>
#include <nlohmann/json.hpp>
#include "McpContent.hpp"  // 提前定义 Prompt, PromptMessage 等

using json = nlohmann::json;

namespace mcp::protocol {
// ===== GetPromptRequest / GetPromptResult =====

struct GetPromptRequest {
    std::string method = "prompts/get";
    struct Params {
        std::string name;
        std::optional<std::map<std::string, std::string>> arguments;
    } params;
};

inline void to_json(json& j, const GetPromptRequest::Params& p) {
    j = json{{"name", p.name}};
    if (p.arguments) j["arguments"] = *p.arguments;
}

inline void from_json(const json& j, GetPromptRequest::Params& p) {
    j.at("name").get_to(p.name);
    if (j.contains("arguments")) j.at("arguments").get_to(p.arguments.emplace());
}

inline void to_json(json& j, const GetPromptRequest& r) {
    j = json{{"method", r.method}, {"params", r.params}};
}

inline void from_json(const json& j, GetPromptRequest& r) {
    // method 为固定值，可选校验
    j.at("params").get_to(r.params);
}

struct GetPromptResult {
    std::optional<json> _meta;
    std::optional<std::string> description;
    std::vector<PromptMessage> messages;
};

inline void to_json(json& j, const GetPromptResult& r) {
    j = json{};
    if (r._meta)      j["_meta"] = *r._meta;
    if (r.description) j["description"] = *r.description;
    j["messages"] = r.messages;
}

inline void from_json(const json& j, GetPromptResult& r) {
    if (j.contains("_meta"))      j.at("_meta").get_to(r._meta);
    if (j.contains("description")) j.at("description").get_to(r.description);
    j.at("messages").get_to(r.messages);
}


// ===== ListPromptsRequest / ListPromptsResult =====

struct ListPromptsRequest {
    std::string method = "prompts/list";
    struct Params {
        std::optional<std::string> cursor;
    } params;
};

inline void to_json(json& j, const ListPromptsRequest::Params& p) {
    j = json{};
    if (p.cursor) j["cursor"] = *p.cursor;
}

inline void from_json(const json& j, ListPromptsRequest::Params& p) {
    if (j.contains("cursor")) j.at("cursor").get_to(p.cursor);
}

inline void to_json(json& j, const ListPromptsRequest& r) {
    j = json{{"method", r.method}, {"params", r.params}};
}

inline void from_json(const json& j, ListPromptsRequest& r) {
    j.at("params").get_to(r.params);
}

struct ListPromptsResult {
    std::optional<json> _meta;
    std::optional<std::string> nextCursor;
    std::vector<Prompt> prompts;
};

inline void to_json(json& j, const ListPromptsResult& r) {
    j = {{"prompts", r.prompts}}; // 必须包含 prompts
    if (r._meta)      j["_meta"]      = *r._meta;
    if (r.nextCursor) j["nextCursor"] = *r.nextCursor;

//        j = {{"prompts", r.prompts}}; // prompts 是必须字段
//    if (r._meta.has_value()) {
//        j["_meta"] = r._meta.value();
//    }
//    if (r.nextCursor.has_value()) {
//        j["nextCursor"] = r.nextCursor.value();
//    }
  
}

inline void from_json(const json& j, ListPromptsResult& r) {
    if (j.contains("_meta"))      j.at("_meta").get_to(r._meta);
    if (j.contains("nextCursor")) j.at("nextCursor").get_to(r.nextCursor);
    j.at("prompts").get_to(r.prompts);
}


// ===== PromptListChangedNotification =====
    struct PromptListChangedNotification {
    std::string method = "notifications/prompts/list_changed";
    struct Params {
        std::optional<json> _meta;
        json additionalProperties; // 存储其他属性
    };
    std::optional<Params> params; // 保持可选性
};

    // Params 序列化
    inline void to_json(json& j, const PromptListChangedNotification::Params& p) {
        j = p.additionalProperties;
        if (p._meta) j["_meta"] = *p._meta;
    }

    inline void from_json(const json& j, PromptListChangedNotification::Params& p) {
        p.additionalProperties = j;
        if (j.contains("_meta")) {
            p._meta = j["_meta"];
            p.additionalProperties.erase("_meta");
        }
    }

    // Notification 序列化
    inline void to_json(json& j, const PromptListChangedNotification& n) {
        j = {{"method", n.method}};
        if (n.params) j["params"] = *n.params;
    }

    inline void from_json(const json& j, PromptListChangedNotification& n) {
        j.at("method").get_to(n.method);
        if (j.contains("params")) {
            j.at("params").get_to(n.params.emplace());
        }
    }
// struct PromptListChangedNotification {
//     std::string method = "notifications/prompts/list_changed";
//     struct Params {
//         std::optional<json> _meta;
//     } params;
// };
//
//     inline void to_json(json& j, const PromptListChangedNotification::Params& p) {
//         j = json::object();
//         if (p._meta) j["_meta"] = *p._meta;
//     }
//
//     inline void from_json(const json& j, PromptListChangedNotification::Params& p) {
//         if (j.contains("_meta")) j.at("_meta").get_to(p._meta);
//     }
//
//     inline void to_json(json& j, const PromptListChangedNotification& n) {
//         j = json{
//             {"method", n.method},
//             {"params", n.params}  // 自动调用Params的序列化
//         };
//     }
//
//     inline void from_json(const json& j, PromptListChangedNotification& n) {
//         j.at("method").get_to(n.method);
//         j.at("params").get_to(n.params);
//     }

// {
//     "method": "notifications/prompts/list_changed",
//     "params": {
//         "_meta": {
//             "timestamp": "2023-07-01T12:00:00Z"
//           }
//     }
// }


// ===== PromptReference =====
// 引用一个已注册的提示模板
struct PromptReference {
    std::string name;
    std::string type{"ref/prompt"};
};

inline void to_json(json& j, const PromptReference& r) {
    j = json{{"name", r.name}, {"type", r.type}};
}
inline void from_json(const json& j, PromptReference& r) {
    j.at("name").get_to(r.name);
    // type 固定，无需读入
}


}
// Completion Autocomplete（CompleteRequest/CompleteResult）或其他功能
