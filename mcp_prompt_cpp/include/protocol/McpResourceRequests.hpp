#pragma once

#include <string>
#include <vector>
#include <optional>
#include <nlohmann/json.hpp>
#include "McpContent.hpp"  // 提前定义 Resource, ResourceContents 等
//#include "McpPromptRequests.hpp"
using json = nlohmann::json;

namespace mcp::protocol {
// ===== ListResourcesRequest / ListResourcesResult =====

struct ListResourcesRequest {
    std::string method = "resources/list";
    struct Params {
        std::optional<std::string> cursor;
    } params;
};

inline void to_json(json& j, const ListResourcesRequest::Params& p) {
    j = json{};
    if (p.cursor) j["cursor"] = *p.cursor;
}

inline void from_json(const json& j, ListResourcesRequest::Params& p) {
    if (j.contains("cursor")) j.at("cursor").get_to(p.cursor);
}

inline void to_json(json& j, const ListResourcesRequest& r) {
    j = json{{"method", r.method}, {"params", r.params}};
}

inline void from_json(const json& j, ListResourcesRequest& r) {
    j.at("params").get_to(r.params);
}

struct ListResourcesResult {
    std::optional<json> _meta;
    std::optional<std::string> nextCursor;
    std::vector<Resource> resources;
};

inline void to_json(json& j, const ListResourcesResult& r) {
    j = json{};
    if (r._meta)      j["_meta"]      = *r._meta;
    if (r.nextCursor) j["nextCursor"] = *r.nextCursor;
    j["resources"]  = r.resources;
}

inline void from_json(const json& j, ListResourcesResult& r) {
    if (j.contains("_meta"))      j.at("_meta").get_to(r._meta);
    if (j.contains("nextCursor")) j.at("nextCursor").get_to(r.nextCursor);
    j.at("resources").get_to(r.resources);
}


// ===== ReadResourceRequest / ReadResourceResult =====

struct ReadResourceRequest {
    std::string method = "resources/read";
    struct Params {
        std::string uri;
    } params;
};

inline void to_json(json& j, const ReadResourceRequest::Params& p) {
    j = json{{"uri", p.uri}};
}

inline void from_json(const json& j, ReadResourceRequest::Params& p) {
    j.at("uri").get_to(p.uri);
}

inline void to_json(json& j, const ReadResourceRequest& r) {
    j = json{{"method", r.method}, {"params", r.params}};
}

inline void from_json(const json& j, ReadResourceRequest& r) {
    j.at("params").get_to(r.params);
}

    // ===== ReadResourceResult 修正实现 =====
    struct ReadResourceResult {
    std::optional<json> _meta;
    std::vector<std::variant<TextResourceContents, BlobResourceContents>> contents;
};

    inline void to_json(json& j, const ReadResourceResult& r) {
        j = json{};
        if (r._meta) j["_meta"] = *r._meta;

        // 手动序列化 variant 数组
        j["contents"] = json::array();
        for (const auto& content : r.contents) {
            std::visit([&j](auto&& arg) {
                j.back().push_back(arg); // 利用各类型的 to_json 实现
                // j["content"].push_back(arg);
            }, content);
        }
    }

    inline void from_json(const json& j, ReadResourceResult& r) {
        if (j.contains("_meta")) j.at("_meta").get_to(r._meta);

        // 手动反序列化 variant 数组
        for (const auto& item : j.at("contents")) {
            if (item.contains("text")) {
                r.contents.emplace_back(item.get<TextResourceContents>());
            } else if (item.contains("blob")) {
                r.contents.emplace_back(item.get<BlobResourceContents>());
            } else {
                // throw json::parse_error::create(501, "Invalid resource content type", &item);
            }
        }
    }


// struct ReadResourceResult {
//     std::optional<json> _meta;
//     std::vector<ResourceContents> contents;
// };
//
// inline void to_json(json& j, const ReadResourceResult& r) {
//     j = json{};
//     if (r._meta)     j["_meta"]    = *r._meta;
//     j["contents"]   = r.contents;
// }
//
// inline void from_json(const json& j, ReadResourceResult& r) {
//     if (j.contains("_meta")) j.at("_meta").get_to(r._meta);
//     j.at("contents").get_to(r.contents);
// }





// ===== ListResourceTemplatesRequest / ListResourceTemplatesResult =====

struct ListResourceTemplatesRequest {
    std::string method = "resources/templates/list";
    struct Params {
        std::optional<std::string> cursor;
    } params;
};

inline void to_json(json& j, const ListResourceTemplatesRequest::Params& p) {
    j = json{};
    if (p.cursor) j["cursor"] = *p.cursor;
}
inline void from_json(const json& j, ListResourceTemplatesRequest::Params& p) {
    if (j.contains("cursor")) j.at("cursor").get_to(p.cursor);
}
inline void to_json(json& j, const ListResourceTemplatesRequest& r) {
    j = json{{"method", r.method}, {"params", r.params}};
}
inline void from_json(const json& j, ListResourceTemplatesRequest& r) {
    j.at("params").get_to(r.params);
}

struct ListResourceTemplatesResult {
    std::optional<json> _meta;
    std::optional<std::string> nextCursor;
    std::vector<ResourceTemplate> resourceTemplates;
};

inline void to_json(json& j, const ListResourceTemplatesResult& r) {
    j = json{};
    if (r._meta)        j["_meta"]            = *r._meta;
    if (r.nextCursor)   j["nextCursor"]      = *r.nextCursor;
    j["resourceTemplates"] = r.resourceTemplates;
}
inline void from_json(const json& j, ListResourceTemplatesResult& r) {
    if (j.contains("_meta"))        j.at("_meta").get_to(r._meta);
    if (j.contains("nextCursor"))   j.at("nextCursor").get_to(r.nextCursor);
    j.at("resourceTemplates").get_to(r.resourceTemplates);
}


// ===== SubscribeRequest / UnsubscribeRequest =====

struct SubscribeRequest {
    std::string method = "resources/subscribe";
    struct Params { std::string uri; } params;
};
inline void to_json(json& j, const SubscribeRequest::Params& p) { j = json{{"uri", p.uri}}; }
inline void from_json(const json& j, SubscribeRequest::Params& p) { j.at("uri").get_to(p.uri); }
inline void to_json(json& j, const SubscribeRequest& r) { j = json{{"method", r.method}, {"params", r.params}}; }
inline void from_json(const json& j, SubscribeRequest& r) { j.at("params").get_to(r.params); }

struct UnsubscribeRequest {
    std::string method = "resources/unsubscribe";
    struct Params { std::string uri; } params;
};
inline void to_json(json& j, const UnsubscribeRequest::Params& p) { j = json{{"uri", p.uri}}; }
inline void from_json(const json& j, UnsubscribeRequest::Params& p) { j.at("uri").get_to(p.uri); }
inline void to_json(json& j, const UnsubscribeRequest& r) { j = json{{"method", r.method}, {"params", r.params}}; }
inline void from_json(const json& j, UnsubscribeRequest& r) { j.at("params").get_to(r.params); }


// ===== Notifications =====

// ResourceListChangedNotification
struct ResourceListChangedNotification {
    std::string method = "notifications/resources/list_changed";
    std::optional<json> _meta;
};
inline void to_json(json& j, const ResourceListChangedNotification& n) {
    j = json{{"method", n.method}};
    json p = json{};
    if (n._meta) p["_meta"] = *n._meta;
    j["params"] = p;
}
inline void from_json(const json& j, ResourceListChangedNotification& n) {
    if (j.contains("params") && j["params"].contains("_meta"))
        j["params"].at("_meta").get_to(n._meta);
}

// ResourceUpdatedNotification
struct ResourceUpdatedNotification {
    std::string method = "notifications/resources/updated";
    struct Params { std::string uri; } params;
};

inline void to_json(json& j, const ResourceUpdatedNotification& n) {
    j = json{{"method", n.method}, {"params", { {"uri", n.params.uri} }}};
}
inline void from_json(const json& j, ResourceUpdatedNotification& n) {
    j.at("params").at("uri").get_to(n.params.uri);
}

}
