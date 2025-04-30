#pragma once

#include <string>
#include <vector>
#include <optional>
#include <variant>
#include <nlohmann/json.hpp>
#include "McpContent.hpp"  // 包含 PromptReference, ResourceReference 等
#include "McpPromptRequests.hpp"
using json = nlohmann::json;

namespace mcp::protocol {
//// ===== ResourceReference =====
//// 引用一个资源（或资源模板）
//struct ResourceReference {
//    std::string type{"ref/resource"};
//    std::string uri;
//};
//
//inline void to_json(json& j, const ResourceReference& r) {
//    j = json{{"type", r.type}, {"uri", r.uri}};
//}
//inline void from_json(const json& j, ResourceReference& r) {
//    j.at("uri").get_to(r.uri);
//}

// ===== CompletionArgument =====
// 自动补全参数
struct CompletionArgument {
    std::string name;
    std::string value;
};

inline void to_json(json& j, const CompletionArgument& a) {
    j = json{{"name", a.name}, {"value", a.value}};
}
inline void from_json(const json& j, CompletionArgument& a) {
    j.at("name").get_to(a.name);
    j.at("value").get_to(a.value);
}

// ===== CompleteRequest / CompleteResult =====

struct CompleteRequest {
    std::string method = "completion/complete";
    struct Params {
        CompletionArgument argument;
        std::variant<PromptReference, ResourceReference> ref;
    } params;
};

inline void to_json(json& j, const CompleteRequest::Params& p) {
    j = json{{"argument", p.argument}};
    std::visit([&j](auto&& arg){ j["ref"] = arg; }, p.ref);
}
inline void from_json(const json& j, CompleteRequest::Params& p) {
    j.at("argument").get_to(p.argument);
    auto node = j.at("ref");
    // 判断是 PromptReference 还是 ResourceReference
    if (node.contains("type") && node["type"] == "ref/prompt") {
        p.ref = node.get<PromptReference>();
    } else {
        p.ref = node.get<ResourceReference>();
    }
}
inline void to_json(json& j, const CompleteRequest& r) {
    j = json{{"method", r.method}, {"params", r.params}};
}
inline void from_json(const json& j, CompleteRequest& r) {
    j.at("params").get_to(r.params);
}

struct CompleteResult {
    std::optional<json> _meta;
    struct Completion {
        std::vector<std::string> values;
        std::optional<bool> hasMore;
        std::optional<int> total;
    } completion;
};

inline void to_json(json& j, const CompleteResult& r) {
    j = json{};
    if (r._meta) j["_meta"] = *r._meta;
    json comp = json{{"values", r.completion.values}};
    if (r.completion.hasMore) comp["hasMore"] = *r.completion.hasMore;
    if (r.completion.total)   comp["total"]   = *r.completion.total;
    j["completion"] = comp;
}
inline void from_json(const json& j, CompleteResult& r) {
    if (j.contains("_meta")) j.at("_meta").get_to(r._meta);
    auto c = j.at("completion");
    c.at("values").get_to(r.completion.values);
        if (c.contains("hasMore")) {
        r.completion.hasMore = c.at("hasMore").get<bool>(); // 显式处理optional<bool>
    }
    if (c.contains("total")) {
        r.completion.total = c.at("total").get<int>(); // 显式处理optional<int>
    }
//    if (c.contains("hasMore")) c.at("hasMore").get_to(r.completion.hasMore);
//    if (c.contains("total"))   c.at("total").get_to(r.completion.total);
}

}