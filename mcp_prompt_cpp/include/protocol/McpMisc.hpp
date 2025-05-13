#pragma once

#include <string>
#include <variant>
#include <optional>
#include <vector>
#include <nlohmann/json.hpp>
#include "McpPromptRequests.hpp"
#include "McpResourceRequests.hpp"
#include "McpToolRequests.hpp"
#include "McpCompletionRequests.hpp"
#include "McpCoreRequests.hpp"
#include "McpRootsAndLoggingRequests.hpp"
#include "McpJsonRpc.hpp"
#include "McpCapabilities.hpp"
#include "McpContent.hpp"
#include "McpSamplingBatch.hpp"




// 第一步：在全局命名空间定义nlohmann的特化
namespace nlohmann {


// 第二步：添加optional的通用处理（如果尚未添加）
template <typename T>
struct adl_serializer<std::optional<T>> {
    static void to_json(json& j, const std::optional<T>& opt) {
        if (opt) j = *opt;
        else j = nullptr;
    }
    static void from_json(const json& j, std::optional<T>& opt) {
        if (j.is_null()) opt = std::nullopt;
        else opt = j.get<T>();
    }
};
} // namespace nlohmann



// namespace nlohmann {
//     // 添加对 RequestId 的序列化支持
//     template <>
//     struct adl_serializer<mcp::protocol::RequestId> {
//         static void to_json(json& j, const mcp::protocol::RequestId& id) {
//             mcp::protocol::idToJson(id).swap(j); // 复用已有的 idToJson 逻辑
//         }
//
//         static void from_json(const json& j, mcp::protocol::RequestId& id) {
//             id = mcp::protocol::parseId(j); // 复用已有的 parseId 逻辑
//         }
//     };
// } // namespace nlohmann

using json = nlohmann::json;

// 为std::vector<ModelHint>添加序列化支持（取消注释并修复原有代码）




namespace mcp::protocol {
//#include <nlohmann/json.hpp>
//using json = nlohmann::json;
// RequestId and ProgressToken
// using RequestId = std::variant<std::string, int>;
using ProgressToken = RequestId;

    // ===== Request 结构改进 =====
    struct Request {
        struct Params {
            struct Meta {
                std::optional<RequestId> progressToken; // RequestId = variant<string, int>
            };
            std::optional<Meta> _meta;
            json otherParams; // 其他未明确定义的参数
        };

        std::string method;
        std::optional<Params> params;
    };

    // 序列化/反序列化
    inline void to_json(json& j, const Request::Params::Meta& m) {
        if (m.progressToken) j["progressToken"] = *m.progressToken;
    }

    inline void from_json(const json& j, Request::Params::Meta& m) {
        if (j.contains("progressToken")) m.progressToken = j.at("progressToken").get<RequestId>();
    }

    inline void to_json(json& j, const Request::Params& p) {
        if (p._meta) j["_meta"] = *p._meta;
        j.update(p.otherParams); // 合并其他参数
    }

    inline void from_json(const json& j, Request::Params& p) {
        if (j.contains("_meta")) j.at("_meta").get_to(p._meta);
        p.otherParams = j; // 保留原始数据
    }

    // ===== Result 结构改进 =====
    struct Result {
        struct Meta {
            json additionalMetadata; // 允许任意元数据
        };
        std::optional<Meta> _meta;
        json otherProperties; // 其他扩展属性
    };

    inline void to_json(json& j, const Result::Meta& m) {
        j = m.additionalMetadata;
    }

    inline void from_json(const json& j, Result::Meta& m) {
        m.additionalMetadata = j;
    }

    inline void to_json(json& j, const Result& r) {
        if (r._meta) j["_meta"] = *r._meta;
        j.update(r.otherProperties); // 合并其他属性
    }

    inline void from_json(const json& j, Result& r) {
        if (j.contains("_meta")) j.at("_meta").get_to(r._meta);
        r.otherProperties = j; // 保留原始数据
    }


// PaginatedResult
struct PaginatedResult {
    std::optional<json> _meta;
    std::optional<std::string> nextCursor;
};
inline void to_json(json& j, const PaginatedResult& p) {
    j = json{};
    if (p._meta)      j["_meta"]      = *p._meta;
    if (p.nextCursor) j["nextCursor"] = *p.nextCursor;
}
inline void from_json(const json& j, PaginatedResult& p) {
    if (j.contains("_meta"))      j.at("_meta").get_to(p._meta);
    if (j.contains("nextCursor")) j.at("nextCursor").get_to(p.nextCursor);
}

// Notification (generic)
struct Notification {
    std::string method;
    std::optional<json> params;
};
inline void to_json(json& j, const Notification& n) {
    j = json{{"method", n.method}};
    if (n.params) j["params"] = *n.params;
}
inline void from_json(const json& j, Notification& n) {
    j.at("method").get_to(n.method);
    if (j.contains("params")) n.params = j.at("params");
}

// ModelHint
struct ModelHint {
    std::string name;
};
inline void to_json(json& j, const ModelHint& m) { j = json{{"name", m.name}}; }
inline void from_json(const json& j, ModelHint& m) { j.at("name").get_to(m.name); }


// ModelPreferences
struct ModelPreferences {
    std::optional<double> costPriority;
    std::optional<double> speedPriority;
    std::optional<double> intelligencePriority;
    std::optional<std::vector<ModelHint>> hints;
};



inline void to_json(json& j, const ModelPreferences& m) {
    j = json{};
    if (m.costPriority)        j["costPriority"]        = *m.costPriority;
    if (m.speedPriority)       j["speedPriority"]       = *m.speedPriority;
    if (m.intelligencePriority)j["intelligencePriority"]=*m.intelligencePriority;
//    if (m.hints)               j["hints"]               = *m.hints;
}
inline void from_json(const json& j, ModelPreferences& m) {
    if (j.contains("costPriority") && !j["costPriority"].is_null()) {
//        j.at("costPriority").get_to(m.costPriority);
        m.costPriority = j.at("costPriority").get<double>();
    }
    if (j.contains("speedPriority") && !j["speedPriority"].is_null()) {
//        j.at("speedPriority").get_to(m.speedPriority);
		m.speedPriority = j.at("speedPriority").get<double>();
    }
    if (j.contains("intelligencePriority") && !j["intelligencePriority"].is_null()) {
//        j.at("intelligencePriority").get_to(m.intelligencePriority);
		m.intelligencePriority = j.at("intelligencePriority").get<double>();
    }
    if (j.contains("hints") && !j["hints"].is_null()) {
//        j.at("hints").get_to(m.hints);
		m.hints = j.at("hints").get<std::vector<ModelHint>>();
    }
}



// ServerRequest
using ServerRequest = std::variant<PingRequest, CreateMessageRequest, ListRootsRequest>;
inline void from_json(const json& j, ServerRequest& sr) {
    const std::string& method = j.at("method").get<std::string>();
    if (method == "ping")           sr = j.get<PingRequest>();
    else if (method == "sampling/createMessage") sr = j.get<CreateMessageRequest>();
    else if (method == "roots/list")      sr = j.get<ListRootsRequest>();
}
inline void to_json(json& j, const ServerRequest& sr) {
    std::visit([&](auto&& arg){ j = arg; }, sr);
}

// ServerResult
using ServerResult = std::variant<
    Result,
    InitializeResult,
    ListResourcesResult,
    ReadResourceResult,
    ListPromptsResult,
    GetPromptResult,
    ListToolsResult,
    CallToolResult,
    CompleteResult
>;
inline void to_json(json& j, const ServerResult& sr) {
    std::visit([&j](auto&& arg){ j = json(arg); }, sr);
}
inline void from_json(const json& j, ServerResult& sr) {
    // 判别可根据 keys
    if (j.contains("capabilities") && j.contains("protocolVersion")) sr = j.get<InitializeResult>();
    else if (j.contains("resources")) sr = j.get<ListResourcesResult>();
    else if (j.contains("contents"))  sr = j.get<ReadResourceResult>();
    else if (j.contains("prompts"))   sr = j.get<ListPromptsResult>();
    else if (j.contains("messages"))  sr = j.get<GetPromptResult>();
    else if (j.contains("tools"))     sr = j.get<ListToolsResult>();
    else if (j.contains("content"))   sr = j.get<CallToolResult>();
    else if (j.contains("completion"))sr = j.get<CompleteResult>();
}

}


