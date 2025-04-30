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



//namespace nlohmann {
//template<typename T>
//struct adl_serializer<std::optional<T>> {
//    static void to_json(json& j, const std::optional<T>& opt) {
//        if (opt) j = *opt;
//        else j = nullptr;
//    }
//    static void from_json(const json& j, std::optional<T>& opt) {
//        if (j.is_null()) opt = std::nullopt;
//        else opt = j.get<T>();
//    }
//};
//}

// 第一步：在全局命名空间定义nlohmann的特化
namespace nlohmann {
// 先为vector<ModelHint>添加序列化支持
//template <>
//struct adl_serializer<std::vector<mcp::protocol::ModelHint>> {
//    static void to_json(json& j, const std::vector<mcp::protocol::ModelHint>& v) {
//        j = json::array();
//        for (const auto& item : v) {
//            j.push_back(item); // 这里会调用mcp::protocol命名空间中的to_json
//        }
//    }
//    static void from_json(const json& j, std::vector<mcp::protocol::ModelHint>& v) {
//        for (const auto& item : j) {
//            v.push_back(item.get<mcp::protocol::ModelHint>());
//        }
//    }
//};

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





using json = nlohmann::json;

// 为std::vector<ModelHint>添加序列化支持（取消注释并修复原有代码）




namespace mcp::protocol {
//#include <nlohmann/json.hpp>
//using json = nlohmann::json;
// RequestId and ProgressToken
using RequestId = std::variant<std::string, int>;
using ProgressToken = RequestId;

// Generic MCP Request
struct Request {
    std::string method;
    std::optional<json> params;
};
inline void to_json(json& j, const Request& r) {
    j = json{{"method", r.method}};
    if (r.params) j["params"] = *r.params;
}
inline void from_json(const json& j, Request& r) {
    j.at("method").get_to(r.method);
    if (j.contains("params")) r.params = j.at("params");
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

//// 在nlohmann命名空间内添加序列化特化
//namespace nlohmann {
//// 为std::vector<ModelHint>添加序列化支持
//template <>
//struct ::nlohmann::adl_serializer<std::vector<mcp::protocol::ModelHint>> {
//    static void to_json(json& j, const std::vector<mcp::protocol::ModelHint>& v) {
//        j = json::array();
//        for (const auto& item : v) {
//            j.push_back(item);
//        }
//    }
//    static void from_json(const json& j, std::vector<mcp::protocol::ModelHint>& v) {
//        for (const auto& item : j) {
//            v.push_back(item.get<mcp::protocol::ModelHint>());
//        }
//    }
//};
//
//// 为optional类型添加通用序列化支持（如果需要）
//template <typename T>
//struct ::nlohmann::adl_serializer<std::optional<T>> {
//    static void to_json(json& j, const std::optional<T>& opt) {
//        if (opt.has_value()) j = *opt;
//        else j = nullptr;
//    }
//    static void from_json(const json& j, std::optional<T>& opt) {
//        if (j.is_null()) opt = std::nullopt;
//        else opt = j.get<T>();
//    }
//};
//} // namespace nlohmann


//namespace nlohmann {
//template <>
//struct adl_serializer<std::vector<mcp::protocol::ModelHint>> {
//    static void to_json(json& j, const std::vector<mcp::protocol::ModelHint>& v) {
//        j = json::array();
//        for (const auto& item : v) {
//            j.push_back(item); // 这里会调用ModelHint的to_json
//        }
//    }
//    static void from_json(const json& j, std::vector<mcp::protocol::ModelHint>& v) {
//        for (const auto& item : j) {
//            v.push_back(item.get<mcp::protocol::ModelHint>());
//        }
//    }
//};
//} // namespace nlohmann

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

// Result (generic)
struct Result {
    std::optional<json> _meta;
};
inline void to_json(json& j, const Result& r) {
   json temp = json::object();
    if (r._meta) temp = json{{"_meta", *r._meta}};
    else         temp = json::object();
    j.swap(temp);
}
inline void from_json(const json& j, Result& r) {
    if (j.contains("_meta")) j.at("_meta").get_to(r._meta);
}

//
//// ServerCapabilities (extension)
//struct ServerCapabilities {
//    std::optional<json> completions;
//    std::optional<json> experimental;
//    std::optional<json> logging;
//    struct Prompts { std::optional<bool> listChanged;
//   		 // 添加完整的序列化方法（替代宏）
//        friend void to_json(json& j, const Prompts& p) {
//            j = json{};
//            if (p.listChanged) j["listChanged"] = *p.listChanged;
//        }
//        friend void from_json(const json& j, Prompts& p) {
//            if (j.contains("listChanged")) {
//                p.listChanged = j.at("listChanged").get<bool>();
//            }
//        }
//    };
//    std::optional<Prompts> prompts;
//    struct Resources { std::optional<bool> listChanged; std::optional<bool> subscribe;
//      	 // 添加完整的序列化方法
//        friend void to_json(json& j, const Resources& r) {
//            j = json{};
//            if (r.listChanged) j["listChanged"] = *r.listChanged;
//            if (r.subscribe)   j["subscribe"]   = *r.subscribe;
//        }
//        friend void from_json(const json& j, Resources& r) {
//            if (j.contains("listChanged")) r.listChanged = j.at("listChanged").get<bool>();
//            if (j.contains("subscribe"))   r.subscribe   = j.at("subscribe").get<bool>();
//        }
//
//    };
//    std::optional<Resources> resources;
//    struct Tools { std::optional<bool> listChanged;
// // 添加完整的序列化方法
//        friend void to_json(json& j, const Tools& t) {
//            j = json{};
//            if (t.listChanged) j["listChanged"] = *t.listChanged;
//        }
//        friend void from_json(const json& j, Tools& t) {
//            if (j.contains("listChanged")) {
//                t.listChanged = j.at("listChanged").get<bool>();
//            }
//        }
//    };
//    std::optional<Tools> tools;
//};
//inline void to_json(json& j, const ServerCapabilities& s) {
//    j = json{};
//    if (s.completions)   j["completions"]   = *s.completions;
//    if (s.experimental)  j["experimental"]  = *s.experimental;
//    if (s.logging)       j["logging"]       = *s.logging;
//    if (s.prompts) {
//        j["prompts"]["listChanged"] = *s.prompts->listChanged;
//    }
//    if (s.resources) {
//        auto& r = *s.resources;
//        j["resources"]["listChanged"] = *r.listChanged;
//        j["resources"]["subscribe"]   = *r.subscribe;
//    }
//    if (s.tools) j["tools"]["listChanged"] = *s.tools->listChanged;
//}
//inline void from_json(const json& j, ServerCapabilities& s) {
//    if (j.contains("completions"))   j.at("completions").get_to(s.completions);
//    if (j.contains("experimental"))  j.at("experimental").get_to(s.experimental);
//    if (j.contains("logging"))       j.at("logging").get_to(s.logging);
//    if (j.contains("prompts"))      s.prompts = j.at("prompts").get<ServerCapabilities::Prompts>();
//    if (j.contains("resources"))    s.resources = j.at("resources").get<ServerCapabilities::Resources>();
//    if (j.contains("tools"))        s.tools = j.at("tools").get<ServerCapabilities::Tools>();
//}

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


