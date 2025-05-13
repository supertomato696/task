#pragma once

#include <string>
#include <vector>
#include <variant>
#include <optional>
#include <nlohmann/json.hpp>

// 在nlohmann命名空间内添加variant的序列化支持
namespace nlohmann {
template <>
struct adl_serializer<std::variant<std::string, int>> {
    static void to_json(json& j, const std::variant<std::string, int>& v) {
        std::visit([&j](auto&& arg) {
            j = arg; // 自动推导类型
        }, v);
    }

    static void from_json(const json& j, std::variant<std::string, int>& v) {
        if (j.is_string()) {
            v = j.get<std::string>();
        } else if (j.is_number_integer()) {
            v = j.get<int>();
        } else {
            throw json::type_error::create(302, "type must be string or integer", &j);
        }
    }
};
} // namespace nlohmann

namespace nlohmann {
    template <>
    struct adl_serializer<std::variant<std::monostate, int64_t, std::string>> {
        static void to_json(json& j, const std::variant<std::monostate, int64_t, std::string>& id) {
            std::visit([&j](auto&& v) {
                using T = std::decay_t<decltype(v)>;
                if constexpr (std::is_same_v<T, std::monostate>) {
                    j = nullptr;
                } else {
                    j = v;
                }
            }, id);
        }

        static void from_json(const json& j, std::variant<std::monostate, int64_t, std::string>& id) {
            if (j.is_null()) {
                id = std::monostate{};
            } else if (j.is_number_integer()) {
                id = j.get<int64_t>();
            } else if (j.is_string()) {
                id = j.get<std::string>();
            } else {
                throw json::type_error::create(302, "id must be null, integer or string", &j);
            }
        }
    };
} // namespace nlohmann

using json = nlohmann::json;

namespace mcp::protocol {

    
// ===== JSON-RPC Error Object =====
struct JSONRPCError {
    int code;
    std::string message;
    std::optional<json> data;
};

inline void to_json(json& j, const JSONRPCError& e) {
    j = json{{"code", e.code}, {"message", e.message}};
    if (e.data) j["data"] = *e.data;
}
inline void from_json(const json& j, JSONRPCError& e) {
    j.at("code").get_to(e.code);
    j.at("message").get_to(e.message);
    if (j.contains("data")) j.at("data").get_to(e.data);
}

// ===== JSON-RPC Request =====
struct JSONRPCRequest {
    std::string jsonrpc = "2.0";
    std::variant<std::string, int> id;
    std::string method;
    std::optional<json> params;
};

inline void to_json(json& j, const JSONRPCRequest& r) {
    j["jsonrpc"] = r.jsonrpc;
    j["id"]      = r.id;
    j["method"]  = r.method;
    if (r.params) j["params"] = *r.params;
}
inline void from_json(const json& j, JSONRPCRequest& r) {
    j.at("jsonrpc").get_to(r.jsonrpc);
    j.at("id").get_to(r.id);
    j.at("method").get_to(r.method);
    if (j.contains("params")) r.params = j.at("params");
}

// ===== JSON-RPC Notification =====
struct JSONRPCNotification {
    std::string jsonrpc = "2.0";
    std::string method;
    std::optional<json> params;
};

inline void to_json(json& j, const JSONRPCNotification& n) {
    j["jsonrpc"] = n.jsonrpc;
    j["method"]  = n.method;
    if (n.params) j["params"] = *n.params;
}
inline void from_json(const json& j, JSONRPCNotification& n) {
    j.at("jsonrpc").get_to(n.jsonrpc);
    j.at("method").get_to(n.method);
    if (j.contains("params")) n.params = j.at("params");
}

// ===== JSON-RPC Response =====
struct JSONRPCResponse {
    std::string jsonrpc = "2.0";
    std::variant<std::string, int> id;
    json result;
};

inline void to_json(json& j, const JSONRPCResponse& r) {
    j["jsonrpc"] = r.jsonrpc;
    j["id"]      = r.id;
    j["result"]  = r.result;
}
inline void from_json(const json& j, JSONRPCResponse& r) {
    j.at("jsonrpc").get_to(r.jsonrpc);
    j.at("id").get_to(r.id);
    j.at("result").get_to(r.result);
}

// ===== JSON-RPC Error Response =====
struct JSONRPCErrorResponse {
    std::string jsonrpc = "2.0";
    std::variant<std::string, int> id;
    JSONRPCError error;
};

inline void to_json(json& j, const JSONRPCErrorResponse& r) {
    j["jsonrpc"] = r.jsonrpc;
    j["id"]      = r.id;
    j["error"]   = r.error;
}
inline void from_json(const json& j, JSONRPCErrorResponse& r) {
    j.at("jsonrpc").get_to(r.jsonrpc);
    j.at("id").get_to(r.id);
    j.at("error").get_to(r.error);
}

// ===== JSON-RPC Batch =====


// ===== JSON-RPC Batch Types =====

// Batch request: array of Request or Notification
using JSONRPCBatchRequest  = std::vector<std::variant<JSONRPCRequest, JSONRPCNotification>>;
// Batch response: array of Response or ErrorResponse
using JSONRPCBatchResponse = std::vector<std::variant<JSONRPCResponse, JSONRPCErrorResponse>>;

inline void to_json(json& j, const JSONRPCBatchRequest& batch) {
    j = json::array();
    for (const auto& msg : batch) {
        std::visit([&j](auto&& arg){ j.push_back(arg); }, msg);
    }
}

inline void from_json(const json& j, JSONRPCBatchRequest& batch) {
    for (const auto& elem : j) {
        if (elem.contains("jsonrpc") && elem.contains("method")) {
            if (elem.contains("id")) {
                batch.emplace_back(elem.get<JSONRPCRequest>());
            } else {
                batch.emplace_back(elem.get<JSONRPCNotification>());
            }
        }
    }
}

inline void to_json(json& j, const JSONRPCBatchResponse& batch) {
    j = json::array();
    for (const auto& msg : batch) {
        std::visit([&j](auto&& arg){ j.push_back(arg); }, msg);
    }
}

inline void from_json(const json& j, JSONRPCBatchResponse& batch) {
    for (const auto& elem : j) {
        if (elem.contains("jsonrpc") && elem.contains("id")) {
            if (elem.contains("result")) {
                batch.emplace_back(elem.get<JSONRPCResponse>());
            } else if (elem.contains("error")) {
                batch.emplace_back(elem.get<JSONRPCErrorResponse>());
            }
        }
    }
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


    /* 本项目用了 string / integer 两类 ID，统一存成 nlohmann::json */
using JsonRpcId = nlohmann::json;
using json      = nlohmann::json;

/* ------------------------------------------------------------------ */
/*  makeJsonRpcResult()                                               */
/* ------------------------------------------------------------------ */
template<typename ResultT>
inline json makeJsonRpcResult(const JsonRpcId& id, const ResultT& result)
{
    return json{
        {"jsonrpc", "2.0"},
        {"id",      id},
        {"result",  result}
    };
}

/* ------------------------------------------------------------------ */
/*  makeJsonRpcError()                                                */
/* ------------------------------------------------------------------ */
template<typename DataT = std::nullptr_t>
inline json makeJsonRpcError(const JsonRpcId& id,
                             int               code,
                             const std::string& message,
                             const DataT&       data = nullptr)
{
    json err = {
        {"code",    code},
        {"message", message}
    };
    if constexpr(!std::is_same_v<DataT,std::nullptr_t>)
        err["data"] = data;

    return json{
        {"jsonrpc", "2.0"},
        {"id",      id},
        {"error",   std::move(err)}
    };
}

    using RequestId = std::variant<std::monostate, int64_t, std::string>;

/* tiny helper 把 Id → json */
inline json idToJson(const RequestId& id) {
    return std::visit([](auto&& v) -> json {
        using T = std::decay_t<decltype(v)>;
        if constexpr (std::is_same_v<T, std::monostate>) {
            return nullptr; // 或 json::object() 根据协议要求
        } else {
            return json(v); // 正常处理 int/string
        }
    }, id);
}


    inline void to_json(nlohmann::json& j, const RequestId& id) {
    j = std::visit([](auto&& v) -> nlohmann::json {
        using T = std::decay_t<decltype(v)>;
        if constexpr (std::is_same_v<T, std::monostate>) {
            return nullptr; // 或 json::object() 根据协议要求
        } else {
            return nlohmann::json(v); // 正常处理 int/string
        }
    }, id);
}

    /* -------  id 解析工具  ------- */
    inline void from_json(const nlohmann::json& j, RequestId& id) {
    if (j.contains("id")) {
        if (j["id"].is_number_integer()) {
            id = j["id"].get<int64_t>();
        } else if (j["id"].is_string()) {
            id = j["id"].get<std::string>();
        } else {
            throw std::invalid_argument("Invalid type for 'id' field");
        }
    } else {
        id = {}; // empty (std::monostate)
    }
}


/* -------  构造器  ------- */
inline json makeRequest(const RequestId& id,
                        const std::string& method,
                        json params = json::object())
{
    json j = { {"jsonrpc","2.0"}, {"method",method}, {"params",std::move(params)} };
     json idJson = idToJson(id);
    if (!idJson.is_null()) { // ✅ 更安全的空值检查
        j["id"] = std::move(idJson);
    }
    return j;
 //   if(!std::holds_alternative<std::monostate>(id))
  //      j["id"] = idToJson(id);
  //  return j;
}

inline json makeResult(const RequestId& id, json result = json::object()){
        return { {"jsonrpc","2.0"},
             {"id",   idToJson(id)},
             {"result", std::move(result)} };
}

inline json makeError (const RequestId& id, int code, const std::string& msg){
    return { {"jsonrpc","2.0"},
             {"id",   idToJson(id)},
             {"error", {{"code",code},{"message",msg}} } };
}

/* -------  id 解析工具  ------- */
inline RequestId parseId(const json& j) {
    if(j.contains("id")){
        if(j["id"].is_number_integer()) return j["id"].get<int64_t>();
        if(j["id"].is_string())         return j["id"].get<std::string>();
    }
    return {}; // empty
}

/* ------------------------------------------------------------------ */
/*  JSON‑RPC "id" 支持三种形态：null / integer / string               */
/* ------------------------------------------------------------------ */
// using RequestId = std::variant<std::monostate, int64_t, std::string>;
//
// /* 序列化 / 反序列化  */
// inline void to_json(json& j, const RequestId& id)
// {
//     std::visit([&](auto&& v){
//         using T = std::decay_t<decltype(v)>;
//         if constexpr (std::is_same_v<T, std::monostate>)
//             j = nullptr;
//         else
//             j = v;
//     }, id);
// }
//
// inline void from_json(const json& j, RequestId& id)
// {
//     if(j.is_null())               id = std::monostate{};
//     else if(j.is_number_integer()) id = j.get<int64_t>();
//     else if(j.is_string())        id = j.get<std::string>();
//     else
//         throw std::runtime_error("invalid JSON‑RPC id");
// }

/* ------------------------------------------------------------------ */
/*  构造 Request / Result / Error                                      */
/* ------------------------------------------------------------------ */
inline json
makeJsonRpcRequest(const RequestId& id,
                   std::string_view method,
                   json params = json::object())
{
    json req{
        {"jsonrpc", "2.0"},
        {"method",  method},
        {"params",  std::move(params)}
    };
    if(!std::holds_alternative<std::monostate>(id))
        req["id"] = id;
    return req;
}

inline json
makeJsonRpcResult(const RequestId& id, json result)
{
    return json{
        {"jsonrpc", "2.0"},
        {"id",      id},
        {"result",  std::move(result)}
    };
}

inline json
makeJsonRpcError(const RequestId& id,
                 int        code,
                 std::string message,
                 json        data = json::object())
{
    json err{
        {"code",    code},
        {"message", std::move(message)}
    };
    if(!data.empty())
        err["data"] = std::move(data);

    return {
        {"jsonrpc", "2.0"},
        {"id",      id},
        {"error",   std::move(err)}
    };
}

/* ------------------------------------------------------------------ */
/*  快捷解析：提取 id / method / params；若格式非法返回 false          */
/* ------------------------------------------------------------------ */
inline bool
getBasicRequestFields(const json&  j,
                      RequestId&          idOut,
                      std::string& methodOut,
                      json&        paramsOut)
{
    if(!j.is_object() || j.value("jsonrpc","") != "2.0" || !j.contains("method"))
        return false;

    methodOut = j["method"].get<std::string>();
    paramsOut = j.value("params", json::object());
    // idOut     = j.value("id",     RequestId{std::monostate{}}).get<RequestId>();
    idOut = j.value("id", idToJson(RequestId{})) // 使用辅助函数转换
                .get<RequestId>();
    return true;
}

}