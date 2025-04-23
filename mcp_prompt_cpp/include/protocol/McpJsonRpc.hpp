#pragma once

#include <string>
#include <vector>
#include <variant>
#include <optional>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

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
using JSONRPCBatchRequest  = std::vector<json>;  // array of JSONRPCRequest or JSONRPCNotification
using JSONRPCBatchResponse = std::vector<json>;  // array of JSONRPCResponse or JSONRPCErrorResponse

