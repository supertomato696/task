#pragma once

#include <string>
#include <vector>
#include <optional>
#include <nlohmann/json.hpp>
#include "McpContent.hpp"  // 包含 TextContent, ImageContent, AudioContent, PromptMessage

using json = nlohmann::json;

// ===== Implementation =====
// MCP 协议中的实现元信息
struct Implementation {
    std::string name;
    std::string version;
};
inline void to_json(json& j, const Implementation& impl) {
    j = json{{"name", impl.name}, {"version", impl.version}};
}
inline void from_json(const json& j, Implementation& impl) {
    j.at("name").get_to(impl.name);
    j.at("version").get_to(impl.version);
}

// ===== InitializeRequest / InitializeResult =====

struct InitializeRequest {
    std::string method = "initialize";

    struct Params {
        json capabilities;           // 客户端能力声明
        Implementation clientInfo;
        std::string protocolVersion;
    } params;
};

inline void to_json(json& j, const InitializeRequest::Params& p) {
    j = json{
        {"capabilities",    p.capabilities},
        {"clientInfo",      p.clientInfo},
        {"protocolVersion", p.protocolVersion}
    };
}
inline void from_json(const json& j, InitializeRequest::Params& p) {
    j.at("capabilities").get_to(p.capabilities);
    j.at("clientInfo").get_to(p.clientInfo);
    j.at("protocolVersion").get_to(p.protocolVersion);
}
inline void to_json(json& j, const InitializeRequest& r) {
    j = json{{"method", r.method}, {"params", r.params}};
}
inline void from_json(const json& j, InitializeRequest& r) {
    j.at("params").get_to(r.params);
}

struct InitializeResult {
    std::optional<json> _meta;
    std::string protocolVersion;
    json capabilities;           // 服务器能力
    Implementation serverInfo;
    std::optional<std::string> instructions;
};
inline void to_json(json& j, const InitializeResult& r) {
    j = json{
        {"protocolVersion", r.protocolVersion},
        {"capabilities",    r.capabilities},
        {"serverInfo",      r.serverInfo}
    };
    if (r._meta)        j["_meta"]        = *r._meta;
    if (r.instructions) j["instructions"] = *r.instructions;
}
inline void from_json(const json& j, InitializeResult& r) {
    if (j.contains("_meta"))        j.at("_meta").get_to(r._meta);
    j.at("protocolVersion").get_to(r.protocolVersion);
    j.at("capabilities").get_to(r.capabilities);
    j.at("serverInfo").get_to(r.serverInfo);
    if (j.contains("instructions")) j.at("instructions").get_to(r.instructions);
}

// ===== PingRequest / PingResult =====

struct PingRequest {
    std::string method = "ping";
    // 可选 _meta 进度令牌省略
};
inline void to_json(json& j, const PingRequest& r) {
    j = json{{"method", r.method}, {"params", json::object()}};
}
inline void from_json(const json& j, PingRequest& r) {
    // 无 params 字段数据
}

// Ping 不返回额外结果，使用空结构体
struct PingResult {};
inline void to_json(json& j, const PingResult&) { j = json{}; }
inline void from_json(const json&, PingResult&) {}

// ===== Sampling: CreateMessageRequest / CreateMessageResult =====

// 将 PromptMessage 用作 SamplingMessage
using SamplingMessage = PromptMessage;

struct CreateMessageRequest {
    std::string method = "sampling/createMessage";

    struct Params {
        std::optional<std::string> includeContext;
        int maxTokens;
        std::vector<SamplingMessage> messages;
        std::optional<json> metadata;
        std::optional<json> modelPreferences;
        std::optional<std::vector<std::string>> stopSequences;
        std::optional<std::string> systemPrompt;
        std::optional<double> temperature;
    } params;
};

inline void to_json(json& j, const CreateMessageRequest::Params& p) {
    j = json{{"maxTokens", p.maxTokens}, {"messages", p.messages}};
    if (p.includeContext)   j["includeContext"]   = *p.includeContext;
    if (p.metadata)         j["metadata"]         = *p.metadata;
    if (p.modelPreferences) j["modelPreferences"] = *p.modelPreferences;
    if (p.stopSequences)    j["stopSequences"]    = *p.stopSequences;
    if (p.systemPrompt)     j["systemPrompt"]     = *p.systemPrompt;
    if (p.temperature)      j["temperature"]      = *p.temperature;
}
inline void from_json(const json& j, CreateMessageRequest::Params& p) {
    if (j.contains("includeContext"))   j.at("includeContext").get_to(p.includeContext);
    j.at("maxTokens").get_to(p.maxTokens);
    j.at("messages").get_to(p.messages);
    if (j.contains("metadata"))         j.at("metadata").get_to(p.metadata);
    if (j.contains("modelPreferences")) j.at("modelPreferences").get_to(p.modelPreferences);
    if (j.contains("stopSequences"))    j.at("stopSequences").get_to(p.stopSequences);
    if (j.contains("systemPrompt"))     j.at("systemPrompt").get_to(p.systemPrompt);
    if (j.contains("temperature"))      j.at("temperature").get_to(p.temperature);
}
inline void to_json(json& j, const CreateMessageRequest& r) {
    j = json{{"method", r.method}, {"params", r.params}};
}
inline void from_json(const json& j, CreateMessageRequest& r) {
    j.at("params").get_to(r.params);
}

struct CreateMessageResult {
    std::optional<json> _meta;
    std::variant<TextContent, ImageContent, AudioContent> content;
    std::string model;
    Role role;
    std::optional<std::string> stopReason;
};

inline void to_json(json& j, const CreateMessageResult& r) {
    if (r._meta)     j["_meta"]     = *r._meta;
    // content
    std::visit([&](auto&& arg){ j["content"] = arg; }, r.content);
    j["model"]      = r.model;
    j["role"]       = r.role;
    if (r.stopReason) j["stopReason"] = *r.stopReason;
}
inline void from_json(const json& j, CreateMessageResult& r) {
    if (j.contains("_meta"))     j.at("_meta").get_to(r._meta);
    const auto& c = j.at("content");
    auto type = c.at("type").get<std::string>();
    if (type == "text")     r.content = c.get<TextContent>();
    else if (type == "image")   r.content = c.get<ImageContent>();
    else if (type == "audio")   r.content = c.get<AudioContent>();
    j.at("model").get_to(r.model);
    j.at("role").get_to(r.role);
    if (j.contains("stopReason")) j.at("stopReason").get_to(r.stopReason);
}
