#pragma once

#include <vector>
#include <variant>
#include <nlohmann/json.hpp>
#include "McpContent.hpp"      // TextContent, ImageContent, AudioContent, Role
#include "McpJsonRpc.hpp"       // JSONRPCRequest, JSONRPCNotification, JSONRPCResponse, JSONRPCErrorResponse

namespace mcp::protocol {
using json = nlohmann::json;

// ===== SamplingMessage =====
// Describes a message issued to or received from an LLM API.
struct SamplingMessage {
    std::variant<TextContent, ImageContent, AudioContent> content;
    Role role;
};

inline void to_json(json& j, const SamplingMessage& m) {
    std::visit([&j](auto&& arg){ j["content"] = arg; }, m.content);
    j["role"] = m.role;
}

inline void from_json(const json& j, SamplingMessage& m) {
    const auto& c = j.at("content");
    auto type = c.at("type").get<std::string>();
    if (type == "text")     m.content = c.get<TextContent>();
    else if (type == "image")   m.content = c.get<ImageContent>();
    else if (type == "audio")   m.content = c.get<AudioContent>();
    j.at("role").get_to(m.role);
}


// ===== Sampling: CreateMessageRequest / CreateMessageResult =====

// 将 PromptMessage 用作 SamplingMessage
//using SamplingMessage = PromptMessage;

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
    if (j.contains("stopSequences") && !j.at("stopSequences").is_null())   {
      p.stopSequences = j.at("stopSequences").get<std::vector<std::string>>();
     }

    if (j.contains("systemPrompt"))     j.at("systemPrompt").get_to(p.systemPrompt);
    if (j.contains("temperature") && !j.at("temperature").is_null())   {
      p.temperature = j.at("temperature").get<double>();
      }
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


}
