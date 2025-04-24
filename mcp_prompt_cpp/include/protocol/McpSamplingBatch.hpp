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


}
