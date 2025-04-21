#pragma once
#include <nlohmann/json.hpp>
#include <vector>
#include <string>
#include <unordered_map>

namespace mcp::client {

struct PromptInfo {
    std::string name;
    std::string description;
    std::vector<std::string> arguments;
};

struct PromptMessage {
    nlohmann::json content;   // 直接保留 JSON, 方便传给 LLM
    std::string    role;
};

struct GetPromptResult {
    std::string                   description;
    std::vector<PromptMessage>    messages;
};

using ArgMap = std::unordered_map<std::string,std::string>;

} // namespace mcp::client