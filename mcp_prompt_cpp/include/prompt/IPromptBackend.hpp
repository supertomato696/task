#pragma once
#include "protocol/McpPromptRequests.hpp"
#include <optional>
#include <vector>

namespace mcp::prompt {

struct StoredPrompt {
    protocol::Prompt                       meta;
    std::vector<protocol::PromptMessage>   messages;
    int                                    version = 1;
};

class IPromptBackend {
public:
    virtual ~IPromptBackend() = default;

    virtual bool                     upsert(const StoredPrompt&)               = 0;
    virtual bool                     erase (const std::string& name)           = 0;
    virtual std::optional<StoredPrompt> get(const std::string& name)   const   = 0;
    virtual std::vector<StoredPrompt>   list()                         const   = 0;
};

} // namespace