#pragma once
#include "prompt/IPromptBackend.hpp"
#include <unordered_map>
#include <mutex>

namespace mcp::prompt {

class MemoryBackend : public IPromptBackend {
public:
    bool upsert(const StoredPrompt&)                override;
    bool erase (const std::string& name)            override;
    std::optional<StoredPrompt> get(const std::string& name) const override;
    std::vector<StoredPrompt>   list() const        override;

private:
    mutable std::mutex                         mtx_;
    std::unordered_map<std::string,StoredPrompt> map_;
};

} // namespace