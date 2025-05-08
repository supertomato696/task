#pragma once
#include "prompt/IPromptBackend.hpp"
#include <sqlite_modern_cpp.h>
#include <mutex>

namespace mcp::prompt {

class SqliteBackend : public IPromptBackend {
public:
    explicit SqliteBackend(const std::string& dbPath);

    bool upsert(const StoredPrompt&)                override;
    bool erase (const std::string& name)            override;
    std::optional<StoredPrompt> get(const std::string& name) const override;
    std::vector<StoredPrompt>   list() const        override;

private:
    void ensureSchema();

    mutable std::mutex      mtx_;
    mutable sqlite::database        db_;
};

} // namespace