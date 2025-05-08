#pragma once
// #include "prompt/PromptStore.hpp"          // 复用 PromptTemplate
#include <sqlite_modern_cpp.h>
#include <mutex>
#include <prompt/PromptTemplate.hpp>

namespace mcp::prompt {

class SqlitePromptStore : public IPromptStore {
public:
    /** dbPath: e.g. "/var/lib/mcp/prompts.db" */
    explicit SqlitePromptStore(const std::string& dbPath);

    /* PromptStore API 覆盖 */
    void add(const PromptTemplate& t) override;
    const PromptTemplate* find(const std::string& n) const override;
    std::vector<PromptTemplate> all() const override;

private:
    void ensureSchema();

    mutable sqlite::database db_;
    mutable std::mutex       mtx_;          // 简单串行化
    mutable std::unordered_map<std::string, PromptTemplate> cache_;
    void fillCache_locked() const;          // 懒加载
};

} // namespace mcp