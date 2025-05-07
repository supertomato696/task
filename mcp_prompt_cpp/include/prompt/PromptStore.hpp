#pragma once
#include "prompt/PromptTemplate.hpp"
#include "prompt/TemplateEngine.hpp"
#include "prompt/MultiModalAssembler.hpp"
#include "protocol/McpPromptRequests.hpp"

#include <unordered_map>
#include <vector>
#include <optional>
#include <mutex>

namespace mcp::prompt {

/**
 * 已渲染 Prompt (Filled Prompt)
 * --------------------------------------------------
 * - key         唯一标识：模板名 + 实参哈希，可自行决定算法
 * - description 保留模板里的 description
 * - messages    渲染完成、富媒体已就位的 PromptMessage 列表
 */
struct FilledPrompt {
    std::string                                  key;
    protocol::GetPromptResult                    data;   // {description, messages}
};

/**
 * PromptStore
 * --------------------------------------------------
 * 线程安全（内部使用 std::mutex），支持：
 *   • 模板 CRUD
 *   • 运行时按模板 + arguments 渲染 → FilledPrompt 并缓存
 *   • 列表查询
 */
class PromptStore {
public:
    /* ---------- 模板 ---------- */
    bool addTemplate (PromptTemplate t);            ///< insert/overwrite
    bool removeTemplate(const std::string& name);
    std::optional<PromptTemplate> getTemplate(const std::string& name) const;
    std::vector<protocol::Prompt> listTemplates() const;       ///< prompts/list

    /* ---------- 渲染 / Filled ---------- */
    /** 渲染并返回 FilledPrompt，若已缓存则直接返回 */
    FilledPrompt renderPrompt(const std::string& name,
                              const std::unordered_map<std::string,std::string>& args);

    /** 只读获取已渲染 Prompt（若存在于缓存） */
    std::optional<FilledPrompt> getFilled(const std::string& key) const;

    /** 清空全部已渲染缓存（可选） */
    void clearFilledCache();

private:
    /* 生成缓存 key = name + "|" + md5(sorted args) */
    static std::string makeKey(const std::string& name,
                               const std::unordered_map<std::string,std::string>& args);

    /* --- data --- */
    std::unordered_map<std::string, PromptTemplate> templates_;
    std::unordered_map<std::string, FilledPrompt>  filledCache_;

    mutable std::mutex mtx_;   ///< 粗粒度锁即可（在内存版）
};

} // namespace