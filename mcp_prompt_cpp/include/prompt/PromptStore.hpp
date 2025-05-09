// #pragma once
// #include "prompt/PromptTemplate.hpp"
// #include "prompt/TemplateEngine.hpp"
// #include "prompt/MultiModalAssembler.hpp"
// #include "protocol/McpPromptRequests.hpp"
//
// #include <unordered_map>
// #include <vector>
// #include <optional>
// #include <mutex>
// #include <functional>
//
// namespace mcp::prompt {
//
// /**
//  * 已渲染 Prompt (Filled Prompt)
//  * --------------------------------------------------
//  * - key         唯一标识：模板名 + 实参哈希，可自行决定算法
//  * - description 保留模板里的 description
//  * - messages    渲染完成、富媒体已就位的 PromptMessage 列表
//  */
// struct FilledPrompt {
//     std::string                                  key;
//     protocol::GetPromptResult                    data;   // {description, messages}
// };
//
// /**
//  * PromptStore
//  * --------------------------------------------------
//  * 线程安全（内部使用 std::mutex），支持：
//  *   • 模板 CRUD
//  *   • 运行时按模板 + arguments 渲染 → FilledPrompt 并缓存
//  *   • 列表查询
//  */
// class PromptStore {
// public:
// using OnChanged = std::function<void()>;      // ★ 新增
//     /* ---------- 模板 ---------- */
//     bool addTemplate (PromptTemplate t);            ///< insert/overwrite
//     bool removeTemplate(const std::string& name);
//     std::optional<PromptTemplate> getTemplate(const std::string& name) const;
//     std::vector<protocol::Prompt> listTemplates() const;       ///< prompts/list
//
//     /* ---------- 渲染 / Filled ---------- */
//     /** 渲染并返回 FilledPrompt，若已缓存则直接返回 */
//     FilledPrompt renderPrompt(const std::string& name,
//                               const std::unordered_map<std::string,std::string>& args);
//
//     /** 只读获取已渲染 Prompt（若存在于缓存） */
//     std::optional<FilledPrompt> getFilled(const std::string& key) const;
//
//     /** 清空全部已渲染缓存（可选） */
//     void clearFilledCache();
//
//      /* --------- 订阅变更 --------- */
//      void setOnChanged(OnChanged cb) { onChanged_ = std::move(cb); }  // ★ 新增
//
// private:
//     /* 生成缓存 key = name + "|" + md5(sorted args) */
//     static std::string makeKey(const std::string& name,
//                                const std::unordered_map<std::string,std::string>& args);
//
//     /* --- data --- */
//     std::unordered_map<std::string, PromptTemplate> templates_;
//     std::unordered_map<std::string, FilledPrompt>  filledCache_;
//
//     mutable std::mutex mtx_;   ///< 粗粒度锁即可（在内存版）
//     OnChanged onChanged_;                         // ★ 新增
// };
//
// } // namespace

// #pragma once
// #include "prompt/MemoryBackend.hpp"
// #include "prompt/SqliteBackend.hpp"
// #include <functional>

// namespace mcp::prompt {

// class PromptStore {
// public:
//     using OnChanged = std::function<void()>;

//     explicit PromptStore(const std::string& sqlitePath = "prompts.db")
//     : disk_(sqlitePath)
//     {
//         // 首次启动把磁盘所有数据灌入内存
//         for (auto& sp : disk_.list()) mem_.upsert(sp);
//     }

//     /* CRUD 走内存，并同步磁盘 */
//     bool add(const StoredPrompt& sp) {
//         bool fresh = mem_.upsert(sp);
//         disk_.upsert(sp);
//         if(onChanged_) onChanged_();
//         return fresh;
//     }
//     bool remove(const std::string& n) {
//         bool ok = mem_.erase(n);
//         if (ok) { disk_.erase(n); if(onChanged_) onChanged_(); }
//         return ok;
//     }

//     std::optional<StoredPrompt> get(const std::string& n) const {
//         return mem_.get(n);
//     }
//     std::vector<StoredPrompt>   list() const {
//         return mem_.list();
//     }

//     void setOnChanged(OnChanged cb){ onChanged_=std::move(cb); }

// private:
//     MemoryBackend  mem_;
//     SqliteBackend  disk_;
//     OnChanged      onChanged_;
// };

// } // namespace


#pragma once
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <string>
#include <optional>
#include <mutex>
#include <functional>

#include <nlohmann/json.hpp>

#include "protocol/McpContent.hpp"          // PromptMessage / TextContent …
#include "protocol/McpPromptRequests.hpp"   // Prompt / GetPromptResult
#include "prompt/MultiModalAssembler.hpp"   // 富媒体转换
#include "prompt/PromptTemplate.hpp"

namespace mcp::prompt {

/* ------------------------------------------------------------------ */
/*  PromptTemplate —— 纯“模板”，未渲染                                */
/* ------------------------------------------------------------------ */
// struct PromptTemplate {
//     protocol::Prompt                         meta;      // name / description / arguments[]
//     std::vector<protocol::PromptMessage>     messages;  // 可包含 {{arg}} 占位
// };

/* ------------------------------------------------------------------ */
/*  FilledPrompt —— render 后的结果（可直接作为 prompts/get Result）   */
/* ------------------------------------------------------------------ */
struct FilledPrompt {
    std::string                       key;   // template+args → 唯一 key
    protocol::GetPromptResult         data;  // {description, messages}
};

/* ------------------------------------------------------------------ */
/*  PromptStore                                                       */
/*    · 线程安全 · 内存缓存 · 支持 onChanged 回调                      */
/*    · renderPrompt() 会自动调用 MultiModalAssembler                 */
/* ------------------------------------------------------------------ */
class PromptStore {
public:
    /* onChanged 用于通知 PromptService 发布 list_changed */
    using ChangedCb = std::function<void()>;
    void setOnChanged(ChangedCb cb) { onChanged_ = std::move(cb); }

    /* ---------------- 模板 CRUD ---------------- */
    bool addTemplate(PromptTemplate tpl);            // return true if new
    bool removeTemplate(const std::string& name);

    std::optional<PromptTemplate> getTemplate(const std::string& name) const;
    std::vector<protocol::Prompt> listTemplates() const;

    /* ---------------- 渲染接口 ------------------ */
    FilledPrompt renderPrompt(
        const std::string&                          name,
        const std::unordered_map<std::string,std::string>& args);

    /* 填充结果缓存查询 / 清空 */
    std::optional<FilledPrompt> getFilled(const std::string& key) const;
    void clearFilledCache();

private:
    /* 生成唯一缓存 key（模板名 + args → md5） */
    static std::string makeKey(
        const std::string&                          name,
        const std::unordered_map<std::string,std::string>& args);

    /* ---------------- 数据成员 ------------------ */
    mutable std::mutex                                      mtx_;
    std::unordered_map<std::string, PromptTemplate>         templates_;
    std::unordered_map<std::string, FilledPrompt>           filledCache_;
    ChangedCb                                               onChanged_;
};

} // namespace mcp::prompt
