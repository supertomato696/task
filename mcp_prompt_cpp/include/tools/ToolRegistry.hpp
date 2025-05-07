#pragma once
/*****************************************************************************
 * ToolRegistry  ——  强类型工具注册中心
 * --------------------------------------------------------------------------
 *  · 维护  <tool name , ToolDef>
 *  · listTools()      →  std::vector<protocol::Tool>
 *  · invoke()         →  ToolReply (= std::vector<PromptMessage>)
 *  · register / remove 时触发 onChanged_()，由 ToolService 订阅以广播
 *
 *  Author: ChatGPT implementation – 2025‑05‑07
 *****************************************************************************/
#include <nlohmann/json.hpp>
#include <functional>
#include <unordered_map>
#include <vector>

#include "protocol/McpToolRequests.hpp"   // Tool, PromptMessage, Role 等

namespace mcp::tools {

/* ---- 便捷别名 --------------------------------------------------------- */
using ToolArgs  = nlohmann::json;                // 参数采用原生 json
using ToolReply = std::vector<protocol::PromptMessage>;   // 返回给 LLM 的内容

using ToolCallback = std::function<ToolReply(const ToolArgs&)>;

/* ---- 描述一个工具 ----------------------------------------------------- */
struct ToolDef {
    /* 元信息直接采用 MCP 的 Tool 结构体 */
    protocol::Tool  meta;          // name / description / inputSchema / annotations

    /* 执行回调 —— 必须保证线程安全或自行加锁 */
    ToolCallback    callback;
};

/* ---------------------------------------------------------------------- */
class ToolRegistry {
public:
    /* ---- 注册 / 移除 -------------------------------------------------- */
    void registerTool(const ToolDef& def);                 // meta.name 唯一
    void removeTool  (const std::string& name);

    /* ---- 查询 -------------------------------------------------------- */
    std::vector<protocol::Tool> listTools() const;

    /* ---- 调用 -------------------------------------------------------- */
    ToolReply invoke(const std::string& name,
                     const ToolArgs&    args) const;

    /* ---- 变化回调 ---------------------------------------------------- */
    using ChangedCb = std::function<void()>;
    void setChangedCb(ChangedCb cb) { onChanged_ = std::move(cb); }

private:
    std::unordered_map<std::string, ToolDef> map_;
    ChangedCb onChanged_;          // nullptr if not set
};

} // namespace mcp::tools
