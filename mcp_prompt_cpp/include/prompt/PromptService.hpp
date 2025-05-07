#pragma once
#include "protocol/McpPromptRequests.hpp"   // ListPrompts*, GetPrompt*
#include "protocol/McpJsonRpc.hpp"
#include "prompt/PromptStore.hpp"

#include <nlohmann/json.hpp>
#include <functional>

namespace mcp::prompt {

/**
 * PromptService —— JSON‑RPC 处理器
 * ---------------------------------
 *  • prompts/list          → ListPromptsResult
 *  • prompts/get           → GetPromptResult  (通过 PromptStore::renderPrompt)
 *  • fireListChanged()     → 主动发 list_changed 通知
 *
 *  Notify 回调 typdef：void(const nlohmann::json& readyJson)
 */
class PromptService {
public:
    using json   = nlohmann::json;
    using Notify = std::function<void(const json&)>;

    explicit PromptService(PromptStore& store, Notify notifier = {});

    /** 主入口：处理到来的 JSON‑RPC 请求 */
    json handleRpc(const json& request);

    /** 当模板集发生变更时可调用，向客户端推送通知 */
    void fireListChanged() const;

private:
    /* 各 RPC 的强类型实现 */
    protocol::ListPromptsResult handleList() const;

    protocol::GetPromptResult handleGet(
        const protocol::GetPromptRequest& req);

    PromptStore& store_;
    Notify       notify_;
};

} // namespace
