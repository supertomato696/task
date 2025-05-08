// #pragma once
// #include "protocol/McpPromptRequests.hpp"   // ListPrompts*, GetPrompt*
// #include "protocol/McpJsonRpc.hpp"
// #include "prompt/PromptStore.hpp"

// #include <nlohmann/json.hpp>
// #include <functional>

// namespace mcp::prompt {

// /**
//  * PromptService —— JSON‑RPC 处理器
//  * ---------------------------------
//  *  • prompts/list          → ListPromptsResult
//  *  • prompts/get           → GetPromptResult  (通过 PromptStore::renderPrompt)
//  *  • fireListChanged()     → 主动发 list_changed 通知
//  *
//  *  Notify 回调 typdef：void(const nlohmann::json& readyJson)
//  */
// class PromptService {
// public:
//     using json   = nlohmann::json;
//     using Notify = std::function<void(const json&)>;

//     explicit PromptService(PromptStore& store, Notify notifier = {});

//     /** 主入口：处理到来的 JSON‑RPC 请求 */
//     json handleRpc(const json& request);

//     /** 当模板集发生变更时可调用，向客户端推送通知 */
//     void fireListChanged() const;

// private:
//     /* 各 RPC 的强类型实现 */
//     protocol::ListPromptsResult handleList() const;

//     protocol::GetPromptResult handleGet(
//         const protocol::GetPromptRequest& req);

//     PromptStore& store_;
//     Notify       notify_;
// };

// } // namespace
#pragma once
#include <nlohmann/json.hpp>
#include <functional>

#include "prompt/PromptStore.hpp"
#include "protocol/McpPromptRequests.hpp"   // ListPromptsResult / GetPrompt*
#include "protocol/McpJsonRpc.hpp"          // makeJsonRpc*

namespace mcp::prompt {

/**
 * PromptService
 * ---------------------------------------------------------------
 * 负责处理 prompts/list 及 prompts/get JSON‑RPC 请求，
 * 并在模板增删时自动广播 notifications/prompts/list_changed。
 *
 * 使用方式：
 *   PromptService svc(store, /* lambda 发送 JSON 通知 *​/);
 *   transport.onMessage([&](auto& msg){ transport.send( svc.handleRpc(msg) ); });
 */
class PromptService {
public:
    using json   = nlohmann::json;
    using Notify = std::function<void(const json&)>;   // 发送通知的回调

    PromptService(PromptStore& store, Notify notifier = nullptr);

    /** 统一入口——收到任意 prompts/* 方法调用时交给它 */
    json handleRpc(const json& request);

private:
    /* 实际的业务处理： */
    protocol::ListPromptsResult handleList() const;
    protocol::GetPromptResult   handleGet(const protocol::GetPromptRequest& req);

    /** 触发 notifications/prompts/list_changed */
    void fireListChanged() const;

    /* ------------------------------------------------------------------ */
    PromptStore& store_;
    Notify       notify_;
};

} // namespace mcp::prompt
