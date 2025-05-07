#pragma once
#include "mcp/client2/JsonRpcClient.hpp"
#include "McpPromptRequests.hpp"     
#include <future>
#include <optional>
#include <shared_mutex>

namespace mcp::client2 {

/**
 * PromptClient — 封装 prompts/* 子协议
 *
 * ✦ listPrompts        同步 / 异步
 * ✦ getPrompt          同步 / 异步
 *
 * 后期若要加入 completion/complete 等，只需再包装即可。
 */

 using namespace mcp::protocol;
 
class PromptClient {
    
public:
    /** 直接管理内部 JsonRpcClient（单独一条 TCP） */
    PromptClient(std::string host, uint16_t port)
        : rpc_(std::move(host), port) 
        {
            /* 注册 list_changed 通知 → 本地 lambda 再转发给用户回调 */
        rpc_.onNotification("notifications/prompts/list_changed",
            [this](const nlohmann::json&) {
                std::shared_lock lk(cbMtx_);
                if (promptListChanged_) promptListChanged_();
            });
        }

            /* =========== 业务 API =========== */
    /* -------------------------------------------------- prompts/list */

    /** 同步：可选 cursor，返回整个 ListPromptsResult */
    ListPromptsResult listPrompts(const std::optional<std::string>& cursor = {})
    {
        ListPromptsRequest req;
        if (cursor) req.params.cursor = *cursor;
        return rpc_.call<ListPromptsRequest, ListPromptsResult>(req);
    }

    /** 异步：返回 future<ListPromptsResult> */
    std::future<ListPromptsResult> listPromptsAsync(
            const std::optional<std::string>& cursor = {})
    {
        ListPromptsRequest req;
        if (cursor) req.params.cursor = *cursor;
        return rpc_.callAsync<ListPromptsRequest, ListPromptsResult>(req);
    }

    /* -------------------------------------------------- prompts/get */

    /** 同步：直接传已构造好的 GetPromptRequest */
    GetPromptResult getPrompt(const GetPromptRequest& req)
    {
        return rpc_.call<GetPromptRequest, GetPromptResult>(req);
    }

    /** 便捷同步：name + args */
    GetPromptResult getPrompt(const std::string& name,
                              const nlohmann::json& args = {})
    {
        GetPromptRequest req;
        req.params.name       = name;
        req.params.arguments  = args;
        return getPrompt(req);
    }

    /** 异步：future<GetPromptResult> */
    std::future<GetPromptResult> getPromptAsync(const GetPromptRequest& req)
    {
        return rpc_.callAsync<GetPromptRequest, GetPromptResult>(req);
    }

    std::future<GetPromptResult> getPromptAsync(const std::string& name,
                                                const nlohmann::json& args = {})
    {
        GetPromptRequest req;
        req.params.name      = name;
        req.params.arguments = args;
        return getPromptAsync(req);
    }

       /* =========== 通知回调注册 =========== */
    void setPromptListChangedHandler(std::function<void()> cb)
    {
        std::unique_lock lk(cbMtx_);
        promptListChanged_ = std::move(cb);
    }

private:
    JsonRpcClient rpc_;
        /* --- 用户回调 --- */
    std::shared_mutex      cbMtx_;
    std::function<void()>  promptListChanged_;
};

} // namespace mcp::client2