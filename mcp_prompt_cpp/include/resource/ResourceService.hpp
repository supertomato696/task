


#pragma once
#include <functional>
#include <nlohmann/json.hpp>

#include "protocol/McpJsonRpc.hpp"          // Id / makeJsonRpc*
#include "protocol/McpResourceRequests.hpp" // 各种请求 & 结果/通知 结构体
#include "resource/ResourceManager.hpp"     // list/subscribe 等

namespace mcp::resource {

/**
 * 发送 JSON‑RPC 通知用回调
 *   由上层 (Transport / Dispatcher) 注入
 */
using Notify = std::function<void(const nlohmann::json&)>;

/**
 * ------------------------------------------------------------------
 * ResourceService
 *   · 处理 resources/… 方法
 *   · 自动广播 list_changed / updated
 * ------------------------------------------------------------------
 */
class ResourceService {
public:
    explicit ResourceService(ResourceManager& rm, Notify n = nullptr);

    /**
     * handle()
     * ------------------------------------------------------------
     * 传入一条 JSON‑RPC 请求（仅限 resources/*）
     * 返回 JSON‑RPC Result / Error。
     */
    nlohmann::json handle(const nlohmann::json& rpc);

private:
    /* ---- 具体 handler ---- */
    nlohmann::json onList( const protocol::RequestId&,
                           const protocol::ListResourcesParams&);

    nlohmann::json onTemplatesList(
                           const protocol::RequestId&,
                           const protocol::PaginatedParams&);

    nlohmann::json onRead( const protocol::RequestId&,
                           const protocol::ReadResourceParams&);

    nlohmann::json onSubscribe( const protocol::RequestId&,
                                const protocol::SubscribeParams&);

    nlohmann::json onUnsubscribe(const protocol::RequestId&,
                                 const protocol::UnsubscribeParams&);

    /* ---- 通知封装 ---- */
    void fireListChanged();                // notifications/resources/list_changed
    void fireUpdated(const std::string&);  // notifications/resources/updated

    /* ---- 成员 ---- */
    ResourceManager&  rm_;
    Notify            notify_;
};

} // namespace mcp::resource
