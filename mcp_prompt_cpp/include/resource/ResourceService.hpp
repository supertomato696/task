#pragma once
/*****************************************************************************
 *  ResourceService
 *  -------------------------------------------------------------------------
 *  JSON‑RPC 2.0 服务端适配 MCP “resources/*” 全部方法
 *
 *  · 依赖 ResourceManager 作为实际数据源
 *  · 通过 protocol::Notify 回调向 Transport 广播通知
 *
 *  Author: ChatGPT implementation – 2025‑05‑07
 *****************************************************************************/
#include "protocol/McpResourceRequests.hpp"   // 所有请求 / 结果 / 通知结构体
#include "protocol/McpJsonRpc.hpp"            // makeJsonRpcResult / Error / parse
#include "resource/ResourceManager.hpp"

#include <functional>

namespace mcp::resource {

/* notify(json) 由 Transport 层注入，用于广播通知给所有连接 */
using Notify = std::function<void(const nlohmann::json&)>;

class ResourceService {
public:
    ResourceService(ResourceManager& rm, Notify notify);

    /* 入口：解析并执行一条 JSON‑RPC Request */
    nlohmann::json handle(const nlohmann::json& rpc);

private:
    /* ---- 单个方法实现 ---- */
    nlohmann::json onList          (const protocol::Id&,
                                    const protocol::ListResourcesParams&);

    nlohmann::json onTemplatesList (const protocol::Id&,
                                    const protocol::PaginatedParams&);

    nlohmann::json onRead          (const protocol::Id&,
                                    const protocol::ReadResourceParams&);

    nlohmann::json onSubscribe     (const protocol::Id&,
                                    const protocol::SubscribeParams&);

    nlohmann::json onUnsubscribe   (const protocol::Id&,
                                    const protocol::UnsubscribeParams&);

    /* ---- 通知封装 ---- */
    void fireListChanged();
    void fireUpdated(const std::string& uri);

    /* ---- state ---- */
    ResourceManager& rm_;
    Notify           notify_;
};

} // namespace mcp::resource
