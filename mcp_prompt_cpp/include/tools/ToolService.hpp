#pragma once
/*****************************************************************************
 * ToolService  ——  MCP 服务器端 “tools/*” 协议处理
 * --------------------------------------------------------------------------
 *  · tools/list            → 返回注册表中的 protocol::Tool 向量
 *  · tools/call            → 执行 ToolCallback，返回 CallToolResult
 *  · 自动广播 list_changed → Registry 变化时发送
 *
 *  发送/接收 JSON‑RPC 均使用 protocol/McpJsonRpc.hpp 提供的工具函数
 *****************************************************************************/
#include "tools/ToolRegistry.hpp"
#include "protocol/McpToolRequests.hpp"   // CallTool*, ListToolsResult, …
#include "protocol/McpJsonRpc.hpp"        // makeJsonRpcResult/Error, parseRequest

#include <nlohmann/json.hpp>
#include <functional>

namespace mcp::tools {

class ToolService {
public:
    /**
     * @param reg     工具注册表（需生命周期长于 Service）
     * @param notify  用于发送通知 (notifications/tools/list_changed)
     */
    explicit ToolService(ToolRegistry& reg,
                         std::function<void(const nlohmann::json&)> notify);

    /** 处理一条 JSON‑RPC 请求，返回 JSON‑RPC Response/Error */
    nlohmann::json handle(const nlohmann::json& rpcRequest);

private:
    /* 内部分发 —— 对应 MCP 方法 */
    nlohmann::json onList(const mcp::protocol::Id& id,
                          const protocol::PaginatedParams& p);

    nlohmann::json onCall(const protocol::Id& id,
                          const protocol::CallToolParams& p);

    /* 当 Registry 变化时自动触发 */
    void fireListChanged();

    ToolRegistry& reg_;
    std::function<void(const nlohmann::json&)> notify_;
};

} // namespace mcp::tools
