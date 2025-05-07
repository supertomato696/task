#include "tools/ToolService.hpp"

using namespace mcp;
using namespace mcp::tools;
using json = nlohmann::json;

/* ---------------------------------------------------------------------- */
ToolService::ToolService(ToolRegistry& reg,
                         std::function<void(const json&)> notify)
    : reg_(reg), notify_(std::move(notify))
{
    /* 注册回调：Registry 增删工具时广播 list_changed */
    reg_.setChangedCb([this]{ fireListChanged(); });
}

/* ---------------------------------------------------------------------- */
json ToolService::handle(const json& rpc)
{
    protocol::Id     id;
    std::string      method;
    json             params;

    /* 解析 JSON‑RPC；失败直接回错误 */
    if(!protocol::parseRequest(rpc, id, method, params))
        return protocol::makeJsonRpcError(id, -32600, "invalid request");

    try {
        if(method == "tools/list")
            return onList(id, params.get<protocol::PaginatedParams>());

        if(method == "tools/call")
            return onCall(id, params.get<protocol::CallToolParams>());

        /* 未知方法 */
        return protocol::makeJsonRpcError(id, -32601, "method not found");
    }
    catch(const std::exception& e){
        return protocol::makeJsonRpcError(id, -32000, e.what());
    }
}

/* ====== tools/list ===================================================== */
json ToolService::onList(const protocol::Id& id,
                         const protocol::PaginatedParams& /*unused*/)
{
    protocol::ListToolsResult res;
    res.tools = reg_.listTools();
    /* 如需分页，可在此根据 cursor & page size 切片并返回 nextCursor */
    return protocol::makeJsonRpcResult(id, res);
}

/* ====== tools/call ===================================================== */
json ToolService::onCall(const protocol::Id& id,
                         const protocol::CallToolParams& p)
{
    protocol::CallToolResult out;

    try {
        out.content = reg_.invoke(p.name, p.arguments);
        out.isError = false;
    }
    catch(const std::exception& e){
        out.isError = true;
        protocol::PromptMessage errMsg;
        errMsg.role = protocol::Role::assistant;
        errMsg.content = protocol::TextContent{ .text = "tool error: " + std::string(e.what()) };
        out.content.push_back(std::move(errMsg));
    }
    return protocol::makeJsonRpcResult(id, out);
}

/* ====== notifications/tools/list_changed ============================== */
void ToolService::fireListChanged()
{
    if(!notify_) return;
    protocol::ToolListChangedNotification n;
    notify_(json(n));          // 直接序列化并推送
}


// ToolRegistry registry;
// registerDemoTools(registry);   // 演示工具

// transport::AsioTcpTransport transport(port);

// tools::ToolService toolSvc(registry,
//     [&](const json& note){ transport.send(note); });

// transport.onMessage([&](const json& req){
//     if(req.at("method").get<std::string>().rfind("tools/",0)==0){
//         transport.send( toolSvc.handle(req) );
//     }
//     /* 其余模块 … */
// });
